"""
MCL Node — Monte Carlo Localization Online

Filtro de partículas que se localiza usando:
  - Un mapa base (mapabase.jpeg) cargado al inicio
  - El mapa en construcción (/map OccupancyGrid) que actualiza el likelihood field
  - Odometría (/odom) como modelo de movimiento
  - LiDAR (/scan) para scoring de partículas

Publica:
  /mcl_pose  (geometry_msgs/PoseStamped) — pose corregida
  TF: map → odom (corrección de localización)

Suscribe:
  /odom  (nav_msgs/Odometry)
  /scan  (sensor_msgs/LaserScan)
  /map   (nav_msgs/OccupancyGrid)
"""

import os
import math
from collections import deque

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    qos_profile_sensor_data,
)

from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


# ─────────────────────────────────────────────────────────────────────
#  Utilidades
# ─────────────────────────────────────────────────────────────────────
def euler_from_quaternion(x, y, z, w):
    """Convierte un cuaternión a yaw (navegación 2D)."""
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


def yaw_to_quaternion(yaw):
    return Quaternion(
        x=0.0, y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0)
    )


# ─────────────────────────────────────────────────────────────────────
#  Nodo MCL
# ─────────────────────────────────────────────────────────────────────
class MCLNode(Node):
    def __init__(self):
        super().__init__('mcl_node')

        # =====================================================
        # PARÁMETROS DEL MAPA
        # =====================================================

        self.resolution = 0.05  # m/pixel (debe coincidir con el mapper)
        self.map_origin_x = -7.5  # origen del mapa en mundo (m)
        self.map_origin_y = -7.5

        # Cargar mapa base
        base_map_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            'mapabase.png'
        )

        base_img = cv2.imread(base_map_path, cv2.IMREAD_GRAYSCALE)

        if base_img is not None:
            self.map_img = base_img
            self.get_logger().info(
                f'📍 Mapa base cargado: {base_img.shape} desde {base_map_path}')
        else:
            # Sin mapa base: crear un mapa vacío (todo gris = desconocido)
            self.get_logger().warn(
                f'⚠️  No se encontró mapa base en {base_map_path}. '
                f'Iniciando con mapa vacío — esperando /map...')
            self.map_img = np.full((300, 300), 128, dtype=np.uint8)

        self.map_h, self.map_w = self.map_img.shape

        # Calcular Likelihood Field del mapa base
        self.likelihood_field = self._compute_likelihood_field(self.map_img)
        self.map_ready = base_img is not None

        # =====================================================
        # PARÁMETROS DEL FILTRO DE PARTÍCULAS
        # =====================================================

        self.num_particles = 300
        self.particles = np.zeros((self.num_particles, 4))  # x, y, theta, peso

        self.num_rays_subsample = 60
        self.log_field_floor = -10.0

        # Qué tanto confiar en odometría como prior
        self.odom_sigma_xy = 0.45
        self.odom_sigma_theta = 0.6

        self.lidar_yaw_offset = 0.0  
        self.exploration_ratio = 0.10

        # Suavizado temporal de la estimación
        self.filtered_estimate = None
        self.smooth_alpha = 0.8

        self.estimate_trail = deque(maxlen=80)
        self.mcl_estimate = None

        self.prev_odom = None
        self.current_robot_pose = None
        self.particles_initialized = False

        self.display_available = True
        self.scan_count = 0

        # =====================================================
        # SUSCRIPTORES ROS2
        # =====================================================

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        # /map con QoS TRANSIENT_LOCAL (para recibir el último mapa publicado)
        qos_map = QoSProfile(depth=1)
        qos_map.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos_map.reliability = QoSReliabilityPolicy.RELIABLE

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, qos_map)

        # =====================================================
        # PUBLICADORES ROS2
        # =====================================================

        self.pose_pub = self.create_publisher(PoseStamped, '/mcl_pose', 10)
        self.tf_br = TransformBroadcaster(self)

        self.get_logger().info(
            '🎯 MCL Node iniciado — esperando /odom, /scan y /map...')

    # =====================================================
    # LIKELIHOOD FIELD
    # =====================================================

    def _compute_likelihood_field(self, gray_img):
        """Crea un likelihood field a partir de una imagen grayscale del mapa.
        Pixels oscuros (<128) = obstáculo, aplicamos Gaussian blur."""
        obstacle_binary = (gray_img < 128).astype(np.float64)

        sigma_pixels = 5.0
        likelihood = cv2.GaussianBlur(
            obstacle_binary, (0, 0),
            sigmaX=sigma_pixels, sigmaY=sigma_pixels)

        lf_max = likelihood.max()
        if lf_max > 0:
            likelihood /= lf_max

        self.get_logger().info(
            f'Likelihood field calculado — '
            f'rango: {likelihood.min():.4f} - {likelihood.max():.4f}')

        return likelihood

    # =====================================================
    # CALLBACK DE /map (actualización dinámica)
    # =====================================================

    def map_callback(self, msg):
        """Recibe el OccupancyGrid en vivo y actualiza el likelihood field."""
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution

        # Convertir OccupancyGrid → imagen grayscale
        #   -1 (desconocido) → 128 (gris)
        #    0 (libre)        → 255 (blanco)
        #  100 (ocupado)      → 0   (negro)
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        gray = np.full((h, w), 128, dtype=np.uint8)
        gray[data == 0] = 255    # libre
        gray[data == 100] = 0    # ocupado

        # Mezclar con mapa base si tienen las mismas dimensiones
        if self.map_img.shape == gray.shape:
            # Donde el mapa en vivo tiene información (no desconocido),
            # usar esa información. Donde es desconocido, mantener el mapa base.
            known_mask = (data != -1)
            merged = self.map_img.copy()
            merged[known_mask] = gray[known_mask]
            self.map_img = merged
        else:
            # Dimensiones diferentes: usar el mapa en vivo directamente
            self.map_img = gray
            self.map_h, self.map_w = h, w

        # Actualizar metadatos
        self.resolution = res
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_h, self.map_w = self.map_img.shape

        # Recalcular likelihood field
        self.likelihood_field = self._compute_likelihood_field(self.map_img)
        self.map_ready = True

    # =====================================================
    # CONVERSIONES MAPA <-> MUNDO
    # =====================================================

    def world_to_pixel(self, x, y):
        """Convierte coordenadas del mundo a pixeles del mapa."""
        px = int((x - self.map_origin_x) / self.resolution)
        py = int((y - self.map_origin_y) / self.resolution)
        return px, py

    def pixel_to_world(self, px, py):
        """Convierte pixeles del mapa a coordenadas del mundo (flip Y inverso)."""
        x = px * self.resolution + self.map_origin_x
        y = py * self.resolution + self.map_origin_y
        return x, y

    def is_free_world(self, x, y):
        """Verifica si una coordenada del mundo cae en zona libre del mapa."""
        px, py = self.world_to_pixel(x, y)
        if px < 0 or px >= self.map_w or py < 0 or py >= self.map_h:
            return False
        return self.map_img[py, px] > 200

    # =====================================================
    # INICIALIZACIÓN DE PARTÍCULAS
    # =====================================================

    def init_particles_around_pose(self, x, y, theta):
        """Inicializa partículas alrededor de la pose inicial de odometría."""
        self.particles = np.zeros((self.num_particles, 4))

        for i in range(self.num_particles):
            px = x + np.random.normal(0, 0.35)
            py = y + np.random.normal(0, 0.35)
            ptheta = theta + np.random.normal(0, 0.25)

            attempts = 0
            while not self.is_free_world(px, py) and attempts < 30:
                px = x + np.random.normal(0, 0.35)
                py = y + np.random.normal(0, 0.35)
                ptheta = theta + np.random.normal(0, 0.25)
                attempts += 1

            self.particles[i] = [px, py, ptheta, 1.0 / self.num_particles]

        self.filtered_estimate = (x, y, theta)
        self.mcl_estimate = (x, y, theta)

        self.get_logger().info(
            f'Partículas inicializadas alrededor de odom: '
            f'x={x:.2f}, y={y:.2f}, θ={theta:.2f}')

    # =====================================================
    # ODOMETRÍA — MODELO DE MOVIMIENTO
    # =====================================================

    def odom_callback(self, msg):
        """Recibe odometría y mueve las partículas con dx, dy, dtheta."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        theta = euler_from_quaternion(q.x, q.y, q.z, q.w)

        current_odom = np.array([x, y, theta])
        self.current_robot_pose = current_odom

        if self.prev_odom is None:
            self.prev_odom = current_odom

            if not self.particles_initialized:
                self.init_particles_around_pose(x, y, theta)
                self.particles_initialized = True
            return

        dx = current_odom[0] - self.prev_odom[0]
        dy = current_odom[1] - self.prev_odom[1]
        dtheta = current_odom[2] - self.prev_odom[2]
        dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))

        if abs(dx) > 0.002 or abs(dy) > 0.002 or abs(dtheta) > 0.005:
            self.move_particles(dx, dy, dtheta)
            self.prev_odom = current_odom

    def move_particles(self, dx, dy, dtheta):
        """Mueve partículas con ruido gaussiano + detección de colisión."""
        dist = math.hypot(dx, dy)
        noise_xy = 0.005 + 0.04 * dist
        noise_theta = 0.005 + 0.04 * abs(dtheta)

        for i in range(self.num_particles):
            old_x, old_y, old_theta = self.particles[i, :3]
            old_is_free = self.is_free_world(old_x, old_y)

            new_x = old_x + dx + np.random.normal(0, noise_xy)
            new_y = old_y + dy + np.random.normal(0, noise_xy)
            new_theta = old_theta + dtheta + np.random.normal(0, noise_theta)
            new_theta = math.atan2(math.sin(new_theta), math.cos(new_theta))

            # Si estaba libre y el movimiento la mete en pared, revertir
            if old_is_free and not self.is_free_world(new_x, new_y):
                continue  # mantener pose anterior
            else:
                self.particles[i, 0] = new_x
                self.particles[i, 1] = new_y
                self.particles[i, 2] = new_theta

    # =====================================================
    # LIDAR — SCORING DE PARTÍCULAS
    # =====================================================

    def scan_callback(self, msg):
        """Procesa LiDAR y asigna puntajes con Likelihood Field."""
        if self.prev_odom is None or not self.map_ready:
            return

        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)

        n = min(len(angles), len(ranges))
        angles = angles[:n]
        ranges = ranges[:n]

        # Filtrar lecturas válidas
        valid_idx = (
            (ranges > msg.range_min) &
            (ranges < msg.range_max - 0.1) &
            ~np.isinf(ranges) &
            ~np.isnan(ranges)
        )
        angles = angles[valid_idx]
        ranges = ranges[valid_idx]

        if len(ranges) == 0:
            self.compute_estimate()
            self.publish_pose()
            self.visualize()
            return

        # Submuestrear rayos para rendimiento
        step = max(1, len(ranges) // self.num_rays_subsample)
        angles = angles[::step]
        ranges = ranges[::step]

        map_w = self.map_w
        map_h = self.map_h
        resolution = self.resolution
        origin_x = self.map_origin_x
        origin_y = self.map_origin_y
        likelihood_field = self.likelihood_field
        log_floor = self.log_field_floor

        log_scores = np.zeros(self.num_particles)

        for i in range(self.num_particles):
            x, y, theta, _ = self.particles[i]

            if not self.is_free_world(x, y):
                log_scores[i] = -1e6
                continue

            global_angles = theta + angles + self.lidar_yaw_offset

            # Proyectar puntos de impacto del LiDAR
            hit_x = x + ranges * np.cos(global_angles)
            hit_y = y + ranges * np.sin(global_angles)

            hit_px = ((hit_x - origin_x) / resolution).astype(int)
            hit_py = ((hit_y - origin_y) / resolution).astype(int)
            in_bounds = (
                (hit_px >= 0) & (hit_px < map_w) &
                (hit_py >= 0) & (hit_py < map_h)
            )

            log_prob = 0.0
            for j in range(len(ranges)):
                if in_bounds[j]:
                    field_val = likelihood_field[hit_py[j], hit_px[j]]
                else:
                    field_val = 0.0

                if field_val > 1e-6:
                    log_prob += math.log(field_val)
                else:
                    log_prob += log_floor

            # Prior de odometría
            if self.current_robot_pose is not None:
                rx, ry, rt = self.current_robot_pose
                dist_xy = math.hypot(x - rx, y - ry)
                dtheta_odom = math.atan2(
                    math.sin(theta - rt), math.cos(theta - rt))

                odom_prior = (
                    -0.5 * (dist_xy / self.odom_sigma_xy) ** 2
                    - 0.5 * (dtheta_odom / self.odom_sigma_theta) ** 2
                )
                log_scores[i] = log_prob + odom_prior
            else:
                log_scores[i] = log_prob

        # Convertir log-scores a pesos normalizados
        max_log = np.max(log_scores)
        scores = np.exp(log_scores - max_log)
        sum_scores = np.sum(scores)

        if sum_scores > 0:
            scores /= sum_scores
        else:
            scores = np.ones(self.num_particles) / self.num_particles

        self.particles[:, 3] = scores

        self.compute_estimate()
        self.resample_particles()
        self.publish_pose()
        self.visualize()

    # =====================================================
    # ESTIMACIÓN
    # =====================================================

    def compute_estimate(self):
        """Promedio ponderado de partículas con suavizado temporal."""
        if self.current_robot_pose is not None:
            rx, ry, _ = self.current_robot_pose
            distances = np.sqrt(
                (self.particles[:, 0] - rx) ** 2 +
                (self.particles[:, 1] - ry) ** 2
            )
            nearby = np.where(distances < 1.5)[0]
            selected = self.particles[nearby] if len(nearby) > 0 else self.particles
        else:
            selected = self.particles

        weights = selected[:, 3].copy()
        w_sum = np.sum(weights)

        if w_sum <= 0:
            weights = np.ones(len(selected)) / len(selected)
        else:
            weights /= w_sum

        est_x = np.sum(selected[:, 0] * weights)
        est_y = np.sum(selected[:, 1] * weights)

        sin_sum = np.sum(np.sin(selected[:, 2]) * weights)
        cos_sum = np.sum(np.cos(selected[:, 2]) * weights)
        est_theta = math.atan2(sin_sum, cos_sum)

        # Suavizado temporal
        if self.filtered_estimate is None:
            self.filtered_estimate = (est_x, est_y, est_theta)
        else:
            prev_x, prev_y, prev_theta = self.filtered_estimate
            filt_x = self.smooth_alpha * prev_x + (1.0 - self.smooth_alpha) * est_x
            filt_y = self.smooth_alpha * prev_y + (1.0 - self.smooth_alpha) * est_y

            dtheta = math.atan2(
                math.sin(est_theta - prev_theta),
                math.cos(est_theta - prev_theta))
            filt_theta = prev_theta + (1.0 - self.smooth_alpha) * dtheta
            filt_theta = math.atan2(math.sin(filt_theta), math.cos(filt_theta))

            self.filtered_estimate = (filt_x, filt_y, filt_theta)

        self.mcl_estimate = self.filtered_estimate
        self.estimate_trail.append((self.mcl_estimate[0], self.mcl_estimate[1]))

    # =====================================================
    # PUBLICACIÓN DE POSE Y TF
    # =====================================================

    def publish_pose(self):
        """Publica /mcl_pose y TF map → odom."""
        if self.mcl_estimate is None:
            return

        est_x, est_y, est_theta = self.mcl_estimate
        now = self.get_clock().now().to_msg()

        # Publicar PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = est_x
        pose_msg.pose.position.y = est_y
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation = yaw_to_quaternion(est_theta)

        self.pose_pub.publish(pose_msg)

        # Publicar TF: map → odom (corrección MCL)
        # corrección = pose_mcl - pose_odom
        if self.current_robot_pose is not None:
            ox, oy, ot = self.current_robot_pose

            # Calcular la transformación map→odom que hace que
            # odom→base_link compuesto con map→odom dé la pose MCL
            dyaw = est_theta - ot
            cos_d = math.cos(dyaw)
            sin_d = math.sin(dyaw)

            # La corrección en posición debe tener en cuenta la rotación
            corr_x = est_x - (cos_d * ox - sin_d * oy)
            corr_y = est_y - (sin_d * ox + cos_d * oy)

            tf_map = TransformStamped()
            tf_map.header.stamp = now
            tf_map.header.frame_id = 'map'
            tf_map.child_frame_id = 'odom'

            tf_map.transform.translation.x = corr_x
            tf_map.transform.translation.y = corr_y
            tf_map.transform.translation.z = 0.0
            tf_map.transform.rotation = yaw_to_quaternion(dyaw)

            self.tf_br.sendTransform(tf_map)

    # =====================================================
    # RESAMPLING
    # =====================================================

    def resample_particles(self):
        """Low-Variance Systematic Resampling con exploración."""
        weights = self.particles[:, 3].copy()
        w_sum = weights.sum()

        if w_sum > 0:
            weights /= w_sum
        else:
            weights = np.ones(self.num_particles) / self.num_particles

        N = self.num_particles
        cumsum = np.cumsum(weights)
        cumsum[-1] = 1.0

        r = np.random.uniform(0, 1.0 / N)
        positions = r + np.arange(N) / N
        indices = np.searchsorted(cumsum, positions)
        indices = np.clip(indices, 0, N - 1)

        new_particles = self.particles[indices].copy()

        # Ruido post-resampling
        new_particles[:, 0] += np.random.normal(0, 0.02, N)
        new_particles[:, 1] += np.random.normal(0, 0.02, N)
        new_particles[:, 2] += np.random.normal(0, 0.03, N)
        new_particles[:, 2] = np.arctan2(
            np.sin(new_particles[:, 2]),
            np.cos(new_particles[:, 2]))

        # Partículas de exploración alrededor de la odometría
        num_explore = max(1, int(N * self.exploration_ratio))

        if self.current_robot_pose is not None:
            ox, oy, otheta = self.current_robot_pose

            for i in range(num_explore):
                idx = N - 1 - i
                nx = ox + np.random.normal(0, 0.35)
                ny = oy + np.random.normal(0, 0.35)
                ntheta = otheta + np.random.normal(0, 0.25)

                attempts = 0
                while not self.is_free_world(nx, ny) and attempts < 15:
                    nx = ox + np.random.normal(0, 0.35)
                    ny = oy + np.random.normal(0, 0.35)
                    attempts += 1

                new_particles[idx] = [
                    nx, ny,
                    math.atan2(math.sin(ntheta), math.cos(ntheta)),
                    1.0 / N
                ]

        # Revertir partículas en pared
        for i in range(N):
            if not self.is_free_world(new_particles[i, 0], new_particles[i, 1]):
                parent_idx = indices[i] if i < len(indices) else 0
                new_particles[i, :3] = self.particles[parent_idx, :3]

        new_particles[:, 3] = 1.0 / N
        self.particles = new_particles

    # =====================================================
    # VISUALIZACIÓN
    # =====================================================

    def visualize(self):
        return
        """Dibuja mapa, partículas, estimación MCL y odometría."""
        if not self.display_available:
            return

        try:
            vis_map = cv2.cvtColor(self.map_img, cv2.COLOR_GRAY2BGR)

            # Trail de estimaciones
            trail_list = list(self.estimate_trail)
            for j in range(len(trail_list)):
                tx, ty = trail_list[j]
                tpx, tpy = self.world_to_pixel(tx, ty)
                if 0 <= tpx < self.map_w and 0 <= tpy < self.map_h:
                    alpha = int(80 + 175 * (j / max(len(trail_list), 1)))
                    cv2.circle(vis_map, (tpx, tpy), 2, (0, alpha, alpha), -1)

            # Partículas (verde)
            for i in range(self.num_particles):
                px, py = self.world_to_pixel(
                    self.particles[i, 0], self.particles[i, 1])
                if 0 <= px < self.map_w and 0 <= py < self.map_h:
                    cv2.circle(vis_map, (px, py), 3, (0, 255, 0), -1)

            # Estimación MCL (rojo)
            if self.mcl_estimate is not None:
                est_x, est_y, est_theta = self.mcl_estimate
                bx, by = self.world_to_pixel(est_x, est_y)
                if 0 <= bx < self.map_w and 0 <= by < self.map_h:
                    cv2.circle(vis_map, (bx, by), 8, (0, 0, 255), 2)
                    end_x = int(bx + 20 * math.cos(est_theta))
                    end_y = int(by - 20 * math.sin(est_theta))
                    cv2.line(vis_map, (bx, by), (end_x, end_y), (0, 0, 255), 2)

            # Pose real por odometría (azul)
            if self.current_robot_pose is not None:
                rx, ry, rt = self.current_robot_pose
                rpx, rpy = self.world_to_pixel(rx, ry)
                if 0 <= rpx < self.map_w and 0 <= rpy < self.map_h:
                    cv2.circle(vis_map, (rpx, rpy), 7, (255, 0, 0), -1)
                    rend_x = int(rpx + 22 * math.cos(rt))
                    rend_y = int(rpy - 22 * math.sin(rt))
                    cv2.line(vis_map, (rpx, rpy), (rend_x, rend_y), (255, 0, 0), 2)

            display = cv2.resize(vis_map, (700, 700))
            cv2.imshow("MCL — Monte Carlo Localization", display)
            cv2.waitKey(1)

        except cv2.error as e:
            self.get_logger().warn(
                f'No se puede mostrar ventana OpenCV: {e}. '
                f'Desactivando visualización.', once=True)
            self.display_available = False


# ─────────────────────────────────────────────────────────────────────
#  main
# ─────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = MCLNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Cerrando MCL node...')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == '__main__':
    main()
