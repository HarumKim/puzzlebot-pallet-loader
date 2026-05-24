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
from pathlib import Path

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
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Quaternion, TransformStamped, PoseWithCovarianceStamped
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

def get_source_images_path():
    current_file = Path(__file__).resolve()

    for parent in current_file.parents:
        if (parent / "package.xml").exists() and parent.name == "slam":
            return str(parent / "images")

    return str(Path.cwd() / "src" / "slam" / "images")

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

        # ── MAPA DESDE CERO ──
        # Crear un mapa vacío (todo gris = desconocido)
        self.map_img = np.full((300, 300), 128, dtype=np.uint8)
        self.map_h, self.map_w = self.map_img.shape

        self.get_logger().info(
            f'📍 Iniciando SLAM desde cero con mapa vacío de {self.map_w}x{self.map_h} celdas')

        # Log-odds grid para mapeo interno (estilo CoreSLAM)
        # 0 = desconocido
        self.log_odds = np.zeros((self.map_h, self.map_w), dtype=np.int16)

        self.log_occ_hit  = 20    # incremento al marcar ocupada
        self.log_occ_miss = -2    # incremento al marcar libre
        self.log_occ_max  = 100
        self.log_occ_min  = -100

        # Calcular Likelihood Field del mapa base
        self.likelihood_field = self._compute_likelihood_field(self.map_img)
        self.map_ready = True  # Siempre listo, empiece de cero o con base
        self.lf_update_counter = 0
        self.lf_update_interval = 15  # recalcular likelihood field cada N scans

        # =====================================================
        # PARÁMETROS DEL FILTRO DE PARTÍCULAS
        # =====================================================

        self.num_particles = 300
        self.particles = np.zeros((self.num_particles, 4))  # x, y, theta, peso

        self.num_rays_subsample = 60
        self.log_field_floor = -10.0

        # Qué tanto confiar en odometría como prior
        self.odom_sigma_xy = 1.0
        self.odom_sigma_theta = 0.8

        self.lidar_yaw_offset = math.pi  # LiDAR montado al revés (180°)
        self.exploration_ratio = 0.10

        # Suavizado temporal de la estimación
        self.filtered_estimate = None
        self.smooth_alpha = 0.3

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

        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)

        # NOTA: Ya no nos suscribimos a /map porque este nodo hace su propio mapa.

        # =====================================================
        # PUBLICADORES ROS2
        # =====================================================

        self.pose_pub = self.create_publisher(PoseStamped, '/mcl_pose', 10)
        self.particles_pub = self.create_publisher(PoseArray, '/slam/particles', 10)

        # Publicador del mapa interno (SLAM)
        qos_slam_map = QoSProfile(depth=1)
        qos_slam_map.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos_slam_map.reliability = QoSReliabilityPolicy.RELIABLE
        self.slam_map_pub = self.create_publisher(
            OccupancyGrid, '/map', qos_slam_map)

        self.tf_br = TransformBroadcaster(self)

        # Timer para publicar mapa SLAM cada 2 segundos
        self.create_timer(2.0, self.publish_slam_map)

        self.get_logger().info(
            '🎯 MCL+SLAM Node iniciado — esperando /odom y /scan...')

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
    # CONVERSIONES MAPA <-> MUNDO
    # =====================================================

    def world_to_pixel(self, x, y):
        """Convierte coordenadas del mundo a pixeles del mapa."""
        px = int((x - self.map_origin_x) / self.resolution)
        py = int((y - self.map_origin_y) / self.resolution)
        return px, py

    def pixel_to_world(self, px, py):
        """Convierte pixeles del mapa a coordenadas del mundo."""
        x = px * self.resolution + self.map_origin_x
        y = py * self.resolution + self.map_origin_y
        return x, y

    def is_free_world(self, x, y):
        """Verifica si una coordenada del mundo cae en zona libre del mapa."""
        px, py = self.world_to_pixel(x, y)
        if px < 0 or px >= self.map_w or py < 0 or py >= self.map_h:
            return False
        return self.map_img[py, px] > 50  # Libre o desconocido

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
            f'Partículas inicializadas en: '
            f'x={x:.2f}, y={y:.2f}, θ={theta:.2f}')

    def initial_pose_callback(self, msg):
        """Recibe una pose inicial manual desde RViz (2D Pose Estimate)."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        theta = euler_from_quaternion(q.x, q.y, q.z, q.w)

        self.get_logger().info(
            f'📍 Pose inicial manual recibida desde RViz: x={x:.2f}, y={y:.2f}, θ={theta:.2f}')

        # Re-inicializar partículas en esta nueva posición
        self.init_particles_around_pose(x, y, theta)

        # Actualizar la odometría previa para no dar saltos bruscos
        if self.current_robot_pose is not None:
            self.prev_odom = self.current_robot_pose.copy()

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

        # Resampling condicional: solo si las partículas degeneran
        n_eff = 1.0 / (np.sum(scores ** 2) + 1e-10)
        if n_eff < self.num_particles / 2:
            self.resample_particles()

        # Actualizar mapa interno con la mejor partícula
        best_idx = int(np.argmax(self.particles[:, 3]))
        best = self.particles[best_idx]
        self.update_map_with_scan(best[0], best[1], best[2],
                                  ranges, angles, msg)

        self.publish_pose()
        self.publish_particles()
        self.visualize()

    # =====================================================
    # ESTIMACIÓN
    # =====================================================

    def compute_estimate(self):
        """Promedio ponderado de partículas con suavizado temporal."""
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
    # MAPEO INTERNO (SLAM) Y PUBLICADORES EXTRA
    # =====================================================

    def _bresenham(self, x0, y0, x1, y1):
        """Bresenham para trazar rayos en la cuadrícula."""
        cells = []
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return cells

    def update_map_with_scan(self, rx, ry, rtheta, ranges, angles, msg):
        """Actualiza el mapa interno log-odds usando la mejor partícula."""
        crx, cry = self.world_to_pixel(rx, ry)

        for i in range(len(ranges)):
            r = ranges[i]
            if r < msg.range_min or r > msg.range_max - 0.1 or np.isinf(r) or np.isnan(r):
                continue

            angle = rtheta + angles[i] + self.lidar_yaw_offset
            hit_x = rx + r * math.cos(angle)
            hit_y = ry + r * math.sin(angle)
            chx, chy = self.world_to_pixel(hit_x, hit_y)

            cells_on_ray = self._bresenham(crx, cry, chx, chy)

            # Celdas previas al impacto -> libre
            for (cx, cy) in cells_on_ray[:-1]:
                if 0 <= cx < self.map_w and 0 <= cy < self.map_h:
                    self.log_odds[cy, cx] = max(
                        self.log_odds[cy, cx] + self.log_occ_miss, self.log_occ_min)

            # Celda de impacto -> ocupada
            if 0 <= chx < self.map_w and 0 <= chy < self.map_h:
                self.log_odds[chy, chx] = min(
                    self.log_odds[chy, chx] + self.log_occ_hit, self.log_occ_max)

        # Actualizar map_img basado en log_odds
        known = self.log_odds != 0
        prob = 1.0 / (1.0 + np.exp(-self.log_odds[known].astype(np.float32) / 10.0))
        self.map_img[known] = ((1.0 - prob) * 255).astype(np.uint8)

        # ── CERRAR HUECOS ENTRE ESTANTES (Morphological Closing) ──
        # Identificar obstáculos (pixeles negros, valor < 50)
        #obstacle_mask = (self.map_img < 50).astype(np.uint8) * 255
        # Kernel de 5x5 celdas (aprox 25x25 cm)
        #kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))
        #closed_obstacles = cv2.morphologyEx(obstacle_mask, cv2.MORPH_CLOSE, kernel)
        # Pintar los huecos cerrados como obstáculos en la imagen del mapa
        #self.map_img[closed_obstacles == 255] = 0

        # Recalcular Likelihood Field periódicamente
        self.lf_update_counter += 1
        if self.lf_update_counter >= self.lf_update_interval:
            self.likelihood_field = self._compute_likelihood_field(self.map_img)
            self.lf_update_counter = 0

    def publish_particles(self):
        """Publica las partículas como PoseArray para RViz."""
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for p in self.particles:
            pose = Pose()
            pose.position.x = float(p[0])
            pose.position.y = float(p[1])
            pose.orientation = yaw_to_quaternion(float(p[2]))
            msg.poses.append(pose)
        self.particles_pub.publish(msg)

    def publish_slam_map(self):
        """Publica el mapa log-odds como OccupancyGrid."""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = float(self.resolution)
        msg.info.width = int(self.map_w)
        msg.info.height = int(self.map_h)
        msg.info.origin.position.x = float(self.map_origin_x)
        msg.info.origin.position.y = float(self.map_origin_y)
        msg.info.origin.orientation.w = 1.0

        data = np.full((self.map_h, self.map_w), -1, dtype=np.int8)
        known = self.log_odds != 0
        prob = 1.0 / (1.0 + np.exp(-self.log_odds[known].astype(np.float32) / 10.0))
        data[known] = (prob * 100).astype(np.int8)
        
        # ── CERRAR HUECOS ENTRE ESTANTES PARA RVIZ ──
        rviz_obs_mask = (data >= 50).astype(np.uint8) * 255
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        closed_rviz_obs = cv2.morphologyEx(rviz_obs_mask, cv2.MORPH_CLOSE, kernel)
        data[closed_rviz_obs == 255] = 100

        msg.data = data.flatten().tolist()
        self.slam_map_pub.publish(msg)

    def save_map(self):
        """Guarda el mapa interno generado como un archivo PNG."""
        save_dir = get_source_images_path()
        os.makedirs(save_dir, exist_ok=True)

        save_path = os.path.join(save_dir, 'map.png')
        cv2.imwrite(save_path, self.map_img)

        print(f'✅ Mapa guardado exitosamente en: {save_path}')

    # =====================================================
    # VISUALIZACIÓN
    # =====================================================

    def visualize(self):
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
        print('Cerrando MCL node...')
        node.save_map()
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
