"""
MCL Localization Node — Solo Localización (sin mapeo)

Localiza el robot en un mapa pre-construido usando filtro de partículas:
  - Carga map.png generado por el nodo SLAM al inicio (el mapa no se modifica)
  - Odometría (/odom) como modelo de movimiento
  - LiDAR (/scan) para scoring contra un likelihood field FIJO

Publica:
  /mcl_pose        (geometry_msgs/PoseStamped)       — pose estimada MCL
  /mcl_odom        (nav_msgs/Odometry)               — misma pose como odometría
  /mcl/particles   (geometry_msgs/PoseArray)         — nube de partículas
  /map             (nav_msgs/OccupancyGrid, latched)  — mapa estático para RViz
  TF: map → odom

Suscribe:
  /odom        (nav_msgs/Odometry)
  /scan        (sensor_msgs/LaserScan)
  /initialpose (geometry_msgs/PoseWithCovarianceStamped)  — desde RViz
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
from geometry_msgs.msg import (
    Point, PoseStamped, PoseArray, Pose, Quaternion,
    TransformStamped, PoseWithCovarianceStamped,
)
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster


# ─────────────────────────────────────────────────────────────────────
#  Utilidades
# ─────────────────────────────────────────────────────────────────────

def euler_from_quaternion(x, y, z, w):
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


def yaw_to_quaternion(yaw):
    return Quaternion(
        x=0.0, y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )


def find_map_image():
    """Busca el mapa orientado del paquete slam — primero en install, luego en src."""
    preferred_names = ('map_oriented.png', 'map.png', 'map.pgm')
    try:
        from ament_index_python.packages import get_package_share_directory
        images_dir = os.path.join(get_package_share_directory('slam'), 'images')
        for name in preferred_names:
            candidate = os.path.join(images_dir, name)
            if os.path.exists(candidate):
                return candidate
    except Exception:
        pass

    current_file = Path(__file__).resolve()
    for parent in current_file.parents:
        images_dir = parent / 'slam' / 'images'
        for name in preferred_names:
            candidate = images_dir / name
            if candidate.exists():
                return str(candidate)

    return str(Path.cwd() / 'src' / 'slam' / 'images' / 'map_oriented.png')


# ─────────────────────────────────────────────────────────────────────
#  Nodo de localización MCL
# ─────────────────────────────────────────────────────────────────────

class MCLLocalizationNode(Node):
    def __init__(self):
        super().__init__('mcl_localization_node')

        # ── Parámetros configurables ───────────────────────────────────
        self.declare_parameter('map_image_path', '')
        self.declare_parameter('odom_topic', '/wheel_odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('initial_pose_topic', '/initialpose')
        self.declare_parameter('mcl_pose_topic', '/mcl_pose')
        self.declare_parameter('mcl_odom_topic', '/mcl_odom')
        self.declare_parameter('particles_topic', '/mcl/particles')
        self.declare_parameter('particles_points_topic', '/mcl/particles_points')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('show_debug_window', False)
        self.declare_parameter('global_localization_on_start', True)

        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_origin_x', -7.5)
        self.declare_parameter('map_origin_y', -7.5)

        self.declare_parameter('num_particles', 1200)
        self.declare_parameter('num_rays_subsample', 60)
        self.declare_parameter('lidar_yaw_offset_deg', 180.0)
        self.declare_parameter('exploration_ratio', 0.10)
        self.declare_parameter('smooth_alpha', 0.3)
        self.declare_parameter('likelihood_sigma_px', 5.0)
        self.declare_parameter('free_threshold', 240)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.initial_pose_topic = self.get_parameter('initial_pose_topic').value
        self.mcl_pose_topic = self.get_parameter('mcl_pose_topic').value
        self.mcl_odom_topic = self.get_parameter('mcl_odom_topic').value
        self.particles_topic = self.get_parameter('particles_topic').value
        self.particles_points_topic = self.get_parameter('particles_points_topic').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.show_debug_window = bool(self.get_parameter('show_debug_window').value)
        self.global_localization_on_start = bool(
            self.get_parameter('global_localization_on_start').value)
        self.display_available = self.show_debug_window

        # ── Parámetros del mapa ────────────────────────────────────────
        self.resolution = float(self.get_parameter('map_resolution').value)
        self.map_origin_x = float(self.get_parameter('map_origin_x').value)
        self.map_origin_y = float(self.get_parameter('map_origin_y').value)
        self.free_threshold = int(self.get_parameter('free_threshold').value)

        # ── Cargar mapa pre-construido ─────────────────────────────────
        map_path = self.get_parameter('map_image_path').value or find_map_image()

        if not os.path.exists(map_path):
            self.get_logger().error(f'Mapa no encontrado: {map_path}')
            raise FileNotFoundError(f'Mapa no encontrado: {map_path}')

        self.map_img = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
        if self.map_img is None:
            raise RuntimeError(f'No se pudo leer la imagen: {map_path}')

        self.map_h, self.map_w = self.map_img.shape
        self.free_pixels = self._find_free_pixels(self.map_img)
        self.get_logger().info(
            f'Mapa cargado: {map_path} '
            f'({self.map_w}x{self.map_h} px, {self.resolution} m/px, '
            f'{len(self.free_pixels)} celdas libres)')

        # Likelihood field — calculado UNA SOLA VEZ, nunca se modifica
        sigma_px = float(self.get_parameter('likelihood_sigma_px').value)
        self.likelihood_field = self._compute_likelihood_field(self.map_img, sigma_px)
        self.map_ready = True

        # ── Parámetros del filtro de partículas ───────────────────────
        self.num_particles = int(self.get_parameter('num_particles').value)
        self.particles = np.zeros((self.num_particles, 4))  # x, y, theta, peso

        self.num_rays_subsample = int(self.get_parameter('num_rays_subsample').value)
        self.log_field_floor = -10.0

        self.lidar_yaw_offset = math.radians(
            float(self.get_parameter('lidar_yaw_offset_deg').value))
        self.exploration_ratio = float(self.get_parameter('exploration_ratio').value)

        self.filtered_estimate = None
        self.smooth_alpha = float(self.get_parameter('smooth_alpha').value)
        self.estimate_trail = deque(maxlen=80)
        self.mcl_estimate = None

        self.prev_odom = None
        self.current_robot_pose = None
        self.particles_initialized = False

        # ── Suscriptores ──────────────────────────────────────────────
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.initial_pose_topic,
            self.initial_pose_callback, 10)

        # ── Publicadores ──────────────────────────────────────────────
        self.pose_pub = self.create_publisher(PoseStamped, self.mcl_pose_topic, 10)
        self.mcl_odom_pub = self.create_publisher(Odometry, self.mcl_odom_topic, 10)
        self.particles_pub = self.create_publisher(PoseArray, self.particles_topic, 10)
        self.particles_points_pub = self.create_publisher(
            Marker, self.particles_points_topic, 10)

        qos_map = QoSProfile(depth=1)
        qos_map.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos_map.reliability = QoSReliabilityPolicy.RELIABLE
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos_map)

        self.tf_br = TransformBroadcaster(self)

        # Publicar mapa estático una vez al arranque (delay para que RViz conecte)
        self._map_timer = self.create_timer(1.0, self._publish_static_map_once)

        self.get_logger().info(
            f'MCL Localization Node listo — '
            f'odom={self.odom_topic}, scan={self.scan_topic}, '
            f'pose={self.mcl_pose_topic}')

    # ── Mapa estático ──────────────────────────────────────────────────

    def _publish_static_map_once(self):
        """Publica el mapa cargado como OccupancyGrid (TRANSIENT_LOCAL) y cancela el timer."""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = float(self.resolution)
        msg.info.width = int(self.map_w)
        msg.info.height = int(self.map_h)
        msg.info.origin.position.x = float(self.map_origin_x)
        msg.info.origin.position.y = float(self.map_origin_y)
        msg.info.origin.orientation.w = 1.0

        # Convertir escala de grises a occupancy conservando bordes suavizados:
        # blanco=libre(0), negro=ocupado(100), grises=probabilidad intermedia.
        data = np.rint((255.0 - self.map_img.astype(np.float32)) * 100.0 / 255.0)
        data = np.clip(data, 0, 100).astype(np.int8)

        msg.data = np.flipud(data).flatten().tolist()
        self.map_pub.publish(msg)
        self.get_logger().info('Mapa estático publicado en /map')

        self._map_timer.cancel()

    # ── Likelihood Field ───────────────────────────────────────────────

    def _compute_likelihood_field(self, gray_img, sigma_px=5.0):
        """Likelihood field gaussiano alrededor de obstáculos (calculado una sola vez)."""
        obstacle_binary = (gray_img < 128).astype(np.float64)
        likelihood = cv2.GaussianBlur(
            obstacle_binary, (0, 0), sigmaX=sigma_px, sigmaY=sigma_px)
        lf_max = likelihood.max()
        if lf_max > 0:
            likelihood /= lf_max
        self.get_logger().info(
            f'Likelihood field listo — '
            f'rango [{likelihood.min():.4f}, {likelihood.max():.4f}]')
        return likelihood

    def _find_free_pixels(self, gray_img):
        free = np.column_stack(np.where(gray_img >= self.free_threshold))
        if len(free) == 0:
            free = np.column_stack(np.where(gray_img > 200))
        if len(free) == 0:
            raise RuntimeError('El mapa no tiene celdas libres para inicializar MCL')
        return free

    # ── Conversiones mapa ↔ mundo ──────────────────────────────────────

    def world_to_pixel(self, x, y):
        px = int((x - self.map_origin_x) / self.resolution)
        py_ros = int((y - self.map_origin_y) / self.resolution)
        py = self.map_h - 1 - py_ros
        return px, py

    def pixel_to_world(self, px, py):
        x = self.map_origin_x + (float(px) + 0.5) * self.resolution
        y = self.map_origin_y + (self.map_h - 1 - float(py) + 0.5) * self.resolution
        return x, y

    def is_free_world(self, x, y):
        px, py = self.world_to_pixel(x, y)
        if px < 0 or px >= self.map_w or py < 0 or py >= self.map_h:
            return False
        return int(self.map_img[py, px]) >= self.free_threshold

    # ── Inicialización de partículas ───────────────────────────────────

    def _random_free_particles(self, count):
        indices = np.random.randint(0, len(self.free_pixels), size=count)
        sampled = self.free_pixels[indices]
        particles = np.zeros((count, 4), dtype=float)
        for i, (py, px) in enumerate(sampled):
            x, y = self.pixel_to_world(px, py)
            particles[i] = [
                x + np.random.uniform(-0.5, 0.5) * self.resolution,
                y + np.random.uniform(-0.5, 0.5) * self.resolution,
                np.random.uniform(-math.pi, math.pi),
                1.0 / count,
            ]
        return particles

    def init_particles_global(self):
        self.particles = self._random_free_particles(self.num_particles)
        self.filtered_estimate = None
        self.mcl_estimate = None
        self.get_logger().info(
            f'Localización global iniciada — {self.num_particles} partículas '
            f'distribuidas en todo el mapa libre')

    def init_particles_around_pose(self, x, y, theta):
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
            f'Partículas inicializadas — '
            f'x={x:.2f}, y={y:.2f}, θ={math.degrees(theta):.1f}°')

    def initial_pose_callback(self, msg):
        """Recibe pose inicial desde RViz (botón 2D Pose Estimate)."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        theta = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.get_logger().info(
            f'Pose inicial desde RViz: x={x:.2f}, y={y:.2f}, θ={math.degrees(theta):.1f}°')
        self.init_particles_around_pose(x, y, theta)
        if self.current_robot_pose is not None:
            self.prev_odom = self.current_robot_pose.copy()

    # ── Modelo de movimiento ───────────────────────────────────────────

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)

        current_odom = np.array([x, y, theta])
        self.current_robot_pose = current_odom

        if self.prev_odom is None:
            self.prev_odom = current_odom
            if not self.particles_initialized:
                if self.global_localization_on_start:
                    self.init_particles_global()
                    self.publish_particles()
                else:
                    self.init_particles_around_pose(x, y, theta)
                self.particles_initialized = True
            return

        dx = current_odom[0] - self.prev_odom[0]
        dy = current_odom[1] - self.prev_odom[1]
        dtheta = math.atan2(
            math.sin(current_odom[2] - self.prev_odom[2]),
            math.cos(current_odom[2] - self.prev_odom[2]))

        if abs(dx) > 0.002 or abs(dy) > 0.002 or abs(dtheta) > 0.005:
            self.move_particles(dx, dy, dtheta)
            self.prev_odom = current_odom

    def move_particles(self, dx, dy, dtheta):
        dist = math.hypot(dx, dy)
        noise_xy = 0.005 + 0.04 * dist
        noise_theta = 0.005 + 0.04 * abs(dtheta)

        for i in range(self.num_particles):
            old_x, old_y, old_theta = self.particles[i, :3]
            old_is_free = self.is_free_world(old_x, old_y)

            new_x = old_x + dx + np.random.normal(0, noise_xy)
            new_y = old_y + dy + np.random.normal(0, noise_xy)
            new_theta = math.atan2(
                math.sin(old_theta + dtheta + np.random.normal(0, noise_theta)),
                math.cos(old_theta + dtheta + np.random.normal(0, noise_theta)))

            if old_is_free and not self.is_free_world(new_x, new_y):
                continue  # no mover a celda ocupada
            self.particles[i, 0] = new_x
            self.particles[i, 1] = new_y
            self.particles[i, 2] = new_theta

    # ── Modelo sensor (likelihood field FIJO) ─────────────────────────

    def scan_callback(self, msg):
        if self.prev_odom is None or not self.map_ready:
            return

        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        ranges = np.array(msg.ranges)
        n = min(len(angles), len(ranges))
        angles, ranges = angles[:n], ranges[:n]

        valid = (
            (ranges > msg.range_min) &
            (ranges < msg.range_max - 0.1) &
            ~np.isinf(ranges) & ~np.isnan(ranges))
        angles, ranges = angles[valid], ranges[valid]

        if len(ranges) == 0:
            self.compute_estimate()
            self.publish_pose()
            self.visualize()
            return

        step = max(1, len(ranges) // self.num_rays_subsample)
        angles = angles[::step]
        ranges = ranges[::step]

        lf = self.likelihood_field
        origin_x, origin_y = self.map_origin_x, self.map_origin_y
        res = self.resolution
        map_w, map_h = self.map_w, self.map_h
        log_floor = self.log_field_floor

        log_scores = np.zeros(self.num_particles)

        for i in range(self.num_particles):
            x, y, theta, _ = self.particles[i]

            if not self.is_free_world(x, y):
                log_scores[i] = -1e6
                continue

            global_angles = theta + angles + self.lidar_yaw_offset
            hit_x = x + ranges * np.cos(global_angles)
            hit_y = y + ranges * np.sin(global_angles)
            hit_px = ((hit_x - origin_x) / res).astype(int)
            hit_py = map_h - 1 - ((hit_y - origin_y) / res).astype(int)
            in_bounds = (
                (hit_px >= 0) & (hit_px < map_w) &
                (hit_py >= 0) & (hit_py < map_h))

            log_prob = 0.0
            for j in range(len(ranges)):
                v = lf[hit_py[j], hit_px[j]] if in_bounds[j] else 0.0
                log_prob += math.log(v) if v > 1e-6 else log_floor
            log_scores[i] = log_prob

        max_log = np.max(log_scores)
        scores = np.exp(log_scores - max_log)
        s = np.sum(scores)
        scores = scores / s if s > 0 else np.ones(self.num_particles) / self.num_particles
        self.particles[:, 3] = scores

        self.compute_estimate()

        n_eff = 1.0 / (np.sum(scores ** 2) + 1e-10)
        if n_eff < self.num_particles / 2:
            self.resample_particles()

        self.publish_pose()
        self.publish_particles()
        self.visualize()

    # ── Estimación ─────────────────────────────────────────────────────

    def compute_estimate(self):
        weights = self.particles[:, 3].copy()
        w_sum = np.sum(weights)
        weights = weights / w_sum if w_sum > 0 else np.ones(len(weights)) / len(weights)

        est_x = np.sum(self.particles[:, 0] * weights)
        est_y = np.sum(self.particles[:, 1] * weights)
        sin_sum = np.sum(np.sin(self.particles[:, 2]) * weights)
        cos_sum = np.sum(np.cos(self.particles[:, 2]) * weights)
        est_theta = math.atan2(sin_sum, cos_sum)

        if self.filtered_estimate is None:
            self.filtered_estimate = (est_x, est_y, est_theta)
        else:
            px, py, pt = self.filtered_estimate
            a = self.smooth_alpha
            filt_x = a * px + (1.0 - a) * est_x
            filt_y = a * py + (1.0 - a) * est_y
            dt = math.atan2(math.sin(est_theta - pt), math.cos(est_theta - pt))
            filt_theta = math.atan2(
                math.sin(pt + (1.0 - a) * dt),
                math.cos(pt + (1.0 - a) * dt))
            self.filtered_estimate = (filt_x, filt_y, filt_theta)

        self.mcl_estimate = self.filtered_estimate
        self.estimate_trail.append((self.mcl_estimate[0], self.mcl_estimate[1]))

    # ── Publicación de pose y TF ───────────────────────────────────────

    def publish_pose(self):
        if self.mcl_estimate is None:
            return

        est_x, est_y, est_theta = self.mcl_estimate
        now = self.get_clock().now().to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = est_x
        pose_msg.pose.position.y = est_y
        pose_msg.pose.orientation = yaw_to_quaternion(est_theta)
        self.pose_pub.publish(pose_msg)

        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = est_x
        odom_msg.pose.pose.position.y = est_y
        odom_msg.pose.pose.orientation = yaw_to_quaternion(est_theta)
        odom_msg.pose.covariance[0] = 0.03
        odom_msg.pose.covariance[7] = 0.03
        odom_msg.pose.covariance[35] = 0.02
        self.mcl_odom_pub.publish(odom_msg)

        if self.publish_tf and self.current_robot_pose is not None:
            ox, oy, ot = self.current_robot_pose
            dyaw = est_theta - ot
            cos_d, sin_d = math.cos(dyaw), math.sin(dyaw)
            corr_x = est_x - (cos_d * ox - sin_d * oy)
            corr_y = est_y - (sin_d * ox + cos_d * oy)

            tf_msg = TransformStamped()
            tf_msg.header.stamp = now
            tf_msg.header.frame_id = 'map'
            tf_msg.child_frame_id = 'odom'
            tf_msg.transform.translation.x = corr_x
            tf_msg.transform.translation.y = corr_y
            tf_msg.transform.rotation = yaw_to_quaternion(dyaw)
            self.tf_br.sendTransform(tf_msg)

    def publish_particles(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        points_msg = Marker()
        points_msg.header = msg.header
        points_msg.ns = 'mcl_particles'
        points_msg.id = 0
        points_msg.type = Marker.POINTS
        points_msg.action = Marker.ADD
        points_msg.pose.orientation.w = 1.0
        points_msg.scale.x = 0.035
        points_msg.scale.y = 0.035
        points_msg.color.r = 1.0
        points_msg.color.g = 0.05
        points_msg.color.b = 0.0
        points_msg.color.a = 1.0
        for p in self.particles:
            pose = Pose()
            pose.position.x = float(p[0])
            pose.position.y = float(p[1])
            pose.orientation = yaw_to_quaternion(float(p[2]))
            msg.poses.append(pose)
            points_msg.points.append(Point(x=float(p[0]), y=float(p[1]), z=0.03))
        self.particles_pub.publish(msg)
        self.particles_points_pub.publish(points_msg)

    # ── Resampling ─────────────────────────────────────────────────────

    def resample_particles(self):
        """Low-Variance Systematic Resampling con partículas de exploración."""
        weights = self.particles[:, 3].copy()
        w_sum = weights.sum()
        weights = weights / w_sum if w_sum > 0 else np.ones(self.num_particles) / self.num_particles

        N = self.num_particles
        cumsum = np.cumsum(weights)
        cumsum[-1] = 1.0
        r = np.random.uniform(0, 1.0 / N)
        positions = r + np.arange(N) / N
        indices = np.clip(np.searchsorted(cumsum, positions), 0, N - 1)

        new_particles = self.particles[indices].copy()
        new_particles[:, 0] += np.random.normal(0, 0.02, N)
        new_particles[:, 1] += np.random.normal(0, 0.02, N)
        new_particles[:, 2] += np.random.normal(0, 0.03, N)
        new_particles[:, 2] = np.arctan2(
            np.sin(new_particles[:, 2]), np.cos(new_particles[:, 2]))

        num_explore = max(1, int(N * self.exploration_ratio))
        if self.global_localization_on_start:
            new_particles[-num_explore:] = self._random_free_particles(num_explore)
        elif self.current_robot_pose is not None:
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
                    1.0 / N,
                ]

        for i in range(N):
            if not self.is_free_world(new_particles[i, 0], new_particles[i, 1]):
                new_particles[i, :3] = self.particles[indices[i], :3]

        new_particles[:, 3] = 1.0 / N
        self.particles = new_particles

    # ── Visualización ──────────────────────────────────────────────────

    def visualize(self):
        """Muestra el mapa estático con partículas y estimación MCL superpuestas."""
        if not self.display_available:
            return
        try:
            vis = cv2.cvtColor(self.map_img, cv2.COLOR_GRAY2BGR)

            for j, (tx, ty) in enumerate(self.estimate_trail):
                tpx, tpy = self.world_to_pixel(tx, ty)
                if 0 <= tpx < self.map_w and 0 <= tpy < self.map_h:
                    alpha = int(80 + 175 * j / max(len(self.estimate_trail), 1))
                    cv2.circle(vis, (tpx, tpy), 2, (0, alpha, alpha), -1)

            for p in self.particles:
                px, py = self.world_to_pixel(p[0], p[1])
                if 0 <= px < self.map_w and 0 <= py < self.map_h:
                    cv2.circle(vis, (px, py), 3, (0, 255, 0), -1)

            if self.mcl_estimate is not None:
                ex, ey, et = self.mcl_estimate
                bx, by = self.world_to_pixel(ex, ey)
                if 0 <= bx < self.map_w and 0 <= by < self.map_h:
                    cv2.circle(vis, (bx, by), 8, (0, 0, 255), 2)
                    cv2.line(vis, (bx, by),
                             (int(bx + 20 * math.cos(et)),
                              int(by - 20 * math.sin(et))),
                             (0, 0, 255), 2)

            if self.current_robot_pose is not None:
                rx, ry, rt = self.current_robot_pose
                rpx, rpy = self.world_to_pixel(rx, ry)
                if 0 <= rpx < self.map_w and 0 <= rpy < self.map_h:
                    cv2.circle(vis, (rpx, rpy), 7, (255, 0, 0), -1)
                    cv2.line(vis, (rpx, rpy),
                             (int(rpx + 22 * math.cos(rt)),
                              int(rpy - 22 * math.sin(rt))),
                             (255, 0, 0), 2)

            cv2.imshow('MCL — Localización', cv2.resize(vis, (700, 700)))
            cv2.waitKey(1)

        except cv2.error as e:
            self.get_logger().warn(f'cv2 display error: {e}', once=True)
            self.display_available = False


# ─────────────────────────────────────────────────────────────────────
#  main
# ─────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = MCLLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
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
