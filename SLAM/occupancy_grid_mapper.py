"""
Occupancy Grid Mapper — Mapping con poses conocidas (odometría)

Construye un mapa de ocupación 2D usando:
  - LiDAR 2D (/scan)
  - Odometría (/odom) como pose conocida
  - Bresenham ray tracing
  - Representación log-odds

Publica:
  - /map (nav_msgs/OccupancyGrid)
  - TF estática map → odom (identidad)

Guarda el mapa como .pgm + .yaml + .png al hacer Ctrl+C
"""

import math
import os

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class OccupancyGridMapper(Node):
    def __init__(self):
        super().__init__('occupancy_grid_mapper')

        # ── Parámetros ──────────────────────────────────────────────
        self.declare_parameter('resolution', 0.05)        # m/celda
        self.declare_parameter('map_width', 15.0)          # metros
        self.declare_parameter('map_height', 15.0)         # metros
        self.declare_parameter('log_odds_occ', 1.2)        # incremento al marcar ocupada
        self.declare_parameter('log_odds_free', -0.4)      # incremento al marcar libre
        self.declare_parameter('log_odds_min', -3.0)       # clamp inferior
        self.declare_parameter('log_odds_max', 15.0)       # clamp superior
        self.declare_parameter('map_publish_rate', 1.0)    # Hz
        self.declare_parameter('save_path', os.path.join(
            os.path.expanduser('~'), 'SLAM', 'maps'))

        self.resolution = self.get_parameter('resolution').value
        map_w = self.get_parameter('map_width').value
        map_h = self.get_parameter('map_height').value
        self.log_occ = self.get_parameter('log_odds_occ').value
        self.log_free = self.get_parameter('log_odds_free').value
        self.log_min = self.get_parameter('log_odds_min').value
        self.log_max = self.get_parameter('log_odds_max').value
        pub_rate = self.get_parameter('map_publish_rate').value
        self.save_path = self.get_parameter('save_path').value

        # ── Grid ─────────────────────────────────────────────────────
        self.width = int(map_w / self.resolution)   # celdas en x
        self.height = int(map_h / self.resolution)  # celdas en y

        # Origen: el robot empieza en el centro del mapa
        self.origin_x = -map_w / 2.0
        self.origin_y = -map_h / 2.0

        # Log-odds grid: 0.0 = desconocido (50% probabilidad)
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)

        # ── Estado del robot (última odometría) ──────────────────────
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False

        # ── Suscripciones ────────────────────────────────────────────
        # LiDAR: muchos drivers publican con BEST_EFFORT
        qos_scan = QoSProfile(depth=5)
        qos_scan.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_scan)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # ── Publicador del mapa ──────────────────────────────────────
        qos_map = QoSProfile(depth=1)
        qos_map.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos_map.reliability = QoSReliabilityPolicy.RELIABLE

        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos_map)

        # TFs son publicados por ekf_slam_node — el mapper solo construye el grid

        # ── Timer para publicar mapa ─────────────────────────────────
        self.create_timer(1.0 / pub_rate, self._publish_map)

        self.get_logger().info(
            f'🗺️  Mapper listo — grid {self.width}×{self.height} '
            f'({map_w}×{map_h} m, res {self.resolution} m/cel)')

    # ─────────────────────────────────────────────────────────────────
    #  TF estática map → odom
    # ─────────────────────────────────────────────────────────────────
    def _publish_static_tfs(self):
        now = self.get_clock().now().to_msg()

        # map → odom (identidad: asumimos odometría perfecta)
        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = now
        t_map_odom.header.frame_id = 'map'
        t_map_odom.child_frame_id = 'odom'
        t_map_odom.transform.rotation.w = 1.0

        # base_link → laser (ajusta x/y/z si el LiDAR no está en el centro del robot)
        t_base_laser = TransformStamped()
        t_base_laser.header.stamp = now
        t_base_laser.header.frame_id = 'base_link'
        t_base_laser.child_frame_id = 'laser'
        t_base_laser.transform.rotation.w = 1.0  # sin rotación

        self._static_tf.sendTransform([t_map_odom, t_base_laser])

    # ─────────────────────────────────────────────────────────────────
    #  Callbacks
    # ─────────────────────────────────────────────────────────────────
    def _odom_cb(self, msg):
        """Almacena la última pose del robot."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_received = True

    def _scan_cb(self, msg):
        """Procesa un escaneo LiDAR: ray trace + actualización log-odds."""
        if not self.odom_received:
            return

        rx, ry, ryaw = self.robot_x, self.robot_y, self.robot_yaw
        gx0, gy0 = self._world_to_grid(rx, ry)

        # Si el robot está fuera del grid, no procesar
        if not (0 <= gx0 < self.width and 0 <= gy0 < self.height):
            self.get_logger().warn(
                f'Robot fuera del mapa: ({rx:.2f}, {ry:.2f}) → grid ({gx0}, {gy0})',
                throttle_duration_sec=2.0)
            return

        angle = msg.angle_min

        for r in msg.ranges:
            # Saltar lecturas inválidas
            if math.isinf(r) or math.isnan(r) or r < msg.range_min:
                angle += msg.angle_increment
                continue

            hit = r < msg.range_max

            # Si el rayo no golpea nada, no trazar (evita borrar paredes lejanas)
            if not hit:
                angle += msg.angle_increment
                continue

            eff_range = r

            # Punto final en coordenadas del mundo
            world_angle = ryaw + angle
            ex = rx + eff_range * math.cos(world_angle)
            ey = ry + eff_range * math.sin(world_angle)

            gx1, gy1 = self._world_to_grid(ex, ey)

            # Clamp a los bordes del grid
            gx1 = max(0, min(self.width - 1, gx1))
            gy1 = max(0, min(self.height - 1, gy1))

            # Bresenham: todas las celdas del rayo
            cells = self._bresenham(gx0, gy0, gx1, gy1)

            # Celdas intermedias → libres
            for cx, cy in cells[:-1]:
                if 0 <= cx < self.width and 0 <= cy < self.height:
                    self.log_odds[cy, cx] += self.log_free

            # Celda final → ocupada (el rayo pegó en algo)
            if cells:
                lx, ly = cells[-1]
                if 0 <= lx < self.width and 0 <= ly < self.height:
                    self.log_odds[ly, lx] += self.log_occ

            angle += msg.angle_increment

        # Clamp log-odds
        np.clip(self.log_odds, self.log_min, self.log_max, out=self.log_odds)

    # ─────────────────────────────────────────────────────────────────
    #  Utilidades de grid
    # ─────────────────────────────────────────────────────────────────
    def _world_to_grid(self, wx, wy):
        """Convierte coordenadas mundo (m) → coordenadas grid (celdas)."""
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return gx, gy

    @staticmethod
    def _bresenham(x0, y0, x1, y1):
        """Algoritmo de Bresenham — devuelve lista de celdas (x, y)."""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return cells

    # ─────────────────────────────────────────────────────────────────
    #  Publicación del mapa
    # ─────────────────────────────────────────────────────────────────
    def _publish_map(self):
        """Convierte log-odds → OccupancyGrid y publica en /map."""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0

        # Conversión: log-odds → valor de celda ROS
        #   -1 = desconocida,  0 = libre,  100 = ocupada
        grid = np.full(self.log_odds.shape, -1, dtype=np.int8)
        grid[self.log_odds < -0.5] = 0     # libre
        grid[self.log_odds > 0.5] = 100    # ocupada

        msg.data = grid.ravel().tolist()
        self.map_pub.publish(msg)

    # ─────────────────────────────────────────────────────────────────
    #  Guardar mapa a disco (.pgm + .yaml)
    # ─────────────────────────────────────────────────────────────────
    def save_map(self):
        """Guarda el mapa en formato compatible con ROS2 map_server."""
        os.makedirs(self.save_path, exist_ok=True)

        # Imagen PGM: 0=negro=ocupado, 254=blanco=libre, 205=gris=desconocido
        img = np.full((self.height, self.width), 205, dtype=np.uint8)
        img[self.log_odds < -0.5] = 254   # libre  → blanco
        img[self.log_odds > 0.5] = 0      # ocupada → negro

        # PGM usa origen arriba-izquierda; el mapa usa abajo-izquierda → flip
        img = np.flipud(img)

        pgm_path = os.path.join(self.save_path, 'map.pgm')
        with open(pgm_path, 'wb') as f:
            f.write(f'P5\n{self.width} {self.height}\n255\n'.encode())
            f.write(img.tobytes())

        yaml_path = os.path.join(self.save_path, 'map.yaml')
        with open(yaml_path, 'w') as f:
            f.write(f'image: map.pgm\n')
            f.write(f'resolution: {self.resolution}\n')
            f.write(f'origin: [{self.origin_x}, {self.origin_y}, 0.0]\n')
            f.write(f'negate: 0\n')
            f.write(f'occupied_thresh: 0.65\n')
            f.write(f'free_thresh: 0.196\n')

        # Guardar PNG
        import struct
        import zlib

        png_path = os.path.join(self.save_path, 'map.png')
        try:
            h, w = img.shape
            raw = b''
            for row in img:
                raw += b'\x00' + row.tobytes()

            compressed = zlib.compress(raw)

            def _chunk(chunk_type, data):
                c = chunk_type + data
                crc = struct.pack('>I', zlib.crc32(c) & 0xFFFFFFFF)
                return struct.pack('>I', len(data)) + c + crc

            with open(png_path, 'wb') as fp:
                fp.write(b'\x89PNG\r\n\x1a\n')
                ihdr_data = struct.pack('>IIBBBBB', w, h, 8, 0, 0, 0, 0)
                fp.write(_chunk(b'IHDR', ihdr_data))
                fp.write(_chunk(b'IDAT', compressed))
                fp.write(_chunk(b'IEND', b''))

            self.get_logger().info(f'🖼️  Mapa PNG guardado en {png_path}')
        except Exception as e:
            self.get_logger().warn(f'No se pudo guardar PNG: {e}')

        self.get_logger().info(f'💾 Mapa guardado en {self.save_path}/')

# ─────────────────────────────────────────────────────────────────────
#  main
# ─────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridMapper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Guardando mapa antes de cerrar... holiiis')
        node.save_map()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()