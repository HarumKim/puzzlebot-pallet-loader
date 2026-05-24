"""
Map Viewer — Visualizador ligero del mapa de ocupación (sin RViz)

Suscribe a:
  - /map   (OccupancyGrid)  → dibuja el mapa como imagen
  - /odom  (Odometry)       → dibuja pose del robot (flecha)
  - /scan  (LaserScan)      → dibuja los puntos LiDAR sobre el mapa

Uso:
  ros2 run slam_pkg map_viewer

Controles:
  - La ventana se actualiza automáticamente (~5 Hz)
  - Ctrl+C para cerrar
"""

import math
import threading

import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # Backend interactivo
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.patches import FancyArrowPatch

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid


class MapViewer(Node):
    def __init__(self):
        super().__init__('map_viewer')

        # ── Parámetros del LiDAR (deben coincidir con el mapper) ────
        self.declare_parameter('lidar_x', 0.07)
        self.declare_parameter('lidar_y', 0.0)
        self.declare_parameter('lidar_yaw', 0.0)
        self.declare_parameter('invert_scan', True)
        self.declare_parameter('update_rate', 5.0)  # Hz de refresco visual

        self.lidar_x = self.get_parameter('lidar_x').value
        self.lidar_y = self.get_parameter('lidar_y').value
        self.lidar_yaw = self.get_parameter('lidar_yaw').value
        self.invert_scan = self.get_parameter('invert_scan').value
        update_rate = self.get_parameter('update_rate').value

        # ── Estado compartido (protegido con lock) ──────────────────
        self._lock = threading.Lock()

        # Mapa
        self._map_img = None        # np.array HxW con valores -1, 0..100
        self._map_resolution = 0.05
        self._map_origin_x = 0.0
        self._map_origin_y = 0.0
        self._map_width = 0
        self._map_height = 0
        self._map_updated = False

        # Odometría
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._odom_received = False

        # Scan (puntos LiDAR en coordenadas mundo)
        self._scan_xs = np.array([])
        self._scan_ys = np.array([])
        self._scan_updated = False

        # ── Suscripciones ───────────────────────────────────────────
        qos_map = QoSProfile(depth=1)
        qos_map.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos_map.reliability = QoSReliabilityPolicy.RELIABLE

        qos_scan = QoSProfile(depth=5)
        qos_scan.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.create_subscription(OccupancyGrid, '/map', self._map_cb, qos_map)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_scan)

        # ── Timer para refrescar matplotlib ─────────────────────────
        self._timer_period = 1.0 / update_rate
        # (el timer de matplotlib se maneja desde el hilo principal)

        self.get_logger().info(
            f'👁️  MapViewer listo — refresco a {update_rate} Hz')

    # ────────────────────────────────────────────────────────────────
    #  Callbacks (corren en el hilo de rclpy.spin)
    # ────────────────────────────────────────────────────────────────
    def _map_cb(self, msg):
        w = msg.info.width
        h = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))

        with self._lock:
            self._map_img = data
            self._map_resolution = msg.info.resolution
            self._map_origin_x = msg.info.origin.position.x
            self._map_origin_y = msg.info.origin.position.y
            self._map_width = w
            self._map_height = h
            self._map_updated = True

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        with self._lock:
            self._robot_x = msg.pose.pose.position.x
            self._robot_y = msg.pose.pose.position.y
            self._robot_yaw = yaw
            self._odom_received = True

    def _scan_cb(self, msg):
        with self._lock:
            rx, ry, ryaw = self._robot_x, self._robot_y, self._robot_yaw

        # Posición del LiDAR en el mundo
        lx = rx + math.cos(ryaw) * self.lidar_x - math.sin(ryaw) * self.lidar_y
        ly = ry + math.sin(ryaw) * self.lidar_x + math.cos(ryaw) * self.lidar_y
        lyaw = ryaw + self.lidar_yaw

        xs = []
        ys = []
        angle = msg.angle_min
        for r in msg.ranges:
            if not (math.isinf(r) or math.isnan(r) or r < msg.range_min or r >= msg.range_max):
                scan_angle = -angle if self.invert_scan else angle
                world_angle = lyaw + scan_angle
                xs.append(lx + r * math.cos(world_angle))
                ys.append(ly + r * math.sin(world_angle))
            angle += msg.angle_increment

        with self._lock:
            self._scan_xs = np.array(xs)
            self._scan_ys = np.array(ys)
            self._scan_updated = True

    # ────────────────────────────────────────────────────────────────
    #  Utilidades
    # ────────────────────────────────────────────────────────────────
    def get_snapshot(self):
        """Devuelve una copia thread-safe de todos los datos para dibujar."""
        with self._lock:
            return {
                'map_img': self._map_img.copy() if self._map_img is not None else None,
                'map_res': self._map_resolution,
                'map_ox': self._map_origin_x,
                'map_oy': self._map_origin_y,
                'map_w': self._map_width,
                'map_h': self._map_height,
                'map_updated': self._map_updated,
                'robot_x': self._robot_x,
                'robot_y': self._robot_y,
                'robot_yaw': self._robot_yaw,
                'odom_ok': self._odom_received,
                'scan_xs': self._scan_xs.copy(),
                'scan_ys': self._scan_ys.copy(),
            }


def _render_map_image(data):
    """
    Convierte OccupancyGrid data (int8) a imagen RGBA para imshow.
      -1 → gris medio (desconocido)
       0 → blanco (libre)
     100 → negro (ocupado)
    """
    h, w = data.shape
    img = np.zeros((h, w, 3), dtype=np.float32)

    unknown = data == -1
    free = data == 0
    occupied = data == 100

    # Desconocido → gris
    img[unknown] = [0.55, 0.55, 0.60]
    # Libre → blanco cremoso
    img[free] = [0.95, 0.95, 0.92]
    # Ocupado → negro
    img[occupied] = [0.1, 0.1, 0.12]

    return img


def run_viewer(node):
    """Bucle principal de matplotlib (corre en el hilo principal)."""

    plt.ion()
    fig, ax = plt.subplots(1, 1, figsize=(9, 9))
    fig.canvas.manager.set_window_title('🗺️  Map Viewer (sin RViz)')
    fig.patch.set_facecolor('#1e1e2e')
    ax.set_facecolor('#1e1e2e')
    ax.set_aspect('equal')
    ax.set_title('Esperando datos del mapa...', color='white', fontsize=14)
    ax.tick_params(colors='#888888')
    for spine in ax.spines.values():
        spine.set_color('#444444')

    im_handle = None
    robot_dot = None
    robot_arrow = None
    scan_dots = None

    frame_count = 0

    try:
        while rclpy.ok():
            snap = node.get_snapshot()

            # ── Dibujar mapa ────────────────────────────────────────
            if snap['map_img'] is not None:
                res = snap['map_res']
                ox = snap['map_ox']
                oy = snap['map_oy']
                w = snap['map_w']
                h = snap['map_h']

                extent = [ox, ox + w * res, oy, oy + h * res]
                rgb = _render_map_image(snap['map_img'])

                if im_handle is None:
                    im_handle = ax.imshow(
                        rgb, origin='lower', extent=extent,
                        interpolation='nearest')
                    ax.set_xlabel('X (m)', color='#aaaaaa')
                    ax.set_ylabel('Y (m)', color='#aaaaaa')
                else:
                    im_handle.set_data(rgb)
                    im_handle.set_extent(extent)

            # ── Dibujar scan (puntos LiDAR) ─────────────────────────
            if scan_dots is not None:
                scan_dots.remove()
                scan_dots = None

            if len(snap['scan_xs']) > 0:
                scan_dots = ax.scatter(
                    snap['scan_xs'], snap['scan_ys'],
                    s=2, c='#ff5555', alpha=0.7, zorder=5,
                    label='LiDAR' if frame_count == 0 else '')

            # ── Dibujar robot ───────────────────────────────────────
            if snap['odom_ok']:
                rx = snap['robot_x']
                ry = snap['robot_y']
                ryaw = snap['robot_yaw']
                arrow_len = 0.25

                if robot_dot is not None:
                    robot_dot.remove()
                if robot_arrow is not None:
                    robot_arrow.remove()

                robot_dot = ax.plot(
                    rx, ry, 'o', color='#50fa7b', markersize=8,
                    markeredgecolor='white', markeredgewidth=1.5,
                    zorder=10)[0]

                robot_arrow = ax.annotate(
                    '', xy=(rx + arrow_len * math.cos(ryaw),
                            ry + arrow_len * math.sin(ryaw)),
                    xytext=(rx, ry),
                    arrowprops=dict(
                        arrowstyle='->', color='#50fa7b',
                        lw=2.5),
                    zorder=10)

            # ── Título con info ─────────────────────────────────────
            if snap['odom_ok']:
                ax.set_title(
                    f'x={snap["robot_x"]:.2f}  y={snap["robot_y"]:.2f}  '
                    f'yaw={math.degrees(snap["robot_yaw"]):.1f}°  |  '
                    f'Grid {snap["map_w"]}×{snap["map_h"]}  '
                    f'res={snap["map_res"]:.3f} m/cel',
                    color='white', fontsize=12, fontfamily='monospace')

            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            frame_count += 1

            plt.pause(node._timer_period)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Viewer cerrado: {e}')
    finally:
        plt.close('all')


# ────────────────────────────────────────────────────────────────────
#  main
# ────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = MapViewer()

    # Correr rclpy.spin en un hilo aparte para no bloquear matplotlib
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        run_viewer(node)
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
