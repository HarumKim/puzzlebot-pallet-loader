import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

"""
Suscribe /camera/image_raw/compressed, detecta un QR y publica /cmd_vel
para alinear el robot al centro del QR y acercarse hasta target_distance.

Ejecutar:
  ros2 run camera_dataset qr_align
  ros2 run camera_dataset qr_align --ros-args \
    -p calib_file:=/ruta/calibration.npz \
    -p target_distance:=0.4
"""

# ── DIMENSIONES DEL QR EN EL PALLET ───────────────────────────────────────
QR_WIDTH  = 0.12   # metros
QR_HEIGHT = 0.11   # metros

QR_3D_POINTS = np.array([
    [0,        0,         0],
    [QR_WIDTH, 0,         0],
    [QR_WIDTH, QR_HEIGHT, 0],
    [0,        QR_HEIGHT, 0],
], dtype=np.float32)
# ──────────────────────────────────────────────────────────────────────────


class QRAlignNode(Node):
    def __init__(self):
        super().__init__('qr_align')

        import os
        _pkg = os.path.join(os.path.dirname(__file__), '..')
        self.declare_parameter('calib_file',
            os.path.join(_pkg, 'camera_calibration.npz'))
        self.declare_parameter('kp_angular',    1.5)   # ganancia giro
        self.declare_parameter('kp_linear',     0.3)   # ganancia avance
        self.declare_parameter('target_distance', 0.4) # metros para detenerse
        self.declare_parameter('align_threshold', 0.05) # error lateral máx para avanzar (ratio)
        self.declare_parameter('search_speed',   0.3)  # rad/s cuando no ve QR
        self.declare_parameter('max_linear',     0.20) # m/s
        self.declare_parameter('max_angular',    0.50) # rad/s

        calib_file = self.get_parameter('calib_file').value
        try:
            calib = np.load(calib_file)
            self.camera_mat  = calib['camera_matrix']
            self.dist_coeffs = calib['dist_coeffs']
            self._have_calib = True
        except Exception as e:
            self.get_logger().warn(f'No se cargó calibración ({e}). Se usará solo alineación por píxeles.')
            self._have_calib = False

        self._qr = cv2.QRCodeDetector()

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed',
            self._on_image, 10)

        self.get_logger().info('QRAlignNode iniciado — esperando imágenes...')

    # ── CALLBACK ──────────────────────────────────────────────────────────
    def _on_image(self, msg: CompressedImage):
        buf   = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            return

        _, w  = frame.shape[:2]
        img_cx = w / 2.0

        data, points, _ = self._qr.detectAndDecode(frame)

        cmd = Twist()

        if points is None or len(points) == 0:
            # Sin QR: girar despacio para buscarlo
            cmd.angular.z = self.get_parameter('search_speed').value
            self._cmd_pub.publish(cmd)
            self.get_logger().debug('QR no detectado — buscando...')
            return

        img_pts = points[0].astype(np.float32)
        qr_cx   = float(np.mean(img_pts[:, 0]))

        # Error lateral normalizado en [-1, 1]
        error = (qr_cx - img_cx) / (w / 2.0)

        kp_ang   = self.get_parameter('kp_angular').value
        max_ang  = self.get_parameter('max_angular').value
        cmd.angular.z = float(np.clip(-kp_ang * error, -max_ang, max_ang))

        # Avanzar solo si está suficientemente centrado
        align_thresh = self.get_parameter('align_threshold').value
        z_dist       = None

        if self._have_calib:
            ok, _, tvec = cv2.solvePnP(
                QR_3D_POINTS, img_pts,
                self.camera_mat, self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                z_dist = float(tvec[2])

        if abs(error) < align_thresh:
            target = self.get_parameter('target_distance').value
            kp_lin = self.get_parameter('kp_linear').value
            max_lin = self.get_parameter('max_linear').value

            if z_dist is not None:
                advance = kp_lin * max(0.0, z_dist - target)
            else:
                # Sin calibración: avanzar a velocidad fija hasta que el QR ocupe el frame
                advance = 0.1

            cmd.linear.x = float(np.clip(advance, 0.0, max_lin))

        self._cmd_pub.publish(cmd)
        self.get_logger().info(
            f"QR='{data}' | error={error:+.2f} | "
            f"dist={z_dist:.3f}m | " if z_dist else
            f"QR='{data}' | error={error:+.2f} | dist=? | "
            f"cmd=(lin={cmd.linear.x:.2f}, ang={cmd.angular.z:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = QRAlignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
