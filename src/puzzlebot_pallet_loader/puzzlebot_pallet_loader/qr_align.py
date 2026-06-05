import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

GREEN  = (0, 255, 0)
RED    = (0, 0, 255)
YELLOW = (0, 255, 255)
WHITE  = (255, 255, 255)

# Dimensiones reales del QR en metros
QR_W = 0.047
QR_H = 0.055

QR_3D = np.array([
    [0,    0,    0],
    [QR_W, 0,    0],
    [QR_W, QR_H, 0],
    [0,    QR_H, 0],
], dtype=np.float32)

STATE_ALIGNING = 'ALIGNING'
STATE_DONE        = 'DONE'


class QRAlignNode(Node):
    def __init__(self):
        super().__init__('qr_align')

        self.declare_parameter('kp_center', 0.4)
        self.declare_parameter('kp_skew', 0.8)
        self.declare_parameter('kp_skew_backup', 1.2)
        self.declare_parameter('max_angular',     0.1)
        self.declare_parameter('forward_speed',   0.02)
        self.declare_parameter('backup_speed_ratio', 0.7)
        self.declare_parameter('center_threshold', 0.02)
        self.declare_parameter('skew_threshold', 0.10)
        self.declare_parameter('too_close_ratio', 0.33)
        self.declare_parameter('calib_file', '/home/kim/puzzlebot-pallet-loader/src/camera_dataset/camera_calibration.npz')

        calib_file = self.get_parameter('calib_file').value
        try:
            calib = np.load(calib_file)
            self._camera_mat  = calib['camera_matrix']
            self._dist_coeffs = calib['dist_coeffs']
            self._have_calib  = True
            self.get_logger().info('Calibración cargada.')
        except Exception as e:
            self.get_logger().warn(f'Sin calibración: {e}')
            self._have_calib = False

        self._qr    = cv2.QRCodeDetector()
        self._state = STATE_ALIGNING

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self._on_image,
            10,
        )

        self.get_logger().info('QRAlignNode listo para alinear.')

    # ── CALLBACK IMAGEN ──────────────────────────────────────────────────────
    def _on_image(self, msg: CompressedImage):
        if self._state == STATE_DONE:
            return

        buf   = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            return

        h, w = frame.shape[:2]
        img_cx, img_cy = w / 2.0, h / 2.0
        cx, cy = int(img_cx), int(img_cy)

        cv2.drawMarker(frame, (cx, cy), RED,
                       markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)

        _, points, _ = self._qr.detectAndDecode(frame)
        cmd = Twist()

        if points is None or len(points) == 0:
            cv2.putText(frame, 'Buscando QR...', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, WHITE, 2)
            self._cmd_pub.publish(cmd)
            cv2.imshow('QR Align', frame)
            cv2.waitKey(1)
            return

        pts = points[0].astype(np.float32)
        
        # --- 1. Error de centrado horizontal ---
        qr_cx = float(np.mean(pts[:, 0]))
        error_center = (qr_cx - img_cx) / (w / 2.0)

        # --- 2. Error de sesgo (perspectiva) para alineación frontal ---
        # Compara la altura de los bordes verticales del QR en la imagen.
        # Si el robot no está de frente, un borde se verá más alto que el otro.
        p0, p1, p2, p3 = pts[0], pts[1], pts[2], pts[3]
        h_left = np.linalg.norm(p3 - p0)
        h_right = np.linalg.norm(p2 - p1)
        
        error_skew = 0.0
        if (h_left + h_right) > 1e-6:
            error_skew = (h_left - h_right) / ((h_left + h_right) / 2.0)

        # --- Parámetros de control ---
        kp_center = self.get_parameter('kp_center').value
        kp_skew   = self.get_parameter('kp_skew').value
        kp_skew_backup = self.get_parameter('kp_skew_backup').value
        max_ang   = self.get_parameter('max_angular').value
        fwd       = self.get_parameter('forward_speed').value
        backup_ratio = self.get_parameter('backup_speed_ratio').value
        thresh_center = self.get_parameter('center_threshold').value
        thresh_skew   = self.get_parameter('skew_threshold').value
        too_close_ratio = self.get_parameter('too_close_ratio').value

        is_centered = abs(error_center) < thresh_center
        is_frontal  = abs(error_skew) < thresh_skew

        # --- Visualización ---
        qr_center = (int(qr_cx), int(np.mean(pts[:, 1])))
        cv2.polylines(frame, [pts.astype(np.int32)], True, GREEN, 2)
        cv2.drawMarker(frame, qr_center, GREEN,
                       markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
        cv2.line(frame, (cx, cy), qr_center, YELLOW, 1)
        cv2.putText(frame, f'Error Centro: {error_center:+.3f}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, WHITE, 2)
        cv2.putText(frame, f'Error Sesgo: {error_skew:+.3f}',
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, WHITE, 2)

        # --- Lógica de estados y control ---
        if is_centered and is_frontal:
            self._state = STATE_DONE
            self._cmd_pub.publish(Twist())
            self.get_logger().info('Alineación completa. Robot detenido.')
            cv2.putText(frame, 'ALINEADO', (cx - 50, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, GREEN, 2)
        else:
            # --- Lógica de maniobra tipo estacionamiento ---
            avg_qr_height = (h_left + h_right) / 2.0
            is_too_close = avg_qr_height > (h * too_close_ratio)

            # Activar maniobra de reversa si el ángulo es muy cerrado y estamos
            # cerca, o si ya estamos centrados pero no de frente.
            needs_backup = abs(error_skew) > thresh_skew and (is_too_close or abs(error_center) < 0.25)

            if needs_backup:
                self.get_logger().info('Maniobra de reversa: ángulo cerrado.', throttle_duration_sec=1)
                cv2.putText(frame, 'REVERSA', (cx - 60, cy - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, YELLOW, 2)

                cmd.linear.x = -fwd * backup_ratio
                angular_z = -kp_skew_backup * error_skew
                cmd.angular.z = float(np.clip(angular_z, -max_ang, max_ang))
            else:
                cv2.putText(frame, 'AVANZANDO', (cx - 70, cy - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, YELLOW, 2)
                # Maniobra de avance normal
                angular_z = -kp_center * error_center + kp_skew * error_skew
                cmd.angular.z = float(np.clip(angular_z, -max_ang, max_ang))
                
                scale = max(0, 1.0 - 2.0 * abs(error_center))
                cmd.linear.x = float(fwd * scale)
            
            self._cmd_pub.publish(cmd)

        cv2.imshow('QR Align', frame)
        cv2.waitKey(1)

    # ── ESTIMACIÓN DE DISTANCIA (solo para info, no se usa en control) ──
    def _estimate_distance(self, img_pts: np.ndarray):
        if not self._have_calib:
            return None
        ok, _, tvec = cv2.solvePnP(
            QR_3D, img_pts,
            self._camera_mat, self._dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE)
        if ok:
            return float(tvec[2])
        return None

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


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
