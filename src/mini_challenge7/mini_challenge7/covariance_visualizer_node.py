"""
Covariance Visualizer Node — Elipse de confianza en RViz

Suscribe:
    /ekf_odom  (nav_msgs/Odometry) — pose con covarianza del EKF

Publica:
    /covariance_ellipse  (visualization_msgs/Marker)
        Elipse 95% de confianza como un CYLINDER aplanado en RViz.
        - Crece cuando solo hay predicción (odometría).
        - Se encoge cuando hay corrección ArUco.
        - Color: verde (baja incertidumbre) → rojo (alta incertidumbre).
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion


# χ² con 2 grados de libertad al 95% de confianza
CHI2_95 = 5.991


class CovarianceVisualizerNode(Node):

    def __init__(self):
        super().__init__('covariance_visualizer_node')

        # ── Subscription ──────────────────────────────────────────────
        self.create_subscription(Odometry, '/ekf_odom', self._odom_cb, 10)

        # ── Publisher ─────────────────────────────────────────────────
        self._pub = self.create_publisher(Marker, '/covariance_ellipse', 10)

        self.get_logger().info('🔵 Covariance visualizer started')

    def _odom_cb(self, msg: Odometry):
        # Extract 2×2 position covariance from the 6×6 Odometry covariance
        # Indices: cov[0]=xx, cov[1]=xy, cov[6]=yx, cov[7]=yy
        cov = msg.pose.covariance
        Sigma_2d = np.array([
            [cov[0], cov[1]],
            [cov[6], cov[7]]
        ])

        # Eigendecomposition for ellipse axes
        try:
            eigenvalues, eigenvectors = np.linalg.eigh(Sigma_2d)
        except np.linalg.LinAlgError:
            return

        # Clamp eigenvalues to avoid sqrt of negative
        eigenvalues = np.maximum(eigenvalues, 1e-10)

        # Ellipse semi-axes = sqrt(eigenvalue * χ²₀.₉₅)
        # Scale factor 2 to get the full axis (diameter)
        scale_x = 2.0 * math.sqrt(eigenvalues[1] * CHI2_95)
        scale_y = 2.0 * math.sqrt(eigenvalues[0] * CHI2_95)

        # Orientation of the ellipse (angle of the major eigenvector)
        angle = math.atan2(eigenvectors[1, 1], eigenvectors[0, 1])

        # Color interpolation: green (small) → yellow → red (large)
        # Use the trace of the covariance as a scalar measure
        trace = float(eigenvalues[0] + eigenvalues[1])
        # Normalize: 0.0 at trace=0, 1.0 at trace≥0.5
        intensity = min(1.0, trace / 0.5)

        r = min(1.0, 2.0 * intensity)
        g = min(1.0, 2.0 * (1.0 - intensity))

        # Build marker
        marker = Marker()
        marker.header.stamp = msg.header.stamp
        marker.header.frame_id = msg.header.frame_id  # "odom"
        marker.ns = 'covariance'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        # Position at the robot's estimated pose
        marker.pose.position.x = msg.pose.pose.position.x
        marker.pose.position.y = msg.pose.pose.position.y
        marker.pose.position.z = 0.01  # slightly above ground

        # Orientation from eigenvector angle
        marker.pose.orientation = _yaw_to_quaternion(angle)

        # Scale: x,y are the ellipse diameters, z is thin
        marker.scale.x = max(scale_x, 0.01)
        marker.scale.y = max(scale_y, 0.01)
        marker.scale.z = 0.01

        marker.color.r = r
        marker.color.g = g
        marker.color.b = 0.0
        marker.color.a = 0.4

        self._pub.publish(marker)


def _yaw_to_quaternion(yaw):
    return Quaternion(
        x=0.0, y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0)
    )


def main(args=None):
    rclpy.init(args=args)
    node = CovarianceVisualizerNode()

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


if __name__ == '__main__':
    main()
