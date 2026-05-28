"""
ArUco Detector Node — Detección de marcadores ArUco con cámara

Suscribe:
    /camera       (sensor_msgs/Image)       — imagen de la cámara
    /camera_info  (sensor_msgs/CameraInfo)  — parámetros intrínsecos

Publica:
    /aruco_measurements  (std_msgs/Float32MultiArray)
        Cada mensaje contiene [marker_id, range, bearing] de UN marcador detectado.
        range  = distancia euclidiana al marcador (m) en frame del robot
        bearing = ángulo horizontal al marcador (rad) en frame del robot

    /aruco_markers_viz   (visualization_msgs/MarkerArray)
        Esferas en RViz para depuración visual de detecciones.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from cv_bridge import CvBridge
import cv2


class ArucoDetectorNode(Node):

    def __init__(self):
        super().__init__('aruco_detector_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('aruco_dictionary', 0)
        self.declare_parameter('marker_size', 0.15)
        self.declare_parameter('camera_topic', '/camera')
        self.declare_parameter('camera_info_topic', '/camera_info')

        dict_id = self.get_parameter('aruco_dictionary').value
        self._marker_size = self.get_parameter('marker_size').value
        camera_topic = self.get_parameter('camera_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value

        # ── ArUco setup ───────────────────────────────────────────────
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self._aruco_params = cv2.aruco.DetectorParameters_create()

        # ── Camera intrinsics (filled on first /camera_info) ─────────
        self._camera_matrix = None
        self._dist_coeffs = None

        # ── CV bridge ─────────────────────────────────────────────────
        self._bridge = CvBridge()

        # ── QoS ──────────────────────────────────────────────────────
        qos_sensor = QoSProfile(depth=5)
        qos_sensor.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # ── Subscriptions ─────────────────────────────────────────────
        self.create_subscription(
            Image, camera_topic, self._image_cb, qos_sensor)
        self.create_subscription(
            CameraInfo, camera_info_topic, self._camera_info_cb, qos_sensor)

        # ── Publishers ────────────────────────────────────────────────
        self._meas_pub = self.create_publisher(
            Float32MultiArray, '/aruco_measurements', 10)
        self._viz_pub = self.create_publisher(
            MarkerArray, '/aruco_markers_viz', 10)

        self.get_logger().info(
            f'📷 ArUco detector started — dict={dict_id}, '
            f'marker_size={self._marker_size}m')

    # ── Camera info callback ──────────────────────────────────────────

    def _camera_info_cb(self, msg: CameraInfo):
        if self._camera_matrix is not None:
            return  # already initialized
        # K is a 9-element row-major array: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        K = np.array(msg.k).reshape(3, 3)
        D = np.array(msg.d) if len(msg.d) > 0 else np.zeros(5)
        self._camera_matrix = K
        self._dist_coeffs = D
        self.get_logger().info(
            f'📸 Camera intrinsics received — fx={K[0,0]:.1f}, '
            f'fy={K[1,1]:.1f}, cx={K[0,2]:.1f}, cy={K[1,2]:.1f}')

    # ── Image callback ────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        if self._camera_matrix is None:
            return  # wait for camera_info

        # Convert ROS Image → OpenCV BGR
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(
            frame, self._aruco_dict, parameters=self._aruco_params)

        if ids is None or len(ids) == 0:
            return

        # Estimate pose of each marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self._marker_size,
            self._camera_matrix, self._dist_coeffs)

        viz_markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, marker_id in enumerate(ids.flatten()):
            tvec = tvecs[i][0]  # [x, y, z] in camera_optical frame

            # Camera optical frame convention:
            #   x = right, y = down, z = forward
            # Convert to robot base_link convention:
            #   x = forward, y = left, z = up
            # The transform camera_link_optical → camera_link is:
            #   R = Rz(-π/2) · Rx(-π/2)  which gives:
            #   x_robot ≈ z_cam, y_robot ≈ -x_cam
            #
            # We only need range and bearing in 2D (horizontal plane):
            z_cam = tvec[2]  # depth = forward distance in camera
            x_cam = tvec[0]  # horizontal offset in camera

            # Range = Euclidean distance in the horizontal plane
            # Using camera optical z (depth) and x (horizontal)
            range_m = math.sqrt(z_cam**2 + x_cam**2)

            # Bearing = angle from robot's forward axis
            # In camera optical frame: atan2(-x_cam, z_cam)
            # The negative on x_cam converts camera-right to robot-left
            bearing_rad = math.atan2(-x_cam, z_cam)

            # Publish measurement
            meas = Float32MultiArray()
            meas.data = [float(marker_id), range_m, bearing_rad]
            self._meas_pub.publish(meas)

            # Visualization marker (sphere at detected position relative to robot)
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = 'base_link'
            m.ns = 'aruco_detections'
            m.id = int(marker_id)
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            # Place sphere at the detected 3D position (in base_link frame)
            m.pose.position.x = z_cam   # forward
            m.pose.position.y = -x_cam  # left
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.12
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.b = 0.0
            m.color.a = 0.9
            m.lifetime.sec = 1  # auto-expire after 1s
            viz_markers.markers.append(m)

            self.get_logger().debug(
                f'ArUco #{marker_id}: range={range_m:.2f}m, '
                f'bearing={math.degrees(bearing_rad):.1f}°')

        self._viz_pub.publish(viz_markers)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()

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
