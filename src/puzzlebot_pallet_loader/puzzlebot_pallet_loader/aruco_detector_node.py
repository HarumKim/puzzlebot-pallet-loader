"""
ArUco detector for the real Puzzlebot camera.

Subscribes:
    /camera/image_raw/compressed (sensor_msgs/CompressedImage)

Publishes:
    /aruco_measurements (std_msgs/Float32MultiArray)
        [marker_id, range, bearing]
    /aruco_markers_viz (visualization_msgs/MarkerArray)
    /aruco/annotated (sensor_msgs/Image), optional debug image
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        self.declare_parameter('camera_topic', '/camera/image_raw/compressed')
        self.declare_parameter(
            'calib_file',
            '/home/valeria/puzzlebot-pallet-loader/src/camera_dataset/camera_calibration.npz',
        )
        self.declare_parameter('marker_size', 0.08)
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('camera_x_offset', 0.08)
        self.declare_parameter('camera_y_offset', 0.0)
        self.declare_parameter('camera_yaw_offset', 0.0)
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('annotated_image_topic', '/aruco/annotated')
        self.declare_parameter('measurements_topic', '/aruco_measurements')
        self.declare_parameter('allowed_marker_ids', list(range(11)))

        camera_topic = self.get_parameter('camera_topic').value
        calib_file = self.get_parameter('calib_file').value
        self._marker_size = float(self.get_parameter('marker_size').value)
        dict_param = self.get_parameter('aruco_dictionary').value
        self.cam_x_offset = float(self.get_parameter('camera_x_offset').value)
        self.cam_y_offset = float(self.get_parameter('camera_y_offset').value)
        self.cam_yaw_offset = float(self.get_parameter('camera_yaw_offset').value)
        self._publish_annotated = bool(
            self.get_parameter('publish_annotated_image').value
        )
        annotated_topic = self.get_parameter('annotated_image_topic').value
        measurements_topic = self.get_parameter('measurements_topic').value
        allowed_marker_ids = self.get_parameter('allowed_marker_ids').value
        self._allowed_marker_ids = {int(marker_id) for marker_id in allowed_marker_ids}

        self._camera_matrix, self._dist_coeffs = self._load_calibration(calib_file)
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(
            self._resolve_dictionary(dict_param)
        )
        self._aruco_params = self._create_detector_parameters()
        self._aruco_detector = self._create_detector()
        self._bridge = CvBridge()

        qos_sensor = QoSProfile(depth=5)
        qos_sensor.reliability = QoSReliabilityPolicy.BEST_EFFORT

        if camera_topic.endswith('/compressed'):
            self.create_subscription(
                CompressedImage, camera_topic, self._compressed_image_cb, qos_sensor
            )
            input_type = 'CompressedImage'
        else:
            self.create_subscription(Image, camera_topic, self._image_cb, qos_sensor)
            input_type = 'Image'

        self._meas_pub = self.create_publisher(
            Float32MultiArray, measurements_topic, 10
        )
        self._viz_pub = self.create_publisher(
            MarkerArray, '/aruco_markers_viz', 10
        )
        self._annotated_pub = None
        if self._publish_annotated:
            self._annotated_pub = self.create_publisher(Image, annotated_topic, 10)

        self.get_logger().info(
            'ArUco detector ready: '
            f'topic={camera_topic} ({input_type}), dict={dict_param}, '
            f'marker_size={self._marker_size:.3f}m, calib={calib_file}, '
            f'allowed_ids={sorted(self._allowed_marker_ids)}'
        )

    def _load_calibration(self, calib_file):
        try:
            calib = np.load(calib_file)
            camera_matrix = np.asarray(calib['camera_matrix'], dtype=np.float64)
            dist_coeffs = np.asarray(calib['dist_coeffs'], dtype=np.float64)
        except Exception as exc:
            raise RuntimeError(
                f'Could not load camera calibration from {calib_file}: {exc}'
            ) from exc

        if camera_matrix.shape != (3, 3):
            raise RuntimeError(
                f'camera_matrix must be 3x3, got {camera_matrix.shape}'
            )

        dist_coeffs = dist_coeffs.reshape(-1)
        self.get_logger().info(
            'Loaded camera calibration: '
            f'fx={camera_matrix[0, 0]:.1f}, fy={camera_matrix[1, 1]:.1f}, '
            f'cx={camera_matrix[0, 2]:.1f}, cy={camera_matrix[1, 2]:.1f}, '
            f'D={dist_coeffs.tolist()}'
        )
        return camera_matrix, dist_coeffs

    def _resolve_dictionary(self, dict_param):
        if isinstance(dict_param, str):
            name = dict_param.strip()
            if name.isdigit():
                return int(name)
            if not name.startswith('DICT_'):
                name = f'DICT_{name}'
            if not hasattr(cv2.aruco, name):
                raise RuntimeError(f'Unknown ArUco dictionary: {dict_param}')
            return getattr(cv2.aruco, name)
        return int(dict_param)

    def _create_detector_parameters(self):
        if hasattr(cv2.aruco, 'DetectorParameters'):
            return cv2.aruco.DetectorParameters()
        return cv2.aruco.DetectorParameters_create()

    def _create_detector(self):
        if hasattr(cv2.aruco, 'ArucoDetector'):
            return cv2.aruco.ArucoDetector(self._aruco_dict, self._aruco_params)
        return None

    def _detect_markers(self, frame):
        if self._aruco_detector is not None:
            return self._aruco_detector.detectMarkers(frame)
        return cv2.aruco.detectMarkers(
            frame, self._aruco_dict, parameters=self._aruco_params
        )

    def _image_cb(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge error: {exc}')
            return

        self._process_frame(frame, msg.header)

    def _compressed_image_cb(self, msg):
        buffer = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('Could not decode compressed camera image')
            return

        self._process_frame(frame, msg.header)

    def _process_frame(self, frame, header):
        corners, ids, _ = self._detect_markers(frame)
        annotated = frame.copy() if self._publish_annotated else None

        if ids is None or len(ids) == 0:
            self._publish_annotated_image(annotated, header)
            return

        _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            self._marker_size,
            self._camera_matrix,
            self._dist_coeffs,
        )

        viz_markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        for index, marker_id in enumerate(ids.flatten()):
            marker_id = int(marker_id)
            if marker_id not in self._allowed_marker_ids:
                self.get_logger().debug(
                    f'ArUco #{marker_id} ignored: not in allowed_marker_ids'
                )
                continue

            if annotated is not None:
                cv2.aruco.drawDetectedMarkers(
                    annotated, [corners[index]], np.array([[marker_id]])
                )

            tvec = tvecs[index][0]
            x_cam = float(tvec[0])
            z_cam = float(tvec[2])

            x_base = self.cam_x_offset + z_cam
            y_base = self.cam_y_offset - x_cam
            range_m = math.hypot(x_base, y_base)
            bearing_rad = math.atan2(y_base, x_base) + self.cam_yaw_offset

            measurement = Float32MultiArray()
            measurement.data = [float(marker_id), range_m, bearing_rad]
            self._meas_pub.publish(measurement)

            viz_markers.markers.append(
                self._make_viz_marker(
                    marker_id=marker_id,
                    stamp=now,
                    x_base=x_base,
                    y_base=y_base,
                )
            )

            self._draw_axis(annotated, corners[index], tvec)
            self.get_logger().info(
                f'ArUco #{marker_id} range={range_m:.2f}m '
                f'bearing={bearing_rad:.2f}rad'
            )

        self._viz_pub.publish(viz_markers)
        self._publish_annotated_image(annotated, header)

    def _make_viz_marker(self, marker_id, stamp, x_base, y_base):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = 'base_link'
        marker.ns = 'aruco_detections'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x_base)
        marker.pose.position.y = float(y_base)
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.12
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.9
        marker.lifetime.sec = 1
        return marker

    def _draw_axis(self, image, corners, tvec):
        if image is None or not hasattr(cv2, 'drawFrameAxes'):
            return

        object_points = np.array(
            [
                [-self._marker_size / 2.0, self._marker_size / 2.0, 0.0],
                [self._marker_size / 2.0, self._marker_size / 2.0, 0.0],
                [self._marker_size / 2.0, -self._marker_size / 2.0, 0.0],
                [-self._marker_size / 2.0, -self._marker_size / 2.0, 0.0],
            ],
            dtype=np.float32,
        )
        ok, rvec, _ = cv2.solvePnP(
            object_points,
            corners.reshape(4, 2).astype(np.float32),
            self._camera_matrix,
            self._dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if ok:
            cv2.drawFrameAxes(
                image,
                self._camera_matrix,
                self._dist_coeffs,
                rvec,
                tvec.reshape(3, 1),
                self._marker_size * 0.5,
            )

    def _publish_annotated_image(self, image, header):
        if self._annotated_pub is None or image is None:
            return

        try:
            msg = self._bridge.cv2_to_imgmsg(image, encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'annotated image bridge error: {exc}')
            return

        msg.header = header
        self._annotated_pub.publish(msg)


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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
