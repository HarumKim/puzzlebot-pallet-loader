#!/usr/bin/env python3
"""
odometry_node.py

Clean odometry node for Puzzlebot.

Features:
- Pure dead reckoning from wheel encoder angular velocities.
- Optional Kalman correction layer controlled by parameter `activate_Kalman`.
- Architecture ready for future absolute pose corrections such as:
  - ArUco pose
  - MCL pose
  - ground_truth in simulation
  - external localization node

Subscriptions:
- /VelocityEncL   std_msgs/Float32
- /VelocityEncR   std_msgs/Float32
- optional correction topic, default /ground_truth, nav_msgs/Odometry

Publications:
- /odom           nav_msgs/Odometry
- TF odom -> base_link
- Static TF base_link -> laser
"""

import math
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert yaw angle to geometry_msgs/Quaternion."""
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )


def quaternion_to_yaw(q: Quaternion) -> float:
    """Convert geometry_msgs/Quaternion to yaw angle."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class OdometryNode(Node):
    def __init__(self):
        super().__init__("odometry_node")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("wheel_separation", 0.19)
        self.declare_parameter("dt", 0.05)

        self.declare_parameter("left_encoder_topic", "/VelocityEncL")
        self.declare_parameter("right_encoder_topic", "/VelocityEncR")
        self.declare_parameter("odom_topic", "/wheel_odom")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("laser_frame", "laser")
        self.declare_parameter("laser_parent_frame", "")

        self.declare_parameter("activate_Kalman", False)
        self.declare_parameter("kalman_correction_topic", "/ground_truth")

        # Process noise: encoder/motion uncertainty.
        self.declare_parameter("kalman_q_x", 0.002)
        self.declare_parameter("kalman_q_y", 0.002)
        self.declare_parameter("kalman_q_yaw", 0.001)

        # Measurement noise: external correction uncertainty.
        self.declare_parameter("kalman_r_x", 0.02)
        self.declare_parameter("kalman_r_y", 0.02)
        self.declare_parameter("kalman_r_yaw", 0.01)

        self.r = float(self.get_parameter("wheel_radius").value)
        self.L = float(self.get_parameter("wheel_separation").value)
        self.dt = float(self.get_parameter("dt").value)

        self.left_encoder_topic = self.get_parameter("left_encoder_topic").value
        self.right_encoder_topic = self.get_parameter("right_encoder_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value

        self.base_frame = self.get_parameter("base_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.laser_frame = self.get_parameter("laser_frame").value
        self.laser_parent_frame = (
            self.get_parameter("laser_parent_frame").value or self.base_frame
        )

        self.activate_Kalman = bool(self.get_parameter("activate_Kalman").value)
        self.kalman_correction_topic = self.get_parameter("kalman_correction_topic").value

        # ------------------------------------------------------------------
        # Robot state
        # ------------------------------------------------------------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.w_l = 0.0
        self.w_r = 0.0

        # Kalman state: [x, y, theta]
        self.state = np.array([0.0, 0.0, 0.0], dtype=float)

        self.P = np.diag([0.01, 0.01, 0.005]).astype(float)
        self.Q = np.diag([
            float(self.get_parameter("kalman_q_x").value),
            float(self.get_parameter("kalman_q_y").value),
            float(self.get_parameter("kalman_q_yaw").value),
        ]).astype(float)
        self.R = np.diag([
            float(self.get_parameter("kalman_r_x").value),
            float(self.get_parameter("kalman_r_y").value),
            float(self.get_parameter("kalman_r_yaw").value),
        ]).astype(float)

        self.latest_correction: Optional[np.ndarray] = None

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------
        qos_enc = QoSProfile(depth=10)
        qos_enc.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.create_subscription(
            Float32,
            self.left_encoder_topic,
            self.left_encoder_cb,
            qos_enc,
        )

        self.create_subscription(
            Float32,
            self.right_encoder_topic,
            self.right_encoder_cb,
            qos_enc,
        )

        if self.activate_Kalman:
            self.create_subscription(
                Odometry,
                self.kalman_correction_topic,
                self.correction_cb,
                10,
            )

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)

        self._publish_laser_tf()
        self.create_timer(self.dt, self.integrate_and_publish)

        mode = "Kalman-corrected odometry" if self.activate_Kalman else "pure dead reckoning"
        self.get_logger().info(f"Odometry node started — {mode}")
        self.get_logger().info(
            f"Encoders: left={self.left_encoder_topic}, right={self.right_encoder_topic}"
        )
        self.get_logger().info(f"Publishing odometry on: {self.odom_topic}")

        if self.activate_Kalman:
            self.get_logger().info(
                f"Kalman correction topic: {self.kalman_correction_topic}"
            )

    # ------------------------------------------------------------------
    # Encoder callbacks
    # ------------------------------------------------------------------
    def left_encoder_cb(self, msg: Float32):
        self.w_l = float(msg.data)

    def right_encoder_cb(self, msg: Float32):
        self.w_r = float(msg.data)

    # ------------------------------------------------------------------
    # External correction callback
    # ------------------------------------------------------------------
    def correction_cb(self, msg: Odometry):
        """
        Receives an absolute or higher-confidence pose estimate.

        In simulation this can be /ground_truth.
        In the real robot this can later be replaced by ArUco/MCL/fused pose.
        """
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.latest_correction = np.array(
            [
                float(p.x),
                float(p.y),
                normalize_angle(quaternion_to_yaw(q)),
            ],
            dtype=float,
        )

    # ------------------------------------------------------------------
    # Main integration loop
    # ------------------------------------------------------------------
    def integrate_and_publish(self):
        v = self.r * (self.w_r + self.w_l) / 2.0
        w = self.r * (self.w_r - self.w_l) / self.L

        if self.activate_Kalman:
            self._kalman_predict(v, w)
            self._kalman_correct_if_available()
            self._sync_pose_from_kalman_state()
        else:
            self._dead_reckoning_step(v, w)

        self._publish_odom_and_tf(v, w)

    def _dead_reckoning_step(self, v: float, w: float):
        self.x += v * self.dt * math.cos(self.theta)
        self.y += v * self.dt * math.sin(self.theta)
        self.theta = normalize_angle(self.theta + w * self.dt)

    # ------------------------------------------------------------------
    # Kalman filter
    # ------------------------------------------------------------------
    def _kalman_predict(self, v: float, w: float):
        x, y, theta = self.state

        self.state[0] = x + v * self.dt * math.cos(theta)
        self.state[1] = y + v * self.dt * math.sin(theta)
        self.state[2] = normalize_angle(theta + w * self.dt)

        F = np.array(
            [
                [1.0, 0.0, -v * self.dt * math.sin(theta)],
                [0.0, 1.0,  v * self.dt * math.cos(theta)],
                [0.0, 0.0,  1.0],
            ],
            dtype=float,
        )

        self.P = F @ self.P @ F.T + self.Q

    def _kalman_correct_if_available(self):
        if self.latest_correction is None:
            return

        z = self.latest_correction
        H = np.eye(3)

        innovation = z - self.state
        innovation[2] = normalize_angle(innovation[2])

        S = H @ self.P @ H.T + self.R

        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.latest_correction = None
            return

        self.state = self.state + K @ innovation
        self.state[2] = normalize_angle(self.state[2])

        I = np.eye(3)
        self.P = (I - K @ H) @ self.P

        self.latest_correction = None

    def _sync_pose_from_kalman_state(self):
        self.x = float(self.state[0])
        self.y = float(self.state[1])
        self.theta = float(self.state[2])

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------
    def _publish_odom_and_tf(self, v: float, w: float):
        now = self.get_clock().now().to_msg()
        q = yaw_to_quaternion(self.theta)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        if self.activate_Kalman:
            odom.pose.covariance[0] = float(self.P[0, 0])
            odom.pose.covariance[1] = float(self.P[0, 1])
            odom.pose.covariance[6] = float(self.P[1, 0])
            odom.pose.covariance[7] = float(self.P[1, 1])
            odom.pose.covariance[35] = float(self.P[2, 2])
        else:
            odom.pose.covariance[0] = 0.01
            odom.pose.covariance[7] = 0.01
            odom.pose.covariance[35] = 0.005

        self.odom_pub.publish(odom)

        tf_odom = TransformStamped()
        tf_odom.header.stamp = now
        tf_odom.header.frame_id = self.odom_frame
        tf_odom.child_frame_id = self.base_frame

        tf_odom.transform.translation.x = self.x
        tf_odom.transform.translation.y = self.y
        tf_odom.transform.translation.z = 0.0
        tf_odom.transform.rotation = q

        self.tf_br.sendTransform(tf_odom)

    def _publish_laser_tf(self):
        tf_static = TransformStamped()
        tf_static.header.stamp = self.get_clock().now().to_msg()
        tf_static.header.frame_id = self.laser_parent_frame
        tf_static.child_frame_id = self.laser_frame

        tf_static.transform.translation.x = 0.07
        tf_static.transform.translation.y = 0.0
        tf_static.transform.translation.z = 0.205

        yaw = math.pi
        tf_static.transform.rotation = yaw_to_quaternion(yaw)

        self.static_br.sendTransform(tf_static)
        self.get_logger().info(
            f"Static TF published: {self.laser_parent_frame} -> {self.laser_frame} (yaw=180 deg)"
        )


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("Odometry node stopped by user.")

    except RuntimeError as e:
        print(f"Odometry node shutting down: {e}")

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


if __name__ == "__main__":
    main()
