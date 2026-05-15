import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from std_msgs.msg import Float32
import numpy as np
import math
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
#import transforms3d

def wrap_to_pi(angle):
    """Keep angle within [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle)) #para evitar singularidades

def yaw_to_quaternion(yaw):
    """Convert a planar yaw angle into a ROS quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class KinematicModelNode(Node):
    def __init__(self):
        super().__init__('puzzlebot_kinematic_model')

        # Robot pose in the odom/world frame
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Pose covariance matrix for [x, y, theta]
        self.pose_covariance = np.zeros((3, 3), dtype=float)

        self.debug_counter = 0

        # Puzzlebot physical parameters
        self.declare_parameter('wheel_base', 0.19)      # meters
        self.declare_parameter('wheel_radius', 0.05)
        self.parent_frame = 'odom'
        self.child_frame = 'base_footprint'

        # Encoder noise gains
        self.declare_parameter('noise_gain_right', 0.016)
        self.declare_parameter('noise_gain_left', 0.016)

        # Motion model covariance gains
        self.declare_parameter('k_linear', 0.08)
        self.declare_parameter('k_angular', 0.08)
        self.declare_parameter('k_drift', 0.04)

        # Simulation sample time
        self.declare_parameter('sample_time', 0.01)

        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        self.noise_gain_right = self.get_parameter('noise_gain_right').value
        self.noise_gain_left = self.get_parameter('noise_gain_left').value

        self.k_linear = self.get_parameter('k_linear').value
        self.k_angular = self.get_parameter('k_angular').value
        self.k_drift = self.get_parameter('k_drift').value

        self.sample_time = self.get_parameter('sample_time').value

        # Velocity inputs
        self.v = 0.0  # Linear velocity (m/s)
        self.omega = 0.0  # Angular velocity (rad/s)

        # Wheel speed messages
        self.right_wheel_msg = Float32()
        self.left_wheel_msg = Float32()

        # Last update time
        self.last_time = self.get_clock().now()

        # ROS2 Subscribers and Publishers
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.wl_pub = self.create_publisher(Float32, 'wl', 10)
        self.wr_pub = self.create_publisher(Float32, 'wr', 10)
        self.pose_pub = self.create_publisher(Odometry, 'pose_sim', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to update kinematics at ~100Hz
        self.timer = self.create_timer(self.sample_time, self.update_kinematics)
        self.get_logger().info("Kinematic Model Node Started.")

    def cmd_vel_callback(self, msg):
        """ Callback to update velocity commands """
        self.v = msg.linear.x
        self.omega = msg.angular.z

    def update_kinematics(self):
        """ Updates robot position based on real elapsed time """
        # Get current time and compute dt
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds
        self.last_time = current_time

        if dt > 0:
            ideal_wr = (self.v + self.wheel_base * self.omega / 2.0) / self.wheel_radius
            ideal_wl = (self.v - self.wheel_base * self.omega / 2.0) / self.wheel_radius

            noisy_wr = ideal_wr + np.random.normal(
                0.0,
                self.noise_gain_right * abs(ideal_wr)
            )

            noisy_wl = ideal_wl + np.random.normal(
                0.0,
                self.noise_gain_left * abs(ideal_wl)
            )

            self.right_wheel_msg.data = float(noisy_wr)
            self.left_wheel_msg.data = float(noisy_wl)

            # Reconstruct noisy robot velocities from noisy wheel speeds
            noisy_v = self.wheel_radius * (noisy_wr + noisy_wl) / 2.0
            noisy_omega = self.wheel_radius * (noisy_wr - noisy_wl) / self.wheel_base

            # Numerical integration using Euler method
            self.x += noisy_v * math.cos(self.theta) * dt
            self.y += noisy_v * math.sin(self.theta) * dt
            self.theta += noisy_omega * dt
            self.theta = wrap_to_pi(self.theta)

            # Uncertainty propagation
            self.update_pose_covariance(noisy_v, noisy_omega, dt)

            self.debug_counter += 1

            if self.debug_counter % 100 == 0:
                self.get_logger().info(
                    f"pose -> x: {self.x:.3f}, y: {self.y:.3f}, theta: {self.theta:.3f}"
                )

            # Publish new state
            self.publish_wheel_speed()
            self.publish_odometry(noisy_v, noisy_omega)
            self.publish_tf()

    def publish_wheel_speed(self):
        self.wl_pub.publish(self.left_wheel_msg)
        self.wr_pub.publish(self.right_wheel_msg)

    def publish_odometry(self, linear_velocity, angular_velocity):
        """Publish the simulated robot pose as a nav_msgs/Odometry message."""
        odom_msg = Odometry()

        current_time = self.get_clock().now().to_msg()

        # Header
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = self.parent_frame
        odom_msg.child_frame_id = self.child_frame

        # Pose: position
        odom_msg.pose.pose.position.x = float(self.x)
        odom_msg.pose.pose.position.y = float(self.y)
        odom_msg.pose.pose.position.z = 0.0

        # Pose: orientation
        odom_msg.pose.pose.orientation = yaw_to_quaternion(self.theta)

        # Twist: velocity in the robot frame
        odom_msg.twist.twist.linear.x = float(linear_velocity)
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0

        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = float(angular_velocity)

        # Insert 3x3 covariance [x, y, theta] into ROS 6x6 pose covariance
        odom_msg.pose.covariance = [0.0] * 36

        odom_msg.pose.covariance[0] = float(self.pose_covariance[0, 0])    # x-x
        odom_msg.pose.covariance[1] = float(self.pose_covariance[0, 1])    # x-y
        odom_msg.pose.covariance[5] = float(self.pose_covariance[0, 2])    # x-yaw

        odom_msg.pose.covariance[6] = float(self.pose_covariance[1, 0])    # y-x
        odom_msg.pose.covariance[7] = float(self.pose_covariance[1, 1])    # y-y
        odom_msg.pose.covariance[11] = float(self.pose_covariance[1, 2])   # y-yaw

        odom_msg.pose.covariance[30] = float(self.pose_covariance[2, 0])   # yaw-x
        odom_msg.pose.covariance[31] = float(self.pose_covariance[2, 1])   # yaw-y
        odom_msg.pose.covariance[35] = float(self.pose_covariance[2, 2])   # yaw-yaw

        # For this challenge, velocity covariance is kept as zero
        odom_msg.twist.covariance = [0.0] * 36

        self.pose_pub.publish(odom_msg)

    def publish_tf(self):
        """Publish the transform from odom to base_link."""
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame

        transform.transform.translation.x = float(self.x)
        transform.transform.translation.y = float(self.y)
        transform.transform.translation.z = 0.0

        transform.transform.rotation = yaw_to_quaternion(self.theta)

        self.tf_broadcaster.sendTransform(transform)

    def update_pose_covariance(self, linear_velocity, angular_velocity, dt):
        """Propagate the 3x3 pose covariance matrix using the linearized motion model."""
        theta = self.theta

        # Linearized model Jacobian H = dh/ds
        H = np.array([
            [1.0, 0.0, -linear_velocity * math.sin(theta) * dt],
            [0.0, 1.0,  linear_velocity * math.cos(theta) * dt],
            [0.0, 0.0,  1.0]
        ], dtype=float)

        # Motion noise approximation Q
        q_x = self.k_linear * abs(linear_velocity) * dt
        q_y = self.k_drift * abs(linear_velocity) * dt
        q_theta = self.k_angular * abs(angular_velocity) * dt

        Q = np.array([
            [q_x,     0.0,     0.0],
            [0.0,     q_y,     0.0],
            [0.0,     0.0,     q_theta]
        ], dtype=float)

        self.pose_covariance = H @ self.pose_covariance @ H.T + Q

def main(args=None):

    rclpy.init(args=args)

    node = KinematicModelNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # Ensure shutdown is only called once
            rclpy.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()