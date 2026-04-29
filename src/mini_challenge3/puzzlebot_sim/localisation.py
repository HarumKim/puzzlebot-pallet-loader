import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import numpy as np
import transforms3d
import signal


class DeadReckoning(Node):
    def __init__(self):
        super().__init__('localisation_node')

        # Parámetros físicos del Puzzlebot
        self.wheel_radius = 0.05   # metros
        self.wheel_base = 0.19     # metros

        # Tiempo de muestreo
        self.dt = 0.01

        # Estado estimado por odometría
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Velocidades de ruedas
        self.wr = 0.0
        self.wl = 0.0

        # Subscribers
        self.wr_sub = self.create_subscription(
            Float32,
            'wr',
            self.wr_callback,
            10
        )

        self.wl_sub = self.create_subscription(
            Float32,
            'wl',
            self.wl_callback,
            10
        )

        # Publisher de odometría
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # Timer
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info('Dead reckoning localisation node started.')

    def wr_callback(self, msg):
        self.wr = msg.data

    def wl_callback(self, msg):
        self.wl = msg.data

    def timer_callback(self):
        # Cinemática directa: de velocidades de rueda a velocidad del robot
        v = (self.wheel_radius / 2.0) * (self.wr + self.wl)
        w = (self.wheel_radius / self.wheel_base) * (self.wr - self.wl)

        # Integración numérica por Euler
        self.x += v * np.cos(self.theta) * self.dt
        self.y += v * np.sin(self.theta) * self.dt
        self.theta += w * self.dt

        # Normalizar theta
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        # Crear mensaje Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        odom_msg.pose.pose.position.x = float(self.x)
        odom_msg.pose.pose.position.y = float(self.y)
        odom_msg.pose.pose.position.z = 0.0

        quat = transforms3d.euler.euler2quat(0.0, 0.0, self.theta)

        odom_msg.pose.pose.orientation.w = float(quat[0])
        odom_msg.pose.pose.orientation.x = float(quat[1])
        odom_msg.pose.pose.orientation.y = float(quat[2])
        odom_msg.pose.pose.orientation.z = float(quat[3])

        odom_msg.twist.twist.linear.x = float(v)
        odom_msg.twist.twist.angular.z = float(w)

        self.odom_pub.publish(odom_msg)


def main(args=None):

    rclpy.init(args=args)

    node = DeadReckoning()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()