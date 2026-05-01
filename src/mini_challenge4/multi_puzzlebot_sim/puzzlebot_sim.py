import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import numpy as np
import transforms3d


class KinematicModelNode(Node):
    def __init__(self):
        super().__init__('puzzlebot_sim')

        # Parámetros físicos del Puzzlebot
        self.wheel_radius = 0.05   # metros
        self.wheel_base = 0.19     # metros

        # Tiempo de muestreo
        self.dt = 0.01  # segundos

        # Estado inicial del robot
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)

        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.theta = float(self.get_parameter('initial_theta').value)

        # Entradas de velocidad
        self.v = 0.0
        self.w = 0.0

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'pose_sim', 10)
        self.wr_pub = self.create_publisher(Float32, 'wr', 10)
        self.wl_pub = self.create_publisher(Float32, 'wl', 10)

        # Subscriber
        self.cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Timer para actualizar la simulación
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info('Puzzlebot kinematic simulator started.')

    def cmd_vel_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def timer_callback(self):
        # Modelo cinemático diferencial/no holonómico
        self.x += self.v * np.cos(self.theta) * self.dt
        self.y += self.v * np.sin(self.theta) * self.dt
        self.theta += self.w * self.dt

        # Normalizar theta entre -pi y pi
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        # Cinemática inversa: de velocidad del robot a velocidad de ruedas
        wr = (self.v + (self.w * self.wheel_base / 2.0)) / self.wheel_radius
        wl = (self.v - (self.w * self.wheel_base / 2.0)) / self.wheel_radius

        # Publicar velocidades de ruedas
        wr_msg = Float32()
        wl_msg = Float32()

        wr_msg.data = float(wr)
        wl_msg.data = float(wl)

        self.wr_pub.publish(wr_msg)
        self.wl_pub.publish(wl_msg)

        # Publicar pose simulada como Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        odom_msg.pose.pose.position.x = float(self.x)
        odom_msg.pose.pose.position.y = float(self.y)
        odom_msg.pose.pose.position.z = 0.0

        # Quaternion desde yaw
        quat = transforms3d.euler.euler2quat(0.0, 0.0, self.theta)
        odom_msg.pose.pose.orientation.w = float(quat[0])
        odom_msg.pose.pose.orientation.x = float(quat[1])
        odom_msg.pose.pose.orientation.y = float(quat[2])
        odom_msg.pose.pose.orientation.z = float(quat[3])

        odom_msg.twist.twist.linear.x = float(self.v)
        odom_msg.twist.twist.angular.z = float(self.w)

        self.odom_pub.publish(odom_msg)


def main(args=None):

    rclpy.init(args=args)

    node = KinematicModelNode()

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