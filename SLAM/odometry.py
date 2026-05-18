"""
Odometry Node — Dead reckoning con encoders de ruedas

Suscribe:
  /VelocityEncL (Float32) — velocidad angular rueda izquierda
  /VelocityEncR (Float32) — velocidad angular rueda derecha

Publica:
  /odom (nav_msgs/Odometry)
  TF: odom → base_link
"""

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

from rclpy.qos import QoSProfile, QoSReliabilityPolicy


def yaw_to_quaternion(yaw):
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0)
    )


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry')

        # Parámetros del robot
        self.r = 0.05   # radio rueda
        self.L = 0.19   # distancia entre ruedas
        self.dt = 0.05  # tiempo de integración

        # Estados
        self.w_l = 0.0
        self.w_r = 0.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # QoS para encoders
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # Subscripciones
        self.create_subscription(Float32, '/VelocityEncL', self.left_callback, qos)
        self.create_subscription(Float32, '/VelocityEncR', self.right_callback, qos)

        # Publicadores
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer
        self.create_timer(self.dt, self.update)

        self.get_logger().info('📍 Odometry node running (encoders → /odom + TF)')

    def left_callback(self, msg):
        self.w_l = float(msg.data)

    def right_callback(self, msg):
        self.w_r = float(msg.data)

    def update(self):
        # Cinemática diferencial
        v = self.r * (self.w_r + self.w_l) / 2.0
        w = self.r * (self.w_r - self.w_l) / self.L

        # Integración
        self.x += v * math.cos(self.yaw) * self.dt
        self.y += v * math.sin(self.yaw) * self.dt
        self.yaw = normalize_angle(self.yaw + w * self.dt)

        # Tiempo
        now = self.get_clock().now().to_msg()

        # Cuaternión
        q = yaw_to_quaternion(self.yaw)

        # Mensaje /odom
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = q

        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = w

        self.odom_pub.publish(odom_msg)

        # TF: odom → base_link
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'

        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = q

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
