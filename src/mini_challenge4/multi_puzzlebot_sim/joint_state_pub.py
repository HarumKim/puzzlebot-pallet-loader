import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import transforms3d
import numpy as np


class DronePublisher(Node):

    def __init__(self):
        super().__init__('puzzlebot_jointPub')

        # Evita conflictos TF entre robots
        self.declare_parameter('frame_prefix', 'robot1')
        self.frame_prefix = str(self.get_parameter('frame_prefix').value)
       
        # Transform broadcaster para publicar odom -> base_footprint
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publisher de joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Subscriber a la odometría calculada por localisation.py
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        # Ángulos acumulados de las ruedas
        self.right_wheel_pos = 0.0
        self.left_wheel_pos = 0.0

        # Parámetros físicos
        self.wheel_radius = 0.05

        # Tiempo anterior para integrar movimiento de ruedas
        self.last_time = self.get_clock().now()

        self.get_logger().info('Puzzlebot joint state publisher started.')

    def odom_callback(self, msg):
        current_time = self.get_clock().now()

        # Calcular dt real entre callbacks
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0.0:
            dt = 0.01

        # Leer posición desde /odom
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # Leer orientación desde /odom
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Publicar transformación odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = f'{self.frame_prefix}/base_footprint'

        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)

        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(t)

        # Leer velocidad lineal del robot desde /odom
        v = msg.twist.twist.linear.x

        # Convertir velocidad lineal a velocidad angular de rueda aproximada
        wheel_angular_velocity = v / self.wheel_radius

        # Integrar posición angular de las ruedas
        self.right_wheel_pos += wheel_angular_velocity * dt
        self.left_wheel_pos += wheel_angular_velocity * dt

        # Publicar joint_states
        joint_msg = JointState()
        joint_msg.header.stamp = current_time.to_msg()

        joint_msg.name = [
            'wheel_right_joint',
            'wheel_left_joint'
        ]

        joint_msg.position = [
            float(self.right_wheel_pos),
            float(self.left_wheel_pos)
        ]

        joint_msg.velocity = [
            float(wheel_angular_velocity),
            float(wheel_angular_velocity)
        ]

        self.joint_pub.publish(joint_msg)


def main(args=None):
    rclpy.init(args=args)

    node = DronePublisher()

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