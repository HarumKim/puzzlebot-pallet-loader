import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import numpy as np
import transforms3d


class PointStabilisationNode(Node):
    def __init__(self):
        super().__init__('point_stabilisation_control')

        # Punto objetivo
        self.goal_x = 1.0
        self.goal_y = 1.0
        self.goal_theta = 0.0

        # Ganancias del controlador
        self.k_rho = 0.6      # Control de distancia
        self.k_alpha = 1.5    # Control de orientación hacia el objetivo
        self.k_beta = -0.2    # Control de orientación final

        # Límites de velocidad
        self.max_linear_speed = 0.15
        self.max_angular_speed = 1.0

        # Tolerancias para considerar que llegó al objetivo
        self.distance_tolerance = 0.03
        self.angle_tolerance = 0.08

        # Estado actual del robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Bandera para evitar seguir mandando comandos después de llegar
        self.goal_reached = False

        # Publisher de velocidad
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Publisher para avisar que llegó al objetivo
        self.goal_pub = self.create_publisher(Float32, 'goal_reached', 10)

        # Subscriber a la odometría
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.get_logger().info('Point stabilisation controller started.')
        self.get_logger().info(
            f'Goal: x={self.goal_x:.2f}, y={self.goal_y:.2f}, theta={self.goal_theta:.2f}'
        )

    def odom_callback(self, msg):
        # Leer posición actual
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Leer orientación actual como quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Convertir quaternion a ángulos de Euler
        roll, pitch, yaw = transforms3d.euler.quat2euler([qw, qx, qy, qz])
        self.theta = yaw

        # Ejecutar el controlador cada vez que llegue una nueva odometría
        self.control()

    def normalize_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

    def control(self):
        cmd_msg = Twist()

        # Error de posición
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y

        # Distancia al objetivo
        rho = np.sqrt(dx**2 + dy**2)

        # Ángulo deseado hacia el objetivo
        desired_angle = np.arctan2(dy, dx)

        # Error angular hacia el objetivo
        alpha = self.normalize_angle(desired_angle - self.theta)

        # Error de orientación final
        beta = self.normalize_angle(self.goal_theta - desired_angle)

        # Si ya llegó al objetivo, detenerse
        if rho < self.distance_tolerance:
            final_angle_error = self.normalize_angle(self.goal_theta - self.theta)

            if abs(final_angle_error) < self.angle_tolerance:
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.0
                self.cmd_pub.publish(cmd_msg)

                if not self.goal_reached:
                    self.goal_reached = True
                    reached_msg = Float32()
                    reached_msg.data = 1.0
                    self.goal_pub.publish(reached_msg)
                    self.get_logger().info('Goal reached.')

                return

            # Si ya llegó en posición, solo corrige orientación final
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = self.k_alpha * final_angle_error
            cmd_msg.angular.z = float(
                np.clip(cmd_msg.angular.z, -self.max_angular_speed, self.max_angular_speed)
            )

            self.cmd_pub.publish(cmd_msg)
            return

        # Ley de control para estabilización a punto
        v = self.k_rho * rho
        w = self.k_alpha * alpha + self.k_beta * beta

        # Limitar velocidades
        v = np.clip(v, -self.max_linear_speed, self.max_linear_speed)
        w = np.clip(w, -self.max_angular_speed, self.max_angular_speed)

        # Si el error angular es muy grande, primero gira antes de avanzar
        if abs(alpha) > 1.0:
            v = 0.0

        cmd_msg.linear.x = float(v)
        cmd_msg.angular.z = float(w)

        self.cmd_pub.publish(cmd_msg)


def main(args=None):

    rclpy.init(args=args)

    node = PointStabilisationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)

        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()