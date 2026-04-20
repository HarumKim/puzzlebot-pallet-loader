import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class PuzzlebotMover(Node):
    def __init__(self):
        super().__init__('puzzlebot_mover')     

        # Dedicated broadcaster to constantly update robot's position
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.wheel_radius = 0.05
        self.wheel_base = 0.19
        
        self.time_elapsed = 0.0
        self.omega_robot = 0.1   # rad/s (Velocidad reducida para ver mejor el giro)
        self.radius = 1.0        # meters
        self.circle_radius = 1.0 # meters
        
        self.theta_r = 0.0
        self.theta_l = 0.0

    # Function that runs every 0.01 s
    def timer_callback(self):
        self.time_elapsed += self.dt

        # Calculate where the center of the robot is on a circle
        robot_yaw = self.omega_robot * self.time_elapsed
        x = self.radius * math.cos(robot_yaw)
        y = self.radius * math.sin(robot_yaw)

        # Calculate individual wheel speeds
        v_robot = self.omega_robot * self.circle_radius
        v_r = v_robot + (self.omega_robot * self.wheel_base / 2.0)
        v_l = v_robot - (self.omega_robot * self.wheel_base / 2.0)

        # Integrate angular velocities to get wheel positions
        self.theta_r += (v_r / self.wheel_radius) * self.dt
        self.theta_l += (v_l / self.wheel_radius) * self.dt

        # Capturar el tiempo una sola vez para sincronizar base y llantas
        current_time = self.get_clock().now().to_msg()

        # Pack the Transform (Base Movement)
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = x
        t.transform.translation.y = y


        heading = robot_yaw + (math.pi / 2.0)

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(heading / 2.0)
        t.transform.rotation.w = math.cos(heading / 2.0)
        self.tf_broadcaster.sendTransform(t)
        
        # Pack the Joint States (Wheel Spins)
        js = JointState()
        js.header.stamp = current_time
        js.name = ['wheel_r_joint', 'wheel_l_joint']
        js.position = [self.theta_r, self.theta_l]
        self.joint_pub.publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()