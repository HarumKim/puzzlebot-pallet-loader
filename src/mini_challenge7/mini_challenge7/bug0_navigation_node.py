import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


DEFAULT_WAYPOINTS = [0.0, 0.0, 2.0, 0.0, 2.0, 2.0, 0.0, 2.0, 0.0, 0.0]


class Bug0NavigationNode(Node):
    def __init__(self):
        super().__init__('bug0_navigation_node')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/ekf_odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('waypoints', DEFAULT_WAYPOINTS)
        self.declare_parameter('goal_tolerance', 0.18)
        self.declare_parameter('max_linear_speed', 0.22)
        self.declare_parameter('max_angular_speed', 0.70)
        self.declare_parameter('obstacle_threshold', 0.45)
        self.declare_parameter('obstacle_exit_threshold', 0.75)
        self.declare_parameter('wall_distance', 0.45)
        self.declare_parameter('wall_follow_min_distance', 0.45)
        self.declare_parameter('front_half_angle_deg', 28.0)
        self.declare_parameter('heading_tolerance_deg', 45.0)
        self.declare_parameter('wait_for_localization', True)
        self.declare_parameter('yaw_covariance_threshold', 0.10)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        scan_topic = self.get_parameter('scan_topic').value

        flat = list(self.get_parameter('waypoints').value)
        self.waypoints = [(float(flat[i]), float(flat[i + 1])) for i in range(0, len(flat) - 1, 2)]

        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.max_linear = float(self.get_parameter('max_linear_speed').value)
        self.max_angular = float(self.get_parameter('max_angular_speed').value)
        self.obstacle_threshold = float(self.get_parameter('obstacle_threshold').value)
        self.obstacle_exit_threshold = float(self.get_parameter('obstacle_exit_threshold').value)
        self.wall_distance = float(self.get_parameter('wall_distance').value)
        self.wall_follow_min_distance = float(self.get_parameter('wall_follow_min_distance').value)
        self.front_half_angle = float(self.get_parameter('front_half_angle_deg').value)
        self.heading_tolerance = math.radians(float(self.get_parameter('heading_tolerance_deg').value))
        self.wait_for_localization = bool(self.get_parameter('wait_for_localization').value)
        self.yaw_covariance_threshold = float(self.get_parameter('yaw_covariance_threshold').value)

        self.pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(LaserScan, scan_topic, self._scan_cb, 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.yaw_covariance = float('inf')
        self.odom_ready = False
        self.scan = None

        self.wp_idx = 0
        self.lap_count = 0
        self.state = 'GO_TO_GOAL'
        self.wall_side = 'right'
        self.wall_entry_pos = None
        self.corner_entry_pos = None
        self.localization_ready = False

        self.create_timer(0.05, self._loop)
        self.get_logger().info(
            f'Bug0 navigation started. Waypoints={self.waypoints}, '
            f'odom={odom_topic}, scan={scan_topic}'
        )

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x = p.x
        self.y = p.y
        self.yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)
        self.yaw_covariance = msg.pose.covariance[35]
        self.odom_ready = True

    def _scan_cb(self, msg: LaserScan):
        self.scan = msg

    def _loop(self):
        if not self.odom_ready or self.scan is None:
            return

        if self.wait_for_localization and not self.localization_ready:
            if self.yaw_covariance < self.yaw_covariance_threshold:
                self.localization_ready = True
                self.get_logger().info('Localization covariance is low enough. Starting Bug0.')
            else:
                self._cmd(0.0, 0.0)
                return

        if self.wp_idx >= len(self.waypoints):
            self.wp_idx = 0
            self.lap_count += 1
            self.get_logger().info(f'Lap {self.lap_count} complete. Restarting circuit.')

        gx, gy = self.waypoints[self.wp_idx]
        dist_to_goal = math.hypot(gx - self.x, gy - self.y)

        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info(f'Waypoint {self.wp_idx} reached at ({gx:.2f}, {gy:.2f}).')
            self.wp_idx += 1
            self.state = 'GO_TO_GOAL'
            self.wall_entry_pos = None
            self.corner_entry_pos = None
            self._cmd(0.0, 0.0)
            return

        front = self._min_range(-self.front_half_angle, self.front_half_angle)

        if self.state == 'GO_TO_GOAL':
            if front < self.obstacle_threshold:
                self.state = 'BUG0_AVOID'
                self.wall_entry_pos = (self.x, self.y)
                self.corner_entry_pos = None
                self.wall_side = self._choose_wall_side(gx, gy)
                self.get_logger().warn(
                    f'Obstacle detected before waypoint {self.wp_idx}; '
                    f'following {self.wall_side} wall.'
                )
                return

            self._go_to_goal(gx, gy, dist_to_goal)
            return

        if self._can_leave_wall(gx, gy, front):
            self.get_logger().info('Goal direction clear. Leaving Bug0 wall following.')
            self.state = 'GO_TO_GOAL'
            self.wall_entry_pos = None
            self.corner_entry_pos = None
            return

        self._follow_wall()

    def _go_to_goal(self, gx: float, gy: float, dist: float):
        goal_angle = math.atan2(gy - self.y, gx - self.x)
        heading_error = _wrap(goal_angle - self.yaw)

        angular = _clamp(2.0 * heading_error, -self.max_angular, self.max_angular)
        linear = _clamp(0.55 * dist, 0.0, self.max_linear)

        if abs(heading_error) > math.radians(45.0):
            linear *= 0.25

        self._cmd(linear, angular)

    def _choose_wall_side(self, gx: float, gy: float):
        left = self._finite_range_or_max(self._min_range(30, 120))
        right = self._finite_range_or_max(self._min_range(-120, -30))

        goal_angle = math.atan2(gy - self.y, gx - self.x)
        heading_error = _wrap(goal_angle - self.yaw)

        # 1. Si un lado está demasiado bloqueado, huimos hacia el lado libre
        if left < 0.5 and right > 0.5:
            return 'left'  # Bloqueo izquierdo -> giramos derecha -> seguimos pared izquierda
        elif right < 0.5 and left > 0.5:
            return 'right' # Bloqueo derecho -> giramos izquierda -> seguimos pared derecha

        # 2. Si ambos están libres (o bloqueados igual), priorizamos la dirección de la meta
        if heading_error > 0:  # La meta está a nuestra izquierda
            return 'right'     # Giramos a la izquierda, por ende seguimos la pared derecha
        else:                  # La meta está a nuestra derecha
            return 'left'      # Giramos a la derecha, por ende seguimos la pared izquierda

    def _follow_wall(self):
        front_wide = self._min_range(-45, 45)

        if self.wall_side == 'left':
            side_range = self._min_range(55, 135)
            front_side = self._min_range(25, 65)
            turn_away = -self.max_angular
            turn_toward = self.max_angular
            correction_sign = 1.0
        else:
            side_range = self._min_range(-135, -55)
            front_side = self._min_range(-65, -25)
            turn_away = self.max_angular
            turn_toward = -self.max_angular
            correction_sign = -1.0

        side_range = self._finite_range_or_max(side_range)
        front_side = self._finite_range_or_max(front_side)
        outer_corner = side_range > self.wall_distance * 2.4

        if outer_corner:
            if self.corner_entry_pos is None:
                self.corner_entry_pos = (self.x, self.y)
            corner_distance = math.hypot(
                self.x - self.corner_entry_pos[0],
                self.y - self.corner_entry_pos[1],
            )
            corner_cleared = corner_distance > 0.45
        else:
            self.corner_entry_pos = None
            corner_cleared = True

        if front_wide < self.obstacle_threshold:
            self._cmd(0.0, turn_away)
        elif not outer_corner and front_side < self.obstacle_threshold:
            self._cmd(self.max_linear * 0.45, turn_away * 0.8)
        elif outer_corner:
            if not corner_cleared:
                self._cmd(self.max_linear * 0.55, 0.0)
            else:
                self._cmd(self.max_linear * 0.45, turn_toward * 0.45)
        else:
            error = side_range - self.wall_distance
            angular = _clamp(correction_sign * 1.6 * error, -self.max_angular, self.max_angular)
            self._cmd(self.max_linear * 0.75, angular)

    def _can_leave_wall(self, gx: float, gy: float, front: float):
        if self.wall_entry_pos is None:
            return False

        wall_traveled = math.hypot(
            self.x - self.wall_entry_pos[0],
            self.y - self.wall_entry_pos[1],
        )
        goal_angle = math.atan2(gy - self.y, gx - self.x)
        heading_error = abs(_wrap(goal_angle - self.yaw))

        return (
            front > self.obstacle_exit_threshold
            and heading_error < self.heading_tolerance
            and wall_traveled > self.wall_follow_min_distance
        )

    def _min_range(self, min_deg: float, max_deg: float):
        if self.scan is None:
            return float('inf')
            
        scan = self.scan
        a_min = _wrap(math.radians(min_deg))
        a_max = _wrap(math.radians(max_deg))

        vals = []
        for i, r in enumerate(scan.ranges):
            if not (math.isfinite(r) and scan.range_min < r < scan.range_max):
                continue

            angle_i_wrapped = _wrap(scan.angle_min + i * scan.angle_increment)

            if a_min <= a_max:
                if a_min <= angle_i_wrapped <= a_max:
                    vals.append(r)
            else:
                if angle_i_wrapped >= a_min or angle_i_wrapped <= a_max:
                    vals.append(r)

        return min(vals) if vals else float('inf')

    def _finite_range_or_max(self, value: float):
        if math.isfinite(value):
            return value
        return self.scan.range_max if self.scan is not None else 10.0

    def _cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub.publish(msg)


def _wrap(angle: float):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _clamp(value: float, lo: float, hi: float):
    return max(lo, min(hi, value))


def _quat_to_yaw(x: float, y: float, z: float, w: float):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main(args=None):
    rclpy.init(args=args)
    node = Bug0NavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._cmd(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
