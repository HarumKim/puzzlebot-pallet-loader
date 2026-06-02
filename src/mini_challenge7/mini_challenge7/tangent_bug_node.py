import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


DEFAULT_WAYPOINTS = [0.0, 0.0, 2.0, 0.0, 2.0, 2.0, 0.0, 2.0, 0.0, 0.0]


class TangentBugNode(Node):
    """
    Tangent Bug — dos estados:

    MOTION_TO_GOAL:
        Cada ciclo calcula el mejor ángulo de steering:
        - Si el camino al goal está libre → apunta al goal directamente.
        - Si no → encuentra los bordes de obstáculo visibles (discontinuidades LIDAR),
          elige el que minimiza h = d(robot,T) + d(T,goal), y apunta en ESA dirección
          (no intenta llegar físicamente al punto T).
        Entra a BOUNDARY_FOLLOW solo cuando el frente choca (<obstacle_threshold).

    BOUNDARY_FOLLOW:
        Wall-following clásico. Sale cuando:
          - bf_traveled > bf_min_travel, Y
          - el camino directo al goal está libre, O hay un tangente con
            d(T,goal) < d_bf - bf_exit_margin visible y sin obstáculo entre robot y T.
    """

    def __init__(self):
        super().__init__('tangent_bug_node')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/ekf_odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('waypoints', DEFAULT_WAYPOINTS)
        self.declare_parameter('goal_tolerance', 0.18)
        self.declare_parameter('max_linear_speed', 0.22)
        self.declare_parameter('max_angular_speed', 0.70)
        self.declare_parameter('obstacle_threshold', 0.40)
        self.declare_parameter('wall_distance', 0.40)
        self.declare_parameter('front_half_angle_deg', 30.0)
        self.declare_parameter('goal_cone_deg', 20.0)
        self.declare_parameter('discontinuity_threshold', 0.35)
        self.declare_parameter('min_tangent_dist', 0.20)
        self.declare_parameter('bf_exit_margin', 0.10)
        self.declare_parameter('bf_min_travel', 0.50)
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
        self.wall_distance = float(self.get_parameter('wall_distance').value)
        self.front_half_angle = float(self.get_parameter('front_half_angle_deg').value)
        self.goal_cone = math.radians(float(self.get_parameter('goal_cone_deg').value))
        self.discontinuity_threshold = float(self.get_parameter('discontinuity_threshold').value)
        self.min_tangent_dist = float(self.get_parameter('min_tangent_dist').value)
        self.bf_exit_margin = float(self.get_parameter('bf_exit_margin').value)
        self.bf_min_travel = float(self.get_parameter('bf_min_travel').value)
        self.wait_for_localization = bool(self.get_parameter('wait_for_localization').value)
        self.yaw_covariance_threshold = float(self.get_parameter('yaw_covariance_threshold').value)

        self.pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(LaserScan, scan_topic, self._scan_cb, 10)

        self.x = self.y = self.yaw = 0.0
        self.yaw_covariance = float('inf')
        self.odom_ready = False
        self.scan = None
        self._prev_pos = None

        self.wp_idx = 0
        self.lap_count = 0
        self.state = 'MOTION_TO_GOAL'
        self.wall_side = 'right'
        self.d_bf = float('inf')
        self.bf_traveled = 0.0
        self.corner_entry_pos = None
        self.localization_ready = False
        self._log_tick = 0

        self.create_timer(0.05, self._loop)
        self.get_logger().info(f'Tangent Bug started — {len(self.waypoints)} waypoints.')

    # ------------------------------------------------------------------ callbacks

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        nx, ny = p.x, p.y
        if self._prev_pos is not None and self.state == 'BOUNDARY_FOLLOW':
            self.bf_traveled += math.hypot(nx - self._prev_pos[0], ny - self._prev_pos[1])
        self._prev_pos = (nx, ny)
        self.x, self.y = nx, ny
        self.yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)
        self.yaw_covariance = msg.pose.covariance[35]
        self.odom_ready = True

    def _scan_cb(self, msg: LaserScan):
        self.scan = msg

    # ------------------------------------------------------------------ main loop

    def _loop(self):
        if not self.odom_ready or self.scan is None:
            return

        if self.wait_for_localization and not self.localization_ready:
            if self.yaw_covariance < self.yaw_covariance_threshold:
                self.localization_ready = True
                self.get_logger().info('Localization ready — starting Tangent Bug.')
            else:
                self._cmd(0.0, 0.0)
                return

        gx, gy = self.waypoints[self.wp_idx]
        dist = math.hypot(gx - self.x, gy - self.y)

        if dist < self.goal_tolerance:
            self.get_logger().info(f'WP {self.wp_idx} reached ({gx:.2f},{gy:.2f}).')
            self._cmd(0.0, 0.0)
            self._advance_waypoint()
            return

        self._log_tick += 1
        if self._log_tick >= 20:  # ~1 s at 20 Hz
            front = self._min_range(-self.front_half_angle, self.front_half_angle)
            self.get_logger().info(
                f'State={self.state}, wp={self.wp_idx}, '
                f'dist={dist:.2f}, front={front:.2f}, '
                f'bf_traveled={self.bf_traveled:.2f}'
            )
            self._log_tick = 0

        if self.state == 'MOTION_TO_GOAL':
            self._motion_to_goal(gx, gy, dist)
        else:
            self._boundary_follow(gx, gy, dist)

    # ------------------------------------------------------------------ MOTION_TO_GOAL

    def _motion_to_goal(self, gx: float, gy: float, dist: float):
        front = self._min_range(-self.front_half_angle, self.front_half_angle)

        if front < self.obstacle_threshold:
            self._enter_bf(gx, gy, dist)
            return

        goal_angle = math.atan2(gy - self.y, gx - self.x)

        if self._path_clear_to(gx, gy, dist):
            steer = goal_angle
        else:
            steer = self._best_steer_angle(gx, gy, goal_angle)

        self._steer(steer, dist)

    def _best_steer_angle(self, gx: float, gy: float, goal_angle: float) -> float:
        """
        Returns the world-frame steering angle whose heuristic d(robot,T)+d(T,goal)
        is minimal. If no tangent is found, falls back to goal_angle.
        The robot steers in this DIRECTION — it never 'visits' T as a waypoint,
        so tangent points on obstacle surfaces don't cause confusion.
        """
        tangents = self._find_tangent_points()
        if not tangents:
            return goal_angle

        best_angle = goal_angle
        best_h = float('inf')

        for tx, ty in tangents:
            d_rt = math.hypot(tx - self.x, ty - self.y)
            d_tg = math.hypot(tx - gx, ty - gy)
            h = d_rt + d_tg
            if h < best_h:
                best_h = h
                best_angle = math.atan2(ty - self.y, tx - self.x)

        return best_angle

    def _steer(self, world_angle: float, dist: float):
        heading_error = _wrap(world_angle - self.yaw)
        angular = _clamp(2.0 * heading_error, -self.max_angular, self.max_angular)
        # Slow down proportionally to heading error so we don't overshoot turns
        turn_factor = 1.0 - min(abs(heading_error) / math.radians(90.0), 1.0) * 0.85
        linear = _clamp(0.55 * dist, 0.0, self.max_linear) * turn_factor
        self._cmd(linear, angular)

    # ------------------------------------------------------------------ BOUNDARY_FOLLOW

    def _boundary_follow(self, gx: float, gy: float, dist: float):
        if self.bf_traveled > self.bf_min_travel:
            if self._should_exit_bf(gx, gy, dist):
                return

        self._follow_wall()

    def _should_exit_bf(self, gx: float, gy: float, dist: float) -> bool:
        # Condition 1: direct path to goal is now clear
        if self._path_clear_to(gx, gy, dist):
            self.get_logger().info(
                f'BF exit: goal clear at d={dist:.2f}m.'
            )
            self._exit_bf()
            return True

        # Condition 2: a visible tangent T with d(T,goal) < d_bf - margin
        for tx, ty in self._find_tangent_points():
            d_tg = math.hypot(tx - gx, ty - gy)
            d_rt = math.hypot(tx - self.x, ty - self.y)
            if d_tg >= self.d_bf - self.bf_exit_margin:
                continue
            if d_rt < self.min_tangent_dist:
                continue
            # Check no obstacle between robot and tangent point
            t_angle_robot = _wrap(math.atan2(ty - self.y, tx - self.x) - self.yaw)
            front_to_t = self._min_range(
                math.degrees(t_angle_robot) - 10,
                math.degrees(t_angle_robot) + 10,
            )
            if front_to_t > d_rt - 0.10:
                self.get_logger().info(
                    f'BF exit via tangent ({tx:.2f},{ty:.2f}) '
                    f'd(T,goal)={d_tg:.2f} < d_bf={self.d_bf:.2f}.'
                )
                self._exit_bf()
                return True

        return False

    def _enter_bf(self, gx: float, gy: float, dist: float):
        self.state = 'BOUNDARY_FOLLOW'
        self.d_bf = dist
        self.bf_traveled = 0.0
        self.wall_side = self._choose_wall_side(gx, gy)
        self.corner_entry_pos = None
        self.get_logger().warn(
            f'BF enter — d_bf={dist:.2f}m, side={self.wall_side}.'
        )

    def _exit_bf(self):
        self.state = 'MOTION_TO_GOAL'
        self.d_bf = float('inf')
        self.bf_traveled = 0.0
        self.corner_entry_pos = None

    # ------------------------------------------------------------------ tangent detection

    def _find_tangent_points(self):
        """
        Scans LIDAR for range discontinuities. Returns a list of (world_x, world_y)
        for the near endpoint of each discontinuity (= obstacle edge / tangent point).
        """
        if self.scan is None:
            return []

        scan = self.scan
        n = len(scan.ranges)
        tangents = []

        valid = [
            r if (math.isfinite(r) and scan.range_min < r < scan.range_max) else None
            for r in scan.ranges
        ]

        for i in range(n - 1):
            ri, rj = valid[i], valid[i + 1]
            if ri is None or rj is None:
                continue
            if abs(rj - ri) < self.discontinuity_threshold:
                continue

            # Near endpoint is the tangent point
            if ri <= rj:
                angle_robot = scan.angle_min + i * scan.angle_increment
                r_near = ri
            else:
                angle_robot = scan.angle_min + (i + 1) * scan.angle_increment
                r_near = rj

            d = r_near
            if d < self.min_tangent_dist:
                continue

            angle_world = self.yaw + angle_robot
            tx = self.x + d * math.cos(angle_world)
            ty = self.y + d * math.sin(angle_world)
            tangents.append((tx, ty))

        return tangents

    # ------------------------------------------------------------------ helpers

    def _path_clear_to(self, gx: float, gy: float, dist: float) -> bool:
        """True if no LIDAR reading in the ±goal_cone cone toward goal is closer than it."""
        if self.scan is None:
            return False
        goal_angle_robot = _wrap(math.atan2(gy - self.y, gx - self.x) - self.yaw)
        scan = self.scan
        for i, r in enumerate(scan.ranges):
            if not (math.isfinite(r) and scan.range_min < r < scan.range_max):
                continue
            angle = _wrap(scan.angle_min + i * scan.angle_increment)
            if abs(_wrap(angle - goal_angle_robot)) <= self.goal_cone:
                if r < dist - self.goal_tolerance:
                    return False
        return True

    def _choose_wall_side(self, gx: float, gy: float) -> str:
        left = self._finite_range_or_max(self._min_range(30, 120))
        right = self._finite_range_or_max(self._min_range(-120, -30))
        heading_error = _wrap(math.atan2(gy - self.y, gx - self.x) - self.yaw)
        if left < 0.5 and right > 0.5:
            return 'left'
        if right < 0.5 and left > 0.5:
            return 'right'
        return 'right' if heading_error > 0 else 'left'

    def _follow_wall(self):
        front_wide = self._min_range(-45, 45)

        if self.wall_side == 'left':
            side_range = self._min_range(55, 135)
            front_side = self._min_range(25, 65)
            turn_away = -self.max_angular
            turn_toward = self.max_angular
            corr = 1.0
        else:
            side_range = self._min_range(-135, -55)
            front_side = self._min_range(-65, -25)
            turn_away = self.max_angular
            turn_toward = -self.max_angular
            corr = -1.0

        side_range = self._finite_range_or_max(side_range)
        front_side = self._finite_range_or_max(front_side)
        outer = side_range > self.wall_distance * 2.4

        if outer:
            if self.corner_entry_pos is None:
                self.corner_entry_pos = (self.x, self.y)
            cleared = math.hypot(
                self.x - self.corner_entry_pos[0],
                self.y - self.corner_entry_pos[1],
            ) > 0.45
        else:
            self.corner_entry_pos = None
            cleared = True

        if front_wide < self.obstacle_threshold:
            self._cmd(0.0, turn_away)
        elif not outer and front_side < self.obstacle_threshold:
            self._cmd(self.max_linear * 0.45, turn_away * 0.8)
        elif outer:
            self._cmd(self.max_linear * 0.55, 0.0) if not cleared else \
                self._cmd(self.max_linear * 0.45, turn_toward * 0.45)
        else:
            error = side_range - self.wall_distance
            angular = _clamp(corr * 1.6 * error, -self.max_angular, self.max_angular)
            self._cmd(self.max_linear * 0.75, angular)

    def _advance_waypoint(self):
        self.wp_idx += 1
        if self.wp_idx >= len(self.waypoints):
            self.wp_idx = 0
            self.lap_count += 1
            self.get_logger().info(f'Lap {self.lap_count} complete.')
        self._exit_bf()

    def _min_range(self, min_deg: float, max_deg: float) -> float:
        if self.scan is None:
            return float('inf')
        scan = self.scan
        a_min = _wrap(math.radians(min_deg))
        a_max = _wrap(math.radians(max_deg))
        vals = []
        for i, r in enumerate(scan.ranges):
            if not (math.isfinite(r) and scan.range_min < r < scan.range_max):
                continue
            a = _wrap(scan.angle_min + i * scan.angle_increment)
            if a_min <= a_max:
                if a_min <= a <= a_max:
                    vals.append(r)
            else:
                if a >= a_min or a <= a_max:
                    vals.append(r)
        return min(vals) if vals else float('inf')

    def _finite_range_or_max(self, v: float) -> float:
        return v if math.isfinite(v) else (self.scan.range_max if self.scan else 10.0)

    def _cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub.publish(msg)


# ------------------------------------------------------------------ pure functions

def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main(args=None):
    rclpy.init(args=args)
    node = TangentBugNode()
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
