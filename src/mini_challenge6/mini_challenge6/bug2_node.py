import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# ── Fallback waypoints used when no config YAML is loaded ────────────────────
DEFAULT_WAYPOINTS = [0.0, -2.45]  # flat list: x1 y1  x2 y2 ...

# ── Robot limits ──────────────────────────────────────────────────────────────
MAX_LIN = 0.3   # m/s
MAX_ANG = 0.5   # rad/s

# ── Controller gains ──────────────────────────────────────────────────────────
KP_ANG  = 2.0   # P gain for heading error
KP_LIN  = 0.5   # P gain for distance to goal
KP_WALL = 1.5   # P gain for wall-distance error

# ── Thresholds ────────────────────────────────────────────────────────────────
DEFAULT_GOAL_TOL = 0.15  # m  — fallback when no YAML param is loaded
GOAL_TOL      = DEFAULT_GOAL_TOL
OBS_THRESHOLD = 0.40  # m  — frontal distance to trigger wall following
WALL_DIST     = 0.5   # m  — desired lateral distance while following wall
FORWARD_HALF  = 30    # deg — half-width of front obstacle detection cone
CORNER_CLEARANCE = 0.60  # m — forward travel needed to clear corner physically before turning
MLINE_TOL     = 0.10  # m  — perpendicular distance to consider robot on M-line
MIN_WALL_STEPS = 40   # control ticks before M-line re-entry is checked
STUCK_TIMEOUT = 2.0   # s  — time without movement before unstuck maneuver
STUCK_DIST    = 0.05  # m  — minimum displacement to consider robot moving
OBS_EXIT_THRESHOLD = 0.85  # m  — front clearance required to leave FOLLOW_WALL
HEADING_TOL   = 45    # deg — max heading error to consider path toward goal clear


class Bug2Node(Node):

    def __init__(self):
        super().__init__('bug2_node')

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(Odometry, '/ground_truth', self._odom_cb, 10)

        self.declare_parameter('waypoints', DEFAULT_WAYPOINTS)
        self.declare_parameter('goal_tolerance', DEFAULT_GOAL_TOL)
        flat = list(self.get_parameter('waypoints').value)
        self.waypoints = [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]
        self._goal_tol = float(self.get_parameter('goal_tolerance').value)
        self.get_logger().info(f'Waypoints loaded: {self.waypoints}  goal_tolerance={self._goal_tol}')
        self.wp_idx = 0

        self.x = self.y = self.yaw = 0.0
        self._scan: LaserScan | None = None
        self._odom_ready = False

        # M-line: the straight line from _mline_sx,sy to current waypoint
        self._mline_sx = self._mline_sy = 0.0
        # Distance to goal at the moment the wall was first hit
        self._hit_dist: float | None = None
        # Ticks spent in FOLLOW_WALL (prevents immediate M-line re-entry)
        self._wall_steps = 0
        self._corner_entry_pos: tuple[float, float] | None = None
        self._stuck_check_pos = (0.0, 0.0)
        self._stuck_elapsed   = 0.0

        self._state = 'INIT'

        self._dt = 0.05  # s
        self.create_timer(self._dt, self._loop)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        self._scan = msg

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x   = p.x
        self.y   = p.y
        self.yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)
        self._odom_ready = True

    # ── Main loop ─────────────────────────────────────────────────────────────

    def _loop(self):
        if self._scan is None or not self._odom_ready:
            return

        # Record starting position once odometry is available
        if self._state == 'INIT':
            self._mline_sx = self.x
            self._mline_sy = self.y
            self._state    = 'GO_TO_GOAL'
            return

        if self.wp_idx >= len(self.waypoints):
            self._cmd(0.0, 0.0)
            return

        gx, gy = self.waypoints[self.wp_idx]
        dx, dy = gx - self.x, gy - self.y
        dist   = math.hypot(dx, dy)
        front  = self._min_range(-FORWARD_HALF, FORWARD_HALF)

        if dist < self._goal_tol:
            self.get_logger().info(f'Waypoint {self.wp_idx} reached.')
            self.wp_idx += 1
            if self.wp_idx < len(self.waypoints):
                self._mline_sx = self.x
                self._mline_sy = self.y
            self._state      = 'GO_TO_GOAL'
            self._hit_dist   = None
            self._wall_steps = 0
            return

        if self._state == 'GO_TO_GOAL':
            if front < OBS_THRESHOLD:
                self.get_logger().info('Obstacle hit — entering wall following.')
                self._state           = 'FOLLOW_WALL'
                self._hit_dist        = dist
                self._wall_steps      = 0
                self._stuck_check_pos = (self.x, self.y)
                self._stuck_elapsed   = 0.0
            else:
                self._go_to_goal(dx, dy, dist)

        else:  # FOLLOW_WALL
            self._wall_steps += 1

            # Stuck detection: wheel caught on wall without LIDAR contact
            moved = math.hypot(self.x - self._stuck_check_pos[0],
                               self.y - self._stuck_check_pos[1])
            if moved > STUCK_DIST:
                self._stuck_check_pos = (self.x, self.y)
                self._stuck_elapsed   = 0.0
            else:
                self._stuck_elapsed += self._dt
                if self._stuck_elapsed >= STUCK_TIMEOUT:
                    self.get_logger().info('Stuck detected — nudging left.')
                    self._cmd(0.0, MAX_ANG)
                    self._stuck_elapsed   = 0.0
                    self._stuck_check_pos = (self.x, self.y)
                    return

            right        = self._min_range(-135, -50)
            outer_corner = right > WALL_DIST * 2.5
            if outer_corner:
                if self._corner_entry_pos is None:
                    self._corner_entry_pos = (self.x, self.y)
                corner_cleared = math.hypot(
                    self.x - self._corner_entry_pos[0],
                    self.y - self._corner_entry_pos[1],
                ) >= CORNER_CLEARANCE
            else:
                self._corner_entry_pos = None
                corner_cleared = True

            if self._wall_steps > MIN_WALL_STEPS:
                goal_angle  = math.atan2(dy, dx)
                heading_err = _wrap(goal_angle - self.yaw)
                closer      = dist < self._hit_dist - 0.05

                on_mline   = self._on_mline(gx, gy) and closer
                path_clear = (front > OBS_EXIT_THRESHOLD
                              and abs(heading_err) < math.radians(HEADING_TOL)
                              and closer)

                if on_mline or path_clear:
                    reason = 'M-line' if on_mline else 'path clear'
                    self.get_logger().info(f'Resuming to goal ({reason}).')
                    self._state           = 'GO_TO_GOAL'
                    self._hit_dist        = None
                    self._wall_steps      = 0
                    self._corner_entry_pos = None
                    self._stuck_elapsed   = 0.0
                    return
            self._follow_wall(front, corner_cleared)

    # ── Behaviors ─────────────────────────────────────────────────────────────

    def _go_to_goal(self, dx: float, dy: float, dist: float):
        goal_angle  = math.atan2(dy, dx)
        heading_err = _wrap(goal_angle - self.yaw)
        angular     = _clamp(KP_ANG * heading_err, -MAX_ANG, MAX_ANG)
        linear      = _clamp(KP_LIN * dist, 0.0, MAX_LIN)
        if abs(heading_err) > math.radians(45):
            # Avanzar levemente al rotar ayuda a que las llantas traseras
            # se separen de las paredes en lugar de pivotar y golpearlas.
            linear = MAX_LIN * 0.15
        self._cmd(linear, angular)

    def _follow_wall(self, front: float, corner_cleared: bool = True):
        # Right-hand wall following: keep the wall on the right side
        right        = self._min_range(-135, -50)
        front_right  = self._min_range(-60, -30)   # diagonal blind spot between front and right sensors
        err          = right - WALL_DIST            # positive → too far from wall → turn right
        outer_corner = right > WALL_DIST * 2.5
        if front < OBS_THRESHOLD:                              # inner corner / blocked front → turn left
            self._cmd(0.0, MAX_ANG)
        elif not outer_corner and front_right < OBS_THRESHOLD:  # diagonal wall → turn left
            self._cmd(0.0, MAX_ANG)
        elif outer_corner:
            if not corner_cleared:                             # llanta aún en esquina → avanzar recto
                self._cmd(MAX_LIN * 0.5, 0.0)
            elif front_right < OBS_THRESHOLD:                  # algo en la curva → recto
                self._cmd(MAX_LIN * 0.5, 0.0)
            else:                                              # despejado → girar derecha
                self._cmd(MAX_LIN * 0.5, -MAX_ANG * 0.5)
        else:
            angular = _clamp(-KP_WALL * err, -MAX_ANG, MAX_ANG)
            self._cmd(MAX_LIN * 0.8, angular)

    # ── Geometry helpers ──────────────────────────────────────────────────────

    def _on_mline(self, gx: float, gy: float) -> bool:
        sx, sy = self._mline_sx, self._mline_sy
        lx, ly = gx - sx, gy - sy
        length = math.hypot(lx, ly)
        if length < 1e-6:
            return False
        px, py = self.x - sx, self.y - sy
        # Perpendicular distance from robot to the M-line
        perp = abs(px * ly - py * lx) / length
        return perp < MLINE_TOL

    # ── LIDAR helpers ─────────────────────────────────────────────────────────

    def _min_range(self, a_min_deg: float, a_max_deg: float) -> float:
        scan  = self._scan
        a_min = math.radians(a_min_deg)
        a_max = math.radians(a_max_deg)
        i0 = round((a_min - scan.angle_min) / scan.angle_increment)
        i1 = round((a_max - scan.angle_min) / scan.angle_increment)
        i0 = max(0, min(i0, len(scan.ranges) - 1))
        i1 = max(0, min(i1, len(scan.ranges) - 1))
        if i0 > i1:
            i0, i1 = i1, i0
        vals = [r for r in scan.ranges[i0:i1 + 1]
                if math.isfinite(r) and scan.range_min < r < scan.range_max]
        return min(vals) if vals else float('inf')

    def _cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self._pub.publish(msg)


# ── Utilities ─────────────────────────────────────────────────────────────────

def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main(args=None):
    rclpy.init(args=args)
    node = Bug2Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
