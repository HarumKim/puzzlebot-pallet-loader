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
KP_ANG  = 2.0
KP_LIN  = 0.5
KP_WALL = 1.5

# ── Thresholds ────────────────────────────────────────────────────────────────
GOAL_TOL      = 0.15  # m  — waypoint reached tolerance
OBS_THRESHOLD = 0.40  # m  — frontal distance to trigger wall following
WALL_DIST     = 0.35  # m  — desired lateral distance while following wall
FORWARD_HALF  = 30    # deg — half-width of front obstacle detection cone
MLINE_TOL     = 0.10  # m  — perpendicular distance to consider robot on M-line
MIN_WALL_STEPS = 40   # control ticks before M-line re-entry is checked


class Bug2Node(Node):

    def __init__(self):
        super().__init__('bug2_node')

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(Odometry, '/ground_truth', self._odom_cb, 10)

        self.declare_parameter('waypoints', DEFAULT_WAYPOINTS)
        flat = list(self.get_parameter('waypoints').value)
        self.waypoints = [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]
        self.get_logger().info(f'Waypoints loaded: {self.waypoints}')
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

        if dist < GOAL_TOL:
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
                self._state      = 'FOLLOW_WALL'
                self._hit_dist   = dist
                self._wall_steps = 0
            else:
                self._go_to_goal(dx, dy, dist)

        else:  # FOLLOW_WALL
            self._wall_steps += 1
            if self._wall_steps > MIN_WALL_STEPS:
                # Leave wall when back on M-line AND closer to goal than when we left it
                if self._on_mline(gx, gy) and dist < self._hit_dist - 0.05:
                    self.get_logger().info('Back on M-line closer to goal — resuming.')
                    self._state      = 'GO_TO_GOAL'
                    self._hit_dist   = None
                    self._wall_steps = 0
                    return
            self._follow_wall(front)

    # ── Behaviors ─────────────────────────────────────────────────────────────

    def _go_to_goal(self, dx: float, dy: float, dist: float):
        goal_angle  = math.atan2(dy, dx)
        heading_err = _wrap(goal_angle - self.yaw)
        angular     = _clamp(KP_ANG * heading_err, -MAX_ANG, MAX_ANG)
        linear      = _clamp(KP_LIN * dist, 0.0, MAX_LIN)
        if abs(heading_err) > math.radians(45):
            linear = 0.0
        self._cmd(linear, angular)

    def _follow_wall(self, front: float):
        # Right-hand wall following
        right = self._min_range(-100, -60)
        err   = right - WALL_DIST          # positive → too far → steer right
        if front < OBS_THRESHOLD:          # blocked in front → turn left in place
            self._cmd(0.0, MAX_ANG)
        else:
            angular = _clamp(-KP_WALL * err, -MAX_ANG, MAX_ANG)
            self._cmd(MAX_LIN * 0.6, angular)

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
