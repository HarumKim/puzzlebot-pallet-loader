import heapq
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.parameter import Parameter


DEFAULT_WAYPOINTS = [0.0, -2.45]

MAX_LIN = 0.3
MAX_ANG = 0.5

KP_ANG = 2.0
KP_LIN = 0.5

DEFAULT_GOAL_TOL = 0.15
PATH_POINT_TOL = 0.12
FRONT_STOP_THRESHOLD = 0.35
FORWARD_HALF = 25


class AStarNode(Node):
    def __init__(self):
        super().__init__('a_star_node')

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/ground_truth', self._odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

        self.declare_parameter('waypoints', DEFAULT_WAYPOINTS)
        self.declare_parameter('goal_tolerance', DEFAULT_GOAL_TOL)

        # Static A* map parameters.
        # bounds = [xmin, ymin, xmax, ymax]
        self.declare_parameter('bounds', [-3.0, -3.0, 3.0, 3.0])
        self.declare_parameter('resolution', 0.10)

        # Obstacles as flat rectangles:
        # obstacles = [xmin1, ymin1, xmax1, ymax1, xmin2, ymin2, xmax2, ymax2, ...]
        self.declare_parameter('obstacles', Parameter.Type.DOUBLE_ARRAY)

        # Inflate obstacles to avoid planning too close to walls.
        self.declare_parameter('inflation_radius', 0.20)

        # Use every Nth A* point as a tracking point.
        self.declare_parameter('path_stride', 2)

        flat = list(self.get_parameter('waypoints').value)
        self.waypoints = [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]

        self._goal_tol = float(self.get_parameter('goal_tolerance').value)
        self._bounds = list(self.get_parameter('bounds').value)
        self._resolution = float(self.get_parameter('resolution').value)
        self._obstacles = list(self.get_parameter('obstacles').value)
        if len(self._obstacles) % 4 != 0:
            self.get_logger().error(
                'Parameter "obstacles" must contain groups of 4 values: '
                '[xmin, ymin, xmax, ymax, ...]'
            )
            self._obstacles = []
        self._inflation_radius = float(self.get_parameter('inflation_radius').value)
        self._path_stride = int(self.get_parameter('path_stride').value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._odom_ready = False
        self._scan = None

        self.wp_idx = 0
        self.path = []
        self.path_idx = 0

        self.get_logger().info(
            f'A* loaded: waypoints={self.waypoints}, bounds={self._bounds}, '
            f'resolution={self._resolution}, obstacles={len(self._obstacles) // 4}'
        )

        self.create_timer(0.05, self._loop)

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.x = p.x
        self.y = p.y
        self.yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)
        self._odom_ready = True

    def _scan_cb(self, msg: LaserScan):
        self._scan = msg

    def _loop(self):
        if not self._odom_ready:
            return

        if self.wp_idx >= len(self.waypoints):
            self._cmd(0.0, 0.0)
            return

        gx, gy = self.waypoints[self.wp_idx]
        dist_to_goal = math.hypot(gx - self.x, gy - self.y)

        if dist_to_goal < self._goal_tol:
            self.get_logger().info(f'Waypoint {self.wp_idx} reached.')
            self.wp_idx += 1
            self.path = []
            self.path_idx = 0
            self._cmd(0.0, 0.0)
            return

        if not self.path:
            self._plan_to_goal(gx, gy)
            return

        if self.path_idx >= len(self.path):
            self.path = []
            self.path_idx = 0
            return

#        # Emergency local safety using LiDAR.
#        if self._scan is not None:
#            front = self._min_range(-FORWARD_HALF, FORWARD_HALF)
#            if front < FRONT_STOP_THRESHOLD:
#                self.get_logger().warn('Unexpected obstacle detected — turning in place.')
#                self._cmd(0.0, MAX_ANG)
#                return
            
        tx, ty = self.path[self.path_idx]
        dist_to_point = math.hypot(tx - self.x, ty - self.y)

        if dist_to_point < PATH_POINT_TOL:
            self.path_idx += 1
            return

        self._go_to_point(tx, ty, dist_to_point)

    def _plan_to_goal(self, gx: float, gy: float):
        start = self._world_to_grid(self.x, self.y)
        goal = self._world_to_grid(gx, gy)

        occupancy = self._build_occupancy_grid()

        if not self._inside_grid(start) or not self._inside_grid(goal):
            self.get_logger().error('Start or goal is outside A* bounds.')
            self._cmd(0.0, 0.0)
            return

        if occupancy[start[1]][start[0]]:
            self.get_logger().error('Start cell is occupied.')
            self._cmd(0.0, 0.0)
            return

        if occupancy[goal[1]][goal[0]]:
            self.get_logger().error('Goal cell is occupied.')
            self._cmd(0.0, 0.0)
            return

        grid_path = self._astar(start, goal, occupancy)

        if not grid_path:
            self.get_logger().error('A* could not find a path.')
            self._cmd(0.0, 0.0)
            return

        world_path = [self._grid_to_world(ix, iy) for ix, iy in grid_path]

        stride = max(1, self._path_stride)
        reduced = world_path[::stride]

        if reduced[-1] != world_path[-1]:
            reduced.append(world_path[-1])

        self.path = reduced
        self.path_idx = 0

        self.get_logger().info(
            f'A* path generated: {len(grid_path)} cells, {len(self.path)} tracking points.'
        )

    def _astar(self, start, goal, occupancy):
        open_heap = []
        heapq.heappush(open_heap, (0.0, start))

        came_from = {}
        g_score = {start: 0.0}

        while open_heap:
            _, current = heapq.heappop(open_heap)

            if current == goal:
                return self._reconstruct_path(came_from, current)

            for neighbor, step_cost in self._neighbors(current, occupancy):
                tentative = g_score[current] + step_cost

                if neighbor not in g_score or tentative < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative
                    f_score = tentative + self._heuristic(neighbor, goal)
                    heapq.heappush(open_heap, (f_score, neighbor))

        return []

    def _neighbors(self, cell, occupancy):
        x, y = cell

        candidates = [
            (x + 1, y, 1.0),
            (x - 1, y, 1.0),
            (x, y + 1, 1.0),
            (x, y - 1, 1.0),
            (x + 1, y + 1, math.sqrt(2.0)),
            (x + 1, y - 1, math.sqrt(2.0)),
            (x - 1, y + 1, math.sqrt(2.0)),
            (x - 1, y - 1, math.sqrt(2.0)),
        ]

        valid = []
        for nx, ny, cost in candidates:
            if not self._inside_grid((nx, ny)):
                continue
            if occupancy[ny][nx]:
                continue
            valid.append(((nx, ny), cost))

        return valid

    def _reconstruct_path(self, came_from, current):
        path = [current]

        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()
        return path

    def _build_occupancy_grid(self):
        xmin, ymin, xmax, ymax = self._bounds
        width = int(math.ceil((xmax - xmin) / self._resolution)) + 1
        height = int(math.ceil((ymax - ymin) / self._resolution)) + 1

        grid = [[False for _ in range(width)] for _ in range(height)]

        inflation_cells = int(math.ceil(self._inflation_radius / self._resolution))

        for i in range(0, len(self._obstacles), 4):
            oxmin = float(self._obstacles[i])
            oymin = float(self._obstacles[i + 1])
            oxmax = float(self._obstacles[i + 2])
            oymax = float(self._obstacles[i + 3])

            c0 = self._world_to_grid(oxmin, oymin)
            c1 = self._world_to_grid(oxmax, oymax)

            x0, x1 = sorted([c0[0], c1[0]])
            y0, y1 = sorted([c0[1], c1[1]])

            for gy in range(y0 - inflation_cells, y1 + inflation_cells + 1):
                for gx in range(x0 - inflation_cells, x1 + inflation_cells + 1):
                    if self._inside_grid((gx, gy)):
                        grid[gy][gx] = True

        return grid

    def _world_to_grid(self, x: float, y: float):
        xmin, ymin, _, _ = self._bounds
        ix = int(round((x - xmin) / self._resolution))
        iy = int(round((y - ymin) / self._resolution))
        return ix, iy

    def _grid_to_world(self, ix: int, iy: int):
        xmin, ymin, _, _ = self._bounds
        x = xmin + ix * self._resolution
        y = ymin + iy * self._resolution
        return x, y

    def _inside_grid(self, cell):
        ix, iy = cell
        xmin, ymin, xmax, ymax = self._bounds
        width = int(math.ceil((xmax - xmin) / self._resolution)) + 1
        height = int(math.ceil((ymax - ymin) / self._resolution)) + 1
        return 0 <= ix < width and 0 <= iy < height

    def _heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _go_to_point(self, tx: float, ty: float, dist: float):
        goal_angle = math.atan2(ty - self.y, tx - self.x)
        heading_err = _wrap(goal_angle - self.yaw)

        angular = _clamp(KP_ANG * heading_err, -MAX_ANG, MAX_ANG)
        linear = _clamp(KP_LIN * dist, 0.0, MAX_LIN)

        if abs(heading_err) > math.radians(45):
            linear = MAX_LIN * 0.15

        self._cmd(linear, angular)

    def _min_range(self, a_min_deg: float, a_max_deg: float) -> float:
        scan = self._scan
        a_min = math.radians(a_min_deg)
        a_max = math.radians(a_max_deg)

        i0 = round((a_min - scan.angle_min) / scan.angle_increment)
        i1 = round((a_max - scan.angle_min) / scan.angle_increment)

        i0 = max(0, min(i0, len(scan.ranges) - 1))
        i1 = max(0, min(i1, len(scan.ranges) - 1))

        if i0 > i1:
            i0, i1 = i1, i0

        vals = [
            r for r in scan.ranges[i0:i1 + 1]
            if math.isfinite(r) and scan.range_min < r < scan.range_max
        ]

        return min(vals) if vals else float('inf')

    def _cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self._pub.publish(msg)


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _clamp(val: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, val))


def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main(args=None):
    rclpy.init(args=args)
    node = AStarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()