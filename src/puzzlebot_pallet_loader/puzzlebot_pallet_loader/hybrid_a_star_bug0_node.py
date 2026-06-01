import heapq
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


DEFAULT_WAYPOINTS = [3.5, 2.7]

MAX_LIN = 0.25

LOCALIZATION_COV_THRESHOLD = 0.10
MAX_ANG = 0.60

KP_ANG = 2.0
KP_LIN = 0.5
KP_WALL = 1.5

DEFAULT_GOAL_TOL = 0.15
PATH_POINT_TOL = 0.12

OBS_THRESHOLD = 0.5
OBS_EXIT_THRESHOLD = 0.85
WALL_DIST = 0.65
FORWARD_HALF = 30
HEADING_TOL = 45

CORNER_CLEARANCE = 0.60
WALL_FOLLOW_MIN_DIST = 0.50
REPLAN_DISTANCE = 1.20

class HybridAStarBug0Node(Node):
    def __init__(self):
        super().__init__('hybrid_a_star_bug0_node')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        scan_topic = self.get_parameter('scan_topic').value

        self._pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(LaserScan, scan_topic, self._scan_cb, 10)

        self.get_logger().info(
            f'Using topics: cmd_vel={cmd_vel_topic}, odom={odom_topic}, scan={scan_topic}'
        )

        self.declare_parameter('waypoints', DEFAULT_WAYPOINTS)
        self.declare_parameter('goal_tolerance', DEFAULT_GOAL_TOL)

        self.declare_parameter('bounds', [-3.0, -3.0, 3.0, 3.0])
        self.declare_parameter('resolution', 0.10)
        self.declare_parameter('obstacles', Parameter.Type.DOUBLE_ARRAY)
        #self.declare_parameter('obstacles', []) #prueba en físico
        self.declare_parameter('inflation_radius', 0.25)
        self.declare_parameter('path_stride', 1) # salto entre puntos de la ruta, mayor para rutas más curvas
        self.declare_parameter('known_obstacle_tolerance', 0.15)
        self.declare_parameter('known_obstacle_angle_window_deg', 20.0)
        
        flat = list(self.get_parameter('waypoints').value)
        self.waypoints = [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]

        self._goal_tol = float(self.get_parameter('goal_tolerance').value)
        self._bounds = list(self.get_parameter('bounds').value)
        self._resolution = float(self.get_parameter('resolution').value)
        self._obstacles = list(self.get_parameter('obstacles').value)
        self._inflation_radius = float(self.get_parameter('inflation_radius').value)
        self._path_stride = int(self.get_parameter('path_stride').value)
        self._known_obstacle_tolerance = float(
            self.get_parameter('known_obstacle_tolerance').value
        )

        self._known_obstacle_angle_window = math.radians(
            float(self.get_parameter('known_obstacle_angle_window_deg').value)
        )

        if len(self._obstacles) % 4 != 0:
            self.get_logger().error(
                'Parameter "obstacles" must be groups of 4 values: '
                '[xmin, ymin, xmax, ymax, ...]'
            )
            self._obstacles = []

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._odom_ready = False
        self._localization_converged = False
        self._yaw_cov = float('inf')
        self._scan = None

        self.wp_idx = 0
        self.path = []
        self.path_idx = 0

        self._state = 'FOLLOW_PATH'
        self._wall_entry_pos = None
        self._corner_entry_pos = None
        self._bug0_side = None

        self.create_timer(0.05, self._loop)

        self.get_logger().info(
            f'Hybrid A* + Bug0 started. Waypoints={self.waypoints}, '
            f'obstacles={len(self._obstacles) // 4}'
        )

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.x = p.x
        self.y = p.y
        self.yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)
        self._odom_ready = True

        # Track yaw covariance (index 35 in 6×6 row-major = θθ)
        self._yaw_cov = msg.pose.covariance[35]

    def _scan_cb(self, msg: LaserScan):
        self._scan = msg

    def _loop(self):
        if not self._odom_ready or self._scan is None:
            return

        # Wait for ArUco corrections to converge before moving
        if not self._localization_converged:
            if self._yaw_cov < LOCALIZATION_COV_THRESHOLD:
                self._localization_converged = True
                self.get_logger().info(
                    f'Localization converged (yaw_cov={self._yaw_cov:.4f}). '
                    f'Starting navigation from ({self.x:.2f}, {self.y:.2f}, '
                    f'{math.degrees(self.yaw):.1f}°)'
                )
            else:
                self._cmd(0.0, 0.0)
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
            self._state = 'FOLLOW_PATH'
            self._bug0_side = None
            self._cmd(0.0, 0.0)
            return

        if not self.path:
            self._plan_to_goal(gx, gy)
            return

        front = self._min_range(-FORWARD_HALF, FORWARD_HALF)

        if self._state == 'FOLLOW_PATH':
            if front < OBS_THRESHOLD:
                if self._front_obstacle_is_known():
                    self._follow_astar_path()
                    return

                self._state = 'BUG0_AVOID'
                self._wall_entry_pos = (self.x, self.y)
                self._corner_entry_pos = None
                self._bug0_side = self._choose_bug0_side()
                return

            self._follow_astar_path()

        elif self._state == 'BUG0_AVOID':
            if self._can_return_to_path(front):
                self.get_logger().info('Path is clear — returning to A* path.')
                self._state = 'FOLLOW_PATH'
                self._wall_entry_pos = None
                self._corner_entry_pos = None
                self._bug0_side = None
                return

            if self._distance_from_current_path_point() > REPLAN_DISTANCE:
                self.get_logger().info('Robot moved far from path — replanning A*.')
                self.path = []
                self.path_idx = 0
                self._state = 'FOLLOW_PATH'
                return

            self._bug0_follow_wall(front)

    def _follow_astar_path(self):
        if self.path_idx >= len(self.path):
            # The planned path is already consumed, but the robot may still be
            # slightly outside the waypoint tolerance. Instead of replanning every
            # control tick, directly approach the final global waypoint.
            gx, gy = self.waypoints[self.wp_idx]
            dist = math.hypot(gx - self.x, gy - self.y)

            if dist > self._goal_tol:
                self._go_to_point(gx, gy, dist)
            else:
                self._cmd(0.0, 0.0)

            return

        tx, ty = self.path[self.path_idx]
        dist = math.hypot(tx - self.x, ty - self.y)

        if dist < PATH_POINT_TOL:
            self.path_idx += 1
            return

        self._go_to_point(tx, ty, dist)

    def _can_return_to_path(self, front: float):
        if self.path_idx >= len(self.path):
            return False

        tx, ty = self.path[self.path_idx]
        goal_angle = math.atan2(ty - self.y, tx - self.x)
        heading_err = abs(_wrap(goal_angle - self.yaw))

        wall_traveled = math.hypot(
            self.x - self._wall_entry_pos[0],
            self.y - self._wall_entry_pos[1],
        ) if self._wall_entry_pos else float('inf')

        return (
            front > OBS_EXIT_THRESHOLD
            and heading_err < math.radians(HEADING_TOL)
            and wall_traveled > WALL_FOLLOW_MIN_DIST
        )
    
    def _choose_bug0_side(self):
        left = self._min_range(25, 115)
        front_left = self._min_range(10, 60)

        right = self._min_range(-115, -25)
        front_right = self._min_range(-60, -10)

        max_range = self._scan.range_max if self._scan is not None else 10.0

        left = max_range if math.isinf(left) else left
        front_left = max_range if math.isinf(front_left) else front_left
        right = max_range if math.isinf(right) else right
        front_right = max_range if math.isinf(front_right) else front_right

        left_score = 0.6 * left + 0.4 * front_left
        right_score = 0.6 * right + 0.4 * front_right

        side = 'left' if left_score > right_score else 'right'
        
        self.get_logger().info(
            f'Bug0 side selected: {side} '
            f'(left_score={left_score:.2f}, right_score={right_score:.2f})'
        )

        return side

    def _bug0_follow_wall(self, front: float):
        front_wide = self._min_range(-45, 45)

        if self._bug0_side == 'left':
            side_range = self._min_range(50, 135)
            front_side = self._min_range(30, 60)

            turn_away = -MAX_ANG
            turn_toward = MAX_ANG
            wall_correction_sign = 1.0

        else:
            side_range = self._min_range(-135, -50)
            front_side = self._min_range(-60, -30)

            turn_away = MAX_ANG
            turn_toward = -MAX_ANG
            wall_correction_sign = -1.0

        err = side_range - WALL_DIST
        outer_corner = side_range > WALL_DIST * 2.5

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

        if front_wide < WALL_DIST:
            # Back up strongly and turn away to escape corner
            self._cmd(-MAX_LIN * 0.4, turn_away)

        elif front_wide < OBS_THRESHOLD:
            # Move forward slightly but turn strongly away
            self._cmd(MAX_LIN * 0.3, turn_away)

        elif not outer_corner and front_side < OBS_THRESHOLD:
            # Side is too close, push forward and turn away
            self._cmd(MAX_LIN * 0.5, turn_away)

        elif outer_corner:
            if not corner_cleared:
                self._cmd(MAX_LIN * 0.6, 0.0)

            elif front_side < OBS_THRESHOLD:
                self._cmd(MAX_LIN * 0.6, 0.0)

            else:
                self._cmd(MAX_LIN * 0.5, turn_toward * 0.5)

        else:
            angular = _clamp(
                wall_correction_sign * KP_WALL * err,
                -MAX_ANG,
                MAX_ANG
            )
            self._cmd(MAX_LIN * 0.8, angular)

    def _distance_from_current_path_point(self):
        if self.path_idx >= len(self.path):
            return 0.0

        tx, ty = self.path[self.path_idx]
        return math.hypot(tx - self.x, ty - self.y)

    def _plan_to_goal(self, gx: float, gy: float):
        start = self._world_to_grid(self.x, self.y)
        goal = self._world_to_grid(gx, gy)
        occupancy = self._build_occupancy_grid()

        if not self._inside_grid(start) or not self._inside_grid(goal):
            self.get_logger().error('Start or goal outside A* bounds.')
            self._cmd(0.0, 0.0)
            return

        if occupancy[start[1]][start[0]]:
            free = self._nearest_free_cell(start, occupancy)
            if free is None:
                self.get_logger().error('Start cell is occupied — no free neighbour found.')
                self._cmd(0.0, 0.0)
                return
            self.get_logger().warn(
                f'Start cell {start} occupied, using nearest free cell {free}.'
            )
            start = free

        if occupancy[goal[1]][goal[0]]:
            free = self._nearest_free_cell(goal, occupancy)
            if free is None:
                self.get_logger().error(
                    f'Goal cell is occupied. goal=({gx:.2f},{gy:.2f}) '
                    f'grid={goal} inflation={self._inflation_radius}m — no free neighbour.'
                )
                self._cmd(0.0, 0.0)
                return
            self.get_logger().warn(
                f'Goal cell {goal} occupied, using nearest free cell {free}.'
            )
            goal = free

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

    def _reconstruct_path(self, came_from, current):
        path = [current]

        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()
        return path

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

    def _nearest_free_cell(self, cell, occupancy, max_radius=20):
        """BFS search for the closest free cell starting from *cell*."""
        from collections import deque
        visited = {cell}
        queue = deque([cell])
        while queue:
            cx, cy = queue.popleft()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = cx + dx, cy + dy
                    if (nx, ny) in visited:
                        continue
                    visited.add((nx, ny))
                    if not self._inside_grid((nx, ny)):
                        continue
                    dist = abs(nx - cell[0]) + abs(ny - cell[1])
                    if dist > max_radius:
                        continue
                    if not occupancy[ny][nx]:
                        return (nx, ny)
                    queue.append((nx, ny))
        return None

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

    def _min_range(self, a_min_deg: float, a_max_deg: float):
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
    
    def _front_obstacle_point_global(self):
        """
        Returns the closest obstacle point detected in front of the robot,
        converted from LiDAR polar coordinates to global/world coordinates.

        If there is no valid frontal reading, returns None.
        """

        if self._scan is None:
            return None

        scan = self._scan

        a_min = -self._known_obstacle_angle_window
        a_max = self._known_obstacle_angle_window

        i0 = round((a_min - scan.angle_min) / scan.angle_increment)
        i1 = round((a_max - scan.angle_min) / scan.angle_increment)

        i0 = max(0, min(i0, len(scan.ranges) - 1))
        i1 = max(0, min(i1, len(scan.ranges) - 1))

        if i0 > i1:
            i0, i1 = i1, i0

        best_range = float('inf')
        best_index = None

        for idx in range(i0, i1 + 1):
            r = scan.ranges[idx]

            if not math.isfinite(r):
                continue

            if not (scan.range_min < r < scan.range_max):
                continue

            if r < best_range:
                best_range = r
                best_index = idx

        if best_index is None:
            return None

        # Angle of the detected point in the robot/LiDAR frame
        local_angle = scan.angle_min + best_index * scan.angle_increment

        # Point in robot local frame
        local_x = best_range * math.cos(local_angle)
        local_y = best_range * math.sin(local_angle)

        # Transform from robot frame to world frame
        global_x = self.x + local_x * math.cos(self.yaw) - local_y * math.sin(self.yaw)
        global_y = self.y + local_x * math.sin(self.yaw) + local_y * math.cos(self.yaw)

        return global_x, global_y, best_range
    
    def _point_is_inside_known_obstacle(self, px: float, py: float) -> bool:
        """
        Checks whether a global point falls inside or near one of the
        rectangular known obstacles loaded from the YAML map.
        """

        tol = self._known_obstacle_tolerance

        for i in range(0, len(self._obstacles), 4):
            xmin = float(self._obstacles[i]) - tol
            ymin = float(self._obstacles[i + 1]) - tol
            xmax = float(self._obstacles[i + 2]) + tol
            ymax = float(self._obstacles[i + 3]) + tol

            if xmin <= px <= xmax and ymin <= py <= ymax:
                return True

        return False
    
    def _front_obstacle_is_known(self) -> bool:
        """
        Returns True if the closest frontal LiDAR obstacle corresponds to a known
        obstacle from the YAML map.
        """

        obstacle_point = self._front_obstacle_point_global()

        if obstacle_point is None:
            return False

        px, py, distance = obstacle_point

        is_known = self._point_is_inside_known_obstacle(px, py)

        if is_known:
            self.get_logger().info(
                f'Known obstacle detected at ({px:.2f}, {py:.2f}), '
                f'distance={distance:.2f} m. Continuing A* path.'
            )
        else:
            self.get_logger().warn(
                f'Unknown obstacle detected at ({px:.2f}, {py:.2f}), '
                f'distance={distance:.2f} m. Switching to Bug0.'
            )

        return is_known

    def _cmd(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self._pub.publish(msg)


def _wrap(angle: float):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _clamp(val: float, lo: float, hi: float):
    return max(lo, min(hi, val))


def _quat_to_yaw(x: float, y: float, z: float, w: float):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main(args=None):
    rclpy.init(args=args)
    node = HybridAStarBug0Node()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print('Hybrid A* + Bug0 node stopped by user.')

    except RuntimeError as e:
        # rclpy can throw RuntimeError during launch shutdown / Ctrl+C.
        # Treat it as a clean shutdown instead of printing a traceback.
        print(f'Hybrid A* + Bug0 node shutting down: {e}')

    finally:
        # Send a final stop command for safety.
        try:
            node._cmd(0.0, 0.0)
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()