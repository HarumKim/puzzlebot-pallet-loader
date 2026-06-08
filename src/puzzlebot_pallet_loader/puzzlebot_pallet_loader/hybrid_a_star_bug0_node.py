import heapq
import math
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32MultiArray


DEFAULT_WAYPOINTS = [3.5, 2.7]

MAX_LIN = 0.04

LOCALIZATION_COV_THRESHOLD = 0.50
MAX_ANG = 0.05

KP_ANG = 1.8
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

# LiDAR montado 180° respecto al frente del robot.
# Todos los ángulos del scan se rotan por este offset antes de usarse.
SCAN_ANGLE_OFFSET_DEG = 180.0

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
        self.declare_parameter('navigation_status_topic', '/navigation/status')
        navigation_status_topic = self.get_parameter('navigation_status_topic').value
        self._nav_status_pub = self.create_publisher(String, navigation_status_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(LaserScan, scan_topic, self._scan_cb, 10)
        self.create_subscription(Float32MultiArray, '/navigation/set_goal', self._set_goal_cb, 10)

        self.get_logger().info(
            f'Using topics: cmd_vel={cmd_vel_topic}, odom={odom_topic}, scan={scan_topic}'
        )

        self.declare_parameter('waypoints', DEFAULT_WAYPOINTS)
        self.declare_parameter('goal_tolerance', DEFAULT_GOAL_TOL)
        self.declare_parameter('waypoint_yaw_tolerance_deg', 8.0)
        self.declare_parameter('final_yaw_kp', 1.5)
        self.declare_parameter('final_yaw_max_ang', 0.08)

        self.declare_parameter('bounds', [-3.0, -3.0, 3.0, 3.0])
        self.declare_parameter('resolution', 0.10)
        self.declare_parameter('obstacles', Parameter.Type.DOUBLE_ARRAY)
        #self.declare_parameter('obstacles', []) #prueba en físico
        self.declare_parameter('inflation_radius', 0.25)
        self.declare_parameter('path_stride', 1) # salto entre puntos de la ruta, mayor para rutas más curvas
        self.declare_parameter('known_obstacle_tolerance', 0.15)
        self.declare_parameter('known_obstacle_angle_window_deg', 20.0)
        self.declare_parameter('discontinuity_threshold', 0.35)
        self.declare_parameter('min_tangent_dist', 0.20)
        self.declare_parameter('goal_cone_deg', 20.0)
        self.declare_parameter('bf_exit_margin', 0.10)
        
        # flat = list(self.get_parameter('waypoints').value)
        # self.waypoints = [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]
        flat = list(self.get_parameter('waypoints').value)

        if len(flat) % 3 != 0:
            self.get_logger().error(
                'Parameter "waypoints" must use groups of 3 values: '
                '[x0, y0, yaw0_deg, x1, y1, yaw1_deg, ...]'
            )
            self.waypoints = []
        else:
            self.waypoints = [
                (flat[i], flat[i + 1], math.radians(flat[i + 2]))
                for i in range(0, len(flat), 3)
            ]


        self._goal_tol = float(self.get_parameter('goal_tolerance').value)
        self._wp_yaw_tol = math.radians(
            float(self.get_parameter('waypoint_yaw_tolerance_deg').value)
        )
        self._final_yaw_kp = float(self.get_parameter('final_yaw_kp').value)
        self._final_yaw_max_ang = float(self.get_parameter('final_yaw_max_ang').value)
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
        self._discontinuity_threshold = float(self.get_parameter('discontinuity_threshold').value)
        self._min_tangent_dist = float(self.get_parameter('min_tangent_dist').value)
        self._goal_cone = math.radians(float(self.get_parameter('goal_cone_deg').value))
        self._bf_exit_margin = float(self.get_parameter('bf_exit_margin').value)

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
        self._aligning_final_yaw = False

        self._state = 'FOLLOW_PATH'
        self._wall_entry_pos = None
        self._corner_entry_pos = None
        self._bug0_side = None
        self._dynamic_obstacles = []  # (x, y) world coords of unknown obstacles found by Bug0
        self._unknown_obs_frames = 0  # consecutive frames with unknown obstacle in front
        self._d_bf = float('inf')     # distance to goal when entering tangent boundary follow
        self._bf_traveled = 0.0       # distance traveled in tangent boundary/steer states
        self._prev_pos_bf = None      # previous position for bf_traveled measurement

        self.declare_parameter('enable_keyboard', False)
        self._enable_keyboard = bool(self.get_parameter('enable_keyboard').value)

        self._paused = False

        if self._enable_keyboard and sys.stdin.isatty():
            self._kb_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
            self._kb_thread.start()
            self.get_logger().info('Teclado: ESPACIO = pausar/reanudar | q = salir')
        else:
            self._kb_thread = None
            self.get_logger().info('Keyboard listener disabled for launch mode.')

        self.create_timer(0.05, self._loop)

        self.get_logger().info(
            f'Hybrid A* + Bug0 started. Waypoints={self.waypoints}, '
            f'obstacles={len(self._obstacles) // 4}'
        )
        #self.get_logger().info('Teclado: ESPACIO = pausar/reanudar | q = salir')

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

    def _set_goal_cb(self, msg: Float32MultiArray):
        """Recibe [x, y, yaw_deg] y lo pone como unico waypoint."""
        if len(msg.data) < 3:
            self.get_logger().warn('set_goal: necesita [x, y, yaw_deg]')
            return
        x, y, yaw_deg = msg.data[0], msg.data[1], msg.data[2]
        self.waypoints = [(x, y, math.radians(yaw_deg))]
        self.wp_idx = 0
        self.path = []
        self.path_idx = 0
        self._state = 'FOLLOW_PATH'
        self._aligning_final_yaw = False
        self._bug0_side = None
        self._dynamic_obstacles = []
        self._localization_converged = True
        self.get_logger().info(f'Goal dinamico recibido: ({x:.2f}, {y:.2f}, {yaw_deg:.0f}deg)')

    def _publish_navigation_status(self, status: str):
        msg = String()
        msg.data = status
        self._nav_status_pub.publish(msg)
        self.get_logger().info(f'Navigation status: {status}')

    def _keyboard_listener(self):
        if not sys.stdin.isatty():
            self.get_logger().warn('No interactive stdin available. Keyboard listener disabled.')
            return

        fd = sys.stdin.fileno()

        try:
            old = termios.tcgetattr(fd)
        except termios.error as exc:
            self.get_logger().warn(f'Could not initialize keyboard listener: {exc}')
            return
        try:
            tty.setraw(fd)
            while rclpy.ok():
                ch = sys.stdin.read(1)
                if ch == ' ':
                    self._paused = not self._paused
                    state = 'PAUSADO' if self._paused else 'REANUDADO'
                    self.get_logger().info(f'[TECLADO] Robot {state}')
                    if self._paused:
                        self._cmd(0.0, 0.0)
                elif ch in ('q', 'Q'):
                    self.get_logger().info('[TECLADO] Saliendo...')
                    self._cmd(0.0, 0.0)
                    rclpy.shutdown()
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def _loop(self):
        if self._paused:
            return

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

        # Safety: if the EKF reports a position outside the arena, the odometry
        # has diverged (often after a long Bug0 maneuver without ArUco updates).
        # Stop and wait — the localization convergence check will resume once
        # a good ArUco correction brings the estimate back into bounds.
        xmin, ymin, xmax, ymax = self._bounds
        margin = 0.40
        if not (xmin - margin <= self.x <= xmax + margin and
                ymin - margin <= self.y <= ymax + margin):
            self._cmd(0.0, 0.0)
            self._localization_converged = False
            self.get_logger().warn(
                f'EKF position ({self.x:.2f}, {self.y:.2f}) outside arena — '
                f'waiting for localization to re-converge.'
            )
            return

        if self.wp_idx >= len(self.waypoints):
            self._cmd(0.0, 0.0)
            return

        gx, gy, gyaw = self.waypoints[self.wp_idx]
        dist_to_goal = math.hypot(gx - self.x, gy - self.y)

        if dist_to_goal < self._goal_tol:
            yaw_err = _wrap(gyaw - self.yaw)

            if abs(yaw_err) > self._wp_yaw_tol:
                self._aligning_final_yaw = True
                angular = _clamp(
                    self._final_yaw_kp * yaw_err,
                    -self._final_yaw_max_ang,
                    self._final_yaw_max_ang
                )

                self._cmd(0.0, angular)

                self.get_logger().info(
                    f'Waypoint {self.wp_idx} position reached. '
                    f'Aligning yaw: target={math.degrees(gyaw):.1f}°, '
                    f'current={math.degrees(self.yaw):.1f}°, '
                    f'error={math.degrees(yaw_err):.1f}°'
                )
                return

            self._aligning_final_yaw = False
            self.get_logger().info(
                f'Waypoint {self.wp_idx} reached with yaw '
                f'{math.degrees(self.yaw):.1f}°.'
            )

            self._publish_navigation_status('WAYPOINT_REACHED')

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
                    self._unknown_obs_frames = 0
                    self._follow_astar_path()
                    return

                self._unknown_obs_frames += 1
                if self._unknown_obs_frames < 3:
                    # Debounce: require 3 consecutive frames before triggering Bug0
                    # to filter single-frame phantom LiDAR reflections.
                    self._follow_astar_path()
                    return

                self._unknown_obs_frames = 0
                self._state = 'TANGENT_STEER'
                self._wall_entry_pos = (self.x, self.y)
                self._corner_entry_pos = None
                self._bug0_side = self._choose_bug0_side()
                obs = self._front_obstacle_point_global()
                if obs is not None:
                    self._dynamic_obstacles.append((obs[0], obs[1]))
                    self.get_logger().info(
                        f'Unknown obstacle recorded at ({obs[0]:.2f}, {obs[1]:.2f}).'
                    )
                self._d_bf = math.hypot(gx - self.x, gy - self.y)
                self._bf_traveled = 0.0
                self._prev_pos_bf = (self.x, self.y)
                return

            self._unknown_obs_frames = 0
            self._follow_astar_path()

        elif self._state == 'TANGENT_STEER':
            self._update_bf_travel()
            dist_to_goal = math.hypot(gx - self.x, gy - self.y)

            if front < OBS_THRESHOLD:
                self._state = 'TANGENT_BOUNDARY'
                self._corner_entry_pos = None
                return

            if self._path_clear_to_waypoint(gx, gy, dist_to_goal):
                self.get_logger().info('Tangent steer: path clear — returning to A*.')
                self._exit_tangent()
                return

            goal_angle = math.atan2(gy - self.y, gx - self.x)
            steer = self._best_steer_angle(gx, gy, goal_angle)
            self._steer_to_angle(steer, dist_to_goal)

        elif self._state == 'TANGENT_BOUNDARY':
            self._update_bf_travel()

            if self._distance_from_current_path_point() > REPLAN_DISTANCE:
                self.get_logger().info('Tangent boundary: drifted too far — replanning.')
                self._exit_tangent()
                return

            if self._bf_traveled > WALL_FOLLOW_MIN_DIST:
                dist_to_goal = math.hypot(gx - self.x, gy - self.y)
                if self._should_exit_boundary(gx, gy, dist_to_goal):
                    return

            self._bug0_follow_wall(front)

    def _follow_astar_path(self):
        if self.path_idx >= len(self.path):
            # The planned path is already consumed, but the robot may still be
            # slightly outside the waypoint tolerance. Instead of replanning every
            # control tick, directly approach the final global waypoint.
            gx, gy, _ = self.waypoints[self.wp_idx]
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

        # Bias toward the shorter-detour side, scaled by how off-center the
        # obstacle is. The rotation needed to align: LEFT side = (90° - obs_rel)
        # CW, RIGHT side = (obs_rel + 90°) CCW. LEFT is shorter when obs_rel > 0.
        obs = self._front_obstacle_point_global()
        if obs is not None:
            obs_rel = _wrap(math.atan2(obs[1] - self.y, obs[0] - self.x) - self.yaw)
            # Larger |obs_rel| = bigger angle advantage = stronger bias (capped at 4.0).
            angle_bias = min(2.0 + abs(obs_rel) / math.radians(45), 4.0)
            if obs_rel >= 0:    # obstacle left of forward → LEFT side shorter
                left_score += angle_bias
            else:               # obstacle right of forward → RIGHT side shorter
                right_score += angle_bias

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
            # Phase 1 — back up straight. No spin: pure clearance.
            self._cmd(-MAX_LIN * 0.4, 0.0)

        elif outer_corner and front_wide < OBS_EXIT_THRESHOLD:
            # Phase 2 — obstacle is still roughly ahead (side is empty because
            # the obstacle hasn't moved to the wall side yet). Rotate in place
            # until it arrives in the side_range zone, then phase 3 takes over.
            self._cmd(0.0, turn_away)

        elif not outer_corner and front_side < OBS_THRESHOLD:
            # Obstacle in the front-side quadrant: turn away to clear it.
            self._cmd(MAX_LIN * 0.3, turn_away * 0.7)

        elif outer_corner:
            # Real outer corner: front is clear (>= OBS_EXIT_THRESHOLD).
            # Advance to fully clear the corner, then curve toward the wall.
            if not corner_cleared:
                self._cmd(MAX_LIN * 0.6, 0.0)
            elif front_side < OBS_THRESHOLD:
                self._cmd(MAX_LIN * 0.6, 0.0)
            else:
                self._cmd(MAX_LIN * 0.5, turn_toward * 0.5)

        else:
            # Phase 3 — obstacle is on the wall side: proportional correction.
            angular = _clamp(
                wall_correction_sign * KP_WALL * err,
                -MAX_ANG,
                MAX_ANG
            )
            self._cmd(MAX_LIN * 0.8, angular)

    def _update_bf_travel(self):
        if self._prev_pos_bf is not None:
            self._bf_traveled += math.hypot(
                self.x - self._prev_pos_bf[0],
                self.y - self._prev_pos_bf[1],
            )
        self._prev_pos_bf = (self.x, self.y)

    def _exit_tangent(self):
        self._state = 'FOLLOW_PATH'
        self._wall_entry_pos = None
        self._corner_entry_pos = None
        self._bug0_side = None
        self._d_bf = float('inf')
        self._bf_traveled = 0.0
        self._prev_pos_bf = None
        self.path = []
        self.path_idx = 0

    def _find_tangent_points(self):
        if self._scan is None:
            return []
        scan = self._scan
        n = len(scan.ranges)
        offset = math.radians(SCAN_ANGLE_OFFSET_DEG)
        valid = [
            r if (math.isfinite(r) and scan.range_min < r < scan.range_max) else None
            for r in scan.ranges
        ]
        tangents = []
        for i in range(n - 1):
            ri, rj = valid[i], valid[i + 1]
            if ri is None or rj is None:
                continue
            if abs(rj - ri) < self._discontinuity_threshold:
                continue
            idx, r_near = (i, ri) if ri <= rj else (i + 1, rj)
            if r_near < self._min_tangent_dist:
                continue
            laser_angle = scan.angle_min + idx * scan.angle_increment
            local_angle = laser_angle + offset      # robot frame (corrected for mount)
            world_angle = self.yaw + local_angle    # world frame
            tx = self.x + r_near * math.cos(world_angle)
            ty = self.y + r_near * math.sin(world_angle)
            tangents.append((tx, ty))
        return tangents

    def _path_clear_to_waypoint(self, gx: float, gy: float, dist: float) -> bool:
        if self._scan is None:
            return False
        scan = self._scan
        offset = math.radians(SCAN_ANGLE_OFFSET_DEG)
        goal_angle_local = _wrap(math.atan2(gy - self.y, gx - self.x) - self.yaw)
        for i, r in enumerate(scan.ranges):
            if not (math.isfinite(r) and scan.range_min < r < scan.range_max):
                continue
            local_angle = _wrap(scan.angle_min + i * scan.angle_increment + offset)
            if abs(_wrap(local_angle - goal_angle_local)) <= self._goal_cone:
                if r < dist - self._goal_tol:
                    return False
        return True

    def _best_steer_angle(self, gx: float, gy: float, goal_angle: float) -> float:
        tangents = self._find_tangent_points()
        if not tangents:
            return goal_angle
        best_angle, best_h = goal_angle, float('inf')
        for tx, ty in tangents:
            h = math.hypot(tx - self.x, ty - self.y) + math.hypot(tx - gx, ty - gy)
            if h < best_h:
                best_h = h
                best_angle = math.atan2(ty - self.y, tx - self.x)
        return best_angle

    def _steer_to_angle(self, world_angle: float, dist: float):
        heading_err = _wrap(world_angle - self.yaw)
        angular = _clamp(KP_ANG * heading_err, -MAX_ANG, MAX_ANG)
        linear = _clamp(KP_LIN * dist, 0.0, MAX_LIN)
        if abs(heading_err) > math.radians(45):
            linear = MAX_LIN * 0.15
        self._cmd(linear, angular)

    def _should_exit_boundary(self, gx: float, gy: float, dist: float) -> bool:
        if self._path_clear_to_waypoint(gx, gy, dist):
            self.get_logger().info('Boundary exit: path to goal is clear.')
            self._exit_tangent()
            return True
        for tx, ty in self._find_tangent_points():
            d_tg = math.hypot(tx - gx, ty - gy)
            d_rt = math.hypot(tx - self.x, ty - self.y)
            if d_tg >= self._d_bf - self._bf_exit_margin:
                continue
            if d_rt < self._min_tangent_dist:
                continue
            t_angle_local = _wrap(math.atan2(ty - self.y, tx - self.x) - self.yaw)
            front_to_t = self._min_range(
                math.degrees(t_angle_local) - 10,
                math.degrees(t_angle_local) + 10,
            )
            if front_to_t > d_rt - 0.10:
                self.get_logger().info(
                    f'Boundary exit via tangent ({tx:.2f},{ty:.2f}) '
                    f'd(T,goal)={d_tg:.2f} < d_bf={self._d_bf:.2f}.'
                )
                self._exit_tangent()
                return True
        return False

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
            # The robot is physically here, so clear its own cell regardless of
            # what the occupancy grid says (usually caused by dynamic obstacle
            # inflation overlapping the robot's post-Bug0 position).
            occupancy[start[1]][start[0]] = False
            self.get_logger().warn(
                f'Start cell {start} was occupied — cleared to allow planning from current position.'
            )

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

        # Mark unknown obstacles detected at runtime by Bug0
        dyn_cells = int(math.ceil(0.30 / self._resolution))
        for ox, oy in self._dynamic_obstacles:
            cx, cy = self._world_to_grid(ox, oy)
            for gy in range(cy - dyn_cells, cy + dyn_cells + 1):
                for gx in range(cx - dyn_cells, cx + dyn_cells + 1):
                    if self._inside_grid((gx, gy)):
                        if math.hypot(gx - cx, gy - cy) <= dyn_cells:
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
        offset = math.radians(SCAN_ANGLE_OFFSET_DEG)
        a_min = _wrap(math.radians(a_min_deg) + offset)
        a_max = _wrap(math.radians(a_max_deg) + offset)

        n = len(scan.ranges)

        def _to_idx(a):
            return max(0, min(round((a - scan.angle_min) / scan.angle_increment), n - 1))

        def _valid(r):
            return math.isfinite(r) and scan.range_min < r < scan.range_max

        if a_min <= a_max:
            i0, i1 = _to_idx(a_min), _to_idx(a_max)
            vals = [r for r in scan.ranges[i0:i1 + 1] if _valid(r)]
        else:
            # El rango cruza el límite ±π
            vals = [r for r in scan.ranges[_to_idx(a_min):] + scan.ranges[:_to_idx(a_max) + 1] if _valid(r)]

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
        offset = math.radians(SCAN_ANGLE_OFFSET_DEG)
        a_min = _wrap(-self._known_obstacle_angle_window + offset)
        a_max = _wrap(self._known_obstacle_angle_window + offset)

        n = len(scan.ranges)

        def _to_idx(a):
            return max(0, min(round((a - scan.angle_min) / scan.angle_increment), n - 1))

        if a_min <= a_max:
            indices = range(_to_idx(a_min), _to_idx(a_max) + 1)
        else:
            indices = list(range(_to_idx(a_min), n)) + list(range(0, _to_idx(a_max) + 1))

        best_range = float('inf')
        best_index = None

        for idx in indices:
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

        # Ángulo en frame del láser → convertir a frame del robot con offset
        laser_angle = scan.angle_min + best_index * scan.angle_increment
        local_angle = laser_angle + math.radians(SCAN_ANGLE_OFFSET_DEG)

        local_x = best_range * math.cos(local_angle)
        local_y = best_range * math.sin(local_angle)

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
            self.get_logger().debug(
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