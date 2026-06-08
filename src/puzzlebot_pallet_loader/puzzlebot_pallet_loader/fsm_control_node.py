#!/usr/bin/env python3
"""
fsm_control_node.py — Mission FSM for Puzzlebot Pallet Loader
"""

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String, UInt8


class FSMControlNode(Node):
    # Mission states
    STATE_NAV_TO_WP1 = 'NAV_TO_WP1'
    STATE_WAIT_QR_DETECTION = 'WAIT_QR_DETECTION'
    STATE_QR_ALIGN = 'QR_ALIGN'
    STATE_FORKLIFT_RESET_ENCODER = 'FORKLIFT_RESET_ENCODER'
    STATE_FORKLIFT_CONVEYOR_HEIGHT = 'FORKLIFT_CONVEYOR_HEIGHT'
    STATE_QR_LOAD = 'QR_LOAD'
    STATE_FORKLIFT_LIFT_OUT = 'FORKLIFT_LIFT_OUT'
    STATE_FORKLIFT_LIFT_RACK = 'FORKLIFT_LIFT_RACK'
    STATE_REVERSE_FROM_PALLET = 'REVERSE_FROM_PALLET'
    STATE_FORKLIFT_LOWER = 'FORKLIFT_LOWER'
    STATE_NAV_TO_WP2 = 'NAV_TO_WP2'
    # ── CAMBIO 1: nuevos estados ──────────────────────────────────────
    STATE_YOLO_SCAN = 'YOLO_SCAN'
    STATE_NAV_TO_DROP = 'NAV_TO_DROP'
    # ─────────────────────────────────────────────────────────────────
    STATE_MISSION_DONE = 'MISSION_DONE'
    STATE_STOPPED = 'STOPPED'

    NAV_WAYPOINT_REACHED_STATUSES = {
        'WAYPOINT_REACHED', 'WP_REACHED', 'GOAL_REACHED',
        'FINAL_YAW_REACHED', 'WAYPOINT_1_REACHED',
        'WAYPOINT_2_REACHED', 'REACHED',
    }

    QR_DETECTED_STATUSES = {
        'DETECTED', 'ALIGNING', 'QR_DETECTED',
    }

    QR_DONE_STATUSES = {
        'DONE', 'ALIGNED', 'QR_DONE', 'ALIGNMENT_DONE',
    }

    def __init__(self):
        super().__init__('fsm_control_node')

        # ── Parameters ───────────────────────────────────────────────
        self.declare_parameter('nav_cmd_vel_topic', '/nav/cmd_vel')
        self.declare_parameter('qr_cmd_vel_topic', '/qr_align/cmd_vel')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('navigation_status_topic', '/navigation/status')
        self.declare_parameter('qr_status_topic', '/qr_align/status')
        self.declare_parameter('qr_enable_topic', '/qr_align/enable')
        self.declare_parameter('qr_load_enable_topic', '/qr_align/load_enable')

        self.declare_parameter('yolo_enable_topic', '/yolo/enable')

        self.declare_parameter('forklift_command_topic', '/forklift_command')
        self.declare_parameter('forklift_status_topic', '/forklift_status')
        self.declare_parameter('use_forklift_status', False)

        self.declare_parameter('fsm_state_topic', '/fsm/state')
        self.declare_parameter('mission_event_topic', '/mission/event')
        self.declare_parameter('mission_command_topic', '/puzzlebot/command')
        self.declare_parameter('aruco_topic', '/aruco_measurements')

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('qr_detection_debounce', 2)

        self.declare_parameter('qr_cmd_timeout_sec', 0.5)
        self.declare_parameter('nav_cmd_timeout_sec', 0.5)

        self.declare_parameter('forklift_reset_duration_sec', 0.50)
        self.declare_parameter('forklift_rack_command_duration_sec', 0.50)
        self.declare_parameter('forklift_lift_wait_sec', 3.00)

        self.declare_parameter('forklift_reset_command', 'reset_encoder')
        self.declare_parameter('forklift_rack_command', 'rack')
        self.declare_parameter('forklift_stop_command', 'stop')
        self.declare_parameter('forklift_conveyor_command', 'conveyor')
        self.declare_parameter('forklift_lift_command', 'lift')
        self.declare_parameter('forklift_lower_command', 'lower')
        self.declare_parameter('forklift_conveyor_wait_sec', 3.0)
        self.declare_parameter('forklift_lift_out_wait_sec', 2.0)
        self.declare_parameter('forklift_lower_wait_sec', 2.0)
        self.declare_parameter('reverse_distance_m', 0.20)
        self.declare_parameter('reverse_speed_m_s', 0.03)

        self.declare_parameter('forklift_hold_status', 3)
        self.declare_parameter('forklift_error_status', 224)

        # ── CAMBIO 2: drop waypoints del yaml ────────────────────────────
        self.declare_parameter('drop_waypoints', [0.0, 0.0, 0.0])
        self._drop_wps_flat = list(self.get_parameter('drop_waypoints').value)
        self.declare_parameter('yolo_scan_timeout_sec', 15.0)
        self._yolo_scan_timeout = float(self.get_parameter('yolo_scan_timeout_sec').value)
        # ─────────────────────────────────────────────────────────────────

        self.nav_cmd_vel_topic = self.get_parameter('nav_cmd_vel_topic').value
        self.qr_cmd_vel_topic = self.get_parameter('qr_cmd_vel_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.navigation_status_topic = self.get_parameter('navigation_status_topic').value
        self.qr_status_topic = self.get_parameter('qr_status_topic').value
        self.qr_enable_topic = self.get_parameter('qr_enable_topic').value
        self.qr_load_enable_topic = self.get_parameter('qr_load_enable_topic').value

        self.yolo_enable_topic = self.get_parameter('yolo_enable_topic').value

        self.forklift_command_topic = self.get_parameter('forklift_command_topic').value
        self.forklift_status_topic = self.get_parameter('forklift_status_topic').value
        self.use_forklift_status = bool(self.get_parameter('use_forklift_status').value)
        self.forklift_conveyor_command = str(self.get_parameter('forklift_conveyor_command').value)
        self.forklift_lift_command = str(self.get_parameter('forklift_lift_command').value)
        self.forklift_conveyor_wait_sec = float(self.get_parameter('forklift_conveyor_wait_sec').value)
        self.forklift_lift_out_wait_sec = float(self.get_parameter('forklift_lift_out_wait_sec').value)
        self.forklift_lower_command = str(self.get_parameter('forklift_lower_command').value)
        self.forklift_lower_wait_sec = float(self.get_parameter('forklift_lower_wait_sec').value)
        self.reverse_distance_m = float(self.get_parameter('reverse_distance_m').value)
        self.reverse_speed_m_s = float(self.get_parameter('reverse_speed_m_s').value)

        self.fsm_state_topic = self.get_parameter('fsm_state_topic').value
        self.mission_event_topic = self.get_parameter('mission_event_topic').value
        self.mission_command_topic = self.get_parameter('mission_command_topic').value
        self.aruco_topic = self.get_parameter('aruco_topic').value

        self.qr_detection_debounce = int(self.get_parameter('qr_detection_debounce').value)
        self.qr_cmd_timeout_sec = float(self.get_parameter('qr_cmd_timeout_sec').value)
        self.nav_cmd_timeout_sec = float(self.get_parameter('nav_cmd_timeout_sec').value)

        self.forklift_reset_duration_sec = float(self.get_parameter('forklift_reset_duration_sec').value)
        self.forklift_rack_command_duration_sec = float(self.get_parameter('forklift_rack_command_duration_sec').value)
        self.forklift_lift_wait_sec = float(self.get_parameter('forklift_lift_wait_sec').value)

        self.forklift_reset_command = str(self.get_parameter('forklift_reset_command').value)
        self.forklift_rack_command = str(self.get_parameter('forklift_rack_command').value)
        self.forklift_stop_command = str(self.get_parameter('forklift_stop_command').value)

        self.forklift_hold_status = int(self.get_parameter('forklift_hold_status').value)
        self.forklift_error_status = int(self.get_parameter('forklift_error_status').value)

        control_rate_hz = float(self.get_parameter('control_rate_hz').value)

        # ── Internal state ───────────────────────────────────────────
        self.state = self.STATE_NAV_TO_WP1

        self.nav_twist = Twist()
        self.qr_twist = Twist()
        self.last_nav_time = 0.0
        self.last_qr_cmd_time = 0.0

        self.last_aruco_time = 0.0
        self.last_aruco_id = None

        self.qr_detection_count = 0
        self.qr_align_enabled = False

        self.state_enter_time = time.monotonic()
        self.last_forklift_command = None
        self.last_forklift_status = None
        self._qr_data_value = None

        # ── CAMBIO 3: estado YOLO ────────────────────────────────────────
        self._yolo_confirmed = None
        # ─────────────────────────────────────────────────────────────────

        # ── Publishers ──────────────────────────────────────────────
        self.pub_cmd_vel = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pub_state = self.create_publisher(String, self.fsm_state_topic, 10)
        self.pub_event = self.create_publisher(String, self.mission_event_topic, 10)
        self.pub_qr_enable = self.create_publisher(Bool, self.qr_enable_topic, 10)
        self.pub_qr_load_enable = self.create_publisher(Bool, self.qr_load_enable_topic, 10)
        
        self.pub_yolo_enable = self.create_publisher(Bool, self.yolo_enable_topic, 10)

        self.pub_forklift_command = self.create_publisher(String, self.forklift_command_topic, 10)
        self.pub_qr_data = self.create_publisher(String, '/qr_data', 10)
        # ── CAMBIO 4: publisher para goal dinamico ───────────────────────
        self.pub_nav_goal = self.create_publisher(Float32MultiArray, '/navigation/set_goal', 10)
        # ─────────────────────────────────────────────────────────────────

        # ── Subscriptions ───────────────────────────────────────────
        self.create_subscription(Twist, self.nav_cmd_vel_topic, self._nav_cmd_cb, 10)
        self.create_subscription(Twist, self.qr_cmd_vel_topic, self._qr_cmd_cb, 10)
        self.create_subscription(String, self.navigation_status_topic, self._navigation_status_cb, 10)
        self.create_subscription(String, self.qr_status_topic, self._qr_status_cb, 10)
        self.create_subscription(String, self.mission_command_topic, self._mission_cmd_cb, 10)
        self.create_subscription(String, '/qr_data', self._qr_data_cb, 10)
        self.create_subscription(Float32MultiArray, self.aruco_topic, self._aruco_cb, 10)
        self.create_subscription(UInt8, self.forklift_status_topic, self._forklift_status_cb, 10)
        # ── CAMBIO 5: suscripcion a YOLO target ─────────────────────────
        self.create_subscription(String, '/yolo/target', self._yolo_target_cb, 10)
        # ─────────────────────────────────────────────────────────────────

        period = 1.0 / max(control_rate_hz, 1.0)
        self.create_timer(period, self._control_loop)

        self._publish_state()
        self._publish_event('MISSION_STARTED_NAV_TO_WP1')
        self._set_qr_enable(False)
        self._set_qr_load_enable(False)
        self._set_yolo_enable(False)
        self._publish_forklift_command_once(self.forklift_stop_command)

        self.get_logger().info(
            'Mission FSM ready. '
            f'nav={self.nav_cmd_vel_topic} -> cmd={self.cmd_vel_topic}, '
            f'qr={self.qr_cmd_vel_topic}, qr_status={self.qr_status_topic}, '
            f'nav_status={self.navigation_status_topic}, '
            f'forklift_cmd={self.forklift_command_topic}, '
            f'drop_waypoints={len(self._drop_wps_flat)//3} puntos'
        )

    # ────────────────────────────────────────────────────────────────
    # Callbacks
    # ────────────────────────────────────────────────────────────────
    def _nav_cmd_cb(self, msg: Twist):
        self.nav_twist = msg
        self.last_nav_time = time.monotonic()

    def _qr_cmd_cb(self, msg: Twist):
        self.qr_twist = msg
        self.last_qr_cmd_time = time.monotonic()

    def _qr_data_cb(self, msg: String):
        value = msg.data.strip()
        if value and value != self._qr_data_value:
            self._qr_data_value = value
            self.get_logger().info(f'QR data recibido: {value}')

    # ── CAMBIO 6: callback YOLO target ───────────────────────────────
    def _yolo_target_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._yolo_confirmed = data
        except Exception:
            pass
    # ─────────────────────────────────────────────────────────────────

    def _set_qr_load_enable(self, enabled: bool):
        msg = Bool()
        msg.data = bool(enabled)
        self.pub_qr_load_enable.publish(msg)

    def _set_yolo_enable(self, enabled: bool):
        msg = Bool()
        msg.data = bool(enabled)
        self.pub_yolo_enable.publish(msg)

    def _aruco_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 1:
            self.last_aruco_id = int(msg.data[0])
            self.last_aruco_time = time.monotonic()

    def _forklift_status_cb(self, msg: UInt8):
        self.last_forklift_status = int(msg.data)
        if self.last_forklift_status == self.forklift_error_status:
            self.get_logger().error(
                f'Forklift reported ERROR status: 0x{self.last_forklift_status:02X}')
            self._publish_stop()
            self._transition(self.STATE_STOPPED, 'FORKLIFT_ERROR')

    def _navigation_status_cb(self, msg: String):
        status = msg.data.strip().upper()

        if status not in self.NAV_WAYPOINT_REACHED_STATUSES:
            self.get_logger().debug(f'Navigation status ignored: {status}')
            return

        if self.state == self.STATE_NAV_TO_WP1:
            self.get_logger().info(f'WP1 reached: {status}')
            self._publish_stop()
            self._set_qr_enable(True)
            self.qr_detection_count = 0
            self._transition(self.STATE_WAIT_QR_DETECTION, 'WP1_REACHED_WAITING_QR')

        # ── CAMBIO 7: WP2 → YOLO_SCAN en vez de MISSION_DONE ────────────
        elif self.state == self.STATE_NAV_TO_WP2:
            self.get_logger().info(f'WP2 reached: {status}')
            self._publish_stop()

            # QR ya no debe usar cámara después de recoger pallet.
            self._set_qr_enable(False)
            self._set_qr_load_enable(False)

            # YOLO se activa solo después de llegar y orientar en WP2.
            self._set_yolo_enable(True)

            self._yolo_confirmed = None
            self._transition(self.STATE_YOLO_SCAN, 'WP2_REACHED_YOLO_SCAN')

        elif self.state == self.STATE_NAV_TO_DROP:
            self.get_logger().info(f'Drop waypoint reached: {status}')
            self._publish_stop()
            self._publish_forklift_command_once(self.forklift_stop_command)
            self._transition(self.STATE_MISSION_DONE, 'DROP_REACHED_MISSION_DONE')
        # ─────────────────────────────────────────────────────────────────

        else:
            self.get_logger().info(
                f'Navigation reached in {self.state}, ignored: {status}')

    def _qr_status_cb(self, msg: String):
        status = msg.data.strip().upper()

        if self.state == self.STATE_WAIT_QR_DETECTION:
            if status in self.QR_DETECTED_STATUSES:
                self.qr_detection_count += 1
                self.get_logger().info(
                    f'QR detected debounce {self.qr_detection_count}/{self.qr_detection_debounce}')
                if self.qr_detection_count >= self.qr_detection_debounce:
                    self.get_logger().info('QR accepted. Enabling qr_align.')
                    self._publish_stop()
                    self._set_qr_enable(True)
                    self._transition(self.STATE_QR_ALIGN, 'QR_ALIGN_STARTED')
            else:
                self.qr_detection_count = 0

        elif self.state == self.STATE_QR_ALIGN:
            if status == 'READY_TO_LOAD':
                self.get_logger().info('qr_alignP requested LOAD preparation.')
                self._publish_stop()
                self._set_qr_load_enable(False)
                self._transition(self.STATE_FORKLIFT_RESET_ENCODER,
                                 'QR_READY_TO_LOAD_RESETTING_ENCODER')
            elif status in self.QR_DONE_STATUSES:
                self.get_logger().info('qr_alignP reported DONE while in QR_ALIGN.')
                self._publish_stop()
                self._set_qr_enable(False)
                self._set_qr_load_enable(False)

        elif self.state == self.STATE_QR_LOAD:
            if status in ('LOAD_DONE', 'DONE'):
                self.get_logger().info('qr_alignP completed LOAD distance.')
                self._publish_stop()
                self._set_qr_enable(False)
                self._set_qr_load_enable(False)
                self._transition(self.STATE_FORKLIFT_LIFT_OUT,
                                 'LOAD_DONE_LIFTING_OUT_PALLET')
        else:
            self.get_logger().debug(f'QR status {status} ignored in {self.state}')

    def _mission_cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()

        if cmd == 'stop':
            self.get_logger().warn('Mission command STOP received.')
            self._set_qr_enable(False)
            self._set_qr_load_enable(False)
            self._publish_stop()
            self._publish_forklift_command_once(self.forklift_stop_command)
            self._transition(self.STATE_STOPPED, 'MISSION_STOPPED')

        elif cmd in ('start', 'start_mission', 'restart'):
            self.get_logger().info('Mission command START received.')
            self._reset_mission()

        elif cmd in ('resume', 'nav', 'continue'):
            self.get_logger().info('Mission command RESUME/NAV received.')
            self._set_qr_enable(False)
            self._set_qr_load_enable(False)
            if self.state == self.STATE_STOPPED:
                self._transition(self.STATE_NAV_TO_WP1, 'RESUME_TO_WP1')
            elif self.state == self.STATE_MISSION_DONE:
                self._reset_mission()
            else:
                self._transition(self.STATE_NAV_TO_WP2, 'FORCED_RESUME_NAV_TO_WP2')

        elif cmd in ('wp1_reached', 'waypoint1_reached', 'final_yaw_reached'):
            fake = String()
            fake.data = 'WAYPOINT_REACHED'
            self._navigation_status_cb(fake)

        elif cmd in ('wp2_reached', 'waypoint2_reached', 'mission_done'):
            if self.state != self.STATE_NAV_TO_WP2:
                self._transition(self.STATE_NAV_TO_WP2, 'FORCE_NAV_TO_WP2_FOR_TEST')
            fake = String()
            fake.data = 'WAYPOINT_REACHED'
            self._navigation_status_cb(fake)

        elif cmd in ('force_qr', 'qr'):
            self.get_logger().info('Mission command FORCE_QR received.')
            self._publish_stop()
            self._set_qr_enable(True)
            self._transition(self.STATE_QR_ALIGN, 'QR_FORCED')

        elif cmd in ('force_forklift', 'forklift'):
            self.get_logger().info('Mission command FORCE_FORKLIFT received.')
            self._publish_stop()
            self._set_qr_enable(False)
            self._transition(self.STATE_FORKLIFT_RESET_ENCODER, 'FORKLIFT_FORCED')

        elif cmd in ('skip_forklift', 'skip_lift'):
            self.get_logger().warn('Mission command SKIP_FORKLIFT received.')
            self._publish_stop()
            self._set_qr_enable(False)
            self._set_qr_load_enable(False)
            self._transition(self.STATE_NAV_TO_WP2, 'FORKLIFT_SKIPPED_NAV_TO_WP2')

        elif cmd in ('reset_qr', 'reset'):
            self.get_logger().info('Mission command RESET_QR received.')
            self.qr_detection_count = 0
            self._set_qr_enable(False)
            self._set_qr_load_enable(False)
            if self.state == self.STATE_QR_ALIGN:
                self._transition(self.STATE_WAIT_QR_DETECTION, 'QR_RESET_WAITING')
            else:
                self._publish_state()

        else:
            self.get_logger().warn(f'Unknown mission command ignored: {cmd}')

    # ────────────────────────────────────────────────────────────────
    # Control loop
    # ────────────────────────────────────────────────────────────────
    def _control_loop(self):
        now = time.monotonic()

        if self.state in (self.STATE_NAV_TO_WP1, self.STATE_NAV_TO_WP2):
            self._set_qr_enable(False)
            self._set_qr_load_enable(False)
            self._set_yolo_enable(False)
            if now - self.last_nav_time <= self.nav_cmd_timeout_sec:
                self.pub_cmd_vel.publish(self.nav_twist)
            else:
                self.pub_cmd_vel.publish(Twist())

        elif self.state == self.STATE_WAIT_QR_DETECTION:
            self._set_qr_enable(True)
            self._set_yolo_enable(False)
            self.pub_cmd_vel.publish(Twist())

        elif self.state == self.STATE_QR_ALIGN:
            self._set_qr_enable(True)
            self._set_yolo_enable(False)
            if now - self.last_qr_cmd_time <= self.qr_cmd_timeout_sec:
                self.pub_cmd_vel.publish(self.qr_twist)
            else:
                self.pub_cmd_vel.publish(Twist())

        elif self.state == self.STATE_FORKLIFT_RESET_ENCODER:
            self.pub_cmd_vel.publish(Twist())
            self._set_qr_enable(True)
            self._set_qr_load_enable(False)
            self._set_yolo_enable(False)
            elapsed = now - self.state_enter_time
            if elapsed <= self.forklift_reset_duration_sec:
                self._publish_forklift_command_once(self.forklift_reset_command)
            else:
                self._transition(self.STATE_FORKLIFT_CONVEYOR_HEIGHT,
                                 'ENCODER_RESET_DONE_CONVEYOR_HEIGHT')

        elif self.state == self.STATE_FORKLIFT_CONVEYOR_HEIGHT:
            self.pub_cmd_vel.publish(Twist())
            self._set_qr_enable(True)
            self._set_qr_load_enable(False)
            self._set_yolo_enable(False)
            elapsed = now - self.state_enter_time
            if elapsed <= self.forklift_rack_command_duration_sec:
                self._publish_forklift_command_once(self.forklift_conveyor_command)
            if elapsed >= self.forklift_conveyor_wait_sec:
                self._publish_forklift_command_once(self.forklift_stop_command)
                self._set_qr_load_enable(True)
                self._transition(self.STATE_QR_LOAD,
                                 'CONVEYOR_HEIGHT_DONE_QR_LOAD_ENABLED')

        elif self.state == self.STATE_QR_LOAD:
            self._set_qr_enable(True)
            self._set_qr_load_enable(True)
            self._set_yolo_enable(False)
            if now - self.last_qr_cmd_time <= self.qr_cmd_timeout_sec:
                self.pub_cmd_vel.publish(self.qr_twist)
            else:
                self.pub_cmd_vel.publish(Twist())

        elif self.state == self.STATE_FORKLIFT_LIFT_OUT:
            self.pub_cmd_vel.publish(Twist())
            self._set_qr_enable(False)
            self._set_qr_load_enable(False)
            self._set_yolo_enable(False)
            elapsed = now - self.state_enter_time
            if elapsed <= self.forklift_rack_command_duration_sec:
                self._publish_forklift_command_once(self.forklift_lift_command)
            if elapsed >= self.forklift_lift_out_wait_sec:
                self._publish_forklift_command_once(self.forklift_stop_command)
                self._transition(self.STATE_REVERSE_FROM_PALLET,
                                 'PALLET_LIFTED_REVERSING')

        elif self.state == self.STATE_REVERSE_FROM_PALLET:
            self._set_qr_enable(False)
            self._set_qr_load_enable(False)
            self._set_yolo_enable(False)
            reverse_duration = self.reverse_distance_m / self.reverse_speed_m_s
            elapsed = now - self.state_enter_time
            if elapsed < reverse_duration:
                reverse_cmd = Twist()
                reverse_cmd.linear.x = -self.reverse_speed_m_s
                self.pub_cmd_vel.publish(reverse_cmd)
            else:
                self._publish_stop()
                self._transition(self.STATE_FORKLIFT_LOWER,
                                 'REVERSE_DONE_LOWERING_FORKLIFT')

        elif self.state == self.STATE_FORKLIFT_LOWER:
            self.pub_cmd_vel.publish(Twist())
            self._set_qr_enable(False)
            self._set_qr_load_enable(False)
            self._set_yolo_enable(False)
            elapsed = now - self.state_enter_time
            if elapsed <= self.forklift_rack_command_duration_sec:
                self._publish_forklift_command_once(self.forklift_lower_command)
            if elapsed >= self.forklift_lower_wait_sec:
                self._publish_forklift_command_once(self.forklift_stop_command)
                self._transition(self.STATE_NAV_TO_WP2,
                                 'PALLET_LIFTED_NAV_TO_WP2')

        # ── CAMBIO 8: YOLO_SCAN — espera confirmacion de YOLO ───────────
        elif self.state == self.STATE_YOLO_SCAN:
            self.pub_cmd_vel.publish(Twist())

            # En WP2, QR ya no debe usar cámara.
            self._set_qr_enable(False)
            self._set_qr_load_enable(False)

            # Aquí sí prendemos YOLO.
            self._set_yolo_enable(True)

            elapsed = now - self.state_enter_time

            # Republica /qr_data para que yolo_node lo reciba
            if self._qr_data_value:
                qr_msg = String()
                qr_msg.data = self._qr_data_value
                self.pub_qr_data.publish(qr_msg)

            if self._yolo_confirmed:
                wp_name = self._yolo_confirmed.get('waypoint', '')
                target  = self._yolo_confirmed.get('target', '?')
                try:
                    wp_idx = int(wp_name.split('_')[-1]) - 1
                except (ValueError, IndexError):
                    wp_idx = 0

                base = wp_idx * 3
                if base + 2 < len(self._drop_wps_flat):
                    dx   = self._drop_wps_flat[base]
                    dy   = self._drop_wps_flat[base + 1]
                    dyaw = self._drop_wps_flat[base + 2]

                    goal_msg = Float32MultiArray()
                    goal_msg.data = [float(dx), float(dy), float(dyaw)]
                    self.pub_nav_goal.publish(goal_msg)

                    self.get_logger().info(
                        f'YOLO confirmo: QR={self._qr_data_value} -> '
                        f'{target} -> {wp_name} ({dx:.2f}, {dy:.2f}, {dyaw:.0f}deg)')
                    self._set_yolo_enable(False)
                    self._transition(self.STATE_NAV_TO_DROP, f'NAV_TO_{wp_name}')
                else:
                    self.get_logger().error(f'Drop WP index {wp_idx} fuera de rango')
                    self._transition(self.STATE_MISSION_DONE, 'DROP_WP_ERROR')

            elif elapsed >= self._yolo_scan_timeout:
                self.get_logger().warn('YOLO scan timeout — no se confirmo cliente')
                self._set_yolo_enable(False)
                self._transition(self.STATE_MISSION_DONE, 'YOLO_TIMEOUT')

        # ── CAMBIO 9: NAV_TO_DROP — navega al drop waypoint ─────────────
        elif self.state == self.STATE_NAV_TO_DROP:
            self._set_qr_enable(False)
            self._set_qr_load_enable(False)
            if now - self.last_nav_time <= self.nav_cmd_timeout_sec:
                self.pub_cmd_vel.publish(self.nav_twist)
            else:
                self.pub_cmd_vel.publish(Twist())
        # ─────────────────────────────────────────────────────────────────

        elif self.state in (self.STATE_MISSION_DONE, self.STATE_STOPPED):
            self._set_qr_enable(False)
            self._set_qr_load_enable(False)
            self._set_yolo_enable(False)
            self.pub_cmd_vel.publish(Twist())

        else:
            self.get_logger().error(f'Unknown FSM state: {self.state}. Stopping.')
            self._publish_stop()
            self._transition(self.STATE_STOPPED, 'UNKNOWN_STATE_STOPPED')

    # ────────────────────────────────────────────────────────────────
    # Helpers
    # ────────────────────────────────────────────────────────────────
    def _reset_mission(self):
        self.qr_detection_count = 0
        self.last_forklift_status = None
        self._yolo_confirmed = None
        self._qr_data_value = None
        self._set_qr_enable(False)
        self._publish_stop()
        self._publish_forklift_command_once(self.forklift_stop_command)
        self._transition(self.STATE_NAV_TO_WP1, 'MISSION_RESET_NAV_TO_WP1')
        self._set_qr_load_enable(False)

    def _set_qr_enable(self, enabled: bool):
        if enabled != self.qr_align_enabled:
            self.qr_align_enabled = enabled
            self.get_logger().info(f'qr_align enable = {enabled}')
        msg = Bool()
        msg.data = bool(enabled)
        self.pub_qr_enable.publish(msg)

    def _publish_stop(self):
        self.pub_cmd_vel.publish(Twist())

    def _publish_forklift_command_once(self, command: str):
        msg = String()
        msg.data = str(command)
        self.pub_forklift_command.publish(msg)
        if command != self.last_forklift_command:
            self.last_forklift_command = command
            self.get_logger().info(f'Forklift command: {command}')

    def _transition(self, new_state: str, event: str = ''):
        if new_state != self.state:
            old_state = self.state
            self.state = new_state
            self.state_enter_time = time.monotonic()
            self.last_forklift_command = None
            self.get_logger().info(f'FSM: {old_state} -> {new_state}')
            self._publish_state()
            if event:
                self._publish_event(event)

    def _publish_state(self):
        msg = String()
        msg.data = self.state
        self.pub_state.publish(msg)

    def _publish_event(self, event: str):
        msg = String()
        msg.data = str(event)
        self.pub_event.publish(msg)
        self.get_logger().info(f'Mission event: {event}')

    def destroy_node(self):
        try:
            self._publish_stop()
            self._set_qr_enable(False)
            self._publish_forklift_command_once(self.forklift_stop_command)
            self._set_qr_load_enable(False)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FSMControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        pass
    finally:
        try:
            node._publish_stop()
            node._set_qr_enable(False)
            node._set_qr_load_enable(False)
            node._publish_forklift_command_once(node.forklift_stop_command)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()