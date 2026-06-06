#!/usr/bin/env python3
"""
fsm_control_node.py — Mux de navegación + QR align para Puzzlebot Pallet Loader

Objetivo:
  - Dejar que hybrid_a_star_bug0_node navegue normalmente hacia waypoints.
  - Cuando qr_align detecte un QR, detener navegación autónoma.
  - Activar qr_align para que controle /cmd_vel.
  - Mantener EKF-ArUco corriendo en paralelo; este nodo no lo bloquea.

Flujo recomendado de tópicos:
  hybrid_a_star_bug0_node  -> /nav/cmd_vel
  qr_align                 -> /qr_align/cmd_vel
  fsm_control_node         -> /cmd_vel

Tópicos suscritos:
  /nav/cmd_vel             geometry_msgs/Twist
  /qr_align/cmd_vel        geometry_msgs/Twist
  /qr_align/status         std_msgs/String    SEARCHING | DETECTED | ALIGNING | DONE
  /puzzlebot/command       std_msgs/String    start | stop | reset_qr | force_qr | nav
  /aruco_measurements      std_msgs/Float32MultiArray  opcional, solo monitoreo/frescura

Tópicos publicados:
  /cmd_vel                 geometry_msgs/Twist
  /fsm/state               std_msgs/String
  /qr_align/enable         std_msgs/Bool
"""

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray, String


class FSMControlNode(Node):
    STATE_NAVIGATING = 'NAVIGATING'
    STATE_QR_ALIGN = 'QR_ALIGN'
    STATE_QR_DONE = 'QR_DONE'
    STATE_STOPPED = 'STOPPED'

    QR_DETECTED_STATUSES = {'DETECTED', 'ALIGNING'}
    QR_DONE_STATUSES = {'DONE'}

    def __init__(self):
        super().__init__('fsm_control_node')

        # ── Parameters ───────────────────────────────────────────────
        self.declare_parameter('nav_cmd_vel_topic', '/nav/cmd_vel')
        self.declare_parameter('qr_cmd_vel_topic', '/qr_align/cmd_vel')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('qr_status_topic', '/qr_align/status')
        self.declare_parameter('qr_enable_topic', '/qr_align/enable')
        self.declare_parameter('fsm_state_topic', '/fsm/state')
        self.declare_parameter('voice_command_topic', '/puzzlebot/command')
        self.declare_parameter('aruco_topic', '/aruco_measurements')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('auto_resume_after_qr_done', False)
        self.declare_parameter('qr_detection_debounce', 2)
        self.declare_parameter('qr_cmd_timeout_sec', 0.5)
        self.declare_parameter('nav_cmd_timeout_sec', 0.5)

        self.nav_cmd_vel_topic = self.get_parameter('nav_cmd_vel_topic').value
        self.qr_cmd_vel_topic = self.get_parameter('qr_cmd_vel_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.qr_status_topic = self.get_parameter('qr_status_topic').value
        self.qr_enable_topic = self.get_parameter('qr_enable_topic').value
        self.fsm_state_topic = self.get_parameter('fsm_state_topic').value
        self.voice_command_topic = self.get_parameter('voice_command_topic').value
        self.aruco_topic = self.get_parameter('aruco_topic').value
        self.auto_resume_after_qr_done = bool(self.get_parameter('auto_resume_after_qr_done').value)
        self.qr_detection_debounce = int(self.get_parameter('qr_detection_debounce').value)
        self.qr_cmd_timeout_sec = float(self.get_parameter('qr_cmd_timeout_sec').value)
        self.nav_cmd_timeout_sec = float(self.get_parameter('nav_cmd_timeout_sec').value)
        control_rate_hz = float(self.get_parameter('control_rate_hz').value)

        # ── Internal state ───────────────────────────────────────────
        self.state = self.STATE_NAVIGATING
        self.nav_twist = Twist()
        self.qr_twist = Twist()
        self.last_nav_time = 0.0
        self.last_qr_cmd_time = 0.0
        self.last_aruco_time = 0.0
        self.last_aruco_id = None
        self.qr_detection_count = 0
        self.qr_align_enabled = False

        # ── Publishers ──────────────────────────────────────────────
        self.pub_cmd_vel = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pub_state = self.create_publisher(String, self.fsm_state_topic, 10)
        self.pub_qr_enable = self.create_publisher(Bool, self.qr_enable_topic, 10)

        # ── Subscriptions ───────────────────────────────────────────
        self.create_subscription(Twist, self.nav_cmd_vel_topic, self._nav_cmd_cb, 10)
        self.create_subscription(Twist, self.qr_cmd_vel_topic, self._qr_cmd_cb, 10)
        self.create_subscription(String, self.qr_status_topic, self._qr_status_cb, 10)
        self.create_subscription(String, self.voice_command_topic, self._voice_cmd_cb, 10)
        self.create_subscription(Float32MultiArray, self.aruco_topic, self._aruco_cb, 10)

        period = 1.0 / max(control_rate_hz, 1.0)
        self.create_timer(period, self._control_loop)

        self._publish_state()
        self._set_qr_enable(False)
        self.get_logger().info(
            'FSM integrado listo. '\
            f'nav={self.nav_cmd_vel_topic} -> cmd={self.cmd_vel_topic}, '\
            f'qr={self.qr_cmd_vel_topic}, qr_status={self.qr_status_topic}'
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

    def _aruco_cb(self, msg: Float32MultiArray):
        # No bloquea ni modifica EKF. Solo monitorea que el pipeline ArUco siga vivo.
        if len(msg.data) >= 1:
            self.last_aruco_id = int(msg.data[0])
            self.last_aruco_time = time.monotonic()

    def _qr_status_cb(self, msg: String):
        status = msg.data.strip().upper()

        if status in self.QR_DETECTED_STATUSES:
            self.qr_detection_count += 1
        else:
            self.qr_detection_count = 0

        if self.state == self.STATE_NAVIGATING:
            if self.qr_detection_count >= self.qr_detection_debounce:
                self.get_logger().info('QR detectado: deteniendo navegación y activando qr_align.')
                self._transition(self.STATE_QR_ALIGN)
                self._set_qr_enable(True)
                self._publish_stop()

        elif self.state == self.STATE_QR_ALIGN:
            if status in self.QR_DONE_STATUSES:
                self.get_logger().info('qr_align reportó DONE. Robot detenido.')
                self._publish_stop()
                self._set_qr_enable(False)
                if self.auto_resume_after_qr_done:
                    self._transition(self.STATE_NAVIGATING)
                else:
                    self._transition(self.STATE_QR_DONE)

    def _voice_cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()

        if cmd == 'stop':
            self.get_logger().warn('Comando STOP recibido.')
            self._set_qr_enable(False)
            self._publish_stop()
            self._transition(self.STATE_STOPPED)

        elif cmd in ('start', 'nav', 'resume'):
            self.get_logger().info('Comando START/NAV recibido. Reanudando navegación.')
            self.qr_detection_count = 0
            self._set_qr_enable(False)
            self._transition(self.STATE_NAVIGATING)

        elif cmd in ('force_qr', 'qr'):
            self.get_logger().info('Comando FORCE_QR recibido. Activando qr_align.')
            self._publish_stop()
            self._set_qr_enable(True)
            self._transition(self.STATE_QR_ALIGN)

        elif cmd in ('reset_qr', 'reset'):
            self.get_logger().info('Reset QR recibido. Estado QR_DONE -> NAVIGATING.')
            self.qr_detection_count = 0
            self._set_qr_enable(False)
            self._transition(self.STATE_NAVIGATING)

    # ────────────────────────────────────────────────────────────────
    # Control loop
    # ────────────────────────────────────────────────────────────────
    def _control_loop(self):
        now = time.monotonic()

        if self.state == self.STATE_NAVIGATING:
            self._set_qr_enable(False)
            if now - self.last_nav_time <= self.nav_cmd_timeout_sec:
                self.pub_cmd_vel.publish(self.nav_twist)
            else:
                self.pub_cmd_vel.publish(Twist())

        elif self.state == self.STATE_QR_ALIGN:
            self._set_qr_enable(True)
            if now - self.last_qr_cmd_time <= self.qr_cmd_timeout_sec:
                self.pub_cmd_vel.publish(self.qr_twist)
            else:
                # Si qr_align deja de publicar, falla seguro: robot detenido.
                self.pub_cmd_vel.publish(Twist())

        else:
            self.pub_cmd_vel.publish(Twist())

    # ────────────────────────────────────────────────────────────────
    # Helpers
    # ────────────────────────────────────────────────────────────────
    def _set_qr_enable(self, enabled: bool):
        if enabled != self.qr_align_enabled:
            self.qr_align_enabled = enabled
            self.get_logger().info(f'qr_align enable = {enabled}')

        msg = Bool()
        msg.data = bool(enabled)
        self.pub_qr_enable.publish(msg)

    def _publish_stop(self):
        self.pub_cmd_vel.publish(Twist())

    def _transition(self, new_state: str):
        if new_state != self.state:
            self.get_logger().info(f'FSM: {self.state} -> {new_state}')
            self.state = new_state
            self._publish_state()

    def _publish_state(self):
        msg = String()
        msg.data = self.state
        self.pub_state.publish(msg)

    def destroy_node(self):
        self.get_logger().info('Cerrando FSM: enviando stop.')
        self._publish_stop()
        self._set_qr_enable(False)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FSMControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()