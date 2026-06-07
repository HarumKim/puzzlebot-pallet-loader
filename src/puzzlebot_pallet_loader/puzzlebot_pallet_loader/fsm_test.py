"""
fsm_test.py — Maquina de estados de prueba Mission 1.

Estados:
  WAYPOINT_1    avanza 15cm recto hacia el pallet
  QR_ALIGN      lanza qr_align_test.py, espera topic /qr_align/done
  LIFT          simula levantar pallet (imprime texto, espera 2s)
  REVERSE       retrocede 10cm
  TURN          gira 180 grados
  WAYPOINT_2    avanza 15cm de vuelta
  YOLO          lanza yolo_node.py, escucha /detections, decide lado
  DONE          fin
"""

import subprocess
import threading
import time
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool


# ══════════════════════════════════════════════════════════════════════════════
# Parametros
# ══════════════════════════════════════════════════════════════════════════════

# Rutas a los nodos — ajusta si estan en otro lugar
QR_ALIGN_PATH = 'qr_align_test.py'
YOLO_PATH     = 'yolo_node.py'

# Waypoint 1 y 2 — avance recto
WP_SPEED      = 0.04   # m/s
WP1_DIST      = 0.15   # 15 cm hacia el pallet
WP2_DIST      = 0.15   # 15 cm de vuelta

# Retroceso
REVERSE_SPEED = 0.04
REVERSE_DIST  = 0.10   # 10 cm

# Giro 180
TURN_SPEED    = 0.3    # rad/s
TURN_ANGLE    = math.pi  # 180 grados

# Pausa simulando lift
LIFT_SECS     = 2.0

# Tiempo maximo esperando que qr_align publique done
QR_TIMEOUT    = 60.0   # segundos

# Tiempo maximo esperando deteccion YOLO
YOLO_TIMEOUT  = 15.0   # segundos

# Confianza minima YOLO para considerar deteccion valida
YOLO_MIN_CONF = 0.60

# ══════════════════════════════════════════════════════════════════════════════

ST_WAYPOINT_1 = 'WAYPOINT_1'
ST_QR_ALIGN   = 'QR_ALIGN'
ST_LIFT       = 'LIFT'
ST_REVERSE    = 'REVERSE'
ST_TURN       = 'TURN'
ST_WAYPOINT_2 = 'WAYPOINT_2'
ST_YOLO       = 'YOLO'
ST_DONE       = 'DONE'


class FSMTest(Node):
    def __init__(self):
        super().__init__('fsm_test')

        self._pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Escucha señal de done del qr_align
        self._qr_done = False
        self.create_subscription(Bool, '/qr_align/done', self._qr_done_cb, 10)

        # Escucha detecciones de YOLO
        self._yolo_result = None
        self.create_subscription(String, '/detections', self._yolo_cb, 10)

        # Subprocess handles
        self._qr_proc   = None
        self._yolo_proc = None

        # FSM
        self._state      = ST_WAYPOINT_1
        self._state_start = self._now()
        self._dist_done  = 0.0
        self._angle_done = 0.0
        self._last_time  = None

        self.create_timer(1.0 / 20.0, self._loop)
        self.get_logger().info('FSM Test iniciada — estado: WAYPOINT_1')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _qr_done_cb(self, msg: Bool):
        if msg.data:
            self._qr_done = True

    def _yolo_cb(self, msg: String):
        import json
        if self._state != ST_YOLO or self._yolo_result is not None:
            return
        try:
            data = json.loads(msg.data)
            dets = data.get('detections', [])
            for d in dets:
                if d.get('confidence', 0) >= YOLO_MIN_CONF:
                    self._yolo_result = d.get('class', 'DESCONOCIDO')
                    return
        except Exception:
            pass

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _cmd(self, linear=0.0, angular=0.0):
        t = Twist()
        t.linear.x  = float(linear)
        t.angular.z = float(angular)
        self._pub.publish(t)

    def _stop(self):
        self._cmd(0.0, 0.0)

    def _set_state(self, new_state):
        self._stop()
        self._state       = new_state
        self._state_start = self._now()
        self._dist_done   = 0.0
        self._angle_done  = 0.0
        self._last_time   = None
        self.get_logger().info(f'>>> Estado: {new_state}')

    def _dt(self):
        now = self._now()
        if self._last_time is None:
            self._last_time = now
            return 0.0
        dt = now - self._last_time
        self._last_time = now
        return float(min(dt, 0.2))

    def _launch(self, executable):
        """Lanza un nodo ROS2 como subprocess con el entorno actual."""
        import os
        package = 'puzzlebot_pallet_loader'
        proc = subprocess.Popen(
            ['ros2', 'run', package, executable],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=os.environ.copy()   # hereda el entorno sourced del proceso padre
        )
        self.get_logger().info(f'Lanzado: ros2 run {package} {executable} (pid={proc.pid})')
        # Hilo para loggear stderr del subprocess en tiempo real
        def _log_stderr(p, name):
            for line in p.stderr:
                self.get_logger().warn(f'[{name}] {line.decode().strip()}')
        threading.Thread(target=_log_stderr, args=(proc, executable), daemon=True).start()
        return proc

    def _kill(self, proc, name='proceso'):
        if proc and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                proc.kill()
            self.get_logger().info(f'{name} terminado')

    # ── Loop FSM ──────────────────────────────────────────────────────────────

    def _loop(self):
        now     = self._now()
        elapsed = now - self._state_start

        # ══════════════════════════════════════════════════════════════════════
        if self._state == ST_WAYPOINT_1:
            dt = self._dt()
            self._dist_done += WP_SPEED * dt
            self._cmd(linear=WP_SPEED)

            self.get_logger().info(
                f'WP1: {self._dist_done*100:.1f}/{WP1_DIST*100:.0f}cm',
                throttle_duration_sec=0.5)

            if self._dist_done >= WP1_DIST:
                self._stop()
                # Lanza qr_align
                self._qr_done  = False
                self._qr_proc  = self._launch('qr_alignP')
                self._set_state(ST_QR_ALIGN)

        # ══════════════════════════════════════════════════════════════════════
        elif self._state == ST_QR_ALIGN:
            # Espera el topic /qr_align/done o timeout
            self.get_logger().info(
                f'Esperando QR align... {elapsed:.1f}/{QR_TIMEOUT:.0f}s',
                throttle_duration_sec=1.0)

            if self._qr_done:
                self.get_logger().info('QR align completado')
                self._kill(self._qr_proc, 'qr_align')
                self._set_state(ST_LIFT)

            elif elapsed >= QR_TIMEOUT:
                self.get_logger().warn('QR align timeout — continuando de todas formas')
                self._kill(self._qr_proc, 'qr_align')
                self._set_state(ST_LIFT)

        # ══════════════════════════════════════════════════════════════════════
        elif self._state == ST_LIFT:
            self._stop()
            self.get_logger().info(
                f'[LIFT] Levantando pallet... {elapsed:.1f}/{LIFT_SECS:.0f}s',
                throttle_duration_sec=0.5)

            # Aqui iria la llamada a la FPGA
            # self._pub_fpga.publish(...)

            if elapsed >= LIFT_SECS:
                self.get_logger().info('[LIFT] Pallet levantado')
                self._set_state(ST_REVERSE)

        # ══════════════════════════════════════════════════════════════════════
        elif self._state == ST_REVERSE:
            dt = self._dt()
            self._dist_done += REVERSE_SPEED * dt
            self._cmd(linear=-REVERSE_SPEED)

            self.get_logger().info(
                f'REVERSE: {self._dist_done*100:.1f}/{REVERSE_DIST*100:.0f}cm',
                throttle_duration_sec=0.5)

            if self._dist_done >= REVERSE_DIST:
                self._stop()
                self._set_state(ST_TURN)

        # ══════════════════════════════════════════════════════════════════════
        elif self._state == ST_TURN:
            dt = self._dt()
            self._angle_done += TURN_SPEED * dt
            self._cmd(angular=TURN_SPEED)

            pct = min(self._angle_done / TURN_ANGLE * 100, 100)
            self.get_logger().info(
                f'TURN: {math.degrees(self._angle_done):.1f}/180 deg ({pct:.0f}%)',
                throttle_duration_sec=0.5)

            if self._angle_done >= TURN_ANGLE:
                self._stop()
                self._set_state(ST_WAYPOINT_2)

        # ══════════════════════════════════════════════════════════════════════
        elif self._state == ST_WAYPOINT_2:
            dt = self._dt()
            self._dist_done += WP_SPEED * dt
            self._cmd(linear=WP_SPEED)

            self.get_logger().info(
                f'WP2: {self._dist_done*100:.1f}/{WP2_DIST*100:.0f}cm',
                throttle_duration_sec=0.5)

            if self._dist_done >= WP2_DIST:
                self._stop()
                # Lanza yolo
                self._yolo_result = None
                self._yolo_proc   = self._launch('yolo_node')
                self._set_state(ST_YOLO)

        # ══════════════════════════════════════════════════════════════════════
        elif self._state == ST_YOLO:
            self.get_logger().info(
                f'Esperando deteccion YOLO... {elapsed:.1f}/{YOLO_TIMEOUT:.0f}s',
                throttle_duration_sec=1.0)

            if self._yolo_result is not None:
                self._kill(self._yolo_proc, 'yolo_node')
                self.get_logger().info(
                    f'[YOLO] Detectado: "{self._yolo_result}" '
                    f'— el robot debe ir al lado asignado a: {self._yolo_result}'
                )
                self._set_state(ST_DONE)

            elif elapsed >= YOLO_TIMEOUT:
                self._kill(self._yolo_proc, 'yolo_node')
                self.get_logger().warn('[YOLO] Timeout — no se detecto compania')
                self._set_state(ST_DONE)

        # ══════════════════════════════════════════════════════════════════════
        elif self._state == ST_DONE:
            self._stop()
            self.get_logger().info('MISION COMPLETADA', throttle_duration_sec=2.0)

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._stop()
        self._kill(self._qr_proc,   'qr_align')
        self._kill(self._yolo_proc, 'yolo_node')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FSMTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()