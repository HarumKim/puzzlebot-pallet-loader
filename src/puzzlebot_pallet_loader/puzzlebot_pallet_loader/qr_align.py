import threading
import cv2
import numpy as np
import rclpy
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
from rclpy.node import Node
from geometry_msgs.msg import Twist


# ── Estados FSM principal ────────────────────────────────────────────────────
STATE_ALIGNING    = 'ALINEANDO'
STATE_TAKE_PALLET = 'TOMA_PALLET'
STATE_DONE        = 'DONE'

# ── Sub-estados de maniobra de perspectiva ───────────────────────────────────
PERSP_IDLE    = 0   # control normal, sin maniobra activa
PERSP_REVERSE = 1   # retrocediendo + girando para corregir ángulo
PERSP_FORWARD = 2   # avanzando de vuelta al target tras el recule


class QRAligner(Node):
    def __init__(self):
        super().__init__('qr_aligner')
        Gst.init(None)

        self.declare_parameter('cmd_vel_topic',      'cmd_vel')
        self.declare_parameter('udp_port',           5004)

        # Angular — centrado horizontal
        self.declare_parameter('kp_angular',         0.0012)
        self.declare_parameter('kd_angular',         0.0025)
        self.declare_parameter('max_angular_speed',  0.12)
        self.declare_parameter('angular_dead_zone',  10.0)   # px
        self.declare_parameter('smooth_alpha_ang',   0.20)

        # Linear — distancia por área
        self.declare_parameter('kp_linear',          2.0)
        self.declare_parameter('kd_linear',          8.0)
        self.declare_parameter('max_linear_speed',   0.07)
        self.declare_parameter('linear_dead_zone',   0.045)
        self.declare_parameter('target_area_ratio',  0.28)
        self.declare_parameter('smooth_alpha_lin',   0.25)

        # Perspectiva — perpendicularidad al QR
        self.declare_parameter('kp_persp',           0.15)
        self.declare_parameter('persp_dead_zone',    0.04)

        # Maniobra de recule para corregir perspectiva
        self.declare_parameter('persp_reverse_speed',    0.05)  # m/s hacia atrás
        self.declare_parameter('persp_reverse_distance', 0.15)  # metros a recular
        self.declare_parameter('persp_angular_speed',    0.08)  # rad/s al recular
        self.declare_parameter('persp_forward_speed',    0.06)  # m/s al volver

        # Toma de pallet — avance final tras alineación
        self.declare_parameter('take_pallet_speed',    0.04)
        self.declare_parameter('take_pallet_distance', 0.15)

        # ── Leer parámetros ───────────────────────────────────────────────────
        cmd_vel_topic       = self.get_parameter('cmd_vel_topic').value
        udp_port            = self.get_parameter('udp_port').value
        self._kp_ang        = self.get_parameter('kp_angular').value
        self._kd_ang        = self.get_parameter('kd_angular').value
        self._max_ang       = self.get_parameter('max_angular_speed').value
        self._dz_ang        = self.get_parameter('angular_dead_zone').value
        self._alpha_ang     = self.get_parameter('smooth_alpha_ang').value
        self._kp_lin        = self.get_parameter('kp_linear').value
        self._kd_lin        = self.get_parameter('kd_linear').value
        self._max_lin       = self.get_parameter('max_linear_speed').value
        self._dz_lin        = self.get_parameter('linear_dead_zone').value
        self._target_ratio  = self.get_parameter('target_area_ratio').value
        self._alpha_lin     = self.get_parameter('smooth_alpha_lin').value
        self._kp_persp      = self.get_parameter('kp_persp').value
        self._dz_persp      = self.get_parameter('persp_dead_zone').value
        self._pr_speed      = self.get_parameter('persp_reverse_speed').value
        self._pr_distance   = self.get_parameter('persp_reverse_distance').value
        self._pr_ang_speed  = self.get_parameter('persp_angular_speed').value
        self._pf_speed      = self.get_parameter('persp_forward_speed').value
        self._tp_speed      = self.get_parameter('take_pallet_speed').value
        self._tp_distance   = self.get_parameter('take_pallet_distance').value

        self._detector      = cv2.QRCodeDetector()
        self._pub           = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._latest_frame  = None
        self._lock          = threading.Lock()

        # Filtros EMA
        self._s_err_x     = 0.0
        self._s_err_area  = 0.0
        self._s_angular   = 0.0
        self._s_persp     = 0.0
        self._initialized = False

        # Derivativo
        self._prev_err_area = 0.0
        self._prev_err_x    = 0.0

        # ── FSM principal ─────────────────────────────────────────────────────
        self._state            = STATE_ALIGNING
        self._tp_distance_done = 0.0
        self._tp_last_time     = None

        # ── Sub-FSM maniobra perspectiva ──────────────────────────────────────
        self._persp_phase      = PERSP_IDLE
        self._persp_dist_done  = 0.0     # metros recorridos en fase REVERSE o FORWARD
        self._persp_last_time  = None
        self._persp_sign       = 0.0     # dirección de giro (+1 / -1) para el recule
        self._persp_fwd_target = 0.0     # distancia a recuperar en FORWARD (= lo que reculó)

        pipeline_str = (
            f"udpsrc port={udp_port} caps=\"video/mpegts, systemstream=true\" ! "
            "tsdemux latency=0 ! h264parse ! avdec_h264 ! "
            "videoconvert ! video/x-raw,format=BGR ! "
            "appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true"
        )
        self._pipeline = Gst.parse_launch(pipeline_str)
        self._appsink  = self._pipeline.get_by_name('sink')
        self._appsink.connect('new-sample', self._on_new_sample)

        ret = self._pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error('No se pudo iniciar el pipeline GStreamer!')
        else:
            self.get_logger().info(
                f'QR Aligner listo | area obj={self._target_ratio:.2f} | '
                f'recule={self._pr_distance*100:.0f}cm | '
                f'avance pallet={self._tp_distance*100:.1f}cm'
            )

        self.create_timer(1.0 / 30.0, self._process_frame)

    # ── GStreamer ─────────────────────────────────────────────────────────────

    def _on_new_sample(self, sink):
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.ERROR
        buf    = sample.get_buffer()
        caps   = sample.get_caps()
        struct = caps.get_structure(0)
        width  = struct.get_value('width')
        height = struct.get_value('height')
        ok, map_info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.ERROR
        frame = np.frombuffer(map_info.data, dtype=np.uint8).reshape(height, width, 3).copy()
        buf.unmap(map_info)
        with self._lock:
            self._latest_frame = frame
        return Gst.FlowReturn.OK

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _polygon_area(pts):
        n, area = len(pts), 0.0
        for i in range(n):
            j = (i + 1) % n
            area += pts[i, 0] * pts[j, 1]
            area -= pts[j, 0] * pts[i, 1]
        return abs(area) / 2.0

    @staticmethod
    def _perspective_error(pts):
        """
        Retorna error normalizado [-1, 1]:
          > 0  lado derecho más alto → girar derecha
          < 0  lado izquierdo más alto → girar izquierda
          ≈ 0  perpendicular
        """
        tl, tr, br, bl = pts[0], pts[1], pts[2], pts[3]
        h_left  = float(np.linalg.norm(bl.astype(float) - tl.astype(float)))
        h_right = float(np.linalg.norm(br.astype(float) - tr.astype(float)))
        denom = h_left + h_right
        if denom < 1e-6:
            return 0.0
        return (h_right - h_left) / denom

    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ── Sub-FSM: maniobra de perspectiva ─────────────────────────────────────

    def _tick_persp_maneuver(self, frame, err_persp):
        """
        Ejecuta la maniobra de recule+giro para corregir el ángulo de perspectiva.
        Se llama solo cuando _persp_phase != PERSP_IDLE.
        Retorna True cuando terminó la maniobra completa (vuelve a PERSP_IDLE).
        """
        now = self._now_sec()

        if self._persp_last_time is None:
            self._persp_last_time = now
            return False

        dt = now - self._persp_last_time
        self._persp_last_time = now

        # ── FASE REVERSE: retrocede + gira para corregir ángulo ───────────────
        if self._persp_phase == PERSP_REVERSE:
            self._persp_dist_done += self._pr_speed * dt

            linear  = -self._pr_speed                          # hacia atrás
            angular =  self._persp_sign * self._pr_ang_speed   # giro que corrige

            self._publish_cmd(linear, angular)

            pct = min(self._persp_dist_done / self._pr_distance * 100.0, 100.0)
            cv2.putText(frame, 'MANIOBRA: RECULANDO',
                        (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 165, 255), 2)
            cv2.putText(frame, f'{self._persp_dist_done*100:.1f} / {self._pr_distance*100:.0f} cm  ({pct:.0f}%)',
                        (10, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 200, 255), 1)
            self._draw_progress(frame, pct, (0, 165, 255))

            if self._persp_dist_done >= self._pr_distance:
                # Guardamos cuánto reculamos para recuperarlo en FORWARD
                self._persp_fwd_target = self._persp_dist_done
                self._persp_dist_done  = 0.0
                self._persp_phase      = PERSP_FORWARD
                self.get_logger().info('Maniobra: fin recule → avanzando de vuelta')

            return False   # maniobra no terminada aún

        # ── FASE FORWARD: vuelve hacia adelante (sin giro forzado) ───────────
        if self._persp_phase == PERSP_FORWARD:
            self._persp_dist_done += self._pf_speed * dt

            self._publish_cmd(self._pf_speed, 0.0)   # avanza recto

            pct = min(self._persp_dist_done / self._persp_fwd_target * 100.0, 100.0)
            cv2.putText(frame, 'MANIOBRA: AVANZANDO',
                        (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 220, 100), 2)
            cv2.putText(frame, f'{self._persp_dist_done*100:.1f} / {self._persp_fwd_target*100:.0f} cm  ({pct:.0f}%)',
                        (10, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 220, 100), 1)
            self._draw_progress(frame, pct, (0, 220, 100))

            if self._persp_dist_done >= self._persp_fwd_target:
                # Maniobra completa — reset y volver a control normal
                self._persp_phase     = PERSP_IDLE
                self._persp_dist_done = 0.0
                self._persp_last_time = None
                self._initialized     = False   # forzar re-init del EMA al retomar
                self.get_logger().info('Maniobra: completa → retomando alineación')
                return True   # maniobra terminada

            return False

        return True   # fallback

    @staticmethod
    def _draw_progress(frame, pct, color):
        bar_x0, bar_y0, bar_w, bar_h = 10, 82, 280, 12
        cv2.rectangle(frame, (bar_x0, bar_y0), (bar_x0 + bar_w, bar_y0 + bar_h), (50, 50, 50), -1)
        cv2.rectangle(frame, (bar_x0, bar_y0),
                      (bar_x0 + int(bar_w * pct / 100.0), bar_y0 + bar_h), color, -1)

    # ── FSM: estado TOMA_PALLET ───────────────────────────────────────────────

    def _tick_take_pallet(self, frame):
        now = self._now_sec()
        if self._tp_last_time is None:
            self._tp_last_time = now
            self._publish_cmd(self._tp_speed, 0.0)
            return False

        dt = now - self._tp_last_time
        self._tp_last_time     = now
        self._tp_distance_done += self._tp_speed * dt

        finished = self._tp_distance_done >= self._tp_distance
        self._publish_cmd(0.0 if finished else self._tp_speed, 0.0)

        pct = min(self._tp_distance_done / self._tp_distance * 100.0, 100.0)
        cv2.putText(frame, 'TOMA PALLET',
                    (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 140, 255), 2)
        cv2.putText(frame, f'{self._tp_distance_done*100:.1f} / {self._tp_distance*100:.1f} cm  ({pct:.0f}%)',
                    (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 1)
        self._draw_progress(frame, pct, (0, 200, 255))
        return finished

    # ── Loop principal ────────────────────────────────────────────────────────

    def _process_frame(self):
        with self._lock:
            if self._latest_frame is None:
                return
            frame = self._latest_frame.copy()

        frame_h, frame_w = frame.shape[:2]
        frame_area = frame_h * frame_w
        frame_cx   = frame_w / 2.0

        # ── DONE ──────────────────────────────────────────────────────────────
        if self._state == STATE_DONE:
            cv2.putText(frame, 'DONE — pallet tomado :)',
                        (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow('QR Aligner', frame)
            cv2.waitKey(1)
            return

        data, points, _ = self._detector.detectAndDecode(frame)
        cv2.line(frame, (int(frame_cx), 0), (int(frame_cx), frame_h), (80, 80, 80), 1)

        # ── TOMA_PALLET ───────────────────────────────────────────────────────
        if self._state == STATE_TAKE_PALLET:
            if self._tick_take_pallet(frame):
                self._state = STATE_DONE
                self.get_logger().info('Pallet tomado — DONE')
            cv2.imshow('QR Aligner', frame)
            cv2.waitKey(1)
            return

        # ── ALINEANDO ─────────────────────────────────────────────────────────

        # Si hay maniobra de perspectiva activa, ejecutarla (ciego, sin QR necesario)
        if self._persp_phase != PERSP_IDLE:
            # Calcular persp_err actual si hay QR visible (para el log), sino usar 0
            persp_now = 0.0
            if points is not None:
                persp_now = self._perspective_error(points[0])
            done = self._tick_persp_maneuver(frame, persp_now)
            if done:
                self._persp_phase = PERSP_IDLE
            cv2.imshow('QR Aligner', frame)
            cv2.waitKey(1)
            return

        # Control normal de alineación
        if points is not None:
            pts     = points[0].astype(int)
            qr_cx   = float(np.mean(points[0][:, 0]))
            qr_cy   = float(np.mean(points[0][:, 1]))
            qr_area = self._polygon_area(points[0])

            err_x_raw     = qr_cx - frame_cx
            area_ratio    = qr_area / frame_area
            err_area_raw  = self._target_ratio - area_ratio
            persp_err_raw = self._perspective_error(points[0])

            if not self._initialized:
                self._s_err_x       = err_x_raw
                self._s_err_area    = err_area_raw
                self._s_persp       = persp_err_raw
                self._s_angular     = 0.0
                self._prev_err_area = err_area_raw
                self._prev_err_x    = err_x_raw
                self._initialized   = True
            else:
                self._s_err_x    = self._alpha_ang * err_x_raw    + (1 - self._alpha_ang) * self._s_err_x
                self._s_err_area = self._alpha_lin * err_area_raw + (1 - self._alpha_lin) * self._s_err_area
                self._s_persp    = 0.25 * persp_err_raw + 0.75 * self._s_persp

            err_x     = self._s_err_x
            err_area  = self._s_err_area
            err_persp = self._s_persp

            d_err_area = err_area - self._prev_err_area
            d_err_x    = err_x    - self._prev_err_x
            self._prev_err_area = err_area
            self._prev_err_x    = err_x

            cx_ok    = abs(err_x)     <= self._dz_ang
            persp_ok = abs(err_persp) <= self._dz_persp
            dist_ok  = abs(err_area)  <= self._dz_lin

            # ── Detectar necesidad de maniobra ────────────────────────────────
            # Activar maniobra solo cuando ya está cerca (dist_ok o casi)
            # y la perspectiva sigue mal después de intentar corregirla en línea.
            # Condición: centrado ok, distancia ok, perspectiva NO ok → maniobrar
            if cx_ok and dist_ok and not persp_ok:
                self._persp_phase     = PERSP_REVERSE
                self._persp_dist_done = 0.0
                self._persp_last_time = None
                # Signo: si lado derecho es más alto (err_persp > 0) → girar derecha (+)
                self._persp_sign      = float(np.sign(err_persp))
                self.get_logger().info(
                    f'Iniciando maniobra recule — persp={err_persp:+.3f} sign={self._persp_sign:+.0f}'
                )
                self._publish_cmd(0.0, 0.0)
                cv2.imshow('QR Aligner', frame)
                cv2.waitKey(1)
                return

            # ── Transición a TOMA_PALLET ──────────────────────────────────────
            if cx_ok and dist_ok and persp_ok:
                self._state            = STATE_TAKE_PALLET
                self._tp_last_time     = None
                self._tp_distance_done = 0.0
                self._publish_cmd(0.0, 0.0)
                self.get_logger().info(
                    f'Alineado — iniciando TOMA_PALLET ({self._tp_distance*100:.1f} cm)'
                )
                cv2.imshow('QR Aligner', frame)
                cv2.waitKey(1)
                return

            # ── Linear PD ─────────────────────────────────────────────────────
            if abs(err_area) <= self._dz_lin:
                linear = 0.0
            else:
                pd_lin = self._kp_lin * err_area + self._kd_lin * d_err_area
                linear = float(np.clip(pd_lin, -self._max_lin, self._max_lin))

            # ── Angular: prioridad X primero, luego perspectiva ───────────────
            if cx_ok and persp_ok:
                angular_target = 0.0
            elif not cx_ok:
                # FASE 1: centrar X
                angular_target = float(np.clip(
                    -self._kp_ang * err_x - self._kd_ang * d_err_x,
                    -self._max_ang, self._max_ang
                ))
            else:
                # FASE 2: X centrado, corregir perspectiva en línea (si es leve)
                raw  = self._kp_persp * err_persp
                sign = float(np.sign(raw))
                min_cmd = 0.04   # vencer fricción estática
                if abs(raw) < min_cmd:
                    raw = sign * min_cmd
                angular_target = float(np.clip(raw, -self._max_ang, self._max_ang))

            self._s_angular = self._alpha_ang * angular_target + (1 - self._alpha_ang) * self._s_angular
            if cx_ok and persp_ok:
                self._s_angular *= 0.7
            angular = self._s_angular

            # ── Estado display ─────────────────────────────────────────────────
            if cx_ok and dist_ok and persp_ok:
                estado, color = 'ALINEADO :)', (0, 255, 0)
            elif not dist_ok and (not cx_ok or not persp_ok):
                estado, color = 'CENTRANDO + DIST', (0, 255, 255)
            elif not cx_ok:
                estado, color = 'CENTRANDO X', (0, 255, 255)
            elif not persp_ok:
                estado, color = 'CORRIGIENDO ANGULO', (0, 255, 255)
            else:
                estado, color = 'AJUSTANDO DIST', (255, 165, 0)

            # ── Visualización ─────────────────────────────────────────────────
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
            cv2.circle(frame, (int(qr_cx), int(qr_cy)), 6, (0, 0, 255), -1)
            cv2.line(frame,
                     (int(frame_cx), int(qr_cy)),
                     (int(qr_cx),    int(qr_cy)),
                     (255, 0, 255), 2)

            tl, tr, br, bl = pts[0], pts[1], pts[2], pts[3]
            cv2.line(frame, tuple(tl), tuple(bl), (255, 100, 0), 3)   # izq — naranja
            cv2.line(frame, tuple(tr), tuple(br), (0, 100, 255), 3)   # der — azul

            bar_y  = 15
            bar_ex = int(np.clip(frame_cx - err_x * 2.0, 0, frame_w - 1))
            cv2.line(frame, (int(frame_cx), bar_y), (bar_ex, bar_y), (255, 0, 255), 5)
            cv2.circle(frame, (int(frame_cx), bar_y), 6, (255, 255, 255), -1)

            if data:
                cv2.putText(frame, data, (pts[0][0], pts[0][1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.putText(frame, estado,
                        (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.putText(frame, f'err_x={err_x:+.1f}px  dz={self._dz_ang:.0f}px',
                        (10, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (200, 200, 200), 1)
            cv2.putText(frame, f'area={area_ratio:.3f}  obj={self._target_ratio:.2f}  err={err_area:+.3f}',
                        (10, 86), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (200, 200, 200), 1)
            cv2.putText(frame, f'lin={linear:+.4f}  ang={angular:+.4f}',
                        (10, 104), cv2.FONT_HERSHEY_SIMPLEX, 0.50, color, 1)
            cv2.putText(frame, f'd_area={d_err_area:+.4f}  d_x={d_err_x:+.2f}',
                        (10, 122), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)
            cv2.putText(frame, f'persp={err_persp:+.3f}  dz={self._dz_persp:.2f}  ok={persp_ok}',
                        (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)

            self._publish_cmd(linear, angular)

        else:
            self._initialized   = False
            self._s_angular     = 0.0
            self._s_persp       = 0.0
            self._prev_err_area = 0.0
            self._prev_err_x    = 0.0
            self._publish_cmd(0.0, 0.0)
            cv2.putText(frame, 'QR no detectado',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow('QR Aligner', frame)
        cv2.waitKey(1)

    # ── Publish ───────────────────────────────────────────────────────────────

    def _publish_cmd(self, linear: float, angular: float):
        twist = Twist()
        twist.linear.x  = linear
        twist.angular.z = angular
        self._pub.publish(twist)

    def destroy_node(self):
        self._pipeline.set_state(Gst.State.NULL)
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QRAligner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()