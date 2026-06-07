"""
qr_align_test.py - Aproximacion al QR con pyzbar.
Centra el QR en X y avanza hasta llegar al umbral de diagonal.
Sin SolvePnP, sin area, sin perspectiva - solo centroide.
"""

import threading
import cv2
import numpy as np
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
from pyzbar.pyzbar import decode

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool


# ══════════════════════════════════════════════════════════════════════════════
# Parametros
# ══════════════════════════════════════════════════════════════════════════════

UDP_PORT = 5004

# Deteccion / filtros
MIN_DIAG_PX = 30
MAX_DIAG_PX = 250
MAX_JUMP_PX = 80
MAX_LOST    = 20     # frames lost antes de pasar a BLIND_ADVANCE

# Control angular
KP_ANG      = 0.0010
MAX_ANG     = 0.02
DZ_ANG      = 15.0

# Control lineal - approach
LINEAR_SPEED  = 0.04
STOP_DIAG_PX  = 230   # diagonal donde termina approach y empieza carga

# Avance a ciegas (cuando pierde QR y diag aun no llego al stop)
BLIND_SPEED   = 0.02   # m/s
BLIND_SECS    = 1.0    # segundos que avanza a ciegas antes de rendirse

# Carga de pallet
LOAD_SPEED    = 0.02   # m/s
LOAD_DIST     = 0.10   # metros (10 cm)

# ══════════════════════════════════════════════════════════════════════════════

# Estados
ST_APPROACH     = 'APPROACH'
ST_BLIND        = 'BLIND'      # avanza a ciegas buscando QR
ST_LOAD         = 'LOAD'       # carga pallet 10cm
ST_DONE         = 'DONE'


def bbox_diagonal(pts):
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    w  = max(xs) - min(xs)
    h  = max(ys) - min(ys)
    return float(np.sqrt(w * w + h * h))


class QRApproach(Node):
    def __init__(self):
        super().__init__('qr_alignP')
        Gst.init(None)

        self._pub     = self.create_publisher(Twist, 'cmd_vel', 10)
        self._pub_img = self.create_publisher(
            CompressedImage, '/detection/annotated/compressed', 10)
        self._pub_done = self.create_publisher(Bool, '/qr_align/done', 10)

        pipeline_str = (
            f'udpsrc port={UDP_PORT} caps="video/mpegts, systemstream=true" ! '
            'tsdemux latency=0 ! h264parse ! avdec_h264 ! '
            'videoconvert ! video/x-raw,format=BGR ! '
            'appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true'
        )
        self._pipeline = Gst.parse_launch(pipeline_str)
        appsink        = self._pipeline.get_by_name('sink')
        appsink.connect('new-sample', self._on_new_sample)

        self._latest_frame = None
        self._lock         = threading.Lock()

        if self._pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error('No se pudo iniciar el pipeline GStreamer')

        # Last-known
        self._last_cx     = None
        self._last_cy     = None
        self._last_pts    = None
        self._lost_frames = 0

        # FSM
        self._state = ST_APPROACH

        # Timers de fase
        self._blind_start     = None   # timestamp inicio BLIND
        self._load_start      = None   # timestamp inicio LOAD
        self._load_dist_done  = 0.0
        self._load_last_time  = None

        self.create_timer(1.0 / 30.0, self._loop)
        self.get_logger().info(f'QR Approach listo - STOP_DIAG={STOP_DIAG_PX}px')

    # ── GStreamer ─────────────────────────────────────────────────────────────

    def _on_new_sample(self, sink):
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.ERROR
        buf    = sample.get_buffer()
        caps   = sample.get_caps()
        struct = caps.get_structure(0)
        w      = struct.get_value('width')
        h      = struct.get_value('height')
        ok, mi = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.ERROR
        frame = np.frombuffer(mi.data, dtype=np.uint8).reshape(h, w, 3).copy()
        buf.unmap(mi)
        with self._lock:
            self._latest_frame = frame
        return Gst.FlowReturn.OK

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _cmd(self, linear, angular):
        t = Twist()
        t.linear.x  = float(linear)
        t.angular.z = float(angular)
        self._pub.publish(t)

    def _draw_progress(self, frame, pct, color, y, label):
        fh, fw = frame.shape[:2]
        bx, bw, bh = 10, 220, 10
        cv2.rectangle(frame, (bx, y), (bx + bw, y + bh), (50, 50, 50), -1)
        cv2.rectangle(frame, (bx, y), (bx + int(bw * pct / 100), y + bh), color, -1)
        cv2.putText(frame, label, (bx, y - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, color, 1)

    # ── Loop principal ────────────────────────────────────────────────────────

    def _loop(self):
        with self._lock:
            if self._latest_frame is None:
                return
            frame = self._latest_frame.copy()

        fh, fw   = frame.shape[:2]
        frame_cx = fw / 2.0
        frame_cy = fh / 2.0

        cv2.line(frame, (int(frame_cx), 0), (int(frame_cx), fh), (60, 60, 60), 1)
        cv2.line(frame, (0, int(frame_cy)), (fw, int(frame_cy)), (60, 60, 60), 1)

        # ══════════════════════════════════════════════════════════════════════
        # DONE
        # ══════════════════════════════════════════════════════════════════════
        if self._state == ST_DONE:
            self._cmd(0.0, 0.0)
            cv2.putText(frame, 'PALLET CARGADO - DONE',
                        (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            self._publish_frame(frame)
            return

        # ══════════════════════════════════════════════════════════════════════
        # LOAD - avanza 10cm fijos sin vision
        # ══════════════════════════════════════════════════════════════════════
        if self._state == ST_LOAD:
            now = self._now()
            if self._load_last_time is None:
                self._load_last_time = now

            dt = now - self._load_last_time
            self._load_last_time  = now
            self._load_dist_done += LOAD_SPEED * dt

            pct = min(self._load_dist_done / LOAD_DIST * 100.0, 100.0)
            self._draw_progress(frame, pct, (0, 140, 255), fh - 28,
                                f'CARGA {self._load_dist_done*100:.1f}/{LOAD_DIST*100:.0f}cm ({pct:.0f}%)')

            cv2.putText(frame, 'CARGANDO PALLET',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 140, 255), 2)

            if self._load_dist_done >= LOAD_DIST:
                self._cmd(0.0, 0.0)
                self._state = ST_DONE
                self.get_logger().info('Pallet cargado - DONE')
                done_msg = Bool()
                done_msg.data = True
                self._pub_done.publish(done_msg)
            else:
                self._cmd(LOAD_SPEED, 0.0)

            self._publish_frame(frame)
            return

        # ══════════════════════════════════════════════════════════════════════
        # BLIND - avanza a ciegas buscando el QR
        # ══════════════════════════════════════════════════════════════════════
        if self._state == ST_BLIND:
            now     = self._now()
            elapsed = now - self._blind_start
            pct     = min(elapsed / BLIND_SECS * 100.0, 100.0)

            # Intenta detectar mientras avanza
            codes = decode(frame)
            if codes:
                qr   = codes[0]
                pts  = [(p.x, p.y) for p in qr.polygon]
                cx   = float(np.mean([p[0] for p in pts]))
                cy   = float(np.mean([p[1] for p in pts]))
                diag = bbox_diagonal(pts)

                if MIN_DIAG_PX <= diag <= MAX_DIAG_PX:
                    # Recupero el QR - vuelve a APPROACH
                    self._last_cx     = cx
                    self._last_cy     = cy
                    self._last_pts    = pts
                    self._lost_frames = 0
                    self._state       = ST_APPROACH
                    self.get_logger().info(f'QR recuperado en BLIND (diag={diag:.0f}px) - vuelve a APPROACH')
                    self._publish_frame(frame)
                    return

            # Tiempo agotado sin recuperar QR - para definitivamente
            if elapsed >= BLIND_SECS:
                self._cmd(0.0, 0.0)
                self.get_logger().warn('BLIND agotado - QR no recuperado, PARADO')
                cv2.putText(frame, 'BLIND AGOTADO - PARADO',
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                # Resetea para que el operador pueda reintentar
                self._blind_start = None
                self._state       = ST_APPROACH
                self._last_cx = self._last_cy = self._last_pts = None
                self._lost_frames = 0
            else:
                self._cmd(BLIND_SPEED, 0.0)
                self._draw_progress(frame, pct, (0, 180, 255), fh - 28,
                                    f'BLIND {elapsed:.1f}/{BLIND_SECS:.0f}s ({pct:.0f}%)')
                cv2.putText(frame, 'BUSCANDO QR (avance ciego)',
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 180, 255), 2)

            self._publish_frame(frame)
            return

        # ══════════════════════════════════════════════════════════════════════
        # APPROACH - deteccion + control
        # ══════════════════════════════════════════════════════════════════════
        codes = decode(frame)
        valid = False

        if codes:
            qr   = codes[0]
            pts  = [(p.x, p.y) for p in qr.polygon]
            cx   = float(np.mean([p[0] for p in pts]))
            cy   = float(np.mean([p[1] for p in pts]))
            diag = bbox_diagonal(pts)

            size_ok = MIN_DIAG_PX <= diag <= MAX_DIAG_PX
            jump_ok = (self._last_cx is None) or \
                      (np.hypot(cx - self._last_cx, cy - self._last_cy) < MAX_JUMP_PX)

            if size_ok and jump_ok:
                self._last_cx     = cx
                self._last_cy     = cy
                self._last_pts    = pts
                self._lost_frames = 0
                valid             = True
            else:
                self._lost_frames += 1
                reason = 'tamano' if not size_ok else 'salto'
                cv2.putText(frame, f'DESCARTADO ({reason})  diag={diag:.0f}px',
                            (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 80, 255), 1)
        else:
            self._lost_frames += 1

        # ── Decidir posicion ──────────────────────────────────────────────────
        using_last = False
        if valid:
            use_cx, use_cy, use_pts = self._last_cx, self._last_cy, self._last_pts
        elif self._last_cx is not None and self._lost_frames < MAX_LOST:
            use_cx, use_cy, use_pts = self._last_cx, self._last_cy, self._last_pts
            using_last = True
        else:
            use_cx = use_cy = use_pts = None

        # ── Control ───────────────────────────────────────────────────────────
        if use_pts is not None:
            diag_use = bbox_diagonal(use_pts)
            err_x    = use_cx - frame_cx

            # ¿Llegamos al stop? → inicia LOAD
            if diag_use >= STOP_DIAG_PX:
                self._cmd(0.0, 0.0)
                self._state          = ST_LOAD
                self._load_start     = self._now()
                self._load_last_time = None
                self._load_dist_done = 0.0
                self.get_logger().info(f'Stop alcanzado (diag={diag_use:.1f}px) - iniciando CARGA')
                self._publish_frame(frame)
                return

            # Angular
            angular = float(np.clip(-KP_ANG * err_x, -MAX_ANG, MAX_ANG)) \
                      if abs(err_x) > DZ_ANG else 0.0

            self._cmd(LINEAR_SPEED, angular)

            # Dibujo
            color  = (0, 180, 255) if using_last else (0, 255, 0)
            pts_np = np.array(use_pts, dtype=np.int32)
            cv2.polylines(frame, [pts_np], True, color, 2)
            cv2.circle(frame, (int(use_cx), int(use_cy)), 6, (0, 0, 255), -1)
            cv2.line(frame,
                     (int(frame_cx), int(use_cy)),
                     (int(use_cx),   int(use_cy)),
                     (255, 0, 255), 2)

            pct = min(diag_use / STOP_DIAG_PX * 100.0, 100.0)
            self._draw_progress(frame, pct, color, fh - 28,
                                f'diag {diag_use:.0f}/{STOP_DIAG_PX}px ({pct:.0f}%)')

            estado = f'LAST KNOWN ({self._lost_frames}/{MAX_LOST})' if using_last else 'DETECTADO'
            cv2.putText(frame, estado,
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
            cv2.putText(frame, f'err_x={err_x:+.1f}px   dz={DZ_ANG:.0f}px',
                        (10, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (200, 200, 200), 1)
            cv2.putText(frame, f'diag={diag_use:.1f}px',
                        (10, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (200, 200, 200), 1)

            if valid and codes:
                data = codes[0].data.decode('utf-8', errors='ignore')
                if data:
                    cv2.putText(frame, data,
                                (pts_np[0][0], pts_np[0][1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1)

        else:
            # Lost frames agotados → BLIND
            self._cmd(0.0, 0.0)
            cv2.putText(frame, 'QR PERDIDO - iniciando BLIND',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (0, 0, 255), 2)
            self._state       = ST_BLIND
            self._blind_start = self._now()
            self.get_logger().warn('QR perdido - entrando a BLIND ADVANCE')

        self._publish_frame(frame)

    def _publish_frame(self, frame):
        cv2.imshow('QR Approach', frame)
        cv2.waitKey(1)
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp  = self.get_clock().now().to_msg()
        msg.format        = 'jpeg'
        msg.data          = buf.tobytes()
        self._pub_img.publish(msg)

    def destroy_node(self):
        self._pipeline.set_state(Gst.State.NULL)
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QRApproach()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()