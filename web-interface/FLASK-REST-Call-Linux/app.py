from flask import Flask, render_template, Response, jsonify
import json
import math
import time
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import cv2

app = Flask(__name__, static_folder='templates', static_url_path='')

latest_frame        = None
latest_yolo_frame   = None   # frame anotado de YOLO (/yolo/annotated/compressed)
latest_detections   = {}
latest_voice_status = '---'
latest_fsm_state    = 'UNKNOWN'
ros_node            = None

# MCL map state
latest_map_meta = None   # dict: width, height, resolution, origin_x, origin_y
latest_map_base = None   # numpy BGR image of the occupancy grid (pre-rendered)
latest_particles = []    # list of (x, y) in world frame
latest_mcl_pose  = None  # dict: x, y, theta
latest_mcl_jpeg  = None  # JPEG bytes del último render completo

# Mapeo de estados internos de la FSM a etiquetas descriptivas para la UI
FSM_STATE_LABELS = {
    'NAV_TO_WP1':               'NAVIGATING TO WAYPOINT 1',
    'WAIT_QR_DETECTION':        'WAITING FOR QR DETECTION',
    'QR_ALIGN':                 'QR ALIGNMENT',
    'FORKLIFT_RESET_ENCODER':   'RESETTING FORKLIFT ENCODER',
    'FORKLIFT_CONVEYOR_HEIGHT': 'RAISING FORKLIFT',
    'QR_LOAD':                  'LOADING PALLET',
    'FORKLIFT_LIFT_OUT':        'LIFTING PALLET',
    'REVERSE_FROM_PALLET':      'REVERSING FROM RACK',
    'FORKLIFT_LOWER':           'LOWERING FORKLIFT',
    'NAV_TO_WP2':               'NAVIGATING TO WAYPOINT 2',
    'YOLO_SCAN':                'SCANNING FOR CLIENT (YOLO)',
    'NAV_TO_DROP':              'NAVIGATING TO DROP WAYPOINT',
    'FORKLIFT_LIFT_RACK':       'RAISING FORKLIFT TO RACK',
    'ADVANCE_STRAIGHT':         'ADVANCING TO SHELF',
    'FORKLIFT_LOWER_DROP':      'LOWERING FORKLIFT',
    'REVERSE_DROP':             'REVERSING FROM SHELF',
    'MISSION_DONE':             'MISSION DONE',
    'STOPPED':                  'MISSION STOPPED',
    'UNKNOWN':                  'INITIALIZING...',
}

# Estados de la FSM donde YOLO está activo (yolo_enable=True).
# En estos estados se muestra /yolo/annotated/compressed en la interfaz.
# En cualquier otro estado se muestra /detection/annotated/compressed
# (frame QR anotado o cámara normal via camera_bridge).
_YOLO_STATES = {
    'YOLO_SCAN',    # WP2 reached — buscando cliente con YOLO
    'NAV_TO_DROP',  # navegando al drop waypoint seleccionado por YOLO
}

_CAM_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# TRANSIENT_LOCAL para recibir el mapa aunque el subscriber se conecte tarde
_MAP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('flask_web_subscriber')

        # Frame principal: QR anotado + cámara normal via camera_bridge
        self.create_subscription(
            CompressedImage, '/detection/annotated/compressed',
            self.camera_callback, _CAM_QOS)

        # Frame anotado de YOLO — solo se muestra cuando FSM está en _YOLO_STATES
        self.create_subscription(
            CompressedImage, '/yolo/annotated/compressed',
            self.yolo_camera_callback, _CAM_QOS)

        self.create_subscription(
            OccupancyGrid, '/map',
            self.map_callback, _MAP_QOS)

        self.create_subscription(
            PoseArray, '/mcl/particles',
            self.particles_callback, 10)

        self.create_subscription(
            Odometry, '/ekf_odom',
            self.mcl_odom_callback, 10)

        self.create_subscription(
            String, '/detections',
            self.detections_callback, 10)

        self.create_subscription(
            String, '/voice/status',
            self.voice_callback, 10)

        self.create_subscription(
            String, '/fsm/state',
            self.fsm_state_callback, 10)

        self.voice_pub = self.create_publisher(String, '/voice/command', 10)

    def camera_callback(self, data):
        global latest_frame
        latest_frame = bytes(data.data)

    def yolo_camera_callback(self, data):
        global latest_yolo_frame
        latest_yolo_frame = bytes(data.data)

    def map_callback(self, data):
        global latest_map_meta, latest_map_base
        w   = data.info.width
        h   = data.info.height
        res = data.info.resolution
        ox  = data.info.origin.position.x
        oy  = data.info.origin.position.y

        grid = np.array(data.data, dtype=np.int8).reshape(h, w)

        # unknown=-1 → gray, free=0 → white, occupied>0 → black
        img = np.full((h, w, 3), 128, dtype=np.uint8)
        img[grid == 0] = [255, 255, 255]
        img[grid >  0] = [0,   0,   0  ]

        # ROS origin is bottom-left; flip so top of image = top of map
        img = cv2.flip(img, 0)

        latest_map_meta = {'width': w, 'height': h, 'resolution': res,
                           'origin_x': ox, 'origin_y': oy}
        latest_map_base = img
        _render_and_cache()

    def particles_callback(self, data):
        global latest_particles
        latest_particles = [(p.position.x, p.position.y) for p in data.poses]
        _render_and_cache()

    def mcl_odom_callback(self, data):
        global latest_mcl_pose
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        q = data.pose.pose.orientation
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        latest_mcl_pose = {'x': x, 'y': y, 'theta': theta}

    def detections_callback(self, data):
        global latest_detections
        try:
            latest_detections = json.loads(data.data)
        except json.JSONDecodeError:
            pass

    def voice_callback(self, data):
        global latest_voice_status
        latest_voice_status = data.data

    def fsm_state_callback(self, data):
        global latest_fsm_state
        latest_fsm_state = data.data

    def send_voice_command(self, command):
        msg = String()
        msg.data = command
        self.voice_pub.publish(msg)


def ros2_thread():
    global ros_node
    rclpy.init()
    ros_node = ImageSubscriber()
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()


# ── Rendering MCL ──────────────────────────────────────────────────────────────

def _make_placeholder():
    img = np.zeros((300, 640, 3), dtype=np.uint8)
    cv2.putText(img, 'Waiting for MCL map...',
                (60, 160), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (80, 80, 80), 2)
    _, buf = cv2.imencode('.jpg', img)
    return buf.tobytes()

_MCL_PLACEHOLDER = None   # se inicializa en el primer uso


def _render_and_cache():
    """Renderiza mapa + partículas + pose y guarda el JPEG en latest_mcl_jpeg."""
    global latest_mcl_jpeg
    meta = latest_map_meta
    base = latest_map_base
    if meta is None or base is None:
        return

    img = base.copy()
    h, w = img.shape[:2]
    res = meta['resolution']
    ox  = meta['origin_x']
    oy  = meta['origin_y']

    def world_to_px(wx, wy):
        px = int((wx - ox) / res)
        py = h - 1 - int((wy - oy) / res)
        return px, py

    # Partículas en rojo
    for wx, wy in latest_particles:
        px, py = world_to_px(wx, wy)
        if 0 <= px < w and 0 <= py < h:
            cv2.circle(img, (px, py), 2, (0, 0, 255), -1)

    # Pose del robot: círculo verde + flecha
    pose = latest_mcl_pose
    if pose:
        rx, ry = world_to_px(pose['x'], pose['y'])
        if 0 <= rx < w and 0 <= ry < h:
            arrow_len = max(10, int(0.3 / res))  # ~30 cm en píxeles
            dx =  int(arrow_len * math.cos(pose['theta']))
            dy = -int(arrow_len * math.sin(pose['theta']))
            cv2.circle(img, (rx, ry), 6, (0, 255, 0), -1)
            cv2.arrowedLine(img, (rx, ry), (rx + dx, ry + dy),
                            (0, 200, 0), 2, tipLength=0.4)

    # Escalar para que quepa en ~640 px de ancho
    max_w = 640
    if w > max_w:
        scale = max_w / w
        img = cv2.resize(img, (max_w, int(h * scale)),
                         interpolation=cv2.INTER_NEAREST)

    ret, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 75])
    if ret:
        latest_mcl_jpeg = buf.tobytes()


# ── Generadores de streaming ───────────────────────────────────────────────────

def gen_frames():
    while True:
        # Muestra el frame anotado de YOLO solo cuando la FSM está en un
        # estado con YOLO activo y ya llegó al menos un frame de ese topic.
        # En cualquier otro caso muestra el topic principal
        # (/detection/annotated/compressed), que puede ser el frame QR
        # anotado (cuando qr_alignP está enabled) o la cámara limpia
        # (cuando camera_bridge es el único publisher).
        if latest_fsm_state in _YOLO_STATES and latest_yolo_frame is not None:
            frame = latest_yolo_frame
        else:
            frame = latest_frame

        if frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.033)   # ~30 fps


def gen_map_frames():
    global _MCL_PLACEHOLDER
    if _MCL_PLACEHOLDER is None:
        _MCL_PLACEHOLDER = _make_placeholder()
    while True:
        frame = latest_mcl_jpeg or _MCL_PLACEHOLDER
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.1)     # 10 fps es suficiente para el mapa


# ── Rutas Flask ────────────────────────────────────────────────────────────────

@app.route('/')
def show_result():
    return render_template('result.html')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/map_feed')
def map_feed():
    return Response(gen_map_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/detections')
def get_detections():
    return jsonify(latest_detections)

@app.route('/api/voice_status')
def get_voice_status():
    return latest_voice_status

@app.route('/api/fsm_state')
def get_fsm_state():
    return FSM_STATE_LABELS.get(latest_fsm_state, latest_fsm_state)

@app.route('/api/pose')
def get_pose():
    pose = latest_mcl_pose
    if pose is None:
        return jsonify({'x': None, 'y': None, 'theta_deg': None})
    return jsonify({
        'x':         round(pose['x'], 3),
        'y':         round(pose['y'], 3),
        'theta_deg': round(math.degrees(pose['theta']), 1),
    })

@app.route('/voice/start', methods=['POST'])
def voice_start():
    if ros_node:
        ros_node.send_voice_command('start')
    return '', 204

@app.route('/voice/stop', methods=['POST'])
def voice_stop():
    if ros_node:
        ros_node.send_voice_command('stop')
    return '', 204


# ── Main ───────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    t_ros = threading.Thread(target=ros2_thread, daemon=True)
    t_ros.start()
    app.run(debug=False, port=8002, use_reloader=False, threaded=True)