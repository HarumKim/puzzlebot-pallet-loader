from flask import Flask, render_template, Response, jsonify
import json
import time
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
import cv2

app = Flask(__name__, static_folder='templates', static_url_path='')

latest_frame      = None
latest_map_frame  = None
latest_detections = {}
latest_voice_status = '---'
ros_node = None

_CAM_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('flask_web_subscriber')
        self.bridge = CvBridge()

        self.create_subscription(
            CompressedImage, '/detection/annotated/compressed',
            self.camera_callback, _CAM_QOS)

        self.create_subscription(
            Image, '/localization',
            self.map_callback, 10)

        self.create_subscription(
            String, '/detections',
            self.detections_callback, 10)

        self.create_subscription(
            String, '/voice/status',
            self.voice_callback, 10)

        self.voice_pub = self.create_publisher(String, '/voice/command', 10)

    def camera_callback(self, data):
        global latest_frame
        latest_frame = bytes(data.data)

    def map_callback(self, data):
        global latest_map_frame
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        ret, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 60])
        if ret:
            latest_map_frame = buffer.tobytes()

    def detections_callback(self, data):
        global latest_detections
        try:
            latest_detections = json.loads(data.data)
        except json.JSONDecodeError:
            pass

    def voice_callback(self, data):
        global latest_voice_status
        latest_voice_status = data.data

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


# ── Generadores de streaming ──────────────────────────────────────────────────

def gen_frames():
    while True:
        frame = latest_frame
        if frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.033)   # ~30 fps


def gen_map_frames():
    while True:
        frame = latest_map_frame
        if frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.033)


# ── Rutas Flask ───────────────────────────────────────────────────────────────

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


# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    t_ros = threading.Thread(target=ros2_thread, daemon=True)
    t_ros.start()
    app.run(debug=False, port=8002, use_reloader=False, threaded=True)