from flask import Flask, render_template, Response
import requests
import json
import schedule
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
result = None
latest_frame = None
latest_map_frame = None
latest_voice_status = '---'
ros_node = None  # referencia global para publicar desde las rutas Flask
_frame_event = threading.Event()
_map_event = threading.Event()

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('flask_web_subscriber')
        self.bridge = CvBridge()

        # QoS: best-effort, cola de 1 → siempre el frame más reciente, sin backlog
        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(CompressedImage, '/camera/image_raw/compressed',
                                 self.camera_callback, cam_qos)
        self.create_subscription(Image, '/localization', self.map_callback, 10)
        self.create_subscription(String, '/voice/status', self.voice_callback, 10)
        self.voice_pub = self.create_publisher(String, '/voice/command', 10)

    def camera_callback(self, data):
        global latest_frame
        # data.data ya son bytes JPEG — no hay que re-codificar
        latest_frame = bytes(data.data)
        _frame_event.set()

    def map_callback(self, data):
        global latest_map_frame
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        ret, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 60])
        if ret:
            latest_map_frame = buffer.tobytes()
            _map_event.set()

    def voice_callback(self, data):
        global latest_voice_status
        latest_voice_status = data.data

    def send_voice_command(self, command):
        msg = String()
        msg.data = command
        self.voice_pub.publish(msg)

def call_api():
    global result
    url = 'http://127.0.0.1:8042/restgatewaydemo/getmultcoords'
    data = {}
    headers = {'Content-type': 'application/json'}

    try:
        response = requests.post(url, data=json.dumps(data), headers=headers)
        result = response.json()
        result["values"].pop()
        with open('result.json', 'w') as f:
            json.dump(result, f)
    except Exception as e:
        print("Error en API:", e)

def schedule_api_call():
    while True:
        schedule.run_pending()
        time.sleep(1)

# Hilo dedicado para correr el nodo de ROS 2
def ros2_thread():
    global ros_node
    rclpy.init()
    ros_node = ImageSubscriber()
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()

def gen_frames():
    while True:
        _frame_event.wait()
        _frame_event.clear()
        frame = latest_frame
        if frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

def gen_map_frames():
    while True:
        _map_event.wait()
        _map_event.clear()
        frame = latest_map_frame
        if frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

if __name__ == '__main__':
    call_api()  # Llamada inicial
    schedule.every(0.5).seconds.do(call_api)

    # Iniciar hilo de consultas REST
    t_api = threading.Thread(target=schedule_api_call, daemon=True)
    t_api.start()

    # Iniciar hilo de ROS 2
    t_ros = threading.Thread(target=ros2_thread, daemon=True)
    t_ros.start()

    @app.route('/api/result')
    def get_result():
        if result and "values" in result and len(result["values"]) >= 2:
            return str(result["values"][0]) + "," + str(result["values"][1])
        return "0.0,0.0"

    @app.route('/video_feed')
    def video_feed():
        return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.route('/map_feed')
    def map_feed():
        return Response(gen_map_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

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

    @app.route('/api/voice_status')
    def get_voice_status():
        return latest_voice_status

    @app.route('/')
    def show_result():
        return render_template('result.html')

    app.run(debug=False, port=8002, use_reloader=False)