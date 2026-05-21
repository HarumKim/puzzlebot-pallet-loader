from flask import Flask, render_template, Response
import time
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64MultiArray
from cv_bridge import CvBridge
import cv2

app = Flask(__name__, static_folder='templates', static_url_path='')
result_coords = [0.0, 0.0]  # Store X and Y coordinates directly
latest_frame = None
latest_map_frame = None
latest_voice_status = '---'
ros_node = None  # referencia global para publicar desde las rutas Flask

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('flask_web_subscriber')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/image_result', self.camera_callback, 10)
        self.create_subscription(Image, '/localization', self.map_callback, 10)
        self.create_subscription(String, '/voice/status', self.voice_callback, 10)
        self.create_subscription(Float64MultiArray, '/object_position', self.position_callback, 10)
        self.voice_pub = self.create_publisher(String, '/voice/command', 10)

    def camera_callback(self, data):
        global latest_frame
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        ret, buffer = cv2.imencode('.jpg', cv_image)
        if ret:
            latest_frame = buffer.tobytes()

    def map_callback(self, data):
        global latest_map_frame
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        ret, buffer = cv2.imencode('.jpg', cv_image)
        if ret:
            latest_map_frame = buffer.tobytes()

    def position_callback(self, data):
        global result_coords
        # Ensure we have at least X and Y from the message
        if len(data.data) >= 2:
            result_coords = [data.data[0], data.data[1]]

    def voice_callback(self, data):
        global latest_voice_status
        latest_voice_status = data.data

    def send_voice_command(self, command):
        msg = String()
        msg.data = command
        self.voice_pub.publish(msg)

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
        if latest_frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + latest_frame + b'\r\n')
        time.sleep(0.03)

def gen_map_frames():
    while True:
        if latest_map_frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + latest_map_frame + b'\r\n')
        time.sleep(0.03)

if __name__ == '__main__':
    # Iniciar hilo de ROS 2
    t_ros = threading.Thread(target=ros2_thread, daemon=True)
    t_ros.start()

    @app.route('/api/result')
    def get_result():
        return f"{result_coords[0]},{result_coords[1]}"

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