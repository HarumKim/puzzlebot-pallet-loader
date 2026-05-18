from flask import Flask, render_template, Response
import requests
import json
import schedule
import time
import threading

# Nuevas importaciones para ROS 2 y procesamiento de imágenes
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

app = Flask(__name__)
result = None
latest_frame = None  # Variable global para guardar el frame más reciente

# Clase del Nodo de ROS 2 para suscribirse a la imagen
class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('flask_web_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/image_result',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, data):
        global latest_frame
        # Convertir mensaje de ROS a imagen de OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        # Codificar la imagen a formato JPEG
        ret, buffer = cv2.imencode('.jpg', cv_image)
        if ret:
            latest_frame = buffer.tobytes()

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
    rclpy.init()
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

# Función generadora para el streaming de video
def gen_frames():
    global latest_frame
    while True:
        if latest_frame is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + latest_frame + b'\r\n')
        time.sleep(0.03) # Espera pequeña para limitar el frame rate (~30 FPS)

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

    # Ruta para el streaming de video
    @app.route('/video_feed')
    def video_feed():
        return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.route('/')
    def show_result():
        return render_template('result.html')

    app.run(debug=False, port=8002, use_reloader=False)