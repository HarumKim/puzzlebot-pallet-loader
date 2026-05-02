import rclpy
import cv2  # IMPORTANTE: cv2 siempre antes de cv_bridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        timer_period = 0.1  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.br = CvBridge()
        
        # --- EL SECRETO PARA LA CÁMARA CSI EN JETSON NANO ---
        # Si la cámara se ve de cabeza en el Puzzlebot, cambia flip-method=0 a flip-method=2
        gst_pipeline = (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! "
            "nvvidconv flip-method=0 ! "
            "video/x-raw, width=640, height=480, format=BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! appsink"
        )
        
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            self.get_logger().error('¡No se pudo abrir la cámara CSI! Revisa la conexión física.')
        else:
            self.get_logger().info('Nodo de cámara CSI iniciado. Publicando imágenes...')

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn('Fallo al leer el frame de la cámara CSI.')

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
        
    camera_publisher.cap.release()
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()