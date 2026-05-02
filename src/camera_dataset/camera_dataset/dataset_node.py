import rclpy
import cv2  
import os
import time
import threading # Necesario para leer la terminal sin pausar ROS 2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DatasetCreator(Node):
    def __init__(self):
        super().__init__('dataset_creator')
        
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.br = CvBridge()
        self.recording = False
        self.image_count = 0
        
        self.save_dir = os.path.expanduser('~/ros2_ws/src/camera_dataset/dataset')
        os.makedirs(self.save_dir, exist_ok=True)
        
        self.last_save_time = time.time()
        self.save_interval = 0.1
        self.latest_frame = None

        self.get_logger().info(f'Nodo de dataset iniciado. Guardando en: {self.save_dir}')
        
        # Iniciamos un hilo que se queda escuchando la terminal
        self.input_thread = threading.Thread(target=self.keyboard_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def keyboard_loop(self):
        print("\n=== CONTROLES DE TERMINAL ===")
        print("Escribe la letra y presiona ENTER:")
        print(" [s] = Iniciar continua | [p] = Pausar | [c] = Tomar UNA foto | [q] = Salir")
        
        while True:
            cmd = input().strip().lower()
            
            if cmd == 's':
                self.recording = True
                print('\n>>> GRABANDO CONTINUAMENTE <<<')
            elif cmd == 'p':
                self.recording = False
                print('\n>>> PAUSADO <<<')
            elif cmd == 'c':
                if self.latest_frame is not None:
                    self.save_image(self.latest_frame)
                    print('\n>>> FOTO ÚNICA CAPTURADA <<<')
                else:
                    print('\n[Aviso] Aún no se reciben imágenes de la cámara')
            elif cmd == 'q':
                print('\nSaliendo...')
                os._exit(0) # Cierra todo el proceso limpiamente
            else:
                print('Comando no reconocido. Usa s, p, c, o q.')

    def image_callback(self, msg):
        # Guardamos el último frame recibido en una variable
        self.latest_frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        
        if self.recording:
            current_time = time.time()
            if current_time - self.last_save_time >= self.save_interval:
                self.save_image(self.latest_frame)
                self.last_save_time = current_time

    def save_image(self, frame):
        filename = os.path.join(self.save_dir, f'image_{self.image_count:05d}.jpg')
        cv2.imwrite(filename, frame)
        self.image_count += 1
        self.get_logger().info(f'Guardada: {filename}')

def main(args=None):
    rclpy.init(args=args)
    dataset_creator = DatasetCreator()
    
    try:
        rclpy.spin(dataset_creator)
    except KeyboardInterrupt:
        pass
        
    dataset_creator.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()