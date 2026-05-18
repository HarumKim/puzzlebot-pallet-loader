import numpy as np
import cv2
from time import time
import ctypes

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

so_file = "../CPP-LIB-Linux/src/cpp_lib_demo.so"

class ObjectDetectionNode(Node):

    def __init__(self):
        super().__init__('object_detection')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/object_position', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz
        self.publisher_img = self.create_publisher(Image, '/image_result', 10)
        self.bridge = CvBridge()
        self.mylib = ctypes.CDLL(so_file)

        self.latest_frame = None
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        self.lower_green = np.array([40, 50, 50])
        self.upper_green = np.array([80, 255, 255])
        self.kernel = np.ones((5,5), np.uint8)

        self.val_in = np.array([0.0, 0.0])
        self.val_out = np.array([0.0, 0.0])

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.latest_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    def timer_callback(self):
        if self.latest_frame is None:
            return
        img = cv2.resize(self.latest_frame.copy(), (640, 480))

        noise = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.randn(noise, 0, 60)
        img = cv2.add(img, noise)

        hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask_image = cv2.inRange(hsv_image, self.lower_green, self.upper_green)

        mask_image = cv2.morphologyEx(mask_image, cv2.MORPH_OPEN, self.kernel)
        mask_image = cv2.morphologyEx(mask_image, cv2.MORPH_CLOSE, self.kernel)

        contours, _ = cv2.findContours(mask_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        max_area = 0
        max_contour = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > max_area:
                max_area = area
                max_contour = contour

        if max_contour is not None:
            x, y, w, h = cv2.boundingRect(max_contour)
            center_x = x + w / 2
            center_y = y + h / 2
            self.get_logger().info(f"Center point: ({center_x}, {center_y})")
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            self.val_in = np.array([center_x, center_y])

        self.publisher_img.publish(self.bridge.cv2_to_imgmsg(img))
        cv2.imshow('img', img)
        cv2.waitKey(1)

        res = self.mylib.Mult100(self.val_in.ctypes, self.val_in.shape[0], self.val_out.ctypes, self.val_out.shape[0])
        self.get_logger().info(f"Multiplied Center point: {self.val_out}")

        msg = Float64MultiArray()
        msg.data = [self.val_out[0], self.val_out[1], time()]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetectionNode()
    rclpy.spin(object_detection_node)

    object_detection_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
