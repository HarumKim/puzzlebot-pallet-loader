import os
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

_DIR = os.path.dirname(os.path.abspath(__file__))

# ── TEST MODE ──────────────────────────────────────────────────────────────
# True to use test images instead of camera input.
TEST_MODE = False             # <-- pon False para modo normal
TEST_IMAGE_PATH = os.path.join(_DIR, 'apples.jpg')
TEST_MAP_IMAGE_PATH = os.path.join(_DIR, 'mapa.png')
# ──────────────────────────────────────────────────────────────────────────


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')
        self.publisher_img = self.create_publisher(CompressedImage, '/image_result/compressed', 10)
        self.publisher_map = self.create_publisher(Image, '/localization', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 Hz

        self.latest_frame = None
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.image_callback, 10)

        # ── TEST MODE: Cargar imágenes de prueba ──────────────────────────────
        test_raw = cv2.imread(TEST_IMAGE_PATH)
        self.test_frame = cv2.resize(test_raw, (640, 480)) if test_raw is not None else None

        map_raw = cv2.imread(TEST_MAP_IMAGE_PATH, cv2.IMREAD_UNCHANGED)
        if map_raw is not None:
            if map_raw.ndim == 3 and map_raw.shape[2] == 4:
                map_raw = cv2.cvtColor(map_raw, cv2.COLOR_BGRA2BGR)
            self.test_map_frame = cv2.resize(map_raw, (640, 480))
        else:
            self.test_map_frame = None
        # ─────────────────────────────────────────────────────────────────────

    def image_callback(self, msg):
        if TEST_MODE:
            return
        self.publisher_img.publish(msg)

    def timer_callback(self):
        if not TEST_MODE:
            return
        if self.test_frame is None:
            return
        img = self.test_frame.copy()
        if self.test_map_frame is not None:
            self.publisher_map.publish(self.bridge.cv2_to_imgmsg(self.test_map_frame))
        self.publisher_img.publish(self.bridge.cv2_to_imgmsg(img))


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
