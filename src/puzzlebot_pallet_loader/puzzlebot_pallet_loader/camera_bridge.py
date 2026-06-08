"""
camera_bridge.py - Republica /camera/image_raw/compressed
en /detection/annotated/compressed.
Solo actúa como fallback cuando qr_alignP no está enabled.
Como qr_alignP publica en ese topic siempre (enabled o no),
este bridge solo es relevante si qr_alignP no está corriendo.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

class CameraBridge(Node):
    def __init__(self):
        super().__init__('camera_bridge')
        self._pub = self.create_publisher(
            CompressedImage, '/detection/annotated/compressed', _QOS)
        self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed',
            self._cb, _QOS)
        self.get_logger().info(
            'Camera bridge activo: '
            '/camera/image_raw/compressed -> /detection/annotated/compressed'
        )

    def _cb(self, msg: CompressedImage):
        self._pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()