import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VoiceRecognitionNode(Node):

    def __init__(self):
        super().__init__('voice_recognition')
        self.publisher_ = self.create_publisher(String, '/voice/status', 10)
        self.create_subscription(String, '/voice/command', self.command_callback, 10)
        self.get_logger().info('Nodo de reconocimiento de voz listo.')

    def command_callback(self, msg):
        if msg.data == 'start':
            status = 'RECONOCIMIENTO DE VOZ INICIADO'
        elif msg.data == 'stop':
            status = 'RECONOCIMIENTO DE VOZ PAUSADO'
        else:
            return

        out = String()
        out.data = status
        self.publisher_.publish(out)
        self.get_logger().info(status)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceRecognitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
