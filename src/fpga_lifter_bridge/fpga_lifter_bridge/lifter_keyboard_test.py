import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class LifterKeyboardTest(Node):
    def __init__(self):
        super().__init__('lifter_keyboard_test')
        self.pub = self.create_publisher(Bool, '/lifter', 10)

    def send_state(self, state):
        msg = Bool()
        msg.data = state
        self.pub.publish(msg)

        if state:
            self.get_logger().info('Published /lifter: ON')
        else:
            self.get_logger().info('Published /lifter: OFF')


def main(args=None):
    rclpy.init(args=args)
    node = LifterKeyboardTest()

    print('\nControls:')
    print('  o = ON')
    print('  f = OFF')
    print('  q = quit\n')

    try:
        while rclpy.ok():
            key = input('Command [o/f/q]: ').strip().lower()

            if key == 'o':
                node.send_state(True)
            elif key == 'f':
                node.send_state(False)
            elif key == 'q':
                break
            else:
                print('Invalid command')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()