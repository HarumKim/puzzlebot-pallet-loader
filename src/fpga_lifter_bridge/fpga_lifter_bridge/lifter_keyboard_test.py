#!/usr/bin/env python3

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LifterKeyboardTest(Node):
    def __init__(self):
        super().__init__('lifter_keyboard_test')

        self.pub = self.create_publisher(String, '/forklift_command', 10)

        self.current_command = 'stop'
        self.last_published_command = None

        # Publica continuamente el comando actual a 10 Hz
        self.timer = self.create_timer(0.10, self.publish_current_command)

    def set_command(self, command: str):
        if command != self.current_command:
            self.current_command = command
            self.get_logger().info(f'Keyboard command changed to: {command}')

    def publish_current_command(self):
        msg = String()
        msg.data = self.current_command
        self.pub.publish(msg)

        if self.current_command != self.last_published_command:
            self.get_logger().info(f'Publishing /forklift_command: {self.current_command}')
            self.last_published_command = self.current_command


def get_key_nonblocking():
    dr, _, _ = select.select([sys.stdin], [], [], 0.0)

    if dr:
        return sys.stdin.read(1)

    return None


def main(args=None):
    rclpy.init(args=args)
    node = LifterKeyboardTest()

    old_terminal_settings = termios.tcgetattr(sys.stdin)

    print('\nControls:')
    print('  w = lift')
    print('  s = lower')
    print('  h = hold')
    print('  SPACE = stop')
    print('  r = reset_encoder')
    print('  t = status')
    print('  q = quit')
    print('\nThe selected command is published continuously until another key is pressed.\n')

    try:
        tty.setcbreak(sys.stdin.fileno())

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

            key = get_key_nonblocking()

            if key is None:
                continue

            key = key.lower()

            if key == 'w':
                node.set_command('lift')
            elif key == 's':
                node.set_command('lower')
            elif key == 'h':
                node.set_command('hold')
            elif key == ' ':
                node.set_command('stop')
            elif key == 'r':
                node.set_command('reset_encoder')
            elif key == 't':
                node.set_command('status')
            elif key == 'q':
                node.set_command('stop')
                node.publish_current_command()
                break

    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C detected. Sending stop and closing...')
        node.set_command('stop')
        node.publish_current_command()

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_terminal_settings)

        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()