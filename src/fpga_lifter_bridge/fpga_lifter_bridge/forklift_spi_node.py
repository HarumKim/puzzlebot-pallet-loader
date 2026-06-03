#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8

try:
    import spidev
except ImportError:
    spidev = None


class ForkliftSpiNode(Node):
    def __init__(self):
        super().__init__('forklift_spi_node')

        self.declare_parameter('spi_bus', 0)
        self.declare_parameter('spi_device', 0)
        self.declare_parameter('spi_speed_hz', 1_000_000)

        self.spi_bus = self.get_parameter('spi_bus').value
        self.spi_device = self.get_parameter('spi_device').value
        self.spi_speed_hz = self.get_parameter('spi_speed_hz').value

        self.command_sub = self.create_subscription(
            String,
            '/forklift_command',
            self.command_callback,
            10
        )

        self.status_pub = self.create_publisher(
            UInt8,
            '/forklift_status',
            10
        )

        self.timer = self.create_timer(0.2, self.request_status)

        self.cmd_map = {
            'stop': 0x00,
            'lower': 0x01,
            'lift': 0x02,
            'hold': 0x03,
            'reset_encoder': 0x04,
            'status': 0x10,
        }

        self.spi = None
        self.init_spi()

    def init_spi(self):
        if spidev is None:
            self.get_logger().error(
                'spidev is not installed. Install it with: pip3 install spidev'
            )
            return

        try:
            self.spi = spidev.SpiDev()
            self.spi.open(self.spi_bus, self.spi_device)
            self.spi.max_speed_hz = self.spi_speed_hz
            self.spi.mode = 0b00
            self.spi.bits_per_word = 8

            self.get_logger().info(
                f'SPI initialized on /dev/spidev{self.spi_bus}.{self.spi_device} '
                f'at {self.spi_speed_hz} Hz'
            )

        except Exception as e:
            self.get_logger().error(f'Could not initialize SPI: {e}')
            self.spi = None

    def transfer_byte(self, byte_value: int) -> int:
        if self.spi is None:
            self.get_logger().warn('SPI not available')
            return 0xE0

        try:
            rx = self.spi.xfer2([byte_value & 0xFF])
            return rx[0]
        except Exception as e:
            self.get_logger().error(f'SPI transfer failed: {e}')
            return 0xE0

    def command_callback(self, msg: String):
        command = msg.data.strip().lower()

        if command not in self.cmd_map:
            self.get_logger().warn(
                f'Unknown forklift command: {command}. '
                f'Valid commands: {list(self.cmd_map.keys())}'
            )
            return

        tx_byte = self.cmd_map[command]
        rx_byte = self.transfer_byte(tx_byte)

        self.get_logger().info(
            f'Command sent: {command} / 0x{tx_byte:02X}, '
            f'FPGA status: 0x{rx_byte:02X}'
        )

        status_msg = UInt8()
        status_msg.data = rx_byte
        self.status_pub.publish(status_msg)

    def request_status(self):
        rx_byte = self.transfer_byte(0x10)

        status_msg = UInt8()
        status_msg.data = rx_byte
        self.status_pub.publish(status_msg)

    def destroy_node(self):
        if self.spi is not None:
            self.spi.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ForkliftSpiNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()