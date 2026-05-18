import rclpy
import spidev
from rclpy.node import Node

from std_msgs.msg import Bool

CMD_LIFTER_OFF = 0x00
CMD_LIFTER_ON = 0x01
CMD_READ_STATUS = 0xFF

FPGA_CONFIRM_OFF = 0x5A
FPGA_CONFIRM_ON = 0xA5

class LifterSpiBridge(Node):
    def __init__(self):
        super().__init__('lifter_spi_bridge')

        # parámetros de SPI
        self.declare_parameter('spi_bus', 0)
        self.declare_parameter('spi_device', 0)
        self.declare_parameter('spi_speed_hz', 1000000)

        self.spi_bus = int(self.get_parameter('spi_bus').value)
        self.spi_device = int(self.get_parameter('spi_device').value)
        self.spi_speed_hz = int(self.get_parameter('spi_speed_hz').value)

        self.spi = spidev.SpiDev()
        self.spi.open(self.spi_bus, self.spi_device)

        self.spi.mode = 0b00
        self.spi.max_speed_hz = self.spi_speed_hz
        self.spi.bits_per_word = 8

        self.desired_lifter_state = False
        self.confirmed_lifter_state = False
        self.last_logged_state = None

        self.lifter_sub = self.create_subscription(
            Bool,
            '/lifter',
            self.lifter_callback,
            10
        )

        self.confirm_pub = self.create_publisher(
            Bool,
            '/lifter_confirm',
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            f'Lifter SPI bridge started using /dev/spidev{self.spi_bus}.{self.spi_device}'
        )
        self.get_logger().info(f'SPI speed: {self.spi_speed_hz} Hz, mode 0')
        self.get_logger().info('Waiting for /lifter commands...')
    
    def transfer_byte(self, tx_byte):
        """Send one byte through SPI and return the received byte."""
        try:
            rx_data = self.spi.xfer2([tx_byte & 0xFF])

            if len(rx_data) != 1:
                self.get_logger().error('SPI transfer returned invalid response length.')
                return None

            return int(rx_data[0])

        except OSError as error:
            self.get_logger().error(f'SPI transfer failed: {error}')
            return None
    
    def send_lifter_command(self):
        """
        Send the desired lifter state to the FPGA and read back confirmation.

        Transfer 1:
            Send command 0x01 or 0x00.

        Transfer 2:
            Send dummy/read byte 0xFF to receive FPGA status.
        """
        if self.desired_lifter_state:
            command = CMD_LIFTER_ON
        else:
            command = CMD_LIFTER_OFF

        # First transfer: write desired state to FPGA.
        write_response = self.transfer_byte(command)

        if write_response is None:
            return None

        # Second transfer: read confirmed state from FPGA.
        status_response = self.transfer_byte(CMD_READ_STATUS)

        if status_response is None:
            return None

        if status_response == FPGA_CONFIRM_ON:
            return True

        if status_response == FPGA_CONFIRM_OFF:
            return False

        self.get_logger().warn(
            f'Unexpected FPGA response: 0x{status_response:02X} '
            f'(write response was 0x{write_response:02X})'
        )

        return None

    def lifter_callback(self, msg):
        self.desired_lifter_state = bool(msg.data)

        if self.desired_lifter_state:
            self.get_logger().info('Received /lifter command: ON')
        else:
            self.get_logger().info('Received /lifter command: OFF')

    def timer_callback(self):
        """
        Periodically send the desired lifter state to the FPGA and publish
        the confirmed state returned by the FPGA.
        """
        confirmed_state = self.send_lifter_command()

        if confirmed_state is None:
            return

        self.confirmed_lifter_state = confirmed_state

        confirm_msg = Bool()
        confirm_msg.data = self.confirmed_lifter_state
        self.confirm_pub.publish(confirm_msg)

        if self.last_logged_state != self.confirmed_lifter_state:
            if self.confirmed_lifter_state:
                self.get_logger().info('FPGA confirmation: LIFTER ON')
            else:
                self.get_logger().info('FPGA confirmation: LIFTER OFF')

            self.last_logged_state = self.confirmed_lifter_state

def main(args=None):
    rclpy.init(args=args)

    node = LifterSpiBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Closing lifter SPI bridge...')
    finally:
        node.spi.close()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()