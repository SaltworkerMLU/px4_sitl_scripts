import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TelemetryReader(Node):
    def __init__(self):
        super().__init__('telemetry_reader')
        self.publisher_ = self.create_publisher(String, 'telemetry_data', 10)

        self.ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=0.1)

        # Timer for reading
        self.read_timer = self.create_timer(0.1, self.read_callback)

        # Timer for sending test data
        self.write_timer = self.create_timer(1.0, self.write_callback)

        self.counter = 0

    def read_callback(self):
        if self.ser.in_waiting > 0:
            raw = self.ser.read(self.ser.in_waiting)
            data = raw.decode('utf-8', errors='ignore').strip()

            if data:
                msg = String()
                msg.data = data
                self.publisher_.publish(msg)
                self.get_logger().info(f'Received: {msg.data}')

    def write_callback(self):
        message = f"Ping {self.counter}\n"
        self.ser.write(message.encode('utf-8'))
        self.get_logger().info(f'Sent: {message.strip()}')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()