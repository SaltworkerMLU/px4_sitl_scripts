import rclpy
from rclpy.node import Node
from px4_msgs.msg import ActuatorMotors
import numpy as np

class MotorPublisher(Node):
    def __init__(self):
        super().__init__('motor_publisher')

        self.pub = self.create_publisher(
            ActuatorMotors,
            '/fmu/in/actuator_motors',
            10
        )

        self.timer = self.create_timer(0.05, self.publish_callback)  # 20 Hz

        self.t = 0.0

    def publish_callback(self):
        msg = ActuatorMotors()
        
        # required fields in PX4 uORB bridge
        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        # simple test pattern (hover-ish baseline + oscillation)
        base = 0.5
        amp = 0.1

        self.t += 0.05

        msg.control[0:4] = [
            base + amp * np.sin(self.t),
            base + amp * np.cos(self.t),
            base + amp * np.sin(self.t + 1.0),
            base + amp * np.cos(self.t + 1.0),
        ]

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = MotorPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()