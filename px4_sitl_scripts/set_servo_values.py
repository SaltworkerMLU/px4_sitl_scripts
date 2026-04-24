import rclpy
from rclpy.node import Node
from px4_msgs.msg import ActuatorMotors, OffboardControlMode, VehicleCommand
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class MotorCommander(Node):
    def __init__(self):
        super().__init__('motor_commander')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.actuator_pub = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', qos)
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.command_pub  = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)

        self.counter = 0
        # Run at 20 Hz — offboard mode requires >2 Hz heartbeat
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        self.publish_offboard_control_mode()
        self.publish_motors()

        # After ~0.5s of streaming (10 cycles), arm and switch mode
        if self.counter == 10:
            self.switch_to_offboard_mode()
            self.arm()

        self.counter += 1

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.direct_actuator = True  # Must be True for actuator_motors to work
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_pub.publish(msg)

    def publish_motors(self):
        msg = ActuatorMotors()
        msg.control[0] = 0.9
        msg.control[1] = 0.5
        msg.control[2] = 0.5
        msg.control[3] = 0.5
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.actuator_pub.publish(msg)

    def arm(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.command_pub.publish(msg)
        self.get_logger().info('Arm command sent')

    def switch_to_offboard_mode(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0
        msg.param2 = 6.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.command_pub.publish(msg)
        self.get_logger().info('Offboard mode command sent')


def main(args=None):
    rclpy.init(args=args)
    node = MotorCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


"""import rclpy
from rclpy.node import Node
from px4_msgs.msg import ActuatorMotors
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class MotorCommander(Node):
    def __init__(self):
        super().__init__('motor_commander')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.publisher = self.create_publisher(
            ActuatorMotors,
            '/fmu/in/actuator_motors',
            qos_profile
        )

        # Publish at 20 Hz
        self.timer = self.create_timer(0.05, self.publish_motors)
        self.get_logger().info('Motor commander started')

    def publish_motors(self):
        msg = ActuatorMotors()

        # Normalised thrust per motor: 0.0 (off) to 1.0 (full)
        msg.control[0] = 0.5  # Motor 1
        msg.control[1] = 0.5  # Motor 2
        msg.control[2] = 0.5  # Motor 3
        msg.control[3] = 0.5  # Motor 4
        # control[4:8] default to 0.0 for unused motors

        # Required: timestamp in microseconds
        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        self.publisher.publish(msg)
        self.get_logger().info(
            f'Published motors: {msg.control[0]:.2f} {msg.control[1]:.2f} '
            f'{msg.control[2]:.2f} {msg.control[3]:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = MotorCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()"""


"""import rclpy
from rclpy.node import Node
from px4_msgs.msg import ActuatorOutputs, ActuatorMotors
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class ServoSubscriber(Node):
    def __init__(self):
        super().__init__('servo_subscriber')
        
        # Configure QoS to match PX4 uXRCE-DDS settings
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Subscribe to actuator_outputs
        self.subscription = self.create_subscription(
            ActuatorMotors,
            '/fmu/out/actuator_motors',
            self.listener_callback,
            qos_profile)
        self.subscription

    def listener_callback(self, msg):
        # Print output values
        # For X500, motor 1-4 are usually mapped in output[0:4]
        self.get_logger().info(f'Servo Values: {msg.output[0]} {msg.output[1]} {msg.output[2]} {msg.output[3]}')

def main(args=None):
    rclpy.init(args=args)
    node = ServoSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()"""