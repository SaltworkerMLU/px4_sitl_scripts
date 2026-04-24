import rclpy
from rclpy.node import Node
from px4_msgs.msg import (ActuatorMotors, OffboardControlMode, VehicleCommand, 
                          VehicleAttitudeSetpoint, VehicleOdometry, TrajectorySetpoint)
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
from control import TransferFunction, step_response

class MotorCommander(Node):
    def __init__(self):
        super().__init__('motor_commander')

        self.x, self.y, self.z = 0, 0, 0
        self.dx, self.dy, self.dz = 0, 0, 0

        self.pos = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.acc = [0.0, 0.0, 0.0]



        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos
        )

        #self.q_pub = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint_v1', qos)
        self.vel_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        #self.actuator_pub = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', qos)
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.command_pub  = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)

        self.counter = 0
        # Run at 20 Hz — offboard mode requires >2 Hz heartbeat
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        self.publish_offboard_control_mode()
        self.publish_trajectory()

        # After ~0.5s of streaming (10 cycles), arm and switch mode
        if self.counter == 10:
            self.switch_to_offboard_mode()
            self.arm()

        self.counter += 1

    def publish_offboard_control_mode(self):
        #msg.direct_actuator = True  # Must be True for actuator_motors to work
        msg = OffboardControlMode()
        msg.velocity = False # True
        msg.position = True # False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        
        self.offboard_pub.publish(msg)

    def odometry_callback(self, msg):
        #bige = np.array([self.counter*0.02,0.0,-2.5])-np.array(self.pos)
        self.get_logger().info(f'{msg.position}\t{msg.velocity}')
        self.x, self.y, self.z = msg.position[0], msg.position[1], msg.position[2]
        self.dx, self.dy, self.dz = msg.velocity[0], msg.velocity[1], msg.velocity[2]

    def publish_trajectory(self):
 
        #t = self.counter * 0.05
        #u = 1.0
        #ex = 0.95 * (2.0 - self.x)
        #ey = 0.95 * (1.0 - self.y)
        #ez = 1.0 * (-2.0 - self.z)
        #y_k1 = self.pos[1] + 0.05 * (-self.pos[1] + u)

        #self.acc[1] = self.acc[1] + 0.02 * (-self.acc[1] + u) # np.exp(-self.counter*0.02)
        #self.get_logger().info(f'{self.acc[1]}')

        msg = TrajectorySetpoint()
        
        if self.counter % 2000 < 1000:
            msg.acceleration = [0.0, 0.0, 0.0]
            msg.velocity = [0.0, 0.0, 0.0] # [float('nan')] * 3
            msg.position = [1.0, 0.0, -2.5] # [self.counter*0.02, 0.0, -2.5]
        else:
            msg.acceleration = [0.0, 0.0, 0.0]
            msg.velocity = [0.0, 0.0, 0.0] # [float('nan')] * 3
            msg.position = [0.0, 0.0, -2.5] # [self.counter*0.02, 0.0, -2.5]
        self.get_logger().info(f'{self.counter % 5000}')
        msg.yaw = 0.0
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vel_pub.publish(msg)

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