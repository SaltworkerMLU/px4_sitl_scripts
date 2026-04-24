#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import ActuatorMotors, VehicleOdometry


class ActuatorMotorsListener(Node):
    """
    ROS2 node that subscribes to PX4 actuator motor values
    published via uXRCE-DDS on /fmu/out/actuator_motors.
    """

    def __init__(self):
        super().__init__('actuator_motors_listener')

        # PX4 uXRCE-DDS topics use a specific QoS profile:
        # Best-effort reliability + transient-local or volatile durability
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.actuator_motors_sub = self.create_subscription(
            ActuatorMotors,
            '/fmu/out/actuator_motors',
            self.actuator_motors_callback,
            qos_profile
        )

        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_profile
        )

        self.get_logger().info('ActuatorMotorsListener node started, waiting for data...')

    def actuator_motors_callback(self, msg: ActuatorMotors):
        """
        Called every time a new ActuatorMotors message arrives.

        msg.control is a float32[12] array where:
          - Values are normalized in range [0.0, 1.0] for most setups
          - NaN means the motor is not used / not assigned
        """
        self.get_logger().info('--- Actuator Motors ---')

        for i, value in enumerate(msg.control):
            # Skip motors that are unused (NaN)
            if value != value:  # NaN check (NaN != NaN is True)
                continue
            self.get_logger().info(f'  Motor {i}: {value:.4f}')

        # Access other fields if needed:
        # msg.timestamp         -> uORB timestamp (microseconds)
        # msg.timestamp_sample  -> sample timestamp
        # msg.reversible_flags  -> bitmask of reversible motors
    
    def odom_callback(self, msg: ActuatorMotors):
        self.get_logger().info('--- Odometry ---')

        for i, value in enumerate(msg.position):
            # Skip motors that are unused (NaN)
            if value != value:  # NaN check (NaN != NaN is True)
                continue
            self.get_logger().info(f'  Position {i}: {value:.4f}')

def main(args=None):
    rclpy.init(args=args)
    node = ActuatorMotorsListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()