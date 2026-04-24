import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import Trigger
import math


class PositionController(Node):
    """
    Flies the PX4 X500 drone to a target (x, y, z) position in Gazebo.

    Coordinate frame: NED (North-East-Down)
      - x  = North (forward)
      - y  = East  (right)
      - z  = Down  (NEGATIVE values = altitude above ground)

    ── Runtime target update (3 ways) ──────────────────────────────────────

    1. ROS2 parameter (instant, no restart needed):
       ros2 param set /position_controller target_x 10.0
       ros2 param set /position_controller target_y  5.0
       ros2 param set /position_controller target_z -8.0
       ros2 param set /position_controller target_yaw 1.57

    2. Service call (atomic — all axes update together):
       ros2 service call /go_to std_srvs/srv/Trigger {}
         (first set params, then call this service to apply atomically)

    3. Programmatically via update_target():
       node.update_target(x=10.0, y=5.0, z=-8.0, yaw=1.57)
    """

    def __init__(self):
        super().__init__('position_controller')

        # ── Declare ROS2 parameters ──────────────────────────────────────────
        self.declare_parameter('target_x',   0.0)
        self.declare_parameter('target_y',   0.0)
        self.declare_parameter('target_z',  -5.0)
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('arrival_radius', 0.3)  # metres

        self._load_params()

        # ── State ────────────────────────────────────────────────────────────
        self.local_pos = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0
        self.armed = False
        self.offboard_mode = False

        # ── QoS ─────────────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        cb_group = ReentrantCallbackGroup()

        # ── Publishers ───────────────────────────────────────────────────────
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.local_position_callback, qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos)

        # ── Service: /go_to ──────────────────────────────────────────────────
        # Atomically snapshots current param values and flies to them.
        # Use this after setting multiple params to avoid partial updates.
        self.go_to_service = self.create_service(
            Trigger, '/go_to',
            self.go_to_callback,
            callback_group=cb_group
        )

        # ── Service: /land ───────────────────────────────────────────────────
        self.land_service = self.create_service(
            Trigger, '/land',
            self.land_callback,
            callback_group=cb_group
        )

        # ── Service: /return_home ────────────────────────────────────────────
        self.home_service = self.create_service(
            Trigger, '/return_home',
            self.return_home_callback,
            callback_group=cb_group
        )

        # ── Parameter change callback ────────────────────────────────────────
        # Changing any target_* param mid-flight immediately moves the drone.
        self.add_on_set_parameters_callback(self.parameter_callback)

        # ── 20 Hz control loop ───────────────────────────────────────────────
        self.timer = self.create_timer(0.05, self.control_loop,
                                       callback_group=cb_group)

        self.get_logger().info(
            f'Position controller ready.\n'
            f'  Initial target : x={self.target_x:.1f}  y={self.target_y:.1f}  '
            f'z={self.target_z:.1f}  yaw={self.target_yaw:.2f} rad\n'
            f'  Arrival radius : {self.arrival_radius:.2f} m\n'
            f'  Services       : /go_to  /land  /return_home\n'
            f'  Params         : target_x  target_y  target_z  target_yaw'
        )

    # ── Parameter helpers ─────────────────────────────────────────────────────

    def _load_params(self):
        self.target_x      = self.get_parameter('target_x').value
        self.target_y      = self.get_parameter('target_y').value
        self.target_z      = self.get_parameter('target_z').value
        self.target_yaw    = self.get_parameter('target_yaw').value
        self.arrival_radius = self.get_parameter('arrival_radius').value

    def parameter_callback(self, params):
        """
        Called immediately when any parameter is changed via ros2 param set.
        Each changed parameter takes effect individually and instantly.
        """
        for p in params:
            if p.name == 'target_x':
                self.target_x = p.value
                self.get_logger().info(f'target_x updated → {p.value:.2f}')
            elif p.name == 'target_y':
                self.target_y = p.value
                self.get_logger().info(f'target_y updated → {p.value:.2f}')
            elif p.name == 'target_z':
                self.target_z = p.value
                self.get_logger().info(f'target_z updated → {p.value:.2f}')
            elif p.name == 'target_yaw':
                self.target_yaw = p.value
                self.get_logger().info(f'target_yaw updated → {p.value:.2f} rad')
            elif p.name == 'arrival_radius':
                self.arrival_radius = p.value
        return SetParametersResult(successful=True)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def local_position_callback(self, msg: VehicleLocalPosition):
        self.local_pos = msg

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.offboard_mode = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    # ── Service callbacks ─────────────────────────────────────────────────────

    def go_to_callback(self, request, response):
        """
        Atomically reads all target_* params and applies them as the new target.
        Useful when you've staged multiple param changes and want them to apply
        simultaneously rather than one axis at a time.

        Usage:
          ros2 param set /position_controller target_x 10.0
          ros2 param set /position_controller target_y  5.0
          ros2 param set /position_controller target_z -8.0
          ros2 service call /go_to std_srvs/srv/Trigger {}
        """
        self._load_params()
        response.success = True
        response.message = (
            f'Flying to x={self.target_x:.2f} y={self.target_y:.2f} '
            f'z={self.target_z:.2f} yaw={self.target_yaw:.2f}'
        )
        self.get_logger().info(response.message)
        return response

    def land_callback(self, request, response):
        """
        Commands PX4 to land using its native land mode.

        Usage:
          ros2 service call /land std_srvs/srv/Trigger {}
        """
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        response.success = True
        response.message = 'Land command sent'
        self.get_logger().info(response.message)
        return response

    def return_home_callback(self, request, response):
        """
        Commands PX4 Return-to-Launch (RTL).

        Usage:
          ros2 service call /return_home std_srvs/srv/Trigger {}
        """
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        response.success = True
        response.message = 'Return to home command sent'
        self.get_logger().info(response.message)
        return response

    # ── Control loop ──────────────────────────────────────────────────────────

    def control_loop(self):
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        if self.offboard_setpoint_counter == 10:
            self.switch_to_offboard_mode()
            self.arm()

        if self.offboard_setpoint_counter < 15:
            self.offboard_setpoint_counter += 1

        if self.armed and self.offboard_mode:
            dist = self.distance_to_target()
            self.get_logger().info(
                f'pos=({self.local_pos.x:.2f}, {self.local_pos.y:.2f}, {self.local_pos.z:.2f})  '
                f'target=({self.target_x:.2f}, {self.target_y:.2f}, {self.target_z:.2f})  '
                f'dist={dist:.2f} m  {"✓ REACHED" if dist < self.arrival_radius else "→ FLYING"}',
                throttle_duration_sec=1.0
            )

    # ── PX4 message helpers ───────────────────────────────────────────────────

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.direct_actuator = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [self.target_x, self.target_y, self.target_z]
        msg.yaw = self.target_yaw
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.setpoint_pub.publish(msg)

    def arm(self):
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def switch_to_offboard_mode(self):
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard mode command sent')

    def _send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.command_pub.publish(msg)

    # ── Utilities ─────────────────────────────────────────────────────────────

    def distance_to_target(self) -> float:
        return math.sqrt(
            (self.local_pos.x - self.target_x) ** 2 +
            (self.local_pos.y - self.target_y) ** 2 +
            (self.local_pos.z - self.target_z) ** 2
        )

    def update_target(self, x: float, y: float, z: float, yaw: float = 0.0):
        """Programmatic update — use this from other nodes or scripts."""
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.target_yaw = yaw
        self.get_logger().info(
            f'Target updated → x={x:.2f} y={y:.2f} z={z:.2f} yaw={yaw:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = PositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted — disarming...')
        node.disarm()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


'''import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
import math


class PositionController(Node):
    """
    Flies the PX4 X500 drone to a target (x, y, z) position in Gazebo.

    Coordinate frame: NED (North-East-Down)
      - x  = North (forward)
      - y  = East  (right)
      - z  = Down  (NEGATIVE values = altitude above ground)

    Example: target (0, 0, -5) hovers 5 m above the launch point.
    """

    def __init__(self, target_x=0.0, target_y=0.0, target_z=-5.0, yaw=0.0):
        super().__init__('position_controller')

        # ── Target position (NED, metres) ──────────────────────────────────
        self.target_x = target_x
        self.target_y = target_y
        self.target_z = target_z   # negative = above ground
        self.target_yaw = yaw      # radians, 0 = North

        # ── State ───────────────────────────────────────────────────────────
        self.vehicle_status = VehicleStatus()
        self.local_pos = VehicleLocalPosition()
        self.offboard_setpoint_counter = 0
        self.armed = False
        self.offboard_mode = False
        self.takeoff_complete = False

        # ── QoS ─────────────────────────────────────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.local_position_callback, qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos)

        # ── 20 Hz control loop ───────────────────────────────────────────────
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f'Position controller started. Target: '
            f'x={target_x:.1f} y={target_y:.1f} z={target_z:.1f} (NED metres)'
        )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def local_position_callback(self, msg: VehicleLocalPosition):
        self.local_pos = msg

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.offboard_mode = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    # ── Control loop ─────────────────────────────────────────────────────────

    def control_loop(self):
        # 1. Always publish offboard heartbeat and setpoint first
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # 2. After ~0.5 s of streaming, request offboard + arm
        if self.offboard_setpoint_counter == 10:
            self.switch_to_offboard_mode()
            self.arm()

        if self.offboard_setpoint_counter < 15:
            self.offboard_setpoint_counter += 1

        # 3. Log progress once armed and in offboard mode
        if self.armed and self.offboard_mode:
            dist = self.distance_to_target()
            self.get_logger().info(
                f'pos=({self.local_pos.x:.2f}, {self.local_pos.y:.2f}, {self.local_pos.z:.2f}) '
                f'target=({self.target_x:.2f}, {self.target_y:.2f}, {self.target_z:.2f}) '
                f'dist={dist:.2f} m  {"✓ REACHED" if dist < 0.3 else ""}',
                throttle_duration_sec=1.0
            )

    # ── PX4 message helpers ───────────────────────────────────────────────────

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True          # position control
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.direct_actuator = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_pub.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [self.target_x, self.target_y, self.target_z]
        msg.yaw = self.target_yaw
        # Leave velocity/acceleration as NaN (PX4 default) for pure position control
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.setpoint_pub.publish(msg)

    def arm(self):
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def switch_to_offboard_mode(self):
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard mode command sent')

    def _send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.command_pub.publish(msg)

    # ── Utilities ─────────────────────────────────────────────────────────────

    def distance_to_target(self) -> float:
        return math.sqrt(
            (self.local_pos.x - self.target_x) ** 2 +
            (self.local_pos.y - self.target_y) ** 2 +
            (self.local_pos.z - self.target_z) ** 2
        )

    def update_target(self, x: float, y: float, z: float, yaw: float = 0.0):
        """Call this at runtime to fly to a new position."""
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.target_yaw = yaw
        self.get_logger().info(f'New target: x={x:.1f} y={y:.1f} z={z:.1f}')


def main(args=None):
    rclpy.init(args=args)

    # ── Set your desired target position here (NED metres) ──────────────────
    # z is negative because NED: -5.0 means 5 metres above ground
    node = PositionController(
        target_x=0.0,    # 0 m North
        target_y=0.0,    # 0 m East
        target_z=-5.0,   # 5 m altitude
        yaw=0.0          # face North (radians)
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted — disarming...')
        node.disarm()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()'''