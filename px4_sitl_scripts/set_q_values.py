import rclpy
from rclpy.node import Node
from px4_msgs.msg import (
    OffboardControlMode,
    VehicleCommand,
    VehicleAttitudeSetpoint,
    VehicleOdometry,
)
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np

class AltitudeController:
    """
    Z-axis altitude controller: z_ref [m] → thrust_body[2] (normalized, NED).
 
    Structure (mirrors the x-axis cascaded controller):
        Outer loop (P):   altitude error      → vertical velocity setpoint
        Inner loop (PD):  velocity error      → acceleration command
        Conversion:       a_cmd / g + 1.0     → thrust (normalized)
 
    NED convention:
        - z is NEGATIVE upward (z = -2.0 means 2 m above ground)
        - vz is NEGATIVE when climbing
        - thrust_body[2] is NEGATIVE for upward thrust (PX4 convention)
 
    Parameters
    ----------
    Kp_pos      : P gain on altitude error               [1/s]
    Kp_vel      : P gain on vertical velocity error      [1/s]
    Kd_vel      : D gain (damps vertical velocity)       [1/s]
    tau_vel     : Low-pass filter time constant for vz   [s]
    hover_thrust: Normalized thrust at hover (tune this) [-]
                  Typical X500 in Gazebo: 0.65 – 0.75
    thrust_min  : Lower clamp on output thrust           [-]
    thrust_max  : Upper clamp on output thrust           [-]
    """
 
    def __init__(
        self,
        Kp_pos: float = 1.0,
        Kp_vel: float = 0.2,
        Kd_vel: float = 0.1,
        tau_vel: float = 0.15,
        hover_thrust: float = 0.70,
        thrust_min: float = 0.1,
        thrust_max: float = 0.95,
    ):
        self.Kp_pos = Kp_pos
        self.Kp_vel = Kp_vel
        self.Kd_vel = Kd_vel
        self.tau_vel = tau_vel
        self.hover_thrust = hover_thrust
        self.thrust_min = thrust_min
        self.thrust_max = thrust_max
 
        self._vz_filtered = 0.0
 
    def reset(self):
        self._vz_filtered = 0.0
 
    def update(self, z_ref: float, z_meas: float, vz_meas: float, dt: float) -> float:
        """
        Parameters
        ----------
        z_ref   : Target altitude in NED [m]  e.g. -2.0 = 2 m above ground
        z_meas  : Measured z position    [m]  from VehicleOdometry.position[2]
        vz_meas : Measured z velocity    [m/s] from VehicleOdometry.velocity[2]
        dt      : Time step              [s]
 
        Returns
        -------
        thrust  : Normalized thrust for thrust_body[2], always negative (upward)
        """
        if dt <= 0.0:
            return -self.hover_thrust
 
        # ── 1. Low-pass filter on vz ────────────────────────────────────
        alpha = self.tau_vel / (self.tau_vel + dt)
        self._vz_filtered = alpha * self._vz_filtered + (1.0 - alpha) * vz_meas
 
        # ── 2. Altitude (outer) loop ────────────────────────────────────
        # In NED: z_ref < z_meas means we are too low → need upward force
        alt_error = z_ref - z_meas
        vz_ref = self.Kp_pos * alt_error   # desired climb rate (NED: negative = up)
 
        # ── 3. Velocity (inner) loop ────────────────────────────────────
        vel_error = vz_ref - self._vz_filtered
        a_cmd = self.Kp_vel * vel_error - self.Kd_vel * self._vz_filtered
 
        # ── 4. Acceleration → normalized thrust ─────────────────────────
        # At hover: thrust exactly cancels gravity → hover_thrust
        # Extra acceleration adds/subtracts from that baseline.
        # a_cmd is in NED (negative = upward), so:
        #   thrust = hover_thrust - a_cmd / g   (both in NED sign)
        # Then negate for thrust_body[2] convention (must be negative).
        g = 9.81
        thrust_magnitude = self.hover_thrust - (a_cmd / g)
        thrust = -float(np.clip(thrust_magnitude, self.thrust_min, self.thrust_max))
 
        return thrust

class CascadedPositionController:
    """
    Cascaded position → velocity → acceleration → roll controller.

    Outer loop (P):   position error → velocity reference
    Inner loop (PD):  velocity error → acceleration command
    Conversion:       a_cmd / g → phi_cmd
    """

    def __init__(
        self,
        Kp_pos: float = 0.5,
        Kp_vel: float = 1.5,
        Kd_vel: float = 0.3,
        tau_vel: float = 0.15,
        g: float = 9.81,
        phi_max: float = np.radians(20.0),
    ):
        self.Kp_pos = Kp_pos
        self.Kp_vel = Kp_vel
        self.Kd_vel = Kd_vel
        self.tau_vel = tau_vel
        self.g = g
        self.phi_max = phi_max

        # Filter state for velocity smoothing
        self._v_filtered = 0.0
        self._x_prev = None

    def reset(self):
        self._v_filtered = 0.0
        self._x_prev = None

    def update(self, x_ref: float, x_meas: float, v_meas: float, dt: float) -> float:
        """
        Parameters
        ----------
        x_ref   : Target x position [m]
        x_meas  : Measured x position [m]
        v_meas  : Measured x velocity [m/s]  (from odometry)
        dt      : Time step [s]

        Returns
        -------
        phi_cmd : Commanded roll angle [rad], NED sign convention
        """
        if dt <= 0.0:
            return 0.0

        # ── 1. Low-pass filter on measured velocity (reduces noise) ────
        alpha = self.tau_vel / (self.tau_vel + dt)
        self._v_filtered = alpha * self._v_filtered + (1.0 - alpha) * v_meas

        # ── 2. Position loop (P): position error → velocity setpoint ───
        pos_error = x_ref - x_meas
        v_ref = self.Kp_pos * pos_error

        # ── 3. Velocity loop (PD): velocity error → accel command ──────
        vel_error = v_ref - self._v_filtered
        a_cmd = self.Kp_vel * vel_error - self.Kd_vel * self._v_filtered

        # ── 4. Acceleration → roll angle ────────────────────────────────
        # PX4 NED convention: +x forward, positive roll = right side down
        # To accelerate in +x, we need a negative roll (tilt left/forward)
        phi_cmd = -(a_cmd / self.g)

        return float(np.clip(phi_cmd, -self.phi_max, self.phi_max))


def euler_to_quaternion_wxyz(roll: float, pitch: float, yaw: float):
    """
    Convert Euler angles (ZYX) to quaternion in [w, x, y, z] order.
    This is the order PX4 expects in VehicleAttitudeSetpoint.q_d.
    """
    cr = np.cos(roll  / 2);  sr = np.sin(roll  / 2)
    cp = np.cos(pitch / 2);  sp = np.sin(pitch / 2)
    cy = np.cos(yaw   / 2);  sy = np.sin(yaw   / 2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return w, x, y, z


class MotorCommander(Node):
    def __init__(self):
        super().__init__('motor_commander')

        # ── Parameters ─────────────────────────────────────────────────
        self.x_ref   = 2.0          # Target x position [m]
        self.TIMER_HZ = 20          # Control loop rate [Hz]
        self.dt       = 1.0 / self.TIMER_HZ

        # ── State from odometry ────────────────────────────────────────
        self.x, self.y, self.z  = 0.0, 0.0, 0.0
        self.vx, self.vy, self.vz = 0.0, 0.0, 0.0

        # ── Controller ─────────────────────────────────────────────────
        self.ctrl = CascadedPositionController(
            Kp_pos=0.5,
            Kp_vel=1.5,
            Kd_vel=0.3,
            tau_vel=0.15,
        )

        self.alt_ctrl = AltitudeController(hover_thrust=0.70)

        # ── QoS (PX4 requires BEST_EFFORT) ────────────────────────────
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ── Subscribers ────────────────────────────────────────────────
        self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos,
        )

        # ── Publishers ─────────────────────────────────────────────────
        self.att_pub      = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint_v1', qos)
        self.offboard_pub = self.create_publisher(OffboardControlMode,     '/fmu/in/offboard_control_mode',     qos)
        self.command_pub  = self.create_publisher(VehicleCommand,          '/fmu/in/vehicle_command',           qos)

        self.counter = 0
        self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info(f'MotorCommander started — target x = {self.x_ref} m')

    # ── Callbacks ──────────────────────────────────────────────────────

    def odometry_callback(self, msg: VehicleOdometry):
        self.x, self.y, self.z  = msg.position[0], msg.position[1], msg.position[2]
        self.vx, self.vy, self.vz = msg.velocity[0], msg.velocity[1], msg.velocity[2]
        """self.get_logger().info(
            f'x={self.x:.3f} m  vx={self.vx:.3f} m/s'# ,
            #throttle_duration_sec=0.5,
        )"""

    def timer_callback(self):
        # Always stream the offboard heartbeat first
        self.publish_offboard_control_mode()

        # Arm and switch to offboard after ~0.5 s (10 cycles)
        if self.counter == 10:
            self.switch_to_offboard_mode()
            self.arm()

        self.publish_attitude_setpoint()
        self.counter += 1

    # ── Publishing helpers ─────────────────────────────────────────────

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.attitude  = True          # We are sending attitude setpoints
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_pub.publish(msg)

    def publish_attitude_setpoint(self):
        phi_cmd = self.ctrl.update(self.x_ref, self.x, self.vx, self.dt)

        self.get_logger().info(
            f'x_ref={self.x_ref:.2f}  x={self.x:.3f}  phi={np.degrees(phi_cmd):.2f} deg' #,
            #throttle_duration_sec=0.5,
        )

        roll  = phi_cmd
        pitch = 0.0
        yaw   = 0.0

        # ── CRITICAL: PX4 q_d order is [w, x, y, z] ───────────────────
        w, qx, qy, qz = euler_to_quaternion_wxyz(roll, pitch, yaw)

        msg = VehicleAttitudeSetpoint()
        msg.q_d[0] = w
        msg.q_d[1] = qx
        msg.q_d[2] = qy
        msg.q_d[3] = qz

        #self.get_logger().info(f'{msg.q_d}')

        # Thrust: NED convention, -z = upward.
        # 0.5 is a placeholder — tune for your X500 hover thrust.
        #msg.thrust_body = [0.0, 0.0, -0.75]
        thrust = self.alt_ctrl.update(-5, self.z, self.vz, self.dt)
        msg.thrust_body = [0.0, 0.0, thrust]

        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.att_pub.publish(msg)

    def arm(self):
        msg = VehicleCommand()
        msg.command          = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1           = 1.0
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = self.get_clock().now().nanoseconds // 1000
        self.command_pub.publish(msg)
        self.get_logger().info('Arm command sent')

    def switch_to_offboard_mode(self):
        msg = VehicleCommand()
        msg.command          = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1           = 1.0   # custom mode
        msg.param2           = 6.0   # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = self.get_clock().now().nanoseconds // 1000
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