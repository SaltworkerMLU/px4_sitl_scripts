"""
px4_inner_loop_nodes.py
=======================
Three ROS2 nodes replicating PX4's inner control loop for the X500:

  Node 1 – AttitudeControlNode   (250 Hz)
    Subscribes:  /fmu/out/vehicle_attitude          (q_actual)
                 /fmu/in/vehicle_attitude_setpoint   (q_des, thrust_body[2])
    Publishes:   /custom/rate_setpoint              (VehicleRatesSetpoint)

  Node 2 – RateControlNode       (1000 Hz)
    Subscribes:  /fmu/out/vehicle_angular_velocity   (rate_actual)
                 /custom/rate_setpoint               (rate_sp)
    Publishes:   /custom/torque_setpoint             (VehicleTorqueSetpoint)
                 /custom/thrust_setpoint             (VehicleThrustSetpoint)

  Node 3 – MixerNode             (1000 Hz)
    Subscribes:  /custom/torque_setpoint
                 /custom/thrust_setpoint
    Publishes:   /fmu/in/actuator_motors            (ActuatorMotors)

Sources
-------
AttitudeControl : mc_att_control/AttitudeControl/AttitudeControl.cpp
                  ETH Zurich "Nonlinear Quadrocopter Attitude Control" (2013)
RateControl     : mc_rate_control/RateControl/RateControl.cpp
Mixer           : PX4 X500 airframe geometry (4 motors, quad-x config)

PX4 default gains used (X500 / generic quad):
    MC_ROLL_P   = 6.5   MC_PITCH_P  = 6.5   MC_YAW_P    = 2.8
    MC_ROLLRATE_P  = 0.15  MC_ROLLRATE_I  = 0.2  MC_ROLLRATE_D  = 0.003
    MC_PITCHRATE_P = 0.15  MC_PITCHRATE_I = 0.2  MC_PITCHRATE_D = 0.003
    MC_YAWRATE_P   = 0.2   MC_YAWRATE_I   = 0.1  MC_YAWRATE_D   = 0.0
    MC_ROLLRATE_MAX  = 220 deg/s
    MC_PITCHRATE_MAX = 220 deg/s
    MC_YAWRATE_MAX   = 200 deg/s
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
from px4_msgs.msg import (OffboardControlMode, VehicleCommand)

from px4_msgs.msg import (
    VehicleAttitude,
    VehicleAttitudeSetpoint,
    VehicleAngularVelocity,
    VehicleRatesSetpoint,
    VehicleTorqueSetpoint,
    VehicleThrustSetpoint,
    ActuatorMotors,
)


# ═══════════════════════════════════════════════════════════════════════════════
# Quaternion helpers
# ═══════════════════════════════════════════════════════════════════════════════

def quat_mult(p: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Hamilton product of two [w,x,y,z] quaternions."""
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return np.array([
        pw*qw - px*qx - py*qy - pz*qz,
        pw*qx + px*qw + py*qz - pz*qy,
        pw*qy - px*qz + py*qw + pz*qx,
        pw*qz + px*qy - py*qx + pz*qw,
    ])


def quat_inv(q: np.ndarray) -> np.ndarray:
    """Inverse (= conjugate for unit quaternion) of [w,x,y,z]."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_norm(q: np.ndarray) -> np.ndarray:
    return q / np.linalg.norm(q)


def quat_dcm_z(q: np.ndarray) -> np.ndarray:
    """
    Body z-axis expressed in world frame from quaternion [w,x,y,z].
    Equivalent to last column of the rotation matrix (DCM).
    PX4: Quaternion::dcm_z()
    """
    w, x, y, z = q
    return np.array([
        2.0*(x*z + w*y),
        2.0*(y*z - w*x),
        w*w - x*x - y*y + z*z,
    ])


def sign_no_zero(x: float) -> float:
    """PX4 math::signNoZero — returns +1 for x>=0, -1 for x<0."""
    return 1.0 if x >= 0.0 else -1.0


# ═══════════════════════════════════════════════════════════════════════════════
# Second-order Butterworth low-pass filter (for rate controller D term)
# Matches PX4's LowPassFilter2pVector3f
# ═══════════════════════════════════════════════════════════════════════════════

class LowPassFilter2p:
    """
    Second-order Butterworth low-pass filter.
    Direct Form II. Matches PX4's LowPassFilter2p implementation.
    """
    def __init__(self, sample_freq: float, cutoff_freq: float):
        self._delay_element_1 = 0.0
        self._delay_element_2 = 0.0
        self._set_cutoff(sample_freq, cutoff_freq)

    def _set_cutoff(self, sample_freq: float, cutoff_freq: float):
        fr = sample_freq / cutoff_freq
        ohm = np.tan(np.pi / fr)
        c = 1.0 + 2.0 * np.cos(np.pi / 4.0) * ohm + ohm * ohm
        self._b0 = ohm * ohm / c
        self._b1 = 2.0 * self._b0
        self._b2 = self._b0
        self._a1 = 2.0 * (ohm * ohm - 1.0) / c
        self._a2 = (1.0 - 2.0 * np.cos(np.pi / 4.0) * ohm + ohm * ohm) / c

    def apply(self, sample: float) -> float:
        delay_element_0 = sample - self._delay_element_1 * self._a1 \
                                 - self._delay_element_2 * self._a2
        output = delay_element_0 * self._b0 \
               + self._delay_element_1 * self._b1 \
               + self._delay_element_2 * self._b2
        self._delay_element_2 = self._delay_element_1
        self._delay_element_1 = delay_element_0
        return output

    def reset(self, val: float = 0.0):
        self._delay_element_1 = val
        self._delay_element_2 = val


# ═══════════════════════════════════════════════════════════════════════════════
# Node 1: Attitude Controller  (250 Hz)
# Source: AttitudeControl::update() in AttitudeControl.cpp
# ═══════════════════════════════════════════════════════════════════════════════

class AttitudeControlNode(Node):
    """
    Quaternion attitude controller.
    Exact Python port of PX4 AttitudeControl::update().

    Algorithm (ETH Zurich 2013 paper):
      1. Compute reduced desired attitude (prioritise thrust direction over yaw)
      2. Mix reduced and full desired attitude with yaw_weight
      3. Error quaternion qe = q_actual^-1 * q_desired
      4. Rate setpoint = 2 * sign(qe.w) * qe.xyz  (element-wise * gain)
      5. Add yaw feedforward if provided
    """

    def __init__(self):
        super().__init__('attitude_control_node')

        # ── PX4 default attitude P gains (MC_ROLL_P, MC_PITCH_P, MC_YAW_P) ──
        self.att_p = np.array([6.5, 6.5, 2.8])   # [roll, pitch, yaw]

        # Rate limits [rad/s] (MC_ROLLRATE_MAX, MC_PITCHRATE_MAX, MC_YAWRATE_MAX)
        self.rate_limit = np.radians([220.0, 220.0, 200.0])

        # ── State ──────────────────────────────────────────────────────
        self.q_actual  = np.array([1.0, 0.0, 0.0, 0.0])   # w,x,y,z
        self.q_des     = np.array([1.0, 0.0, 0.0, 0.0])
        self.thrust_sp = -0.73
        self.yaw_sp_move_rate = 0.0
        self.counter = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        #self.create_subscription(VehicleAttitude,
        #    '/fmu/out/vehicle_attitude', self.att_cb, qos)
        self.create_subscription(VehicleAttitudeSetpoint,
            '/fmu/in/vehicle_attitude_setpoint_v1', self.att_sp_cb, qos)

        #self.rate_sp_pub = self.create_publisher(
        #    VehicleRatesSetpoint, '/custom/rate_setpoint', qos)
        self.rate_sp_pub = self.create_publisher(
            VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos)
        self.offboard_pub = self.create_publisher(OffboardControlMode,
                                '/fmu/in/offboard_control_mode', qos)
        self.cmd_pub      = self.create_publisher(VehicleCommand,
                                '/fmu/in/vehicle_command', qos)

        # 250 Hz
        self.create_timer(1.0 / 250.0, self.timer_cb)
        self.get_logger().info('AttitudeControlNode started at 250 Hz')

    def att_cb(self, msg: VehicleAttitude):
        # PX4 q is [w,x,y,z]
        self.q_actual = quat_norm(np.array([
            msg.q[0], msg.q[1], msg.q[2], msg.q[3]
        ]))

    def att_sp_cb(self, msg: VehicleAttitudeSetpoint):
        self.q_des = quat_norm(np.array([
            msg.q_d[0], msg.q_d[1], msg.q_d[2], msg.q_d[3]
        ]))
        self.thrust_sp = msg.thrust_body[2]
        self.yaw_sp_move_rate = msg.yaw_sp_move_rate

    def timer_cb(self):
        self._publish_offboard_mode()

        if self.counter == 10:
            self._switch_offboard()
            self._arm()

        if self.counter >= 10:
            self.update()
        
        self.counter += 1

    def update(self):
        q  = self.q_actual
        qd = self.q_des

        # ── Step 1: reduced desired attitude (prioritise thrust vector) ──────
        # Compute the rotation that maps current body-z to desired body-z,
        # ignoring yaw. PX4: Quatf qd_red(e_z, e_z_d)
        e_z   = quat_dcm_z(q)
        e_z_d = quat_dcm_z(qd)

        # Quaternion from two vectors (shortest rotation)
        cross = np.cross(e_z, e_z_d)
        cross_norm = np.linalg.norm(cross)
        dot = np.clip(np.dot(e_z, e_z_d), -1.0, 1.0)

        if cross_norm < 1e-6:
            # Vectors are parallel — identity or 180° rotation
            if dot > 0:
                qd_red = np.array([1.0, 0.0, 0.0, 0.0])
            else:
                # 180° — pick arbitrary perpendicular axis
                qd_red = np.array([0.0, 1.0, 0.0, 0.0])
        else:
            w_red = np.sqrt(0.5 * (1.0 + dot))
            xyz   = cross / (2.0 * w_red + 1e-12)
            qd_red = quat_norm(np.array([w_red, xyz[0], xyz[1], xyz[2]]))

        # Corner case: thrust vectors exactly opposite — use full qd directly
        if abs(qd_red[1]) > (1.0 - 1e-5) or abs(qd_red[2]) > (1.0 - 1e-5):
            qd_red = qd
        else:
            # Transform into world frame: qd_red = qd_red * q  (right multiply)
            qd_red = quat_mult(qd_red, q)

        # ── Step 2: mix reduced and full attitude using yaw weight ────────────
        # yaw_w derived from gain ratio (PX4 mc_att_control_main.cpp line ~88)
        roll_pitch_gain = (self.att_p[0] + self.att_p[1]) / 2.0
        yaw_w = np.clip(self.att_p[2] / roll_pitch_gain, 0.0, 1.0)

        q_mix = quat_mult(quat_inv(qd_red), qd)
        q_mix *= sign_no_zero(q_mix[0])
        q_mix[0] = np.clip(q_mix[0], -1.0, 1.0)
        q_mix[3] = np.clip(q_mix[3], -1.0, 1.0)

        # Interpolate yaw component
        qd_final = quat_mult(
            qd_red,
            np.array([
                np.cos(yaw_w * np.arccos(q_mix[0])),
                0.0,
                0.0,
                np.sin(yaw_w * np.arcsin(q_mix[3])),
            ])
        )

        # ── Step 3: error quaternion qe = q^-1 * qd ─────────────────────────
        qe = quat_mult(quat_inv(q), qd_final)

        # ── Step 4: rate setpoint from quaternion error ───────────────────────
        # eq = 2 * sign(qe.w) * qe.xyz   (sin(alpha/2) axis-angle representation)
        eq = 2.0 * sign_no_zero(qe[0]) * qe[1:4]

        # Element-wise multiply by attitude gain
        rate_sp = eq * self.att_p

        # ── Step 5: yaw feed-forward (world z-axis in body frame) ────────────
        if abs(self.yaw_sp_move_rate) > 1e-6:
            # q^-1 rotates world-z into body frame → dcm_z of q_inv = last col of R^T
            q_inv = quat_inv(q)
            world_z_body = quat_dcm_z(q_inv)
            rate_sp += world_z_body * self.yaw_sp_move_rate

        # ── Saturate ─────────────────────────────────────────────────────────
        rate_sp = np.clip(rate_sp, -self.rate_limit, self.rate_limit)

        # ── Publish ───────────────────────────────────────────────────────────
        msg = VehicleRatesSetpoint()
        msg.roll  = float(rate_sp[0])
        msg.pitch = float(rate_sp[1])
        msg.yaw   = float(rate_sp[2])
        msg.thrust_body[0] = 0.0
        msg.thrust_body[1] = 0.0
        msg.thrust_body[2] = float(self.thrust_sp)
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.rate_sp_pub.publish(msg)

    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.attitude     = False # True
        msg.body_rate    = True
        msg.position     = False
        msg.velocity     = False
        msg.acceleration = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False # True
        msg.timestamp    = self.get_clock().now().nanoseconds // 1000
        self.offboard_pub.publish(msg)

    def _arm(self):
        msg = VehicleCommand()
        msg.command          = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1           = 1.0
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = self.get_clock().now().nanoseconds // 1000
        self.cmd_pub.publish(msg)
        self.get_logger().info('Arm command sent')
    
    def _switch_offboard(self):
        msg = VehicleCommand()
        msg.command          = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1           = 1.0
        msg.param2           = 6.0
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = self.get_clock().now().nanoseconds // 1000
        self.cmd_pub.publish(msg)
        self.get_logger().info('Offboard mode command sent')


# ═══════════════════════════════════════════════════════════════════════════════
# Node 2: Rate Controller  (1000 Hz)
# Source: RateControl::update() in RateControl.cpp
#
# torque = Kp * rate_error + Ki * integral - Kd_filtered * rate + Kff * rate_sp
# D term uses a 2nd-order Butterworth LP filter (PX4: LowPassFilter2pVector3f)
# ═══════════════════════════════════════════════════════════════════════════════

class RateControlNode(Node):

    RATE_HZ = 1000.0

    def __init__(self):
        super().__init__('rate_control_node')

        # ── PX4 X500 default rate gains ───────────────────────────────
        # MC_ROLLRATE_K=1, MC_ROLLRATE_P/I/D scaled by K
        self.Kp  = np.array([0.15, 0.15, 0.20])   # roll, pitch, yaw
        self.Ki  = np.array([0.20, 0.20, 0.10])
        self.Kd  = np.array([0.003, 0.003, 0.0])
        self.Kff = np.array([0.0,   0.0,   0.0])   # feed-forward (not used for MC)

        # Integrator limits (normalized torque units)
        self.int_lim = np.array([0.30, 0.30, 0.30])

        # D-term low-pass filter: PX4 default cutoff = 30 Hz
        cutoff = 30.0
        self._lp = [
            LowPassFilter2p(self.RATE_HZ, cutoff),
            LowPassFilter2p(self.RATE_HZ, cutoff),
            LowPassFilter2p(self.RATE_HZ, cutoff),
        ]

        # ── State ──────────────────────────────────────────────────────
        self.rate_actual = np.zeros(3)
        self.rate_sp     = np.zeros(3)
        self.thrust_sp   = -0.73
        self._integral   = np.zeros(3)
        self._rate_prev  = np.zeros(3)   # for D-on-measurement
        self._dt         = 1.0 / self.RATE_HZ

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.create_subscription(VehicleAngularVelocity,
            '/fmu/out/vehicle_angular_velocity', self.rate_cb, qos)
        self.create_subscription(VehicleRatesSetpoint,
            '/custom/rate_setpoint', self.rate_sp_cb, qos)

        self.torque_pub = self.create_publisher(
            VehicleTorqueSetpoint, '/custom/torque_setpoint', qos)
        self.thrust_pub = self.create_publisher(
            VehicleThrustSetpoint, '/custom/thrust_setpoint', qos)

        self.create_timer(self._dt, self.update)
        self.get_logger().info('RateControlNode started at 1000 Hz')

    def rate_cb(self, msg: VehicleAngularVelocity):
        self.rate_actual = np.array([msg.xyz[0], msg.xyz[1], msg.xyz[2]])

    def rate_sp_cb(self, msg: VehicleRatesSetpoint):
        self.rate_sp   = np.array([msg.roll, msg.pitch, msg.yaw])
        self.thrust_sp = msg.thrust_body[2]

    def update(self):
        dt = self._dt
        rate    = self.rate_actual
        rate_sp = self.rate_sp

        # ── Rate error ────────────────────────────────────────────────
        rate_error = rate_sp - rate

        # ── D term: 2nd-order LP filter on measured rate ──────────────
        # PX4: _lp_filters_d.apply(rate) then differentiate
        # For Python simplicity: filter the rate directly, D acts on filtered rate
        rate_d_filtered = np.array([
            self._lp[i].apply(rate[i]) for i in range(3)
        ])

        # ── PX4 torque law (RateControl.cpp line 83) ──────────────────
        # torque = Kp*err + integral - Kd*rate_filtered + Kff*rate_sp
        torque = (
            self.Kp  * rate_error
            + self._integral
            - self.Kd * rate_d_filtered
            + self.Kff * rate_sp
        )

        # ── Integrator update with anti-windup ────────────────────────
        # PX4 uses an i_factor that reduces integral for large errors
        # (prevents bounce-back after flips, line 115 RateControl.cpp)
        for i in range(3):
            i_factor = rate_error[i] / np.radians(400.0)
            i_factor = max(0.0, 1.0 - i_factor * i_factor)
            self._integral[i] += self.Ki[i] * rate_error[i] * i_factor * dt
            self._integral[i]  = np.clip(self._integral[i],
                                         -self.int_lim[i], self.int_lim[i])

        # ── Publish torque ────────────────────────────────────────────
        tmsg = VehicleTorqueSetpoint()
        tmsg.xyz[0] = float(torque[0])
        tmsg.xyz[1] = float(torque[1])
        tmsg.xyz[2] = float(torque[2])
        tmsg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.torque_pub.publish(tmsg)

        # ── Publish thrust (pass-through from attitude controller) ────
        fmsg = VehicleThrustSetpoint()
        fmsg.xyz[0] = 0.0
        fmsg.xyz[1] = 0.0
        fmsg.xyz[2] = float(self.thrust_sp)
        fmsg.timestamp = tmsg.timestamp
        self.thrust_pub.publish(fmsg)


# ═══════════════════════════════════════════════════════════════════════════════
# Node 3: Mixer  (1000 Hz)
# Source: PX4 control_allocator + X500 airframe geometry
#
# X500 motor layout (top view, NED body frame, motors numbered PX4-style):
#
#          front (+x)
#            |
#     M4(CW) |  M2(CCW)
#      \     |     /
#       \    |    /
#        ----+----   ---- y (right)
#       /    |    \
#      /     |     \
#     M3(CCW)|  M1(CW)
#            |
#          rear
#
# Motor positions (arm_length from centre, 45° diagonals):
#   M1: (+L, -L)  CW   → positive torque reduces yaw (CCW yaw)
#   M2: (-L, -L)  CCW  → negative torque adds yaw
#   M3: (-L, +L)  CW
#   M4: (+L, +L)  CCW
#
# Mixing matrix B maps [torque_x, torque_y, torque_z, thrust] → [m1,m2,m3,m4]
# Derived from PX4 X500 airframe geometry.
# ═══════════════════════════════════════════════════════════════════════════════

class MixerNode(Node):

    def __init__(self):
        super().__init__('mixer_node')

        # ── X500 geometry ─────────────────────────────────────────────
        # Arm length (centre to motor): X500 = 0.25 m
        L = 0.25 / np.sqrt(2.0)   # projected arm length along x and y

        # Torque constant ratio: reaction torque / thrust  (dimensionless)
        # For T-Motor MN2212 on X500: kappa ≈ 0.016
        kappa = 0.016

        # Mixing matrix: each column is a motor's contribution
        # Rows: [roll torque, pitch torque, yaw torque, thrust]
        # Columns: [M1, M2, M3, M4]
        #
        # Roll  (+x torque → right side down): M1+, M2+, M3-, M4-
        # Pitch (+y torque → nose up):         M1-, M2+, M3-, M4+
        # Yaw   (+z torque → CCW from above):  M1-, M2+, M3+, M4-  (CW=neg)
        # Thrust (always positive, mapped to [-1,0] for PX4 NED):
        self._B = np.array([
        #    M1      M2      M3      M4
            [ L,     L,     -L,     -L  ],   # roll
            [-L,     L,     -L,      L  ],   # pitch
            [-kappa, kappa,  kappa, -kappa],  # yaw
            [ 1.0,   1.0,    1.0,   1.0 ],   # thrust
        ])

        # Pseudo-inverse for allocation: motor_cmds = B_pinv @ [tx, ty, tz, thr]
        self._B_pinv = np.linalg.pinv(self._B)

        # ── State ──────────────────────────────────────────────────────
        self.torque = np.zeros(3)
        self.thrust = -0.73   # NED: negative upward

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.create_subscription(VehicleTorqueSetpoint,
            '/custom/torque_setpoint', self.torque_cb, qos)
        self.create_subscription(VehicleThrustSetpoint,
            '/custom/thrust_setpoint', self.thrust_cb, qos)

        self.motor_pub = self.create_publisher(
            ActuatorMotors, '/fmu/in/actuator_motors', qos)

        self.create_timer(1.0 / 1000.0, self.update)
        self.get_logger().info('MixerNode started at 1000 Hz')

    def torque_cb(self, msg: VehicleTorqueSetpoint):
        self.torque = np.array([msg.xyz[0], msg.xyz[1], msg.xyz[2]])

    def thrust_cb(self, msg: VehicleThrustSetpoint):
        self.thrust = msg.xyz[2]   # NED: negative upward

    def update(self):
        # PX4 convention: collective thrust is positive scalar mapped to [0,1]
        # thrust_body[2] is negative in NED → convert to positive scalar
        thrust_scalar = float(np.clip(-self.thrust, 0.0, 1.0))

        control = np.array([
            self.torque[0],
            self.torque[1],
            self.torque[2],
            thrust_scalar,
        ])

        # ── Allocate: motor_cmds = B_pinv @ control ───────────────────
        motor_cmds = self._B_pinv @ control

        # ── Desaturation (PX4 airmode=disabled: reduce torques if needed) ──
        # Scale down torques uniformly if any motor goes below 0 or above 1
        motor_cmds = self._desaturate(motor_cmds, thrust_scalar)

        # Clamp to [0, 1]
        motor_cmds = np.clip(motor_cmds, 0.0, 1.0)

        # ── Publish ───────────────────────────────────────────────────
        msg = ActuatorMotors()
        for i in range(4):
            msg.control[i] = float(motor_cmds[i])
        # Remaining channels unused
        for i in range(4, 12):
            msg.control[i] = float('nan')
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.motor_pub.publish(msg)

    def _desaturate(self, cmds: np.ndarray, thrust: float) -> np.ndarray:
        """
        PX4 airmode=disabled desaturation:
        If any motor < 0, reduce all torque contributions (not thrust)
        until the minimum motor is exactly 0.
        Mirrors PX4 control_allocator desaturation logic.
        """
        min_cmd = cmds.min()
        max_cmd = cmds.max()

        if min_cmd < 0.0:
            # Scale torque portion down
            # Recompute with reduced torques
            scale = 1.0
            if max_cmd - min_cmd > 1e-6:
                scale = min(1.0, thrust / (thrust - min_cmd + 1e-6))
            torque_scaled = self.torque * scale
            control_scaled = np.array([
                torque_scaled[0], torque_scaled[1], torque_scaled[2], thrust
            ])
            cmds = self._B_pinv @ control_scaled

        if cmds.max() > 1.0:
            # Scale everything down to keep max at 1
            cmds /= cmds.max()

        return cmds


# ═══════════════════════════════════════════════════════════════════════════════
# Entry points — run each node in its own process for real performance
# ═══════════════════════════════════════════════════════════════════════════════

def run_attitude_control(args=None):
    rclpy.init(args=args)
    node = AttitudeControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def run_rate_control(args=None):
    rclpy.init(args=args)
    node = RateControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def run_mixer(args=None):
    rclpy.init(args=args)
    node = MixerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

import sys


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'attitude'

    if mode == 'attitude':
        run_attitude_control()

    elif mode == 'rate':
        run_rate_control()

    elif mode == 'mixer':
        run_mixer()

    else:
        print("Usage: ros2 run px4_sitl_scripts inner_loop_nodes [attitude|rate|mixer]")


if __name__ == '__main__':
    main()

"""if __name__ == '__main__':
    import sys
    mode = sys.argv[1] if len(sys.argv) > 1 else 'attitude'
    if mode == 'attitude':
        run_attitude_control()
    elif mode == 'rate':
        run_rate_control()
    elif mode == 'mixer':
        run_mixer()
    else:
        print("Usage: python px4_inner_loop_nodes.py [attitude|rate|mixer]")"""