"""
X-axis Position → Roll Angle Controller for PX4 / ROS2 (X500)
==============================================================
 
Transfer function (outer position loop):
    C(s) = -(1/g) * [Kp + Kd * s / (tau*s + 1)]
 
Plant model (lateral dynamics):
    P(s) = g / s^2   (double integrator, φ → x)
 
IMU accelerometer is used instead of a velocity measurement.
The lateral accelerometer aₓ is integrated through a low-pass
filter to produce a velocity estimate v̂ₓ, avoiding the need
for EKF-estimated velocity.
 
Complementary filter structure:
    v̂ₓ[k] = alpha * (v̂ₓ[k-1] + aₓ[k] * dt)
              + (1 - alpha) * (x_meas[k] - x_meas[k-1]) / dt
 
Usage example (inside a ROS2 timer callback):
    controller = PositionToRollController(Kp=1.2, Kd=0.8, tau=0.1)
    phi_cmd = controller.update(x_ref, x_meas, ax_imu, dt)
"""
 
import numpy as np
 
 
class PositionToRollController:
    def __init__(
        self,
        Kp: float = 1.2,
        Kd: float = 0.8,
        tau: float = 0.1,
        g: float = 9.81,
        phi_max: float = np.radians(20.0),
    ):
        self.Kp = Kp
        self.Kd = Kd
        self.tau = tau
        self.g = g
        self.phi_max = phi_max

        # States
        self._x_prev: float | None = None
        self._d_state: float = 0.0  # filtered derivative state

    def reset(self):
        self._x_prev = None
        self._d_state = 0.0

    def update(self, x_ref, x_meas, dt):
        if dt <= 0.0:
            return 0.0

        # ── Position error ─────────────────────────────
        e = x_ref - x_meas

        # ── Filtered derivative (matches s/(τs+1)) ─────
        if self._x_prev is None:
            dx = 0.0
        else:
            dx = (x_meas - self._x_prev)

        self._x_prev = x_meas

        alpha = self.tau / (self.tau + dt)

        self._d_state = (
            alpha * self._d_state
            + (1.0 - alpha) * (dx / dt)
        )

        # ── Control law (exact TF form) ────────────────
        phi_cmd = -(1.0 / self.g) * (
            self.Kp * e + self.Kd * self._d_state
        )

        # ── Saturation ────────────────────────────────
        return float(np.clip(phi_cmd, -self.phi_max, self.phi_max))


class CascadedPositionController:
    def __init__(
        self,
        # Position loop
        Kp_pos: float = 1.0,

        # Velocity loop
        Kp_vel: float = 2.0,
        Kd_vel: float = 0.3,
        tau_vel: float = 0.1,

        g: float = 9.81,
        phi_max: float = np.radians(20.0),
    ):
        self.Kp_pos = Kp_pos

        self.Kp_vel = Kp_vel
        self.Kd_vel = Kd_vel
        self.tau_vel = tau_vel

        self.g = g
        self.phi_max = phi_max

        # States
        self._x_prev = None
        self._v_est = 0.0
        self._d_vel = 0.0

    def reset(self):
        self._x_prev = None
        self._v_est = 0.0
        self._d_vel = 0.0

    def update(self, x_ref, x_meas, dt):
        if dt <= 0.0:
            return 0.0

        # ─────────────────────────────
        # 1. Velocity estimate
        # ─────────────────────────────
        if self._x_prev is None:
            v_meas = 0.0
        else:
            v_meas = (x_meas - self._x_prev) / dt

        self._x_prev = x_meas

        # ─────────────────────────────
        # 2. Position loop (P)
        # ─────────────────────────────
        pos_error = x_ref - x_meas
        v_ref = self.Kp_pos * pos_error

        # ─────────────────────────────
        # 3. Velocity loop (PD)
        # ─────────────────────────────
        vel_error = v_ref - v_meas

        # Filtered derivative of velocity
        alpha = self.tau_vel / (self.tau_vel + dt)
        self._d_vel = (
            alpha * self._d_vel
            + (1 - alpha) * (v_meas)
        )

        a_cmd = (
            self.Kp_vel * vel_error
            - self.Kd_vel * self._d_vel
        )

        # ─────────────────────────────
        # 4. Acceleration → roll
        # ─────────────────────────────
        phi_cmd = a_cmd / self.g

        # PX4 sign convention (NED)
        phi_cmd = -phi_cmd

        return float(np.clip(phi_cmd, -self.phi_max, self.phi_max))
 
# ──────────────────────────────────────────────────────────────────────
# ROS2 integration example (drop this into your node)
# ──────────────────────────────────────────────────────────────────────
 

"""import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorCombined, VehicleLocalPosition, VehicleAttitudeSetpoint
from position_to_roll_controller import PositionToRollController
import numpy as np
 
class PositionRollNode(Node):
    def __init__(self):
        super().__init__('position_roll_controller')
 
        self.ctrl = PositionToRollController(Kp=1.2, Kd=0.8, tau=0.1)
        self.x_ref = 0.0
        self.x_meas = 0.0
        self.ax_imu = 0.0
        self.last_time = self.get_clock().now()
 
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.pos_cb, 10
        )
        self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.imu_cb, 10
        )
        self.att_pub = self.create_publisher(
            VehicleAttitudeSetpoint,
            '/fmu/in/vehicle_attitude_setpoint', 10
        )
        self.create_timer(0.02, self.control_loop)  # 50 Hz
 
    def pos_cb(self, msg):
        self.x_meas = msg.x   # NED x [m]
 
    def imu_cb(self, msg):
        # SensorCombined: accelerometer_m_s2[0] = ax (body frame)
        self.ax_imu = msg.accelerometer_m_s2[0]
 
    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
 
        phi_cmd = self.ctrl.update(self.x_ref, self.x_meas, self.ax_imu, dt)
 
        msg = VehicleAttitudeSetpoint()
        msg.roll_body = phi_cmd
        msg.pitch_body = 0.0
        msg.yaw_body = 0.0
        msg.thrust_body[2] = -0.5   # adjust as needed
        self.att_pub.publish(msg)"""

 
 
# ──────────────────────────────────────────────────────────────────────
# Quick self-test / simulation
# ──────────────────────────────────────────────────────────────────────
 
if __name__ == "__main__":
    import matplotlib.pyplot as plt
 
    g = 9.81
    dt = 0.02          # 50 Hz
    T = 10.0           # simulation duration [s]
    steps = int(T / dt)
 
    ctrl = CascadedPositionController() # PositionToRollController(Kp=1.2, Kd=0.8, tau=0.1, g=9.81)
 
    # Simple double-integrator plant simulation
    x = 0.0
    vx = 0.0
    x_ref = 2.0        # step to 2 m
 
    log_t, log_x, log_phi, log_vhat = [], [], [], []
 
    for i in range(steps):
        t = i * dt
 
        # Simulate IMU ax (= φ * g, with small noise)
        ax_true = -np.sin(phi_prev if i > 0 else 0.0) * g
        ax_imu = ax_true + np.random.normal(0, 0.05)
 
        phi_cmd = ctrl.update(x_ref, x, dt) # phi_cmd = ctrl.update(x_ref, x, dt)
        phi_prev = phi_cmd
 
        # Plant: ẍ = g * sin(φ) ≈ g * φ
        ax_plant = -g * np.sin(phi_cmd)
        vx += ax_plant * dt
        x += vx * dt
 
        log_t.append(t)
        log_x.append(x)
        log_phi.append(np.degrees(phi_cmd))
        #log_vhat.append(ctrl.velocity_estimate)
 
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    axes[0].plot(log_t, log_x, label="x_meas")
    axes[0].axhline(x_ref, color="r", linestyle="--", label="x_ref")
    axes[0].set_ylabel("Position [m]")
    axes[0].legend()
 
    axes[1].plot(log_t, log_phi, color="orange", label="φ_cmd")
    axes[1].set_ylabel("Roll cmd [deg]")
    axes[1].legend()
 
    #axes[2].plot(log_t, log_vhat, color="green", label="v̂ₓ (comp. filter)")
    #axes[2].set_ylabel("Vel. estimate [m/s]")
    #axes[2].set_xlabel("Time [s]")
    #axes[2].legend()
 
    plt.suptitle("X-position → Roll controller simulation")
    #plt.tight_layout()
    #plt.savefig("/mnt/user-data/outputs/simulation_result.png", dpi=120)
    plt.show()
    print("Simulation complete. Plot saved.")