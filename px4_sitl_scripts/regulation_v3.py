import numpy as np
import control
import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv("data.csv")

g      = 9.81
Kp_pos = 0.95   # MPC_XY_P
Kp_vel = 1.8    # MPC_XY_VEL_P_ACC
Ki_vel = 0.4    # MPC_XY_VEL_I_ACC
Kd_vel = 0.2    # MPC_XY_VEL_D_ACC

# ── Plant: phi_cmd → x, assuming perfect attitude tracking ────────
# a = g·phi_cmd,  x = a/s² = g/s²
plant = control.tf([g], [1, 0, 0])

# ── Velocity loop (PD forward, no D-on-measurement complication) ──
# PX4 velocity controller: Kp·e + Ki·∫e - Kd·v
# With D on measurement and perfect attitude, the closed velocity loop is:
#
#   C_v(s) = Kp + Ki/s      (forward)
#   plant_v = g/s            (phi_cmd → velocity)
#   D feedback = Kd          (static, since tau_d << position timescale)
#
# Simplified: lump Kd into an equivalent damping → use PID on error
# This matches PX4 to first order without adding extra poles:
vel_ctrl  = control.tf([Kd_vel, Kp_vel, Ki_vel], [1, 0])

# Velocity plant: phi_cmd → velocity = g/s
vel_plant = control.tf([1], [1, 0])

# Closed velocity loop
vel_cl = control.feedback(vel_ctrl * vel_plant, 1)

# ── Position loop (P) ─────────────────────────────────────────────
pos_ctrl  = control.tf([Kp_pos], [1])
pos_plant = control.tf([1], [1, 0])   # velocity → position

# Full closed loop
system = control.feedback(pos_ctrl * pos_plant * vel_cl, 1)

print("Transfer function:")
print(system)
print("\nClosed-loop poles:", control.poles(system))

# ── Data ──────────────────────────────────────────────────────────
dt_csv  = 1.0 / 100.0
t_start = 750
n_samp  = 750
data    = df['/fmu/out/vehicle_odometry/position[0]'].values[t_start:2250]
t_data  = np.arange(len(data)) * dt_csv

# ── Simulate ──────────────────────────────────────────────────────
t_sim = np.linspace(0, t_data[-1], len(t_data))
_, y  = control.forced_response(system, T=t_sim, U=1.0 * np.ones_like(t_sim))

# ── Plot ──────────────────────────────────────────────────────────
plt.figure(figsize=(10, 5))
plt.plot(t_sim,  y,    label='Model response', linewidth=2)
plt.plot(t_data, data, label='PX4 SITL data',  linewidth=2, linestyle='--')
plt.axhline(2.0, color='gray', linestyle=':', label='Setpoint')
plt.title('Closed-loop step response: model vs PX4 SITL')
plt.xlabel('Time [s]')
plt.ylabel('X-axis position [m]')
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()