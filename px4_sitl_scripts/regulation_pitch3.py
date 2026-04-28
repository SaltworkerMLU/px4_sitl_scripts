#!/usr/bin/env python3
import numpy as np
import control
import matplotlib.pyplot as plt
import pandas as pd

P_r = 0.95 # 1.7 # cd
P_v = 1.8 # 2.9
I_v = 0.4
D_v = 0.2

# Load CSV
df1 = pd.read_csv("step_1/data.csv")

# --- Time ---
time_raw = df1['/fmu/out/vehicle_odometry/timestamp'].values
time_raw = (time_raw - time_raw[0]) * 1e-6  # convert microseconds to seconds

# --- Slice indices for the step event ---
idx_start = 4500
idx_end   = 5500
N = idx_end - idx_start

# --- Position data ---
x_pos = df1['/fmu/out/vehicle_odometry/position[0]'].values[idx_start:idx_end]
x_pos = x_pos - x_pos[0]  # normalize to start at 0

# --- Time slice ---
t_slice = time_raw[idx_start:idx_end]
t_slice = t_slice - t_slice[0]  # start at t=0

# --- Quaternion to Euler (PX4 format: q = [w, x, y, z]) ---
quats = df1[[
    '/fmu/out/vehicle_odometry/q[0]',   # w
    '/fmu/out/vehicle_odometry/q[1]',   # x
    '/fmu/out/vehicle_odometry/q[2]',   # y
    '/fmu/out/vehicle_odometry/q[3]'    # z
]].to_numpy()

def quaternion_to_euler_px4(q):
    """PX4 quaternion convention: q = [w, x, y, z]"""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]

    # Roll
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)

    # Pitch
    t2 = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = np.arcsin(t2)

    # Yaw
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)

    return roll, pitch, yaw

roll_all, pitch_all, yaw_all = quaternion_to_euler_px4(quats)
pitch_slice = pitch_all[idx_start:idx_end]  # radians

# --- Transfer function: pitch angle (rad) -> x-position (m) ---
# G(s) = (0.19s^2 + 1.71s + 0.38) / (1.2s^3 + 1.99s^2 + 2.11s + 0.38)
# Create transfer function G(s)
plant_vr = control.tf([1], [1, 0])
plant_av = control.tf([9.81], [1, 0])

controller_av = 1/9.81 * control.tf([D_v, P_v, I_v], [1, 0])
controller_vr = P_r

system_av = control.feedback(controller_av * plant_av, 1)
G = control.feedback(controller_vr * plant_vr * system_av, 1)

G = -plant_vr * plant_av
print("Plant TF:")
print(G)

# --- Use actual pitch as input to the plant TF (Option C) ---
# Resample onto a uniform time grid (required by forced_response)
dt = np.mean(np.diff(t_slice))
t_uniform = np.arange(0, t_slice[-1], dt)
pitch_uniform = np.interp(t_uniform, t_slice, pitch_slice)

# Use pitch signal as input U to the plant
t_out, x_pos_tf = control.forced_response(G, T=t_uniform, U=pitch_uniform)

# Normalize TF output to start at 0
x_pos_tf = x_pos_tf - x_pos_tf[0]

# --- Plot ---
fig = plt.plot()

plt.plot(t_uniform, x_pos_tf, 'b-', label='TF output (driven by real pitch)')
plt.plot(t_slice, x_pos, color='orange', label='PX4 actual x-position')
plt.title('Option C: X-position — TF driven by real pitch vs PX4 actual')
plt.xlabel('Time (s)')
plt.ylabel('X-axis position (m)')
#plt.ylim(-0.5, 1.5)
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()