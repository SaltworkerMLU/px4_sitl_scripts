#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Load CSV
df = pd.read_csv("data.csv")
data = df['/fmu/out/vehicle_odometry/position[0]'].values[750:2250]

# Parameters
g = 9.81

# Controller gains
P_r = 0.95
P_v = 1.8
I_v = 0.4
D_v = 0.2

# Simulation settings
dt = 0.01
T = 15
t = np.arange(0, T, dt)

# States
x = 0.0
v = 0.0

# Integrator state for velocity controller
v_int = 0.0
v_prev_error = 0.0

# Reference
x_ref = 1.0

# Logging
x_hist = []
v_hist = []

for _ in t:
    # --- Outer loop (position → velocity reference) ---
    v_ref = P_r * (x_ref - x)

    # --- Inner loop (velocity PID → normalized acceleration command) ---
    v_error = v_ref - v
    v_int += v_error * dt
    v_der = (v_error - v_prev_error) / dt

    # NOTE: divide by g to match your TF structure
    a_cmd = (P_v * v_error + I_v * v_int + D_v * v_der) / g

    v_prev_error = v_error

    # --- Plant dynamics ---
    # acceleration = g * tilt_command → cancels with division above
    x_ddot = g * a_cmd

    # Integrate
    v += x_ddot * dt
    x += v * dt

    # Log
    x_hist.append(x)
    v_hist.append(v)

# --- Plot ---
plt.figure()
plt.plot(t, x_hist, label="Simulated")
plt.plot(t[:len(data)], data, label="PX4-Autopilot data")
plt.title('Step Response of X-axis position')
plt.xlabel('Time (s)')
plt.ylabel('X-axis position (m)')
plt.legend()
plt.grid()

plt.figure()
plt.plot(t, v_hist, label="Velocity")
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Velocity response')
plt.grid()

plt.show()