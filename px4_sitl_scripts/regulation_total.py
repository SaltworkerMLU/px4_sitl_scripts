import numpy as np
import matplotlib.pyplot as plt
import control as ctrl
import pandas as pd

# Load CSV
df = pd.read_csv("data.csv")

#print(df.iloc[0])
#print(df['/fmu/out/vehicle_odometry/position[0]'])

data = df['/fmu/out/vehicle_odometry/position[0]'].values[750:2250]

# Parameters
g = 9.81
Jy = 0.02

# Design parameters
zeta_theta = 0.7
omega_theta = 10.0

zeta_x = 0.7
omega_x = 2.0

# Gains
Kp_theta = Jy * omega_theta**2
Kd_theta = 2 * Jy * zeta_theta * omega_theta

Kp_x = omega_x**2 / g
Kd_x = 2 * zeta_x * omega_x / g

# Laplace variable
s = ctrl.TransferFunction.s

# Plants
G_theta = 1 / (Jy * s**2)
G_x = g / s**2

# Controllers
C_theta = Kp_theta + Kd_theta * s
C_x = Kp_x + Kd_x * s

# Inner loop closed
T_theta = ctrl.feedback(C_theta * G_theta, 1)

# Outer loop plant
G_outer = G_x * T_theta

# Outer loop closed
T_x = ctrl.feedback(C_x * G_outer, 1)

# Time vector
t = np.linspace(0, 15, 1500)
t1 = np.linspace(0, 15, 1500)

# Step input
u = np.ones_like(t)

# Simulate
t_out, y_out = ctrl.forced_response(T_x, t, u)

# Plot position
plt.figure()
plt.plot(t_out, y_out, label="x position")
plt.plot(t1, data, label="PX4 data")
plt.plot(t_out, u, "--", label="reference")
plt.xlabel("Time [s]")
plt.ylabel("Position [m]")
plt.title("Cascaded Control (Transfer Function)")
plt.legend()
plt.grid()

plt.show()