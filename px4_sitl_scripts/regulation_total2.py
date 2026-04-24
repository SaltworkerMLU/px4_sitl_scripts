import numpy as np
import matplotlib.pyplot as plt
import control
import pandas as pd

# Load CSV
df = pd.read_csv("data.csv")

#print(df.iloc[0])
#print(df['/fmu/out/vehicle_odometry/position[0]'])

data = df['/fmu/out/vehicle_odometry/position[0]'].values[750:2250]

# Parameters
g = 9.81
Jy = 0.2
P_r = 0.95 # 0.95 # 
P_v = 1.8 # 1.8
I_v = 0.4
D_v = 0.2

# Design parameters
zeta_theta = 0.7
omega_theta = 10.0

zeta_x = 0.7
omega_x = 2.0

# Gains
Kp_theta = Jy * omega_theta**2
Kd_theta = 2 * Jy * zeta_theta * omega_theta

# Laplace variable
s = control.TransferFunction.s

# Plants
plant_theta = control.tf([1], [Jy, 0, 0])
plant_r = control.tf([1], [1, 0])
plant_v = control.tf([9.81], [1, 0])

# Controllers
controller_theta = control.tf([Kd_theta, Kp_theta], [1])
controller_v = 1/9.81 * control.tf([D_v, P_v, I_v], [1, 0])
controller_r = P_r

# Inner loop closed
T_theta = control.feedback(controller_theta * plant_theta, 1)
T_v = control.feedback(controller_v * plant_v * T_theta, 1)
T_x = control.feedback(controller_r * plant_r * T_v, 1)

# Time vector
t = np.linspace(0, 15, 1500)
t1 = np.linspace(0, 15, 1500)

# Simulate
t_out, y_out = control.forced_response(T_x, t, u=np.ones_like(t))

print(T_x)

# Plot position
plt.figure()
plt.plot(t_out, y_out, label="x position")
plt.plot(t1, data, label="PX4 data")
plt.plot(t_out, np.ones_like(t), "--", label="reference")
plt.xlabel("Time [s]")
plt.ylabel("Position [m]")
plt.title("Cascaded Control (Transfer Function)")
plt.legend()
plt.grid()

plt.show()