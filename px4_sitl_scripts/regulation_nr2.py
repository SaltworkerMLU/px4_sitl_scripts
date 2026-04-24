#!/usr/bin/env python3

import numpy as np
import control
import matplotlib.pyplot as plt
import pandas as pd

# Load CSV
df = pd.read_csv("data.csv")

#print(df.iloc[0])
#print(df['/fmu/out/vehicle_odometry/position[0]'])

data = df['/fmu/out/vehicle_odometry/position[0]'].values[450:1950]

P_r = 1.0 # 0.95 # 
P_v = 0.6 # 1.8
I_v = 0.0
D_v = 1.2

# Create transfer function G(s)
plant_vr = control.tf([1], [1, 0])
plant_av = control.tf([9.81], [1, 0])

controller_av = 1/9.81 * control.tf([D_v, P_v, I_v], [1, 0])
controller_vr = P_r

system_av = controller_av * plant_av # control.feedback(, 1)
system = control.feedback(controller_vr * plant_vr * system_av, 1)

print(system)

# Time vector
t = np.linspace(0, 15, 1500)
t1 = np.linspace(0, 15, 1500)

# Compute step response
t, y = control.forced_response(system, T=t, U=np.ones_like(t))

plt.figure()
plt.plot(t, y)
plt.plot(t1, data)
plt.title('Step Response of X-axis position')
plt.xlabel('Time (s)')
plt.ylabel('X-axis position (m)')
plt.legend(['Transfer function', 'PX4-Autopilot data'])
plt.grid()
plt.show()