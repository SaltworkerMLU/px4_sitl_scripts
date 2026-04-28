#!/usr/bin/env python3

import numpy as np
import control
import matplotlib.pyplot as plt
import pandas as pd

# Load CSV
df1 = pd.read_csv("step_1/data.csv")
df2 = pd.read_csv("step_10_v2/data.csv")

#print(df.iloc[0])
#print(df['/fmu/out/vehicle_odometry/position[0]'])

data1 = df1['/fmu/out/vehicle_odometry/position[0]'].values[4520:5520]
data2 = df2['/fmu/out/vehicle_odometry/position[0]'].values[3750:4750]
#data2 = df2['/fmu/out/vehicle_odometry/position[0]'].values[3370:4370]

P_r = 0.95 # 1.7 # cd
P_v = 1.8 # 2.9
I_v = 0.4
D_v = 0.2

# Create transfer function G(s)
plant_vr = control.tf([1], [1, 0])
plant_av = control.tf([9.81], [1, 0])

controller_av = 1/9.81 * control.tf([D_v, P_v, I_v], [1, 0])
controller_vr = P_r

system_av = control.feedback(controller_av * plant_av, 1)
system = control.feedback(controller_vr * plant_vr * system_av, 1)

print(system)

# Time vector
t = np.linspace(0, 10, 1500)
t1 = np.linspace(0, 10, 1000)
t2 = np.linspace(0, 10, 1000)

# Compute step response
t, y = control.forced_response(system, T=t, U=np.ones_like(t))

plt.figure()
plt.plot(t, y)
plt.plot(t1, data1)
plt.plot(t2, data2)
plt.title('Step Response of X-axis position')
plt.xlabel('Time (s)')
plt.ylabel('X-axis position (m)')
plt.legend(['Transfer function', 'PX4-Autopilot data (#1)', 'PX4-Autopilot data (#2)'])
plt.grid()
plt.show()