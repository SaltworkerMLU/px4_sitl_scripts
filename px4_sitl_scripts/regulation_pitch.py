#!/usr/bin/env python3

import numpy as np
import control
import matplotlib.pyplot as plt
import pandas as pd

# Load CSV
df1 = pd.read_csv("step_1/data.csv")
#df2 = pd.read_csv("PID_step_1/data.csv")

#print(df.iloc[0])
#print(df['/fmu/out/vehicle_odometry/position[0]'])

data1 = df1['/fmu/out/vehicle_odometry/position[0]'].values[4500:5500]

quats = pd.DataFrame(df1, columns=['/fmu/out/vehicle_odometry/q[0]', 
                                   '/fmu/out/vehicle_odometry/q[1]', 
                                   '/fmu/out/vehicle_odometry/q[2]', 
                                   '/fmu/out/vehicle_odometry/q[3]']).to_numpy()

def quaternion_to_euler(w, x, y, z):
    # Roll (x-axis rotation)
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)

    # Pitch (y-axis rotation)
    t2 = 2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)  # avoid invalid values due to floating point errors
    pitch = np.arcsin(t2)

    # Yaw (z-axis rotation)
    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)

    return roll, pitch, yaw

_,pitch, _ = quaternion_to_euler(quats[:,0], quats[:,1], quats[:,2], quats[:,3])

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
plt.plot(t1, pitch[4500:5500])
plt.plot(t2, data1)
plt.title('Step Response of X-axis position')
plt.xlabel('Time (s)')
plt.ylabel('X-axis position (m)')
plt.legend(['Transfer function', 'PX4-Autopilot data', 'Designed Model data'])
plt.grid()
plt.show()