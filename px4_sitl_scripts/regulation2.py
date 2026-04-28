import control
import numpy as np

s = control.tf('s')
g      = 9.81
Kp_pos = 0.95
Kp_vel = 1.8;  Ki_vel = 0.4;  Kd_vel = 0.2;  tau_d = 0.00
Kp_att = 4.0
Kp_rate = 0.15; Ki_rate = 0.2; Kd_rate = 0.003

# Controllers
C_pos = Kp_pos
C_vel = control.tf([Kd_vel, Kp_vel, Ki_vel], [1, 0])
C_acc = 1 / g
C_att = Kp_att
C_rate = control.tf([Kd_rate, Kp_rate, Ki_rate], [1, 0])

#G_rate = control.feedback(C_rate * control.tf([1], [1, 0]), 1)
G_att = control.feedback(C_att * control.tf([1], [1, 0]), 1)
G_vel = control.feedback(C_vel * C_acc * control.tf([g], [1, 0]) * G_att, 1)
G_pos = control.feedback(C_pos * control.tf([1], [1, 0]) * G_vel, 1)

"""# Blocks
G_att    = control.tf([Kp_att], [1, Kp_att])
C_vel    = (1/g) * (Kp_vel + control.tf([Ki_vel],[1,0])
                            + control.tf([Kd_vel,0],[tau_d,1]))

# Cascade
G_vel_plant = G_att * control.tf([g], [1, 0])       # θ_cmd → ẋ
G_vel_cl    = control.feedback(C_vel * G_vel_plant, 1)
G_pos_plant = G_vel_cl * control.tf([1], [1, 0])    # ẋ → x
G_full      = control.feedback(Kp_pos * G_pos_plant, 1)

# Minimal realisation
G_min = control.minreal(G_full, verbose=False)"""
print(G_pos)
system2 = G_pos

#!/usr/bin/env python3

import numpy as np
import control
import matplotlib.pyplot as plt
import pandas as pd

# Load CSV
df1 = pd.read_csv("step_1/data.csv")
df2 = pd.read_csv("step_1_v3/data.csv")

#print(df.iloc[0])
#print(df['/fmu/out/vehicle_odometry/position[0]'])

#data1 = df1['/fmu/out/vehicle_odometry/position[0]'].values[200:1200]
data1 = df1['/fmu/out/vehicle_odometry/position[0]'].values[4520:5520]
data2 = df2['/fmu/out/vehicle_odometry/position[0]'].values[4200:5200]
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
#t, y = control.forced_response(system, T=t, U=np.ones_like(t))
t, y = control.forced_response(system2, T=t, U=np.ones_like(t))

plt.figure(figsize=(9, 4))
plt.plot(t, y)
plt.plot(t1, data1)
#plt.plot(t, y2)
plt.plot(t2, data2)
plt.title('Step Response of X-axis position')
plt.xlabel('Time (s)')
plt.ylabel('X-axis position (m)')
plt.legend(['Transfer function', 'PX4-Autopilot data (#1)', 'PX4-Autopilot data (#2)'])
plt.grid()
plt.show()