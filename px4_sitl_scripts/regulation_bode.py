import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import detrend, welch, csd, coherence

# ==============================
# USER SETTINGS
# ==============================

idx_start = 4500
idx_end   = 5500
N = idx_end - idx_start

CSV_FILE = "step_1/data.csv"

# Column names in your CSV
TIME_COL = "/fmu/out/vehicle_odometry/timestamp"   # seconds (or microseconds—see below)
#INPUT_COL = "/fmu/out/vehicle_odometry/"     # e.g. rate setpoint
OUTPUT_COL = "/fmu/out/vehicle_odometry/position[0]"    # e.g. measured rate


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

# If timestamps are in microseconds (PX4 ulog default), set this True
TIME_IN_MICROSECONDS = True

# Welch settings
NPERSEG = 1024

# ==============================
# LOAD DATA
# ==============================

df = pd.read_csv(CSV_FILE)

# Convert time to seconds if needed
time = df[TIME_COL].values[idx_start:idx_end].astype(float)
if TIME_IN_MICROSECONDS:
    time = time * 1e-6

#u = df[INPUT_COL].values.astype(float)
y = df[OUTPUT_COL].values[idx_start:idx_end].astype(float)

# --- Quaternion to Euler (PX4 format: q = [w, x, y, z]) ---
quats = df[[
    '/fmu/out/vehicle_odometry/q[0]',   # w
    '/fmu/out/vehicle_odometry/q[1]',   # x
    '/fmu/out/vehicle_odometry/q[2]',   # y
    '/fmu/out/vehicle_odometry/q[3]'    # z
]].to_numpy()

roll_all, pitch_all, yaw_all = quaternion_to_euler_px4(quats)
pitch_slice = pitch_all[idx_start:idx_end]  # radians
u = pitch_slice

# ==============================
# RESAMPLE TO UNIFORM GRID
# ==============================

# Create uniform time vector
dt = np.mean(np.diff(time))
t_uniform = np.arange(time[0], time[-1], dt)

# Interpolate signals
u_interp = np.interp(t_uniform, time, u)
y_interp = np.interp(t_uniform, time, y)

fs = 1.0 / dt  # sampling frequency

# ==============================
# PREPROCESSING
# ==============================

# Detrend
u_d = detrend(u_interp)
y_d = detrend(y_interp)

# Apply window (optional but recommended)
window = np.hanning(len(u_d))
u_d *= window
y_d *= window

# ==============================
# FREQUENCY RESPONSE (Welch/CSD)
# ==============================

f, Puu = welch(u_d, fs=fs, nperseg=NPERSEG)
_, Pyy = welch(y_d, fs=fs, nperseg=NPERSEG)
_, Pyu = csd(y_d, u_d, fs=fs, nperseg=NPERSEG)

H = Pyu / Puu

import control

sys = control.frd(H, f * 2*np.pi)  # Hz → rad/s
control.bode_plot(sys, title="Estimated Bode Plot from Data", dB=True)
plt.grid()
plt.show()

# ==============================
# BODE DATA
# ==============================

magnitude = 20 * np.log10(np.abs(H))
phase = np.angle(H, deg=True)

# ==============================
# COHERENCE (QUALITY CHECK)
# ==============================

f_coh, coh = coherence(u_d, y_d, fs=fs, nperseg=NPERSEG)

# ==============================
# PLOTTING
# ==============================

plt.figure(figsize=(10, 8))

# Magnitude
plt.subplot(3, 1, 1)
plt.semilogx(f, magnitude)
plt.ylabel("Magnitude (dB)")
plt.title("Bode Plot")
plt.grid(True, which="both")

# Phase
plt.subplot(3, 1, 2)
plt.semilogx(f, phase)
plt.ylabel("Phase (deg)")
plt.grid(True, which="both")

# Coherence
plt.subplot(3, 1, 3)
plt.semilogx(f_coh, coh)
plt.ylabel("Coherence")
plt.xlabel("Frequency (Hz)")
plt.grid(True, which="both")

plt.tight_layout()
plt.show()

# ==============================
# OPTIONAL: PRINT BASIC INFO
# ==============================

print(f"Sampling frequency: {fs:.2f} Hz")
print(f"Data length: {len(u_d)} samples")