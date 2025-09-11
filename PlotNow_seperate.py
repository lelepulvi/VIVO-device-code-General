import pandas as pd
import matplotlib.pyplot as plt

import os

# Set input file
trial_no = 1
date_and_time = "20250911_103705"

input_dir = f"Experiment_{date_and_time}"
file_name = f"trial_{trial_no}_{date_and_time}.csv"  # Change to your trial filename
file_path = os.path.join(input_dir, file_name)

# === Load your CSV ===
#file_path = "trial_1_20250808_052600.csv"  # <-- change path if needed
df = pd.read_csv(file_path)

# === Plot 1: Loadcells ===
plt.figure(figsize=(10, 5))
plt.plot(df['Time'], df['R Loadcell'], label="R Loadcell (N)")
plt.plot(df['Time'], df['L Loadcell'], label="L Loadcell (N)")
plt.xlabel("Time (s)")
plt.ylabel("Loadcell (N)")
plt.title("Loadcells vs Time")
plt.legend()
plt.grid(True)
plt.show()

# === Plot 2: Pressures ===
plt.figure(figsize=(10, 5))
pressure_cols = [
    'Pressure R BAM UP', 'Pressure L BAM UP',   # need to change
    'Pressure R BAM Down', 'Pressure L BAM Down'
]
for col in pressure_cols:
    plt.plot(df['Time'], df[col], label=col)
plt.xlabel("Time (s)")
plt.ylabel("Pressure (kPa)")
plt.title("Pressures vs Time")
plt.legend()
plt.grid(True)
plt.show()

# === Plot 3: FSRs ===
plt.figure(figsize=(10, 5))
fsr_cols = ['FSR R Heel', 'FSR R Toe', 'FSR L Heel', 'FSR L Toe']
for col in fsr_cols:
    plt.plot(df['Time'], df[col], label=col)
plt.xlabel("Time (s)")
plt.ylabel("FSR")
plt.title("FSRs vs Time")
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(10, 5))
fsr_cols = ['FSR R Heel', 'FSR L Heel']
for col in fsr_cols:
    plt.plot(df['Time'], df[col], label=col)
plt.xlabel("Time (s)")
plt.ylabel("FSR")
plt.title("FSRs vs Time")
plt.legend()
plt.grid(True)
plt.show()
# === Plot 4+: IMUs ===
imu_groups = [
    ('rtTime', 'R Thigh Angle'),
    ('rsTime', 'R Shank Angle'),
    ('ltTime', 'L Thigh Angle'),
    ('lsTime', 'L Shank Angle'),
    ('hTime', 'Hip Angle')
]

for time_col, angle_col in imu_groups:
    plt.figure(figsize=(10, 5))
    plt.plot(df[time_col], df[angle_col], label=angle_col)
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (deg)")
    plt.title(f"{angle_col} vs {time_col}")
    plt.legend()
    plt.grid(True)
    plt.show()

"""
import pandas as pd
import matplotlib.pyplot as plt
import os

# Set input file
input_dir = "Experiment_20250808_055115"
file_name = "trial_2_20250808_055115.csv"  # Change to your trial filename
file_path = os.path.join(input_dir, file_name)

# Load CSV file
df = pd.read_csv(file_path)

# Time column (in seconds)
time = df["Time"]

# --- 1. Time vs Loadcell Forces ---
plt.figure(figsize=(10, 5))
plt.plot(time, df["R Loadcell"], label="Right Loadcell (N)")
plt.plot(time, df["L Loadcell"], label="Left Loadcell (N)")
plt.xlabel("Time (s)")
plt.ylabel("Force (N)")
plt.title("Time vs Loadcell Forces")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# --- 2. Time vs BAM Pressures (already in kPa) ---
pressure_cols = [
    "Pressure R BAM UP", "Pressure R BAM Down",
    "Pressure L BAM UP", "Pressure L BAM Down"
]

plt.figure(figsize=(10, 5))
for col in pressure_cols:
    if col in df.columns:
        plt.plot(time, df[col], label=col)
plt.xlabel("Time (s)")
plt.ylabel("Pressure (kPa)")
plt.title("Time vs BAM Pressures")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# --- 3–7. Time vs Joint Angles (in degrees) ---
angle_cols = [
    "R Thigh Angle",
    "R Shank Angle",
    "L Thigh Angle",
    "L Shank Angle",
    "Hip Angle"
]

for col in angle_cols:
    if col in df.columns:
        plt.figure(figsize=(10, 4))
        plt.plot(time, df[col])
        plt.xlabel("Time (s)")
        plt.ylabel("Angle (°)")
        plt.title(f"Time vs {col}")
        plt.grid(True)
        plt.tight_layout()
        plt.show()
"""