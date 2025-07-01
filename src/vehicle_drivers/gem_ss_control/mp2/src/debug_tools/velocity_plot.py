import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load CSVs
dgps = pd.read_csv("src/mp2/src/dGPS.csv")
dlidar = pd.read_csv("src/mp2/src/dlidar.csv")

# Define matching tolerance in seconds
tolerance = 0.005

# Prepare data holders
matched_timestamps = []
matched_dgps = []
matched_dlidar = []
matched_velocities = []
dgps_lidar_diff = []

# Extract values
dgps_times = dgps["timestamp"].values
dlidar_times = dlidar["timestamp"].values

for i, dgps_time in enumerate(dgps_times):
    diffs = np.abs(dlidar_times - dgps_time)
    min_idx = np.argmin(diffs)
    if diffs[min_idx] <= tolerance:
        dgps_dist = dgps["distance"].iloc[i]
        dlidar_dist = dlidar["dlidar"].iloc[min_idx]
        vel = dgps["vel"].iloc[i]

        matched_timestamps.append(dgps_time)
        matched_dgps.append(dgps_dist)
        matched_dlidar.append(dlidar_dist)
        matched_velocities.append(vel)
        dgps_lidar_diff.append(abs(dgps_dist - dlidar_dist))


# Print matched results
matched_df = pd.DataFrame({
    "timestamp": matched_timestamps,
    "matched_velocities": matched_velocities,
    "dgps_distance": matched_dgps,
    "dlidar_distance": matched_dlidar,
    "dgps_lidar_diff": dgps_lidar_diff
})

print("\nMatched GPS & LiDAR distances and Velocity:")
print(matched_df.to_string(index=False))

# --- Plot Velocity vs Distance ---
plt.figure(figsize=(10, 12))

# Plot 1: Velocity vs DGPS Distance
plt.subplot(3, 1, 1)
plt.scatter(matched_dgps, matched_velocities, color='blue', label="Velocity vs DGPS")
plt.xlabel("DGPS Distance (m)")
plt.ylabel("Velocity (m/s)")
plt.title("E4 Velocity vs DGPS Distance")
plt.grid(True)
plt.legend()

# Plot 2: Velocity vs LiDAR Distance
plt.subplot(3, 1, 2)
plt.scatter(matched_dlidar, matched_velocities, color='green', label="Velocity vs DLiDAR")
plt.xlabel("LiDAR Distance (m)")
plt.ylabel("Velocity (m/s)")
plt.title("E4 Velocity vs LiDAR Distance")
plt.grid(True)
plt.legend()

# Plot 3: Velocity vs DGPS-LiDAR Difference
plt.subplot(3, 1, 3)
plt.scatter(dgps_lidar_diff, matched_velocities, color='purple', label="Velocity vs |DGPS - DLiDAR|")
plt.xlabel("Absolute Distance Difference (m)")
plt.ylabel("Velocity (m/s)")
plt.title("E4 Velocity vs Distance Error (|DGPS - LiDAR|)")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
