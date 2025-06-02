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
matched_dgps_relx = []
matched_dlidar_relx = []
matched_dgps_rely = []
matched_dlidar_rely = []
matched_velocities = []

# Extract values
dgps_times = dgps["timestamp"].values
dlidar_times = dlidar["timestamp"].values

for i, dgps_time in enumerate(dgps_times):
    diffs = np.abs(dlidar_times - dgps_time)
    min_idx = np.argmin(diffs)
    if diffs[min_idx] <= tolerance:
        matched_timestamps.append(dgps_time)
        matched_dgps.append(dgps["distance"].iloc[i])
        matched_dlidar.append(dlidar["dlidar"].iloc[min_idx])
        matched_dgps_relx.append(abs(dgps["rel_x"].iloc[i]))
        matched_dlidar_relx.append(abs(dlidar["rel_x"].iloc[min_idx]))
        matched_dgps_rely.append(abs(dgps["rel_y"].iloc[i]))
        matched_dlidar_rely.append(abs(dlidar["rel_y"].iloc[min_idx]))
        matched_velocities.append(dgps["vel"].iloc[i])  # Store velocity

# Print matched results
matched_df = pd.DataFrame({
    "timestamp": matched_timestamps,
    "dgps_distance": matched_dgps,
    "dlidar_distance": matched_dlidar,
    "abs_dgps_rel_x": matched_dgps_relx,
    "abs_dlidar_rel_x": matched_dlidar_relx,
    "abs_dgps_rel_y": matched_dgps_rely,
    "abs_dlidar_rel_y": matched_dlidar_rely,
    "velocity": matched_velocities
})

print("\nMatched GPS & LiDAR distances and relative coordinates (absolute):")
print(matched_df.to_string(index=False))

# Plotting
plt.figure(figsize=(12, 12))

# Distance Plot with velocity text
plt.subplot(3, 1, 1)
plt.plot(matched_timestamps, matched_dgps, label="GPS Distance", color="blue", marker="o")
plt.plot(matched_timestamps, matched_dlidar, label="LiDAR Distance", color="green", marker="x")

#Annotate velocities on matched points
for i in range(len(matched_timestamps)):
    plt.text(matched_timestamps[i], matched_dgps[i], f"{matched_velocities[i]:.1f}", fontsize=12, color='black', ha='right', va='bottom')

plt.xlabel("Timestamp (s)")
plt.ylabel("Distance from E4 (m)")
plt.title("Distance: DGPS vs DLiDAR (Velocity Annotated)")
plt.legend()
plt.grid(True)

# Relative X Plot
plt.subplot(3, 1, 2)
plt.plot(matched_timestamps, matched_dgps_relx, label="|GPS rel_x|", color="blue", marker="o")
plt.plot(matched_timestamps, matched_dlidar_relx, label="|LiDAR rel_x|", color="green", marker="x")
plt.xlabel("Timestamp (s)")
plt.ylabel("|Relative X| (m)")
plt.title("Absolute Relative X: DGPS vs DLiDAR")
plt.legend()
plt.grid(True)

# Relative Y Plot
plt.subplot(3, 1, 3)
plt.plot(matched_timestamps, matched_dgps_rely, label="|GPS rel_y|", color="blue", marker="o")
plt.plot(matched_timestamps, matched_dlidar_rely, label="|LiDAR rel_y|", color="green", marker="x")
plt.xlabel("Timestamp (s)")
plt.ylabel("|Relative Y| (m)")
plt.title("Absolute Relative Y: DGPS vs DLiDAR")
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
