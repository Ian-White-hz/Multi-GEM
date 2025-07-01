import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load CSVs
dgps = pd.read_csv("src/mp2/src/dGPS.csv")
dlidar = pd.read_csv("src/mp2/src/dlidar.csv")
e4_coords = pd.read_csv("src/mp2/src/e4_coords_new.csv")

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
        matched_dgps_relx.append(dgps["rel_x"].iloc[i])
        matched_dlidar_relx.append(dlidar["rel_x"].iloc[min_idx])
        matched_dgps_rely.append(dgps["rel_y"].iloc[i])
        matched_dlidar_rely.append(dlidar["rel_y"].iloc[min_idx])

# ===================== PLOT E4 TRAJECTORY + Points chosen to get dLiDAR =====================
e2x, e2y =39.0338, -8.8014
e4_all = e4_coords.copy()
e4_all["timestamp"] = e4_all["timestamp"].round(3)
matched_ts_set = set(round(ts, 3) for ts in matched_timestamps)

# Separate matched points
e4_matched = e4_all[e4_all["timestamp"].isin(matched_ts_set)]
e4_unmatched = e4_all[~e4_all["timestamp"].isin(matched_ts_set)]
print(len(e4_matched))
plt.figure(figsize=(8, 8))
plt.plot(e4_all["x"].values, e4_all["y"].values, label="E4 Trajectory", color="gray", linewidth=1)
plt.scatter(e4_matched["x"].values, e4_matched["y"].values, color="blue", label="Matched Timestamps", zorder=5)
plt.scatter(e2x, e2y, color='red', label='E2 Position', zorder=5)
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("E4 Trajectory with Highlighted Matched Timestamps")
plt.grid(True)
plt.axis("equal")
plt.legend()
plt.tight_layout()
plt.show()
