import csv
import matplotlib.pyplot as plt

csv_file = "src/mp2/src/dGPS.csv"

timestamps = []
distances = []

# Read CSV
with open(csv_file, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        timestamps.append(float(row["timestamp"]))
        distances.append(float(row["distance"]))

# Plot
plt.figure(figsize=(10, 6))
plt.plot(timestamps, distances, color='blue', marker='o', linestyle='-')
plt.title("dGPS vs Timestamp")
plt.xlabel("Timestamp (UNIX time)")
plt.ylabel("Distance (meters)")
plt.grid(True)
plt.tight_layout()
plt.show()
