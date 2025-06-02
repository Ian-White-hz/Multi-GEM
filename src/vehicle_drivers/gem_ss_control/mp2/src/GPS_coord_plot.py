import csv
import matplotlib.pyplot as plt

csv_file = "src/mp2/src/e4_coords_new.csv"

x_vals = []
y_vals = []

with open(csv_file, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        x_vals.append(float(row["x"]))
        y_vals.append(float(row["y"]))

#E2 static position - Date = Apr 17,2025 Testing - relative to olat/olon
e2x, e2y =39.0338, -8.8014

# Plot
plt.figure(figsize=(8, 6))
plt.plot(x_vals, y_vals, marker='o', linestyle='-', color='blue', label='E4')
plt.scatter(e2x, e2y, color='red', label='E2', zorder=5)
plt.title("E4 Trajectory(X vs Y)")
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.grid(True)
plt.axis("equal")
plt.legend()
plt.tight_layout()
plt.show()
