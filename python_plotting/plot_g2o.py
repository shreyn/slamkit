import sys
import pandas as pd
import matplotlib.pyplot as plt

csv_filename = sys.argv[1]
data = pd.read_csv(csv_filename)

figure, (start_axes, opt_axes) = plt.subplots(1, 2, figsize=(14, 6))

start_axes.plot(data["start_x"], data["start_y"], linewidth=0.5)
start_axes.set_title("Before: starting guess")

opt_axes.plot(data["opt_x"], data["opt_y"], linewidth=0.5, color="green")
opt_axes.set_title("After: pose-graph optimized")

for axes in (start_axes, opt_axes):
    axes.set_aspect("equal")
    axes.set_xlabel("x (meters)")
    axes.set_ylabel("y (meters)")

plt.savefig(csv_filename.replace(".csv", ".png"), dpi=150)
plt.show()