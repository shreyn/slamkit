import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv("pose_graph.csv")

plt.plot(data["true_x"], data["true_y"], label="True path")
plt.plot(data["noisy_x"], data["noisy_y"], label="Dead reckoning (before)")
plt.plot(data["opt_x"], data["opt_y"], label="Pose-graph optimized (after)")

plt.axis("equal")
plt.xlabel("x (meters)")
plt.ylabel("y (meters)")
plt.title("Pose-graph SLAM with one loop closure")
plt.legend()
plt.savefig("pose_graph_plot.png", dpi=150)
plt.show()