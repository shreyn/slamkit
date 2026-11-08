import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

path_data = pd.read_csv("path.csv")
landmark_data = pd.read_csv("landmarks.csv")

# Position error at every step: distance from the true position
dead_reckoning_error = np.sqrt((path_data["noisy_x"] - path_data["true_x"]) ** 2 +
                               (path_data["noisy_y"] - path_data["true_y"]) ** 2)
ekf_error = np.sqrt((path_data["ekf_x"] - path_data["true_x"]) ** 2 +
                    (path_data["ekf_y"] - path_data["true_y"]) ** 2)

# Two pictures side by side
figure, (map_axes, error_axes) = plt.subplots(1, 2, figsize=(14, 6))

# Left: the map
map_axes.plot(path_data["true_x"], path_data["true_y"], label="True path")
map_axes.plot(path_data["noisy_x"], path_data["noisy_y"], label="Dead reckoning")
map_axes.plot(path_data["ekf_x"], path_data["ekf_y"], label="EKF")
map_axes.scatter(landmark_data["x"], landmark_data["y"], marker="*", s=120,
                 color="black", label="Landmarks", zorder=5)
map_axes.set_aspect("equal")
map_axes.set_xlabel("x (meters)")
map_axes.set_ylabel("y (meters)")
map_axes.set_title("Paths")
map_axes.legend()

# Right: error over time
error_axes.plot(path_data["step"], dead_reckoning_error, label="Dead reckoning")
error_axes.plot(path_data["step"], ekf_error, label="EKF")
error_axes.set_xlabel("step")
error_axes.set_ylabel("position error (meters)")
error_axes.set_title("Error vs truth")
error_axes.legend()

plt.savefig("ekf_plot.png", dpi=150)
plt.show()