import pandas as pd
import matplotlib.pyplot as plt

path_data = pd.read_csv("path.csv")
landmark_data = pd.read_csv("landmarks.csv")
measurement_data = pd.read_csv("measurements.csv")

plt.plot(path_data["true_x"], path_data["true_y"], label="True path")
plt.plot(path_data["noisy_x"], path_data["noisy_y"], label="Dead reckoning")

# Draw the landmarks as stars, with their id numbers
plt.scatter(landmark_data["x"], landmark_data["y"], marker="*", s=150,
            color="black", label="Landmarks", zorder=5)
for row_index in range(len(landmark_data)):
    landmark_row = landmark_data.iloc[row_index]
    plt.text(landmark_row["x"] + 0.5, landmark_row["y"] + 0.5, str(int(landmark_row["id"])))

# Draw a line from the TRUE robot position to each landmark it saw (every 10th step)
for row_index in range(len(measurement_data)):
    measurement_row = measurement_data.iloc[row_index]
    step = int(measurement_row["step"])
    if step % 10 != 0:
        continue
    robot_row = path_data[path_data["step"] == step].iloc[0]
    landmark_row = landmark_data[landmark_data["id"] == measurement_row["landmark_id"]].iloc[0]
    plt.plot([robot_row["true_x"], landmark_row["x"]],
             [robot_row["true_y"], landmark_row["y"]],
             color="gray", linewidth=0.5)

plt.axis("equal")
plt.xlabel("x (meters)")
plt.ylabel("y (meters)")
plt.title("The simulated world")
plt.legend()
plt.savefig("world_plot.png", dpi=150)
plt.show()