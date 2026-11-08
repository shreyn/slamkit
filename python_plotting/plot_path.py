import pandas as pd
import matplotlib.pyplot as plt

path_data = pd.read_csv("path.csv")

plt.plot(path_data["true_x"], path_data["true_y"], label="True path")
plt.plot(path_data["noisy_x"], path_data["noisy_y"], label="Dead reckoning")

# Mark where both paths start and where each one ends
plt.scatter(0, 0, color="green", label="Start", zorder=5)
last_row = path_data.iloc[-1]
plt.scatter(last_row["noisy_x"], last_row["noisy_y"], color="red", label="Dead reckoning end", zorder=5)

plt.axis("equal")
plt.xlabel("x (meters)")
plt.ylabel("y (meters)")
plt.title("True path vs dead reckoning")
plt.legend()

plt.savefig("path_plot.png", dpi=150)
plt.show()