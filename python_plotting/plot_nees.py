import pandas as pd
import matplotlib.pyplot as plt
from scipy.stats import chi2

num_runs = 1000
degrees_of_freedom = 3 

nees_data = pd.read_csv("nees.csv")

# The sum of NEES over all runs follows a chi-square curve with (3 * num_runs) degrees of freedom. Divide by num_runs to get the average.
lower_bound = chi2.ppf(0.025, degrees_of_freedom * num_runs) / num_runs
upper_bound = chi2.ppf(0.975, degrees_of_freedom * num_runs) / num_runs

plt.plot(nees_data["step"], nees_data["average_nees"], label="Average NEES")
plt.axhline(degrees_of_freedom, color="green", linestyle="--", label="Ideal (3)")
plt.axhline(lower_bound, color="gray", linestyle=":", label="95% band")
plt.axhline(upper_bound, color="gray", linestyle=":")

plt.xlabel("step")
plt.ylabel("average NEES")
plt.title("EKF consistency over 1000 runs")
plt.legend()
plt.savefig("nees_plot.png", dpi=150)
plt.show()