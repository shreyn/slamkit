import sys
import time
import numpy as np
import pandas as pd
import gtsam

g2o_filename = sys.argv[1]
our_csv = sys.argv[2] # the csv written by ./build/g2o_demo

# read the same file two times: one copy for scoring, one copy to optimize
scoring_graph, initial = gtsam.readG2o(g2o_filename, False)   # False = 2D
graph, _ = gtsam.readG2o(g2o_filename, False)
num_poses = initial.size()

# the test with no optimizer
start_error = scoring_graph.error(initial)
print(f"GTSAM start error: {start_error:.6g}   (x2 = {2 * start_error:.6g})")

# fix pose 0
pose_zero = initial.atPose2(0)
prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-6, 1e-6, 1e-8]))
graph.add(gtsam.PriorFactorPose2(0, pose_zero, prior_noise))

#optimize (GaussNewton)
params = gtsam.GaussNewtonParams()
params.setMaxIterations(20)
optimizer = gtsam.GaussNewtonOptimizer(graph, initial, params)
start_time = time.perf_counter()
result = optimizer.optimize()
seconds = time.perf_counter() - start_time

final_error = scoring_graph.error(result)
print(f"GTSAM final error: {final_error:.6g}   (x2 = {2 * final_error:.6g})")
print(f"GTSAM time for the whole optimization: {seconds:.4f} s")

# score our poses using GTSAM's own cost
ours = pd.read_csv(our_csv)
our_values = gtsam.Values()
for i in range(num_poses):
    our_values.insert(i, gtsam.Pose2(ours["opt_x"][i], ours["opt_y"][i], ours["opt_theta"][i]))
our_error = scoring_graph.error(our_values)
print(f"GTSAM's cost at OUR poses: {our_error:.6g}   (x2 = {2 * our_error:.6g})")

# how far apart are the two solutions?
gtsam_xy = np.array([[result.atPose2(i).x(), result.atPose2(i).y()] for i in range(num_poses)])
our_xy = ours[["opt_x", "opt_y"]].to_numpy()
offset = gtsam_xy[0] - our_xy[0]
difference = np.linalg.norm(gtsam_xy - (our_xy + offset), axis=1)
print(f"Position difference vs GTSAM: RMS {np.sqrt(np.mean(difference**2)):.4g} m, max {difference.max():.4g} m")