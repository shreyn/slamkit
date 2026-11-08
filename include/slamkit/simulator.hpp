#pragma once

#include <random>
#include "slamkit/pose2d.hpp"

// takes a true motion (in the robot's own frame) and returns a noisy version of it, like a wheel sensor would report.
Pose2D add_odometry_noise(Pose2D true_motion, std::mt19937& random_generator, double position_noise_std, double angle_noise_std);
// position_noise_std: how big the error is for x and y
// angle_noise_std: how big the error is for turning (rad)
