#pragma once

#include <random>
#include "slamkit/pose2d.hpp"

struct Landmark {
    double x;
    double y;
};

struct Measurement {
    double range;
    double bearing;
};

// perfect sensor reading of landmark
Measurement measure_landmark(Pose2D robot_pose, Landmark landmark);

//above, with noise
Measurement measure_landmark_noisy(Pose2D robot_pose, Landmark landmark, std::mt19937& random_generator, double range_noise_std, double bearing_noise_std);
