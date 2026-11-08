#pragma once

#include <Eigen/Dense>
#include "slamkit/pose2d.hpp"
#include "slamkit/sensor.hpp"

//belief: best guess + uncertainty
struct EkfState {
    Eigen::Vector3d mean; //(x,y , theta)
    Eigen::Matrix3d covariance; 
};

// Jacobian of compose(pose, wheel_motion) w.r.t. old pose
Eigen::Matrix3d motion_jacobian_pose(Pose2D pose, Pose2D wheel_motion);

// Jacobian of compose(pose, wheel_motion) w.r.t wheel motion
Eigen::Matrix3d motion_jacobian_wheel(Pose2D pose);

// Jacobian of the landmark measurement w.r.t the robot pose.
// 2 rows (range, bearing) and 3 columns (x, y, theta).
Eigen::Matrix<double, 2, 3> measurement_jacobian(Pose2D pose, Landmark landmark);

//Predict step: move the belief by one wheel reading.
void ekf_predict(EkfState& ekf,
                 Pose2D wheel_motion,
                 double position_noise_std,
                 double angle_noise_std);

//Update step: correct the belief using one landmark reading
void ekf_update(EkfState& ekf,
                Landmark landmark,
                Measurement measurement,
                double range_noise_std,
                double bearing_noise_std);