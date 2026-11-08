#include "slamkit/ekf.hpp"
#include <cmath>

Eigen::Matrix3d motion_jacobian_pose(Pose2D pose, Pose2D wheel_motion) {
    double sin_theta = std::sin(pose.theta);
    double cos_theta = std::cos(pose.theta);

    Eigen::Matrix3d F;
    F << 1.0, 0.0, -sin_theta * wheel_motion.x - cos_theta * wheel_motion.y,
         0.0, 1.0,  cos_theta * wheel_motion.x - sin_theta * wheel_motion.y,
         0.0, 0.0, 1.0;
    return F;
}

Eigen::Matrix3d motion_jacobian_wheel(Pose2D pose) {
    double sin_theta = std::sin(pose.theta);
    double cos_theta = std::cos(pose.theta);

    Eigen::Matrix3d G;
    G << cos_theta, -sin_theta, 0.0,
         sin_theta,  cos_theta, 0.0,
         0.0,        0.0,       1.0;
    return G;
}

Eigen::Matrix<double, 2, 3> measurement_jacobian(Pose2D pose, Landmark landmark) {
    double dx = landmark.x - pose.x;
    double dy = landmark.y - pose.y;
    double q = dx * dx + dy * dy;
    double r = std::sqrt(q);

    Eigen::Matrix<double, 2, 3> H;
    H << -dx / r, -dy / r,  0.0,
          dy / q, -dx / q, -1.0;
    return H;
}

void ekf_predict(EkfState& ekf,
                 Pose2D wheel_motion,
                 double position_noise_std,
                 double angle_noise_std) {
    Pose2D old_pose = {ekf.mean(0), ekf.mean(1), ekf.mean(2)};

    // build F and G from the OLD pose
    Eigen::Matrix3d F = motion_jacobian_pose(old_pose, wheel_motion);
    Eigen::Matrix3d G = motion_jacobian_wheel(old_pose);

    //build Q (wheel noise as variances (square of std))
    Eigen::Matrix3d Q;
    Q.setZero();
    Q(0, 0) = position_noise_std*position_noise_std;
    Q(1, 1) = position_noise_std*position_noise_std;
    Q(2, 2) = angle_noise_std*angle_noise_std;

    //move guess (compose)
    Pose2D new_pose = compose(old_pose, wheel_motion);
    ekf.mean(0) = new_pose.x;
    ekf.mean(1) = new_pose.y;
    ekf.mean(2) = new_pose.theta;

    //grow uncertainty
    ekf.covariance = F * ekf.covariance * F.transpose() + G * Q * G.transpose();
}

void ekf_update(EkfState& ekf,
                Landmark landmark,
                Measurement measurement,
                double range_noise_std,
                double bearing_noise_std) {
    Pose2D current_pose = {ekf.mean(0), ekf.mean(1), ekf.mean(2)};

    //what would perfect sensor read given current pose?
    Measurement expected = measure_landmark(current_pose, landmark);

    //innovation (measurement - expected)
    Eigen::Vector2d innovation;
    innovation(0) = measurement.range - expected.range;
    innovation(1) = wrap_angle(measurement.bearing-expected.bearing);

    //Jacobian H
    Eigen::Matrix<double, 2, 3> H = measurement_jacobian(current_pose, landmark);

    // sensor noise R (variances)
    Eigen::Matrix2d R;
    R.setZero();
    R(0, 0) = range_noise_std*range_noise_std;
    R(1, 1) = bearing_noise_std*bearing_noise_std;

    Eigen::Matrix2d S = H * ekf.covariance * H.transpose() + R;

    //Kalman gain: how much of the innovation do we trust?: my uncertainty, out of my uncertainty plus sensor's uncertainty
    Eigen::Matrix<double, 3, 2> K = ekf.covariance * H.transpose() * S.inverse();

    //correct the guess, shrink uncertainty
    ekf.mean = ekf.mean + K*innovation; //guess = guess + trust*innovation
    ekf.mean(2) = wrap_angle(ekf.mean(2));

    //update uncertainty: cov = (1-K*H)*cov
    Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    ekf.covariance = (identity - K * H) * ekf.covariance; 

}