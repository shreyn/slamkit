#include "slamkit/sensor.hpp"
#include <cmath>

//perfect sensor reading of landmark
Measurement measure_landmark(Pose2D robot_pose, Landmark landmark) {
    double dx = landmark.x - robot_pose.x;
    double dy = landmark.y - robot_pose.y;

    Measurement measurement;
    measurement.range = std::sqrt(dx*dx + dy*dy);
    measurement.bearing = wrap_angle(std::atan2(dy, dx) - robot_pose.theta);
    return measurement;
}

Measurement measure_landmark_noisy(Pose2D robot_pose, Landmark landmark, std::mt19937& random_generator, double range_noise_std, double bearing_noise_std) {
    Measurement measurement = measure_landmark(robot_pose, landmark);

    std::normal_distribution<double> range_noise(0.0, range_noise_std);
    std::normal_distribution<double> bearing_noise(0.0, bearing_noise_std);

    Measurement noisy_measurement;
    noisy_measurement.range = measurement.range+range_noise(random_generator);
    noisy_measurement.bearing = wrap_angle(measurement.bearing+ bearing_noise(random_generator));
    return noisy_measurement;
}
