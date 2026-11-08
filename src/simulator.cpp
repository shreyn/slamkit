#include "slamkit/simulator.hpp"

Pose2D add_odometry_noise(Pose2D true_motion,
                          std::mt19937& random_generator,
                          double position_noise_std,
                          double angle_noise_std) {
    std::normal_distribution<double> position_noise(0.0, position_noise_std);
    std::normal_distribution<double> angle_noise(0.0, angle_noise_std);

    Pose2D noisy_motion;
    noisy_motion.x = true_motion.x + position_noise(random_generator);
    noisy_motion.y = true_motion.y + position_noise(random_generator);
    noisy_motion.theta = true_motion.theta + angle_noise(random_generator);
    return noisy_motion;
}