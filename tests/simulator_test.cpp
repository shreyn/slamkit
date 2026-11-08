#include <gtest/gtest.h>
#include "slamkit/simulator.hpp"

TEST(SimulatorTest, NoNoiseEqualsTrue) {
    std::mt19937 random_generator(42);
    Pose2D true_motion = {1.0, 0.0, 0.1};
    //no noise:
    Pose2D noisy_motion = add_odometry_noise(true_motion, random_generator, 0.0, 0.0);

    EXPECT_NEAR(noisy_motion.x, 1.0, 1e-9);
    EXPECT_NEAR(noisy_motion.y, 0.0, 1e-9);
    EXPECT_NEAR(noisy_motion.theta, 0.1, 1e-9);
}

TEST(SimulatorTest, SameSeed) {
    Pose2D true_motion = {1.0, 0.0, 0.1};

    std::mt19937 generator_one(7); //same seed
    std::mt19937 generator_two(7);

    Pose2D result_one = add_odometry_noise(true_motion, generator_one, 0.1, 0.05);
    Pose2D result_two = add_odometry_noise(true_motion, generator_two, 0.1, 0.05);

    //should be same noise
    EXPECT_EQ(result_one.x, result_two.x);
    EXPECT_EQ(result_one.y, result_two.y);
    EXPECT_EQ(result_one.theta, result_two.theta);
}