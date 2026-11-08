#include <gtest/gtest.h>
#include <cmath>
#include "slamkit/sensor.hpp"

TEST(SensorTest, LandmarkUpAndRight) {
    Pose2D robot_pose = {0.0, 0.0, 0.0}; //origin, facing east
    Landmark landmark = {3.0, 4.0}; //+3x, +4y, from origin

    Measurement measurement = measure_landmark(robot_pose, landmark);

    EXPECT_NEAR(measurement.range, 5.0, 1e-9); //hypotenuse = 5
    EXPECT_NEAR(measurement.bearing, std::atan2(4.0, 3.0), 1e-9);
}

TEST(SensorTest, LandmarkStraight) {
    Pose2D robot_pose = {0.0, 0.0, M_PI/2}; // facing north
    Landmark landmark = {0.0, 5.0}; // 5 meters north

    Measurement measurement = measure_landmark(robot_pose, landmark);

    EXPECT_NEAR(measurement.range, 5.0, 1e-9);
    EXPECT_NEAR(measurement.bearing, 0.0, 1e-9); //ahead
}
