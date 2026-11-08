#include <gtest/gtest.h>
#include <cmath>
#include "slamkit/ekf.hpp"

TEST(EkfTest, PredictMovesTheGuess) {
    EkfState ekf;
    ekf.mean << 0.0, 0.0, M_PI / 2; 
    ekf.covariance.setZero();

    Pose2D wheel_motion = {1.0, 0.0, 0.0}; // 1 meter forward
    ekf_predict(ekf, wheel_motion, 0.1, 0.05);

    EXPECT_NEAR(ekf.mean(0), 0.0, 1e-9);
    EXPECT_NEAR(ekf.mean(1), 1.0, 1e-9);
    EXPECT_NEAR(ekf.mean(2), M_PI / 2, 1e-9);
}

TEST(EkfTest, PredictFromZeroUncertaintyAddsWheelNoise) {
    EkfState ekf;
    ekf.mean << 0.0, 0.0, 0.0;
    ekf.covariance.setZero(); 

    Pose2D wheel_motion = {1.0, 0.0, 0.0};
    ekf_predict(ekf, wheel_motion, 0.1, 0.05);

    // At theta = 0, G is the identity, so P becomes exactly Q
    EXPECT_NEAR(ekf.covariance(0, 0), 0.01, 1e-9);     // 0.1^2
    EXPECT_NEAR(ekf.covariance(1, 1), 0.01, 1e-9);     // 0.1^2
    EXPECT_NEAR(ekf.covariance(2, 2), 0.0025, 1e-9);   // 0.05^2
}