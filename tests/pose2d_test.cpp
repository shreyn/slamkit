#include <gtest/gtest.h>
#include <cmath>
#include "slamkit/pose2d.hpp"

TEST(Pose2DTest, ComposeForwardFacingNorth) {
    //robot is facing north. so robots, frame, +x is forward north, +y is left
    Pose2D a = {0.0, 0.0, M_PI/2};   // at origin, facing north
    Pose2D b = {1.0, 0.0, 0.0}; //move forward by 1

    Pose2D result = compose(a, b);

    //results in robot being +1 y, facing north
    EXPECT_NEAR(result.x, 0.0, 1e-9); 
    EXPECT_NEAR(result.y, 1.0, 1e-9);
    EXPECT_NEAR(result.theta, M_PI/2, 1e-9);
}

TEST(Pose2DTest, ComposeLeftFacingNorth) {
    Pose2D a = {0.0, 0.0, M_PI/2};   // at origin, facing north
    Pose2D b = {0.0, 1.0, 0.0}; //move left by 1

    Pose2D result = compose(a, b);

    //results in robot moving left by 1, facing north
    EXPECT_NEAR(result.x, -1.0, 1e-9);
    EXPECT_NEAR(result.y, 0.0, 1e-9);
    EXPECT_NEAR(result.theta, M_PI/2, 1e-9);
}



// WRAP:
TEST(Pose2DTest, WrapAngleTooBig) {
    EXPECT_NEAR(wrap_angle(3.5), 3.5 - 2.0*M_PI, 1e-9);
}


TEST(Pose2DTest, RelativePoseOneMeterAhead) {
    Pose2D a = {0.0, 0.0, M_PI / 2};  // origin, facing north
    Pose2D b = {0.0, 1.0, M_PI / 2}; // 1 meter north

    Pose2D relative = relative_pose(a, b);

    EXPECT_NEAR(relative.x, 1.0, 1e-9);// 1 m forward
    EXPECT_NEAR(relative.y, 0.0, 1e-9);
    EXPECT_NEAR(relative.theta, 0.0, 1e-9);
}

TEST(Pose2DTest, PoseComposedWithInverseIsOrigin) {
    Pose2D p = {1.5, -2.0, 0.7};

    Pose2D result = compose(p, inverse(p));

    EXPECT_NEAR(result.x, 0.0, 1e-9);
    EXPECT_NEAR(result.y, 0.0, 1e-9);
    EXPECT_NEAR(result.theta, 0.0, 1e-9);
}

TEST(Pose2DTest, RelativePoseRoundTrip) {
    Pose2D a = {1.0, 2.0, 0.7};
    Pose2D b = {-3.0, 0.5, -2.5};

    Pose2D rebuilt = compose(a, relative_pose(a, b));

    EXPECT_NEAR(rebuilt.x, b.x, 1e-9);
    EXPECT_NEAR(rebuilt.y, b.y, 1e-9);
    EXPECT_NEAR(rebuilt.theta, b.theta, 1e-9);
}