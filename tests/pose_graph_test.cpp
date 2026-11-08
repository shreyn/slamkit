#include <gtest/gtest.h>
#include <cmath>
#include "slamkit/pose_graph.hpp"

TEST(PoseGraphTest, RelaxedBandHasZeroResidual) {
    Pose2D pose_from = {0.0, 0.0, M_PI / 2};  // origin, facing north
    Pose2D pose_to = {0.0, 1.0, M_PI / 2};// 1 m north
    Pose2D measurement = {1.0, 0.0, 0.0};   // 1 m ahead

    Eigen::Vector3d residual = edge_residual(pose_from, pose_to, measurement);

    EXPECT_NEAR(residual(0), 0.0, 1e-9);
    EXPECT_NEAR(residual(1), 0.0, 1e-9);
    EXPECT_NEAR(residual(2), 0.0, 1e-9);
}

TEST(PoseGraphTest, StretchedBandHasKnownResidual) {
    Pose2D pose_from = {0.0, 0.0, M_PI / 2};
    Pose2D pose_to = {0.0, 1.0, M_PI / 2};
    Pose2D measurement = {1.2, 0.0, 0.0};  // measurement says 1.2 m ahead

    Eigen::Vector3d residual = edge_residual(pose_from, pose_to, measurement);

    // dots say 1.0, measurement says 1.2, so residual = 1.0 - 1.2 = -0.2
    EXPECT_NEAR(residual(0), -0.2, 1e-9);
    EXPECT_NEAR(residual(1), 0.0, 1e-9);
    EXPECT_NEAR(residual(2), 0.0, 1e-9);
}

TEST(PoseGraphTest, HeadingResidualIsWrapped) {
    Pose2D pose_from = {0.0, 0.0, 0.0};
    Pose2D pose_to = {0.0, 0.0, -M_PI + 0.1};
    Pose2D measurement = {0.0, 0.0, M_PI - 0.1};

    Eigen::Vector3d residual = edge_residual(pose_from, pose_to, measurement);

    // the two headings are only 0.2 rad apart, not about 6 rad
    EXPECT_NEAR(residual(2), 0.2, 1e-9);
}

TEST(PoseGraphTest, TotalCostAddsUpEdges) {
    std::vector<Pose2D> poses = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
    Eigen::Matrix3d info = diagonal_information(1.0, 1.0);

    std::vector<Edge> edges;
    edges.push_back({0, 1, {1.0, 0.0, 0.0}, info});
    edges.push_back({1, 2, {1.0, 0.0, 0.0}, info});
    edges.push_back({0, 2, {1.8, 0.0, 0.0}, info});

    double cost = total_cost(poses, edges);

    EXPECT_NEAR(cost, 0.04, 1e-9);
}

TEST(PoseGraphTest, LinearSystemMatchesHallwayExample) {
    std::vector<Pose2D> poses = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
    Eigen::Matrix3d info = diagonal_information(1.0, 1.0);

    std::vector<Edge> edges;
    edges.push_back({0, 1, {1.0, 0.0, 0.0}, info});
    edges.push_back({1, 2, {1.0, 0.0, 0.0}, info});
    edges.push_back({0, 2, {1.8, 0.0, 0.0}, info});

    LinearSystem system = build_linear_system(poses, edges);

    // Entries for the x number of each dot (indices 0, 3, 6)
    EXPECT_NEAR(system.H(0, 0), 2.0, 1e-9);
    EXPECT_NEAR(system.H(3, 3), 2.0, 1e-9);
    EXPECT_NEAR(system.H(6, 6), 2.0, 1e-9);
}