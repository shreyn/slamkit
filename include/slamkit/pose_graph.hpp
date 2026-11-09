#pragma once

#include <Eigen/Dense>
#include "slamkit/pose2d.hpp"
#include <vector>
#include <string>
#include <Eigen/Sparse>

//rubber band between 2 poses
struct Edge {
    int from_index;
    int to_index; 
    Pose2D measurement; //wheel reading: to_index is this far from from_index, seen from pose from_index's point of view

    Eigen::Matrix3d information; //trust table (inverse of band's cov)
};

// how stretched is this band, given where the two poses currently are?
Eigen::Vector3d edge_residual(Pose2D pose_from, Pose2D pose_to, Pose2D measurement);

//old trust table (diag, from 2 std values)
Eigen::Matrix3d diagonal_information(double position_std, double angle_std);

//total cost of a pose graph
// loops over all edges (both wheel edges, and loop closure edges)
double total_cost(std::vector<Pose2D> poses, std::vector<Edge> edges);
    

//Jacobian of edge_residual w.r.t the "from" pose
Eigen::Matrix3d edge_jacobian_from(Pose2D pose_from, Pose2D pose_to);

//Jacobian of edge_residual w.r.t the "to" pose
Eigen::Matrix3d edge_jacobian_to(Pose2D pose_from, Pose2D pose_to);


//Gauss-Newton step: H * delta = -b
struct LinearSystem {
    Eigen::MatrixXd H; // 3N x 3N: how strongly the dots are tied together
    Eigen::VectorXd b; // 3N: how hard the bands are currently pulling each dot
};

//builds H and b by looping over all edges.
LinearSystem build_linear_system(std::vector<Pose2D> poses, std::vector<Edge> edges);

// Gauss-Newton optimize. Dot 0 fixed, returns improved poses
std::vector<Pose2D> optimize(std::vector<Pose2D> poses, std::vector<Edge> edges, int max_iters);


///////// g2o File ////////:

struct PoseGraph {
    std::vector<Pose2D> poses;
    std::vector<Edge> edges;
};

// reads VERTEX_SE2 and EDGE_SE2 lines from g2o file.
PoseGraph load_g2o(std::string filename);



/// Sparse:

//Same as LinearSystem, but H stores only nonzero entries:
struct SparseLinearSystem {
    Eigen::SparseMatrix<double> H;
    Eigen::VectorXd b;
};

SparseLinearSystem build_sparse_linear_system(std::vector<Pose2D> poses,  std::vector<Edge> edges);

std::vector<Pose2D> optimize_sparse(std::vector<Pose2D> poses, std::vector<Edge> edges, int max_iters);

std::vector<Pose2D> optimize_lm(std::vector<Pose2D> poses, const std::vector<Edge>& edges, int max_iters);