#include "slamkit/pose_graph.hpp"
#include <cmath>
#include <iostream>

Eigen::Vector3d edge_residual(Pose2D pose_from, Pose2D pose_to, Pose2D measurement) {
    //what the dots currently say (B seen from A)
    Pose2D predicted = relative_pose(pose_from, pose_to);

    // Residual: predicted - measurement
    Eigen::Vector3d residual;
    residual(0) = predicted.x - measurement.x;
    residual(1) = predicted.y - measurement.y;
    residual(2) = wrap_angle(predicted.theta - measurement.theta);
    return residual;
}

//old truth table
Eigen::Matrix3d diagonal_information(double position_std, double angle_std) {
    Eigen::Matrix3d information;
    information.setZero();
    information(0, 0) = 1.0 / (position_std * position_std);
    information(1, 1) = 1.0 / (position_std * position_std);
    information(2, 2) = 1.0 / (angle_std * angle_std);
    return information;
}

double total_cost(std::vector<Pose2D> poses, std::vector<Edge> edges) {
    double cost = 0.0;
    for (int i = 0; i < edges.size(); i++) {
        Eigen::Vector3d residual = edge_residual(poses[edges[i].from_index], poses[edges[i].to_index], edges[i].measurement);
        // cost of this band = residual^T * information * residual (one number)
        double band_cost = residual.transpose() * edges[i].information * residual;
        cost += band_cost;
    }
    return cost;
}

Eigen::Matrix3d edge_jacobian_from(Pose2D pose_from, Pose2D pose_to) {
    double c = std::cos(pose_from.theta);
    double s = std::sin(pose_from.theta);

    // p = what the dots say (B seen from A)
    Pose2D p = relative_pose(pose_from, pose_to);

    Eigen::Matrix3d jacobian;
    jacobian << -c, -s,  p.y,
                 s, -c, -p.x,
                0.0, 0.0, -1.0;
    return jacobian;
}

Eigen::Matrix3d edge_jacobian_to(Pose2D pose_from, Pose2D pose_to) {
    double c = std::cos(pose_from.theta);
    double s = std::sin(pose_from.theta);

    Eigen::Matrix3d jacobian;
    jacobian <<  c,   s,  0.0,
                -s,   c,  0.0,
                0.0, 0.0, 1.0;
    return jacobian;
}


LinearSystem build_linear_system(std::vector<Pose2D> poses, std::vector<Edge> edges) {
    int num_poses = poses.size();
    //init: all zeros
    LinearSystem system;
    system.H = Eigen::MatrixXd::Zero(3*num_poses, 3*num_poses);
    system.b = Eigen::VectorXd::Zero(3*num_poses);

    //add each edge's contribution
    for (int k = 0; k < edges.size(); k++) {
        Edge edge = edges[k];
        int i = edge.from_index;
        int j = edge.to_index;
        Eigen::Matrix3d omega = edge.information; //trust table

        Pose2D pose_from = poses[i];
        Pose2D pose_to = poses[j];

        Eigen::Vector3d residual = edge_residual(pose_from, pose_to, edge.measurement);
        Eigen::Matrix3d jacobian_from = edge_jacobian_from(pose_from, pose_to);
        Eigen::Matrix3d jacobian_to = edge_jacobian_to(pose_from, pose_to);

        //four blocks of H. several edges land in the same block, contributions add up
        system.H.block(3 * i, 3 * i, 3, 3) += jacobian_from.transpose() * omega * jacobian_from;
        system.H.block(3 * i, 3 * j, 3, 3) += jacobian_from.transpose() * omega * jacobian_to;
        system.H.block(3 * j, 3 * i, 3, 3) += jacobian_to.transpose() * omega * jacobian_from;
        system.H.block(3 * j, 3 * j, 3, 3) += jacobian_to.transpose() * omega * jacobian_to;

        // two pieces of b
        system.b.segment(3 * i, 3) += jacobian_from.transpose() * omega * residual;
        system.b.segment(3 * j, 3) += jacobian_to.transpose() * omega * residual;
    }

    return system;
}

std::vector<Pose2D> optimize(std::vector<Pose2D> poses, std::vector<Edge> edges, int max_iters) {
    int num_poses = poses.size();

    for (int iter = 0; iter < max_iters; iter++) {
        //build H, b from current dots
        LinearSystem system = build_linear_system(poses, edges);

        //fix dot 0 (make it extremely hard to move)
        for (int k = 0; k < 3; k++) {
            system.H(k, k) += 1.0e8; //very large
        }

        //solve H*delta = -b
        Eigen::VectorXd delta = system.H.ldlt().solve(-system.b);

        //move each dot by its 3 numbers
        for (int i = 0; i < num_poses; i++) {
            poses[i].x += delta(3*i);
            poses[i].y += delta(3*i + 1);
            poses[i].theta = wrap_angle(poses[i].theta + delta(3*i + 2));
        }

        //threshold, stop
        if (delta.norm() < 1.0e-6) {
            break;
        }
    }

    return poses;
}


////// SPARSE /////


//pushes the 9 numbers of a 3x3 block into the triplet list.
static void add_block(std::vector<Eigen::Triplet<double>>& triplets, int row_start, int col_start, Eigen::Matrix3d block) {
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            triplets.push_back(Eigen::Triplet<double>(row_start + r, col_start + c, block(r, c)));
        }
    }
}

SparseLinearSystem build_sparse_linear_system(std::vector<Pose2D> poses, std::vector<Edge> edges) {
    int num_poses = poses.size();

    // list of (row, column, value) entries
    std::vector<Eigen::Triplet<double>> triplets;

    SparseLinearSystem system;
    system.b = Eigen::VectorXd::Zero(3*num_poses);

    for (int k = 0; k < edges.size(); k++) {
        Edge edge = edges[k];
        int i = edge.from_index;
        int j = edge.to_index;
        Pose2D pose_from = poses[i];
        Pose2D pose_to = poses[j];

        Eigen::Vector3d residual = edge_residual(pose_from, pose_to, edge.measurement);
        Eigen::Matrix3d jacobian_from = edge_jacobian_from(pose_from, pose_to);
        Eigen::Matrix3d jacobian_to = edge_jacobian_to(pose_from, pose_to);
        Eigen::Matrix3d omega = edge.information;

        // the four blocks of H: pushed into the list instead of added into a big table
        add_block(triplets, 3 * i, 3 * i, jacobian_from.transpose() * omega * jacobian_from);
        add_block(triplets, 3 * i, 3 * j, jacobian_from.transpose() * omega * jacobian_to);
        add_block(triplets, 3 * j, 3 * i, jacobian_to.transpose() * omega * jacobian_from);
        add_block(triplets, 3 * j, 3 * j, jacobian_to.transpose() * omega * jacobian_to);

        // b is small (one number per unknown), so it stays an ordinary list
        system.b.segment(3 * i, 3) += jacobian_from.transpose() * omega * residual;
        system.b.segment(3 * j, 3) += jacobian_to.transpose() * omega * residual;
    }

    //build the compact matrix. Entries with the same (row, column) are summed.
    system.H.resize(3 * num_poses, 3 * num_poses);
    system.H.setFromTriplets(triplets.begin(), triplets.end());

    return system;
}

std::vector<Pose2D> optimize_sparse(std::vector<Pose2D> poses, std::vector<Edge> edges, int max_iters) {
    int num_poses = poses.size();

    for (int iter = 0; iter < max_iters; iter++) {
        //build H, b from current dots
        SparseLinearSystem system = build_sparse_linear_system(poses, edges);

        //fix dot 0 (make it extremely hard to move)
        for (int k = 0; k < 3; k++) {
            system.H.coeffRef(k, k) += 1.0e8;
        }

        //sparse factor H, then solve H*delta = -b
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
        solver.compute(system.H);
        if (solver.info() != Eigen::Success) {
            std::cout << "Sparse factorization failed\n";
            break;
        }
        Eigen::VectorXd delta = solver.solve(-system.b);

        // Step 4: move every dot (same as before)
        for (int i = 0; i < num_poses; i++) {
            poses[i].x += delta(3 * i);
            poses[i].y += delta(3 * i + 1);
            poses[i].theta = wrap_angle(poses[i].theta + delta(3 * i + 2));
        }

        //move each dot by its 3 numbers
        if (delta.norm() < 1.0e-6) {
            break;
        }
    }

    return poses;
}

//Levenberg-Marquardt:
std::vector<Pose2D> optimize_lm(std::vector<Pose2D> poses, const std::vector<Edge>& edges, int max_iters) {
    int num_poses = poses.size();

    //damping dial (start small: nearly Gauss-Newton)
    double lambda = 1.0e-3;

    double curr_cost = total_cost(poses, edges);

    for (int iter = 1; iter <= max_iters; iter++) {
        //build H, b at the current poses (once per iteration)
        SparseLinearSystem system = build_sparse_linear_system(poses, edges);

        //fix dot 0 (make it extremely hard to move)
        for (int k = 0; k < 3; k++) {
            system.H.coeffRef(k, k) += 1.0e8;
        }

        // how stiff each unknown is on its own (the diagonal of H)
        Eigen::VectorXd diagonal_of_H = system.H.diagonal();

        // keep raising lambda until we find a step that lowers the cost
        bool step_accepted = false;
        Eigen::VectorXd delta;
        for (int attempt = 0; attempt < 10; attempt++) {
            Eigen::SparseMatrix<double> damped_H = system.H;
            for (int k = 0; k < 3 * num_poses; k++) {
                damped_H.coeffRef(k, k) += lambda * diagonal_of_H(k);
            }

            Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
            solver.compute(damped_H);
            if (solver.info() != Eigen::Success) {
                lambda *= 10.0; // factorization failed: more damping
                continue;
            }
            delta = solver.solve(-system.b);

            std::vector<Pose2D> candidate = poses;
            for (int i = 0; i < num_poses; i++) {
                candidate[i].x += delta(3*i);
                candidate[i].y += delta(3*i + 1);
                candidate[i].theta = wrap_angle(candidate[i].theta + delta(3*i + 2));
            }
            double candidate_cost = total_cost(candidate, edges);

            if (candidate_cost < curr_cost) {
                // better: keep it, and trust the model more
                poses = candidate;
                curr_cost = candidate_cost;
                lambda /= 10.0;
                step_accepted = true;
                break;
            }
            //worse: throw it away, trust the model less, try again
            lambda *= 10.0;
        }

        if (iter <= 5 || iter % 20 == 0) {
            std::cout << "iteration " << iter << ": cost = " << curr_cost << "  lambda = " << lambda << "\n";
        }

        //nothing accepted after 10 tries, or step is small, break
        if (!step_accepted || delta.norm() < 1.0e-6) {
            break;
        }
    }

    return poses;
}