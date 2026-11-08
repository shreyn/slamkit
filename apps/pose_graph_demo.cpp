#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <vector>
#include "slamkit/pose2d.hpp"
#include "slamkit/simulator.hpp"
#include "slamkit/pose_graph.hpp"

//RMS error between two lists of poses
double rms_position_error(std::vector<Pose2D> estimated, std::vector<Pose2D> truth) {
    double sum_squared_error = 0.0;
    for (int i = 0; i < estimated.size(); i++) {
        double dx = estimated[i].x - truth[i].x;
        double dy = estimated[i].y - truth[i].y;
        sum_squared_error += dx*dx + dy*dy;
    }
    return std::sqrt(sum_squared_error / estimated.size());
}

int main() {
    std::mt19937 wheel_generator(5); // same seed as simulate, so same drift

    //Settings
    int num_steps = 100;
    double position_noise_std = 0.02; // meters
    double angle_noise_std = 0.01;  // rad
    Eigen::Matrix3d information = diagonal_information(position_noise_std, angle_noise_std);

    Pose2D true_motion = {1.0, 0.0, 2.0 * M_PI / num_steps};

    std::vector<Pose2D> true_poses; 
    std::vector<Pose2D> noisy_poses; //optimizer's starting guesses
    std::vector<Edge> edges; //bands

    Pose2D true_pose = {0.0, 0.0, 0.0};
    Pose2D noisy_pose = {0.0, 0.0, 0.0};
    true_poses.push_back(true_pose); // dot 0
    noisy_poses.push_back(noisy_pose);

    //each step adds one dot and one wheel band
    for (int step = 1; step <= num_steps; step++) {
        Pose2D noisy_motion = add_odometry_noise(true_motion, wheel_generator, position_noise_std, angle_noise_std);
        true_pose = compose(true_pose, true_motion);
        noisy_pose = compose(noisy_pose, noisy_motion);
        true_poses.push_back(true_pose);
        noisy_poses.push_back(noisy_pose);

        Edge wheel_edge;
        wheel_edge.from_index = step-1; // moving from prev -> curr
        wheel_edge.to_index = step;
        wheel_edge.measurement = noisy_motion; // band's rest length = the wheel reading
        wheel_edge.information = information;
        edges.push_back(wheel_edge);
    }

    //Loop closure: (fake sensor by using the true relative pose (0,0,0))
    Edge loop_edge;
    loop_edge.from_index = 0; 
    loop_edge.to_index = num_steps;
    loop_edge.measurement = relative_pose(true_poses[0], true_poses[num_steps]);
    loop_edge.information = information;
    edges.push_back(loop_edge);

    std::cout << "iterations -> total cost\n";
    for (int iters = 0; iters <= 8; iters++) { //8 iters of optimize
        std::vector<Pose2D> result = optimize(noisy_poses, edges, iters);
        std::cout << iters << " -> " << total_cost(result, edges) << "\n";
    }

    std::vector<Pose2D> optimized_poses = optimize(noisy_poses, edges, 10);
    
    std::cout << "\nRMS position error vs truth:\n";
    std::cout << "  dead reckoning: " << rms_position_error(noisy_poses, true_poses) << " m\n";
    std::cout << "  optimized:      " << rms_position_error(optimized_poses, true_poses) << " m\n";

    //save for plotting
    std::ofstream output_file("pose_graph.csv");
    output_file << "step,true_x,true_y,noisy_x,noisy_y,opt_x,opt_y\n";
    for (int i = 0; i <= num_steps; i++) {
        output_file << i << ","
                    << true_poses[i].x << "," << true_poses[i].y << ","
                    << noisy_poses[i].x << "," << noisy_poses[i].y << ","
                    << optimized_poses[i].x << "," << optimized_poses[i].y << "\n";
    }

    return 0;
}