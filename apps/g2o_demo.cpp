#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "slamkit/pose_graph.hpp"

int main(int argc, char** argv) {
    //file names come from the command line (same as C: argv[1], argv[2])
    if (argc < 3) {
        std::cout << "Usage: g2o_demo <input.g2o> <output.csv>\n";
        return 1;
    }
    std::string input_filename = argv[1];
    std::string output_filename = argv[2];

    PoseGraph graph = load_g2o(input_filename);
    std::cout << "Loaded " << graph.poses.size() << " poses and "
              << graph.edges.size() << " edges\n";

    //stop if the file was not found
    if (graph.poses.empty()) {
        return 1;
    }

    //cost before optimizing
    std::cout << "Starting cost: " << total_cost(graph.poses, graph.edges) << "\n\n";

    //optimize with Levenberg-Marquardt (prints one line per iteration)
    auto start_time = std::chrono::steady_clock::now();
    std::vector<Pose2D> poses = optimize_lm(graph.poses, graph.edges, 2000);
    auto end_time = std::chrono::steady_clock::now();

    double seconds = std::chrono::duration<double>(end_time - start_time).count();
    double final_cost = total_cost(poses, graph.edges);
    std::cout << "\nFinal cost: " << final_cost << "  (" << seconds << " s total)\n";

    //stop if the optimizer blew up
    if (std::isnan(final_cost)) {
        std::cout << "Cost is not a number: the optimizer diverged.\n";
        return 1;
    }

    //save for plots
    std::ofstream output_file(output_filename);
    output_file << "index,start_x,start_y,start_theta,opt_x,opt_y,opt_theta\n";
    for (int i = 0; i < graph.poses.size(); i++) {
        output_file << i << "," << graph.poses[i].x << "," << graph.poses[i].y << "," << graph.poses[i].theta << "," << poses[i].x << "," << poses[i].y << "," << poses[i].theta << "\n";
    }

    return 0;
}