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

    //optimize
    std::vector<Pose2D> poses = graph.poses;
    for (int iteration = 1; iteration <= 20; iteration++) { //20 now, M3500 may need more than 10
        auto start_time = std::chrono::steady_clock::now();
        poses = optimize_sparse(poses, graph.edges, 1); //sparse now
        auto end_time = std::chrono::steady_clock::now();

        double seconds = std::chrono::duration<double>(end_time - start_time).count();
        double cost = total_cost(poses, graph.edges);
        std::cout << "iteration " << iteration << ": cost = " << cost << "  (" << seconds << " s)\n";

        //stop if the optimizer blew up
        if (std::isnan(cost)) {
            std::cout << "Cost is not a number: the optimizer diverged.\n";
            break;
        }
    }

    //save for plots
    std::ofstream output_file(output_filename);
    output_file << "index,start_x,start_y,start_theta,opt_x,opt_y,opt_theta\n";
    for (int i = 0; i < graph.poses.size(); i++) {
        output_file << i << "," << graph.poses[i].x << "," << graph.poses[i].y << "," << graph.poses[i].theta << "," << poses[i].x << "," << poses[i].y << "," << poses[i].theta << "\n";
    }

    return 0;
}