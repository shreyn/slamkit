#include "slamkit/pose_graph.hpp"
#include <fstream>
#include <iostream>
#include <sstream>

PoseGraph load_g2o(std::string filename) {
    PoseGraph graph;

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "Couldn't open " << filename << "\n";
        return graph;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream stream(line);
        std::string word;
        stream >> word; // the first word tells us what kind of line this is

        if (word == "VERTEX_SE2") { 
            int id; //dot id
            Pose2D pose;
            //stream is the text of the curr line. Each >> takes next piece of text, converts it to type of var on the right, stores it there
            stream >> id >> pose.x >> pose.y >> pose.theta;
            graph.poses.push_back(pose); //pushes pose to list
        }
        else if (word == "EDGE_SE2") { 
            Edge edge;
            double a, b, c, d, e, f;
            stream >> edge.from_index >> edge.to_index
                   >> edge.measurement.x >> edge.measurement.y >> edge.measurement.theta
                   >> a >> b >> c >> d >> e >> f; 

            //order of 6 numbers into 3x3 symmetric matrix:
            edge.information << a, b, c,
                                b, d, e,
                                c, e, f;
            graph.edges.push_back(edge);
        }
}

    return graph;
}