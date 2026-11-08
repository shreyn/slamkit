#pragma once

#include <vector>
#include "slamkit/pose2d.hpp"
#include "slamkit/ekf.hpp"

// Everything one simulation run produces.
struct SimulationResult {
    std::vector<Pose2D> true_poses; //where robot actual was
    std::vector<EkfState> ekf_states; 
    double noisy_rms_error; // meters
    double ekf_rms_error;  // meters
};

// Runs one full simulation. Seeds for randoms. If write_files true, then creates the csvs. 
SimulationResult run_simulation(int wheel_seed, int sensor_seed, bool write_files);