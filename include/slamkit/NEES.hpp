#pragma once

#include <vector>
#include "slamkit/run_simulation.hpp"

//Compute NESS at every step of a simulation run. elem 0 has 0 uncertainty
std::vector<double> compute_nees(SimulationResult result);