#include <iostream>
#include "slamkit/run_simulation.hpp"

int main() {
    SimulationResult result = run_simulation(5, 10, true); 

    std::cout << "Noisy RMS position error: " << result.noisy_rms_error << " m\n";
    std::cout << "EKF RMS position error: " << result.ekf_rms_error << " m\n";
    
    return 0;
}