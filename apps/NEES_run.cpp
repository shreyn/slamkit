#include <fstream>
#include <iostream>
#include <vector>
#include "slamkit/run_simulation.hpp"
#include "slamkit/NEES.hpp"

int main() {
    int num_runs = 100;
    int num_poses = 101; //0 to 100

    //sum of nees across runs
    std::vector<double> nees_sum(num_poses, 0.0);

    for (int run=0; run < num_runs; run++) {
        //diff seed each run
        SimulationResult result = run_simulation(5 + run, 1000 + run, false);
        std::vector<double> nees_list = compute_nees(result); //one runs NEES (one number per step)

        for (int step = 1; step < num_poses; step++) {
            nees_sum[step] += nees_list[step]; //for each step's bucket, update it with current run's nees_list
        }
    }

    //write the average NEES at each step
    std::ofstream nees_file("nees.csv");
    nees_file << "step,average_nees\n";
    double total = 0.0;
    for (int step = 1; step < num_poses; step++) {
        double average_nees = nees_sum[step] / num_runs; //average NEES at that step, across runs
        nees_file << step << "," << average_nees << "\n";
        total += average_nees; //adds up the averages: overall average NEES across all steps and runs
    }

    std::cout << "Overall average NEES: " << (total / (num_poses-1)) << " (honest filter is about 3)\n";
    return 0;
}