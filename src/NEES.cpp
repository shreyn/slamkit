#include "slamkit/NEES.hpp"


std::vector<double> compute_nees(SimulationResult result) {
    std::vector<double> nees_list;
    nees_list.push_back(0.0); //step 0
    for(int step = 1; step <result.true_poses.size(); step++) {
        Pose2D true_pose = result.true_poses[step];
        EkfState ekf = result.ekf_states[step];
        
        Eigen::Vector3d error; //guess - truth
        error(0) = ekf.mean(0) - true_pose.x;
        error(1) = ekf.mean(1) - true_pose.y;
        error(2) = wrap_angle(ekf.mean(2) - true_pose.theta);

        double nees = error.transpose() * ekf.covariance.inverse() * error;
        nees_list.push_back(nees);
    }
    return nees_list;
}