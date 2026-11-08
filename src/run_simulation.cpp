#include "slamkit/run_simulation.hpp"
#include <cmath>
#include <fstream>
#include <random>
#include "slamkit/simulator.hpp"
#include "slamkit/sensor.hpp"

SimulationResult run_simulation(int wheel_seed, int sensor_seed, bool write_files) {
    std::mt19937 wheel_generator(wheel_seed); //wheel noise
    std::mt19937 sensor_generator(sensor_seed);  // separate seed for sensor noise

    // Settings
    int num_steps = 100;
    double position_noise_std = 0.02; // meters
    double angle_noise_std = 0.01; // rad
    double range_noise_std = 0.1;  // meters
    double bearing_noise_std = 0.02;  // rad
    double max_sensor_range = 10.0; // meters

    // world: 8 fixed landmarks
    int num_landmarks = 8;
    Landmark landmarks[8] = {
        {8.0, 16.0}, {-8.0, 16.0}, {0.0, 24.0}, {0.0, 8.0},
        {24.0, 16.0}, {-24.0, 16.0}, {0.0, 40.0}, {0.0, -8.0}
    };

    //1 meter forward, turn a little
    Pose2D true_motion = {1.0, 0.0, 2.0 * M_PI / num_steps};

    //start at orign, facing east
    Pose2D true_pose = {0.0, 0.0, 0.0};
    Pose2D noisy_pose = {0.0, 0.0, 0.0};

    //starts at origin, with zero uncertainty
    EkfState ekf;
    ekf.mean << 0.0, 0.0, 0.0;
    ekf.covariance.setZero();

    double noisy_sum_squared_error = 0.0;
    double ekf_sum_squared_error = 0.0;

    // info that is returned.
    SimulationResult result;
    result.true_poses.push_back(true_pose); //starting values are elem 0
    result.ekf_states.push_back(ekf);

    //files (only opened if write_files is true)
    std::ofstream landmark_file;
    std::ofstream output_file;
    std::ofstream measurement_file;

    if (write_files) {
        // Landmarks file
        landmark_file.open("landmarks.csv");
        landmark_file << "id,x,y\n";
        for (int i = 0; i < num_landmarks; i++) {
            landmark_file << i << "," << landmarks[i].x << "," << landmarks[i].y << "\n";
        }

        // Paths file
        output_file.open("path.csv");
        output_file << "step,true_x,true_y,true_theta,noisy_x,noisy_y,noisy_theta,ekf_x,ekf_y,ekf_theta\n";
        output_file << 0 << ","
                    << true_pose.x << "," << true_pose.y << "," << true_pose.theta << ","
                    << noisy_pose.x << "," << noisy_pose.y << "," << noisy_pose.theta << ","
                    << ekf.mean(0) << "," << ekf.mean(1) << "," << ekf.mean(2) << "\n";

        // Sensor file
        measurement_file.open("measurements.csv");
        measurement_file << "step,landmark_id,range,bearing\n";
    }

    // Loop
    for (int step = 1; step <= num_steps; step++) {
        //move true robot and noisy robot
        Pose2D noisy_motion = add_odometry_noise(true_motion, wheel_generator,
                                                 position_noise_std, angle_noise_std);
        true_pose = compose(true_pose, true_motion);
        noisy_pose = compose(noisy_pose, noisy_motion);

        //EKF predict, using the same noisy wheel reading
        ekf_predict(ekf, noisy_motion, position_noise_std, angle_noise_std);

        //for each landmark in range (measured from the TRUE pose), record it and run an EKF update
        for (int i=0; i < num_landmarks; i++) {
            Measurement perfect = measure_landmark(true_pose, landmarks[i]);
            if (perfect.range <= max_sensor_range) { // within sensor range
                Measurement noisy = measure_landmark_noisy(true_pose, landmarks[i],
                                                           sensor_generator,
                                                           range_noise_std,
                                                           bearing_noise_std);
                if (write_files) {
                    measurement_file << step << "," << i << "," << noisy.range << "," << noisy.bearing << "\n";
                }
                //update:
                ekf_update(ekf, landmarks[i], noisy, range_noise_std, bearing_noise_std);
            }
        }

        if (write_files) {
            output_file << step << "," << true_pose.x << "," << true_pose.y << "," << true_pose.theta << "," << noisy_pose.x << "," << noisy_pose.y << "," << noisy_pose.theta << "," << ekf.mean(0) << "," << ekf.mean(1) << "," << ekf.mean(2) << "\n";
        }

        //save this step's values in the lists that are returned
        result.true_poses.push_back(true_pose);
        result.ekf_states.push_back(ekf);

        //position errors
        double noisy_dx = noisy_pose.x - true_pose.x;
        double noisy_dy = noisy_pose.y - true_pose.y;
        double ekf_dx = ekf.mean(0) - true_pose.x;
        double ekf_dy = ekf.mean(1) - true_pose.y;
        noisy_sum_squared_error += noisy_dx * noisy_dx + noisy_dy*noisy_dy;
        ekf_sum_squared_error += ekf_dx * ekf_dx + ekf_dy * ekf_dy;
    }

    // Root mean squared position error = square root of the average squared error
    result.noisy_rms_error = std::sqrt(noisy_sum_squared_error / num_steps);
    result.ekf_rms_error = std::sqrt(ekf_sum_squared_error / num_steps);

    return result;
}