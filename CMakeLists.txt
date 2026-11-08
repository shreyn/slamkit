cmake_minimum_required(VERSION 3.20)
project(slamkit LANGUAGES CXX)

# Build with optimization unless told otherwise (Eigen is very slow without it)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

# The slamkit library (all our real code goes here)
add_library(slamkit src/pose2d.cpp src/simulator.cpp src/sensor.cpp src/ekf.cpp src/run_simulation.cpp src/NEES.cpp src/pose_graph.cpp src/g2o_loader.cpp)
target_include_directories(slamkit PUBLIC include)
target_link_libraries(slamkit PUBLIC Eigen3::Eigen)

# Simulate program: runs one simulation and writes the CSV files
add_executable(simulate apps/simulate.cpp)
target_link_libraries(simulate PRIVATE slamkit)

# NEES program: runs 100 simulations and writes nees.csv
add_executable(NEES_run apps/NEES_run.cpp)
target_link_libraries(NEES_run PRIVATE slamkit)

# Pose graph demo: optimizes a simulated lap with a loop closure
add_executable(pose_graph_demo apps/pose_graph_demo.cpp)
target_link_libraries(pose_graph_demo PRIVATE slamkit)

# g2o demo: optimizes any 2D g2o dataset (Intel, M3500, ...)
add_executable(g2o_demo apps/g2o_demo.cpp)
target_link_libraries(g2o_demo PRIVATE slamkit)

# GoogleTest: CMake downloads it for us
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.tar.gz
)
FetchContent_MakeAvailable(googletest)

# Turn on testing
enable_testing()

# First test (sanity check that GoogleTest works)
add_executable(first_test tests/first_test.cpp)
target_link_libraries(first_test PRIVATE GTest::gtest_main)
add_test(NAME first_test COMMAND first_test)

# Pose2D test
add_executable(pose2d_test tests/pose2d_test.cpp)
target_link_libraries(pose2d_test PRIVATE slamkit GTest::gtest_main)
add_test(NAME pose2d_test COMMAND pose2d_test)

# Simulator test
add_executable(simulator_test tests/simulator_test.cpp)
target_link_libraries(simulator_test PRIVATE slamkit GTest::gtest_main)
add_test(NAME simulator_test COMMAND simulator_test)

# Sensor test
add_executable(sensor_test tests/sensor_test.cpp)
target_link_libraries(sensor_test PRIVATE slamkit GTest::gtest_main)
add_test(NAME sensor_test COMMAND sensor_test)

# EKF test
add_executable(ekf_test tests/ekf_test.cpp)
target_link_libraries(ekf_test PRIVATE slamkit GTest::gtest_main)
add_test(NAME ekf_test COMMAND ekf_test)

# Pose graph test
add_executable(pose_graph_test tests/pose_graph_test.cpp)
target_link_libraries(pose_graph_test PRIVATE slamkit GTest::gtest_main)
add_test(NAME pose_graph_test COMMAND pose_graph_test)