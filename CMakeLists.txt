cmake_minimum_required(VERSION 3.1)
project(EKF)
set( CMAKE_CXX_STANDARD 17)

include_directories( include / )
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


# Library targets
add_library(robot src/robot.cpp)
add_library(ekf src/ekf.cpp)

# Executable
add_executable(ekf_main src/main.cpp)

# Link libraries to executable
target_link_libraries(ekf_main PRIVATE robot ekf)