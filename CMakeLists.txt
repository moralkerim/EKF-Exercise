cmake_minimum_required(VERSION 3.1)
project(EKF)
set( CMAKE_CXX_STANDARD 17)

include_directories( include / )
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


add_library(ekf_lib src/ekf.cpp)
add_executable(ekf_main src/main.cpp)
target_link_libraries(ekf_main ekf_lib)
