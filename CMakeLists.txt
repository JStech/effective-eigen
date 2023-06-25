cmake_minimum_required (VERSION 3.0)
project (efficient_eigen)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable (efficient_eigen efficient_eigen.cpp)
target_link_libraries (efficient_eigen Eigen3::Eigen)
