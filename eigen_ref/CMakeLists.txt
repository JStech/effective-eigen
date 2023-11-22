project(eigen_ref)
cmake_minimum_required(VERSION 3.0)

find_package(Eigen3)
find_package(benchmark)

add_executable(eigen_ref eigen_ref.cpp)
target_link_libraries(eigen_ref Eigen3::Eigen)
target_link_libraries(eigen_ref benchmark::benchmark)

add_executable(io_example io_example.cpp)
target_link_libraries(io_example Eigen3::Eigen)
