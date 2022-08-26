cmake_minimum_required(VERSION 3.10)

project(optimized_icp)

set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

add_executable(optimized_icp main.cpp optimized_ICP_GN.cpp)
target_link_libraries(optimized_icp ${PCL_LIBRARIES})