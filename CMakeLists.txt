cmake_minimum_required(VERSION 3.10)
project(LidarRegistration)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")


find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Iridescence REQUIRED)

option(BUILD_WITH_OPENMP "Build with OpenMP" ON)
option(BUILD_WITH_CUDA "Build with CUDA" OFF)


add_subdirectory(include/gtsam_points)
# Thirdparty
add_subdirectory(thirdparty)


# build
include_directories(
    include/gtsam_points/include  # gtsam_points 
    thirdparty/spdlog/include     # spdlog 
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${GTSAM_INCLUDE_DIR}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(calculate_rt src/main.cpp)
target_link_libraries(calculate_rt
    ${PCL_LIBRARIES}
    gtsam
    gtsam_points   
    OpenMP::OpenMP_CXX
    Iridescence::Iridescence
)
