cmake_minimum_required(VERSION 3.10)
project(LidarRegistration)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")


find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Iridescence)
find_package(Armadillo REQUIRED)

option(BUILD_WITH_OPENMP "Build with OpenMP" ON)
option(BUILD_WITH_CUDA "Build with CUDA" OFF)


# Thirdparty
add_subdirectory(thirdparty/gtsam_points)
add_subdirectory(thirdparty)


# build
include_directories(
    thirdparty/gtsam_points/include  # gtsam_points 
    thirdparty/spdlog/include        # spdlog 
    include                          # loam_feature.hpp
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${GTSAM_INCLUDE_DIR}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(lidar_registration_benchmark 
    src/main.cpp
    src/loam_feature.cpp
    src/featureExtraction.cpp
    src/ndt/integrated_ndt_factor.cpp
    src/ndt/integrated_light_ndt_factor.cpp
    src/gmm/gmm_voxelmap_cpu.cpp
    src/gmm/mixture_em_backend.cpp
    src/gmm/integrated_mixture_light_ndt_factor.cpp
)
target_link_libraries(lidar_registration_benchmark
    ${PCL_LIBRARIES}
    gtsam
    gtsam_points   
    OpenMP::OpenMP_CXX
    ${ARMADILLO_LIBRARIES}
)
target_include_directories(lidar_registration_benchmark PRIVATE ${ARMADILLO_INCLUDE_DIRS})

if(TARGET Iridescence::Iridescence)
  target_link_libraries(lidar_registration_benchmark Iridescence::Iridescence)
endif()


# test_all_factors는 사용하지 않으므로 빌드 제외
# add_executable(test_all_factors
#     src/test_all_factors.cpp
#     src/loam_feature.cpp
# )
# target_link_libraries(test_all_factors
#     ${PCL_LIBRARIES}
#     gtsam
#     gtsam_points   
#     OpenMP::OpenMP_CXX
# )

# GMM voxel tests
find_package(GTest REQUIRED)
enable_testing()

add_executable(test_gmm_voxelmap
    src/gmm/test/test_gmm_voxelmap.cpp
    src/gmm/gmm_voxelmap_cpu.cpp
    src/gmm/mixture_em_backend.cpp
    src/gmm/integrated_mixture_light_ndt_factor.cpp
)
target_link_libraries(test_gmm_voxelmap
    GTest::gtest
    GTest::gtest_main
    gtsam
    gtsam_points
    ${PCL_LIBRARIES}
    ${ARMADILLO_LIBRARIES}
    OpenMP::OpenMP_CXX
)
target_include_directories(test_gmm_voxelmap PRIVATE ${ARMADILLO_INCLUDE_DIRS})

add_test(NAME GMMVoxelTests COMMAND test_gmm_voxelmap)
