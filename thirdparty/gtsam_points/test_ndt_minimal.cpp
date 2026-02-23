#include <iostream>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/factors/integrated_ndt_factor.hpp>
#include <gtsam/inference/Symbol.h>

using namespace gtsam_points;
using gtsam::symbol_shorthand::X;

int main() {
  std::cout << "Creating target point cloud..." << std::endl;
  
  // Create a simple target point cloud
  std::vector<Eigen::Vector4d> target_points;
  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++) {
      target_points.push_back(Eigen::Vector4d(i * 0.5, j * 0.5, 0.0, 1.0));
    }
  }
  auto target = std::make_shared<PointCloudCPU>(target_points);
  std::cout << "Target has " << target->size() << " points" << std::endl;

  std::cout << "Creating target voxelmap..." << std::endl;
  auto target_voxels = std::make_shared<GaussianVoxelMapCPU>(1.0);
  
  std::cout << "Inserting points into voxelmap..." << std::endl;
  target_voxels->insert(*target);
  std::cout << "Voxelmap created successfully" << std::endl;

  // Create source point cloud
  std::vector<Eigen::Vector4d> source_points;
  for (const auto& pt : target_points) {
    source_points.push_back(pt);
  }
  auto source = std::make_shared<PointCloudCPU>(source_points);
  std::cout << "Source has " << source->size() << " points" << std::endl;

  std::cout << "Creating NDT factor..." << std::endl;
  try {
    auto factor = std::make_shared<IntegratedNDTFactor>(
      X(0), X(1),
      target_voxels,
      source
    );
    std::cout << "Factor created successfully!" << std::endl;
    std::cout << "Factor has " << factor->keys().size() << " keys" << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return 1;
  }

  std::cout << "Test passed!" << std::endl;
  return 0;
}
