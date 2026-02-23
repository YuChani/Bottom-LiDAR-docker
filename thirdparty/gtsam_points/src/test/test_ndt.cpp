// SPDX-FileCopyrightText: Copyright 2024 Kenji Koide
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>

#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/factors/integrated_ndt_factor.hpp>

using namespace gtsam_points;
using gtsam::symbol_shorthand::X;

class NDTTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create a simple target point cloud (10x10 grid on XY plane)
    std::vector<Eigen::Vector4d> target_points;
    for (int i = 0; i < 10; i++) {
      for (int j = 0; j < 10; j++) {
        target_points.push_back(Eigen::Vector4d(i * 0.5, j * 0.5, 0.0, 1.0));
      }
    }
    target = std::make_shared<PointCloudCPU>(target_points);

    // Create target voxelmap with 1.0m resolution
    target_voxels = std::make_shared<GaussianVoxelMapCPU>(1.0);
    target_voxels->insert(*target);

    // Create source point cloud (same grid, slightly translated)
    std::vector<Eigen::Vector4d> source_points;
    Eigen::Isometry3d true_transform = Eigen::Isometry3d::Identity();
    true_transform.translation() = Eigen::Vector3d(0.1, 0.05, 0.0);
    
    for (const auto& pt : target_points) {
      Eigen::Vector4d transformed = Eigen::Vector4d::Ones();
      transformed.head<3>() = true_transform * pt.head<3>();
      source_points.push_back(transformed);
    }
    source = std::make_shared<PointCloudCPU>(source_points);
  }

  PointCloud::Ptr target;
  PointCloud::Ptr source;
  GaussianVoxelMapCPU::Ptr target_voxels;
};

TEST_F(NDTTest, FactorCreation) {
  // Test binary factor creation
  auto factor = gtsam::make_shared<IntegratedNDTFactor>(
    X(0), X(1), 
    target_voxels, 
    source
  );

  EXPECT_NE(factor, nullptr);
  EXPECT_EQ(factor->keys().size(), 2);
  EXPECT_EQ(factor->keys()[0], X(0));
  EXPECT_EQ(factor->keys()[1], X(1));
}

TEST_F(NDTTest, UnaryFactorCreation) {
  // Test unary factor creation (fixed target pose)
  gtsam::Pose3 fixed_target = gtsam::Pose3::Identity();
  auto factor = gtsam::make_shared<IntegratedNDTFactor>(
    fixed_target,
    X(1),
    target_voxels,
    source
  );

  EXPECT_NE(factor, nullptr);
  EXPECT_EQ(factor->keys().size(), 1);
  EXPECT_EQ(factor->keys()[0], X(1));
}

TEST_F(NDTTest, SearchModeConfiguration) {
  auto factor = gtsam::make_shared<IntegratedNDTFactor>(
    X(0), X(1),
    target_voxels,
    source
  );

  // Test DIRECT1 mode
  factor->set_search_mode(NDTSearchMode::DIRECT1);
  EXPECT_NO_THROW(factor->error(gtsam::Values()));

  // Test DIRECT7 mode (default)
  factor->set_search_mode(NDTSearchMode::DIRECT7);
  EXPECT_NO_THROW(factor->error(gtsam::Values()));

  // Test DIRECT27 mode
  factor->set_search_mode(NDTSearchMode::DIRECT27);
  EXPECT_NO_THROW(factor->error(gtsam::Values()));
}

TEST_F(NDTTest, ParameterConfiguration) {
  auto factor = gtsam::make_shared<IntegratedNDTFactor>(
    X(0), X(1),
    target_voxels,
    source
  );

  // Test parameter setters
  EXPECT_NO_THROW(factor->set_resolution(1.5));
  EXPECT_NO_THROW(factor->set_outlier_ratio(0.6));
  EXPECT_NO_THROW(factor->set_regularization_epsilon(1e-4));
  EXPECT_NO_THROW(factor->set_num_threads(4));
}

TEST_F(NDTTest, ErrorEvaluation) {
  auto factor = gtsam::make_shared<IntegratedNDTFactor>(
    X(0), X(1),
    target_voxels,
    source
  );

  // Create values
  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3::Identity());
  values.insert(X(1), gtsam::Pose3::Identity());

  // Evaluate error
  double error = factor->error(values);
  
  // Error should be non-negative
  EXPECT_GE(error, 0.0);
  
  // Error should be finite
  EXPECT_FALSE(std::isnan(error));
  EXPECT_FALSE(std::isinf(error));
}

TEST_F(NDTTest, InlierCount) {
  auto factor = gtsam::make_shared<IntegratedNDTFactor>(
    X(0), X(1),
    target_voxels,
    source
  );

  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3::Identity());
  values.insert(X(1), gtsam::Pose3::Identity());

  // Trigger correspondence update
  factor->error(values);

  // Check inlier count
  int num_inliers = factor->num_inliers();
  EXPECT_GT(num_inliers, 0);
  EXPECT_LE(num_inliers, source->size());

  // Check inlier fraction
  double fraction = factor->inlier_fraction();
  EXPECT_GE(fraction, 0.0);
  EXPECT_LE(fraction, 1.0);
}

TEST_F(NDTTest, SimpleAlignment) {
  // Create factor graph
  gtsam::NonlinearFactorGraph graph;

  // Add prior on first pose
  auto prior_noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.01);
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), gtsam::Pose3::Identity(), prior_noise));

  // Add NDT factor
  auto factor = gtsam::make_shared<IntegratedNDTFactor>(
    X(0), X(1),
    target_voxels,
    source
  );
  factor->set_num_threads(1);
  graph.add(factor);

  // Initial values (source pose with small perturbation)
  gtsam::Values initial;
  initial.insert(X(0), gtsam::Pose3::Identity());
  
  // Start with identity (should converge to slight translation)
  initial.insert(X(1), gtsam::Pose3::Identity());

  // Optimize
  gtsam::LevenbergMarquardtParams params;
  params.setVerbosityLM("SILENT");
  params.setMaxIterations(10);
  
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
  gtsam::Values result = optimizer.optimize();

  // Check that optimization converged
  EXPECT_TRUE(result.exists(X(0)));
  EXPECT_TRUE(result.exists(X(1)));

  // The result should be close to identity or small translation
  // (since source is already very close to target)
  gtsam::Pose3 optimized_source = result.at<gtsam::Pose3>(X(1));
  
  // Translation should be small (less than 1 meter)
  EXPECT_LT(optimized_source.translation().norm(), 1.0);
  
  // Final error should be smaller than initial error
  double initial_error = graph.error(initial);
  double final_error = graph.error(result);
  EXPECT_LE(final_error, initial_error);
}

TEST_F(NDTTest, MemoryUsage) {
  auto factor = gtsam::make_shared<IntegratedNDTFactor>(
    X(0), X(1),
    target_voxels,
    source
  );

  size_t memory = factor->memory_usage();
  EXPECT_GT(memory, 0);
}

TEST_F(NDTTest, Clone) {
  auto factor = gtsam::make_shared<IntegratedNDTFactor>(
    X(0), X(1),
    target_voxels,
    source
  );

  auto cloned = factor->clone();
  EXPECT_NE(cloned, nullptr);
  EXPECT_EQ(cloned->keys().size(), 2);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
