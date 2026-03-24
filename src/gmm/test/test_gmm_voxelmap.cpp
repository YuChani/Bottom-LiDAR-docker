// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <random>
#include <cmath>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include "gmm/gmm_voxelmap_cpu.hpp"
#include "gmm/mixture_em_backend.hpp"

#include <gtsam_points/types/point_cloud_cpu.hpp>

namespace {

std::vector<Eigen::Vector4d> generate_cluster(
    const Eigen::Vector3d& center,
    double sigma,
    int count,
    std::mt19937& rng) {
  std::normal_distribution<double> dist(0.0, sigma);
  std::vector<Eigen::Vector4d> points;
  points.reserve(count);
  for (int i = 0; i < count; i++) {
    Eigen::Vector4d pt;
    pt << center.x() + dist(rng), center.y() + dist(rng), center.z() + dist(rng), 0.0;
    points.push_back(pt);
  }
  return points;
}

gtsam_points::PointCloudCPU::Ptr make_point_cloud(const std::vector<Eigen::Vector4d>& points) {
  auto cloud = std::make_shared<gtsam_points::PointCloudCPU>();
  cloud->num_points = points.size();
  cloud->points_storage = points;
  cloud->points = cloud->points_storage.data();

  cloud->covs_storage.resize(points.size(), Eigen::Matrix4d::Identity() * 0.01);
  cloud->covs = cloud->covs_storage.data();

  return cloud;
}

}  // namespace

// ============================================================
// Phase 1: GMMVoxel tests
// ============================================================

TEST(GMMVoxel, DefaultConstructEmpty) {
  gtsam_points::GMMVoxel voxel;
  EXPECT_EQ(voxel.size(), 0u);
  EXPECT_TRUE(voxel.components().empty());
  EXPECT_TRUE(voxel.reservoir().empty());
  EXPECT_FALSE(voxel.is_dirty());
}

TEST(GMMVoxel, ReservoirAccumulates) {
  gtsam_points::GMMVoxel voxel;
  gtsam_points::GMMVoxel::Setting setting;
  setting.reservoir_capacity = 10;

  std::mt19937 rng(123);
  auto pts = generate_cluster(Eigen::Vector3d(1, 2, 3), 0.5, 5, rng);
  auto cloud = make_point_cloud(pts);

  for (size_t i = 0; i < cloud->num_points; i++) {
    voxel.add(setting, *cloud, i);
  }
  EXPECT_EQ(voxel.reservoir().size(), 5u);
  EXPECT_TRUE(voxel.is_dirty());

  auto more_pts = generate_cluster(Eigen::Vector3d(1, 2, 3), 0.5, 20, rng);
  auto more_cloud = make_point_cloud(more_pts);
  for (size_t i = 0; i < more_cloud->num_points; i++) {
    voxel.add(setting, *more_cloud, i);
  }
  EXPECT_EQ(voxel.reservoir().size(), 10u);
}

TEST(GMMVoxel, FinalizeStubSingleComponent) {
  gtsam_points::GMMVoxel voxel;
  gtsam_points::GMMVoxel::Setting setting;
  setting.reservoir_capacity = 256;
  setting.covariance_regularization = 1e-3;

  std::mt19937 rng(42);
  auto pts = generate_cluster(Eigen::Vector3d(1, 2, 3), 0.5, 50, rng);
  auto cloud = make_point_cloud(pts);

  for (size_t i = 0; i < cloud->num_points; i++) {
    voxel.add(setting, *cloud, i);
  }
  voxel.finalize();

  EXPECT_EQ(voxel.size(), 1u);
  EXPECT_NEAR(voxel.components()[0].weight, 1.0, 1e-10);

  const auto& comp_mean = voxel.components()[0].mean;
  EXPECT_NEAR(comp_mean(0), 1.0, 0.5);
  EXPECT_NEAR(comp_mean(1), 2.0, 0.5);
  EXPECT_NEAR(comp_mean(2), 3.0, 0.5);
  EXPECT_NEAR(comp_mean(3), 0.0, 1e-10);
}

TEST(GMMVoxel, DirtyFlagBehavior) {
  gtsam_points::GMMVoxel voxel;
  gtsam_points::GMMVoxel::Setting setting;
  setting.reservoir_capacity = 256;
  setting.covariance_regularization = 1e-3;

  std::mt19937 rng(42);
  auto pts = generate_cluster(Eigen::Vector3d(0, 0, 0), 1.0, 10, rng);
  auto cloud = make_point_cloud(pts);

  EXPECT_FALSE(voxel.is_dirty());

  voxel.add(setting, *cloud, 0);
  EXPECT_TRUE(voxel.is_dirty());

  voxel.finalize();
  EXPECT_FALSE(voxel.is_dirty());

  // Second finalize without add should be no-op
  const size_t size_before = voxel.size();
  voxel.finalize();
  EXPECT_EQ(voxel.size(), size_before);
  EXPECT_FALSE(voxel.is_dirty());
}

TEST(GMMVoxelMapCPU, InstantiatesAndInserts) {
  auto voxelmap = std::make_shared<gtsam_points::GMMVoxelMapCPU>(1.0);

  std::mt19937 rng(42);
  auto pts = generate_cluster(Eigen::Vector3d(0, 0, 0), 2.0, 100, rng);
  auto cloud = make_point_cloud(pts);

  voxelmap->insert(*cloud);
  EXPECT_GT(voxelmap->num_voxels(), 0u);

  auto base_ptr = std::dynamic_pointer_cast<gtsam_points::GaussianVoxelMap>(voxelmap);
  EXPECT_NE(base_ptr, nullptr);
}

TEST(GMMVoxelMapCPU, KnnSearchReturnsResults) {
  auto voxelmap = std::make_shared<gtsam_points::GMMVoxelMapCPU>(1.0);

  std::mt19937 rng(42);
  auto pts = generate_cluster(Eigen::Vector3d(0, 0, 0), 0.3, 50, rng);
  auto cloud = make_point_cloud(pts);

  voxelmap->insert(*cloud);

  Eigen::Vector4d query(0, 0, 0, 1.0);
  size_t k_index;
  double k_sq_dist;
  size_t found = voxelmap->knn_search(query.data(), 1, &k_index, &k_sq_dist);
  EXPECT_GE(found, 1u);
}

// ============================================================
// Phase 2: EM backend tests
// ============================================================

TEST(GMMFit, SingleCluster) {
  std::mt19937 rng(42);
  auto pts = generate_cluster(Eigen::Vector3d(1, 2, 3), 0.5, 200, rng);

  gtsam_points::GMMFitParams params;
  params.max_components = 3;
  auto result = gtsam_points::fit_gmm(pts, params);

  EXPECT_TRUE(result.converged);
  // Should collapse to 1 component since data is unimodal
  EXPECT_GE(result.components.size(), 1u);
  EXPECT_LE(result.components.size(), 3u);

  // Check the dominant component is near the true mean
  double max_weight = 0;
  int best_k = 0;
  for (size_t k = 0; k < result.components.size(); k++) {
    if (result.components[k].weight > max_weight) {
      max_weight = result.components[k].weight;
      best_k = k;
    }
  }
  EXPECT_NEAR(result.components[best_k].mean(0), 1.0, 0.5);
  EXPECT_NEAR(result.components[best_k].mean(1), 2.0, 0.5);
  EXPECT_NEAR(result.components[best_k].mean(2), 3.0, 0.5);
}

TEST(GMMFit, BimodalSeparation) {
  std::mt19937 rng(42);
  auto pts1 = generate_cluster(Eigen::Vector3d(0, 0, 0), 0.1, 100, rng);
  auto pts2 = generate_cluster(Eigen::Vector3d(5, 0, 0), 0.1, 100, rng);

  std::vector<Eigen::Vector4d> all_pts;
  all_pts.insert(all_pts.end(), pts1.begin(), pts1.end());
  all_pts.insert(all_pts.end(), pts2.begin(), pts2.end());

  gtsam_points::GMMFitParams params;
  params.max_components = 3;
  auto result = gtsam_points::fit_gmm(all_pts, params);

  EXPECT_TRUE(result.converged);
  EXPECT_GE(result.components.size(), 2u);

  // Sort by mean x for deterministic checking
  auto comps = result.components;
  std::sort(comps.begin(), comps.end(),
            [](const auto& a, const auto& b) { return a.mean(0) < b.mean(0); });

  EXPECT_NEAR(comps.front().mean(0), 0.0, 0.3);
  EXPECT_NEAR(comps.back().mean(0), 5.0, 0.3);
}

TEST(GMMFit, TrimodalSeparation) {
  std::mt19937 rng(42);
  auto pts1 = generate_cluster(Eigen::Vector3d(0, 0, 0), 0.1, 100, rng);
  auto pts2 = generate_cluster(Eigen::Vector3d(5, 0, 0), 0.1, 100, rng);
  auto pts3 = generate_cluster(Eigen::Vector3d(0, 5, 0), 0.1, 100, rng);

  std::vector<Eigen::Vector4d> all_pts;
  all_pts.insert(all_pts.end(), pts1.begin(), pts1.end());
  all_pts.insert(all_pts.end(), pts2.begin(), pts2.end());
  all_pts.insert(all_pts.end(), pts3.begin(), pts3.end());

  gtsam_points::GMMFitParams params;
  params.max_components = 3;
  auto result = gtsam_points::fit_gmm(all_pts, params);

  EXPECT_TRUE(result.converged);
  EXPECT_EQ(result.components.size(), 3u);

  for (const auto& comp : result.components) {
    EXPECT_NEAR(comp.weight, 1.0 / 3.0, 0.15);
  }
}

TEST(GMMFit, TooFewPointsFallback) {
  std::vector<Eigen::Vector4d> pts = {Eigen::Vector4d(1, 2, 3, 0)};

  gtsam_points::GMMFitParams params;
  auto result = gtsam_points::fit_gmm(pts, params);

  EXPECT_EQ(result.components.size(), 1u);
  EXPECT_NEAR(result.components[0].mean(0), 1.0, 1e-10);
}

TEST(GMMFit, CovariancePD) {
  std::mt19937 rng(42);
  auto pts = generate_cluster(Eigen::Vector3d(0, 0, 0), 1.0, 200, rng);

  gtsam_points::GMMFitParams params;
  params.max_components = 3;
  auto result = gtsam_points::fit_gmm(pts, params);

  for (const auto& comp : result.components) {
    // Check 3x3 block is positive definite
    Eigen::Matrix3d cov3 = comp.cov.block<3, 3>(0, 0);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov3);
    EXPECT_GT(solver.eigenvalues().minCoeff(), 0.0);
  }
}

TEST(GMMFit, WeightsSumToOne) {
  std::mt19937 rng(42);
  auto pts1 = generate_cluster(Eigen::Vector3d(0, 0, 0), 0.1, 100, rng);
  auto pts2 = generate_cluster(Eigen::Vector3d(5, 0, 0), 0.1, 100, rng);

  std::vector<Eigen::Vector4d> all_pts;
  all_pts.insert(all_pts.end(), pts1.begin(), pts1.end());
  all_pts.insert(all_pts.end(), pts2.begin(), pts2.end());

  gtsam_points::GMMFitParams params;
  params.max_components = 3;
  auto result = gtsam_points::fit_gmm(all_pts, params);

  double total_weight = 0.0;
  for (const auto& comp : result.components) {
    total_weight += comp.weight;
  }
  EXPECT_NEAR(total_weight, 1.0, 1e-6);
}

TEST(GMMFit, WarmStartDoesNotDiverge) {
  std::mt19937 rng(42);
  auto pts = generate_cluster(Eigen::Vector3d(1, 2, 3), 0.5, 200, rng);

  gtsam_points::GMMFitParams params;
  params.max_components = 2;

  auto cold_result = gtsam_points::fit_gmm(pts, params);
  auto warm_result = gtsam_points::fit_gmm(pts, params, cold_result.components);

  EXPECT_TRUE(warm_result.converged);
  EXPECT_GE(warm_result.components.size(), 1u);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
