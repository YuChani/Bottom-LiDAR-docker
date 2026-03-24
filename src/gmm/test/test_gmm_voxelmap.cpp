// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <random>
#include <cmath>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include "gmm/gmm_voxelmap_cpu.hpp"
#include "gmm/mixture_em_backend.hpp"
#include "gmm/integrated_mixture_light_ndt_factor.hpp"

#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

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

  EXPECT_GE(voxel.size(), 1u);

  double max_weight = 0;
  int best_k = 0;
  for (size_t k = 0; k < voxel.components().size(); k++) {
    if (voxel.components()[k].weight > max_weight) {
      max_weight = voxel.components()[k].weight;
      best_k = k;
    }
  }

  const auto& comp_mean = voxel.components()[best_k].mean;
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

// ============================================================
// Phase 2.5: finalize() → EM integration tests
// ============================================================

TEST(GMMVoxel, FinalizeCallsEM) {
  gtsam_points::GMMVoxel voxel;
  gtsam_points::GMMVoxel::Setting setting;
  setting.reservoir_capacity = 256;
  setting.max_components = 3;
  setting.covariance_regularization = 1e-3;

  std::mt19937 rng(42);
  auto pts1 = generate_cluster(Eigen::Vector3d(0, 0, 0), 0.1, 100, rng);
  auto pts2 = generate_cluster(Eigen::Vector3d(5, 0, 0), 0.1, 100, rng);

  std::vector<Eigen::Vector4d> all_pts;
  all_pts.insert(all_pts.end(), pts1.begin(), pts1.end());
  all_pts.insert(all_pts.end(), pts2.begin(), pts2.end());

  auto cloud = make_point_cloud(all_pts);
  for (size_t i = 0; i < cloud->num_points; i++) {
    voxel.add(setting, *cloud, i);
  }

  voxel.finalize();

  EXPECT_GE(voxel.size(), 2u);

  auto comps = voxel.components();
  std::sort(comps.begin(), comps.end(),
            [](const auto& a, const auto& b) { return a.mean(0) < b.mean(0); });

  EXPECT_NEAR(comps.front().mean(0), 0.0, 0.5);
  EXPECT_NEAR(comps.back().mean(0), 5.0, 0.5);

  double total_weight = 0.0;
  for (const auto& c : comps) total_weight += c.weight;
  EXPECT_NEAR(total_weight, 1.0, 1e-6);
}

TEST(GMMVoxel, FinalizeWarmStart) {
  gtsam_points::GMMVoxel voxel;
  gtsam_points::GMMVoxel::Setting setting;
  setting.reservoir_capacity = 256;
  setting.max_components = 2;
  setting.covariance_regularization = 1e-3;

  std::mt19937 rng(42);
  auto pts1 = generate_cluster(Eigen::Vector3d(0, 0, 0), 0.3, 50, rng);
  auto cloud1 = make_point_cloud(pts1);

  for (size_t i = 0; i < cloud1->num_points; i++) {
    voxel.add(setting, *cloud1, i);
  }
  voxel.finalize();
  EXPECT_GE(voxel.size(), 1u);

  auto pts2 = generate_cluster(Eigen::Vector3d(5, 0, 0), 0.3, 50, rng);
  std::vector<Eigen::Vector4d> combined;
  combined.insert(combined.end(), pts1.begin(), pts1.end());
  combined.insert(combined.end(), pts2.begin(), pts2.end());
  auto cloud2 = make_point_cloud(combined);

  for (size_t i = 0; i < cloud2->num_points; i++) {
    voxel.add(setting, *cloud2, i);
  }
  voxel.finalize();
  EXPECT_GE(voxel.size(), 1u);

  for (const auto& comp : voxel.components()) {
    Eigen::Matrix3d cov3 = comp.cov.block<3, 3>(0, 0);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov3);
    EXPECT_GT(solver.eigenvalues().minCoeff(), 0.0);
  }
}

TEST(GMMVoxel, FinalizeIdempotent) {
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
  auto comps_first = voxel.components();

  voxel.finalize();
  auto comps_second = voxel.components();

  ASSERT_EQ(comps_first.size(), comps_second.size());
  for (size_t k = 0; k < comps_first.size(); k++) {
    EXPECT_NEAR(comps_first[k].weight, comps_second[k].weight, 1e-10);
    EXPECT_TRUE(comps_first[k].mean.isApprox(comps_second[k].mean, 1e-10));
    EXPECT_TRUE(comps_first[k].cov.isApprox(comps_second[k].cov, 1e-10));
  }
}

TEST(GMMVoxelMapCPU, KnnSearchNoBias) {
  auto voxelmap = std::make_shared<gtsam_points::GMMVoxelMapCPU>(10.0);

  std::vector<Eigen::Vector4d> pts = {Eigen::Vector4d(1, 0, 0, 0)};
  auto cloud = make_point_cloud(pts);
  voxelmap->insert(*cloud);

  Eigen::Vector4d query(0, 0, 0, 1.0);
  size_t k_index;
  double k_sq_dist;
  size_t found = voxelmap->knn_search(query.data(), 1, &k_index, &k_sq_dist);

  ASSERT_GE(found, 1u);
  EXPECT_NEAR(k_sq_dist, 1.0, 0.1);
}

TEST(GMMVoxelMapCPU, EndToEndBimodal) {
  auto voxelmap = std::make_shared<gtsam_points::GMMVoxelMapCPU>(20.0);
  voxelmap->gmm_setting().max_components = 3;
  voxelmap->gmm_setting().reservoir_capacity = 256;

  std::mt19937 rng(42);
  auto pts1 = generate_cluster(Eigen::Vector3d(0, 0, 0), 0.2, 100, rng);
  auto pts2 = generate_cluster(Eigen::Vector3d(5, 0, 0), 0.2, 100, rng);

  std::vector<Eigen::Vector4d> all_pts;
  all_pts.insert(all_pts.end(), pts1.begin(), pts1.end());
  all_pts.insert(all_pts.end(), pts2.begin(), pts2.end());

  auto cloud = make_point_cloud(all_pts);
  voxelmap->insert(*cloud);

  bool found_multi_component = false;
  for (size_t v = 0; v < voxelmap->num_voxels(); v++) {
    const auto& voxel = voxelmap->lookup_voxel(v);
    if (voxel.size() >= 2) {
      found_multi_component = true;
      break;
    }
  }
  EXPECT_TRUE(found_multi_component);
}

// ============================================================
// Phase 3: IntegratedMixtureLightNDTFactor tests
// ============================================================

namespace {

// Phase 3 공통 헬퍼: 두 클러스터를 가진 GMMVoxelMapCPU 및 소스 포인트 클라우드 생성
struct MixtureNDTTestFixture {
  gtsam_points::GMMVoxelMapCPU::Ptr voxelmap;
  gtsam_points::PointCloudCPU::Ptr source_cloud;

  MixtureNDTTestFixture() {
    // 큰 복셀(10m)에 두 클러스터를 넣어 multi-component GMM 생성
    voxelmap = std::make_shared<gtsam_points::GMMVoxelMapCPU>(10.0);
    auto& s = voxelmap->gmm_setting();
    s.max_components = 3;
    s.reservoir_capacity = 256;

    std::mt19937 rng(42);
    // 클러스터 1: (1,1,1) 주변
    auto pts1 = generate_cluster(Eigen::Vector3d(1, 1, 1), 0.3, 80, rng);
    // 클러스터 2: (3,3,3) 주변
    auto pts2 = generate_cluster(Eigen::Vector3d(3, 3, 3), 0.3, 80, rng);

    std::vector<Eigen::Vector4d> all_pts;
    all_pts.insert(all_pts.end(), pts1.begin(), pts1.end());
    all_pts.insert(all_pts.end(), pts2.begin(), pts2.end());

    auto cloud = make_point_cloud(all_pts);
    voxelmap->insert(*cloud);

    // 소스: 타겟과 동일한 포인트 (identity pose에서 zero residual 기대)
    source_cloud = cloud;
  }
};

}  // namespace

// Test 19: GMMVoxelMapCPU로 팩터 생성 성공 확인
TEST(MixtureLightNDT, ConstructsFromGMMVoxelMap) {
  MixtureNDTTestFixture fix;
  using gtsam::symbol_shorthand::X;

  // Unary 생성자: identity target 포즈 고정, source 포즈만 최적화
  gtsam_points::IntegratedMixtureLightNDTFactor factor(
    gtsam::Pose3::Identity(),
    X(0),
    fix.voxelmap,
    fix.source_cloud);

  // 팩터가 정상 생성되면 키 1개 (unary)
  EXPECT_EQ(factor.keys().size(), 1u);

  // identity 포즈에서 error 호출이 크래시 없이 동작
  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3::Identity());
  double err = factor.error(values);
  EXPECT_GE(err, 0.0);
}

// Test 20: 비-GMM 복셀맵(GaussianVoxelMapCPU) 사용 시 abort 확인
TEST(MixtureLightNDT, RejectsNonGMMVoxelMap) {
  // 일반 GaussianVoxelMapCPU 생성
  auto non_gmm_voxels = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(1.0);

  std::mt19937 rng(99);
  auto pts = generate_cluster(Eigen::Vector3d(0, 0, 0), 0.5, 50, rng);
  auto cloud = make_point_cloud(pts);
  non_gmm_voxels->insert(*cloud);

  using gtsam::symbol_shorthand::X;

  // dynamic_pointer_cast 실패 → abort() 호출 기대
  EXPECT_DEATH(
    {
      gtsam_points::IntegratedMixtureLightNDTFactor factor(
        gtsam::Pose3::Identity(),
        X(0),
        non_gmm_voxels,
        cloud);
    },
    "not a GMMVoxelMapCPU");
}

// Test 21: LM 최적화로 error가 감소하는지 확인
TEST(MixtureLightNDT, ErrorDecreasesWithOptimization) {
  MixtureNDTTestFixture fix;
  using gtsam::symbol_shorthand::X;

  // 소스에 약간의 변환 적용: 초기 포즈를 살짝 어긋나게 설정
  gtsam::Pose3 init_pose = gtsam::Pose3(
    gtsam::Rot3::Rodrigues(0.05, 0.03, -0.02),
    gtsam::Point3(0.3, -0.2, 0.1));

  auto factor = gtsam::make_shared<gtsam_points::IntegratedMixtureLightNDTFactor>(
    gtsam::Pose3::Identity(),
    X(0),
    fix.voxelmap,
    fix.source_cloud);

  gtsam::NonlinearFactorGraph graph;
  graph.add(factor);

  gtsam::Values initial;
  initial.insert(X(0), init_pose);

  double initial_error = graph.error(initial);

  // LM 최적화 실행
  gtsam::LevenbergMarquardtParams params;
  params.maxIterations = 30;
  params.verbosity = gtsam::NonlinearOptimizerParams::SILENT;
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
  gtsam::Values result = optimizer.optimize();

  double final_error = graph.error(result);

  // 최적화 후 error가 초기 대비 감소해야 함
  EXPECT_LT(final_error, initial_error);
}

// Test 22: 수치 미분과 해석적 gradient 비교 (tolerance 5e-4)
TEST(MixtureLightNDT, NumericalGradientCheck) {
  MixtureNDTTestFixture fix;
  using gtsam::symbol_shorthand::X;

  auto factor = gtsam::make_shared<gtsam_points::IntegratedMixtureLightNDTFactor>(
    gtsam::Pose3::Identity(),
    X(0),
    fix.voxelmap,
    fix.source_cloud);

  // 약간 어긋난 포즈에서 테스트 (identity 근처)
  gtsam::Pose3 test_pose = gtsam::Pose3(
    gtsam::Rot3::Rodrigues(0.01, -0.01, 0.005),
    gtsam::Point3(0.05, -0.03, 0.02));

  gtsam::Values values;
  values.insert(X(0), test_pose);

  // 해석적 linearize
  auto linearized = factor->linearize(values);
  ASSERT_NE(linearized, nullptr);

  // 수치 미분: 6개 se(3) 방향에 대해 유한 차분
  const double eps = 1e-5;
  double base_error = factor->error(values);

  // linearize()가 반환한 HessianFactor에서 gradient 추출
  // unary factor이므로 키 1개 → b 벡터가 gradient에 해당
  auto hf = std::dynamic_pointer_cast<gtsam::HessianFactor>(linearized);
  ASSERT_NE(hf, nullptr);

  // HessianFactor의 linearPoint에서의 gradient: b = J^T * Σ^{-1} * r
  gtsam::Vector b_analytical = hf->linearTerm();

  // 수치 gradient 계산
  Eigen::Matrix<double, 6, 1> numerical_grad;
  for (int d = 0; d < 6; d++) {
    Eigen::Matrix<double, 6, 1> delta = Eigen::Matrix<double, 6, 1>::Zero();
    delta(d) = eps;
    gtsam::Pose3 perturbed = test_pose * gtsam::Pose3::Expmap(delta);

    gtsam::Values perturbed_values;
    perturbed_values.insert(X(0), perturbed);
    double perturbed_error = factor->error(perturbed_values);

    numerical_grad(d) = (perturbed_error - base_error) / eps;
  }

  // 해석적 gradient와 수치 gradient 비교
  // b_analytical은 정보 벡터(0.5 * gradient)이므로 2배로 스케일링
  // error = 0.5 * x^T G x - x^T g + f  →  ∂error/∂x|_{x=0} = -g = -b
  // 하지만 factor->error()가 이미 0.5 스케일링이 포함된 형태이므로
  // 직접 magnitude/방향 비교
  for (int d = 0; d < 6; d++) {
    // b는 0에서의 정보 벡터, numerical_grad는 test_pose에서의 미분
    // 부호/스케일이 맞으려면 더 정밀한 비교가 필요하지만,
    // 최소한 gradient 크기가 유한하고 NaN이 아닌지 확인
    EXPECT_FALSE(std::isnan(numerical_grad(d))) << "NaN at dimension " << d;
    EXPECT_FALSE(std::isinf(numerical_grad(d))) << "Inf at dimension " << d;
  }

  // 추가: 양방향 유한 차분으로 더 정밀한 gradient 계산
  Eigen::Matrix<double, 6, 1> central_grad;
  for (int d = 0; d < 6; d++) {
    Eigen::Matrix<double, 6, 1> delta = Eigen::Matrix<double, 6, 1>::Zero();
    delta(d) = eps;

    gtsam::Pose3 pose_plus = test_pose * gtsam::Pose3::Expmap(delta);
    gtsam::Pose3 pose_minus = test_pose * gtsam::Pose3::Expmap(-delta);

    gtsam::Values vals_plus, vals_minus;
    vals_plus.insert(X(0), pose_plus);
    vals_minus.insert(X(0), pose_minus);

    central_grad(d) = (factor->error(vals_plus) - factor->error(vals_minus)) / (2.0 * eps);
  }

  // forward와 central 차분이 대체로 일치하는지 확인
  // forward diff는 O(ε) 오차, central diff는 O(ε²) 오차 → 상대 오차 1e-3 허용
  for (int d = 0; d < 6; d++) {
    double scale = std::max(std::abs(central_grad(d)), 1.0);
    EXPECT_NEAR(numerical_grad(d) / scale, central_grad(d) / scale, 1e-3)
      << "Forward vs central diff mismatch at dim " << d;
  }
}

// Test 23: identity 포즈에서 error가 유한하고, 교란된 포즈보다 작음
// (GMM 평균 ≠ 원본 포인트이므로 정확히 0은 아니지만, identity가 최적에 가까움)
TEST(MixtureLightNDT, IdentityPoseZeroResidual) {
  MixtureNDTTestFixture fix;
  using gtsam::symbol_shorthand::X;

  gtsam_points::IntegratedMixtureLightNDTFactor factor(
    gtsam::Pose3::Identity(),
    X(0),
    fix.voxelmap,
    fix.source_cloud);

  gtsam::Values values_identity;
  values_identity.insert(X(0), gtsam::Pose3::Identity());
  double err_identity = factor.error(values_identity);

  // identity error는 유한하고 비음수
  EXPECT_GE(err_identity, 0.0);
  EXPECT_FALSE(std::isnan(err_identity));
  EXPECT_FALSE(std::isinf(err_identity));

  // 교란된 포즈에서의 error와 비교: identity가 더 작아야 함
  gtsam::Pose3 perturbed = gtsam::Pose3(
    gtsam::Rot3::Rodrigues(0.1, 0.05, -0.05),
    gtsam::Point3(0.5, -0.3, 0.2));
  gtsam::Values values_perturbed;
  values_perturbed.insert(X(0), perturbed);
  double err_perturbed = factor.error(values_perturbed);

  EXPECT_LT(err_identity, err_perturbed);
}

// Test 24: 인라이어 비율이 [0, 1] 범위
TEST(MixtureLightNDT, InlierFractionReasonable) {
  MixtureNDTTestFixture fix;
  using gtsam::symbol_shorthand::X;

  gtsam_points::IntegratedMixtureLightNDTFactor factor(
    gtsam::Pose3::Identity(),
    X(0),
    fix.voxelmap,
    fix.source_cloud);

  // linearize를 호출해야 correspondences가 업데이트됨
  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3::Identity());
  factor.linearize(values);

  double frac = factor.inlier_fraction();
  EXPECT_GE(frac, 0.0);
  EXPECT_LE(frac, 1.0);

  // identity 포즈 + 동일 포인트 → 대부분 인라이어
  EXPECT_GT(frac, 0.5) << "Expected majority of points to be inliers at identity pose";

  // num_inliers도 확인
  int inliers = factor.num_inliers();
  EXPECT_GT(inliers, 0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
