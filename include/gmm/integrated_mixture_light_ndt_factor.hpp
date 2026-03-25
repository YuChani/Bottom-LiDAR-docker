// SPDX-License-Identifier: MIT
// Copyright (c) 2026
//
// Phase 3: IntegratedMixtureLightNDTFactor — GMM 기반 NDT 정합 비용 팩터.
// LightNDT의 구조를 미러링하되, 복셀당 K개 GMM 컴포넌트를 순회하여
// 최소 마할라노비스 거리 컴포넌트(winner-take-all)를 대응으로 선택.
//
// 비용 함수: C(T) = Σ_i π_{k*(i)} · r_i^T Σ_{k*(i)}^{-1} r_i
//   r_i = μ_{k*} - T·p_i  (target - transformed source, GTSAM 컨벤션)
//   k*(i) = argmin_k (T·p_i - μ_k)^T Σ_k^{-1} (T·p_i - μ_k)

#pragma once

#include <memory>
#include <vector>
#include <Eigen/Core>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <gtsam_points/util/gtsam_migration.hpp>
#include <gtsam_points/types/point_cloud.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/factors/integrated_matching_cost_factor.hpp>
#include "ndt/integrated_ndt_factor.hpp"       // NDTSearchMode, compute_ndt_inverse_covariance
#include "gmm/gmm_voxelmap_cpu.hpp"            // GMMVoxelMapCPU, GMMVoxel, GMMComponent

namespace gtsam_points {

/// @brief GMM-NDT 대응 데이터: winner-take-all로 선택된 단일 컴포넌트 정보.
// LightNDT의 NdtCorrespondence와 유사하나, 혼합 가중치 π_k를 추가로 보유.
struct MixtureNdtCorrespondence
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector4d mean = Eigen::Vector4d::Zero();     // 대응 GMM 컴포넌트 평균 μ_{k*}
  Eigen::Matrix4d inv_cov = Eigen::Matrix4d::Zero();  // 정규화된 역공분산 Σ_{k*}^{-1} (4x4, 상단 3x3만 유효)
  double weight = 0.0;                                // 혼합 가중치 π_{k*} ∈ [0,1]
  bool valid = false;                                 // 유효한 대응 여부 (복셀 미발견 시 false)
  MixtureNdtCorrespondence() = default;
};

/// @brief GMM 기반 LightNDT 정합 팩터.
///        복셀당 K개 GMM 컴포넌트 중 최소 마할라노비스 거리 컴포넌트를
///        winner-take-all로 선택하고, π_k 가중 Hessian을 누적하는 변형 NDT.
// IntegratedLightNDTFactor_를 미러링: 동일한 IntegratedMatchingCostFactor 상속,
// 동일한 update_correspondences/evaluate 가상 함수 오버라이드 패턴.
template <typename SourceFrame = gtsam_points::PointCloud>
class IntegratedMixtureLightNDTFactor_ : public gtsam_points::IntegratedMatchingCostFactor
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = gtsam_points::shared_ptr<IntegratedMixtureLightNDTFactor_>;

  // Binary 생성자: target_key + source_key (두 포즈 모두 최적화)
  IntegratedMixtureLightNDTFactor_(
    gtsam::Key target_key,
    gtsam::Key source_key,
    const GaussianVoxelMap::ConstPtr& target_voxels,
    const std::shared_ptr<const SourceFrame>& source);

  // Unary 생성자: 고정된 target 포즈 + source_key (source 포즈만 최적화)
  IntegratedMixtureLightNDTFactor_(
    const gtsam::Pose3& fixed_target_pose,
    gtsam::Key source_key,
    const GaussianVoxelMap::ConstPtr& target_voxels,
    const std::shared_ptr<const SourceFrame>& source);

  virtual ~IntegratedMixtureLightNDTFactor_() override;

  virtual void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override;
  virtual size_t memory_usage() const override;

  // 병렬 스레드 수 설정 (OMP 백엔드)
  void set_num_threads(int n)
  {
    num_threads = n;
  }

  // 공분산 정칙화 ε 설정: 작은 고유값을 ε·λ_max로 클램프.
  // 변경 시 inv_cov_cache 무효화 (재계산 필요)
  void set_regularization_epsilon(double eps)
  {
    regularization_epsilon = eps;
    inv_cov_cached = false;  // 역공분산 캐시 무효화
  }

  // 대응 업데이트 허용 오차: 변위가 임계치 미만이면 대응 재탐색 생략
  void set_correspondence_update_tolerance(double angle, double trans)
  {
    correspondence_update_tolerance_rot = angle;
    correspondence_update_tolerance_trans = trans;
  }

  // 이웃 복셀 탐색 모드: DIRECT1(현재만), DIRECT7(+6면), DIRECT27(+26이웃)
  void set_search_mode(NDTSearchMode mode)
  {
    search_mode = mode;
  }

  // 유효 대응이 있는 소스 포인트 수 (linearize() 이후 호출)
  int num_inliers() const
  {
    int count = 0;
    for (const auto& c : correspondences) {
      if (c.valid) count++;
    }
    return count;
  }

  // 인라이어 비율: num_inliers / |source|, [0, 1] 범위
  double inlier_fraction() const
  {
    return correspondences.empty() ? 0.0 : num_inliers() / static_cast<double>(correspondences.size());
  }

  // GMMVoxelMapCPU 타겟 맵 반환
  const std::shared_ptr<const GMMVoxelMapCPU>& get_target() const
  {
    return target_voxels;
  }

  // 팩터 딥 카피 (gtsam 그래프 복제용)
  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return gtsam::NonlinearFactor::shared_ptr(new IntegratedMixtureLightNDTFactor_(*this));
  }

private:
  // 소스 포인트별 최적 GMM 컴포넌트 대응 탐색
  virtual void update_correspondences(const Eigen::Isometry3d& delta) const override;

  // 비용 함수 및 Jacobian/Hessian 계산
  virtual double evaluate(
    const Eigen::Isometry3d& delta,
    Eigen::Matrix<double, 6, 6>* H_target = nullptr,
    Eigen::Matrix<double, 6, 6>* H_source = nullptr,
    Eigen::Matrix<double, 6, 6>* H_target_source = nullptr,
    Eigen::Matrix<double, 6, 1>* b_target = nullptr,
    Eigen::Matrix<double, 6, 1>* b_source = nullptr) const override;

private:
  int num_threads;                       // OMP 스레드 수
  double regularization_epsilon;         // 역공분산 정칙화 파라미터 ε
  NDTSearchMode search_mode;             // 이웃 복셀 탐색 모드

  double correspondence_update_tolerance_rot;    // 회전 변위 임계치 (rad)
  double correspondence_update_tolerance_trans;  // 병진 변위 임계치 (m)

  mutable Eigen::Isometry3d linearization_point;        // 현재 선형화 포즈
  mutable Eigen::Isometry3d last_correspondence_point;  // 마지막 대응 탐색 포즈
  mutable std::vector<MixtureNdtCorrespondence> correspondences;  // 소스 포인트별 대응

  // LightNDT와 달리 복셀당 K개 컴포넌트 역공분산을 2차원 벡터로 캐시.
  // inv_cov_cache[voxel_id][k] = Σ_k^{-1} (정규화된 역공분산)
  mutable std::vector<std::vector<Eigen::Matrix4d>> inv_cov_cache;
  mutable bool inv_cov_cached;  // 캐시 유효 여부. set_regularization_epsilon() 호출 시 false로 재설정

  std::shared_ptr<const GMMVoxelMapCPU> target_voxels;   // GMM 복셀맵 타겟
  std::shared_ptr<const SourceFrame> source;             // 소스 포인트 클라우드
};

// 기본 SourceFrame = PointCloud로 특수화된 alias
using IntegratedMixtureLightNDTFactor = IntegratedMixtureLightNDTFactor_<>;

}  // namespace gtsam_points
