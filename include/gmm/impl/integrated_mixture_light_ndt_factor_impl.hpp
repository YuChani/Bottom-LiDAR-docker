// SPDX-License-Identifier: MIT
// Copyright (c) 2026
//
// Phase 3 구현: IntegratedMixtureLightNDTFactor_ 템플릿 메서드 정의.
// LightNDT impl을 기반으로 GMM 컴포넌트 순회 및 π_k 가중 비용 함수 구현.

#include "gmm/integrated_mixture_light_ndt_factor.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam_points/config.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/util/parallelism.hpp>
#include <gtsam_points/factors/impl/scan_matching_reduction.hpp>

#ifdef GTSAM_POINTS_USE_TBB
#include <tbb/parallel_for.h>
#endif

namespace gtsam_points {

// ============================================================
// Binary 생성자: 두 포즈(target, source) 모두 최적화 대상
// ============================================================
template <typename SourceFrame>
IntegratedMixtureLightNDTFactor_<SourceFrame>::IntegratedMixtureLightNDTFactor_(
  gtsam::Key target_key,
  gtsam::Key source_key,
  const GaussianVoxelMap::ConstPtr& target_voxels,
  const std::shared_ptr<const SourceFrame>& source)
: gtsam_points::IntegratedMatchingCostFactor(target_key, source_key),
  num_threads(1),
  regularization_epsilon(1e-3),
  search_mode(NDTSearchMode::DIRECT7),  // 기본값: 현재 복셀 + 6면 이웃
  correspondence_update_tolerance_rot(0.0),
  correspondence_update_tolerance_trans(0.0),
  inv_cov_cached(false),
  // dynamic_pointer_cast: GaussianVoxelMap → GMMVoxelMapCPU 다운캐스트
  // 실패 시 nullptr → 아래에서 abort()
  target_voxels(std::dynamic_pointer_cast<const GMMVoxelMapCPU>(target_voxels)),
  source(source)
{
  // 소스 포인트 유효성 검사
  if (!frame::has_points(*source)) {
    std::cerr << "error: source points have not been allocated!!" << std::endl;
    abort();
  }
  // GMMVoxelMapCPU로 다운캐스트 실패 = 비-GMM 복셀맵 사용 시도
  if (!this->target_voxels) {
    std::cerr << "error: target voxelmap is not a GMMVoxelMapCPU!!" << std::endl;
    abort();
  }
}

// ============================================================
// Unary 생성자: target 포즈 고정, source 포즈만 최적화
// ============================================================
template <typename SourceFrame>
IntegratedMixtureLightNDTFactor_<SourceFrame>::IntegratedMixtureLightNDTFactor_(
  const gtsam::Pose3& fixed_target_pose,
  gtsam::Key source_key,
  const GaussianVoxelMap::ConstPtr& target_voxels,
  const std::shared_ptr<const SourceFrame>& source)
: gtsam_points::IntegratedMatchingCostFactor(fixed_target_pose, source_key),
  num_threads(1),
  regularization_epsilon(1e-3),
  search_mode(NDTSearchMode::DIRECT7),
  correspondence_update_tolerance_rot(0.0),
  correspondence_update_tolerance_trans(0.0),
  inv_cov_cached(false),
  target_voxels(std::dynamic_pointer_cast<const GMMVoxelMapCPU>(target_voxels)),
  source(source)
{
  if (!frame::has_points(*source)) {
    std::cerr << "error: source points have not been allocated!!" << std::endl;
    abort();
  }
  if (!this->target_voxels) {
    std::cerr << "error: target voxelmap is not a GMMVoxelMapCPU!!" << std::endl;
    abort();
  }
}

template <typename SourceFrame>
IntegratedMixtureLightNDTFactor_<SourceFrame>::~IntegratedMixtureLightNDTFactor_()
{
}

// ============================================================
// print: 팩터 정보 출력 (디버깅용)
// ============================================================
template <typename SourceFrame>
void IntegratedMixtureLightNDTFactor_<SourceFrame>::print(const std::string& s, const gtsam::KeyFormatter& keyFormatter) const
{
  std::cout << s << "IntegratedMixtureLightNDTFactor";
  if (is_binary) {
    std::cout << "(" << keyFormatter(this->keys()[0]) << ", " << keyFormatter(this->keys()[1]) << ")" << std::endl;
  } else {
    std::cout << "(fixed, " << keyFormatter(this->keys()[0]) << ")" << std::endl;
  }
  std::cout << "target_resolution=" << target_voxels->voxel_resolution()
            << ", |source|=" << frame::size(*source) << "pts" << std::endl;
}

// ============================================================
// memory_usage: 메모리 사용량 근사 추정
// ============================================================
template <typename SourceFrame>
size_t IntegratedMixtureLightNDTFactor_<SourceFrame>::memory_usage() const
{
  return sizeof(*this) + sizeof(MixtureNdtCorrespondence) * correspondences.capacity();
}

// ============================================================
// update_correspondences: 소스 포인트별 최적 GMM 컴포넌트 대응 탐색
// ============================================================
// 알고리즘:
//   1. 포즈 변위 허용 오차 검사 (불필요한 재탐색 방지)
//   2. 이웃 오프셋 목록 구성 (DIRECT1/7/27)
//   3. inv_cov_cache 구축 (최초 1회): 모든 복셀의 모든 컴포넌트에 대해
//      compute_ndt_inverse_covariance() 호출
//   4. 소스 포인트별:
//      q_i = delta * source_point[i]  (변환된 소스 포인트)
//      이웃 복셀 탐색 → 각 복셀의 각 GMM 컴포넌트에 대해 마할라노비스 거리 계산
//      → 최소 거리 컴포넌트를 winner-take-all로 선택
template <typename SourceFrame>
void IntegratedMixtureLightNDTFactor_<SourceFrame>::update_correspondences(const Eigen::Isometry3d& delta) const
{
  linearization_point = delta;

  // 포즈 변위 허용 오차 검사: 직전 탐색 포즈로부터의 변위가 임계치 미만이면 스킵
  bool do_update = true;
  if (correspondences.size() == static_cast<size_t>(frame::size(*source)) &&
      (correspondence_update_tolerance_trans > 0.0 || correspondence_update_tolerance_rot > 0.0)) {
    Eigen::Isometry3d diff = delta.inverse() * last_correspondence_point;
    double diff_rot = Eigen::AngleAxisd(diff.linear()).angle();   // 회전 변위 (rad)
    double diff_trans = diff.translation().norm();                // 병진 변위 (m)
    if (diff_rot < correspondence_update_tolerance_rot && diff_trans < correspondence_update_tolerance_trans) {
      do_update = false;  // 변위 작음 → 기존 대응 재사용
    }
  }

  if (do_update) {
    last_correspondence_point = delta;  // 탐색 포즈 갱신
  }

  correspondences.resize(frame::size(*source));

  // 이웃 복셀 오프셋 구성
  std::vector<Eigen::Vector3i> neighbor_offsets;
  switch (search_mode) {
    case NDTSearchMode::DIRECT1:
      // 현재 복셀만 (가장 빠름)
      neighbor_offsets.push_back(Eigen::Vector3i(0, 0, 0));
      break;
    case NDTSearchMode::DIRECT7:
      // 현재 + 6면 이웃 (균형 잡힌 기본값)
      neighbor_offsets.push_back(Eigen::Vector3i(0, 0, 0));
      neighbor_offsets.push_back(Eigen::Vector3i(1, 0, 0));
      neighbor_offsets.push_back(Eigen::Vector3i(-1, 0, 0));
      neighbor_offsets.push_back(Eigen::Vector3i(0, 1, 0));
      neighbor_offsets.push_back(Eigen::Vector3i(0, -1, 0));
      neighbor_offsets.push_back(Eigen::Vector3i(0, 0, 1));
      neighbor_offsets.push_back(Eigen::Vector3i(0, 0, -1));
      break;
    case NDTSearchMode::DIRECT27:
      // 3×3×3 이웃 전체 (가장 정확, 가장 느림)
      for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
          for (int dz = -1; dz <= 1; dz++) {
            neighbor_offsets.push_back(Eigen::Vector3i(dx, dy, dz));
          }
        }
      }
      break;
  }

  // ============================================================
  // inv_cov_cache 구축 (최초 1회 또는 ε 변경 후)
  // ============================================================
  // LightNDT: inv_cov_cache[voxel_id] = Σ^{-1} (복셀당 단일)
  // GMM 변형: inv_cov_cache[voxel_id][k] = Σ_k^{-1} (복셀당 K개)
  if (!inv_cov_cached) {
    const size_t num_voxels = target_voxels->num_voxels();
    inv_cov_cache.resize(num_voxels);
    for (size_t v = 0; v < num_voxels; v++) {
      const auto& voxel = target_voxels->lookup_voxel(v);
      const auto& comps = voxel.components();
      inv_cov_cache[v].resize(comps.size());
      for (size_t k = 0; k < comps.size(); k++) {
        // SelfAdjointEigenSolver로 고유값 분해 → 작은 고유값 클램프 → 역행렬
        // λ_min = max(λ_i, ε·λ_max)로 정칙화하여 특이행렬 방지
        inv_cov_cache[v][k] = compute_ndt_inverse_covariance(comps[k].cov, regularization_epsilon);
      }
    }
    inv_cov_cached = true;
  }

  // ============================================================
  // 소스 포인트별 대응 탐색 (병렬)
  // ============================================================
  const auto perpoint_task = [&](int i)
  {
    if (do_update) {
      correspondences[i].valid = false;

      // q_i = delta * source_point[i]: 현재 추정 포즈로 변환한 소스 포인트
      Eigen::Vector4d pt = delta * frame::point(*source, i);
      // 변환된 포인트의 복셀 좌표 계산
      Eigen::Vector3i coord = target_voxels->voxel_coord(pt);

      double min_mahalanobis = std::numeric_limits<double>::max();
      MixtureNdtCorrespondence best_corr;

      // 이웃 복셀 순회
      for (const auto& offset : neighbor_offsets) {
        Eigen::Vector3i neighbor_coord = coord + offset;
        const auto voxel_id = target_voxels->lookup_voxel_index(neighbor_coord);
        if (voxel_id < 0) {
          continue;  // 해당 좌표에 복셀 없음
        }

        const auto& voxel = target_voxels->lookup_voxel(voxel_id);
        const auto& comps = voxel.components();

        // 해당 복셀의 모든 GMM 컴포넌트 순회 (winner-take-all)
        for (size_t k = 0; k < comps.size(); k++) {
          if (inv_cov_cache[voxel_id].empty()) {
            continue;
          }

          // Σ_k^{-1} 캐시에서 가져옴
          const Eigen::Matrix4d& inv_cov = inv_cov_cache[voxel_id][k];

          // 4D 차이벡터: diff = q_i - μ_k
          Eigen::Vector4d diff = pt - comps[k].mean;
          diff(3) = 0.0;  // w-component 차이 제거 (mean w=0, pt w=1 → 편향 방지)
          // 마할라노비스 거리: m = diff^T · Σ_k^{-1} · diff
          double mahalanobis_dist = diff.transpose() * inv_cov * diff;

          // 최소 마할라노비스 거리 컴포넌트 선택 (winner-take-all)
          if (mahalanobis_dist < min_mahalanobis) {
            min_mahalanobis = mahalanobis_dist;
            best_corr.mean = comps[k].mean;          // μ_{k*}
            best_corr.inv_cov = inv_cov;             // Σ_{k*}^{-1}
            best_corr.weight = comps[k].weight;      // π_{k*}
            best_corr.valid = true;
          }
        }
      }

      correspondences[i] = best_corr;
    }
  };

  // OMP 또는 TBB 병렬 실행
  if (is_omp_default() || num_threads == 1) {
#pragma omp parallel for num_threads(num_threads) schedule(guided, 8)
    for (int i = 0; i < frame::size(*source); i++) {
      perpoint_task(i);
    }
  } else {
#ifdef GTSAM_POINTS_USE_TBB
    tbb::parallel_for(tbb::blocked_range<int>(0, frame::size(*source), 8), [&](const tbb::blocked_range<int>& range)
    {
      for (int i = range.begin(); i < range.end(); i++) {
        perpoint_task(i);
      }
    });
#else
    std::cerr << "error: TBB is not available" << std::endl;
    abort();
#endif
  }
}

// ============================================================
// evaluate: GMM-NDT 비용 함수 및 Jacobian/Hessian 계산
// ============================================================
// 비용 함수: C(T) = Σ_i π_{k*(i)} · r_i^T Σ_{k*(i)}^{-1} r_i
//
// Jacobian (4×6, SE(3) 리 대수 매개변수화 ξ = (φ, ρ)):
//   J_target = [−[T·p]×, I]  (target 포즈에 대한 미분)
//   J_source = [R·[p]×, −R]  (source 포즈에 대한 미분)
//
// π_k 가중 Hessian 누적:
//   H += π_k · J^T · Σ^{-1} · J
//   b += π_k · J^T · Σ^{-1} · r
template <typename SourceFrame>
double IntegratedMixtureLightNDTFactor_<SourceFrame>::evaluate(
  const Eigen::Isometry3d& delta,
  Eigen::Matrix<double, 6, 6>* H_target,
  Eigen::Matrix<double, 6, 6>* H_source,
  Eigen::Matrix<double, 6, 6>* H_target_source,
  Eigen::Matrix<double, 6, 1>* b_target,
  Eigen::Matrix<double, 6, 1>* b_source) const
{

  // 대응이 아직 계산되지 않았으면 업데이트
  if (correspondences.size() != static_cast<size_t>(frame::size(*source))) {
    update_correspondences(delta);
  }

  // 소스 포인트별 비용/Jacobian 계산 람다
  const auto perpoint_task = [&](int i,
                                 Eigen::Matrix<double, 6, 6>* H_target,
                                 Eigen::Matrix<double, 6, 6>* H_source,
                                 Eigen::Matrix<double, 6, 6>* H_target_source,
                                 Eigen::Matrix<double, 6, 1>* b_target,
                                 Eigen::Matrix<double, 6, 1>* b_source)
  {
    const auto& corr = correspondences[i];
    if (!corr.valid) {
      return 0.0;  // 대응 없는 포인트는 비용 기여 없음
    }

    // mean_A: 소스 포인트 (원래 좌표계)
    const auto& mean_A = frame::point(*source, i);
    // mean_B: 대응 GMM 컴포넌트 평균 (타겟 좌표계)
    const auto& mean_B = corr.mean;
    // inv_cov_B: 대응 컴포넌트의 정규화된 역공분산
    const auto& inv_cov_B = corr.inv_cov;
    // π_k: 대응 컴포넌트의 혼합 가중치
    const double pi_k = corr.weight;

    // 변환된 소스 포인트: T·p_i
    Eigen::Vector4d transed_mean_A = delta * mean_A;
    // 잔차: r_i = μ_{k*} - T·p_i (GTSAM 컨벤션: target - transformed source)
    Eigen::Vector4d residual = mean_B - transed_mean_A;
    residual(3) = 0.0;  // w-component 차이 제거 (mean w=0, pt w=1 → 편향 방지)

    // π_k 가중 비용: C_i = π_{k*} · r_i^T · Σ_{k*}^{-1} · r_i
    // .value()로 1×1 Eigen 행렬 → double 스칼라 추출
    const double cost = pi_k * (residual.transpose() * inv_cov_B * residual).value();

    if (!H_target) {
      return cost;  // error-only 경로 (Hessian 불필요)
    }

    // ============================================================
    // SE(3) Jacobian 계산
    // ============================================================
    // ξ = (φ, ρ) ∈ se(3): φ = 회전(so(3)), ρ = 병진(R³)
    //
    // Target Jacobian: ∂r/∂ξ_target
    //   J_target[0:3, 0:3] = −[T·p_i]× (skew-symmetric hat of transformed point)
    //   J_target[0:3, 3:6] = I₃ (병진 기여)
    //   J_target[3, :] = 0 (동차 좌표 w 성분 불변)
    Eigen::Matrix<double, 4, 6> J_target = Eigen::Matrix<double, 4, 6>::Zero();
    J_target.block<3, 3>(0, 0) = -gtsam::SO3::Hat(transed_mean_A.head<3>());
    J_target.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    // Source Jacobian: ∂r/∂ξ_source
    //   J_source[0:3, 0:3] = R · [p_i]× (회전에 의한 소스 포인트 변화)
    //   J_source[0:3, 3:6] = −R (병진에 의한 소스 포인트 변화, 음부호: target - source 컨벤션)
    Eigen::Matrix<double, 4, 6> J_source = Eigen::Matrix<double, 4, 6>::Zero();
    J_source.block<3, 3>(0, 0) = delta.linear() * gtsam::SO3::Hat(mean_A.template head<3>());
    J_source.block<3, 3>(0, 3) = -delta.linear();

    // π_k 가중 Hessian 누적: H += π_k · J^T · Σ^{-1} · J
    // 이를 분해하면: J_weighted = π_k · J^T · Σ^{-1}, H += J_weighted · J
    Eigen::Matrix<double, 6, 4> J_target_weighted = pi_k * J_target.transpose() * inv_cov_B;
    Eigen::Matrix<double, 6, 4> J_source_weighted = pi_k * J_source.transpose() * inv_cov_B;

    // 6×6 Hessian 블록 누적 (Gauss-Newton 근사)
    *H_target += J_target_weighted * J_target;
    *H_source += J_source_weighted * J_source;
    *H_target_source += J_target_weighted * J_source;

    // 6×1 gradient 벡터 누적: b += π_k · J^T · Σ^{-1} · r
    *b_target += J_target_weighted * residual;
    *b_source += J_source_weighted * residual;

    return cost;
  };

  // OMP 또는 TBB 병렬 reduction으로 전체 비용/Hessian 합산
  if (is_omp_default() || num_threads == 1) {
    return scan_matching_reduce_omp(perpoint_task, frame::size(*source), num_threads, H_target, H_source, H_target_source, b_target, b_source);
  } else {
    return scan_matching_reduce_tbb(perpoint_task, frame::size(*source), H_target, H_source, H_target_source, b_target, b_source);
  }
}

}  // namespace gtsam_points
