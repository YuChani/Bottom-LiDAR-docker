// SPDX-License-Identifier: MIT
#pragma once

#include <vector>
#include <Eigen/Core>
#include "gmm/gmm_voxelmap_cpu.hpp"

namespace gtsam_points {

// GMM 피팅 결과 구조체: EM 알고리즘 실행 후 반환되는 컴포넌트 목록 및 수렴 정보.
struct GMMFitResult
{
  std::vector<GMMComponent> components;  // 피팅된 K개 컴포넌트 (pruning 후)
  bool converged = false;                // EM이 convergence_tol 이내로 수렴했는지 여부
  int iterations_run = 0;                // 실제 수행된 EM 반복 횟수
};

// GMM 피팅 하이퍼파라미터: GMMVoxel::Setting에서 복사하여 fit_gmm()에 전달.
struct GMMFitParams
{
  int max_components = 3;                  // 최대 가우시안 컴포넌트 수 K
  int max_em_iterations = 20;              // EM 최대 반복 횟수
  double convergence_tol = 1e-4;           // 로그-우도 변화량 수렴 임계치
  double covariance_regularization = 1e-3; // 공분산 정칙화 ε: Σ_k += ε·I₃
  double min_weight_threshold = 0.01;      // π_k < threshold인 컴포넌트 제거
};

// Cold-start GMM 피팅: random_spread 초기화 → EM.
// 이전 컴포넌트 없이 데이터로부터 처음부터 학습.
GMMFitResult fit_gmm(
    const std::vector<Eigen::Vector4d>& points,
    const GMMFitParams& params);

// Warm-start GMM 피팅: 기존 컴포넌트를 초기값으로 사용 → keep_existing EM.
// 이전 finalize() 결과를 시드로 사용하여 수렴 속도 향상.
GMMFitResult fit_gmm(
    const std::vector<Eigen::Vector4d>& points,
    const GMMFitParams& params,
    const std::vector<GMMComponent>& initial_components);

}  // namespace gtsam_points
