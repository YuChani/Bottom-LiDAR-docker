// SPDX-License-Identifier: MIT
#include "gmm/mixture_em_backend.hpp"

#include <algorithm>
#include <numeric>
#include <armadillo>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace gtsam_points {

namespace {

// 단일 컴포넌트 폴백: 포인트 수 < 2 또는 EM 실패 시 호출.
// 전체 포인트의 표본 평균과 표본 공분산으로 단일 가우시안 생성.
GMMFitResult single_component_fallback(
    const std::vector<Eigen::Vector4d>& points,
  double regularization)
{
  const size_t N = points.size();

  // 표본 평균 계산: μ = (1/N) Σ p_i
  Eigen::Vector4d mean = Eigen::Vector4d::Zero();
  for (const auto& pt : points) {
    mean += pt;
  }
  mean /= static_cast<double>(N);

  // 표본 공분산 계산: Σ = (1/N) Σ (p_i - μ)(p_i - μ)^T
  // N>1일 때만 계산 (단일 포인트는 공분산 = 0)
  Eigen::Matrix4d cov = Eigen::Matrix4d::Zero();
  if (N > 1) {
    for (const auto& pt : points) {
      const Eigen::Vector4d diff = pt - mean;
      cov += diff * diff.transpose();  // 외적 누적
    }
    cov /= static_cast<double>(N);  // 표본 공분산 (N으로 나눔, N-1 아님)
  }

  // 정칙화: 3x3 블록에 ε·I 추가 → 양의 정부호 보장
  cov.block<3, 3>(0, 0) += regularization * Eigen::Matrix3d::Identity();
  // 4행/4열을 0으로 강제: 동차 좌표 차원은 공분산에 기여하지 않음
  cov.row(3).setZero();
  cov.col(3).setZero();

  GMMComponent comp;
  comp.mean = mean;
  comp.cov = cov;
  comp.weight = 1.0;  // 유일한 컴포넌트이므로 π = 1

  GMMFitResult result;
  result.components.push_back(comp);
  result.converged = true;
  result.iterations_run = 0;
  return result;
}

// Eigen 4D 동차 벡터 → Armadillo 3×N 행렬 변환.
// Armadillo gmm_full은 3D 데이터를 기대하므로 w-component 제거.
arma::mat eigen_to_arma(const std::vector<Eigen::Vector4d>& points)
{
  const size_t N = points.size();
  arma::mat data(3, N);  // 3행(xyz) × N열(포인트 수)
  for (size_t i = 0; i < N; i++) {
    data(0, i) = points[i](0);  // x
    data(1, i) = points[i](1);  // y
    data(2, i) = points[i](2);  // z
  }
  return data;
}

// Armadillo GMM 모델 → GMMFitResult 변환.
// 후처리: 공분산 정칙화, 저가중치 컴포넌트 pruning, 가중치 재정규화.
GMMFitResult extract_result(
    const arma::gmm_full& model,
    const GMMFitParams& params,
  bool converged)
{
  GMMFitResult result;
  result.converged = converged;
  result.iterations_run = params.max_em_iterations;

  const arma::uword K = model.n_gaus();  // EM이 피팅한 컴포넌트 수

  for (arma::uword k = 0; k < K; k++) {
    GMMComponent comp;

    // Armadillo 3D 평균 → Eigen 4D 동차 좌표 (w=0)
    const arma::vec& m = model.means.col(k);
    comp.mean = Eigen::Vector4d(m(0), m(1), m(2), 0.0);

    // Armadillo 3x3 공분산 → Eigen 4x4 (상위-좌측 3x3 복사)
    comp.cov = Eigen::Matrix4d::Zero();
    const arma::mat& fcov = model.fcovs.slice(k);
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        comp.cov(r, c) = fcov(r, c);
      }
    }
    // 사후(post-hoc) 정칙화: Σ_k += ε·I₃
    // Armadillo EM은 자체 정칙화가 약하므로 추가 보장
    comp.cov.block<3, 3>(0, 0) += params.covariance_regularization * Eigen::Matrix3d::Identity();

    // 혼합 가중치 π_k = model.hefts(k), Σ π_k = 1 (Armadillo 보장)
    comp.weight = model.hefts(k);
    result.components.push_back(comp);
  }

  // Pruning 전 가중 평균 계산: 모든 컴포넌트가 제거될 경우의 폴백용
  Eigen::Vector4d weighted_mean = Eigen::Vector4d::Zero();
  double pre_prune_total_weight = 0.0;
  for (const auto& c : result.components) {
    weighted_mean += c.weight * c.mean;  // Σ π_k · μ_k
    pre_prune_total_weight += c.weight;
  }
  if (pre_prune_total_weight > 0.0) {
    weighted_mean /= pre_prune_total_weight;  // 정규화된 가중 평균
  }

  // 저가중치 컴포넌트 제거: π_k < min_weight_threshold인 컴포넌트 삭제
  // remove-erase 이디엄 사용
  result.components.erase(
      std::remove_if(result.components.begin(), result.components.end(),
                     [&](const GMMComponent& c) { return c.weight < params.min_weight_threshold; }),
      result.components.end());

  // 전부 제거된 경우: 가중 평균으로 단일 컴포넌트 폴백 생성
  if (result.components.empty()) {
    GMMComponent fallback;
    fallback.mean = weighted_mean;
    fallback.weight = 1.0;
    fallback.cov.block<3, 3>(0, 0) =
        params.covariance_regularization * Eigen::Matrix3d::Identity();
    result.components.push_back(fallback);
    return result;
  }

  // 가중치 재정규화: pruning 후 Σ π_k = 1 보장
  double total_weight = 0.0;
  for (const auto& c : result.components) {
    total_weight += c.weight;
  }
  if (total_weight > 0.0) {
    for (auto& c : result.components) {
      c.weight /= total_weight;  // π_k' = π_k / Σ π_j
    }
  }

  return result;
}

}  // namespace

// Cold-start GMM 피팅: 초기 컴포넌트 없이 random_spread로 EM 시작.
// random_spread: K개 초기 평균을 데이터에서 유클리드 거리 기반으로 분산 선택.
GMMFitResult fit_gmm(
    const std::vector<Eigen::Vector4d>& points,
  const GMMFitParams& params)
{
  const size_t N = points.size();

  // 포인트 0개이면 빈 결과 반환 (NaN 방지 — empty vector를 single_component_fallback에 넘기면 mean /= 0)
  if (N == 0) {
    GMMFitResult result;
    result.converged = true;
    result.iterations_run = 0;
    return result;
  }

  // 포인트 2개 미만이면 EM 불가 → 단일 컴포넌트 폴백
  if (N < 2) {
    return single_component_fallback(points, params.covariance_regularization);
  }

  // 실제 컴포넌트 수 = min(max_components, N): 포인트보다 많은 컴포넌트는 의미 없음
  const int K = std::min(params.max_components, static_cast<int>(N));

  arma::mat data = eigen_to_arma(points);  // Eigen → Armadillo 변환

  arma::gmm_full model;
  // Armadillo EM 학습: eucl_dist(유클리드 거리), random_spread(초기화),
  // 20 k-means 반복 + max_em_iterations EM 반복, 임계치 1e-6
  bool ok = model.learn(
      data,
      static_cast<arma::uword>(K),
      arma::eucl_dist,       // 거리 메트릭: 유클리드
      arma::random_spread,   // 초기화: 데이터에서 분산 선택
      20,                    // k-means 사전 반복 횟수
      static_cast<arma::uword>(params.max_em_iterations),
      params.convergence_tol,  // EM 수렴 임계치 (GMMFitParams::convergence_tol)
      false);                  // verbose 비활성화

  if (!ok) {
    // EM 실패 시 단일 컴포넌트 폴백 (수렴 불가 등)
    return single_component_fallback(points, params.covariance_regularization);
  }

  return extract_result(model, params, ok);
}

// Warm-start GMM 피팅: 기존 컴포넌트를 Armadillo 모델에 set_params()로 주입 후
// keep_existing 모드로 EM 재실행. 포인트 분포가 점진적으로 변할 때 유리.
GMMFitResult fit_gmm(
    const std::vector<Eigen::Vector4d>& points,
    const GMMFitParams& params,
  const std::vector<GMMComponent>& initial_components)
{
  const size_t N = points.size();

  // 무효한 입력이면 cold-start로 폴백
  if (N < 2 || initial_components.empty()) {
    return fit_gmm(points, params);
  }

  // Oracle 이슈 #1: initial_components 개수가 params.max_components보다 많으면
  // (설정 변경 등으로 K 불일치 발생) warm-start 대신 cold-start로 폴백.
  // Armadillo는 K를 학습 시 고정하므로, 의도와 다른 K로 진행되는 것을 방지.
  const int K = static_cast<int>(initial_components.size());
  if (K > params.max_components) {
    return fit_gmm(points, params);
  }
  arma::mat data = eigen_to_arma(points);

  // 기존 컴포넌트를 Armadillo 형식으로 변환
  arma::mat means(3, K);       // 3 × K 평균 행렬
  arma::cube fcovs(3, 3, K);   // 3 × 3 × K 공분산 큐브
  arma::rowvec hefts(K);       // 1 × K 가중치 벡터

  for (int k = 0; k < K; k++) {
    const auto& comp = initial_components[k];
    means(0, k) = comp.mean(0);
    means(1, k) = comp.mean(1);
    means(2, k) = comp.mean(2);

    // 사후 정칙화를 제거한 "순수" 공분산을 Armadillo에 전달:
    // extract_result()가 EM 후 다시 정칙화를 추가하므로, 이중 적용 방지.
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        double val = comp.cov(r, c);
        if (r == c) {
          val -= params.covariance_regularization;  // ε·I 제거
          val = std::max(val, 1e-6);  // 음수 방지: 수치 안정성
        }
        fcovs(r, c, k) = val;
      }
    }

    hefts(k) = comp.weight;  // π_k
  }

  // 가중치 합 = 1 보장 (Armadillo 요구사항)
  double heft_sum = arma::accu(hefts);
  if (heft_sum > 0.0) {
    hefts /= heft_sum;
  } else {
    hefts.fill(1.0 / K);  // 전부 0이면 균등 분배
  }

  arma::gmm_full model;
  model.set_params(means, fcovs, hefts);  // 초기 파라미터 주입

  // keep_existing: set_params()로 설정한 초기값에서 EM 시작 (k-means 생략)
  bool ok = model.learn(
      data,
      static_cast<arma::uword>(K),
      arma::eucl_dist,
      arma::keep_existing,   // 기존 파라미터 유지하고 EM만 실행
      0,                     // k-means 반복 = 0 (불필요)
      static_cast<arma::uword>(params.max_em_iterations),
      params.convergence_tol,
      false);

  if (!ok) {
    // Warm-start 실패 시 cold-start로 재시도
    return fit_gmm(points, params);
  }

  return extract_result(model, params, ok);
}

}  // namespace gtsam_points
