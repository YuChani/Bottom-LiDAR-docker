// SPDX-License-Identifier: MIT
#include "gmm/mixture_em_backend.hpp"

#include <algorithm>
#include <numeric>
#include <armadillo>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace gtsam_points {

namespace {

GMMFitResult single_component_fallback(
    const std::vector<Eigen::Vector4d>& points,
    double regularization) {
  const size_t N = points.size();

  Eigen::Vector4d mean = Eigen::Vector4d::Zero();
  for (const auto& pt : points) {
    mean += pt;
  }
  mean /= static_cast<double>(N);

  Eigen::Matrix4d cov = Eigen::Matrix4d::Zero();
  if (N > 1) {
    for (const auto& pt : points) {
      const Eigen::Vector4d diff = pt - mean;
      cov += diff * diff.transpose();
    }
    cov /= static_cast<double>(N);
  }

  cov.block<3, 3>(0, 0) += regularization * Eigen::Matrix3d::Identity();
  cov.row(3).setZero();
  cov.col(3).setZero();

  GMMComponent comp;
  comp.mean = mean;
  comp.cov = cov;
  comp.weight = 1.0;

  GMMFitResult result;
  result.components.push_back(comp);
  result.converged = true;
  result.iterations_run = 0;
  return result;
}

// Eigen Vector4d (homogeneous) → arma::mat (3 x N), extracting xyz
arma::mat eigen_to_arma(const std::vector<Eigen::Vector4d>& points) {
  const size_t N = points.size();
  arma::mat data(3, N);
  for (size_t i = 0; i < N; i++) {
    data(0, i) = points[i](0);
    data(1, i) = points[i](1);
    data(2, i) = points[i](2);
  }
  return data;
}

GMMFitResult extract_result(
    const arma::gmm_full& model,
    const GMMFitParams& params,
    bool converged) {
  GMMFitResult result;
  result.converged = converged;
  result.iterations_run = params.max_em_iterations;

  const arma::uword K = model.n_gaus();

  for (arma::uword k = 0; k < K; k++) {
    GMMComponent comp;

    const arma::vec& m = model.means.col(k);
    comp.mean = Eigen::Vector4d(m(0), m(1), m(2), 0.0);

    comp.cov = Eigen::Matrix4d::Zero();
    const arma::mat& fcov = model.fcovs.slice(k);
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        comp.cov(r, c) = fcov(r, c);
      }
    }
    comp.cov.block<3, 3>(0, 0) += params.covariance_regularization * Eigen::Matrix3d::Identity();

    comp.weight = model.hefts(k);
    result.components.push_back(comp);
  }

  // Compute weighted mean before pruning (fallback if all components are pruned)
  Eigen::Vector4d weighted_mean = Eigen::Vector4d::Zero();
  double pre_prune_total_weight = 0.0;
  for (const auto& c : result.components) {
    weighted_mean += c.weight * c.mean;
    pre_prune_total_weight += c.weight;
  }
  if (pre_prune_total_weight > 0.0) {
    weighted_mean /= pre_prune_total_weight;
  }

  result.components.erase(
      std::remove_if(result.components.begin(), result.components.end(),
                     [&](const GMMComponent& c) { return c.weight < params.min_weight_threshold; }),
      result.components.end());

  if (result.components.empty()) {
    GMMComponent fallback;
    fallback.mean = weighted_mean;
    fallback.weight = 1.0;
    fallback.cov.block<3, 3>(0, 0) =
        params.covariance_regularization * Eigen::Matrix3d::Identity();
    result.components.push_back(fallback);
    return result;
  }

  // Renormalize weights
  double total_weight = 0.0;
  for (const auto& c : result.components) {
    total_weight += c.weight;
  }
  if (total_weight > 0.0) {
    for (auto& c : result.components) {
      c.weight /= total_weight;
    }
  }

  return result;
}

}  // namespace

GMMFitResult fit_gmm(
    const std::vector<Eigen::Vector4d>& points,
    const GMMFitParams& params) {
  const size_t N = points.size();

  if (N < 2) {
    return single_component_fallback(points, params.covariance_regularization);
  }

  const int K = std::min(params.max_components, static_cast<int>(N));

  arma::mat data = eigen_to_arma(points);

  arma::gmm_full model;
  bool ok = model.learn(
      data,
      static_cast<arma::uword>(K),
      arma::eucl_dist,
      arma::random_spread,
      20,
      static_cast<arma::uword>(params.max_em_iterations),
      1e-6,
      false);

  if (!ok) {
    return single_component_fallback(points, params.covariance_regularization);
  }

  return extract_result(model, params, ok);
}

GMMFitResult fit_gmm(
    const std::vector<Eigen::Vector4d>& points,
    const GMMFitParams& params,
    const std::vector<GMMComponent>& initial_components) {
  const size_t N = points.size();

  if (N < 2 || initial_components.empty()) {
    return fit_gmm(points, params);
  }

  const int K = static_cast<int>(initial_components.size());
  arma::mat data = eigen_to_arma(points);

  // Set up warm-start parameters from existing components
  arma::mat means(3, K);
  arma::cube fcovs(3, 3, K);
  arma::rowvec hefts(K);

  for (int k = 0; k < K; k++) {
    const auto& comp = initial_components[k];
    means(0, k) = comp.mean(0);
    means(1, k) = comp.mean(1);
    means(2, k) = comp.mean(2);

    // Subtract post-hoc regularization before feeding to Armadillo
    // (extract_result() will re-add it after EM converges)
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        double val = comp.cov(r, c);
        if (r == c) {
          val -= params.covariance_regularization;
          val = std::max(val, 1e-6);
        }
        fcovs(r, c, k) = val;
      }
    }

    hefts(k) = comp.weight;
  }

  // Ensure hefts sum to 1
  double heft_sum = arma::accu(hefts);
  if (heft_sum > 0.0) {
    hefts /= heft_sum;
  } else {
    hefts.fill(1.0 / K);
  }

  arma::gmm_full model;
  model.set_params(means, fcovs, hefts);

  bool ok = model.learn(
      data,
      static_cast<arma::uword>(K),
      arma::eucl_dist,
      arma::keep_existing,
      0,
      static_cast<arma::uword>(params.max_em_iterations),
      1e-6,
      false);

  if (!ok) {
    return fit_gmm(points, params);
  }

  return extract_result(model, params, ok);
}

}  // namespace gtsam_points
