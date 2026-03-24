// SPDX-License-Identifier: MIT
#pragma once

#include <vector>
#include <Eigen/Core>
#include "gmm/gmm_voxelmap_cpu.hpp"

namespace gtsam_points {

struct GMMFitResult {
  std::vector<GMMComponent> components;
  bool converged = false;
  int iterations_run = 0;
};

struct GMMFitParams {
  int max_components = 3;
  int max_em_iterations = 20;
  double convergence_tol = 1e-4;
  double covariance_regularization = 1e-3;
  double min_weight_threshold = 0.01;
};

GMMFitResult fit_gmm(
    const std::vector<Eigen::Vector4d>& points,
    const GMMFitParams& params);

GMMFitResult fit_gmm(
    const std::vector<Eigen::Vector4d>& points,
    const GMMFitParams& params,
    const std::vector<GMMComponent>& initial_components);

}  // namespace gtsam_points
