// SPDX-License-Identifier: MIT
// Copyright (c) 2026

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <gtsam_points/factors/integrated_gmm_ndt_factor.hpp>

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

template <typename SourceFrame>
IntegratedGMMNDTFactor_<SourceFrame>::IntegratedGMMNDTFactor_(
  gtsam::Key target_key,
  gtsam::Key source_key,
  const GaussianVoxelMap::ConstPtr& target_voxels,
  const std::shared_ptr<const SourceFrame>& source)
: gtsam_points::IntegratedMatchingCostFactor(target_key, source_key),
  num_threads(1),
  resolution(1.0),
  outlier_ratio(0.1),
  regularization_epsilon(1e-3),
  search_mode(NDTSearchMode::DIRECT7),
  num_components(2),
  gamma_min(1e-4),
  freeze_mixture_in_lm(false),
  use_uniform_outlier(false),
  uniform_outlier_weight(1e-4),
  mahalanobis_ratio_threshold(5.0),
  gamma_hessian_threshold(0.05),
  correspondence_update_tolerance_rot(0.0),
  correspondence_update_tolerance_trans(0.0),
  gauss_d1(0.0),
  gauss_d2(0.0),
  inv_cov_cached(false),
  target_voxels(std::dynamic_pointer_cast<const GaussianVoxelMapCPU>(target_voxels)),
  source(source) {
  if (!frame::has_points(*source)) {
    std::cerr << "error: source points have not been allocated!!" << std::endl;
    abort();
  }
  if (!this->target_voxels) {
    std::cerr << "error: target voxelmap has not been created!!" << std::endl;
    abort();
  }
}

template <typename SourceFrame>
IntegratedGMMNDTFactor_<SourceFrame>::IntegratedGMMNDTFactor_(
  const gtsam::Pose3& fixed_target_pose,
  gtsam::Key source_key,
  const GaussianVoxelMap::ConstPtr& target_voxels,
  const std::shared_ptr<const SourceFrame>& source)
: gtsam_points::IntegratedMatchingCostFactor(fixed_target_pose, source_key),
  num_threads(1),
  resolution(1.0),
  outlier_ratio(0.1),
  regularization_epsilon(1e-3),
  search_mode(NDTSearchMode::DIRECT7),
  num_components(2),
  gamma_min(1e-4),
  freeze_mixture_in_lm(false),
  use_uniform_outlier(false),
  uniform_outlier_weight(1e-4),
  mahalanobis_ratio_threshold(5.0),
  gamma_hessian_threshold(0.05),
  correspondence_update_tolerance_rot(0.0),
  correspondence_update_tolerance_trans(0.0),
  gauss_d1(0.0),
  gauss_d2(0.0),
  inv_cov_cached(false),
  target_voxels(std::dynamic_pointer_cast<const GaussianVoxelMapCPU>(target_voxels)),
  source(source) {
  if (!frame::has_points(*source)) {
    std::cerr << "error: source points have not been allocated!!" << std::endl;
    abort();
  }
  if (!this->target_voxels) {
    std::cerr << "error: target voxelmap has not been created!!" << std::endl;
    abort();
  }
}

template <typename SourceFrame>
IntegratedGMMNDTFactor_<SourceFrame>::~IntegratedGMMNDTFactor_() {}

template <typename SourceFrame>
void IntegratedGMMNDTFactor_<SourceFrame>::print(const std::string& s, const gtsam::KeyFormatter& keyFormatter) const {
  std::cout << s << "IntegratedGMMNDTFactor";
  if (is_binary) {
    std::cout << "(" << keyFormatter(this->keys()[0]) << ", " << keyFormatter(this->keys()[1]) << ")" << std::endl;
  } else {
    std::cout << "(fixed, " << keyFormatter(this->keys()[0]) << ")" << std::endl;
  }
  std::cout << "target_resolution=" << target_voxels->voxel_resolution() << ", |source|=" << frame::size(*source) << "pts" << std::endl;
}

template <typename SourceFrame>
size_t IntegratedGMMNDTFactor_<SourceFrame>::memory_usage() const {
  size_t component_capacity = 0;
  for (const auto& corr : correspondences) {
    component_capacity += corr.components.capacity();
  }
  return sizeof(*this) + sizeof(GmmPointCorrespondence) * correspondences.capacity() + sizeof(GmmComponentMatch) * component_capacity;
}

template <typename SourceFrame>
void IntegratedGMMNDTFactor_<SourceFrame>::compute_responsibilities(const Eigen::Vector4d& transformed_point,
                                                                     GmmPointCorrespondence* correspondence) const {
  if (!correspondence || !correspondence->valid || correspondence->components.empty()) {
    return;
  }

  std::vector<double> log_weights(correspondence->components.size(), -std::numeric_limits<double>::infinity());
  double max_log_weight = -std::numeric_limits<double>::infinity();

  for (size_t k = 0; k < correspondence->components.size(); ++k) {
    auto& component = correspondence->components[k];
    if (!component.valid) {
      continue;
    }

    const Eigen::Vector4d residual = component.mean - transformed_point;
    const double d2 = residual.transpose() * component.inv_cov * residual;
    const double log_prior = std::log(std::max(component.prior, 1e-12));
    const double log_prob = log_prior - 0.5 * (d2 + component.log_det_cov);

    log_weights[k] = log_prob;
    max_log_weight = std::max(max_log_weight, log_prob);
  }

  if (!std::isfinite(max_log_weight)) {
    const double uniform_gamma = 1.0 / static_cast<double>(correspondence->components.size());
    for (auto& component : correspondence->components) {
      component.gamma = component.valid ? uniform_gamma : 0.0;
    }
    return;
  }

  double sum_exp = 0.0;
  for (const double lw : log_weights) {
    if (std::isfinite(lw)) {
      sum_exp += std::exp(lw - max_log_weight);
    }
  }

  if (use_uniform_outlier) {
    const double log_uniform = std::log(std::max(uniform_outlier_weight, 1e-12));
    sum_exp += std::exp(log_uniform - max_log_weight);
  }

  if (sum_exp <= 0.0) {
    const double uniform_gamma = 1.0 / static_cast<double>(correspondence->components.size());
    for (auto& component : correspondence->components) {
      component.gamma = component.valid ? uniform_gamma : 0.0;
    }
    return;
  }

  for (size_t k = 0; k < correspondence->components.size(); ++k) {
    auto& component = correspondence->components[k];
    if (!component.valid || !std::isfinite(log_weights[k])) {
      component.gamma = 0.0;
      continue;
    }
    component.gamma = std::exp(log_weights[k] - max_log_weight) / sum_exp;
  }

  if (gamma_min > 0.0) {
    double gamma_sum = 0.0;
    for (auto& component : correspondence->components) {
      if (!component.valid) {
        continue;
      }
      component.gamma = std::max(component.gamma, gamma_min);
      gamma_sum += component.gamma;
    }

    if (gamma_sum > 0.0) {
      for (auto& component : correspondence->components) {
        if (!component.valid) {
          continue;
        }
        component.gamma /= gamma_sum;
      }
    }
  }
}

template <typename SourceFrame>
void IntegratedGMMNDTFactor_<SourceFrame>::update_correspondences(const Eigen::Isometry3d& delta) const {
  linearization_point = delta;

  const bool has_correspondences = correspondences.size() == frame::size(*source);
  double diff_rot = std::numeric_limits<double>::infinity();
  double diff_trans = std::numeric_limits<double>::infinity();
  if (has_correspondences) {
    const Eigen::Isometry3d diff = delta.inverse() * last_correspondence_point;
    diff_rot = Eigen::AngleAxisd(diff.linear()).angle();
    diff_trans = diff.translation().norm();

    constexpr double kSamePoseRotEps = 1e-12;
    constexpr double kSamePoseTransEps = 1e-12;
    if (diff_rot < kSamePoseRotEps && diff_trans < kSamePoseTransEps) {
      return;
    }
  }

  if (freeze_mixture_in_lm && has_correspondences) {
    return;
  }

  bool do_update = true;
  if (has_correspondences && (correspondence_update_tolerance_trans > 0.0 || correspondence_update_tolerance_rot > 0.0)) {
    if (diff_rot < correspondence_update_tolerance_rot && diff_trans < correspondence_update_tolerance_trans) {
      do_update = false;
    }
  }

  if (!do_update) {
    return;
  }

  last_correspondence_point = delta;
  correspondences.resize(frame::size(*source));

  compute_ndt_params(resolution, outlier_ratio, gauss_d1, gauss_d2);

  std::vector<Eigen::Vector3i> neighbor_offsets;
  switch (search_mode) {
    case NDTSearchMode::DIRECT1:
      neighbor_offsets.push_back(Eigen::Vector3i(0, 0, 0));
      break;
    case NDTSearchMode::DIRECT7:
      neighbor_offsets.push_back(Eigen::Vector3i(0, 0, 0));
      neighbor_offsets.push_back(Eigen::Vector3i(1, 0, 0));
      neighbor_offsets.push_back(Eigen::Vector3i(-1, 0, 0));
      neighbor_offsets.push_back(Eigen::Vector3i(0, 1, 0));
      neighbor_offsets.push_back(Eigen::Vector3i(0, -1, 0));
      neighbor_offsets.push_back(Eigen::Vector3i(0, 0, 1));
      neighbor_offsets.push_back(Eigen::Vector3i(0, 0, -1));
      break;
    case NDTSearchMode::DIRECT27:
      for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
          for (int dz = -1; dz <= 1; dz++) {
            neighbor_offsets.push_back(Eigen::Vector3i(dx, dy, dz));
          }
        }
      }
      break;
  }

  if (!inv_cov_cached) {
    const size_t num_voxels = target_voxels->num_voxels();
    inv_cov_cache.resize(num_voxels);
    log_det_cov_cache.resize(num_voxels);
    for (size_t v = 0; v < num_voxels; v++) {
      const auto& voxel = target_voxels->lookup_voxel(v);
      inv_cov_cache[v] = compute_ndt_inverse_covariance(voxel.cov, regularization_epsilon);

      // Use 3x3 top-left block for log-determinant (4th dim is homogeneous w=0, not a real DOF)
      // log_det_cov = log(det(Sigma_reg_3x3)) + 3*log(2*pi)
      //             = -log(det(inv_cov_3x3)) + 3*log(2*pi)
      const Eigen::Matrix3d inv_cov_3x3 = inv_cov_cache[v].block<3, 3>(0, 0);
      const double det_inv_3x3 = std::max(inv_cov_3x3.determinant(), 1e-300);
      log_det_cov_cache[v] = -std::log(det_inv_3x3) + 3.0 * std::log(2.0 * M_PI);
    }
    inv_cov_cached = true;
  }

  const int k_components = std::max(1, num_components);
  const auto perpoint_task = [&](int i) {
    auto& corr = correspondences[i];
    corr.valid = false;
    corr.components.clear();

    const Eigen::Vector4d transformed_point = delta * frame::point(*source, i);
    const Eigen::Vector3i coord = target_voxels->voxel_coord(transformed_point);

    std::vector<std::pair<double, int>> candidates;
    candidates.reserve(neighbor_offsets.size());

    for (const auto& offset : neighbor_offsets) {
      const Eigen::Vector3i neighbor_coord = coord + offset;
      const int voxel_id = target_voxels->lookup_voxel_index(neighbor_coord);
      if (voxel_id < 0) {
        continue;
      }

      const auto& voxel = target_voxels->lookup_voxel(voxel_id);
      const Eigen::Vector4d diff = transformed_point - voxel.mean;
      const double d2 = diff.transpose() * inv_cov_cache[voxel_id] * diff;
      candidates.emplace_back(d2, voxel_id);
    }

    if (candidates.empty()) {
      return;
    }

    // Replace full sort with partial selection (K-best)
    // For K=2 and max 7 candidates (DIRECT7), nth_element is sufficient
    if (static_cast<int>(candidates.size()) > k_components) {
      std::nth_element(candidates.begin(), candidates.begin() + k_components, candidates.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });
    }
    // Sort only the top-K for deterministic ordering
    std::sort(candidates.begin(), candidates.begin() + std::min(k_components, static_cast<int>(candidates.size())),
      [](const auto& a, const auto& b) { return a.first < b.first; });

    const int keep_count = std::min(k_components, static_cast<int>(candidates.size()));
    corr.components.reserve(keep_count);
    const double best_d2 = candidates[0].first;

    double prior_sum = 0.0;
    for (int k = 0; k < keep_count; ++k) {
      // Mahalanobis ratio pruning: skip if 2nd+ candidate is too far from best
      if (k > 0 && best_d2 > 0.0 && candidates[k].first > mahalanobis_ratio_threshold * best_d2) {
        break;
      }

      const int voxel_id = candidates[k].second;
      const auto& voxel = target_voxels->lookup_voxel(voxel_id);

      GmmComponentMatch comp;
      comp.mean = voxel.mean;
      comp.inv_cov = inv_cov_cache[voxel_id];
      comp.log_det_cov = log_det_cov_cache[voxel_id];
      comp.prior = std::max(1.0, static_cast<double>(voxel.num_points));
      comp.valid = true;

      prior_sum += comp.prior;
      corr.components.push_back(comp);
    }

    if (prior_sum <= 0.0) {
      const double uniform_prior = 1.0 / static_cast<double>(corr.components.size());
      for (auto& comp : corr.components) {
        comp.prior = uniform_prior;
      }
    } else {
      for (auto& comp : corr.components) {
        comp.prior /= prior_sum;
      }
    }

    corr.valid = !corr.components.empty();
    if (corr.valid) {
      compute_responsibilities(transformed_point, &corr);
    }
  };

  if (is_omp_default() || num_threads == 1) {
#pragma omp parallel for num_threads(num_threads) schedule(guided, 8)
    for (int i = 0; i < frame::size(*source); i++) {
      perpoint_task(i);
    }
  } else {
#ifdef GTSAM_POINTS_USE_TBB
    tbb::parallel_for(tbb::blocked_range<int>(0, frame::size(*source), 8), [&](const tbb::blocked_range<int>& range) {
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

template <typename SourceFrame>
double IntegratedGMMNDTFactor_<SourceFrame>::evaluate(
  const Eigen::Isometry3d& delta,
  Eigen::Matrix<double, 6, 6>* H_target,
  Eigen::Matrix<double, 6, 6>* H_source,
  Eigen::Matrix<double, 6, 6>* H_target_source,
  Eigen::Matrix<double, 6, 1>* b_target,
  Eigen::Matrix<double, 6, 1>* b_source) const {
  if (correspondences.size() != frame::size(*source)) {
    update_correspondences(delta);
  }

  const auto perpoint_task = [&](int i,
                                 Eigen::Matrix<double, 6, 6>* H_target,
                                 Eigen::Matrix<double, 6, 6>* H_source,
                                 Eigen::Matrix<double, 6, 6>* H_target_source,
                                 Eigen::Matrix<double, 6, 1>* b_target,
                                 Eigen::Matrix<double, 6, 1>* b_source) {
    const auto& corr = correspondences[i];
    if (!corr.valid || corr.components.empty()) {
      return 0.0;
    }

    const auto& mean_A = frame::point(*source, i);
    const Eigen::Vector4d transed_mean_A = delta * mean_A;

    Eigen::Matrix<double, 4, 6> J_target;
    Eigen::Matrix<double, 4, 6> J_source;
    if (H_target) {
      J_target = Eigen::Matrix<double, 4, 6>::Zero();
      J_target.block<3, 3>(0, 0) = -gtsam::SO3::Hat(transed_mean_A.head<3>());
      J_target.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

      J_source = Eigen::Matrix<double, 4, 6>::Zero();
      J_source.block<3, 3>(0, 0) = delta.linear() * gtsam::SO3::Hat(mean_A.template head<3>());
      J_source.block<3, 3>(0, 3) = -delta.linear();
    }

    double point_cost = 0.0;
    // For K>1: use algebraic collapse to accumulate weighted precision matrix
    // M = sum_k (w_k * Omega_k) and u = sum_k (w_k * Omega_k * r_k)
    // Then compute J^T * M * J once instead of K times.
    // For K=1: use direct computation to maintain exact numerical parity with NDT.
    const bool use_collapse = (corr.components.size() > 1);
    Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
    Eigen::Vector4d u = Eigen::Vector4d::Zero();

    for (const auto& comp : corr.components) {
      if (!comp.valid || comp.gamma <= 0.0) {
        continue;
      }

      const Eigen::Vector4d residual = comp.mean - transed_mean_A;
      const double mahalanobis_dist = residual.transpose() * comp.inv_cov * residual;

      const double exponent = -gauss_d2 * mahalanobis_dist / 2.0;
      if (exponent < -700.0) {
        continue;
      }

      const double e_term = std::exp(exponent);
      if (std::isnan(e_term)) {
        continue;
      }

      const double score_function = -gauss_d1 * e_term;
      const double component_cost = -gauss_d1 - score_function;
      point_cost += comp.gamma * component_cost;

      if (!H_target || comp.gamma < gamma_hessian_threshold) {
        continue;
      }

      const double weight = comp.gamma * (-gauss_d1 * gauss_d2 * e_term);

      if (use_collapse) {
        // Accumulate for batch Hessian computation
        M.noalias() += weight * comp.inv_cov;
        u.noalias() += weight * (comp.inv_cov * residual);
      } else {
        // Direct computation (K=1 path, exact parity with NDT)
        *H_target += weight * J_target.transpose() * comp.inv_cov * J_target;
        *H_source += weight * J_source.transpose() * comp.inv_cov * J_source;
        *H_target_source += weight * J_target.transpose() * comp.inv_cov * J_source;
        *b_target += weight * J_target.transpose() * comp.inv_cov * residual;
        *b_source += weight * J_source.transpose() * comp.inv_cov * residual;
      }
    }

    // Batch Hessian/gradient for multi-component case (K>1)
    if (use_collapse && H_target && M.squaredNorm() > 0.0) {
      *H_target += J_target.transpose() * M * J_target;
      *H_source += J_source.transpose() * M * J_source;
      *H_target_source += J_target.transpose() * M * J_source;
      *b_target += J_target.transpose() * u;
      *b_source += J_source.transpose() * u;
    }

    return point_cost;
  };

  if (is_omp_default() || num_threads == 1) {
    return scan_matching_reduce_omp(perpoint_task, frame::size(*source), num_threads, H_target, H_source, H_target_source, b_target, b_source);
  }

  return scan_matching_reduce_tbb(perpoint_task, frame::size(*source), H_target, H_source, H_target_source, b_target, b_source);
}

}  // namespace gtsam_points
