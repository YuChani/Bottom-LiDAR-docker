// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)

#include <gtsam_points/factors/integrated_ndt_factor.hpp>

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
IntegratedNDTFactor_<SourceFrame>::IntegratedNDTFactor_(
  gtsam::Key target_key,
  gtsam::Key source_key,
  const GaussianVoxelMap::ConstPtr& target_voxels,
  const std::shared_ptr<const SourceFrame>& source)
: gtsam_points::IntegratedMatchingCostFactor(target_key, source_key),
  num_threads(1),
  resolution(1.0),
  outlier_ratio(0.55),
  regularization_epsilon(1e-3),
  search_mode(NDTSearchMode::DIRECT7),
  gauss_d1(0.0),
  gauss_d2(0.0),
  target_voxels(std::dynamic_pointer_cast<const GaussianVoxelMapCPU>(target_voxels)),
  source(source) {
  //
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
IntegratedNDTFactor_<SourceFrame>::IntegratedNDTFactor_(
  const gtsam::Pose3& fixed_target_pose,
  gtsam::Key source_key,
  const GaussianVoxelMap::ConstPtr& target_voxels,
  const std::shared_ptr<const SourceFrame>& source)
: gtsam_points::IntegratedMatchingCostFactor(fixed_target_pose, source_key),
  num_threads(1),
  resolution(1.0),
  outlier_ratio(0.55),
  regularization_epsilon(1e-3),
  search_mode(NDTSearchMode::DIRECT7),
  gauss_d1(0.0),
  gauss_d2(0.0),
  target_voxels(std::dynamic_pointer_cast<const GaussianVoxelMapCPU>(target_voxels)),
  source(source) {
  //
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
IntegratedNDTFactor_<SourceFrame>::~IntegratedNDTFactor_() {}

template <typename SourceFrame>
void IntegratedNDTFactor_<SourceFrame>::print(const std::string& s, const gtsam::KeyFormatter& keyFormatter) const {
  std::cout << s << "IntegratedNDTFactor";
  if (is_binary) {
    std::cout << "(" << keyFormatter(this->keys()[0]) << ", " << keyFormatter(this->keys()[1]) << ")" << std::endl;
  } else {
    std::cout << "(fixed, " << keyFormatter(this->keys()[0]) << ")" << std::endl;
  }

  std::cout << "target_resolution=" << target_voxels->voxel_resolution() << ", |source|=" << frame::size(*source) << "pts" << std::endl;
  std::cout << "num_threads=" << num_threads << ", search_mode=";
  switch (search_mode) {
    case NDTSearchMode::DIRECT1:
      std::cout << "DIRECT1";
      break;
    case NDTSearchMode::DIRECT7:
      std::cout << "DIRECT7";
      break;
    case NDTSearchMode::DIRECT27:
      std::cout << "DIRECT27";
      break;
  }
  std::cout << std::endl;
}

template <typename SourceFrame>
size_t IntegratedNDTFactor_<SourceFrame>::memory_usage() const {
  return sizeof(*this) + sizeof(const GaussianVoxel*) * correspondences.capacity();
}

template <typename SourceFrame>
void IntegratedNDTFactor_<SourceFrame>::update_correspondences(const Eigen::Isometry3d& delta) const {
  linearization_point = delta;
  correspondences.resize(frame::size(*source));

  // Compute NDT Gaussian parameters (d1, d2) from resolution and outlier_ratio
  GaussianVoxel::compute_ndt_params(resolution, outlier_ratio, gauss_d1, gauss_d2);

  // Generate neighbor offset list based on search mode
  std::vector<Eigen::Vector3i> neighbor_offsets;
  switch (search_mode) {
    case NDTSearchMode::DIRECT1:
      // Current voxel only
      neighbor_offsets.push_back(Eigen::Vector3i(0, 0, 0));
      break;
    case NDTSearchMode::DIRECT7:
      // Current voxel + 6 face neighbors
      neighbor_offsets.push_back(Eigen::Vector3i(0, 0, 0));
      neighbor_offsets.push_back(Eigen::Vector3i(1, 0, 0));
      neighbor_offsets.push_back(Eigen::Vector3i(-1, 0, 0));
      neighbor_offsets.push_back(Eigen::Vector3i(0, 1, 0));
      neighbor_offsets.push_back(Eigen::Vector3i(0, -1, 0));
      neighbor_offsets.push_back(Eigen::Vector3i(0, 0, 1));
      neighbor_offsets.push_back(Eigen::Vector3i(0, 0, -1));
      break;
    case NDTSearchMode::DIRECT27:
      // Current voxel + 26 neighbors (3x3x3 cube)
      for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
          for (int dz = -1; dz <= 1; dz++) {
            neighbor_offsets.push_back(Eigen::Vector3i(dx, dy, dz));
          }
        }
      }
      break;
  }

  const auto perpoint_task = [&](int i) {
    Eigen::Vector4d pt = delta * frame::point(*source, i);
    Eigen::Vector3i coord = target_voxels->voxel_coord(pt);

    // Find the best voxel (minimum Mahalanobis distance)
    const GaussianVoxel* best_voxel = nullptr;
    double min_mahalanobis = std::numeric_limits<double>::max();

    for (const auto& offset : neighbor_offsets) {
      Eigen::Vector3i neighbor_coord = coord + offset;
      const auto voxel_id = target_voxels->lookup_voxel_index(neighbor_coord);

      if (voxel_id < 0) {
        continue;
      }

      auto* voxel = const_cast<GaussianVoxel*>(&target_voxels->lookup_voxel(voxel_id));
      
      if (!voxel->inv_cov_valid) {
        voxel->compute_inverse_covariance(regularization_epsilon);
      }

      Eigen::Vector4d diff = pt - voxel->mean;
      double mahalanobis_dist = diff.transpose() * voxel->inv_cov * diff;

      if (mahalanobis_dist < min_mahalanobis) {
        min_mahalanobis = mahalanobis_dist;
        best_voxel = voxel;
      }
    }

    correspondences[i] = best_voxel;
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
double IntegratedNDTFactor_<SourceFrame>::evaluate(
  const Eigen::Isometry3d& delta,
  Eigen::Matrix<double, 6, 6>* H_target,
  Eigen::Matrix<double, 6, 6>* H_source,
  Eigen::Matrix<double, 6, 6>* H_target_source,
  Eigen::Matrix<double, 6, 1>* b_target,
  Eigen::Matrix<double, 6, 1>* b_source) const {
  //
  if (correspondences.size() != frame::size(*source)) {
    update_correspondences(delta);
  }

  double sum_errors = 0.0;

  const auto perpoint_task = [&](
                               int i,
                               Eigen::Matrix<double, 6, 6>* H_target,
                               Eigen::Matrix<double, 6, 6>* H_source,
                               Eigen::Matrix<double, 6, 6>* H_target_source,
                               Eigen::Matrix<double, 6, 1>* b_target,
                               Eigen::Matrix<double, 6, 1>* b_source) {
    const auto& target_voxel = correspondences[i];
    if (target_voxel == nullptr) {
      return 0.0;
    }

    const auto& mean_A = frame::point(*source, i);
    const auto& mean_B = target_voxel->mean;
    const auto& inv_cov_B = target_voxel->inv_cov;

    Eigen::Vector4d transed_mean_A = delta * mean_A;
    Eigen::Vector4d residual = transed_mean_A - mean_B;

    // Compute Mahalanobis distance (only 3D part)
    Eigen::Vector3d residual_3d = residual.head<3>();
    Eigen::Matrix3d inv_cov_3d = inv_cov_B.topLeftCorner<3, 3>();
    double mahalanobis_dist = residual_3d.transpose() * inv_cov_3d * residual_3d;

    // NDT score: -d1 * exp(-d2/2 * mahalanobis_dist)
    // Exponential NDT formula (Magnusson 2009 Equation 6.9-6.10)
    double exponent = -gauss_d2 * mahalanobis_dist / 2.0;

    // Numerical safeguard: Prevent underflow (exp(-700) ≈ 0)
    if (exponent < -700.0) {
      return 0.0;  // Far outlier: zero contribution
    }

    double e_term = std::exp(exponent);
    double e_scaled = gauss_d2 * e_term;

    // Numerical safeguard: Validate scaled term (catches overflow, NaN)
    if (e_scaled > 1.0 || e_scaled < 0.0 || std::isnan(e_scaled)) {
      return 0.0;  // Invalid value: zero contribution
    }

    const double error = -gauss_d1 * e_term;

    if (!H_target) {
      return error;
    }

    // Compute Jacobians using Lie algebra
    Eigen::Matrix<double, 4, 6> J_target = Eigen::Matrix<double, 4, 6>::Zero();
    J_target.block<3, 3>(0, 0) = -gtsam::SO3::Hat(transed_mean_A.head<3>());
    J_target.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 4, 6> J_source = Eigen::Matrix<double, 4, 6>::Zero();
    J_source.block<3, 3>(0, 0) = delta.linear() * gtsam::SO3::Hat(mean_A.template head<3>());
    J_source.block<3, 3>(0, 3) = -delta.linear();

    // Compute derivative scaling for exponential formula
    // d(error)/d(mahalanobis) = (gauss_d1 * gauss_d2 / 2.0) * exp(-gauss_d2 * mahalanobis / 2.0)
    double derivative_scale = (gauss_d1 * gauss_d2 / 2.0) * e_term;

    // Apply inverse covariance (Mahalanobis weighting) and derivative scaling
    Eigen::Matrix<double, 6, 4> J_target_weighted = derivative_scale * J_target.transpose() * inv_cov_B;
    Eigen::Matrix<double, 6, 4> J_source_weighted = derivative_scale * J_source.transpose() * inv_cov_B;

    *H_target += J_target_weighted * J_target;
    *H_source += J_source_weighted * J_source;
    *H_target_source += J_target_weighted * J_source;
    *b_target += J_target_weighted * residual;
    *b_source += J_source_weighted * residual;

    return error;
  };

  if (is_omp_default() || num_threads == 1) {
    return scan_matching_reduce_omp(perpoint_task, frame::size(*source), num_threads, H_target, H_source, H_target_source, b_target, b_source);
  } else {
    return scan_matching_reduce_tbb(perpoint_task, frame::size(*source), H_target, H_source, H_target_source, b_target, b_source);
  }
}

}  // namespace gtsam_points
