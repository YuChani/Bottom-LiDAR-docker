// SPDX-License-Identifier: MIT
// Copyright (c) 2026

#include "ndt/integrated_light_ndt_factor.hpp"

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
IntegratedLightNDTFactor_<SourceFrame>::IntegratedLightNDTFactor_(
  gtsam::Key target_key,
  gtsam::Key source_key,
  const GaussianVoxelMap::ConstPtr& target_voxels,
  const std::shared_ptr<const SourceFrame>& source)
: gtsam_points::IntegratedMatchingCostFactor(target_key, source_key),
  num_threads(1),
  regularization_epsilon(1e-3),
  search_mode(NDTSearchMode::DIRECT7),
  correspondence_update_tolerance_rot(0.0),
  correspondence_update_tolerance_trans(0.0),
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
IntegratedLightNDTFactor_<SourceFrame>::IntegratedLightNDTFactor_(
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
IntegratedLightNDTFactor_<SourceFrame>::~IntegratedLightNDTFactor_() {}

template <typename SourceFrame>
void IntegratedLightNDTFactor_<SourceFrame>::print(const std::string& s, const gtsam::KeyFormatter& keyFormatter) const {
  std::cout << s << "IntegratedLightNDTFactor";
  if (is_binary) {
    std::cout << "(" << keyFormatter(this->keys()[0]) << ", " << keyFormatter(this->keys()[1]) << ")" << std::endl;
  } else {
    std::cout << "(fixed, " << keyFormatter(this->keys()[0]) << ")" << std::endl;
  }
  std::cout << "target_resolution=" << target_voxels->voxel_resolution()
            << ", |source|=" << frame::size(*source) << "pts" << std::endl;
}

template <typename SourceFrame>
size_t IntegratedLightNDTFactor_<SourceFrame>::memory_usage() const {
  return sizeof(*this) + sizeof(NdtCorrespondence) * correspondences.capacity();
}

template <typename SourceFrame>
void IntegratedLightNDTFactor_<SourceFrame>::update_correspondences(const Eigen::Isometry3d& delta) const {
  linearization_point = delta;

  bool do_update = true;
  if (correspondences.size() == frame::size(*source) && (correspondence_update_tolerance_trans > 0.0 || correspondence_update_tolerance_rot > 0.0)) {
    Eigen::Isometry3d diff = delta.inverse() * last_correspondence_point;
    double diff_rot = Eigen::AngleAxisd(diff.linear()).angle();
    double diff_trans = diff.translation().norm();
    if (diff_rot < correspondence_update_tolerance_rot && diff_trans < correspondence_update_tolerance_trans) {
      do_update = false;
    }
  }

  if (do_update) {
    last_correspondence_point = delta;
  }

  correspondences.resize(frame::size(*source));

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
      for (int dx = -1; dx <= 1; dx++)
        for (int dy = -1; dy <= 1; dy++)
          for (int dz = -1; dz <= 1; dz++)
            neighbor_offsets.push_back(Eigen::Vector3i(dx, dy, dz));
      break;
  }

  if (!inv_cov_cached) {
    const size_t num_voxels = target_voxels->num_voxels();
    inv_cov_cache.resize(num_voxels);
    for (size_t v = 0; v < num_voxels; v++) {
      const auto& voxel = target_voxels->lookup_voxel(v);
      inv_cov_cache[v] = compute_ndt_inverse_covariance(voxel.cov, regularization_epsilon);
    }
    inv_cov_cached = true;
  }

  const auto perpoint_task = [&](int i) {
    if (do_update) {
      correspondences[i].valid = false;

      Eigen::Vector4d pt = delta * frame::point(*source, i);
      Eigen::Vector3i coord = target_voxels->voxel_coord(pt);

      const GaussianVoxel* best_voxel = nullptr;
      Eigen::Matrix4d best_inv_cov = Eigen::Matrix4d::Zero();
      double min_mahalanobis = std::numeric_limits<double>::max();

      for (const auto& offset : neighbor_offsets) {
        Eigen::Vector3i neighbor_coord = coord + offset;
        const auto voxel_id = target_voxels->lookup_voxel_index(neighbor_coord);
        if (voxel_id < 0) continue;

        const auto& voxel = target_voxels->lookup_voxel(voxel_id);
        const Eigen::Matrix4d& inv_cov = inv_cov_cache[voxel_id];

        Eigen::Vector4d diff = pt - voxel.mean;
        double mahalanobis_dist = diff.transpose() * inv_cov * diff;

        if (mahalanobis_dist < min_mahalanobis) {
          min_mahalanobis = mahalanobis_dist;
          best_voxel = &voxel;
          best_inv_cov = inv_cov;
        }
      }

      if (best_voxel) {
        correspondences[i].mean = best_voxel->mean;
        correspondences[i].inv_cov = best_inv_cov;
        correspondences[i].valid = true;
      }
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
double IntegratedLightNDTFactor_<SourceFrame>::evaluate(
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
    if (!corr.valid) {
      return 0.0;
    }

    const auto& mean_A = frame::point(*source, i);
    const auto& mean_B = corr.mean;
    const auto& inv_cov_B = corr.inv_cov;

    Eigen::Vector4d transed_mean_A = delta * mean_A;
    Eigen::Vector4d residual = mean_B - transed_mean_A;

    const double cost = residual.transpose() * inv_cov_B * residual;
    if (!H_target) {
      return cost;
    }

    Eigen::Matrix<double, 4, 6> J_target = Eigen::Matrix<double, 4, 6>::Zero();
    J_target.block<3, 3>(0, 0) = -gtsam::SO3::Hat(transed_mean_A.head<3>());
    J_target.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 4, 6> J_source = Eigen::Matrix<double, 4, 6>::Zero();
    J_source.block<3, 3>(0, 0) = delta.linear() * gtsam::SO3::Hat(mean_A.template head<3>());
    J_source.block<3, 3>(0, 3) = -delta.linear();

    Eigen::Matrix<double, 6, 4> J_target_weighted = J_target.transpose() * inv_cov_B;
    Eigen::Matrix<double, 6, 4> J_source_weighted = J_source.transpose() * inv_cov_B;

    *H_target += J_target_weighted * J_target;
    *H_source += J_source_weighted * J_source;
    *H_target_source += J_target_weighted * J_source;
    *b_target += J_target_weighted * residual;
    *b_source += J_source_weighted * residual;

    return cost;
  };

  if (is_omp_default() || num_threads == 1) {
    return scan_matching_reduce_omp(perpoint_task, frame::size(*source), num_threads, H_target, H_source, H_target_source, b_target, b_source);
  } else {
    return scan_matching_reduce_tbb(perpoint_task, frame::size(*source), H_target, H_source, H_target_source, b_target, b_source);
  }
}

}  // namespace gtsam_points
