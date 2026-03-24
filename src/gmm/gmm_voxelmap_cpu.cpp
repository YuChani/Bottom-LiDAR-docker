// SPDX-License-Identifier: MIT
#include "gmm/gmm_voxelmap_cpu.hpp"

#include <iostream>
#include <algorithm>
#include <numeric>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <gtsam_points/util/fast_floor.hpp>
#include <gtsam_points/ann/incremental_voxelmap.hpp>
#include <gtsam_points/ann/impl/incremental_voxelmap_impl.hpp>

namespace gtsam_points {

template class IncrementalVoxelMap<GMMVoxel>;

GMMVoxel::GMMVoxel() : total_points_seen_(0), dirty_(false), rng_(42) {}

size_t GMMVoxel::size() const {
  return components_.size();
}

void GMMVoxel::add(const Setting& setting, const PointCloud& points, size_t i) {
  cached_setting_ = setting;
  total_points_seen_++;
  const int capacity = setting.reservoir_capacity;

  if (static_cast<int>(reservoir_.size()) < capacity) {
    reservoir_.push_back(points.points[i]);
  } else {
    std::uniform_int_distribution<size_t> dist(0, total_points_seen_ - 1);
    const size_t j = dist(rng_);
    if (j < static_cast<size_t>(capacity)) {
      reservoir_[j] = points.points[i];
    }
  }

  dirty_ = true;
}

void GMMVoxel::finalize() {
  if (!dirty_) {
    return;
  }

  if (reservoir_.empty()) {
    components_.clear();
    dirty_ = false;
    return;
  }

  // Phase 1 stub: single-Gaussian fallback from reservoir points
  const size_t N = reservoir_.size();

  Eigen::Vector4d mean = Eigen::Vector4d::Zero();
  for (const auto& pt : reservoir_) {
    mean += pt;
  }
  mean /= static_cast<double>(N);

  Eigen::Matrix4d cov = Eigen::Matrix4d::Zero();
  for (const auto& pt : reservoir_) {
    const Eigen::Vector4d diff = pt - mean;
    cov += diff * diff.transpose();
  }
  if (N > 1) {
    cov /= static_cast<double>(N);
  }

  // Regularize 3x3 block to ensure positive definiteness
  cov.block<3, 3>(0, 0) += cached_setting_.covariance_regularization * Eigen::Matrix3d::Identity();
  // Ensure 4th row/col stays zero (homogeneous coordinate)
  cov.row(3).setZero();
  cov.col(3).setZero();

  GMMComponent comp;
  comp.mean = mean;
  comp.cov = cov;
  comp.weight = 1.0;

  components_.clear();
  components_.push_back(comp);

  dirty_ = false;
}

GMMVoxelMapCPU::GMMVoxelMapCPU(double resolution) : IncrementalVoxelMap<GMMVoxel>(resolution) {
  offsets = neighbor_offsets(1);
}

GMMVoxelMapCPU::~GMMVoxelMapCPU() {}

double GMMVoxelMapCPU::voxel_resolution() const {
  return leaf_size();
}

void GMMVoxelMapCPU::insert(const PointCloud& frame) {
  IncrementalVoxelMap<GMMVoxel>::insert(frame);
}

void GMMVoxelMapCPU::save_compact(const std::string& path) const {
  std::cerr << "warning: save_compact not yet implemented for GMMVoxelMapCPU" << std::endl;
}

Eigen::Vector3i GMMVoxelMapCPU::voxel_coord(const Eigen::Vector4d& x) const {
  return fast_floor(x * inv_leaf_size).head<3>();
}

int GMMVoxelMapCPU::lookup_voxel_index(const Eigen::Vector3i& coord) const {
  auto found = voxels.find(coord);
  if (found == voxels.end()) {
    return -1;
  }
  return found->second;
}

const GMMVoxel& GMMVoxelMapCPU::lookup_voxel(int voxel_id) const {
  return flat_voxels[voxel_id]->second;
}

}  // namespace gtsam_points
