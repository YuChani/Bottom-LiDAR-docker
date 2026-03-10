// SPDX-License-Identifier: MIT
// Copyright (c) 2026

#pragma once

#include <memory>
#include <vector>
#include <Eigen/Core>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <gtsam_points/util/gtsam_migration.hpp>
#include <gtsam_points/types/point_cloud.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/factors/integrated_matching_cost_factor.hpp>
#include <gtsam_points/factors/integrated_ndt_factor.hpp>

namespace gtsam_points {

struct GmmComponentMatch {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector4d mean;
  Eigen::Matrix4d inv_cov;
  double log_det_cov;
  double prior;
  double gamma;
  bool valid;

  GmmComponentMatch()
  : mean(Eigen::Vector4d::Zero()),
    inv_cov(Eigen::Matrix4d::Zero()),
    log_det_cov(0.0),
    prior(0.0),
    gamma(0.0),
    valid(false) {}
};

struct GmmPointCorrespondence {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::vector<GmmComponentMatch, Eigen::aligned_allocator<GmmComponentMatch>> components;
  bool valid;

  GmmPointCorrespondence() : valid(false) {}
};

template <typename SourceFrame = gtsam_points::PointCloud>
class IntegratedGMMNDTFactor_ : public gtsam_points::IntegratedMatchingCostFactor {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = gtsam_points::shared_ptr<IntegratedGMMNDTFactor_>;

  IntegratedGMMNDTFactor_(
    gtsam::Key target_key,
    gtsam::Key source_key,
    const GaussianVoxelMap::ConstPtr& target_voxels,
    const std::shared_ptr<const SourceFrame>& source);

  IntegratedGMMNDTFactor_(
    const gtsam::Pose3& fixed_target_pose,
    gtsam::Key source_key,
    const GaussianVoxelMap::ConstPtr& target_voxels,
    const std::shared_ptr<const SourceFrame>& source);

  virtual ~IntegratedGMMNDTFactor_() override;

  virtual void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override;
  virtual size_t memory_usage() const override;

  void set_num_threads(int n) { num_threads = n; }
  void set_resolution(double res) { resolution = res; }
  void set_outlier_ratio(double ratio) { outlier_ratio = ratio; }
  void set_regularization_epsilon(double eps) {
    regularization_epsilon = eps;
    inv_cov_cached = false;
  }
  void set_correspondence_update_tolerance(double angle, double trans) {
    correspondence_update_tolerance_rot = angle;
    correspondence_update_tolerance_trans = trans;
  }
  void set_search_mode(NDTSearchMode mode) { search_mode = mode; }
  void set_num_components(int k) { num_components = std::max(1, k); }
  void set_gamma_min(double gmin) { gamma_min = std::max(0.0, gmin); }
  void set_freeze_mixture_in_lm(bool freeze) { freeze_mixture_in_lm = freeze; }
  void set_use_uniform_outlier(bool use_uniform, double weight = 1e-4) {
    use_uniform_outlier = use_uniform;
    uniform_outlier_weight = std::max(0.0, weight);
  }

  int num_inliers() const {
    int count = 0;
    for (const auto& c : correspondences) {
      if (c.valid) {
        count++;
      }
    }
    return count;
  }

  double inlier_fraction() const {
    return correspondences.empty() ? 0.0 : num_inliers() / static_cast<double>(correspondences.size());
  }

  const std::shared_ptr<const GaussianVoxelMapCPU>& get_target() const { return target_voxels; }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new IntegratedGMMNDTFactor_(*this));
  }

private:
  virtual void update_correspondences(const Eigen::Isometry3d& delta) const override;

  virtual double evaluate(
    const Eigen::Isometry3d& delta,
    Eigen::Matrix<double, 6, 6>* H_target = nullptr,
    Eigen::Matrix<double, 6, 6>* H_source = nullptr,
    Eigen::Matrix<double, 6, 6>* H_target_source = nullptr,
    Eigen::Matrix<double, 6, 1>* b_target = nullptr,
    Eigen::Matrix<double, 6, 1>* b_source = nullptr) const override;

  void compute_responsibilities(const Eigen::Vector4d& transformed_point, GmmPointCorrespondence* correspondence) const;

private:
  int num_threads;
  double resolution;
  double outlier_ratio;
  double regularization_epsilon;
  NDTSearchMode search_mode;

  int num_components;
  double gamma_min;
  bool freeze_mixture_in_lm;
  bool use_uniform_outlier;
  double uniform_outlier_weight;

  double correspondence_update_tolerance_rot;
  double correspondence_update_tolerance_trans;

  mutable double gauss_d1;
  mutable double gauss_d2;

  mutable Eigen::Isometry3d linearization_point;
  mutable Eigen::Isometry3d last_correspondence_point;
  mutable std::vector<GmmPointCorrespondence, Eigen::aligned_allocator<GmmPointCorrespondence>> correspondences;

  mutable std::vector<Eigen::Matrix4d> inv_cov_cache;
  mutable std::vector<double> log_det_cov_cache;
  mutable bool inv_cov_cached;

  std::shared_ptr<const GaussianVoxelMapCPU> target_voxels;
  std::shared_ptr<const SourceFrame> source;
};

using IntegratedGMMNDTFactor = IntegratedGMMNDTFactor_<>;

}  // namespace gtsam_points
