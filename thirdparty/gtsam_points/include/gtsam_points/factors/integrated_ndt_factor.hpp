// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
//
// NDT (Normal Distributions Transform) matching cost factor
// Based on Magnusson 2009, integrated into GTSAM via IntegratedMatchingCostFactor.
//
// Bug fixes applied (vs original skeleton):
//   1. Negated derivative_scale to make Hessian positive semi-definite
//   2. Flipped residual direction to match GTSAM convention (target - source)
//   3. Use NdtCorrespondence with cached inv_cov instead of raw GaussianVoxel*

#pragma once

#include <cmath>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <gtsam_points/util/gtsam_migration.hpp>
#include <gtsam_points/types/point_cloud.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/factors/integrated_matching_cost_factor.hpp>

namespace gtsam_points {

/// @brief Cached NDT correspondence data extracted from a GaussianVoxel.
///        Holds voxel mean and regularized inverse covariance for NDT evaluation.
struct NdtCorrespondence {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector4d mean;      ///< Voxel mean (4D homogeneous)
  Eigen::Matrix4d inv_cov;   ///< Regularized inverse covariance (4x4, only top-left 3x3 is meaningful)
  bool valid;                 ///< Whether this correspondence is valid

  NdtCorrespondence() : mean(Eigen::Vector4d::Zero()), inv_cov(Eigen::Matrix4d::Zero()), valid(false) {}
};

/// @brief Compute regularized inverse covariance from a 4x4 covariance matrix.
/// @param cov                    4x4 covariance matrix
/// @param regularization_epsilon Regularization parameter (default 1e-3). Small eigenvalues
///                               are clamped to epsilon * lambda_max.
/// @return Regularized inverse covariance (4x4)
inline Eigen::Matrix4d compute_ndt_inverse_covariance(const Eigen::Matrix4d& cov, double regularization_epsilon = 1e-3) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(cov);
  Eigen::Vector4d eigenvalues = solver.eigenvalues();
  Eigen::Matrix4d eigenvectors = solver.eigenvectors();

  double lambda_max = eigenvalues.maxCoeff();
  Eigen::Vector4d clamped = eigenvalues.array().max(regularization_epsilon * lambda_max).matrix();

  Eigen::Matrix4d cov_reg = eigenvectors * clamped.asDiagonal() * eigenvectors.transpose();
  return cov_reg.inverse();
}

/// @brief Compute NDT Gaussian parameters d1, d2 from resolution and outlier ratio.
///        Based on Magnusson 2009, Equations 6.9-6.10.
///
///        The NDT score for a single point is:
///          s(x) = -d1 * exp(-d2/2 * x^T * Sigma^{-1} * x)
///        where d1 < 0 (so -d1 > 0, making the score positive when points align).
///
/// @param resolution    Voxel resolution (meters)
/// @param outlier_ratio Fraction of points assumed to be outliers (0.0 to 1.0)
/// @param[out] d1       Output Gaussian parameter d1 (negative)
/// @param[out] d2       Output Gaussian parameter d2 (positive)
inline void compute_ndt_params(double resolution, double outlier_ratio, double& d1, double& d2) {
  double c1 = 10.0 * (1.0 - outlier_ratio);
  double c2 = outlier_ratio / (resolution * resolution * resolution);
  double d3 = -std::log(c2);
  d1 = -std::log(c1 + c2) - d3;
  d2 = -2.0 * std::log((-std::log(c1 * std::exp(-0.5) + c2) - d3) / d1);
}

/// NDT correspondence search mode
enum class NDTSearchMode {
  DIRECT1,   ///< Current voxel only (fastest)
  DIRECT7,   ///< Current voxel + 6 face neighbors (balanced, default)
  DIRECT27   ///< Current voxel + 26 neighbors (slowest, most accurate)
};

/**
 * @brief NDT (Normal Distributions Transform) matching cost factor
 *        Biber & Strasser, "The Normal Distributions Transform: A New Approach to Laser Scan Matching", IROS2003
 *        Magnusson, "The Three-Dimensional Normal-Distributions Transform", PhD thesis, 2009
 *
 *        Uses NdtCorrespondence to cache regularized inverse covariance per correspondence,
 *        since GaussianVoxel does not store inv_cov.
 */
template <typename SourceFrame = gtsam_points::PointCloud>
class IntegratedNDTFactor_ : public gtsam_points::IntegratedMatchingCostFactor {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = gtsam_points::shared_ptr<IntegratedNDTFactor_>;

  /**
   * @brief Create a binary NDT factor between target and source poses.
   * @param target_key          Target key
   * @param source_key          Source key
   * @param target_voxels       Target voxelmap
   * @param source              Source point cloud frame
   */
  IntegratedNDTFactor_(
    gtsam::Key target_key,
    gtsam::Key source_key,
    const GaussianVoxelMap::ConstPtr& target_voxels,
    const std::shared_ptr<const SourceFrame>& source);

  /**
   * @brief Create a unary NDT factor between a fixed target pose and an active source pose.
   * @param fixed_target_pose   Fixed target pose
   * @param source_key          Source key
   * @param target_voxels       Target voxelmap
   * @param source              Source point cloud frame
   */
  IntegratedNDTFactor_(
    const gtsam::Pose3& fixed_target_pose,
    gtsam::Key source_key,
    const GaussianVoxelMap::ConstPtr& target_voxels,
    const std::shared_ptr<const SourceFrame>& source);

  virtual ~IntegratedNDTFactor_() override;

  /// @brief Print the factor information.
  virtual void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override;

  /**
   * @brief  Calculate the memory usage of this factor
   * @note   The result is approximate and does not account for objects not owned by this factor (e.g., point clouds)
   * @return Memory usage in bytes (Approximate size in bytes)
   */
  virtual size_t memory_usage() const override;

  /// @brief Set the number of thread used for linearization of this factor.
  /// @note If your GTSAM is built with TBB, linearization is already multi-threaded
  ///       and setting n>1 can rather affect the processing speed.
  void set_num_threads(int n) { num_threads = n; }

  /// @brief Set the voxel resolution for NDT correspondence search.
  void set_resolution(double res) { resolution = res; }

  /// @brief Set the outlier ratio for computing NDT Gaussian parameters (d1, d2).
  /// @note Default is 0.55 (55% outliers assumed)
  void set_outlier_ratio(double ratio) { outlier_ratio = ratio; }

  /// @brief Set the regularization epsilon for covariance matrix regularization.
  /// @note Default is 1e-3. Smaller eigenvalues will be clamped to epsilon * lambda_max
  void set_regularization_epsilon(double eps) {
    regularization_epsilon = eps;
    inv_cov_cached = false;  // invalidate cache when epsilon changes
  }

  /// @brief Correspondences are updated only when the displacement from the last update point is larger than these threshold values.
  /// @note  Default values are angle=trans=0 and correspondences are updated every linearization call.
  void set_correspondence_update_tolerance(double angle, double trans) {
    correspondence_update_tolerance_rot = angle;
    correspondence_update_tolerance_trans = trans;
  }

  /// @brief Set the NDT correspondence search mode (DIRECT1/7/27).
  void set_search_mode(NDTSearchMode mode) { search_mode = mode; }

  /// @brief  Get the number of inlier points.
  /// @note   This function must be called after the factor is linearized.
  int num_inliers() const {
    int count = 0;
    for (const auto& c : correspondences) {
      if (c.valid) count++;
    }
    return count;
  }

  /// @brief Compute the fraction of inlier points that have correspondences fell in a voxel.
  /// @note  This function must be called after the factor is linearized.
  double inlier_fraction() const {
    return correspondences.empty() ? 0.0 : num_inliers() / static_cast<double>(correspondences.size());
  }

  /// @brief Get the target voxelmap.
  const std::shared_ptr<const GaussianVoxelMapCPU>& get_target() const { return target_voxels; }

  gtsam::NonlinearFactor::shared_ptr clone() const override { return gtsam::NonlinearFactor::shared_ptr(new IntegratedNDTFactor_(*this)); }

private:
  virtual void update_correspondences(const Eigen::Isometry3d& delta) const override;

  virtual double evaluate(
    const Eigen::Isometry3d& delta,
    Eigen::Matrix<double, 6, 6>* H_target = nullptr,
    Eigen::Matrix<double, 6, 6>* H_source = nullptr,
    Eigen::Matrix<double, 6, 6>* H_target_source = nullptr,
    Eigen::Matrix<double, 6, 1>* b_target = nullptr,
    Eigen::Matrix<double, 6, 1>* b_source = nullptr) const override;

private:
  int num_threads;
  double resolution;
  double outlier_ratio;
  double regularization_epsilon;
  NDTSearchMode search_mode;

  double correspondence_update_tolerance_rot;
  double correspondence_update_tolerance_trans;

  mutable double gauss_d1;
  mutable double gauss_d2;

  mutable Eigen::Isometry3d linearization_point;
  mutable Eigen::Isometry3d last_correspondence_point;
  mutable std::vector<NdtCorrespondence> correspondences;

  mutable std::vector<Eigen::Matrix4d> inv_cov_cache;
  mutable bool inv_cov_cached;

  std::shared_ptr<const GaussianVoxelMapCPU> target_voxels;
  std::shared_ptr<const SourceFrame> source;
};

using IntegratedNDTFactor = IntegratedNDTFactor_<>;

}  // namespace gtsam_points
