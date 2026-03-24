// SPDX-License-Identifier: MIT
// GMM Voxel + GMMVoxelMapCPU — Phase 1
#pragma once

#include <random>
#include <vector>
#include <memory>

#include <Eigen/Core>
#include <gtsam_points/types/point_cloud.hpp>
#include <gtsam_points/types/gaussian_voxelmap.hpp>
#include <gtsam_points/ann/incremental_voxelmap.hpp>

namespace gtsam_points {

/// @brief One Gaussian component of a GMM.
struct GMMComponent {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector4d mean = Eigen::Vector4d::Zero();  ///< homogeneous mean (w=0)
  Eigen::Matrix4d cov = Eigen::Matrix4d::Zero();   ///< 4x4 covariance (only 3x3 used)
  double weight = 0.0;                             ///< mixture weight
};

/// @brief GMM voxel that accumulates raw points via reservoir sampling
///        and fits a K-component GMM on finalize().
struct GMMVoxel {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<GMMVoxel>;
  using ConstPtr = std::shared_ptr<const GMMVoxel>;

  /// @brief Per-voxel configuration, stored in IncrementalVoxelMap::voxel_setting.
  struct Setting {
    int max_components = 3;                ///< Maximum GMM components per voxel
    int max_em_iterations = 20;            ///< EM iteration limit
    double convergence_tol = 1e-4;         ///< EM convergence tolerance
    double covariance_regularization = 1e-3;  ///< Regularization added to covariance diagonal
    double min_weight_threshold = 0.01;    ///< Prune components with weight below this
    int reservoir_capacity = 256;          ///< Maximum points stored per voxel
  };

  /// @brief Default constructor.
  GMMVoxel();

  /// @brief Number of GMM components (K after finalize, 0 before).
  size_t size() const;

  /// @brief Add a point via reservoir sampling (Algorithm R).
  /// @param setting  Voxel insertion setting (contains reservoir_capacity)
  /// @param points   Input point cloud
  /// @param i        Index of the point to add
  void add(const Setting& setting, const PointCloud& points, size_t i);

  /// @brief Fit GMM from reservoir. Phase 1 stub: single-Gaussian fallback.
  void finalize();

  /// @brief KNN search over all GMM component means.
  /// @param pt      Query point (homogeneous 4D)
  /// @param result  KNN result accumulator (push local index 0..K-1)
  template <typename Result>
  void knn_search(const Eigen::Vector4d& pt, Result& result) const {
    for (size_t k = 0; k < components_.size(); k++) {
      Eigen::Vector4d diff = pt - components_[k].mean;
      diff(3) = 0.0;  // Ignore w-component (query w=1, mean w=0 → +1 bias)
      const double sq_dist = diff.squaredNorm();
      result.push(k, sq_dist);
    }
  }

  /// @brief Access finalized GMM components.
  const std::vector<GMMComponent>& components() const { return components_; }

  /// @brief Access raw reservoir points.
  const std::vector<Eigen::Vector4d>& reservoir() const { return reservoir_; }

  /// @brief True if new points added since last finalize.
  bool is_dirty() const { return dirty_; }

  /// @brief Total number of points ever seen by this voxel.
  size_t total_points_seen() const { return total_points_seen_; }

private:
  // Reservoir sampling state
  std::vector<Eigen::Vector4d> reservoir_;
  size_t total_points_seen_ = 0;
  bool dirty_ = false;
  std::mt19937 rng_{42};  ///< Per-voxel RNG for deterministic reservoir sampling

  // Finalized GMM components
  std::vector<GMMComponent> components_;
  Setting cached_setting_;  ///< Snapshot from last finalize
};

// ---------------------------------------------------------------------------
// frame::traits<GMMVoxel> — exposes K component means/covs as "points"
// ---------------------------------------------------------------------------
namespace frame {

template <>
struct traits<GMMVoxel> {
  static int size(const GMMVoxel& v) { return static_cast<int>(v.size()); }

  static bool has_points(const GMMVoxel& v) { return v.size() > 0; }
  static bool has_normals(const GMMVoxel&) { return false; }
  static bool has_covs(const GMMVoxel& v) { return v.size() > 0; }
  static bool has_intensities(const GMMVoxel&) { return false; }

  // Returns by VALUE (not reference) — components live in a vector,
  // and decltype(auto) in frame::point() free function handles this correctly.
  static Eigen::Vector4d point(const GMMVoxel& v, size_t k) { return v.components()[k].mean; }
  static Eigen::Vector4d normal(const GMMVoxel&, size_t) { return Eigen::Vector4d::Zero(); }
  static Eigen::Matrix4d cov(const GMMVoxel& v, size_t k) { return v.components()[k].cov; }
  static double intensity(const GMMVoxel&, size_t) { return 0.0; }
};

}  // namespace frame

// ---------------------------------------------------------------------------
// frame::traits<IncrementalVoxelMap<GMMVoxel>> — returns by VALUE to avoid
// dangling reference (generic traits returns const&, but GMMVoxel traits
// returns temporaries, not references to stored data).
// ---------------------------------------------------------------------------
namespace frame {

template <>
struct traits<IncrementalVoxelMap<GMMVoxel>> {
  static bool has_points(const IncrementalVoxelMap<GMMVoxel>& ivox) { return ivox.has_points(); }
  static bool has_normals(const IncrementalVoxelMap<GMMVoxel>& ivox) { return ivox.has_normals(); }
  static bool has_covs(const IncrementalVoxelMap<GMMVoxel>& ivox) { return ivox.has_covs(); }
  static bool has_intensities(const IncrementalVoxelMap<GMMVoxel>& ivox) { return ivox.has_intensities(); }

  static Eigen::Vector4d point(const IncrementalVoxelMap<GMMVoxel>& ivox, size_t i) { return ivox.point(i); }
  static Eigen::Vector4d normal(const IncrementalVoxelMap<GMMVoxel>& ivox, size_t i) { return ivox.normal(i); }
  static Eigen::Matrix4d cov(const IncrementalVoxelMap<GMMVoxel>& ivox, size_t i) { return ivox.cov(i); }
  static double intensity(const IncrementalVoxelMap<GMMVoxel>& ivox, size_t i) { return ivox.intensity(i); }
};

}  // namespace frame

// ---------------------------------------------------------------------------
// GMMVoxelMapCPU — sibling of GaussianVoxelMapCPU (NOT a subclass)
// ---------------------------------------------------------------------------
class GMMVoxelMapCPU : public GaussianVoxelMap, public IncrementalVoxelMap<GMMVoxel> {
public:
  using Ptr = std::shared_ptr<GMMVoxelMapCPU>;
  using ConstPtr = std::shared_ptr<const GMMVoxelMapCPU>;

  /// @brief Constructor.
  /// @param resolution  Voxel resolution (edge length in meters)
  explicit GMMVoxelMapCPU(double resolution);
  virtual ~GMMVoxelMapCPU();

  /// @brief Voxel resolution.
  virtual double voxel_resolution() const override;

  /// @brief Insert a point cloud frame into the voxelmap.
  virtual void insert(const PointCloud& frame) override;

  /// @brief Save compact (stub — not yet implemented for GMM).
  virtual void save_compact(const std::string& path) const override;

  /// @brief GMM-specific setting accessor.
  GMMVoxel::Setting& gmm_setting() { return voxel_insertion_setting(); }

  /// @brief Compute voxel coordinate from a point.
  Eigen::Vector3i voxel_coord(const Eigen::Vector4d& x) const;

  /// @brief Look up voxel index by coordinate. Returns -1 if not found.
  int lookup_voxel_index(const Eigen::Vector3i& coord) const;

  /// @brief Look up a voxel by index.
  const GMMVoxel& lookup_voxel(int voxel_id) const;

  /// @brief Number of voxels.
  size_t num_voxels() const { return flat_voxels.size(); }
};

// ---------------------------------------------------------------------------
// frame::traits<GMMVoxelMapCPU>
// ---------------------------------------------------------------------------
namespace frame {

template <>
struct traits<GMMVoxelMapCPU> {
  static bool has_points(const GMMVoxelMapCPU& ivox) { return ivox.has_points(); }
  static bool has_normals(const GMMVoxelMapCPU& ivox) { return ivox.has_normals(); }
  static bool has_covs(const GMMVoxelMapCPU& ivox) { return ivox.has_covs(); }
  static bool has_intensities(const GMMVoxelMapCPU& ivox) { return ivox.has_intensities(); }

  static decltype(auto) point(const GMMVoxelMapCPU& ivox, size_t i) { return ivox.point(i); }
  static decltype(auto) normal(const GMMVoxelMapCPU& ivox, size_t i) { return ivox.normal(i); }
  static decltype(auto) cov(const GMMVoxelMapCPU& ivox, size_t i) { return ivox.cov(i); }
  static decltype(auto) intensity(const GMMVoxelMapCPU& ivox, size_t i) { return ivox.intensity(i); }
};

}  // namespace frame

}  // namespace gtsam_points
