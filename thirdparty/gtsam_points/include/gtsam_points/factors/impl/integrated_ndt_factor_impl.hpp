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
  resolution(1.0),    // NDT에서 사용하는 voxel의 해상도. 0.5m ~ 1.0m 사이로 설정
  outlier_ratio(0.1),  // outlier 비율. 낮을수록 gradient가 커져 수렴 속도 향상. 0.1 = 10% outlier 가정
  regularization_epsilon(1e-3),
  search_mode(NDTSearchMode::DIRECT7),
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
IntegratedNDTFactor_<SourceFrame>::IntegratedNDTFactor_(
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
IntegratedNDTFactor_<SourceFrame>::~IntegratedNDTFactor_() {}

template <typename SourceFrame>
void IntegratedNDTFactor_<SourceFrame>::print(const std::string& s, const gtsam::KeyFormatter& keyFormatter) const {
  std::cout << s << "IntegratedNDTFactor";
  if (is_binary) {
    std::cout << "(" << keyFormatter(this->keys()[0]) << ", " << keyFormatter(this->keys()[1]) << ")" << std::endl;
  } else {
    std::cout << "(fixed, " << keyFormatter(this->keys()[0]) << ")" << std::endl;
  }
  std::cout << "target_resolution=" << target_voxels->voxel_resolution()
            << ", |source|=" << frame::size(*source) << "pts" << std::endl;
}

template <typename SourceFrame>
size_t IntegratedNDTFactor_<SourceFrame>::memory_usage() const {
  return sizeof(*this) + sizeof(NdtCorrespondence) * correspondences.capacity();
}

template <typename SourceFrame>
void IntegratedNDTFactor_<SourceFrame>::update_correspondences(const Eigen::Isometry3d& delta) const {
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
  // compute_ndt_params : intergrated_ndt_factor.hpp에서 선언됨
  // c1 : 인라이어 Gaussian 스케일상수 / c2 : 아웃라이어 분포항 
  // d1 : 최종 score 진폭 / d2 : Mahalanobis 거리 감소기울기 
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
        correspondences[i].one_over_Z = std::sqrt((2 * M_PI * M_PI * M_PI) * std::abs(best_voxel->cov.determinant())); // empirically determined for typical LIDAR covariances
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
double IntegratedNDTFactor_<SourceFrame>::evaluate(
  const Eigen::Isometry3d& delta,
  Eigen::Matrix<double, 6, 6>* H_target,
  Eigen::Matrix<double, 6, 6>* H_source,
  Eigen::Matrix<double, 6, 6>* H_target_source,
  Eigen::Matrix<double, 6, 1>* b_target,
  Eigen::Matrix<double, 6, 1>* b_source) const {

  if (correspondences.size() != frame::size(*source)) {
    update_correspondences(delta);
  }

  const auto perpoint_task = [&](
                                int i,
                                Eigen::Matrix<double, 6, 6>* H_target,
                                Eigen::Matrix<double, 6, 6>* H_source,
                                Eigen::Matrix<double, 6, 6>* H_target_source,
                                Eigen::Matrix<double, 6, 1>* b_target,
                                Eigen::Matrix<double, 6, 1>* b_source) {
    const auto& corr = correspondences[i];
    if (!corr.valid) 
    {
      return 0.0;
    }

    // SWAN
    double d1, d2;
    compute_ndt_params(resolution, outlier_ratio, corr.one_over_Z, d1, d2);

    // SWAN
    // mean_A: p_i (source point)
    // mean_B: mu_i (target point)
    // inv_cov_B: Σ_i^{-1} (target inverse covariance)
    const auto& mean_A = frame::point(*source, i);  // source point (a_i)
    const auto& mean_B = corr.mean;                 // target point (mu_i)
    const auto& inv_cov_B = corr.inv_cov;          // inverse covariance of target point

    // SWAN
    // transed_mean_A: p_i' = R * p_i + t (transformed source point)
    Eigen::Vector4d transed_mean_A = delta * mean_A;  // 변형된 source : q_i = T*a_i

    // residual = target_mean - transformed_source_point (GTSAM convention)
    // SWAN
    // residual: e_i = mu_i - p_i'
    Eigen::Vector4d residual = mean_B - transed_mean_A; // target과 변형된 source의 차이 벡터 : r_i = mu_i - q_i

    // Mahalanobis 거리(4x4행렬)
    double mahalanobis_dist = residual.transpose() * inv_cov_B * residual;

    //   d1 < 0 이므로 -d1 > 0, 정렬이 좋을수록 score가 크다 (최대화 문제).
    // SWAN: -d2/2.0을 미리 계산해 놓자.
    double exponent = -d2 * mahalanobis_dist / 2.0;
    //double exponent = -gauss_d2 * mahalanobis_dist / 2.0;
    if (exponent < -700.0) 
    {
      return 0.0;  // underflow 방지
    }
    
    double e_term = std::exp(exponent);  // exp(-d2/2 * m), 범위: (0, 1]
    if (std::isnan(e_term)) 
    {
      return 0.0;
    }

    // Magnusson Eq. 6.9 원본 score function
    const double cost = d1 * e_term; // d1 < 0, 음수
    //const double cost = gauss_d1 * e_term; // gauss_d1 < 0, 음수
    //const double score_function = -cost;  // 양수

    if (!H_target) 
    {
      return cost;
    }

    // 변환 포즈에 대한 residual의 자코비안 (4x6, SE(3) Lie algebra 기반)
    Eigen::Matrix<double, 4, 6> J_target = Eigen::Matrix<double, 4, 6>::Zero();
    J_target.block<3, 3>(0, 0) = -gtsam::SO3::Hat(transed_mean_A.head<3>());  // rotation
    J_target.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();                 // translation

    Eigen::Matrix<double, 4, 6> J_source = Eigen::Matrix<double, 4, 6>::Zero();
    J_source.block<3, 3>(0, 0) = delta.linear() * gtsam::SO3::Hat(mean_A.template head<3>());
    J_source.block<3, 3>(0, 3) = -delta.linear();

    const double weight = -d1 * d2 * e_term;
    //const double weight = -gauss_d1 * gauss_d2 * e_term;

    //  Gauss-Newton 근사 Hessian (Magnusson Eq. 6.13의 H1 항) 
    // H ≈ weight * J^T * Σ^{-1} * J   (H2, H3 항 생략 → PSD 보장)
    *H_target += weight * J_target.transpose() * inv_cov_B * J_target;
    *H_source += weight * J_source.transpose() * inv_cov_B * J_source;
    *H_target_source += weight * J_target.transpose() * inv_cov_B * J_source;
    // b = weight * J^T * Σ^{-1} * q    (= gradient)
    *b_target += weight * J_target.transpose() * inv_cov_B * residual;
    *b_source += weight * J_source.transpose() * inv_cov_B * residual;

    return cost;
  };

  if (is_omp_default() || num_threads == 1) 
  {
    return scan_matching_reduce_omp(perpoint_task, frame::size(*source), num_threads, H_target, H_source, H_target_source, b_target, b_source);
  } 
  else 
  {
    return scan_matching_reduce_tbb(perpoint_task, frame::size(*source), H_target, H_source, H_target_source, b_target, b_source);
  }
}

}  // namespace gtsam_points