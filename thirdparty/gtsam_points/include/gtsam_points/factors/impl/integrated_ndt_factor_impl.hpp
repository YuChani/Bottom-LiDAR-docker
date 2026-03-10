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

    const auto& mean_A = frame::point(*source, i);  // source point (a_i)
    const auto& mean_B = corr.mean;                 // target point (mu_i)
    const auto& inv_cov_B = corr.inv_cov;          // inverse covariance of target point

    Eigen::Vector4d transed_mean_A = delta * mean_A;  // 변형된 source : q_i = T*a_i

    // residual = target_mean - transformed_source_point (GTSAM convention)
    Eigen::Vector4d residual = mean_B - transed_mean_A; // target과 변형된 source의 차이 벡터 : r_i = mu_i - q_i

    // Mahalanobis 거리(4x4행렬)
    double mahalanobis_dist = residual.transpose() * inv_cov_B * residual;

    //   d1 < 0 이므로 -d1 > 0, 정렬이 좋을수록 score가 크다 (최대화 문제).
    double exponent = -gauss_d2 * mahalanobis_dist / 2.0;
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
    const double score_function = -gauss_d1 * e_term;  // = |d1| * exp(...), 양수
    // gauss_d1을 묶어서 정리하면 (-gauss_d1)(1 - e_term) 형태가 됨.
    // e_term = 1일때 cost = 0(정합 잘됨) / e_term = 0일때 cost = gauss_d1값으로 최대값 (정합 안됨)
    const double cost = -gauss_d1 - score_function;  // gauss_d1 약 -6.9

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
    // weight는 정합잘된점 = w가 큼, 정합안된점 = w가 작음 -> robust하게함
    const double weight = -gauss_d1 * gauss_d2 * e_term;

    //  Gauss-Newton 근사 Hessian (Magnusson Eq. 6.13의 H1 항) 
    // H ≈ weight * J^T * Σ^{-1} * J   (H2, H3 항 생략 → PSD 보장)
    *H_target += weight * J_target.transpose() * inv_cov_B * J_target;
    *H_source += weight * J_source.transpose() * inv_cov_B * J_source;
    *H_target_source += weight * J_target.transpose() * inv_cov_B * J_source;
    // b = weight * J^T * Σ^{-1} * q    (= gradient)
    *b_target += weight * J_target.transpose() * inv_cov_B * residual;  // gradient 역할 벡터
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
