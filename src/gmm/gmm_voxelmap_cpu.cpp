// SPDX-License-Identifier: MIT
#include "gmm/gmm_voxelmap_cpu.hpp"
#include "gmm/mixture_em_backend.hpp"

#include <iostream>
#include <algorithm>
#include <numeric>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <gtsam_points/util/fast_floor.hpp>
#include <gtsam_points/ann/incremental_voxelmap.hpp>
#include <gtsam_points/ann/impl/incremental_voxelmap_impl.hpp>

namespace gtsam_points {

// IncrementalVoxelMap<GMMVoxel>의 명시적 템플릿 인스턴스화:
// 링커가 복셀맵 삽입/검색 코드를 이 TU에 생성하도록 강제.
template class IncrementalVoxelMap<GMMVoxel>;

// 기본 생성자: 저수지 비어있음, 컴포넌트 없음, dirty=false
GMMVoxel::GMMVoxel()
: total_points_seen_(0), dirty_(false), rng_(42)
{
}

// 피팅된 GMM 컴포넌트 수 반환 (finalize 전: 0)
size_t GMMVoxel::size() const
{
  return components_.size();
}

// Vitter의 Algorithm R (저수지 샘플링)으로 포인트 하나를 복셀에 추가.
// 전체 스트림에서 capacity개의 균일 무작위 표본을 유지하는 온라인 알고리즘.
void GMMVoxel::add(const Setting& setting, const PointCloud& points, size_t i)
{
  cached_setting_ = setting;  // 설정 스냅샷 저장 (finalize에서 사용)
  total_points_seen_++;       // 관측 카운터 N 증가
  const int capacity = setting.reservoir_capacity;

  // 동차 좌표 w=0으로 강제: 평균/공분산 계산 시 w-leakage 방지
  Eigen::Vector4d pt = points.points[i];
  pt(3) = 0.0;

  if (static_cast<int>(reservoir_.size()) < capacity) {
    // 저수지 미만: 무조건 삽입 (채우기 단계)
    reservoir_.push_back(pt);
  } else {
    // 저수지 가득: 확률 C/N으로 기존 원소를 교체 (Algorithm R 핵심)
    std::uniform_int_distribution<size_t> dist(0, total_points_seen_ - 1);
    const size_t j = dist(rng_);  // [0, N-1] 균일 분포에서 j 추출
    if (j < static_cast<size_t>(capacity)) {
      reservoir_[j] = pt;  // j < C이면 교체, 아니면 버림
    }
  }

  dirty_ = true;  // finalize 필요 플래그 설정
}

// 저수지 포인트로부터 GMM 피팅 실행.
// dirty_==false이면 불필요한 재피팅 방지 (멱등성).
void GMMVoxel::finalize()
{
  if (!dirty_)
  {
    return;  // 새 포인트 없으면 스킵 (멱등성 보장)
  }

  if (reservoir_.empty())
  {
    components_.clear();  // 빈 저수지 → 빈 컴포넌트
    dirty_ = false;
    return;
  }

  // GMMVoxel::Setting → GMMFitParams 변환
  GMMFitParams params;
  params.max_components = cached_setting_.max_components;
  params.max_em_iterations = cached_setting_.max_em_iterations;
  params.convergence_tol = cached_setting_.convergence_tol;
  params.covariance_regularization = cached_setting_.covariance_regularization;
  params.min_weight_threshold = cached_setting_.min_weight_threshold;

  GMMFitResult result;
  if (components_.empty())
  {
    // Cold-start: 기존 컴포넌트 없음 → random_spread 초기화로 EM 실행
    result = fit_gmm(reservoir_, params);
  }
  else
  {
    // Warm-start: 이전 컴포넌트를 초기값으로 사용 → keep_existing EM 실행
    // 포인트 분포가 점진적으로 변할 때 수렴 속도 향상
    result = fit_gmm(reservoir_, params, components_);
  }

  components_ = std::move(result.components);  // 결과 이동 할당
  dirty_ = false;  // 피팅 완료, 더 이상 dirty 아님
}

// GMMVoxelMapCPU 생성자: 복셀 해상도 설정 및 이웃 오프셋 초기화
GMMVoxelMapCPU::GMMVoxelMapCPU(double resolution)
: IncrementalVoxelMap<GMMVoxel>(resolution)
{
  // neighbor_offsets(1): 반경 1 내 이웃 복셀 오프셋 (KNN 검색용)
  offsets = neighbor_offsets(1);
}

GMMVoxelMapCPU::~GMMVoxelMapCPU()
{
}

// leaf_size()는 IncrementalVoxelMap에서 상속받은 복셀 해상도
double GMMVoxelMapCPU::voxel_resolution() const
{
  return leaf_size();
}

// 지연 finalize 패턴: Phase1(포인트 누적) + Phase2(LRU) 수행, Phase3(finalize) 생략.
// GMMVoxel EM 비용 O(K×N×D²×iters)이 GaussianVoxel O(N)과 달라 매 insert마다
// 전체 finalize 시 ~35분 병목 발생 → knn_search() 시점에 1회 일괄 수행.
void GMMVoxelMapCPU::insert(const PointCloud& frame)
{
  for (size_t i = 0; i < frame.size(); i++)
  {
    const Eigen::Vector3i coord = fast_floor(frame.points[i] * inv_leaf_size).template head<3>();

    auto found = voxels.find(coord);
    if (found == voxels.end())
    {
      auto voxel = std::make_shared<std::pair<VoxelInfo, GMMVoxel>>(VoxelInfo(coord, lru_counter), GMMVoxel());
      found = voxels.emplace_hint(found, coord, flat_voxels.size());
      flat_voxels.emplace_back(voxel);
    }

    auto& [info, voxel] = *flat_voxels[found->second];
    info.lru = lru_counter;
    voxel.add(voxel_setting, frame, i);
  }

  if ((++lru_counter) % lru_clear_cycle == 0)
  {
    auto remove_counter = std::remove_if(
      flat_voxels.begin(), flat_voxels.end(),
      [&](const std::shared_ptr<std::pair<VoxelInfo, GMMVoxel>>& voxel)
      {
        return voxel->first.lru + lru_horizon < lru_counter;
      });
    flat_voxels.erase(remove_counter, flat_voxels.end());

    voxels.clear();
    for (size_t i = 0; i < flat_voxels.size(); i++)
    {
      voxels[flat_voxels[i]->first.coord] = i;
    }
  }

  needs_finalize_ = true;
}

void GMMVoxelMapCPU::save_compact(const std::string& path) const
{
  std::cerr << "warning: save_compact not yet implemented for GMMVoxelMapCPU" << std::endl;
}

// 4D 동차 좌표를 3D 정수 복셀 좌표로 변환.
// fast_floor: SIMD 최적화된 바닥 함수. inv_leaf_size = 1/resolution.
Eigen::Vector3i GMMVoxelMapCPU::voxel_coord(const Eigen::Vector4d& x) const
{
  return fast_floor(x * inv_leaf_size).head<3>();
}

// 해시맵(voxels)에서 3D 정수 좌표로 flat 인덱스 검색.
// 존재하지 않으면 -1 반환. O(1) 평균 시간 복잡도.
int GMMVoxelMapCPU::lookup_voxel_index(const Eigen::Vector3i& coord) const
{
  auto found = voxels.find(coord);
  if (found == voxels.end())
  {
    return -1;
  }
  return found->second;  // flat_voxels 벡터의 인덱스
}

// flat_voxels[voxel_id]의 second(= GMMVoxel 참조)를 반환.
// flat_voxels는 해시맵 이터레이터를 보관하는 벡터로, O(1) 인덱스 접근 제공.
const GMMVoxel& GMMVoxelMapCPU::lookup_voxel(int voxel_id) const
{
  return flat_voxels[voxel_id]->second;
}

// dirty 복셀 일괄 EM 피팅. OpenMP로 복셀 간 독립 병렬 실행.
void GMMVoxelMapCPU::finalize_all()
{
  if (!needs_finalize_)
  {
    return;
  }

  const int n = static_cast<int>(flat_voxels.size());
#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < n; i++)
  {
    flat_voxels[i]->second.finalize();
  }

  needs_finalize_ = false;
}

size_t GMMVoxelMapCPU::knn_search(
  const double* pt, size_t k, size_t* k_indices, double* k_sq_dists, double max_sq_dist) const {
  if (needs_finalize_)
  {
    const_cast<GMMVoxelMapCPU*>(this)->finalize_all();
  }
  return IncrementalVoxelMap<GMMVoxel>::knn_search(pt, k, k_indices, k_sq_dists, max_sq_dist);
}

}  // namespace gtsam_points
