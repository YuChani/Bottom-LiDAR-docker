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
// GMM 혼합 모델의 단일 가우시안 컴포넌트.
// 각 복셀 내 K개 컴포넌트 중 하나로, mean(μ_k), cov(Σ_k), weight(π_k)를 보유.
struct GMMComponent
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector4d mean = Eigen::Vector4d::Zero();  ///< homogeneous mean (w=0)
  // 동차 좌표 평균: (x, y, z, 0). w=0으로 설정하여 4x4 연산에서 차원 일관성 유지
  Eigen::Matrix4d cov = Eigen::Matrix4d::Zero();   ///< 4x4 covariance (only 3x3 used)
  // 4x4 공분산 행렬: 상위-좌측 3x3 블록만 유효, 나머지 행/열은 0 (동차 좌표 패딩)
  double weight = 0.0;                             ///< mixture weight
  // 혼합 가중치 π_k ∈ [0,1], Σ_k π_k = 1. EM 알고리즘의 책임도(responsibility)로부터 추정
};

/// @brief GMM voxel that accumulates raw points via reservoir sampling
///        and fits a K-component GMM on finalize().
// 복셀 단위 GMM: 포인트를 저수지 샘플링(Algorithm R)으로 수집한 뒤,
// finalize() 호출 시 Armadillo EM으로 K-컴포넌트 GMM을 피팅.
struct GMMVoxel
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<GMMVoxel>;
  using ConstPtr = std::shared_ptr<const GMMVoxel>;

  /// @brief Per-voxel configuration, stored in IncrementalVoxelMap::voxel_setting.
  // 복셀별 GMM 설정: IncrementalVoxelMap의 voxel_insertion_setting()으로 전달
  struct Setting
  {
    int max_components = 3;                ///< Maximum GMM components per voxel
    // 복셀당 최대 가우시안 컴포넌트 수 K. 포인트 분포가 단순하면 pruning으로 줄어듬
    int max_em_iterations = 20;            ///< EM iteration limit
    // EM 알고리즘 최대 반복 횟수. 수렴 전 조기 종료 가능
    double convergence_tol = 1e-4;         ///< EM convergence tolerance
    // EM 로그-우도 변화량 임계치: |ΔL| < tol 이면 수렴으로 판정
    double covariance_regularization = 1e-3;  ///< Regularization added to covariance diagonal
    // 공분산 정칙화: Σ_k += ε·I (3x3). 특이행렬 방지 및 수치 안정성 보장
    double min_weight_threshold = 0.01;    ///< Prune components with weight below this
    // 최소 가중치 임계치: π_k < threshold인 컴포넌트를 제거 후 나머지 재정규화
    int reservoir_capacity = 256;          ///< Maximum points stored per voxel
    // 저수지 최대 용량: 메모리 제한 내에서 포인트를 균일 확률로 유지
  };

  /// @brief Default constructor.
  GMMVoxel();

  /// @brief Number of GMM components (K after finalize, 0 before).
  // finalize() 이전에는 0, 이후에는 피팅된 컴포넌트 수 K 반환
  size_t size() const;

  /// @brief Add a point via reservoir sampling (Algorithm R).
  /// @param setting  Voxel insertion setting (contains reservoir_capacity)
  /// @param points   Input point cloud
  /// @param i        Index of the point to add
  // Vitter의 Algorithm R: N번째 포인트가 도착하면 확률 C/N으로 저수지 내 임의 슬롯 교체.
  // 이로써 전체 스트림에서 균일 무작위 표본 C개를 유지함.
  void add(const Setting& setting, const PointCloud& points, size_t i);

  /// @brief Fit GMM from reservoir. Phase 1 stub: single-Gaussian fallback.
  // 저수지 포인트로부터 GMM 피팅 수행.
  // dirty_==true일 때만 실행, components_가 비어있으면 cold-start, 아니면 warm-start.
  void finalize();

  /// @brief KNN search over all GMM component means.
  /// @param pt      Query point (homogeneous 4D)
  /// @param result  KNN result accumulator (push local index 0..K-1)
  // 복셀 내 모든 GMM 컴포넌트 평균에 대해 유클리드 거리 기반 KNN 탐색.
  // w-component 차이를 0으로 강제하여 동차 좌표 편향 방지.
  template <typename Result>
  void knn_search(const Eigen::Vector4d& pt, Result& result) const
  {
    for (size_t k = 0; k < components_.size(); k++) {
      Eigen::Vector4d diff = pt - components_[k].mean;
      diff(3) = 0.0;  // w 성분 무시 (query w=1, mean w=0 → +1 편향 제거)
      const double sq_dist = diff.squaredNorm();
      result.push(k, sq_dist);  // 결과 힙에 (인덱스, 제곱거리) 삽입
    }
  }

  /// @brief Access finalized GMM components.
  const std::vector<GMMComponent>& components() const
  {
    return components_;
  }

  /// @brief Access raw reservoir points.
  const std::vector<Eigen::Vector4d>& reservoir() const
  {
    return reservoir_;
  }

  /// @brief True if new points added since last finalize.
  // add() 호출 후 finalize() 전까지 true. 불필요한 재피팅 방지용 플래그.
  bool is_dirty() const
  {
    return dirty_;
  }

  /// @brief Total number of points ever seen by this voxel.
  // 저수지를 거친 총 포인트 수 (저수지 크기와 무관). Algorithm R의 N에 해당.
  size_t total_points_seen() const
  {
    return total_points_seen_;
  }

private:
  // 저수지 샘플링 상태
  std::vector<Eigen::Vector4d> reservoir_;  // 균일 무작위 표본 버퍼, 최대 capacity개
  size_t total_points_seen_ = 0;            // 관측된 전체 포인트 수 N
  bool dirty_ = false;                      // finalize 필요 여부 플래그
  std::mt19937 rng_{42};  ///< Per-voxel RNG for deterministic reservoir sampling
  // 복셀별 고정 시드 RNG: 결정론적 저수지 교체를 위해 시드 42 사용

  // 피팅된 GMM 컴포넌트 (finalize 후)
  std::vector<GMMComponent> components_;
  Setting cached_setting_;  ///< Snapshot from last finalize
  // 마지막 finalize 시 사용된 설정 스냅샷: warm-start 시 재활용
};

// ---------------------------------------------------------------------------
// frame::traits<GMMVoxel> — exposes K component means/covs as "points"
// ---------------------------------------------------------------------------
// GMMVoxel을 gtsam_points의 프레임 인터페이스에 연결하는 특수화.
// 각 GMM 컴포넌트를 "포인트"로 노출하여 KNN, 복셀맵 삽입 등에 호환.
// 주의: point(), cov()는 값(value)으로 반환 — GMMComponent 벡터의 임시 복사.
namespace frame {

template <>
struct traits<GMMVoxel>
{
  static int size(const GMMVoxel& v)
  {
    return static_cast<int>(v.size());
  }
  // 컴포넌트 개수 K 반환

  static bool has_points(const GMMVoxel& v)
  {
    return v.size() > 0;
  }
  // 최소 1개 컴포넌트 존재 시 true
  static bool has_normals(const GMMVoxel&)
  {
    return false;
  }
  static bool has_covs(const GMMVoxel& v)
  {
    return v.size() > 0;
  }
  // GMM 컴포넌트는 항상 공분산을 보유
  static bool has_intensities(const GMMVoxel&)
  {
    return false;
  }

  // 값(by-value) 반환: components_ 벡터 내부 데이터의 복사본.
  // decltype(auto) in frame::point() free function이 이를 올바르게 처리.
  static Eigen::Vector4d point(const GMMVoxel& v, size_t k)
  {
    return v.components()[k].mean;
  }
  static Eigen::Vector4d normal(const GMMVoxel&, size_t)
  {
    return Eigen::Vector4d::Zero();
  }
  static Eigen::Matrix4d cov(const GMMVoxel& v, size_t k)
  {
    return v.components()[k].cov;
  }
  static double intensity(const GMMVoxel&, size_t)
  {
    return 0.0;
  }
};

}  // namespace frame

// ---------------------------------------------------------------------------
// frame::traits<IncrementalVoxelMap<GMMVoxel>> — returns by VALUE to avoid
// dangling reference (generic traits returns const&, but GMMVoxel traits
// returns temporaries, not references to stored data).
// ---------------------------------------------------------------------------
// IncrementalVoxelMap<GMMVoxel> 전용 traits: 전역 인덱스 i → (복셀, 로컬 k) 매핑 후
// GMMVoxel traits를 통해 점/공분산 접근. 댕글링 참조 방지를 위해 값 반환.
namespace frame {

template <>
struct traits<IncrementalVoxelMap<GMMVoxel>>
{
  static bool has_points(const IncrementalVoxelMap<GMMVoxel>& ivox)
  {
    return ivox.has_points();
  }
  static bool has_normals(const IncrementalVoxelMap<GMMVoxel>& ivox)
  {
    return ivox.has_normals();
  }
  static bool has_covs(const IncrementalVoxelMap<GMMVoxel>& ivox)
  {
    return ivox.has_covs();
  }
  static bool has_intensities(const IncrementalVoxelMap<GMMVoxel>& ivox)
  {
    return ivox.has_intensities();
  }

  // ivox.point(i): 전역 flat 인덱스 i에 해당하는 복셀/컴포넌트의 평균
  static Eigen::Vector4d point(const IncrementalVoxelMap<GMMVoxel>& ivox, size_t i)
  {
    return ivox.point(i);
  }
  static Eigen::Vector4d normal(const IncrementalVoxelMap<GMMVoxel>& ivox, size_t i)
  {
    return ivox.normal(i);
  }
  static Eigen::Matrix4d cov(const IncrementalVoxelMap<GMMVoxel>& ivox, size_t i)
  {
    return ivox.cov(i);
  }
  static double intensity(const IncrementalVoxelMap<GMMVoxel>& ivox, size_t i)
  {
    return ivox.intensity(i);
  }
};

}  // namespace frame

// ---------------------------------------------------------------------------
// GMMVoxelMapCPU — sibling of GaussianVoxelMapCPU (NOT a subclass)
// ---------------------------------------------------------------------------
// GaussianVoxelMap 인터페이스를 구현하면서 내부적으로 IncrementalVoxelMap<GMMVoxel>을 사용.
// GaussianVoxelMapCPU와 달리 복셀당 K개 GMM 컴포넌트를 보유.
// 다이아몬드 상속 아님: GaussianVoxelMap(추상)과 IncrementalVoxelMap(구현)을 다중 상속.
class GMMVoxelMapCPU : public GaussianVoxelMap, public IncrementalVoxelMap<GMMVoxel>
{
public:
  using Ptr = std::shared_ptr<GMMVoxelMapCPU>;
  using ConstPtr = std::shared_ptr<const GMMVoxelMapCPU>;

  /// @brief Constructor.
  /// @param resolution  Voxel resolution (edge length in meters)
  // 복셀 해상도(미터 단위 한 변 길이). 내부적으로 inv_leaf_size = 1/resolution 저장.
  explicit GMMVoxelMapCPU(double resolution);
  virtual ~GMMVoxelMapCPU();

  /// @brief Voxel resolution.
  virtual double voxel_resolution() const override;

  /// @brief Insert a point cloud frame into the voxelmap.
  // IncrementalVoxelMap::insert()에 위임: 포인트별 복셀 좌표 계산 → add() → finalize()
  virtual void insert(const PointCloud& frame) override;

  /// @brief Save compact (stub — not yet implemented for GMM).
  virtual void save_compact(const std::string& path) const override;

  /// @brief GMM-specific setting accessor.
  // IncrementalVoxelMap::voxel_insertion_setting()을 GMMVoxel::Setting& 타입으로 노출
  GMMVoxel::Setting& gmm_setting()
  {
    return voxel_insertion_setting();
  }

  /// @brief Compute voxel coordinate from a point.
  // 4D 동차 좌표 → 3D 정수 복셀 좌표: floor(x * inv_leaf_size).head<3>()
  Eigen::Vector3i voxel_coord(const Eigen::Vector4d& x) const;

  /// @brief Look up voxel index by coordinate. Returns -1 if not found.
  // 해시맵(voxels)에서 좌표로 flat 인덱스 검색. O(1) 평균 시간.
  int lookup_voxel_index(const Eigen::Vector3i& coord) const;

  /// @brief Look up a voxel by index.
  // flat_voxels[voxel_id]로 직접 접근. voxel_id는 lookup_voxel_index()로 얻은 값.
  const GMMVoxel& lookup_voxel(int voxel_id) const;

  /// @brief Number of voxels.
  // 현재 맵에 존재하는 복셀 수. flat_voxels 벡터 크기.
  size_t num_voxels() const
  {
    return flat_voxels.size();
  }

  /// @brief Finalize all dirty voxels (deferred EM fitting).
  // 지연된 EM 피팅 일괄 실행: insert() 시 생략된 finalize()를 모든 dirty 복셀에 대해 수행.
  // OpenMP로 복셀 간 독립적인 EM을 병렬 실행. needs_finalize_ 해제.
  void finalize_all();

  /// @brief Override knn_search to auto-finalize before querying.
  // 검색 전 지연된 finalize 자동 실행: needs_finalize_ 플래그 확인 후 finalize_all() 호출.
  // IncrementalVoxelMap의 insert()가 매 insert마다 전체 복셀 finalize를 실행하는
  // 설계 불일치(GaussianVoxel용 O(N) vs GMMVoxel EM O(K×N×D²×iters))를 우회하기 위함.
  virtual size_t knn_search(
    const double* pt, size_t k, size_t* k_indices, double* k_sq_dists,
    double max_sq_dist = std::numeric_limits<double>::max()) const override;

private:
  // 지연 finalize 플래그: insert() 후 true, finalize_all() 후 false.
  // mutable: const knn_search()에서 lazy evaluation 트리거 가능.
  mutable bool needs_finalize_ = false;
};

// ---------------------------------------------------------------------------
// frame::traits<GMMVoxelMapCPU>
// ---------------------------------------------------------------------------
// GMMVoxelMapCPU를 프레임 인터페이스에 연결. IncrementalVoxelMap<GMMVoxel>의
// point/cov/normal 접근자를 그대로 위임. decltype(auto)로 반환 타입 추론.
namespace frame {

template <>
struct traits<GMMVoxelMapCPU>
{
  static bool has_points(const GMMVoxelMapCPU& ivox)
  {
    return ivox.has_points();
  }
  static bool has_normals(const GMMVoxelMapCPU& ivox)
  {
    return ivox.has_normals();
  }
  static bool has_covs(const GMMVoxelMapCPU& ivox)
  {
    return ivox.has_covs();
  }
  static bool has_intensities(const GMMVoxelMapCPU& ivox)
  {
    return ivox.has_intensities();
  }

  static decltype(auto) point(const GMMVoxelMapCPU& ivox, size_t i)
  {
    return ivox.point(i);
  }
  static decltype(auto) normal(const GMMVoxelMapCPU& ivox, size_t i)
  {
    return ivox.normal(i);
  }
  static decltype(auto) cov(const GMMVoxelMapCPU& ivox, size_t i)
  {
    return ivox.cov(i);
  }
  static decltype(auto) intensity(const GMMVoxelMapCPU& ivox, size_t i)
  {
    return ivox.intensity(i);
  }
};

}  // namespace frame

}  // namespace gtsam_points
