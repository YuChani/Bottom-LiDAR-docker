# NDT 구현 코드와 수식의 1:1 매핑 해설

> **대상 파일**
> - `integrated_ndt_factor.hpp` (221줄) — 헤더: 자료구조, 유틸리티 함수, 클래스 선언
> - `integrated_ndt_factor_impl.hpp` (293줄) — 구현: 생성자, correspondence 탐색, evaluate
> - `integrated_matching_cost_factor.hpp` / `.cpp` — 기반 클래스: linearize 흐름

---

## 목차

1. [전체 구조 개요](#1-전체-구조-개요)
2. [NdtCorrespondence — 대응점 자료구조](#2-ndtcorrespondence--대응점-자료구조)
3. [compute_ndt_inverse_covariance — 역공분산 계산](#3-compute_ndt_inverse_covariance--역공분산-계산)
4. [compute_ndt_params — 가우시안 파라미터 d1, d2](#4-compute_ndt_params--가우시안-파라미터-d1-d2)
5. [IntegratedNDTFactor_ 클래스 — 멤버 변수](#5-integratedndt_factor_-클래스--멤버-변수)
6. [생성자 — 초기화](#6-생성자--초기화)
7. [update_correspondences — 대응점 탐색](#7-update_correspondences--대응점-탐색)
8. [evaluate — 비용 함수 · 그래디언트 · 헤시안 계산](#8-evaluate--비용-함수--그래디언트--헤시안-계산)
9. [기반 클래스 linearize — GTSAM으로의 통합](#9-기반-클래스-linearize--gtsam으로의-통합)
10. [전체 수식-코드 대조표](#10-전체-수식-코드-대조표)

---

## 1. 전체 구조 개요

NDT factor는 GTSAM의 팩터 그래프 프레임워크 위에서 동작한다. 최적화 반복마다 다음 순서로 호출된다:

```
GTSAM LevenbergMarquardtOptimizer
  └─ linearize(values)                  [integrated_matching_cost_factor.cpp:37]
       ├─ calc_delta(values)             → T_target_source 계산
       ├─ update_correspondences(delta)  → 각 소스 포인트에 대응 복셀 찾기
       └─ evaluate(delta, &H, &b)       → 비용 E, 헤시안 H, 그래디언트 b 계산
            └─ HessianFactor(H, b, E) 반환  → GTSAM이 (H + λI)Δp = -b 풀기
```

**핵심 수식 (Magnusson 2009, Eq. 6.9)**: NDT의 score function은 타겟 복셀 분포에 대한 소스 포인트의 likelihood 합이다:

$$E(\mathbf{p}) = \sum_{i=1}^{N} \left[ -d_1 \exp\!\left( -\frac{d_2}{2} \, \mathbf{r}_i^T \boldsymbol{\Sigma}_{k(i)}^{-1} \mathbf{r}_i \right) \right]$$

여기서 $\mathbf{r}_i = \boldsymbol{\mu}_{k(i)} - T(\mathbf{p}) \cdot \mathbf{a}_i$ (잔차), $k(i)$는 소스 포인트 $i$에 대응하는 타겟 복셀의 인덱스이다.

---

## 2. NdtCorrespondence — 대응점 자료구조

**파일**: `integrated_ndt_factor.hpp`, 라인 27-37

```cpp
struct NdtCorrespondence {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector4d mean;      // 복셀 평균 μ_k (동차좌표 4D)
  Eigen::Matrix4d inv_cov;   // 정규화된 역공분산 Σ_k^{-1} (4×4)
  bool valid;                 // 유효한 대응인지 여부
  NdtCorrespondence() : mean(Eigen::Vector4d::Zero()),
                         inv_cov(Eigen::Matrix4d::Zero()), valid(false) {}
};
```

**수식 대응**:
- `mean` = $\boldsymbol{\mu}_{k(i)}$ : NDT 수식의 복셀 평균 벡터
- `inv_cov` = $\boldsymbol{\Sigma}_{k(i)}^{-1}$ : NDT 수식의 역공분산 행렬
- `valid` : 대응이 없는 포인트(outlier)를 건너뛰기 위한 플래그

> **왜 4D인가?** gtsam_points는 모든 포인트를 동차좌표 $(x, y, z, 0)^T$로 표현한다. 4번째 성분은 항상 0이므로 $\boldsymbol{\Sigma}^{-1}$의 4행·4열은 실질적으로 사용되지 않지만, Eigen의 4×4 고정 크기 행렬을 사용하여 메모리 정렬과 SIMD 최적화를 유지한다.

---

## 3. compute_ndt_inverse_covariance — 역공분산 계산

**파일**: `integrated_ndt_factor.hpp`, 라인 44-54

```cpp
inline Eigen::Matrix4d compute_ndt_inverse_covariance(
    const Eigen::Matrix4d& cov, double regularization_epsilon = 1e-3) {
  // ① 고유값 분해: Σ = V Λ V^T
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(cov);
  Eigen::Vector4d eigenvalues = solver.eigenvalues();       // Λ = diag(λ_1, ..., λ_4)
  Eigen::Matrix4d eigenvectors = solver.eigenvectors();     // V

  // ② 정규화: 작은 고유값 클램핑
  double lambda_max = eigenvalues.maxCoeff();
  Eigen::Vector4d clamped = eigenvalues.array()
      .max(regularization_epsilon * lambda_max).matrix();

  // ③ 정규화된 공분산 재구성 후 역행렬
  Eigen::Matrix4d cov_reg = eigenvectors * clamped.asDiagonal() * eigenvectors.transpose();
  return cov_reg.inverse();
}
```

### 수식 설명

NDT에서 각 복셀의 공분산 $\boldsymbol{\Sigma}_k$는 해당 복셀 내 포인트들로부터 추정된다. 그러나 포인트 수가 적거나 포인트가 평면/직선상에 분포하면 $\boldsymbol{\Sigma}_k$가 **특이(singular)** 또는 **ill-conditioned**가 된다.

이를 해결하기 위한 정규화 과정:

**① 고유값 분해**:

$$\boldsymbol{\Sigma}_k = \mathbf{V} \boldsymbol{\Lambda} \mathbf{V}^T, \quad \boldsymbol{\Lambda} = \text{diag}(\lambda_1, \lambda_2, \lambda_3, \lambda_4)$$

**② 고유값 클램핑**:

$$\tilde{\lambda}_j = \max(\lambda_j, \, \epsilon \cdot \lambda_{\max}), \quad \epsilon = 10^{-3}$$

이는 가장 작은 고유값이 최대 고유값의 $\epsilon$배 이상이 되도록 보장한다. 물리적 의미: 복셀 내 포인트가 평면에 분포하더라도 법선 방향으로 최소한의 분산을 부여하여, 역행렬이 발산하지 않도록 한다.

**③ 역행렬**:

$$\boldsymbol{\Sigma}_k^{-1} = \left( \mathbf{V} \tilde{\boldsymbol{\Lambda}} \mathbf{V}^T \right)^{-1} = \mathbf{V} \tilde{\boldsymbol{\Lambda}}^{-1} \mathbf{V}^T$$

> **참고**: 코드에서는 `cov_reg.inverse()`로 일반 역행렬을 계산하지만, 수학적으로는 위와 동일하다. 고유값 분해를 이미 수행했으므로 $\mathbf{V} \tilde{\boldsymbol{\Lambda}}^{-1} \mathbf{V}^T$로 직접 계산하는 것이 더 효율적이겠으나, 코드는 명확성을 우선한 구현이다.

---

## 4. compute_ndt_params — 가우시안 파라미터 d1, d2

**파일**: `integrated_ndt_factor.hpp`, 라인 67-73

```cpp
inline void compute_ndt_params(double resolution, double outlier_ratio,
                                double& d1, double& d2) {
  double c1 = 10.0 * (1.0 - outlier_ratio);                            // 라인 68
  double c2 = outlier_ratio / (resolution * resolution * resolution);   // 라인 69
  double d3 = -std::log(c2);                                            // 라인 70
  d1 = -std::log(c1 + c2) - d3;                                        // 라인 71
  d2 = -2.0 * std::log((-std::log(c1 * std::exp(-0.5) + c2) - d3) / d1); // 라인 72
}
```

### 수식 유도 (Magnusson 2009, Eq. 6.10)

NDT의 score function은 inlier(가우시안)와 outlier(균일분포)의 **혼합 모델**에서 유도된다. 포인트 $\mathbf{q}$가 복셀에 속할 확률:

$$p(\mathbf{q}) = c_1 \exp\!\left(-\frac{\mathbf{q}^T \boldsymbol{\Sigma}^{-1} \mathbf{q}}{2}\right) + c_2$$

- $c_1$: 가우시안 성분의 가중치 (inlier)
- $c_2$: 균일 분포의 확률 밀도 (outlier)

파라미터 의미:
- `outlier_ratio` ($p_o$): 전체 포인트 중 outlier 비율 (벤치마크에서 0.1 사용)
- `resolution` ($r$): 복셀 한 변의 길이 (미터)

**코드-수식 매핑 (라인별)**:

| 라인 | 코드 | 수식 | 의미 |
|------|------|------|------|
| 68 | `c1 = 10.0 * (1.0 - outlier_ratio)` | $c_1 = 10(1 - p_o)$ | inlier 가중치. $p_o = 0.1$이면 $c_1 = 9.0$ |
| 69 | `c2 = outlier_ratio / (res³)` | $c_2 = \frac{p_o}{r^3}$ | 복셀 부피 내 균일 분포 밀도. $r = 0.5$이면 $c_2 = 0.8$ |
| 70 | `d3 = -log(c2)` | $d_3 = -\ln c_2$ | 정규화 상수. $c_2 = 0.8$이면 $d_3 \approx 0.223$ |
| 71 | `d1 = -log(c1 + c2) - d3` | $d_1 = -\ln(c_1 + c_2) - d_3$ | 가우시안 진폭. $d_1 \approx -2.505$ (**음수**) |
| 72 | `d2 = -2*log((...)/d1)` | $d_2 = -2\ln\!\left(\frac{-\ln(c_1 e^{-1/2} + c_2) - d_3}{d_1}\right)$ | 가우시안 폭. $d_2 > 0$ |

**$d_1$이 음수인 이유**: 원본 score function $s(\mathbf{p}) = -d_1 \exp(\cdots)$에서 $-d_1 > 0$이 되어야 score가 양수가 된다. 즉 $d_1 < 0$은 의도된 부호이다.

**$d_2$의 역할**: 지수함수의 감쇠율을 조절한다. $d_2$가 클수록 Mahalanobis 거리에 대해 비용이 빠르게 감소하여, 가까운 포인트에만 높은 비용을 부여한다.

### 수치 예시 (벤치마크 설정)

`outlier_ratio = 0.1`, `resolution = 0.5`:

$$c_1 = 9.0, \quad c_2 = 0.8, \quad d_3 = 0.223$$
$$d_1 = -\ln(9.8) - 0.223 = -2.282 - 0.223 = -2.505$$
$$d_2 = -2\ln\!\left(\frac{-\ln(9.0 \cdot e^{-0.5} + 0.8) - 0.223}{-2.505}\right) > 0$$

---

## 5. IntegratedNDTFactor_ 클래스 — 멤버 변수

**파일**: `integrated_ndt_factor.hpp`, 라인 196-216

```cpp
private:
  int num_threads;                          // 병렬 처리 스레드 수
  double resolution;                        // 복셀 해상도 r (기본값 1.0)
  double outlier_ratio;                     // outlier 비율 p_o (기본값 0.55)
  double regularization_epsilon;            // 정규화 ε (기본값 1e-3)
  NDTSearchMode search_mode;                // 탐색 모드 (DIRECT1/7/27)

  double correspondence_update_tolerance_rot;   // 대응점 갱신 회전 허용치
  double correspondence_update_tolerance_trans;  // 대응점 갱신 병진 허용치

  mutable double gauss_d1;                  // NDT 파라미터 d1 (음수)
  mutable double gauss_d2;                  // NDT 파라미터 d2 (양수)

  mutable Eigen::Isometry3d linearization_point;       // 현재 선형화 지점 T
  mutable Eigen::Isometry3d last_correspondence_point; // 마지막 대응점 갱신 지점
  mutable std::vector<NdtCorrespondence> correspondences; // 소스 포인트별 대응

  mutable std::vector<Eigen::Matrix4d> inv_cov_cache;  // 복셀별 Σ^{-1} 캐시
  mutable bool inv_cov_cached;                          // 캐시 유효성 플래그

  std::shared_ptr<const GaussianVoxelMapCPU> target_voxels; // 타겟 복셀맵
  std::shared_ptr<const SourceFrame> source;                 // 소스 포인트 클라우드
```

> **`mutable` 키워드**: GTSAM의 `linearize()`가 `const` 메서드이므로, 내부 상태를 갱신하려면 `mutable`이 필요하다. `correspondences`, `gauss_d1/d2`, `inv_cov_cache` 등이 최적화 반복마다 변경되지만 외부에서는 `const`로 취급된다.

---

## 6. 생성자 — 초기화

**파일**: `integrated_ndt_factor_impl.hpp`, 라인 20-46

```cpp
template <typename SourceFrame>
IntegratedNDTFactor_<SourceFrame>::IntegratedNDTFactor_(
  gtsam::Key target_key,
  gtsam::Key source_key,
  const GaussianVoxelMap::ConstPtr& target_voxels,
  const std::shared_ptr<const SourceFrame>& source)
: gtsam_points::IntegratedMatchingCostFactor(target_key, source_key),  // 기반 클래스
  num_threads(1),
  resolution(1.0),              // 기본 복셀 해상도 1.0m
  outlier_ratio(0.55),          // 기본 outlier 비율 55%
  regularization_epsilon(1e-3), // 기본 정규화 ε
  search_mode(NDTSearchMode::DIRECT7),  // 기본: 7개 복셀 탐색
  correspondence_update_tolerance_rot(0.0),
  correspondence_update_tolerance_trans(0.0),
  gauss_d1(0.0),               // evaluate 시 계산됨
  gauss_d2(0.0),               // evaluate 시 계산됨
  inv_cov_cached(false),       // 아직 역공분산 미계산
  target_voxels(std::dynamic_pointer_cast<const GaussianVoxelMapCPU>(target_voxels)),
  source(source) {
  // 검증: 소스 포인트 존재 확인
  if (!frame::has_points(*source)) { abort(); }
  // 검증: 타겟 복셀맵 변환 확인
  if (!this->target_voxels) { abort(); }
}
```

**벤치마크에서의 설정 오버라이드** (`main.cpp`, 라인 453-462):
```cpp
factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT7);
factor->set_outlier_ratio(0.1);                  // 0.55 → 0.1로 변경
factor->set_regularization_epsilon(1e-3);
factor->set_correspondence_update_tolerance(rot, trans);
```

즉 벤치마크에서는 `outlier_ratio`를 기본값 0.55에서 0.1로 낮춰 사용한다.

---

## 7. update_correspondences — 대응점 탐색

**파일**: `integrated_ndt_factor_impl.hpp`, 라인 98-204

이 함수는 현재 변환 추정치 $T$로 소스 포인트를 변환한 후, 각 포인트에 가장 가까운 타겟 복셀을 찾는다.

### 7.1 대응점 갱신 조건 판단 (라인 101-113)

```cpp
bool do_update = true;
if (correspondences.size() == frame::size(*source) &&
    (correspondence_update_tolerance_trans > 0.0 ||
     correspondence_update_tolerance_rot > 0.0)) {
  // 이전 갱신 지점과의 변위 계산
  Eigen::Isometry3d diff = delta.inverse() * last_correspondence_point;
  double diff_rot = Eigen::AngleAxisd(diff.linear()).angle();
  double diff_trans = diff.translation().norm();
  // 변위가 허용치 미만이면 갱신 건너뛰기
  if (diff_rot < correspondence_update_tolerance_rot &&
      diff_trans < correspondence_update_tolerance_trans) {
    do_update = false;
  }
}
```

**수식**:

$$\Delta T = T_{\text{current}}^{-1} \cdot T_{\text{last\_update}}$$
$$\Delta\theta = \angle(\Delta T_R), \quad \Delta t = \|\Delta T_t\|$$

$\Delta\theta < \theta_{\text{tol}}$ **이고** $\Delta t < t_{\text{tol}}$이면 대응점을 재계산하지 않는다. 이는 LM이 미세한 스텝을 반복할 때 불필요한 재탐색을 방지하는 최적화이다.

### 7.2 NDT 파라미터 및 이웃 오프셋 설정 (라인 115-138)

```cpp
correspondences.resize(frame::size(*source));
compute_ndt_params(resolution, outlier_ratio, gauss_d1, gauss_d2); // 매번 재계산

std::vector<Eigen::Vector3i> neighbor_offsets;
switch (search_mode) {
  case NDTSearchMode::DIRECT1:   // 현재 복셀 1개
    neighbor_offsets.push_back({0,0,0});
    break;
  case NDTSearchMode::DIRECT7:   // 현재 + 6면 이웃 = 7개
    neighbor_offsets.push_back({0,0,0});
    neighbor_offsets.push_back({1,0,0}); neighbor_offsets.push_back({-1,0,0});
    neighbor_offsets.push_back({0,1,0}); neighbor_offsets.push_back({0,-1,0});
    neighbor_offsets.push_back({0,0,1}); neighbor_offsets.push_back({0,0,-1});
    break;
  case NDTSearchMode::DIRECT27:  // 3×3×3 = 27개
    for (dx=-1..1) for (dy=-1..1) for (dz=-1..1) neighbor_offsets.push_back({dx,dy,dz});
    break;
}
```

DIRECT7이 기본값이다. 포인트가 복셀 경계 근처에 있을 때 인접 복셀의 가우시안이 더 적합할 수 있기 때문이다.

> **`compute_ndt_params` 중복 호출 문제**: `gauss_d1`, `gauss_d2`는 `resolution`과 `outlier_ratio`에만 의존하는 상수인데, `update_correspondences()`가 호출될 때마다 재계산된다. 이는 불필요한 중복이지만 정확성에는 영향 없다.

### 7.3 역공분산 캐싱 (라인 140-148)

```cpp
if (!inv_cov_cached) {
  const size_t num_voxels = target_voxels->num_voxels();
  inv_cov_cache.resize(num_voxels);
  for (size_t v = 0; v < num_voxels; v++) {
    const auto& voxel = target_voxels->lookup_voxel(v);
    inv_cov_cache[v] = compute_ndt_inverse_covariance(voxel.cov, regularization_epsilon);
  }
  inv_cov_cached = true;
}
```

모든 복셀의 $\boldsymbol{\Sigma}_k^{-1}$를 **한 번만** 계산하여 캐싱한다. 이후 `regularization_epsilon`이 변경되면 `inv_cov_cached = false`로 무효화된다 (`set_regularization_epsilon()` 참조, 라인 148-151).

### 7.4 포인트별 대응 탐색 (라인 150-184)

```cpp
const auto perpoint_task = [&](int i) {
  if (do_update) {
    correspondences[i].valid = false;

    // ① 소스 포인트를 현재 변환으로 변환
    Eigen::Vector4d pt = delta * frame::point(*source, i);
    // ② 변환된 포인트가 속하는 복셀 좌표 계산
    Eigen::Vector3i coord = target_voxels->voxel_coord(pt);

    const GaussianVoxel* best_voxel = nullptr;
    Eigen::Matrix4d best_inv_cov = Eigen::Matrix4d::Zero();
    double min_mahalanobis = std::numeric_limits<double>::max();

    // ③ 이웃 복셀들 중 최소 Mahalanobis 거리를 갖는 복셀 선택
    for (const auto& offset : neighbor_offsets) {
      Eigen::Vector3i neighbor_coord = coord + offset;
      const auto voxel_id = target_voxels->lookup_voxel_index(neighbor_coord);
      if (voxel_id < 0) continue;  // 빈 복셀이면 건너뛰기

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

    // ④ 최적 복셀을 대응으로 저장
    if (best_voxel) {
      correspondences[i].mean = best_voxel->mean;
      correspondences[i].inv_cov = best_inv_cov;
      correspondences[i].valid = true;
    }
  }
};
```

**수식 대응**:

| 단계 | 코드 | 수식 |
|------|------|------|
| ① 포인트 변환 | `pt = delta * point(source, i)` | $\mathbf{q}_i = T \cdot \mathbf{a}_i$ |
| ② 복셀 좌표 | `coord = voxel_coord(pt)` | $\mathbf{c} = \lfloor \mathbf{q}_i / r \rfloor$ |
| ③ Mahalanobis | `diff^T * inv_cov * diff` | $d_M = (\mathbf{q}_i - \boldsymbol{\mu}_k)^T \boldsymbol{\Sigma}_k^{-1} (\mathbf{q}_i - \boldsymbol{\mu}_k)$ |
| ④ 최적 대응 | `min_mahalanobis` | $k^*(i) = \arg\min_{k \in \mathcal{N}} d_M(\mathbf{q}_i, k)$ |

여기서 $\mathcal{N}$은 DIRECT7의 경우 현재 복셀 + 6면 이웃 = 7개 후보이다.

### 7.5 병렬 실행 (라인 187-203)

```cpp
if (is_omp_default() || num_threads == 1) {
  #pragma omp parallel for num_threads(num_threads) schedule(guided, 8)
  for (int i = 0; i < frame::size(*source); i++) {
    perpoint_task(i);
  }
} else {
  // TBB 병렬 처리
  tbb::parallel_for(...);
}
```

각 포인트의 대응 탐색은 독립적이므로 OpenMP 또는 TBB로 병렬화된다. `schedule(guided, 8)`은 포인트별 작업량이 다를 수 있으므로 동적 부하 분산을 사용한다.

---

## 8. evaluate — 비용 함수 · 그래디언트 · 헤시안 계산

**파일**: `integrated_ndt_factor_impl.hpp`, 라인 206-291

이 함수가 NDT의 수학적 핵심이다. 모든 소스 포인트에 대해 비용 $E$, 헤시안 $\mathbf{H}$, 그래디언트 $\mathbf{b}$를 누적 계산한다.

### 8.1 포인트별 비용 계산 — 전체 코드

```cpp
const auto perpoint_task = [&](int i, ...) {
  const auto& corr = correspondences[i];
  if (!corr.valid) return 0.0;                              // [A] 무효 대응 건너뛰기

  const auto& mean_A = frame::point(*source, i);            // [B] 소스 포인트 a_i
  const auto& mean_B = corr.mean;                           // [C] 타겟 복셀 평균 μ_k
  const auto& inv_cov_B = corr.inv_cov;                     // [D] 역공분산 Σ_k^{-1}

  Eigen::Vector4d transed_mean_A = delta * mean_A;          // [E] 변환된 소스: q_i = T·a_i

  Eigen::Vector4d residual = mean_B - transed_mean_A;       // [F] 잔차: r_i = μ_k - q_i

  double mahalanobis_dist = residual.transpose()             // [G] Mahalanobis 거리
                            * inv_cov_B * residual;

  double exponent = -gauss_d2 * mahalanobis_dist / 2.0;     // [H] 지수부: s_i
  if (exponent < -700.0) return 0.0;                         // [I] 언더플로 보호

  double e_term = std::exp(exponent);                        // [J] exp(s_i)
  double e_scaled = gauss_d2 * e_term;                       // [K] 수치 안정성 검사용
  if (e_scaled > 1.0 || e_scaled < 0.0 || std::isnan(e_scaled))
    return 0.0;                                              // [L] 비정상 값 보호

  const double error = -gauss_d1 * (1.0 - e_term);          // [M] 비용 E_i

  if (!H_target) return error;                               // [N] 비용만 필요시 조기 반환

  // --- 야코비안 계산 ---
  Eigen::Matrix<double, 4, 6> J_target = ...;               // [O] ∂r/∂p_target
  Eigen::Matrix<double, 4, 6> J_source = ...;               // [P] ∂r/∂p_source

  double derivative_scale = -gauss_d1 * gauss_d2 * e_term;  // [Q] 미분 스케일

  // --- 헤시안·그래디언트 누적 ---
  Eigen::Matrix<double, 6, 4> J_target_weighted =
      derivative_scale * J_target.transpose() * inv_cov_B;   // [R] 가중 야코비안
  *H_target += J_target_weighted * J_target;                  // [S] 헤시안 누적
  *b_target += J_target_weighted * residual;                  // [T] 그래디언트 누적

  return error;
};
```

### 8.2 수식-코드 상세 매핑

아래에서 각 코드 블록 [A]~[T]를 수식과 1:1로 대응시킨다.

---

#### [B]~[F] 잔차 계산

| 코드 | 변수 | 수식 |
|------|------|------|
| `mean_A` | $\mathbf{a}_i$ | 소스 포인트 (4D 동차좌표) |
| `mean_B` | $\boldsymbol{\mu}_{k(i)}$ | 대응 복셀의 평균 |
| `transed_mean_A = delta * mean_A` | $\mathbf{q}_i$ | $\mathbf{q}_i = T \cdot \mathbf{a}_i$ |
| `residual = mean_B - transed_mean_A` | $\mathbf{r}_i$ | $\mathbf{r}_i = \boldsymbol{\mu}_k - T \cdot \mathbf{a}_i$ |

> **잔차 방향**: `mean_B - transed_mean_A` 즉 "타겟 - 변환된소스" 이다. 이것은 GTSAM convention을 따른 것이며, 원본 NDT 논문의 방향과 동일하다.

---

#### [G] Mahalanobis 거리

```cpp
double mahalanobis_dist = residual.transpose() * inv_cov_B * residual;
```

$$d_M^{(i)} = \mathbf{r}_i^T \, \boldsymbol{\Sigma}_{k(i)}^{-1} \, \mathbf{r}_i$$

이것은 스칼라 값이다. 잔차가 공분산의 주축 방향으로 정렬될수록 작고, 수직 방향으로 벗어날수록 크다. 물리적으로 "포인트가 복셀 분포로부터 얼마나 떨어져 있는가"를 해당 분포의 형상을 고려하여 측정한 것이다.

---

#### [H]~[J] 지수함수 계산

```cpp
double exponent = -gauss_d2 * mahalanobis_dist / 2.0;   // s_i
double e_term = std::exp(exponent);                       // exp(s_i)
```

$$s_i = -\frac{d_2}{2} \cdot d_M^{(i)} = -\frac{d_2}{2} \, \mathbf{r}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i$$

$$e_i = \exp(s_i) = \exp\!\left(-\frac{d_2}{2} \, \mathbf{r}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i\right)$$

**범위**: $d_M \geq 0$이고 $d_2 > 0$이므로 $s_i \leq 0$, 따라서 $0 < e_i \leq 1$.
- 완벽 정합 ($\mathbf{r}_i = 0$): $s_i = 0$, $e_i = 1$
- 불량 정합 ($\|\mathbf{r}_i\| \to \infty$): $s_i \to -\infty$, $e_i \to 0$

---

#### [I], [L] 수치 안정성 보호

```cpp
if (exponent < -700.0) return 0.0;        // exp(-700) ≈ 0 (double 하한)
double e_scaled = gauss_d2 * e_term;
if (e_scaled > 1.0 || e_scaled < 0.0 || std::isnan(e_scaled)) return 0.0;
```

- `exponent < -700`: `std::exp(-700.0)`은 `double`의 최소 양수(≈ $5 \times 10^{-324}$)보다 작아 0을 반환한다. 연산 자체의 언더플로를 방지.
- `e_scaled = d2 * exp(s)` 검사: `derivative_scale` 계산에 사용되는 `d2 * exp(s)` 가 유효 범위 내인지 확인. `d2 * exp(s) > 1`이면 비정상적 상태.

---

#### [M] 비용 함수

```cpp
const double error = -gauss_d1 * (1.0 - e_term);
```

**원본 NDT score (Magnusson 2009, Eq. 6.9)**:

$$E_i^{\text{original}} = -d_1 \cdot \exp(s_i) = -d_1 \cdot e_i$$

**코드의 비음수 변환**:

$$E_i^{\text{code}} = -d_1 \cdot (1 - e_i)$$

두 형태의 관계:

$$E_i^{\text{code}} = E_i^{\text{original}} - (-d_1) = E_i^{\text{original}} + d_1$$

상수 $d_1$의 차이는 미분에 영향을 주지 않으므로 **그래디언트와 헤시안은 동일**하다.

**비음수 변환의 목적**: GTSAM의 `error()` 메서드는 비용이 음수가 아닌 것을 기대한다. $d_1 < 0$이므로 $-d_1 > 0$이고:
- 완벽 정합: $e_i = 1$ → $E_i = -d_1 \cdot 0 = 0$ (최솟값)
- 불량 정합: $e_i = 0$ → $E_i = -d_1 \cdot 1 = |d_1|$ (최댓값, 유한)

---

#### [O]~[P] 야코비안 계산

```cpp
// 타겟 포즈에 대한 잔차의 미분 (4×6)
Eigen::Matrix<double, 4, 6> J_target = Eigen::Matrix<double, 4, 6>::Zero();
J_target.block<3, 3>(0, 0) = -gtsam::SO3::Hat(transed_mean_A.head<3>());  // 회전 부분
J_target.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();                  // 병진 부분

// 소스 포즈에 대한 잔차의 미분 (4×6)
Eigen::Matrix<double, 4, 6> J_source = Eigen::Matrix<double, 4, 6>::Zero();
J_source.block<3, 3>(0, 0) = delta.linear() * gtsam::SO3::Hat(mean_A.template head<3>());
J_source.block<3, 3>(0, 3) = -delta.linear();
```

잔차 $\mathbf{r} = \boldsymbol{\mu} - T \cdot \mathbf{a}$이고, $T = T_{\text{target}}^{-1} \cdot T_{\text{source}}$이다. GTSAM의 SE(3) 파라미터화에서 포즈의 미소 변위 $\boldsymbol{\xi} = (\boldsymbol{\omega}, \mathbf{v}) \in \mathfrak{se}(3)$에 대해:

**J_target** ($\frac{\partial \mathbf{r}}{\partial \boldsymbol{\xi}_{\text{target}}}$, 4×6):

$$\mathbf{J}_{\text{target}} = \begin{bmatrix} -[\mathbf{q}_i]_\times & \mathbf{I}_3 \\ \mathbf{0}^T & 0 \end{bmatrix}$$

여기서 $[\mathbf{q}_i]_\times$는 변환된 소스 포인트 $\mathbf{q}_i$의 skew-symmetric 행렬이다.

- 회전 부분 (0~2열): $-[\mathbf{q}_i]_\times$ — 포인트를 $\boldsymbol{\omega}$ 방향으로 회전시킬 때 잔차의 변화
- 병진 부분 (3~5열): $\mathbf{I}_3$ — 순수 병진은 잔차에 직접 더해짐

코드에서 `SO3::Hat(v)`는 $[\mathbf{v}]_\times$를 계산한다:

$$[\mathbf{v}]_\times = \begin{bmatrix} 0 & -v_z & v_y \\ v_z & 0 & -v_x \\ -v_y & v_x & 0 \end{bmatrix}$$

**J_source** ($\frac{\partial \mathbf{r}}{\partial \boldsymbol{\xi}_{\text{source}}}$, 4×6):

$$\mathbf{J}_{\text{source}} = \begin{bmatrix} R \cdot [\mathbf{a}_i]_\times & -R \\ \mathbf{0}^T & 0 \end{bmatrix}$$

여기서 $R = T_{\text{linear}}$ (변환의 회전 부분).

> **4행이 0인 이유**: 동차좌표의 4번째 성분은 포인트에 대해 항상 0이므로, 야코비안의 4행도 항상 0이다.

---

#### [Q] derivative_scale — 미분 스케일

```cpp
double derivative_scale = -gauss_d1 * gauss_d2 * e_term;
```

$$\alpha_i = -d_1 \cdot d_2 \cdot e_i = -d_1 \cdot d_2 \cdot \exp(s_i)$$

**유도**: 비용 $E_i = -d_1(1 - e^{s_i})$를 $s_i$로 미분하면:

$$\frac{\partial E_i}{\partial s_i} = -d_1 \cdot (-e^{s_i}) \cdot (-1) = -d_1 \cdot e^{s_i}$$

... 이 아니라, $E_i = -d_1 + d_1 e^{s_i}$이므로:

$$\frac{\partial E_i}{\partial s_i} = d_1 \cdot e^{s_i}$$

그리고 $s_i = -\frac{d_2}{2} \mathbf{r}^T \boldsymbol{\Sigma}^{-1} \mathbf{r}$이므로:

$$\frac{\partial s_i}{\partial \mathbf{r}} = -d_2 \cdot \boldsymbol{\Sigma}^{-1} \mathbf{r}$$

$$\frac{\partial E_i}{\partial \mathbf{r}} = d_1 e^{s_i} \cdot (-d_2) \cdot \boldsymbol{\Sigma}^{-1} \mathbf{r} = -d_1 d_2 e^{s_i} \cdot \boldsymbol{\Sigma}^{-1} \mathbf{r}$$

연쇄 법칙으로 파라미터 $\mathbf{p}$에 대해:

$$\frac{\partial E_i}{\partial \mathbf{p}} = \frac{\partial E_i}{\partial \mathbf{r}} \cdot \frac{\partial \mathbf{r}}{\partial \mathbf{p}} = (-d_1 d_2 e^{s_i}) \cdot \boldsymbol{\Sigma}^{-1} \mathbf{r} \cdot \mathbf{J}$$

여기서 $(-d_1 d_2 e^{s_i})$가 바로 **derivative_scale** $\alpha_i$이다.

**부호 확인**: $d_1 < 0$이므로 $-d_1 > 0$, $d_2 > 0$, $e^{s_i} > 0$ → $\alpha_i > 0$ (항상 양수). 이는 헤시안이 PSD(positive semi-definite)가 되기 위한 필수 조건이다.

---

#### [R]~[T] 가중 야코비안, 헤시안, 그래디언트 누적

```cpp
// 가중 야코비안: (6×4) = scalar * (6×4) * (4×4)
Eigen::Matrix<double, 6, 4> J_target_weighted =
    derivative_scale * J_target.transpose() * inv_cov_B;

// 헤시안 누적: (6×6) += (6×4) * (4×6)
*H_target += J_target_weighted * J_target;

// 그래디언트 누적: (6×1) += (6×4) * (4×1)
*b_target += J_target_weighted * residual;
```

**수식 — 헤시안**:

$$\mathbf{H} \mathrel{+}= \alpha_i \cdot \mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{J}_i$$

이것을 전개하면:

$$\mathbf{H} = \sum_{i=1}^{N} \underbrace{(-d_1 \cdot d_2 \cdot e^{s_i})}_{\alpha_i} \cdot \underbrace{\mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{J}_i}_{6 \times 6}$$

> 이것은 완전한 Newton 헤시안의 **1차 항**에 해당한다. 완전한 형태는 $\alpha_i$를 포함하는 외적 항을 추가로 가지지만, Gauss-Newton 유사 근사로서 이 항만 사용한다.

**수식 — 그래디언트**:

$$\mathbf{b} \mathrel{+}= \alpha_i \cdot \mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i$$

이것을 전개하면:

$$\mathbf{b} = \sum_{i=1}^{N} (-d_1 \cdot d_2 \cdot e^{s_i}) \cdot \mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i = \sum_{i=1}^{N} \frac{\partial E_i}{\partial \mathbf{p}}$$

즉 **그래디언트 $\mathbf{b}$는 정확히 비용 함수의 1차 미분**이다. 근사 없음.

---

#### 소스 포즈도 동일한 구조

```cpp
Eigen::Matrix<double, 6, 4> J_source_weighted =
    derivative_scale * J_source.transpose() * inv_cov_B;
*H_source += J_source_weighted * J_source;
*H_target_source += J_target_weighted * J_source;     // 교차 헤시안
*b_source += J_source_weighted * residual;
```

binary factor(타겟-소스 두 포즈가 모두 변수)일 때, 12×12 전체 헤시안은:

$$\begin{bmatrix} \mathbf{H}_{\text{target}} & \mathbf{H}_{\text{target,source}} \\ \mathbf{H}_{\text{target,source}}^T & \mathbf{H}_{\text{source}} \end{bmatrix}, \quad \begin{bmatrix} \mathbf{b}_{\text{target}} \\ \mathbf{b}_{\text{source}} \end{bmatrix}$$

---

### 8.3 병렬 리덕션 (라인 286-291)

```cpp
if (is_omp_default() || num_threads == 1) {
  return scan_matching_reduce_omp(perpoint_task, frame::size(*source),
         num_threads, H_target, H_source, H_target_source, b_target, b_source);
} else {
  return scan_matching_reduce_tbb(perpoint_task, frame::size(*source),
         H_target, H_source, H_target_source, b_target, b_source);
}
```

`scan_matching_reduce_omp/tbb`는 포인트별 `perpoint_task`를 병렬로 실행하면서, 각 스레드의 로컬 H, b를 최종적으로 합산(reduce)한다. 이는 `#pragma omp parallel for reduction(+:...)` 패턴의 행렬 버전이다.

최종 반환값은 전체 비용 $E = \sum_i E_i$이다.

---

## 9. 기반 클래스 linearize — GTSAM으로의 통합

**파일**: `integrated_matching_cost_factor.cpp`, 라인 37-55

```cpp
gtsam::GaussianFactor::shared_ptr
IntegratedMatchingCostFactor::linearize(const gtsam::Values& values) const {
  // ① 현재 변환 계산: T = T_target^{-1} * T_source
  auto delta = calc_delta(values);

  // ② 대응점 갱신 (NDT: 7개 복셀 탐색)
  update_correspondences(delta);

  // ③ 비용·헤시안·그래디언트 계산
  Eigen::Matrix<double, 6, 6> H_target, H_source, H_target_source;
  Eigen::Matrix<double, 6, 1> b_target, b_source;
  double error = evaluate(delta, &H_target, &H_source, &H_target_source,
                          &b_target, &b_source);

  // ④ GTSAM HessianFactor로 패키징
  if (is_binary) {
    return HessianFactor(keys[0], keys[1],
                         H_target, H_target_source, -b_target,
                         H_source, -b_source, error);
  } else {
    return HessianFactor(keys[0], H_source, -b_source, error);
  }
}
```

`HessianFactor`는 GTSAM의 이차 근사를 표현한다:

$$E(\mathbf{p} + \delta\mathbf{p}) \approx E + (-\mathbf{b})^T \delta\mathbf{p} + \frac{1}{2} \delta\mathbf{p}^T \mathbf{H} \delta\mathbf{p}$$

> **`-b`에 대한 주의**: 코드에서 `evaluate()`가 반환하는 `b`는 $\frac{\partial E}{\partial \mathbf{p}}$ (그래디언트)이다. GTSAM의 `HessianFactor`는 $-\mathbf{b}$ (음의 그래디언트)를 받으므로, 코드에서 `-b_target`, `-b_source`로 전달한다.

LM 최적화기는 이 `HessianFactor`를 받아 다음을 풀어 갱신량을 계산한다:

$$(\mathbf{H} + \lambda \mathbf{I}) \, \delta\mathbf{p} = \mathbf{b}$$

여기서 $\lambda$는 LM의 damping factor이다.

---

## 10. 전체 수식-코드 대조표

| 수식 | 기호 | 코드 변수/표현 | 파일:라인 |
|------|------|--------------|----------|
| 소스 포인트 | $\mathbf{a}_i$ | `frame::point(*source, i)` | impl:231 |
| 변환된 소스 | $\mathbf{q}_i = T \cdot \mathbf{a}_i$ | `delta * mean_A` | impl:235 |
| 타겟 복셀 평균 | $\boldsymbol{\mu}_k$ | `corr.mean` | impl:232 |
| 역공분산 | $\boldsymbol{\Sigma}_k^{-1}$ | `corr.inv_cov` | impl:233 |
| 잔차 | $\mathbf{r}_i = \boldsymbol{\mu}_k - \mathbf{q}_i$ | `mean_B - transed_mean_A` | impl:238 |
| Mahalanobis 거리 | $d_M = \mathbf{r}_i^T \boldsymbol{\Sigma}^{-1} \mathbf{r}_i$ | `residual.transpose() * inv_cov_B * residual` | impl:240 |
| 지수부 | $s_i = -\frac{d_2}{2} d_M$ | `-gauss_d2 * mahalanobis_dist / 2.0` | impl:243 |
| 지수함수 | $e_i = \exp(s_i)$ | `std::exp(exponent)` | impl:248 |
| 비용 | $E_i = -d_1(1 - e_i)$ | `-gauss_d1 * (1.0 - e_term)` | impl:257 |
| 야코비안 (target, 회전) | $-[\mathbf{q}_i]_\times$ | `-SO3::Hat(transed_mean_A.head<3>())` | impl:264 |
| 야코비안 (target, 병진) | $\mathbf{I}_3$ | `Eigen::Matrix3d::Identity()` | impl:265 |
| 야코비안 (source, 회전) | $R [\mathbf{a}_i]_\times$ | `delta.linear() * SO3::Hat(mean_A.head<3>())` | impl:268 |
| 야코비안 (source, 병진) | $-R$ | `-delta.linear()` | impl:269 |
| 미분 스케일 | $\alpha_i = -d_1 d_2 e_i$ | `-gauss_d1 * gauss_d2 * e_term` | impl:272 |
| 가중 야코비안 | $\alpha_i \mathbf{J}^T \boldsymbol{\Sigma}^{-1}$ | `derivative_scale * J_target.transpose() * inv_cov_B` | impl:274 |
| 헤시안 | $\mathbf{H} \mathrel{+}= \alpha_i \mathbf{J}^T \boldsymbol{\Sigma}^{-1} \mathbf{J}$ | `H_target += J_target_weighted * J_target` | impl:277 |
| 그래디언트 | $\mathbf{b} \mathrel{+}= \alpha_i \mathbf{J}^T \boldsymbol{\Sigma}^{-1} \mathbf{r}$ | `b_target += J_target_weighted * residual` | impl:280 |
| NDT 파라미터 $d_1$ | $d_1 = -\ln(c_1 + c_2) - d_3$ | `-std::log(c1 + c2) - d3` | hpp:71 |
| NDT 파라미터 $d_2$ | $d_2 = -2\ln(\cdots / d_1)$ | `-2.0 * std::log((...) / d1)` | hpp:72 |
| 정규화 역공분산 | $\boldsymbol{\Sigma}_{\text{reg}}^{-1} = (\mathbf{V}\tilde{\boldsymbol{\Lambda}}\mathbf{V}^T)^{-1}$ | `cov_reg.inverse()` | hpp:53 |
| 고유값 클램핑 | $\tilde{\lambda}_j = \max(\lambda_j, \epsilon \lambda_{\max})$ | `eigenvalues.array().max(eps * lambda_max)` | hpp:50 |
| 대응 선택 기준 | $k^* = \arg\min_{k \in \mathcal{N}} d_M(\mathbf{q}_i, k)$ | `min_mahalanobis` 비교 루프 | impl:161-177 |

---

## 부록: 전체 데이터 흐름 다이어그램

```
┌─────────────────────────────────────────────────────────────┐
│                   GTSAM LM Optimizer                         │
│   values = {T_target, T_source}                              │
│   반복: (H + λI)Δp = b  →  values 갱신  →  수렴 판정         │
└──────────────────┬──────────────────────────────────────────┘
                   │ linearize(values)
                   ▼
┌─────────────────────────────────────────────────────────────┐
│  IntegratedMatchingCostFactor::linearize()                   │
│  [integrated_matching_cost_factor.cpp:37-55]                 │
│                                                              │
│  ① delta = T_target⁻¹ · T_source                            │
│  ② update_correspondences(delta)  ──────────────────┐        │
│  ③ evaluate(delta, &H, &b)  ───────────────────┐    │        │
│  ④ return HessianFactor(H, -b, E)              │    │        │
└─────────────────────────────────────────────────┼────┼───────┘
                                                  │    │
              ┌───────────────────────────────────┘    │
              ▼                                        ▼
┌──────────────────────────────┐  ┌──────────────────────────────────┐
│ evaluate() [impl:206-291]    │  │ update_correspondences()         │
│                              │  │ [impl:98-204]                    │
│ for each source point i:     │  │                                  │
│                              │  │ ① tolerance 검사 (갱신 필요?)     │
│  q = T · a_i                 │  │ ② compute_ndt_params(d1, d2)     │
│  r = μ_k - q                 │  │ ③ inv_cov 캐시 (최초 1회)         │
│  dM = r^T Σ⁻¹ r             │  │ ④ for each point:                │
│  s = -d2/2 · dM              │  │    pt = T · a_i                  │
│  e = exp(s)                  │  │    coord = voxel_coord(pt)       │
│  E += -d1·(1-e)              │  │    for each neighbor (7개):      │
│                              │  │      dM = (pt-μ)^T Σ⁻¹ (pt-μ)  │
│  α = -d1·d2·e               │  │    best = argmin(dM)             │
│  H += α · J^T Σ⁻¹ J         │  │    store (mean, inv_cov)         │
│  b += α · J^T Σ⁻¹ r         │  │                                  │
│                              │  │ ⑤ OMP/TBB 병렬 실행              │
│ return E                     │  └──────────────────────────────────┘
└──────────────────────────────┘
```

---

## 참고 문헌

- Biber, P. & Straßer, W. (2003). *"The Normal Distributions Transform: A New Approach to Laser Scan Matching."* IROS 2003.
- Magnusson, M. (2009). *"The Three-Dimensional Normal-Distributions Transform — an Efficient Representation for Registration, Surface Analysis, and Loop Detection."* PhD Thesis, Örebro University. (Chapter 6, Eq. 6.9-6.10)
- gtsam_points: https://github.com/koide3/gtsam_points (Kenji Koide)
- GTSAM: https://gtsam.org/ (Georgia Tech)
