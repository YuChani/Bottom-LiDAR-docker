---
title: "LightNDT vs MixtureLightNDT — 코드 레벨 정밀 비교"
related_docs:
  - docs/gmm/gmm-voxel-structure-study.md
  - docs/gmm/gmm-mathematical-foundations.md
  - docs/gmm/gmm-design.md
date: 2026-03-25
---

# LightNDT vs MixtureLightNDT — 코드 레벨 정밀 비교

> **관련 문서**:
> - `docs/gmm/gmm-voxel-structure-study.md` — Section 3에서 두 factor의 중간 수준 비교를 다룸. 이 문서는 그 후속으로, 실제 C++ 코드를 라인 단위로 발췌하여 비교함.
> - `docs/gmm/gmm-mathematical-foundations.md` — EM 유도, SE(3) Jacobian, Gauss-Newton Hessian 수학적 기반
> - `docs/gmm/gmm-design.md` — 전체 GMM 아키텍처 설계 결정 근거
>
> **날짜**: 2026-03-25

---

## 목차

- [0. 이 문서의 목적과 범위](#0-이-문서의-목적과-범위)
  - [0.1 학습 목표](#01-학습-목표)
  - [0.2 대상 파일 4개](#02-대상-파일-4개)
  - [0.3 공유 유틸리티](#03-공유-유틸리티)
- [1. 아키텍처 개요 — 공통 기반과 분기점](#1-아키텍처-개요--공통-기반과-분기점)
  - [1.1 상속 구조](#11-상속-구조)
  - [1.2 공통 파이프라인: linearize() → update_correspondences() → evaluate()](#12-공통-파이프라인-linearize--update_correspondences--evaluate)
  - [1.3 scan_matching_reduce: 동일한 병렬 합산](#13-scan_matching_reduce-동일한-병렬-합산)
- [2. 헤더 비교 — 멤버 변수 차이](#2-헤더-비교--멤버-변수-차이)
  - [2.1 복셀맵 타입 & 다운캐스트](#21-복셀맵-타입--다운캐스트)
  - [2.2 대응 구조체 비교](#22-대응-구조체-비교)
  - [2.3 inv_cov_cache 차원](#23-inv_cov_cache-차원)
  - [2.4 동일한 멤버 변수들](#24-동일한-멤버-변수들)
- [3. update_correspondences() 구현 비교](#3-update_correspondences-구현-비교)
- [4. evaluate() 구현 비교](#4-evaluate-구현-비교)
- [5. 생성자 & 유틸리티 메서드 비교](#5-생성자--유틸리티-메서드-비교)
- [6. 왜 이런 차이가 필요한가 — 수학적 동기](#6-왜-이런-차이가-필요한가--수학적-동기)
- [7. 요약 비교 테이블](#7-요약-비교-테이블)
- [8. 학습 확인 질문](#8-학습-확인-질문)

---

## 0. 이 문서의 목적과 범위

### 0.1 학습 목표

이 문서는 `IntegratedLightNDTFactor`(이하 LightNDT)와 `IntegratedMixtureLightNDTFactor`(이하 MixtureLightNDT)를 **코드 레벨에서 라인 단위로 비교**하는 학습용 레퍼런스다. 두 factor는 수학적 구조와 코드 패턴을 거의 공유하면서도, 핵심 데이터 구조와 대응 계산 방식에서 체계적으로 갈라진다. 그 분기점을 정확히 파악하는 것이 이 문서의 목표다.

`docs/gmm/gmm-voxel-structure-study.md` Section 3에서 두 factor를 중간 수준으로 소개했다. 이 문서는 그 연장선으로, 실제 C++ 구현체를 직접 인용하여 각 차이의 *이유*까지 설명한다.

### 0.2 대상 파일 4개

| 파일 | 줄 수 | 역할 |
|------|-------|------|
| `include/ndt/integrated_light_ndt_factor.hpp` | 102줄 | LightNDT 헤더 — 클래스 선언, 멤버 변수, 공개 API |
| `include/ndt/impl/integrated_light_ndt_factor_impl.hpp` | 260줄 | LightNDT 구현 — 생성자, `update_correspondences()`, `evaluate()` |
| `include/gmm/integrated_mixture_light_ndt_factor.hpp` | 163줄 | MixtureLightNDT 헤더 — 클래스 선언, `MixtureNdtCorrespondence` 구조체 |
| `include/gmm/impl/integrated_mixture_light_ndt_factor_impl.hpp` | 385줄 | MixtureLightNDT 구현 — 생성자, `update_correspondences()`, `evaluate()` |

두 구현 파일은 `.hpp` 확장자를 쓰지만 실질적으로 `.cpp`에 해당한다. 명시적 템플릿 인스턴스화 덕분에 헤더에 포함하지 않아도 된다.

### 0.3 공유 유틸리티

두 factor는 `include/ndt/integrated_ndt_factor.hpp`에서 다음 세 가지를 공유한다.

- `NdtCorrespondence` 구조체 (LightNDT만 직접 사용, MixtureLightNDT는 유사한 자체 구조체 정의)
- `compute_ndt_inverse_covariance()` 함수 — 정규화된 역공분산 Σ⁻¹ 계산
- `NDTSearchMode` 열거형 — `DIRECT1`, `DIRECT7` 등 이웃 탐색 반경 설정

---

## 1. 아키텍처 개요 — 공통 기반과 분기점

### 1.1 상속 구조

두 factor는 `gtsam_points` 라이브러리가 제공하는 동일한 base class에서 출발한다.

```
IntegratedMatchingCostFactor (gtsam_points base)
├── IntegratedLightNDTFactor_<SourceFrame>
│   └── target: GaussianVoxelMapCPU (1 voxel = 1 Gaussian)
└── IntegratedMixtureLightNDTFactor_<SourceFrame>
    └── target: GMMVoxelMapCPU (1 voxel = K Gaussians)
```

`IntegratedMatchingCostFactor`는 GTSAM 팩터 그래프의 최적화 루프와 인터페이스하는 골격을 제공한다. 두 factor는 각각 이 base class를 상속하면서 `update_correspondences()`와 `evaluate()` 두 메서드만 다르게 오버라이드한다. 나머지 최적화 인프라는 100% 동일하다.

타겟 복셀맵 타입이 유일한 구조적 분기점이다. `GaussianVoxelMapCPU`는 복셀 하나당 하나의 가우시안을 저장하고, `GMMVoxelMapCPU`는 복셀 하나당 K개의 가우시안 컴포넌트를 저장한다.

### 1.2 공통 파이프라인: linearize() → update_correspondences() → evaluate()

GTSAM LM 옵티마이저는 매 반복마다 각 팩터의 `error()`와 `linearize()`를 호출한다. 두 메서드 모두 base class인 `IntegratedMatchingCostFactor`에서 구현되어 있으며, 내부적으로 동일한 3단계 파이프라인을 따른다.

```cpp
// gtsam_points/factors/integrated_matching_cost_factor.cpp
// error() — 비용만 계산 (gradient 불필요)
double IntegratedMatchingCostFactor::error(const gtsam::Values& values) const {
  Eigen::Isometry3d delta = calc_delta(values);
  update_correspondences(delta);
  return evaluate(delta);
}

// linearize() — 비용 + Hessian/gradient 계산
gtsam::GaussianFactor::shared_ptr IntegratedMatchingCostFactor::linearize(
    const gtsam::Values& values) const {
  Eigen::Isometry3d delta = calc_delta(values);
  update_correspondences(delta);
  double error = evaluate(delta, &H_target, &H_source, &H_target_source,
                          &b_target, &b_source);
  // → HessianFactor 생성하여 GTSAM에 전달
}
```

`delta = T_target⁻¹ · T_source`로 정의된 두 포즈 간의 상대 변환이다. source 포인트 클라우드를 target 좌표계로 가져오는 변환으로, 대응 탐색과 비용 계산 모두 이 변환을 기준으로 수행된다.

핵심은 두 factor가 오버라이드하는 메서드가 `update_correspondences()`와 `evaluate()` **딱 두 개**라는 점이다. 최적화 루프 진입, delta 계산, HessianFactor 조립 — 이 모든 인프라는 공유된다.

### 1.3 scan_matching_reduce: 동일한 병렬 합산

두 factor의 `evaluate()` 내부는 모두 `scan_matching_reduce_omp()` 또는 `scan_matching_reduce_tbb()`를 호출한다. 이 함수는 `gtsam_points/factors/impl/scan_matching_reduction.hpp`에 정의되어 있으며, 포인트별 비용 계산 람다 `f(i, H_target*, ...) → double`을 받아 전체 포인트에 걸쳐 비용과 Hessian을 병렬로 합산한다.

```
for each source point i (병렬):
    cost_i = f(i, &H_target_i, &H_source_i, ...)
total_cost = Σ cost_i
H_total   = Σ H_target_i, Σ H_source_i, ...
```

두 factor 사이에서 이 reduce 구조는 완전히 동일하다. 달라지는 것은 람다 `f`의 내용, 즉 포인트 하나에 대해 어떤 가우시안과 대응을 맺고 어떤 비용을 계산하느냐다.

---

## 2. 헤더 비교 — 멤버 변수 차이

### 2.1 복셀맵 타입 & 다운캐스트

헤더 파일에서 타겟 복셀맵을 저장하는 멤버 변수를 비교하면 두 factor의 차이가 한눈에 드러난다.

```cpp
// include/ndt/integrated_light_ndt_factor.hpp (line 96)
std::shared_ptr<const GaussianVoxelMapCPU> target_voxels;
```

```cpp
// include/gmm/integrated_mixture_light_ndt_factor.hpp (line 156)
std::shared_ptr<const GMMVoxelMapCPU> target_voxels;
```

두 factor의 생성자는 모두 다형성 base 타입인 `GaussianVoxelMap::ConstPtr`로 타겟을 받는다. 생성자 이니셜라이저 리스트에서 구체 타입으로 다운캐스트한다.

```cpp
// include/ndt/impl/integrated_light_ndt_factor_impl.hpp (line 32)
target_voxels(std::dynamic_pointer_cast<const GaussianVoxelMapCPU>(target_voxels))

// include/gmm/impl/integrated_mixture_light_ndt_factor_impl.hpp (line 40)
target_voxels(std::dynamic_pointer_cast<const GMMVoxelMapCPU>(target_voxels))
```

다운캐스트 실패, 즉 잘못된 타입의 복셀맵을 넘기면 `dynamic_pointer_cast`가 `nullptr`을 반환하고 이후 코드에서 abort한다. LightNDT는 `GaussianVoxelMapCPU`(복셀당 가우시안 1개), MixtureLightNDT는 `GMMVoxelMapCPU`(복셀당 K개 컴포넌트)를 요구한다.

### 2.2 대응 구조체 비교

`update_correspondences()`가 채워 넣는 대응 구조체도 서로 다르다. 두 구조체는 필드 구성이 비슷해 보이지만, 각 필드의 의미와 초기화 방식에 미묘한 차이가 있다.

**NdtCorrespondence** (LightNDT용):

```cpp
// include/ndt/integrated_ndt_factor.hpp (lines 29-38)
struct NdtCorrespondence {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector4d mean;      // 복셀 평균 (4D homogeneous, w=1)
  Eigen::Matrix4d inv_cov;   // 정규화된 역공분산 Σ⁻¹
  double one_over_Z;         // 1/√((2π)³|Σ|) — NDT 정규화 상수
  bool valid;
};
```

**MixtureNdtCorrespondence** (MixtureLightNDT용):

```cpp
// include/gmm/integrated_mixture_light_ndt_factor.hpp (lines 30-38)
struct MixtureNdtCorrespondence {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector4d mean = Eigen::Vector4d::Zero();     // GMM 컴포넌트 평균 μ_{k*} (w=0)
  Eigen::Matrix4d inv_cov = Eigen::Matrix4d::Zero();  // Σ_{k*}⁻¹
  double weight = 0.0;                                // π_{k*} — 혼합 가중치
  bool valid = false;
};
```

각 필드의 차이를 정리하면 다음과 같다.

| 필드 | NdtCorrespondence | MixtureNdtCorrespondence | 차이 이유 |
|------|-------------------|--------------------------|-----------|
| `mean.w` | **1** (GaussianVoxel convention) | **0** (GMMComponent convention) | `GaussianVoxel`은 삽입 시 동차 좌표 w=1로 mean을 누적하지만, `GMMVoxel`은 reservoir에 w=0으로 저장하고 EM이 3D mean을 반환하므로 w=0 |
| `one_over_Z` | 있음 (NDT score 정규화 상수) | **없음** | LightNDT는 NDT scoring을 쓰지 않고 순수 마할라노비스 거리만 사용하므로 정규화 불필요 — 필드 이름이 "NDT"를 포함하지만 실제로는 사용되지 않는 유산 |
| `weight` (π_k) | **없음** | 있음 (0~1 범위) | GMM 각 컴포넌트의 혼합 가중치 π_k — 비용 함수에 곱해져 어떤 컴포넌트가 이 포인트를 설명하는지 반영 |
| 멤버 초기화 | 기본 생성자, zero 초기화 없음 | **멤버 이니셜라이저** 사용 | 기능 차이는 없고 스타일 차이. `MixtureNdtCorrespondence`는 집합체 초기화 시 안전한 기본값을 보장 |

`mean.w` 차이를 조금 더 풀면: `GaussianVoxel::insert()`는 새 포인트를 누적할 때 동차 좌표(w=1)를 그대로 sum에 더하기 때문에 최종 `mean` 벡터의 w 성분이 1이 된다. 반면 `GMMComponent`는 EM 알고리즘이 3D 공간에서 평균 위치를 계산하고 `Eigen::Vector4d`에 담을 때 w=0으로 패딩한다. evaluate() 내부의 잔차 계산 `delta.linear() * mean_3d - pt`에서 w 성분은 참여하지 않으므로 수치 결과에는 영향이 없지만, 코드를 읽을 때 혼동 요인이 된다.

### 2.3 inv_cov_cache 차원

역공분산 행렬을 캐싱하는 멤버 변수의 차원이 다르다.

```cpp
// include/ndt/integrated_light_ndt_factor.hpp (line 93)
mutable std::vector<Eigen::Matrix4d> inv_cov_cache;            // 1D: cache[voxel_id]
```

```cpp
// include/gmm/integrated_mixture_light_ndt_factor.hpp (line 153)
mutable std::vector<std::vector<Eigen::Matrix4d>> inv_cov_cache;  // 2D: cache[voxel_id][k]
```

복셀당 가우시안이 하나인 LightNDT는 캐시가 1D 벡터로 충분하다. 복셀당 K개의 컴포넌트가 있는 MixtureLightNDT는 각 복셀마다 K개의 역공분산 행렬을 저장해야 하므로 2D 중첩 벡터가 된다.

메모리 사용량 비교:

- LightNDT: V × 128 bytes (V = 복셀 수, `Matrix4d` = 128 bytes)
- MixtureLightNDT: V × K × 128 bytes (K = 평균 컴포넌트 수, 보통 2~4)

예시로 V=1000, K=4라면 LightNDT는 약 125 KB, MixtureLightNDT는 약 500 KB다. 복셀 수가 수만 개 규모로 늘어나도 상대 비율은 동일하다.

`mutable` 한정자는 두 클래스 모두에서 쓴다. `linearize()`는 `const`로 선언되어 있지만, 캐시는 논리적 상수성을 깨지 않는 lazy 계산 결과물이기 때문에 `mutable`로 선언해 쓰기를 허용한다.

### 2.4 동일한 멤버 변수들

위의 세 가지 차이를 제외하면, 두 factor의 private 멤버 변수는 완전히 동일하다.

```
int num_threads
double regularization_epsilon
NDTSearchMode search_mode
double correspondence_update_tolerance_rot
double correspondence_update_tolerance_trans
mutable Eigen::Isometry3d linearization_point
mutable Eigen::Isometry3d last_correspondence_point
mutable bool inv_cov_cached
std::shared_ptr<const SourceFrame> source
```

공개 메서드도 동일한 시그니처로 제공된다.

```
set_num_threads(int)
set_regularization_epsilon(double)
set_correspondence_update_tolerance(double rot, double trans)
set_search_mode(NDTSearchMode)
num_inliers() → int
inlier_fraction() → double
clone() → shared_ptr<IntegratedMatchingCostFactor>
```

두 factor를 교체 가능한(drop-in replaceable) 형태로 사용할 수 있는 이유가 여기에 있다. 타겟 복셀맵 타입과 대응 구조체만 바꾸면 나머지 설정 코드는 그대로 유지된다.

---

## 3. update_correspondences() 구현 비교

`update_correspondences()`는 LM 반복마다 호출되는 가장 빈번한 연산이다. 두 구현의 구조를 단계별로 해부한다.

### 3.1 공통 부분: tolerance 검사 및 이웃 오프셋 구성

두 factor의 tolerance 검사 로직은 완전히 동일하다. LM optimizer는 매 선형화 단계마다 `update_correspondences()`를 호출하지만, 포즈 변위가 임계치 미만이면 대응 재탐색을 생략한다. 불필요한 재계산을 막아 반복당 비용을 줄이는 핵심 최적화다.

```cpp
// include/ndt/impl/integrated_light_ndt_factor_impl.hpp, lines 90-129 (공통 패턴)
linearization_point = delta;

bool do_update = true;
if (correspondences.size() == frame::size(*source) && ...) {
  Eigen::Isometry3d diff = delta.inverse() * last_correspondence_point;
  double diff_rot = Eigen::AngleAxisd(diff.linear()).angle();
  double diff_trans = diff.translation().norm();
  if (diff_rot < tolerance_rot && diff_trans < tolerance_trans) {
    do_update = false;
  }
}

// neighbor_offsets 구성 — DIRECT1/7/27 분기도 양쪽 동일
```

`do_update = false`이면 `perpoint_task` 내부의 대응 탐색 분기를 건너뛰고 기존 `correspondences[]`를 재사용한다.

### 3.2 inv_cov_cache 구축 차이

역공분산 캐시는 처음 한 번만 구축하고, 이후 `inv_cov_cached` 플래그로 재구축을 막는다. 구조의 차이가 여기서부터 시작된다.

**LightNDT — 단일 루프**

```cpp
// include/ndt/impl/integrated_light_ndt_factor_impl.hpp, lines 131-139
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

각 복셀은 단일 `GaussianVoxel`이고, `voxel.cov`는 멤버 변수로 직접 접근한다. `inv_cov_cache`는 `std::vector<Eigen::Matrix4d>` — 복셀 인덱스로 바로 참조할 수 있는 1차원 배열이다.

**MixtureLightNDT — 이중 루프**

```cpp
// include/gmm/impl/integrated_mixture_light_ndt_factor_impl.hpp, lines 183-197
if (!inv_cov_cached) {
  const size_t num_voxels = target_voxels->num_voxels();
  inv_cov_cache.resize(num_voxels);
  for (size_t v = 0; v < num_voxels; v++) {
    const auto& voxel = target_voxels->lookup_voxel(v);
    const auto& comps = voxel.components();
    inv_cov_cache[v].resize(comps.size());
    for (size_t k = 0; k < comps.size(); k++) {
      inv_cov_cache[v][k] = compute_ndt_inverse_covariance(comps[k].cov, regularization_epsilon);
    }
  }
  inv_cov_cached = true;
}
```

각 복셀은 `GMMVoxel`이고, `voxel.components()`로 GMM 컴포넌트 벡터를 반환한다. `inv_cov_cache`는 `std::vector<std::vector<Eigen::Matrix4d>>` — 복셀마다 컴포넌트 수만큼 inner vector가 할당된다. `inv_cov_cache[v].resize(comps.size())`가 없으면 다음 단계에서 인덱스 접근이 UB가 된다.

두 구현 모두 `compute_ndt_inverse_covariance(cov, epsilon)`를 동일하게 호출한다. 내부는 고유값 분해 후 최솟값을 `epsilon`으로 clamping하는 정규화 과정이다.

### 3.3 핵심 분기: perpoint_task 대응 탐색

`perpoint_task`는 소스 포인트 하나를 받아 가장 가까운 후보를 winner-take-all 방식으로 선택한다. 두 구현의 루프 구조가 여기서 갈린다.

**LightNDT — 복셀 단위 단일 루프**

```cpp
// include/ndt/impl/integrated_light_ndt_factor_impl.hpp, lines 141-176
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
```

이웃 복셀 순회만 있고 내부 루프는 없다. 각 복셀이 후보 하나를 제공한다. `GaussianVoxel`의 `mean.w = 1`이고 변환된 소스 포인트도 `pt.w = 1`이므로, `diff = pt - voxel.mean`에서 `diff(3) = 0`이 자연스럽게 성립한다. 별도 보정이 필요 없다.

대응 결과는 `{mean, inv_cov, valid}` 세 필드다.

**MixtureLightNDT — 복셀 × 컴포넌트 이중 루프**

```cpp
// include/gmm/impl/integrated_mixture_light_ndt_factor_impl.hpp, lines 202-254
const auto perpoint_task = [&](int i)
{
  if (do_update) {
    correspondences[i].valid = false;

    Eigen::Vector4d pt = delta * frame::point(*source, i);
    Eigen::Vector3i coord = target_voxels->voxel_coord(pt);

    double min_mahalanobis = std::numeric_limits<double>::max();
    MixtureNdtCorrespondence best_corr;

    for (const auto& offset : neighbor_offsets) {
      Eigen::Vector3i neighbor_coord = coord + offset;
      const auto voxel_id = target_voxels->lookup_voxel_index(neighbor_coord);
      if (voxel_id < 0) {
        continue;
      }

      const auto& voxel = target_voxels->lookup_voxel(voxel_id);
      const auto& comps = voxel.components();

      for (size_t k = 0; k < comps.size(); k++) {
        if (inv_cov_cache[voxel_id].empty()) {
          continue;
        }

        const Eigen::Matrix4d& inv_cov = inv_cov_cache[voxel_id][k];

        Eigen::Vector4d diff = pt - comps[k].mean;
        diff(3) = 0.0;  // w-component 차이 제거
        double mahalanobis_dist = diff.transpose() * inv_cov * diff;

        if (mahalanobis_dist < min_mahalanobis) {
          min_mahalanobis = mahalanobis_dist;
          best_corr.mean = comps[k].mean;
          best_corr.inv_cov = inv_cov;
          best_corr.weight = comps[k].weight;
          best_corr.valid = true;
        }
      }
    }

    correspondences[i] = best_corr;
  }
};
```

이웃 복셀마다 컴포넌트 수만큼 내부 루프를 돈다. winner-take-all 경쟁은 복셀 경계를 넘어 전체 후보 풀에서 이루어진다. DIRECT7 + 컴포넌트 4개면 최대 28개 후보를 비교하는 셈이다.

`diff(3) = 0.0` 보정이 명시적으로 필요하다. EM이 반환하는 `comps[k].mean`은 3D 좌표를 `Eigen::Vector4d`로 패딩할 때 `w = 0`으로 초기화되기 때문이다 (자세한 이유는 3.4절 참고).

대응 결과는 `{mean, inv_cov, weight(π_k), valid}` 네 필드다. `weight` 필드가 추가된 것이 MixtureLightNDT의 가장 직접적인 구조 변화다.

### 3.4 diff(3) = 0 보정의 근본 원인

두 구현에서 `diff(3)` 처리 방식이 다른 이유는 복셀 구조가 동차 좌표를 다루는 방식이 다르기 때문이다.

**GaussianVoxel (LightNDT)**: 포인트가 삽입될 때 동차 좌표 `w = 1`을 포함한 `Eigen::Vector4d` 전체를 누적한다. 따라서 `mean.w`는 정규화 후에도 1이다. 변환된 소스 포인트 `pt`도 `delta * frame::point(...)` 연산으로 `w = 1`을 유지하므로, `diff.w = pt.w - mean.w = 1 - 1 = 0`이 자동으로 성립한다.

**GMMComponent (MixtureLightNDT)**: EM 알고리즘은 3D 공간에서 평균을 계산한 뒤 `Eigen::Vector4d`로 패딩한다. 이때 `w = 0`으로 초기화된다. 소스 포인트는 여전히 `pt.w = 1`이므로, `diff.w = 1 - 0 = 1`이 된다. `inv_cov`의 `(3,3)` 원소가 0이 아니면 이 차이가 마할라노비스 거리에 허위 비용으로 더해진다.

이 보정이 빠지면 대응 탐색에서 잘못된 컴포넌트가 선택되고, 결과적으로 정합 정확도가 크게 떨어진다. 실제로 Phase 3.6 버그 수정 전후 비교에서 `diff(3) = 0.0` 한 줄이 없었을 때 Mean Rotation Error가 4.577°였고, 보정 후 0.474°로 줄었다 (9.7배 개선).

### 3.5 계산 복잡도 비교

대응 탐색의 시간 복잡도를 형식적으로 표기하면 다음과 같다.

**LightNDT:**

$$O(N \cdot V)$$

여기서 $N$은 소스 포인트 수, $V$는 평균 이웃 복셀 수다. DIRECT7 모드면 $V \leq 7$이다.

**MixtureLightNDT:**

$$O(N \cdot V \cdot K)$$

$K$는 복셀당 평균 컴포넌트 수로, 보통 2~4다. $K = 4$이면 이론상 최악 4배 느리다.

실측 벤치마크에서는 LightNDT 479ms, MixtureLightNDT 1476ms로 약 3.1배 차이가 났다. 이론 상한(4배)보다 낮은 이유는 실제 복셀 히트율이 100%가 아니고, 캐시 재사용이나 조기 종료 등의 효과가 겹치기 때문이다.

---

## 4. evaluate() 구현 비교

`evaluate()`는 현재 포즈 추정치에서 비용과 Jacobian, Hessian을 계산해 optimizer에 돌려준다. `update_correspondences()`가 "어느 가우시안과 매칭할지"를 결정한다면, `evaluate()`는 "그 매칭이 얼마나 나쁜지"를 수치화한다.

### 4.1 비용 함수 수식

**LightNDT:**

$$C(T) = \sum_{i} \mathbf{r}_i^\top \Sigma_i^{-1} \mathbf{r}_i$$

**MixtureLightNDT:**

$$C(T) = \sum_{i} \pi_{k^*(i)} \cdot \mathbf{r}_i^\top \Sigma_{k^*(i)}^{-1} \mathbf{r}_i$$

$k^*(i)$는 포인트 $i$에 대해 winner-take-all로 선택된 컴포넌트 인덱스다. 유일한 수학적 차이는 $\pi_{k^*(i)}$ 가중치 하나다. 나머지 구조는 동일하다.

### 4.2 residual 계산 차이

두 구현 모두 `residual = mean_B - transed_mean_A`로 시작하지만, 후처리가 다르다.

**LightNDT:**

```cpp
// include/ndt/impl/integrated_light_ndt_factor_impl.hpp, lines 210-251
Eigen::Vector4d transed_mean_A = delta * mean_A;
Eigen::Vector4d residual = mean_B - transed_mean_A;
// residual(3) 보정 없음 — mean_B.w=1, transed_mean_A.w=1 → residual.w=0 자동 성립
```

`mean_B`는 `GaussianVoxel::mean`으로 `w = 1`, `transed_mean_A`도 `delta * mean_A`에서 `w = 1`을 유지하므로 `residual.w = 0`이 자연스럽게 보장된다.

**MixtureLightNDT:**

```cpp
// include/gmm/impl/integrated_mixture_light_ndt_factor_impl.hpp, lines 305-375
Eigen::Vector4d transed_mean_A = delta * mean_A;
Eigen::Vector4d residual = mean_B - transed_mean_A;
residual(3) = 0.0;  // ★ w-component 강제 보정 — mean_B.w=0, transed.w=1 → residual.w=-1
```

`mean_B`는 `GMMComponent::mean`으로 `w = 0`이다. 보정 없이 두면 `residual.w = -1`이 되고, `inv_cov`의 `(3,3)` 원소를 통해 허위 비용이 발생한다. `update_correspondences()`의 `diff(3) = 0.0` 보정과 동일한 문제가 `evaluate()`에서도 반복된다.

### 4.3 cost 계산 라인 차이

```cpp
// LightNDT — include/ndt/impl/integrated_light_ndt_factor_impl.hpp
const double cost = residual.transpose() * inv_cov_B * residual;

// MixtureLightNDT — include/gmm/impl/integrated_mixture_light_ndt_factor_impl.hpp
const double cost = pi_k * (residual.transpose() * inv_cov_B * residual).value();
```

두 라인은 겉보기엔 비슷하지만 타입 처리 방식이 다르다.

LightNDT에서 `residual.transpose() * inv_cov_B * residual`의 결과는 `Eigen::Matrix<double, 1, 1>`이다. `const double cost =`로 대입할 때 Eigen의 암시적 변환이 자동으로 스칼라를 꺼낸다.

MixtureLightNDT에서는 `pi_k *` 곱셈이 먼저 필요하다. `double * Eigen::Matrix<double,1,1>`은 컴파일되지 않으므로, `.value()`로 1×1 행렬에서 스칼라를 명시적으로 추출한 뒤 곱한다.

### 4.4 Hessian/gradient의 π_k 가중 차이

Jacobian을 계산한 뒤 Hessian과 gradient를 누적하는 코드가 핵심 분기점이다.

**LightNDT:**

```cpp
// include/ndt/impl/integrated_light_ndt_factor_impl.hpp
Eigen::Matrix<double, 6, 4> J_target_weighted = J_target.transpose() * inv_cov_B;
Eigen::Matrix<double, 6, 4> J_source_weighted = J_source.transpose() * inv_cov_B;

*H_target += J_target_weighted * J_target;
*H_source += J_source_weighted * J_source;
*H_target_source += J_target_weighted * J_source;
*b_target += J_target_weighted * residual;
*b_source += J_source_weighted * residual;
```

**MixtureLightNDT:**

```cpp
// include/gmm/impl/integrated_mixture_light_ndt_factor_impl.hpp
Eigen::Matrix<double, 6, 4> J_target_weighted = pi_k * J_target.transpose() * inv_cov_B;
Eigen::Matrix<double, 6, 4> J_source_weighted = pi_k * J_source.transpose() * inv_cov_B;

*H_target += J_target_weighted * J_target;
*H_source += J_source_weighted * J_source;
*H_target_source += J_target_weighted * J_source;
*b_target += J_target_weighted * residual;
*b_source += J_source_weighted * residual;
```

`pi_k *` 하나만 다르고 나머지는 완전히 동일하다.

수식으로 표현하면:

$$\text{LightNDT:} \quad H \mathrel{+}= J^\top \Sigma^{-1} J, \quad b \mathrel{+}= J^\top \Sigma^{-1} r$$

$$\text{MixtureLightNDT:} \quad H \mathrel{+}= \pi_k J^\top \Sigma^{-1} J, \quad b \mathrel{+}= \pi_k J^\top \Sigma^{-1} r$$

$\pi_k$의 효과는 단순히 스케일링이 아니다. 가중치가 낮은 컴포넌트(예: $\pi_k = 0.1$)는 Hessian 기여가 10% 수준으로 감쇠된다. Optimizer 입장에서는 신뢰도 높은 컴포넌트($\pi_k$가 큰 쪽)가 포즈 업데이트를 주도하고, 약한 컴포넌트는 부드럽게 영향을 준다. 단일 가우시안에 winner-take-all로 맞추는 LightNDT보다 수렴 경관(loss landscape)이 더 완만해지는 이유가 여기에 있다.

### 4.5 SE(3) Jacobian은 동일함

Jacobian 블록 계산 코드는 두 구현에서 한 글자도 다르지 않다.

```cpp
// 양쪽 모두 동일 — LightNDT: lines 210-251, MixtureLightNDT: lines 305-375
Eigen::Matrix<double, 4, 6> J_target = Eigen::Matrix<double, 4, 6>::Zero();
J_target.block<3, 3>(0, 0) = -gtsam::SO3::Hat(transed_mean_A.head<3>());
J_target.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

Eigen::Matrix<double, 4, 6> J_source = Eigen::Matrix<double, 4, 6>::Zero();
J_source.block<3, 3>(0, 0) = delta.linear() * gtsam::SO3::Hat(mean_A.template head<3>());
J_source.block<3, 3>(0, 3) = -delta.linear();
```

수식으로는:

$$J_\text{target} = \begin{bmatrix} -[T\mathbf{p}]_\times & I_3 \end{bmatrix}, \quad J_\text{source} = \begin{bmatrix} R[\mathbf{p}]_\times & -R \end{bmatrix}$$

여기서 $[\cdot]_\times$는 3D 벡터의 반대칭 행렬(skew-symmetric matrix)이고, $R$은 현재 상대 포즈의 회전 행렬이다.

Jacobian은 포즈와 포인트 사이의 기하학적 관계에서 유도된다. 비용 함수에 가중치 $\pi_k$가 붙든 아니든, SE(3) 위에서의 미분 구조 자체는 변하지 않는다. 가중치는 그 위에 스칼라 곱으로 얹힐 뿐이다.

---

## 5. 생성자 & 유틸리티 메서드 비교

### 5.1 생성자: 다운캐스트 타입의 차이

두 팩터 모두 기반 클래스인 `GaussianVoxelMap::ConstPtr`를 인자로 받는다. 런타임에 실제 타입으로 내려받기 위해 `std::dynamic_pointer_cast`를 사용하는데, 캐스트 대상 타입이 다르다.

```cpp
// LightNDT — integrated_light_ndt_factor_impl.hpp, lines 32, 57
target_voxels(std::dynamic_pointer_cast<const GaussianVoxelMapCPU>(target_voxels))

// MixtureLightNDT — integrated_mixture_light_ndt_factor_impl.hpp, lines 40, 71
target_voxels(std::dynamic_pointer_cast<const GMMVoxelMapCPU>(target_voxels))
```

캐스트 실패 시 `nullptr`가 돌아오고, 곧바로 에러를 출력하고 `abort()`를 호출한다. 에러 메시지도 의도적으로 다르게 작성되어 있어서 디버깅 시 어떤 팩터에서 문제가 발생했는지 즉시 파악할 수 있다.

```cpp
// LightNDT: "has not been created" — 복셀맵 자체가 없는 상황
if (!this->target_voxels) {
  std::cerr << "error: target voxelmap has not been created!!" << std::endl;
  abort();
}

// MixtureLightNDT: "is not a GMMVoxelMapCPU" — 복셀맵은 있지만 타입이 틀린 상황
if (!this->target_voxels) {
  std::cerr << "error: target voxelmap is not a GMMVoxelMapCPU!!" << std::endl;
  abort();
}
```

LightNDT의 에러 메시지는 복셀맵이 아예 생성되지 않았음을 암시한다. MixtureLightNDT의 에러 메시지는 복셀맵이 존재하되 `GMMVoxelMapCPU`가 아닌 다른 타입(예: `GaussianVoxelMapCPU`)이 전달됐다는 사실을 명확히 알려준다.

### 5.2 `print()` 출력 내용

두 팩터의 `print()` 구현은 팩터 이름만 다르고 나머지는 동일하다.

```cpp
// LightNDT — impl.hpp, line 74
std::cout << s << "IntegratedLightNDTFactor";

// MixtureLightNDT — impl.hpp, line 95
std::cout << s << "IntegratedMixtureLightNDTFactor";
```

이후 키 정보, `target_resolution`, `|source|` 포인트 수를 출력하는 코드는 한 글자도 다르지 않다. GTSAM 그래프를 디버깅할 때 `graph.print()`를 호출하면 팩터 이름으로 어떤 팩터인지 구별할 수 있다.

### 5.3 `memory_usage()`: 대응 구조체 크기의 차이

```cpp
// LightNDT — impl.hpp, line 86
return sizeof(*this) + sizeof(NdtCorrespondence) * correspondences.capacity();

// MixtureLightNDT — impl.hpp, line 111
return sizeof(*this) + sizeof(MixtureNdtCorrespondence) * correspondences.capacity();
```

`MixtureNdtCorrespondence`는 `NdtCorrespondence`에 `double weight` 필드가 추가된 구조체다(헤더 파일 lines 33-36). `weight`는 8바이트이지만, Eigen 정렬 패딩과 `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` 때문에 실제 구조체 크기 차이는 컴파일러와 플랫폼에 따라 다소 달라질 수 있다. 소스 포인트가 N개라면 총 추가 메모리는 대략 $N \times 8$ 바이트 수준이다.

추가로, MixtureLightNDT는 `inv_cov_cache`가 2차원 벡터(`std::vector<std::vector<Eigen::Matrix4d>>`)이므로 `sizeof(*this)` 자체도 1차원 벡터를 쓰는 LightNDT보다 약간 크다. 이 차이는 2절에서 다룬 캐시 차이의 메모리 측면 귀결이다.

### 5.4 `get_target()` 반환 타입

헤더 파일에서 바로 인라인으로 정의되는 getter이다.

```cpp
// LightNDT — integrated_light_ndt_factor.hpp, line 64
const std::shared_ptr<const GaussianVoxelMapCPU>& get_target() const { return target_voxels; }

// MixtureLightNDT — integrated_mixture_light_ndt_factor.hpp, lines 115-118
const std::shared_ptr<const GMMVoxelMapCPU>& get_target() const
{
  return target_voxels;
}
```

반환 타입이 다르므로, 팩터를 받아서 복셀맵에 접근하는 코드는 팩터 타입에 맞게 작성해야 한다. `GMMVoxelMapCPU`에만 있는 GMM 컴포넌트 접근자(예: `voxel.components`)를 쓰려면 반드시 `get_target()`이 `GMMVoxelMapCPU`를 돌려줘야 한다.

### 5.5 나머지 메서드: 완전히 동일

아래 메서드들은 두 팩터에서 코드가 동일하다.

| 메서드 | 동일한 이유 |
|---|---|
| `set_num_threads(int n)` | OMP 스레드 수 설정, 타입 무관 |
| `set_regularization_epsilon(double eps)` | $\varepsilon$ 설정 후 `inv_cov_cached = false` |
| `set_correspondence_update_tolerance(double, double)` | 포즈 변위 임계치 설정 |
| `set_search_mode(NDTSearchMode)` | DIRECT1/7/27 탐색 모드 |
| `num_inliers()` / `inlier_fraction()` | `c.valid` 필드 확인 로직 |
| `clone()` | `new FooFactor_(*this)` 딥 카피 패턴 |

기본값도 동일하다: `num_threads=1`, `regularization_epsilon=1e-3`, `search_mode=DIRECT7`, 허용 오차 회전/병진 모두 `0.0`.

---

## 6. 왜 이런 차이가 필요한가 — 수학적 동기

### 6.1 단일 가우시안에서 GMM으로의 확장

LightNDT는 각 복셀 안의 포인트들을 하나의 가우시안 $\mathcal{N}(\mu, \Sigma)$으로 요약한다. 복셀 안에 하나의 평탄한 표면만 있다면 이 근사는 충분하다. 그런데 실제 LiDAR 데이터에는 복셀 하나에 벽 모서리, 기둥 옆면, 바닥 등 이질적인 표면이 동시에 들어오는 경우가 흔하다.

이런 상황에서 단일 가우시안은 두 표면의 중간 어딘가를 향하는 넓고 납작한 분포를 만든다. 그 결과 역공분산 행렬이 과도하게 큰 고유값을 가지게 되고, 정합 비용 함수의 기울기가 모호해진다. GMM은 이 문제를 K개의 컴포넌트로 분리해 각 표면을 개별적으로 모델링함으로써 해결한다.

$$\mathcal{N}(\mu, \Sigma) \quad \longrightarrow \quad \sum_{k=1}^{K} \pi_k \, \mathcal{N}(\mu_k, \Sigma_k)$$

winner-take-all 전략은 포인트 $p_i$를 가장 가까운 컴포넌트 하나에만 대응시킨다. 이렇게 하면 연속적인 소프트 할당 없이도 복잡한 기하 구조를 다룰 수 있고, 기존 NDT 코드 구조(하나의 대응, 하나의 Hessian 기여)를 그대로 재사용할 수 있다.

### 6.2 $\pi_k$ 가중치의 역할

혼합 가중치 $\pi_k$는 EM 알고리즘이 수렴한 뒤 각 컴포넌트가 전체 포인트 집합에서 차지하는 상대적 책임을 나타낸다. 포인트 수가 적거나 공분산이 불안정한 컴포넌트는 낮은 $\pi_k$를 받는다.

Hessian 누적 단계에서 이 값은 신뢰도 가중치로 작용한다.

$$H \mathrel{+}= \pi_k \, J^\top \Sigma_k^{-1} J$$

$\pi_k = 0.1$인 컴포넌트는 $\pi_k = 1.0$인 컴포넌트에 비해 Hessian에 10%의 기여만 한다. Optimizer 입장에서 이는 약한 컴포넌트의 정합 신호를 자동으로 감쇠시키는 효과다. 임시로 튀어나온 노이즈 클러스터가 포즈 추정을 끌어당기는 현상이 완화된다.

단일 가우시안(LightNDT)에서는 이런 가중치가 없다. 복셀에 대응이 발견되면 항상 $1.0$의 가중치로 Hessian에 더해진다.

### 6.3 `diff(3) = 0` 보정의 필연성

섹션 3에서 다룬 `diff(3) = 0` 보정은 임시 처리가 아니다. 두 데이터 구조의 좌표 표현 방식이 다르기 때문에 발생하는 필연적인 불일치다.

`GaussianVoxel`(LightNDT 측)은 포인트를 `Eigen::Vector4d`로 저장할 때 동차 좌표 $w=1$을 사용한다. 변환 행렬과의 곱셈 결과도 $w=1$로 유지된다. diff의 4번째 성분은 자연스럽게 $1 - 1 = 0$이 된다.

반면 `GMMComponent`(MixtureLightNDT 측)의 평균 $\mu_k$는 EM 알고리즘이 순수 3D 벡터로 계산한 뒤 4D 패딩으로 확장한 것이다. 패딩 값은 $w=0$이다. 변환된 소스 포인트는 $w=1$이므로 diff의 4번째 성분은 $0 - 1 = -1$이 된다. 이를 그냥 두면 역공분산과의 곱셈에서 수치 쓰레기가 발생한다.

```
GaussianVoxel:    mean = [μ_x, μ_y, μ_z, 1]  (동차 좌표)
GMMComponent:     mean = [μ_x, μ_y, μ_z, 0]  (3D + 제로 패딩)
```

`diff(3) = 0` 한 줄은 이 설계 차이를 보정하는 정확한 수술이다.

### 6.4 2D 캐시의 트레이드오프

LightNDT의 `inv_cov_cache`는 `std::vector<Eigen::Matrix4d>`이다. 복셀 V개에 대해 $V \times 128$ 바이트를 차지한다.

MixtureLightNDT는 `std::vector<std::vector<Eigen::Matrix4d>>`로 확장된다. K개 컴포넌트를 모두 캐시하므로 $V \times K \times 128$ 바이트가 필요하다. K=4이면 메모리가 4배 늘어난다.

그럼에도 2D 캐시를 선택한 이유는 LM 반복 횟수와 관련이 있다. `inv_cov_cache`는 `update_correspondences()` 최초 호출 시 한 번만 채워진다. 이후 LM이 수십 번 반복하는 동안 `evaluate()`는 캐시에서 $O(1)$로 역공분산을 읽기만 한다. 캐시 없이 매 반복마다 `compute_ndt_inverse_covariance()`를 K번 호출하면, LM 반복이 $I$회라면 총 $V \times K \times I$번의 고유값 분해가 필요하다. K가 클수록 이 비용이 선형으로 증가하므로 캐시의 이점이 더 커진다.

---

## 7. 요약 비교 테이블

| 비교 항목 | LightNDT | MixtureLightNDT | 비고 |
|---|---|---|---|
| 타겟 복셀맵 타입 | `GaussianVoxelMapCPU` | `GMMVoxelMapCPU` | 헤더 인클루드도 다름 |
| 대응 구조체 | `NdtCorrespondence` | `MixtureNdtCorrespondence` | Mixture 쪽에 `weight` 필드 추가 |
| `inv_cov_cache` 차원 | `vector<Matrix4d>` (1D, 복셀×1) | `vector<vector<Matrix4d>>` (2D, 복셀×K) | K=4이면 메모리 4배 |
| 대응 탐색 루프 구조 | 복셀 순회만 | 복셀 순회 + 컴포넌트 내부 순회 | 중첩 루프 추가 |
| `diff(3)` / `residual(3)` 보정 | 없음 (자연스럽게 0) | `diff(3) = 0` 명시적 강제 | $w$=0 패딩 설계 차이 보정 |
| 비용 함수 수식 | $r^\top \Sigma^{-1} r$ | $\pi_k \cdot r^\top \Sigma_k^{-1} r$ | $\pi_k$ 혼합 가중치 추가 |
| Hessian 가중 방식 | 가중치 없음 (암묵적 1.0) | `pi_k *` 스칼라 곱 | cost 라인에서 `.value()`는 Eigen 1×1 행렬→스칼라 추출 |
| 복잡도 (대응 탐색) | $O(N \cdot V)$ | $O(N \cdot V \cdot K)$ | K = 복셀당 컴포넌트 수 |
| 생성자 다운캐스트 타입 | `GaussianVoxelMapCPU` | `GMMVoxelMapCPU` | 실패 시 에러 메시지도 다름 |
| `get_target()` 반환 타입 | `shared_ptr<const GaussianVoxelMapCPU>` | `shared_ptr<const GMMVoxelMapCPU>` | 인터페이스 호환 불가 |
| 에러 메시지 | "target voxelmap has not been created!!" | "target voxelmap is not a GMMVoxelMapCPU!!" | 원인 진단 정밀도 차이 |
| `memory_usage` 기준 구조체 | `NdtCorrespondence` | `MixtureNdtCorrespondence` | Mixture 쪽이 약간 큼 |

---

## 8. 학습 확인 질문

1. **`diff(3) = 0` 보정이 MixtureLightNDT에만 필요한 이유**

   LightNDT는 `GaussianVoxel`의 mean이 동차 좌표 $w=1$로 저장되어 있어서 diff의 4번째 성분이 자연스럽게 0이 된다. 반면 MixtureLightNDT는 EM 알고리즘이 산출한 GMM 컴포넌트 mean이 $w=0$ 패딩으로 저장된다. 변환된 소스 포인트의 $w$는 1이므로 보정 없이 두면 diff(3) = -1이 된다. 이 불일치가 역공분산 곱셈에 미치는 수치적 영향을 구체적으로 설명하시오.

   <details><summary>힌트</summary>섹션 3.2 및 섹션 6.3 참고. Σ^{-1}의 4행 4열 성분이 0이더라도 diff(3)≠0이면 중간 계산 단계에서 쓰레기 값이 누적될 수 있다.</details>

2. **$\pi_k$가 Hessian에 미치는 영향**

   두 소스 포인트 $p_a$, $p_b$가 각각 $\pi_{k_a} = 0.9$, $\pi_{k_b} = 0.1$인 컴포넌트에 대응된다고 가정하자. 동일한 Jacobian $J$와 역공분산 $\Sigma^{-1}$을 가진다고 할 때, 두 포인트가 전체 Hessian에 기여하는 상대적 크기를 수식으로 나타내고, 이것이 포즈 업데이트 방향에 어떤 의미를 갖는지 설명하시오.

   <details><summary>힌트</summary>섹션 4.4 및 섹션 6.2 참고. Hessian은 J^T Σ^{-1} J의 가중합이다.</details>

3. **`inv_cov_cache`가 2D여야 하는 이유**

   MixtureLightNDT의 `inv_cov_cache`를 1D `vector<Matrix4d>`로 단순화하면 어떤 문제가 발생하는가? LM 반복 중 `evaluate()`가 캐시에서 역공분산을 읽는 방식과 연결 지어 설명하시오.

   <details><summary>힌트</summary>섹션 2.3 및 섹션 6.4 참고. 복셀 v, 컴포넌트 k를 함께 인덱싱해야 한다.</details>

4. **`GaussianVoxelMapCPU`를 MixtureLightNDT 생성자에 전달하면**

   `GaussianVoxelMapCPU` 객체를 `GaussianVoxelMap::ConstPtr`로 래핑해 `IntegratedMixtureLightNDTFactor_` 생성자에 넘기면 어떤 일이 일어나는가? 런타임에서 어떤 코드 경로가 실행되고, 최종적으로 프로그램이 어떻게 종료되는지 생성자 코드를 근거로 설명하시오.

   <details><summary>힌트</summary>섹션 5.1 참고. `dynamic_pointer_cast`는 타입 불일치 시 nullptr를 반환한다.</details>

5. **복잡도 $O(N \cdot V)$ vs $O(N \cdot V \cdot K)$의 실제 영향**

   소스 포인트 수 $N=5000$, 이웃 복셀 탐색 수 $V=7$ (DIRECT7), GMM 컴포넌트 수 $K=4$라고 가정하자. LightNDT와 MixtureLightNDT의 대응 탐색 시 마할라노비스 거리 계산 횟수를 각각 구하고, 그 차이가 `update_correspondences()` 실행 시간에 미치는 영향을 논하시오.

   <details><summary>힌트</summary>섹션 2.2 및 섹션 7의 복잡도 행 참고. 대응 탐색은 LM 반복마다 수행되지 않을 수도 있다(tolerance 조건).</details>

6. **`.value()` 호출이 MixtureLightNDT 비용 계산에만 필요한 이유**

   MixtureLightNDT의 비용 계산 라인은 `pi_k * (residual.transpose() * inv_cov_B * residual).value()`이고, LightNDT는 `residual.transpose() * inv_cov_B * residual`이다. `.value()`는 Eigen의 `Matrix<double, 1, 1>` 타입에서 스칼라 `double`을 추출하는 메서드다. LightNDT에서는 `.value()` 없이 `const double cost =`에 대입하는데 왜 컴파일이 되는가? 그리고 MixtureLightNDT에서 `.value()` 없이 `pi_k *`를 곱하면 왜 컴파일 오류가 발생하는가? Eigen의 암시적 스칼라 변환 규칙을 근거로 설명하시오.

   <details><summary>힌트</summary>섹션 4.3 참고. Eigen은 1×1 행렬을 `double`로 암시적 변환할 수 있지만, `double * Eigen::Matrix<double,1,1>` 곱셈은 Eigen 표현식이 아닌 스칼라-행렬 곱으로 해석되어 타입 불일치가 생긴다.</details>
