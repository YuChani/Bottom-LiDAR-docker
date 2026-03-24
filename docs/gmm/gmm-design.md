# GMM 기반 복셀맵 설계 문서

> **범위**: 아키텍처, 수학적 근거, 코드 동작 상세.  
> **작업 이력 / 벤치마크 결과** → [`gmm-work-status.md`](./gmm-work-status.md) 참조.

---

## 목차

1. [설계 동기](#1-설계-동기)
2. [아키텍처 개요](#2-아키텍처-개요)
3. [Phase 1: GMMVoxel + Reservoir Sampling](#3-phase-1-gmmvoxel--reservoir-sampling)
4. [Phase 2: EM Backend (Armadillo)](#4-phase-2-em-backend-armadillo)
5. [Phase 3: IntegratedMixtureLightNDTFactor](#5-phase-3-integratedmixturelightndtfactor)
6. [Phase 3.5–3.6: 벤치마크 통합 + 버그 수정](#6-phase-3536-벤치마크-통합--버그-수정)
7. [데이터 흐름 종합](#7-데이터-흐름-종합)
8. [수식 – 코드 매핑 테이블](#8-수식--코드-매핑-테이블)
9. [설계 결정 근거 (Design Rationale)](#9-설계-결정-근거-design-rationale)
10. [하이퍼파라미터 튜닝 가이드](#10-하이퍼파라미터-튜닝-가이드)
11. [LightNDT vs MixtureLightNDT 구조 비교](#11-lightndt-vs-mixturelightndt-구조-비교)

---

## 1. 설계 동기

기존 `GaussianVoxelMapCPU`는 복셀당 **단일 가우시안**(single Gaussian)으로 포인트 분포를 모델링한다. 이는 복셀 내 포인트가 단봉(unimodal) 분포일 때 효과적이지만, 환경의 경계면·모서리 등에서 **다봉(multimodal)** 분포가 발생하면 표현력이 부족하다.

GMM(Gaussian Mixture Model)은 복셀당 $K$개 가우시안 컴포넌트로 분포를 근사하여:

- **경계면 교차 복셀**에서 각 면을 별도 컴포넌트로 분리
- **혼합 가중치** $\pi_k$를 통해 컴포넌트별 기여도 반영
- **Winner-take-all** correspondence로 계산량을 억제하면서 다봉 이점을 확보

벤치마크에서 MixtureLightNDT는 **전체 8개 factor 중 최고 rotation 정확도**(Mean R = 0.474°)를 달성했다.

---

## 2. 아키텍처 개요

### 2.1 클래스 계층 구조

```
GaussianVoxelMap (추상 인터페이스)
├── GaussianVoxelMapCPU (기존: 단일 가우시안, IncrementalVoxelMap<GaussianVoxel>)
└── GMMVoxelMapCPU      (신규: GMM, IncrementalVoxelMap<GMMVoxel>)
```

`GMMVoxelMapCPU`는 **다중 상속**으로 두 개의 부모를 갖는다:

| 부모 | 역할 |
|------|------|
| `GaussianVoxelMap` | 추상 인터페이스 (`voxel_resolution()`, `insert()`, `save_compact()`) |
| `IncrementalVoxelMap<GMMVoxel>` | LRU 공간 해시맵 기반 저장 엔진 |

이 설계로 기존 `GaussianVoxelMap::Ptr`을 기대하는 코드(`run_optimization()` 등)에 **다형성**으로 GMM 복셀맵을 주입할 수 있다. 다이아몬드 상속은 없다.

### 2.2 파일 구조

```
include/gmm/
├── gmm_voxelmap_cpu.hpp                         # GMMComponent, GMMVoxel, GMMVoxelMapCPU 선언
├── mixture_em_backend.hpp                        # GMMFitParams, GMMFitResult, fit_gmm() 선언
├── integrated_mixture_light_ndt_factor.hpp       # MixtureNdtCorrespondence, Factor 선언
└── impl/
    └── integrated_mixture_light_ndt_factor_impl.hpp  # Factor 템플릿 구현

src/gmm/
├── gmm_voxelmap_cpu.cpp                         # GMMVoxel, GMMVoxelMapCPU 구현
├── mixture_em_backend.cpp                       # Armadillo EM 구현
├── integrated_mixture_light_ndt_factor.cpp      # 명시적 템플릿 인스턴스화
└── CMakeLists.txt                               # 빌드 설정
```

### 2.3 의존성 방향

```
main.cpp
  ↓ include "gmm/gmm_voxelmap_cpu.hpp"
  ↓ include "gmm/integrated_mixture_light_ndt_factor.hpp"
  ↓ include "gmm/impl/integrated_mixture_light_ndt_factor_impl.hpp"

Factor (impl) → GMMVoxelMapCPU → mixture_em_backend → Armadillo
                       ↓
              IncrementalVoxelMap<GMMVoxel>  (thirdparty/gtsam_points/)
```

**include 규칙**: 프로젝트 소유 코드는 `"gmm/..."` (quoted), thirdparty는 `<gtsam_points/...>` (angle-bracket).

---

## 3. Phase 1: GMMVoxel + Reservoir Sampling

### 3.1 GMMComponent 구조체

```cpp
// include/gmm/gmm_voxelmap_cpu.hpp
struct GMMComponent {
  Eigen::Vector4d mean;    // μ_k, 동차 좌표 (w=0)
  Eigen::Matrix4d cov;     // Σ_k, 4×4 (상위-좌측 3×3만 유효)
  double weight;           // π_k ∈ [0,1], Σ π_k = 1
};
```

**w=0 불변식**: `mean`의 4번째 성분은 항상 0이다. `GaussianVoxel`은 w=1을 사용하는데, GMM은 w=0으로 통일하여 EM backend(Armadillo 3D)와의 변환을 단순화한다. 이 차이가 Phase 3.6 w-component leakage 버그의 근본 원인이 된다 ([§6.2](#62-w-component-leakage-fix) 참조).

### 3.2 GMMVoxel 구조체

복셀 단위로 포인트를 수집하고, `finalize()` 호출 시 GMM을 피팅한다.

| 멤버 | 타입 | 역할 |
|------|------|------|
| `reservoir_` | `vector<Vector4d>` | Algorithm R 샘플 버퍼 (최대 `capacity`개) |
| `total_points_seen_` | `size_t` | 관측된 전체 포인트 수 $N$ |
| `dirty_` | `bool` | `add()` 후 true, `finalize()` 후 false |
| `rng_` | `mt19937(42)` | 복셀별 고정 시드 RNG (재현성 보장) |
| `components_` | `vector<GMMComponent>` | EM 피팅 결과 ($K$개 컴포넌트) |
| `cached_setting_` | `Setting` | 마지막 finalize 시 설정 스냅샷 |

#### GMMVoxel::Setting — EM 하이퍼파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `max_components` | 3 | 복셀당 최대 가우시안 수 $K$ |
| `max_em_iterations` | 20 | EM 최대 반복 |
| `convergence_tol` | 1e-4 | EM 로그-우도 변화량 수렴 임계치 |
| `covariance_regularization` | 1e-3 | $\Sigma_k \mathrel{+}= \varepsilon \cdot I_3$ |
| `min_weight_threshold` | 0.01 | $\pi_k < \tau$인 컴포넌트 pruning |
| `reservoir_capacity` | 256 | 저수지 최대 크기 $C$ |

### 3.3 Algorithm R (Vitter, 1985) — Reservoir Sampling

스트리밍 환경에서 전체 $N$개 포인트 중 $C$개의 **균일 무작위 표본**을 유지하는 온라인 알고리즘.

**수학적 보장**:

$$P(\text{포인트 } i \text{ 가 최종 저수지에 존재}) = \frac{C}{N}, \quad \forall i \in \{1, \ldots, N\}$$

**알고리즘** (`GMMVoxel::add()`):

```
입력: 새 포인트 p_i, 현재 저수지 R (크기 ≤ C)
1. N ← N + 1  (total_points_seen 증가)
2. p_i.w ← 0  (동차 좌표 w=0 강제)
3. if |R| < C:
     R.push_back(p_i)          // 채우기 단계
   else:
     j ~ Uniform(0, N-1)
     if j < C:
       R[j] ← p_i              // 확률 C/N으로 교체
4. dirty ← true
```

**코드 위치**: `src/gmm/gmm_voxelmap_cpu.cpp:32-54`

**설계 결정**: Reservoir는 `finalize()` 후에도 유지된다. 이는 warm-start 재진입 시 기존 데이터를 활용하기 위함이다.

### 3.4 frame::traits 특수화

`GMMVoxel`을 gtsam_points의 프레임 인터페이스에 연결하는 3단계 traits 체인:

1. **`traits<GMMVoxel>`**: 컴포넌트 수 = `size()`, `point(v, k)` = k-번째 mean, `cov(v, k)` = k-번째 cov
2. **`traits<IncrementalVoxelMap<GMMVoxel>>`**: 전역 flat 인덱스 → (복셀, 로컬 k) 매핑. **값(by-value) 반환**으로 dangling ref 방지
3. **`traits<GMMVoxelMapCPU>`**: `IncrementalVoxelMap<GMMVoxel>`에 위임

> ⚠️ `point()`는 반드시 **값(value) 반환**이어야 한다. `GMMComponent::mean`은 벡터 원소의 임시 복사본이므로 참조(reference) 반환 시 dangling 위험.

---

## 4. Phase 2: EM Backend (Armadillo)

### 4.1 개요

`fit_gmm()` 함수는 Armadillo의 `gmm_full::learn()`을 감싸서 Eigen 벡터 입출력을 제공한다.

```
fit_gmm(points, params)                    → Cold-start (random_spread 초기화)
fit_gmm(points, params, initial_comps)     → Warm-start (keep_existing 초기화)
```

### 4.2 Cold-start EM

**입력**: 저수지 포인트 `vector<Vector4d>`, `GMMFitParams`  
**출력**: `GMMFitResult { components, converged, iterations_run }`

**수학**:

EM 알고리즘은 로그-우도를 반복적으로 최대화한다:

$$\mathcal{L} = \sum_{i=1}^{N} \log \left( \sum_{k=1}^{K} \pi_k \cdot \mathcal{N}(p_i \mid \mu_k, \Sigma_k) \right)$$

**E-step**: 각 포인트 $p_i$의 컴포넌트 $k$에 대한 책임도(responsibility):

$$\gamma_{ik} = \frac{\pi_k \cdot \mathcal{N}(p_i \mid \mu_k, \Sigma_k)}{\sum_{j=1}^{K} \pi_j \cdot \mathcal{N}(p_i \mid \mu_j, \Sigma_j)}$$

**M-step**: 파라미터 갱신:

$$N_k = \sum_{i=1}^{N} \gamma_{ik}, \quad \pi_k = \frac{N_k}{N}$$

$$\mu_k = \frac{1}{N_k} \sum_{i=1}^{N} \gamma_{ik} \cdot p_i$$

$$\Sigma_k = \frac{1}{N_k} \sum_{i=1}^{N} \gamma_{ik} (p_i - \mu_k)(p_i - \mu_k)^T + \varepsilon \cdot I_3$$

**코드 흐름** (`src/gmm/mixture_em_backend.cpp`):

```
1. N < 2 → single_component_fallback()
2. K = min(max_components, N)
3. eigen_to_arma(): Eigen 4D → Armadillo 3D (w 제거)
4. arma::gmm_full::learn(data, K, eucl_dist, random_spread, 20, max_iters, tol)
5. extract_result(): Armadillo → Eigen + 정칙화 + pruning + 가중치 재정규화
```

### 4.3 Warm-start EM

이전 `finalize()` 결과를 초기값으로 사용하여 수렴 속도를 높인다.

**핵심 처리**:

1. **이중 정칙화 방지**: `set_params()` 전에 기존 컴포넌트의 공분산에서 $\varepsilon \cdot I$를 **제거**한 뒤 Armadillo에 전달. `extract_result()`가 EM 후 다시 추가하므로 이중 적용 방지.

```cpp
// src/gmm/mixture_em_backend.cpp:233-239
val -= params.covariance_regularization;  // ε·I 제거
val = std::max(val, 1e-6);               // 음수 방지
```

2. **K 불일치 폴백**: `initial_components.size() > params.max_components`이면 cold-start로 폴백

3. **`keep_existing` 모드**: k-means 사전 반복 0회, set_params() 초기값에서 바로 EM 시작

### 4.4 후처리 파이프라인

`extract_result()` 함수의 3단계 후처리:

```
1. Armadillo → Eigen 변환: 3D mean → 4D(w=0), 3x3 cov → 4x4
2. 사후 정칙화: Σ_k += ε·I₃
3. Pruning: π_k < min_weight_threshold인 컴포넌트 제거
   → 전부 제거 시: 가중 평균(Σ π_k·μ_k)으로 단일 컴포넌트 폴백
4. 가중치 재정규화: π_k' = π_k / Σ π_j (Σ π_k = 1 보장)
```

### 4.5 Fallback 전략

| 조건 | 처리 |
|------|------|
| N = 0 | 빈 결과 (components = []) |
| N = 1 | `single_component_fallback()` — 표본 평균 + ε·I |
| EM 실패 | `single_component_fallback()` |
| All-pruned | 가중 평균으로 단일 컴포넌트 생성 |
| Warm-start 실패 | Cold-start로 재시도 |
| K > max_components | Cold-start로 폴백 |

---

## 5. Phase 3: IntegratedMixtureLightNDTFactor

### 5.1 개요

`IntegratedMixtureLightNDTFactor_`는 `IntegratedMatchingCostFactor`를 상속하며, `IntegratedLightNDTFactor_`를 미러링하는 구조다. 차이점은 복셀당 **K개 GMM 컴포넌트**를 순회하여 **winner-take-all** 대응을 선택하고, **π_k 가중 비용**을 계산하는 것.

### 5.2 MixtureNdtCorrespondence

```cpp
struct MixtureNdtCorrespondence {
  Eigen::Vector4d mean;      // μ_{k*} — 선택된 컴포넌트의 평균
  Eigen::Matrix4d inv_cov;   // Σ_{k*}^{-1} — 정규화된 역공분산
  double weight;             // π_{k*} — 혼합 가중치
  bool valid;                // 유효한 대응 여부
};
```

LightNDT의 `NdtCorrespondence`와 비교:

| 필드 | NdtCorrespondence | MixtureNdtCorrespondence |
|------|-------------------|--------------------------|
| mean | ✅ | ✅ |
| inv_cov | ✅ | ✅ |
| one_over_z | ✅ (정규화 상수) | ❌ (π_k로 대체) |
| **weight** | ❌ | ✅ (π_k) |
| valid | ✅ | ✅ |

### 5.3 inv_cov_cache — 역공분산 사전 계산

LightNDT가 `inv_cov_cache[voxel_id]` (1D)인 것과 달리, GMM 변형은 2D 캐시를 사용:

$$\texttt{inv\_cov\_cache[v][k]} = \Sigma_k^{-1}$$

**계산 방법**: `compute_ndt_inverse_covariance()`는 고유값 분해 후 작은 고유값을 클램프:

$$\lambda_i' = \max(\lambda_i, \varepsilon \cdot \lambda_{\max})$$

$$\Sigma^{-1} = V \cdot \text{diag}(1/\lambda_1', 1/\lambda_2', 1/\lambda_3', 0) \cdot V^T$$

4번째 고유값(w 차원)은 자연스럽게 0이므로 `inv_cov(3,:) = inv_cov(:,3) = 0`이 보장된다. 단, `diff(3)=0` 불변식이 깨지면 `inv_cov(3,3)` ≈ 20,000~35,000의 허위 비용이 발생한다 (Phase 3.6 버그의 메커니즘).

**캐시 무효화**: `set_regularization_epsilon()` 호출 시 `inv_cov_cached = false`로 리셋.

**메모리**: $V \times K \times 128$ bytes ≈ 3.84 MB (10k 복셀 × 3 컴포넌트).

### 5.4 update_correspondences() — 대응 탐색

**알고리즘**: 각 소스 포인트 $p_i$에 대해 이웃 복셀의 모든 GMM 컴포넌트를 순회하여 **마할라노비스 거리가 최소인 컴포넌트**를 선택.

$$k^*(i) = \arg\min_k \; d_{ik}^T \Sigma_k^{-1} d_{ik}$$

여기서 $d_{ik} = T \cdot p_i - \mu_k$, $d_{ik}(3) = 0$ (w-component 강제).

**이웃 탐색 모드** (`NDTSearchMode`):

| 모드 | 이웃 수 | 설명 |
|------|---------|------|
| `DIRECT1` | 1 | 현재 복셀만 |
| `DIRECT7` | 7 | 현재 + 6면 이웃 (기본값) |
| `DIRECT27` | 27 | 3×3×3 전체 |

**포즈 변위 허용 오차**: 직전 탐색 이후 회전/병진 변위가 임계치 미만이면 대응 재탐색을 생략하여 연산량 절감.

**코드 위치**: `include/gmm/impl/integrated_mixture_light_ndt_factor_impl.hpp:121-259`

### 5.5 비용 함수

$$C(T) = \sum_{i=1}^{N_s} \pi_{k^*(i)} \cdot r_i^T \Sigma_{k^*(i)}^{-1} r_i$$

여기서 잔차(residual)는 **GTSAM 컨벤션** (target - transformed source):

$$r_i = \mu_{k^*} - T \cdot p_i, \quad r_i(3) = 0$$

### 5.6 SE(3) Jacobian

SE(3) 리 대수 매개변수화 $\xi = (\varphi, \rho) \in \mathfrak{se}(3)$에서:

**Target Jacobian** (target 포즈에 대한 잔차 미분):

$$J_{\text{target}} = \begin{bmatrix} -[T \cdot p_i]_\times & I_3 \\ 0 & 0 \end{bmatrix} \in \mathbb{R}^{4 \times 6}$$

**Source Jacobian** (source 포즈에 대한 잔차 미분):

$$J_{\text{source}} = \begin{bmatrix} R \cdot [p_i]_\times & -R \\ 0 & 0 \end{bmatrix} \in \mathbb{R}^{4 \times 6}$$

여기서 $[\cdot]_\times$는 `gtsam::SO3::Hat()` (skew-symmetric 행렬).

4행은 항상 0이므로, `residual(3)=0` + `J[:,3]=0` → w 차원이 비용에 기여하지 않음.

### 5.7 π_k 가중 Gauss-Newton Hessian

**Hessian 근사** (포인트별 누적):

$$H \mathrel{+}= \pi_{k^*} \cdot J^T \Sigma_{k^*}^{-1} J \in \mathbb{R}^{6 \times 6}$$

**Gradient 벡터** (포인트별 누적):

$$b \mathrel{+}= \pi_{k^*} \cdot J^T \Sigma_{k^*}^{-1} r \in \mathbb{R}^{6 \times 1}$$

이는 GTSAM의 `HessianFactor` 인터페이스 $(H, b)$에 직접 전달된다. Cholesky 분해 기반 $\sqrt{\pi_k} \cdot L^{-T} r$ 방식이 아닌 직접 quadratic form을 사용하는데, 이는 GTSAM이 Hessian-form을 기대하기 때문이다.

**병렬 reduction**: `scan_matching_reduce_omp()` 또는 `scan_matching_reduce_tbb()`로 포인트별 $(H, b, \text{cost})$를 병렬 합산.

**코드 위치**: `include/gmm/impl/integrated_mixture_light_ndt_factor_impl.hpp:274-365`

---

## 6. Phase 3.5–3.6: 벤치마크 통합 + 버그 수정

### 6.1 Lazy Init 패턴

**문제**: `insert()` → `finalize()` → EM이 **모든 프레임에서** 실행되어, GMM이 불필요한 factor(LightNDT 등)에서도 ~35분 소요.

**원인**: `IncrementalVoxelMap::insert()`는 매 포인트 삽입 시 `finalize()`를 호출하는 설계. `GaussianVoxel`은 $O(N)$이라 무해하지만, `GMMVoxel`의 EM은 $O(K \times N \times D^2 \times \text{iters})$로 복셀당 수~수십 ms.

**해결**: 3단계 지연 실행 패턴:

1. `GMMVoxelMapCPU::insert()`: 포인트 누적(add) + LRU만 수행, **finalize 생략**. `needs_finalize_ = true`.
2. `GMMVoxelMapCPU::knn_search()`: `needs_finalize_`이면 `finalize_all()` 자동 호출 (lazy trigger).
3. `GMMVoxelMapCPU::finalize_all()`: 모든 dirty 복셀에 대해 **OpenMP 병렬** EM 실행.

**`ensure_gmm_voxelmaps()` (main.cpp)**: MixtureLightNDT factor 선택 시에만 GMM 복셀맵을 생성하는 lazy init 메서드. `insert()` 직후 **`finalize_all()`을 명시적으로 호출**한다.

```cpp
// src/main.cpp — ensure_gmm_voxelmaps()
for (size_t j = 0; j < frames.size(); j++) {
  auto gvm = std::make_shared<GMMVoxelMapCPU>(voxel_resolution);
  gvm->insert(*frames[j]);
  gvm->finalize_all();  // ← 명시적 finalize (Phase 3.6 fix)
  gmm_voxelmaps.push_back(gvm);
}
```

**`finalize_all()` 명시적 호출이 필요한 이유**: Factor의 `update_correspondences()`가 `knn_search()`를 **우회**하고 `lookup_voxel().components()`에 직접 접근하므로, `knn_search()`에 의한 lazy finalize trigger가 발동하지 않는다.

### 6.2 w-component Leakage Fix

**근본 원인**: `GMMComponent::mean`의 $w=0$ vs 변환된 포인트 $T \cdot p_i$의 $w=1$ 불일치.

| 연산 | w 값 | 문제 |
|------|------|------|
| `diff = pt - mean` | $1 - 0 = 1$ | 거리 계산에 상수 +1 편향 |
| `residual = mean - transed` | $0 - 1 = -1$ | 잔차에 상수 -1 편향 |
| `inv_cov(3,3)` | ≈ 20,000~35,000 | $\text{cov}(3,3) = 0$ → 고유값 분해 시 폭발 |

**결과**: 포인트당 $\sim 26{,}850$의 허위 비용 → 총 비용 $\sim 3.7 \times 10^9$ → LM 옵티마이저가 하강 방향을 찾지 못함 → Mean R 4.577°.

**LightNDT에서 문제 없는 이유**: `GaussianVoxel::mean`의 $w=1$이므로 `diff.w = 1-1 = 0` 자연스럽게 해소.

**수정 (2곳)**:

```cpp
// update_correspondences(): diff(3) = 0.0 추가
diff(3) = 0.0;  // w-component 차이 제거

// evaluate(): residual(3) = 0.0 추가
residual(3) = 0.0;  // w-component 차이 제거
```

**수정 후**: Mean R **4.577° → 0.474°** (9.7배 개선).

---

## 7. 데이터 흐름 종합

```
포인트 클라우드
     │
     ▼
┌─────────────────────────────────┐
│  GMMVoxelMapCPU::insert()       │
│  ┌────────────────────────┐     │
│  │ 복셀 좌표 계산          │     │
│  │ fast_floor(pt * inv_ls) │     │
│  └────────────┬───────────┘     │
│               ▼                  │
│  ┌────────────────────────┐     │
│  │ GMMVoxel::add()        │     │
│  │ Algorithm R Reservoir  │     │
│  │ pt.w=0, P(교체)=C/N   │     │
│  └────────────────────────┘     │
│  needs_finalize_ = true          │
└─────────────────────────────────┘
     │
     ▼ (ensure_gmm_voxelmaps 또는 knn_search)
┌─────────────────────────────────┐
│  finalize_all() [OpenMP]         │
│  ┌────────────────────────┐     │
│  │ GMMVoxel::finalize()   │     │
│  │ ┌──────────────────┐   │     │
│  │ │ fit_gmm()         │   │     │
│  │ │ EM: E-step/M-step │   │     │
│  │ │ → K components    │   │     │
│  │ └──────────────────┘   │     │
│  └────────────────────────┘     │
└─────────────────────────────────┘
     │
     ▼ (Factor 생성)
┌─────────────────────────────────┐
│  IntegratedMixtureLightNDTFactor │
│  ┌────────────────────────┐     │
│  │ inv_cov_cache 구축      │     │
│  │ [v][k] = Σ_k^{-1}     │     │
│  └────────────────────────┘     │
└─────────────────────────────────┘
     │
     ▼ (GTSAM linearize())
┌─────────────────────────────────┐
│  update_correspondences()        │
│  소스 포인트별:                    │
│    q = delta * p_i                │
│    이웃 복셀의 모든 컴포넌트 순회    │
│    k* = argmin_k Mahalanobis     │
│    diff(3) = 0                    │
└─────────────────────────────────┘
     │
     ▼
┌─────────────────────────────────┐
│  evaluate() [OMP parallel]       │
│  포인트별:                         │
│    r = μ_{k*} - T·p_i            │
│    r(3) = 0                       │
│    cost += π_{k*} · r^T Σ^{-1} r │
│    H += π_{k*} · J^T Σ^{-1} J   │
│    b += π_{k*} · J^T Σ^{-1} r   │
└─────────────────────────────────┘
     │
     ▼
  GTSAM LM Optimizer
  (H·Δξ = -b → 포즈 갱신)
```

---

## 8. 수식 – 코드 매핑 테이블

| # | 수식 | 코드 위치 | 함수/변수 |
|---|------|-----------|-----------|
| 1 | $P(\text{교체}) = C/N$ | `gmm_voxelmap_cpu.cpp:46-48` | `GMMVoxel::add()` — `dist(0, N-1)`, `j < capacity` |
| 2 | $\mathcal{L} = \sum \log \sum \pi_k \mathcal{N}$ | `mixture_em_backend.cpp:180-188` | `arma::gmm_full::learn()` |
| 3 | $\pi_k = N_k / N$ (M-step) | `mixture_em_backend.cpp:102-103` | `model.hefts(k)` |
| 4 | $\Sigma_k \mathrel{+}= \varepsilon \cdot I_3$ | `mixture_em_backend.cpp:100` | `extract_result()` — 사후 정칙화 |
| 5 | Pruning: $\pi_k < \tau$ | `mixture_em_backend.cpp:120-123` | `remove_if` + erase |
| 6 | $\Sigma \pi_k = 1$ 재정규화 | `mixture_em_backend.cpp:137-145` | `c.weight /= total_weight` |
| 7 | $\lambda_i' = \max(\lambda_i, \varepsilon \lambda_{\max})$ | `integrated_ndt_factor.hpp` | `compute_ndt_inverse_covariance()` |
| 8 | $k^* = \arg\min_k d^T \Sigma_k^{-1} d$ | `*_impl.hpp:221-233` | `update_correspondences()` — min Mahalanobis |
| 9 | $d(3) = 0$ (w-fix) | `*_impl.hpp:222` | `diff(3) = 0.0` |
| 10 | $r = \mu_{k^*} - T \cdot p_i$ | `*_impl.hpp:311` | `residual = mean_B - transed_mean_A` |
| 11 | $r(3) = 0$ (w-fix) | `*_impl.hpp:312` | `residual(3) = 0.0` |
| 12 | $C_i = \pi_{k^*} r^T \Sigma^{-1} r$ | `*_impl.hpp:316` | `cost = pi_k * (...)` |
| 13 | $J_t = [-[Tp]_\times \; I]$ | `*_impl.hpp:331-333` | `J_target` — SO3::Hat |
| 14 | $J_s = [R[p]_\times \; -R]$ | `*_impl.hpp:338-340` | `J_source` |
| 15 | $H \mathrel{+}= \pi_k J^T \Sigma^{-1} J$ | `*_impl.hpp:348-350` | `H_target/source += J_weighted * J` |
| 16 | $b \mathrel{+}= \pi_k J^T \Sigma^{-1} r$ | `*_impl.hpp:353-354` | `b_target/source += J_weighted * residual` |
| 17 | Warm-start: $\Sigma' = \Sigma - \varepsilon I$ | `mixture_em_backend.cpp:236-238` | 이중 정칙화 방지 |

---

## 9. 설계 결정 근거 (Design Rationale)

### 9.1 Winner-take-all vs Soft Assignment

| 방식 | 비용 | 장점 | 단점 |
|------|------|------|------|
| **Winner-take-all** (채택) | $O(K)$ per point | Jacobian 단순, 수치 안정적 | 컴포넌트 경계에서 불연속 |
| Soft (log-sum-exp) | $O(K)$ per point | 매끄러운 그래디언트 | log-sum-exp 오버플로 위험, 복잡한 Jacobian |

Winner-take-all을 선택한 이유:
- GTSAM LM 옵티마이저는 매 iteration마다 `update_correspondences()`를 재호출하므로, 경계 불연속이 실질적으로 평활화됨
- Jacobian이 단일 컴포넌트에 대해서만 계산되므로 코드 단순성 유지
- 벤치마크에서 이미 최고 rotation 정확도 달성 → soft assignment의 추가 이득 불확실

### 9.2 π_k 가중 비용 vs 동일 가중

비용에 $\pi_k$를 곱하는 이유:
- **소수 포인트로 구성된 약한 컴포넌트**의 영향을 자연스럽게 억제
- 가중치가 작은 컴포넌트에 매칭된 포인트는 비용 기여가 낮아, 노이즈에 대한 **로버스트성** 향상
- Hessian에서도 $\pi_k$ 가중이 유지되어 **PSD(양의 반정부호)** 보장

### 9.3 Dual Inheritance 구조

`GMMVoxelMapCPU`가 `GaussianVoxelMapCPU`의 서브클래스가 **아닌** 이유:
- `GaussianVoxelMapCPU`는 내부적으로 `IncrementalVoxelMap<GaussianVoxel>`을 사용
- `GMMVoxelMapCPU`는 `IncrementalVoxelMap<GMMVoxel>`을 사용 → 템플릿 인자가 다름
- 따라서 **형제(sibling)** 관계로 공통 인터페이스 `GaussianVoxelMap`을 상속

### 9.4 GMMComponent::mean w=0 vs w=1

기존 `GaussianVoxel::mean`은 $w=1$ (동차 좌표)이지만, `GMMComponent::mean`은 $w=0$을 사용:
- Armadillo EM은 3D 데이터를 처리하므로, 4D↔3D 변환 시 $w$를 일관되게 0으로 유지하는 것이 자연스러움
- `add()` 시점에 `pt(3) = 0`으로 강제하여 저수지 전체가 $w=0$
- **대가**: Factor에서 `diff(3) = 0`, `residual(3) = 0`을 명시적으로 설정해야 함

---

## 10. 하이퍼파라미터 튜닝 가이드

| 파라미터 | 현재값 | 영향 | 튜닝 방향 |
|----------|--------|------|-----------|
| `max_components` | 3 | K 증가 → 표현력↑, EM 비용↑ | 2~4 범위 실험. 단순 환경은 2로 충분 |
| `reservoir_capacity` | 256 | C 증가 → EM 입력 증가, 정확도↑, 속도↓ | 64~256. 속도 중시 시 64 |
| `max_em_iterations` | 20 | 반복↑ → 수렴 정확도↑, 속도↓ | warm-start 시 10으로 축소 가능 |
| `convergence_tol` | 1e-4 | 작을수록 정밀, 반복 수↑ | 1e-3~1e-5 |
| `covariance_regularization` | 1e-3 | 클수록 안정적, 작을수록 정밀 | 1e-4~1e-2 |
| `min_weight_threshold` | 0.01 | 클수록 공격적 pruning | 0.01~0.05 |
| `regularization_epsilon` (Factor) | 1e-3 | 역공분산 정칙화. 클수록 등방적 | 1e-4~1e-2 |
| `search_mode` | DIRECT7 | 이웃 많을수록 정확↑, 느림↑ | DIRECT7이 속도/정확도 균형 |

**속도 최적화 우선 시**: `reservoir_capacity=64`, `max_components=2`, `max_em_iterations=10`
**정확도 최적화 우선 시**: `reservoir_capacity=256`, `max_components=4`, `search_mode=DIRECT27`

---

## 11. LightNDT vs MixtureLightNDT 구조 비교

| 항목 | LightNDT | MixtureLightNDT |
|------|----------|-----------------|
| 복셀맵 | `GaussianVoxelMapCPU` | `GMMVoxelMapCPU` |
| 복셀 타입 | `GaussianVoxel` (단일 가우시안) | `GMMVoxel` (K개 GMM) |
| mean w 값 | w=1 | w=0 |
| 대응 구조체 | `NdtCorrespondence` | `MixtureNdtCorrespondence` |
| 정규화 상수 | `one_over_z` ($1/Z$) | **π_k** (혼합 가중치) |
| 대응 전략 | 최근접 복셀의 단일 분포 | 이웃 복셀의 **모든 컴포넌트** 중 argmin Mahalanobis |
| inv_cov_cache | 1D: `cache[voxel_id]` | 2D: `cache[voxel_id][k]` |
| Hessian 가중 | 없음 | $\pi_k \cdot J^T \Sigma^{-1} J$ |
| finalize 비용 | $O(N)$ (Welford 업데이트) | $O(K \times N \times D^2 \times \text{iters})$ (EM) |
| Mean R (벤치마크) | 0.591° | **0.474°** ★ |
| 속도 (벤치마크) | 479 ms | 1,476 ms |

---

*최종 수정: 2026-03-24*
