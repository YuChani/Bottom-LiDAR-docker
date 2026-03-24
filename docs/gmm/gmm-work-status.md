# GMM 구현 작업 현황

## 📋 프로젝트 개요

LiDAR 포인트클라우드 정합을 위한 Gaussian Mixture Model(GMM) 기반 복셀맵 및 NDT Factor 구현. 기존 `GaussianVoxelMapCPU`의 단봉(single Gaussian) 한계를 넘어, 복셀당 다봉(mixture) 분포를 표현하고 이를 GTSAM factor graph에 통합하는 것이 목표다.

아키텍처: `GMMVoxelMapCPU` (복셀맵) → `MixtureEMBackend` (Armadillo EM) → `IntegratedMixtureLightNDTFactor` (GTSAM factor)

## ✅ 완료된 작업

### Phase 1: GMMVoxel 구조체 + Reservoir Sampling

- **커밋**: `687bc18` (2026-03-24)
- **구현 내용**:
  - `GMMVoxel` 구조체 정의 (Setting, add, finalize, components, reservoir 등)
  - Algorithm R Reservoir Sampling (capacity=256): capacity 미만이면 push_back, 이상이면 `uniform(0, total-1) < capacity`일 때 교체
  - `GMMVoxelMapCPU` 클래스: `GaussianVoxelMap` + `IncrementalVoxelMap<GMMVoxel>` 이중 상속
  - `frame::traits` 특수화 (`GMMVoxel`, `IncrementalVoxelMap<GMMVoxel>`, `GMMVoxelMapCPU`)
- **생성/수정 파일**:
  - `include/gmm/gmm_voxelmap_cpu.hpp`
  - `src/gmm/gmm_voxelmap_cpu.cpp`
  - `src/gmm/CMakeLists.txt`
- **테스트 결과**: GMMVoxel Suite 기본 4개 + GMMVoxelMapCPU Suite 기본 2개 PASS

### Phase 2: EM Backend

- **커밋**: `687bc18` (2026-03-24, Phase 1과 동일 커밋)
- **구현 내용**:
  - `fit_gmm()` cold-start: Armadillo `gmm_full::learn(eucl_dist, random_spread)`
  - `fit_gmm()` warm-start: `keep_existing` 모드로 기존 컴포넌트 초기값 활용
  - `GMMFitResult` (components, converged, iterations_run) 반환
  - 공분산 정칙화(regularization), 최소 weight 임계값 이하 컴포넌트 pruning
  - All-pruned fallback: weighted mean으로 단일 컴포넌트 생성
- **생성/수정 파일**:
  - `include/gmm/mixture_em_backend.hpp`
  - `src/gmm/mixture_em_backend.cpp`
  - `src/gmm/test/test_gmm_voxelmap.cpp`
- **테스트 결과**: GMMFit Suite 7개 전체 PASS
- **총 신규 코드**: 6개 파일, 877줄

### Phase 2.5: finalize() EM 연결 + Oracle 리뷰 버그 수정

- **커밋**: `5a817eb` (2026-03-24)
- **구현 내용**:
  - `GMMVoxel::finalize()`에서 EM backend 호출 연결: `components_.empty()` → cold-start, 아니면 warm-start
  - `dirty_=false` 설정으로 중복 finalize 방지
  - Oracle 리뷰 기반 5건 버그 수정 (아래 리뷰 결과 참조)
  - `knn_search`에서 `diff(3)=0.0` 설정으로 w 차원 bias 제거
  - `traits<IncrementalVoxelMap<GMMVoxel>>` by-value 반환으로 dangling ref 방지
  - warm-start 이중 regularization 수정 (set_params 전 reg 차감, 1e-6 clamp)
  - all-pruned fallback에서 weighted mean 계산 적용
- **수정 파일**:
  - `include/gmm/gmm_voxelmap_cpu.hpp` (+26줄)
  - `src/gmm/gmm_voxelmap_cpu.cpp` (+50/-7줄)
  - `src/gmm/mixture_em_backend.cpp` (+30/-2줄)
  - `src/gmm/test/test_gmm_voxelmap.cpp` (+161줄)
- **테스트 결과**: 18개 전체 PASS

### Phase 3: IntegratedMixtureLightNDTFactor

- **커밋**: `69b6163` (2026-03-24)
- **구현 내용**:
  - `IntegratedMixtureLightNDTFactor_` 템플릿 클래스: `IntegratedMatchingCostFactor` 상속
  - `MixtureNdtCorrespondence` 구조체: mean, inv_cov, weight(π_k), valid
  - `update_correspondences()`: Winner-take-all argmin Mahalanobis 방식으로 모든 이웃 복셀의 모든 GMM 컴포넌트 탐색
  - `evaluate()`: π_k 가중 Mahalanobis 비용 + SE(3) left-perturbation Jacobian
  - `inv_cov_cache`: 팩터 생성 시 전체 복셀의 전체 컴포넌트 역공분산 사전 계산
  - `dynamic_pointer_cast<GMMVoxelMapCPU>` 검증 (실패 시 abort)
  - OMP 병렬 처리 (correspondence 업데이트 + 잔차 평가)
- **생성 파일**:
  - `include/gmm/integrated_mixture_light_ndt_factor.hpp`
  - `include/gmm/impl/integrated_mixture_light_ndt_factor_impl.hpp`
  - `src/gmm/integrated_mixture_light_ndt_factor.cpp`
  - `src/gmm/CMakeLists.txt` 수정
- **테스트 결과**: 기존 18개 + 신규 6개 = 24개 전체 PASS

### Phase 3.5: 벤치마크 통합 + convergence_tol 버그 수정

- **커밋**: 미커밋 (2026-03-24)
- **구현 내용**:
  - `src/main.cpp`에 GMM include 추가 (`gmm_voxelmap_cpu.hpp`, `integrated_mixture_light_ndt_factor.hpp`, impl)
  - `gmm_voxelmaps` 멤버 변수 추가 (`vector<GaussianVoxelMap::Ptr>`)
  - 프레임 로딩 루프에 `GMMVoxelMapCPU` 생성 + `insert()` 추가
  - `factor_types`에 `"MixtureLightNDT"` 등록
  - `create_factor()`에 MixtureLightNDT 분기 추가 (DIRECT7, ε=1e-3)
  - `run_optimization()`에서 factor type에 따라 `gmm_voxelmaps` vs `voxelmaps` 선택
  - `src/gmm/mixture_em_backend.cpp` convergence_tol dead code 수정 (lines 187, 266): 하드코딩 `1e-6` → `params.convergence_tol`
- **수정 파일**:
  - `src/main.cpp` (6곳 수정)
  - `src/gmm/mixture_em_backend.cpp` (2줄 수정)
- **빌드 결과**: `lidar_registration_benchmark` 타겟 빌드 성공
- **벤치마크 결과**: 아래 Phase 3.5 벤치마크 섹션 참조

### Phase 3.5 성능 수정: Lazy GMM Voxelmap Init

- **문제**: 로딩 루프에서 모든 프레임에 대해 `GMMVoxelMapCPU::insert()` → EM fitting을 무조건 실행. LightNDT 등 GMM 불필요한 factor에서도 ~35분 소요.
- **원인**: `insert()` → `finalize()` → `arma::gmm_full::learn()` (K=4 EM)이 프레임당 수백 복셀에서 실행
- **수정**: `ensure_gmm_voxelmaps()` lazy init 도입. `run_optimization()`에서 MixtureLightNDT 선택 시에만 GMM 복셀맵 생성.
- **결과**: LightNDT 실행 시간 ~35분 → **1.2초** (2,900배 개선)

### Phase 3.5 현재 상태: GMM 임시 비활성화

- **날짜**: 2026-03-24
- **상태**: ~~`src/main.cpp`에서 GMM 관련 코드 6곳 전체 주석 처리~~ → **Phase 3.6에서 복원 완료**
- **사유**: GMM 로딩 병목(~35분) 진단을 위해 기존 factor만으로 비교 실험 수행
- **복원 방법**: ~~`src/main.cpp`에서 `임시 주석 처리`를 검색하여 6곳 주석 해제~~ → 완료

| # | 위치 (main.cpp) | 내용 |
|---|----------------|------|
| 1 | L54-57 | `#include "gmm/..."` (3개 include) |
| 2 | L382 | `factor_types.push_back("MixtureLightNDT")` |
| 3 | L670-679 | `create_factor()` MixtureLightNDT 분기 |
| 4 | L883-893 | `ensure_gmm_voxelmaps()` 메서드 |
| 5 | L895-899 | `run_optimization()` — `voxelmaps`만 사용하도록 단순화 |
| 6 | L928 | `gmm_voxelmaps` 멤버 변수 |

### Phase 3.6: 버그 수정 + GMM 복원

- **날짜**: 2026-03-24 (미커밋)
- **구현 내용**:
  1. **Lazy Finalize Safety Fix** (`src/main.cpp`):
     - `ensure_gmm_voxelmaps()`에서 `insert()` 직후 `gvm->finalize_all()` 호출 추가
     - 원인: Factor의 `update_correspondences()`가 `knn_search()`를 우회하고 직접 `lookup_voxel().components()`에 접근하므로, `knn_search()`에 의한 lazy finalize 트리거가 발동되지 않음
     - 효과: GMM 컴포넌트가 factor 생성 전에 항상 채워져 있음을 보장
  2. **w-component Leakage Fix** (`include/gmm/impl/integrated_mixture_light_ndt_factor_impl.hpp`):
     - `update_correspondences()`의 `diff(3) = 0.0` 추가 (~line 240)
     - `evaluate()`의 `residual(3) = 0.0` 추가 (~line 345)
     - **근본 원인**: `GMMComponent::mean`의 w=0 (3D 포인트로 저장) vs `delta * source_point`의 w=1 (동차 변환 결과) 불일치
       - correspondence 검색 시: `diff.w = pt(w=1) - mean(w=0) = 1` → 거리 계산 왜곡
       - 비용 평가 시: `residual.w = mean(w=0) - transed(w=1) = -1` → 잔차에 상수 편향
       - `inv_cov(3,3)` ≈ 20,000~35,000 (cov(3,3)=0 → 고유값 분해 시 폭발)
       - 결과: 포인트당 ~26,850 허위 비용 → 총 비용 ~37억 → LM이 하강 방향 찾기 불가
     - **LightNDT에서 문제 없는 이유**: `GaussianVoxel::mean`의 w=1이므로 `diff.w = 1-1 = 0` 자연스럽게 해소
  3. **GMM 코드 복원**: Phase 3.5에서 주석 처리된 6곳 전체 해제 + lazy init 패턴 활성화
- **수정 파일**:
  - `src/main.cpp` (`ensure_gmm_voxelmaps()`에 `finalize_all()` 추가)
  - `include/gmm/impl/integrated_mixture_light_ndt_factor_impl.hpp` (`diff(3)=0`, `residual(3)=0` 추가)
- **테스트 결과**: 24/24 PASS
- **벤치마크 결과**: Mean R **4.577° → 0.474°** (9.7배 개선, 전체 8개 factor 중 최고 rotation 정확도)

### 벤치마크 결과

#### Phase 3.5: 전체 Factor 비교 (GMM 비활성화 상태, 버그 수정 전)

데이터: 7프레임, noise_scale=0.1, sparse_connection_window=4, LM optimizer

| Factor | Mean T (m) | Mean R (deg) | Max T (m) | Max R (deg) | 최적화 (ms) | Iters |
|--------|-----------|-------------|----------|------------|------------|-------|
| Point-to-Point | 0.093 | 0.425 | 0.157 | 0.736 | 1601 | 26 |
| Point-to-Plane | **0.065** | 0.456 | **0.128** | 0.892 | 872 | 14 |
| GICP | 0.079 | 0.484 | 0.159 | 1.010 | 784 | 8 |
| VGICP | 0.116 | 0.623 | 0.203 | 1.182 | 1107 | 12 |
| NDT | 0.077 | 0.535 | 0.142 | 0.830 | 4481 | 119 |
| LightNDT | 0.114 | 0.591 | 0.247 | 0.943 | 567 | 7 |
| LOAM_LIOSAM | 0.131 | 0.528 | 0.267 | 0.875 | **48** | 9 |

#### Phase 3.5: LightNDT vs MixtureLightNDT (버그 수정 전, w-leakage 존재)

| Factor | Mean T (m) | Mean R (deg) | Max T (m) | Max R (deg) | 최적화 (ms) | Iters |
|--------|-----------|-------------|----------|------------|------------|-------|
| LightNDT | 0.114 | **0.591** | 0.247 | **0.943** | 550 | 7 |
| MixtureLightNDT | **0.077** | 4.577 | **0.139** | 7.007 | 285 | 11 |

**분석 (수정 전)**: Translation은 우수했으나 Rotation에서 7.7배 열화. w-component leakage가 원인.

#### Phase 3.6: 전체 8-Factor 최종 비교 (버그 수정 후) ★

데이터: 7프레임, noise_scale=0.1, sparse_connection_window=4, LM optimizer

| Factor | Mean T (m) | Mean R (deg) | Max T (m) | Max R (deg) | 최적화 (ms) | Iters |
|--------|-----------|-------------|----------|------------|------------|-------|
| Point-to-Point | 0.093 | 0.425 | 0.157 | 0.736 | 1440 | 26 |
| Point-to-Plane | **0.065** | 0.456 | **0.128** | 0.892 | 752 | 14 |
| GICP | 0.079 | 0.484 | 0.159 | 1.010 | 655 | 8 |
| VGICP | 0.116 | 0.623 | 0.203 | 1.182 | 972 | 12 |
| NDT | 0.077 | 0.535 | 0.142 | 0.830 | 4576 | 119 |
| LightNDT | 0.114 | 0.591 | 0.247 | 0.943 | 479 | 7 |
| **MixtureLightNDT** | 0.116 | **0.474** ★ | 0.209 | **0.804** ★ | 1476 | 21 |
| LOAM_LIOSAM | 0.131 | 0.528 | 0.267 | 0.875 | **47** | 9 |

★ = 전체 factor 중 최고 Rotation 정확도

**분석 (수정 후)**:
- MixtureLightNDT가 Mean R = 0.474°로 **전체 8개 factor 중 최고 rotation 정확도** 달성
- Max R = 0.804°도 전체 factor 중 최고 (NDT의 0.830° 대비 개선)
- Translation은 VGICP와 유사 (0.116m), Point-to-Plane(0.065m) 대비 열위
- 속도는 NDT(4576ms)보다 빠르나, LightNDT(479ms)보다 3배 느림 → Phase 4 최적화 대상
- w-leakage 수정으로 Mean R **4.577° → 0.474°** (9.7배 개선)

### 문서

- **커밋**: `a98222f`
- GMM 설계 문서 및 UML 아티팩트 추가

## 🔧 구현된 파일 구조

```
include/gmm/
├── gmm_voxelmap_cpu.hpp                              # GMMVoxel 구조체 + GMMVoxelMapCPU 클래스 선언
├── mixture_em_backend.hpp                             # fit_gmm() 함수 선언 (cold-start / warm-start)
├── integrated_mixture_light_ndt_factor.hpp            # IntegratedMixtureLightNDTFactor_ 클래스 선언
└── impl/
    └── integrated_mixture_light_ndt_factor_impl.hpp   # update_correspondences() + evaluate() 템플릿 구현

src/gmm/
├── gmm_voxelmap_cpu.cpp                    # GMMVoxelMapCPU 구현 (insert, knn_search, traits 특수화)
├── mixture_em_backend.cpp                   # Armadillo 기반 EM 알고리즘 구현
├── integrated_mixture_light_ndt_factor.cpp  # 명시적 템플릿 인스턴스화
├── CMakeLists.txt                           # GMM 모듈 빌드 설정
└── test/
    └── test_gmm_voxelmap.cpp                # 24개 단위 테스트 (Phase 1~3)

docs/gmm/
├── gmm-design.md                  # GMM 설계 문서
├── gmm-work-status.md             # 본 문서 (작업 현황 추적)
└── (UML 아티팩트)                 # 클래스/시퀀스 다이어그램
```

## 🧪 테스트 현황

전체 24개 테스트, 모두 PASS.

### GMMVoxel Suite (7개)

| # | 테스트명 | 검증 내용 | 상태 |
|---|---------|----------|------|
| 1 | `DefaultConstructEmpty` | `size()==0`, components/reservoir 비어있음, `is_dirty()==false` | ✅ PASS |
| 2 | `ReservoirAccumulates` | cap=10일 때 5개 삽입 → size==5, 20개 삽입 → size==10 (상한 유지) | ✅ PASS |
| 3 | `FinalizeStubSingleComponent` | 50포인트 삽입 → finalize → size≥1, 주요 컴포넌트 mean≈(1,2,3) | ✅ PASS |
| 4 | `DirtyFlagBehavior` | add → dirty=true, finalize → dirty=false, 2회 finalize는 no-op | ✅ PASS |
| 5 | `FinalizeCallsEM` *(Phase 2.5)* | 2클러스터 200포인트 → finalize → size≥2, means≈(0,0,0)/(5,0,0) | ✅ PASS |
| 6 | `FinalizeWarmStart` *(Phase 2.5)* | 순차 삽입 + finalize 2회 → 공분산 양의 정부호(PD) 유지 | ✅ PASS |
| 7 | `FinalizeIdempotent` *(Phase 2.5)* | finalize 2회 호출 → 동일 components (1e-10 이내) | ✅ PASS |

### GMMVoxelMapCPU Suite (4개)

| # | 테스트명 | 검증 내용 | 상태 |
|---|---------|----------|------|
| 1 | `InstantiatesAndInserts` | insert 후 `num_voxels()>0`, `GaussianVoxelMap` dynamic_cast 성공 | ✅ PASS |
| 2 | `KnnSearchReturnsResults` | knn(k=1) found≥1 | ✅ PASS |
| 3 | `KnnSearchNoBias` *(Phase 2.5)* | (1,0,0) 삽입 후 query(0,0,0,1) → knn_dist≈1.0 (w 차원 bias 없음) | ✅ PASS |
| 4 | `EndToEndBimodal` *(Phase 2.5)* | voxel_size=20m 단일 복셀, 2클러스터 → 최소 1복셀 size≥2 | ✅ PASS |

### GMMFit Suite (7개)

| # | 테스트명 | 검증 내용 | 상태 |
|---|---------|----------|------|
| 1 | `SingleCluster` | 단봉 데이터 → converged=true, mean≈(1,2,3) | ✅ PASS |
| 2 | `BimodalSeparation` | 2클러스터 → components≥2, means≈(0,0,0)/(5,0,0) | ✅ PASS |
| 3 | `TrimodalSeparation` | 3클러스터 → components==3, weight≈1/3 | ✅ PASS |
| 4 | `TooFewPointsFallback` | 1포인트 → components==1 (fallback 동작) | ✅ PASS |
| 5 | `CovariancePD` | 모든 컴포넌트의 최소 고유값 > 0 (양의 정부호) | ✅ PASS |
| 6 | `WeightsSumToOne` | 전체 weight 합 ≈ 1.0 (허용오차 1e-6) | ✅ PASS |
| 7 | `WarmStartDoesNotDiverge` | warm-start → converged=true, components≥1 | ✅ PASS |

### MixtureLightNDT Factor Suite (6개)

| # | 테스트명 | 검증 내용 | 상태 |
|---|---------|----------|------|
| 1 | `ConstructsFromGMMVoxelMap` | GMMVoxelMapCPU로부터 factor 생성 성공 | ✅ PASS |
| 2 | `RejectsNonGMMVoxelMap` | 비-GMM 복셀맵 전달 시 abort (SIGABRT 확인) | ✅ PASS |
| 3 | `ErrorDecreasesWithOptimization` | 반복 최적화 시 오차 단조 감소 | ✅ PASS |
| 4 | `NumericalGradientCheck` | 해석적 Jacobian과 수치 그래디언트 차이 < 1e-4 | ✅ PASS |
| 5 | `IdentityPoseZeroResidual` | 동일 포인트클라우드, identity pose → 잔차 ≈ 0 | ✅ PASS |
| 6 | `InlierFractionReasonable` | inlier 비율이 합리적 범위 내 | ✅ PASS |

## 🔍 Oracle 리뷰 결과

### 1차 리뷰 (2026-03-24, Phase 1~2.5)

총 6건 발견, 5건 수정 완료, 1건 기각.

| # | 이슈 | Severity | 수정 여부 | 비고 |
|---|------|----------|-----------|------|
| 1 | `knn_search` w=1 bias: `squaredNorm`에 w 차원 포함되어 거리 ranking 왜곡 | CRITICAL | ✅ 수정 | `diff(3)=0` 강제 |
| 4 | `add()`에서 reservoir 저장 전 w≠0 포인트 미처리 | MODERATE | ✅ 수정 | `pt(3)=0` 강제 |
| 5 | `traits<IncrementalVoxelMap<GMMVoxel>>` generic이 `const&` 반환 → dangling ref | CRITICAL | ✅ 수정 | explicit specialization, by-value 반환 |
| 6c | warm-start 이중 regularization: EM 전 이미 reg 포함된 값을 `set_params` 후 `extract_result`에서 재적용 | MODERATE | ✅ 수정 | set_params 전 reg 차감, 1e-6 clamp |
| 7 | all-pruned fallback이 zero-mean 사용 | MINOR | ✅ 수정 | weighted mean 계산 후 fallback |
| 9 | `neighbor_offsets(1)` 범위가 너무 좁다는 제안 | DISMISSED | ❌ 무시 | `GaussianVoxelMapCPU`와 동일 패턴 사용 중 |

### 2차 리뷰 (2026-03-24, Phase 1~3 전체)

총 13건 검증, 12건 PASS, 1건 CONCERN 수정 완료.

| # | 항목 | 결과 | 비고 |
|---|------|------|------|
| 1 | Algorithm R reservoir sampling (P=C/N) | **PASS** | |
| 2 | EM / Armadillo `learn()` | **CONCERN→수정완료** | `convergence_tol` dead code: 하드코딩 1e-6 → `params.convergence_tol` |
| 3 | 공분산 정칙화 (3×3 block에 ε·I) | **PASS** | |
| 4 | 가중치 정규화 / pruning (Σπ_k=1) | **PASS** | |
| 5 | warm-start fallback (K > max) | **PASS** | |
| 6 | finalize() empty guard | **PASS** | |
| 7 | w-component forcing in KNN | **PASS** | |
| 8 | winner-take-all correspondence | **PASS** | |
| 9 | 비용 함수 잔차 컨벤션 | **PASS** | |
| 10 | Jacobian 정확성 (SE(3) left-perturbation) | **PASS** | |
| 11 | π_k 가중 Hessian (H = Σ π_k J^T Σ^{-1} J) | **PASS** | |
| 12 | inv_cov_cache 구조 | **PASS** | |
| 13 | compute_ndt_inverse_covariance | **PASS** | |

### 3차 리뷰 (2026-03-24, Phase 1~3.5 종합)

총 19건 검증, **19건 전체 PASS**.

| # | 항목 | 결과 |
|---|------|------|
| 1.1 | Algorithm R probability = C/N | **PASS** |
| 1.2 | Per-voxel deterministic RNG (seed 42) | **PASS** |
| 1.3 | w=0 enforcement (reservoir 저장 시) | **PASS** |
| 2.1 | Armadillo EM for K components | **PASS** |
| 2.2 | Covariance regularization ε·I | **PASS** |
| 2.3 | Weight normalization Σπ_k=1 | **PASS** |
| 2.4 | K-pruning (weight < threshold 제거) | **PASS** |
| 2.5 | convergence_tol fix 검증 | **PASS** |
| 3.1 | Reservoir → EM pipeline | **PASS** |
| 3.2 | Empty voxel guard (3중 보호) | **PASS** |
| 3.3 | Warm-start with double-reg prevention | **PASS** |
| 4.1 | Winner-take-all correspondence | **PASS** |
| 4.2 | Residual / cost formulation (직접 quadratic form) | **PASS** |
| 4.3 | Jacobian chain rule (SE(3) left-perturbation) | **PASS** |
| 4.4 | π_k weighting in Hessian | **PASS** |
| 4.5 | inv_cov_cache 정확성 | **PASS** |
| 4.6 | 4D/3D arithmetic safety (w=0 invariant) | **PASS** |
| 5.1 | Polymorphism GMMVoxelMapCPU→GaussianVoxelMap | **PASS** |
| 5.2 | run_optimization routing (use_gmm ternary) | **PASS** |

**비고**:
- 4.2: Cholesky `√(π_k)·L^{-T}·r` 방식이 아닌 직접 `π_k·r^T·Σ^{-1}·r` 방식 사용. GTSAM의 Hessian-form 인터페이스(H, b)에 맞으므로 수학적으로 동치.
- 4.6: `compute_ndt_inverse_covariance`가 4th row/col을 명시적으로 0으로 설정하지 않으나, `diff(3)=0`이 항상 보장되므로 실질적으로 무해. 향후 방어적 zero-out 추가 권장.

## 🚧 미완료 작업

### 즉시 진행 가능

1. **변경사항 git commit**: Phase 3.6 수정 (main.cpp, integrated_mixture_light_ndt_factor_impl.hpp, gmm-work-status.md 등)

### Phase 4: 하이퍼파라미터 튜닝 + 성능 최적화

향후 진행 예정.

- **Translation 정확도 개선**: MixtureLightNDT의 Mean T 0.116m → Point-to-Plane 수준(0.065m) 근접 목표
  - correspondence 업데이트 주기 / tolerance 조정
  - π_k 가중 제거 실험 (unweighted Mahalanobis만 사용)
  - voxel resolution 조정 실험
- **속도 최적화**: MixtureLightNDT 1476ms → LightNDT 수준(479ms) 근접 목표
  - EM 속도 최적화: OpenMP 병렬화, reservoir_capacity 축소(256→64), max_components 축소(4→2)
  - inv_cov_cache 계산 비용 프로파일링
- GMM 파라미터 최적화: `max_components`, `convergence_tol`, `reservoir_capacity` 등
- Multi-resolution GMM: coarse→fine 전략 적용 가능성 검토

> **참고**: Phase 3.6에서 w-component leakage 수정 후 Rotation 정확도는 Mean R = 0.474° (전체 factor 중 최고)로 이미 해결됨.

### Phase 5: GPU 가속 (선택)

GPU에서의 GMM 복셀맵 생성 및 factor 평가 가속화.

## 📐 핵심 수식 참조

### 비용 함수

$$C(T) = \sum_i \pi_{k^*(i)} \cdot d_{ik^*(i)}^T \Sigma_{k^*(i)}^{-1} d_{ik^*(i)}$$

여기서:

$$k^*(i) = \arg\min_k d_{ik}^T \Sigma_k^{-1} d_{ik}$$

$$d_{ik} = \mu_k - T \cdot p_i$$

각 소스 포인트 $p_i$에 대해, 마할라노비스 거리가 최소인 컴포넌트 $k^*$를 선택하고 해당 컴포넌트의 가중치 $\pi_k$를 곱한 마할라노비스 비용을 합산한다.

### Jacobian

$$\frac{\partial r_i}{\partial \xi} = \left[R \cdot [p_i]_\times \;\middle|\; -I_{3 \times 3}\right] \quad (3 \times 6)$$

### Gauss-Newton Hessian 근사

$$H \approx 2 \sum_i \pi_{k^*(i)} \cdot J_i^T \Sigma_{k^*(i)}^{-1} J_i$$

### 정합 오차 메트릭 (Phase 3.5 평가용)

회전 오차:

$$e_R = \arccos\left(\frac{\text{tr}(R_{est}^T R_{gt}) - 1}{2}\right)$$

병진 오차:

$$e_t = \|t_{est} - t_{gt}\|_2$$

## 🔨 빌드/테스트 실행 방법

```bash
# 빌드 (Docker 컨테이너 내부)
docker exec bottom-lidar bash -c "cd /root/workdir/build && cmake .. && make -j$(nproc)"

# GMM 테스트 실행 (24개)
docker exec bottom-lidar bash -c "cd /root/workdir/build && ./test_gmm_voxelmap"

# 벤치마크 실행 (MixtureLightNDT 단독)
docker exec bottom-lidar bash -c "cd /root/workdir && ./build/lidar_registration_benchmark --headless --factor MixtureLightNDT"

# 벤치마크 실행 (전체 factor 비교)
docker exec bottom-lidar bash -c "cd /root/workdir && ./build/lidar_registration_benchmark --headless"

# 컨테이너 ID: 5391fcf42791
```

## 📁 문서 현황

| 파일 | 내용 |
|------|------|
| `docs/gmm/gmm-design.md` | GMM 설계 문서 (아키텍처, 수학적 근거, 코드 동작, 수식-코드 매핑 17건, 설계 결정 근거, 튜닝 가이드) |
| `artifacts/uml/gmm_voxel_implementation.prisma` | GMM 구현 엔티티 다이어그램 (Phase 3.6 반영: w-fix, lazy finalize, needsFinalize) |
| `artifacts/uml/mixture_model_ndt_flow.prisma` | Single-Gaussian vs Mixture 경로 비교 다이어그램 |
| `docs/gmm/gmm-mathematical-foundations.md` | GMM 수학적 기반 문서 (EM 유도, 저수지 샘플링 증명, SE(3) Jacobian, Gauss-Newton Hessian, 수식-코드 매핑 22건, 참고 문헌 10건) |
| `docs/gmm/gmm-work-status.md` | 본 문서 (작업 현황 추적) |

## ⚠️ 알려진 제약사항 & 설계 결정

- `thirdparty/gtsam_points/`에 파일 추가 금지. 모든 GMM 코드는 프로젝트 자체 디렉토리(`include/gmm/`, `src/gmm/`)에 배치한다.
- include 규칙: quoted include `"gmm/..."` (프로젝트 소유), angle-bracket `<gtsam_points/...>` (thirdparty).
- `frame::traits<GMMVoxel>::point()`는 반드시 by-value 반환. reference 반환 시 dangling 위험.
- `finalize()` 시그니처는 `void finalize()` (인자 없음). `IncrementalVoxelMap`의 계약을 따른다.
- Reservoir는 `finalize()` 후에도 유지된다. warm-start 재진입 시 기존 데이터를 활용하기 위함.
- Correspondence 전략은 winner-take-all (argmin Mahalanobis). soft log-sum-exp 방식은 사용하지 않는다.
- 비용 함수에 π_k 가중치를 곱한다: $C_i = \pi_{k^*} \cdot r^T \Sigma_{k^*}^{-1} r$
- `neighbor_offsets(1)` 사용. Oracle이 범위 확대를 제안했으나 `GaussianVoxelMapCPU`와 동일 패턴이므로 기각.
