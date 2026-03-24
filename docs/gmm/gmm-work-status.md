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

### 문서

- **커밋**: `a98222f`
- GMM 설계 문서 및 UML 아티팩트 추가

## 🔧 구현된 파일 구조

```
include/gmm/
├── gmm_voxelmap_cpu.hpp          # GMMVoxel 구조체 + GMMVoxelMapCPU 클래스 선언
└── mixture_em_backend.hpp         # fit_gmm() 함수 선언 (cold-start / warm-start)

src/gmm/
├── gmm_voxelmap_cpu.cpp          # GMMVoxelMapCPU 구현 (insert, knn_search, traits 특수화)
├── mixture_em_backend.cpp         # Armadillo 기반 EM 알고리즘 구현
├── CMakeLists.txt                 # GMM 모듈 빌드 설정
└── test/
    └── test_gmm_voxelmap.cpp      # 18개 단위 테스트 (GMMVoxel + GMMVoxelMapCPU + GMMFit)

docs/
├── gmm-design.md                  # GMM 설계 문서
└── (UML 아티팩트)                 # 클래스/시퀀스 다이어그램
```

## 🧪 테스트 현황

전체 18개 테스트, 모두 PASS.

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

## 🔍 Oracle 리뷰 결과 (2026-03-24)

총 6건 발견, 5건 수정 완료, 1건 기각.

| # | 이슈 | Severity | 수정 여부 | 비고 |
|---|------|----------|-----------|------|
| 1 | `knn_search` w=1 bias: `squaredNorm`에 w 차원 포함되어 거리 ranking 왜곡 | CRITICAL | ✅ 수정 | `diff(3)=0` 강제 |
| 4 | `add()`에서 reservoir 저장 전 w≠0 포인트 미처리 | MODERATE | ✅ 수정 | `pt(3)=0` 강제 |
| 5 | `traits<IncrementalVoxelMap<GMMVoxel>>` generic이 `const&` 반환 → dangling ref | CRITICAL | ✅ 수정 | explicit specialization, by-value 반환 |
| 6c | warm-start 이중 regularization: EM 전 이미 reg 포함된 값을 `set_params` 후 `extract_result`에서 재적용 | MODERATE | ✅ 수정 | set_params 전 reg 차감, 1e-6 clamp |
| 7 | all-pruned fallback이 zero-mean 사용 | MINOR | ✅ 수정 | weighted mean 계산 후 fallback |
| 9 | `neighbor_offsets(1)` 범위가 너무 좁다는 제안 | DISMISSED | ❌ 무시 | `GaussianVoxelMapCPU`와 동일 패턴 사용 중 |

## 🚧 미완료 작업

### Phase 3: IntegratedMixtureLightNDTFactor

GTSAM factor graph에 GMM 기반 NDT factor를 통합하는 핵심 단계.

**생성할 파일**:
- `include/gmm/integrated_mixture_light_ndt_factor.hpp` : `MixtureNdtCorrespondence` 구조체 + `IntegratedMixtureLightNDTFactor_` 클래스 선언
- `include/gmm/impl/integrated_mixture_light_ndt_factor_impl.hpp` : `update_correspondences()` + `evaluate()` 템플릿 구현
- `src/gmm/integrated_mixture_light_ndt_factor.cpp` : 명시적 템플릿 인스턴스화
- `src/gmm/CMakeLists.txt` 수정 : 신규 .cpp 추가

**MixtureNdtCorrespondence 구조체**:
```cpp
struct MixtureNdtCorrespondence {
  Eigen::Vector4d mean = Eigen::Vector4d::Zero();
  Eigen::Matrix4d inv_cov = Eigen::Matrix4d::Zero();
  double weight = 0.0;   // pi_k
  bool valid = false;
};
```

**`update_correspondences()` 알고리즘**:
```
For each source point i:
  q_i = delta * source_point[i]
  coord = voxel_coord(q_i)
  best_corr = invalid, min_maha = INF

  for each offset in neighbor_offsets:
    voxel_id = lookup(coord + offset)
    if voxel_id < 0: continue
    voxel = lookup_voxel(voxel_id)

    for k = 0 .. voxel.size() - 1:     // 모든 컴포넌트 순회
      inv_cov = inv_cov_cache[voxel_id][k]
      diff = q_i - voxel.components()[k].mean
      m = diff^T * inv_cov * diff

      if m < min_maha:
        min_maha = m
        best_corr = {mean, inv_cov, weight=comp.weight, valid=true}

  correspondences[i] = best_corr
```

Winner-take-all 방식: 각 소스 포인트에 대해 모든 이웃 복셀의 모든 컴포넌트 중 마할라노비스 거리가 최소인 하나만 선택한다. soft log-sum-exp가 아닌 argmin 전략이다.

**핵심 구현 로직**:
- `evaluate()`에서 π_k 가중 마할라노비스 비용 계산
- OMP 병렬 처리로 포인트별 correspondence 업데이트 및 잔차 평가
- `inv_cov_cache` 사전 계산으로 반복 역행렬 연산 회피
- `dynamic_pointer_cast<GMMVoxelMapCPU>` 검증

**완료 기준**:
- 컴파일 및 링크 성공
- `dynamic_pointer_cast<GMMVoxelMapCPU>` 성공
- 수치 그래디언트 검사 1e-4 이내 통과
- 최적화 중 오차 감소 확인
- OMP 스레드 안전 병렬 평가
- 기존 18개 테스트 영향 없음

**신규 테스트 (6개)**:

| # | 테스트명 | 검증 내용 |
|---|---------|----------|
| 1 | `ConstructsFromGMMVoxelMap` | GMMVoxelMapCPU로부터 factor 생성 성공 |
| 2 | `RejectsNonGMMVoxelMap` | 비-GMM 복셀맵 전달 시 거부 |
| 3 | `ErrorDecreasesWithOptimization` | 반복 최적화 시 오차 단조 감소 |
| 4 | `NumericalGradientCheck` | 해석적 Jacobian과 수치 그래디언트 차이 < 1e-4 |
| 5 | `IdentityPoseZeroResidual` | 동일 포인트클라우드, identity pose → 잔차 ≈ 0 |
| 6 | `InlierFractionReasonable` | inlier 비율이 합리적 범위 내 |

### Phase 3.5: main.cpp 통합

실제 파이프라인에 MixtureLightNDT factor를 연결하는 마무리 단계.

**수정 파일**: `src/main.cpp` (4곳)

1. include 추가:
   ```cpp
   #include "gmm/gmm_voxelmap_cpu.hpp"
   #include "gmm/integrated_mixture_light_ndt_factor.hpp"
   ```
2. factor type 등록:
   ```cpp
   factor_types.push_back("MixtureLightNDT");
   ```
3. `gmm_voxelmaps` 저장소 생성 루프 추가
4. `create_factor()` 분기 추가:
   ```cpp
   else if (factor_types[factor_type] == "MixtureLightNDT") {
     auto factor = gtsam::make_shared<gtsam_points::IntegratedMixtureLightNDTFactor_<gtsam_points::PointCloud>>(
       target_key, source_key, target_voxelmap, source);
     factor->set_num_threads(num_threads);
     factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT7);
     factor->set_regularization_epsilon(1e-3);
     factor->set_correspondence_update_tolerance(...);
     return factor;
   }
   ```

**완료 기준**:
- `--headless` 실행 완료까지 크래시/NaN 없음
- MixtureLightNDT R/t 오차 ≤ LightNDT + 20% 이내
- 기존 factor type 동작에 영향 없음

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

# GMM 테스트 실행
docker exec bottom-lidar bash -c "cd /root/workdir/build && ./test_gmm_voxelmap"

# 컨테이너 ID: 5391fcf42791
```

## 📁 문서 현황

| 파일 | 내용 |
|------|------|
| `docs/gmm-design.md` | GMM 설계 문서 (아키텍처, 클래스 구조, EM 파라미터 등) |
| `docs/` 내 UML 아티팩트 | 클래스 다이어그램, 시퀀스 다이어그램 |
| `docs/gmm-work-status.md` | 본 문서 (작업 현황 추적) |

## ⚠️ 알려진 제약사항 & 설계 결정

- `thirdparty/gtsam_points/`에 파일 추가 금지. 모든 GMM 코드는 프로젝트 자체 디렉토리(`include/gmm/`, `src/gmm/`)에 배치한다.
- include 규칙: quoted include `"gmm/..."` (프로젝트 소유), angle-bracket `<gtsam_points/...>` (thirdparty).
- `frame::traits<GMMVoxel>::point()`는 반드시 by-value 반환. reference 반환 시 dangling 위험.
- `finalize()` 시그니처는 `void finalize()` (인자 없음). `IncrementalVoxelMap`의 계약을 따른다.
- Reservoir는 `finalize()` 후에도 유지된다. warm-start 재진입 시 기존 데이터를 활용하기 위함.
- Correspondence 전략은 winner-take-all (argmin Mahalanobis). soft log-sum-exp 방식은 사용하지 않는다.
- 비용 함수에 π_k 가중치를 곱한다: $C_i = \pi_{k^*} \cdot r^T \Sigma_{k^*}^{-1} r$
- `neighbor_offsets(1)` 사용. Oracle이 범위 확대를 제안했으나 `GaussianVoxelMapCPU`와 동일 패턴이므로 기각.
