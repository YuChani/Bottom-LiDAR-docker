# NDT (Normal Distributions Transform) Factor 구현 문서

**작성일**: 2026-02-19  
**상태**: gtsam_points 라이브러리에 통합 완료, 빌드 검증 완료

---

## 1. 개요

NDT(Normal Distributions Transform) Factor를 `gtsam_points` 라이브러리 내부에 구현하여,
기존 GICP/VGICP Factor와 동일한 패턴으로 사용할 수 있도록 통합하였다.

**핵심 설계 결정**: `GaussianVoxel`에는 `inv_cov` 필드가 없으므로,
`NdtCorrespondence` 구조체를 도입하여 voxel의 mean과 정규화된 역공분산을 캐싱한다.

---

## 2. 파일 위치 및 역할

### 2.1 소스 파일 (3개)

```
thirdparty/gtsam_points/
├── include/gtsam_points/factors/
│   ├── integrated_ndt_factor.hpp          # [1] 헤더 (클래스 선언 + 유틸리티)
│   └── impl/
│       └── integrated_ndt_factor_impl.hpp # [2] 템플릿 구현부
└── src/gtsam_points/factors/
    └── integrated_ndt_factor.cpp          # [3] 명시적 템플릿 인스턴스화
```

### 2.2 각 파일 상세 설명

#### [1] `integrated_ndt_factor.hpp` — 헤더 (207줄)

| 구성 요소 | 줄 범위 | 설명 |
|-----------|---------|------|
| `NdtCorrespondence` 구조체 | 29-37 | voxel mean, 역공분산, 유효성 플래그를 캐싱하는 구조체 |
| `compute_ndt_inverse_covariance()` | 44-54 | 4x4 공분산 행렬 -> 정규화된 역공분산 (고유값 분해 + 클램핑) |
| `compute_ndt_params()` | 67-73 | 해상도와 outlier 비율로부터 NDT 가우시안 파라미터 d1, d2 계산 (Magnusson 2009 Eq.6.9-6.10) |
| `NDTSearchMode` 열거형 | 76-80 | `DIRECT1` (1개), `DIRECT7` (7개), `DIRECT27` (27개) 이웃 복셀 검색 모드 |
| `IntegratedNDTFactor_<SourceFrame>` 클래스 | 90-204 | NDT 매칭 비용 팩터 본체. `IntegratedMatchingCostFactor` 상속 |

**주요 멤버 변수**:
- `target_voxels` — 타겟 가우시안 복셀맵 (`GaussianVoxelMapCPU`)
- `source` — 소스 포인트 클라우드
- `correspondences` — `NdtCorrespondence` 벡터 (mutable, 매 linearization마다 갱신)
- `resolution`, `outlier_ratio`, `regularization_epsilon` — NDT 하이퍼파라미터
- `search_mode` — 이웃 복셀 검색 모드
- `gauss_d1`, `gauss_d2` — NDT 가우시안 파라미터 (mutable, 캐싱)

#### [2] `integrated_ndt_factor_impl.hpp` — 템플릿 구현부 (271줄)

| 메서드 | 줄 범위 | 설명 |
|--------|---------|------|
| 생성자 (binary) | 20-43 | target_key, source_key, target_voxels, source 초기화 |
| 생성자 (unary) | 46-69 | fixed_target_pose, source_key, target_voxels, source 초기화 |
| `update_correspondences()` | 92-181 | 대응점 탐색 (아래 상세) |
| `evaluate()` | 184-268 | NDT 비용 함수 + Jacobian/Hessian 계산 (아래 상세) |

**`update_correspondences()` 핵심 로직**:
1. `compute_ndt_params()`로 d1, d2 계산
2. 검색 모드에 따라 이웃 오프셋 생성 (1/7/27개)
3. **역공분산 사전 캐싱**: 모든 복셀의 inv_cov를 한 번에 계산 → O(num_voxels) eigendecomp
4. 각 소스 포인트를 변환 → 복셀 좌표 계산 → 이웃 복셀 중 최소 마할라노비스 거리 복셀 선택
5. `NdtCorrespondence`에 mean, inv_cov, valid 저장

**`evaluate()` 핵심 로직**:
1. 잔차 = `mean_B - transed_mean_A` (타겟 - 변환된 소스, GTSAM 규약)
2. 마할라노비스 거리 계산: `r^T * inv_cov * r`
3. NDT 비용: `-d1 * (1 - exp(-d2/2 * mahal))` (비음수 재정의)
4. 야코비안: Lie algebra 기반 (`SO3::Hat`)
5. derivative_scale = `-d1 * d2 * e_term` (양수 → PSD 헤시안 보장)

#### [3] `integrated_ndt_factor.cpp` — 템플릿 인스턴스화 (12줄)

```cpp
template class gtsam_points::IntegratedNDTFactor_<gtsam_points::PointCloud>;
template class gtsam_points::IntegratedNDTFactor_<gtsam_points::DummyFrame>;
```

`PointCloud`와 `DummyFrame` 두 타입에 대해 명시적 인스턴스화하여 링크 타임 에러를 방지한다.

### 2.3 빌드 설정

**`thirdparty/gtsam_points/CMakeLists.txt` (191줄)**:
```cmake
src/gtsam_points/factors/integrated_ndt_factor.cpp
```
기존에 `# 미구현` 주석으로 비활성화되어 있던 것을 활성화하였다.

### 2.4 Main 코드 사용 위치

**`src/main.cpp`**:
```cpp
#include <gtsam_points/factors/integrated_ndt_factor.hpp>

// factor_type == 5 ("NDT") 일 때:
auto factor = gtsam::make_shared<gtsam_points::IntegratedNDTFactor_<gtsam_points::PointCloud>>(
    target_key, source_key, voxelmaps[target_index], frames[source_index]);
factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT7);
```

---

## 3. 기존 VGICP와의 비교

| 항목 | VGICP | NDT |
|------|-------|-----|
| 대응점 | `GaussianVoxel*` 직접 참조 | `NdtCorrespondence` 캐싱 구조체 |
| 역공분산 | `GaussianVoxel`에 내장 | 별도 계산 (`compute_ndt_inverse_covariance`) |
| 비용 함수 | D2D (distribution-to-distribution) | 가우시안 NDT score (Magnusson 2009) |
| 검색 모드 | DIRECT1/7/27 | DIRECT1/7/27 (동일) |
| 파라미터 | `resolution` | `resolution`, `outlier_ratio`, `regularization_epsilon` |

---

## 4. 수학적 배경

### 4.1 NDT 비용 함수 (Magnusson 2009, Eq. 6.9)

단일 포인트의 NDT 점수:

```
s(x) = -d1 * exp(-d2/2 * x^T * Sigma^{-1} * x)
```

여기서:
- `x = mean_B - T * mean_A` (잔차 벡터)
- `Sigma^{-1}` = 정규화된 역공분산 행렬
- `d1`, `d2` = NDT 가우시안 파라미터 (해상도, outlier 비율로부터 계산)

### 4.2 비음수 재정의

GTSAM LM 옵티마이저가 조기 수렴하는 것을 방지하기 위해 비용을 비음수로 재정의:

```
E(x) = -d1 * (1 - exp(-d2/2 * x^T * Sigma^{-1} * x))
```

- 완벽 정렬 시 (x=0): `E = 0`
- 정렬 불량 시: `E → |d1|`

Gradient와 Hessian은 원래 공식과 동일하다.

### 4.3 역공분산 정규화

공분산 행렬의 고유값 분해 후, 작은 고유값을 클램핑:

```
lambda_clamped = max(lambda_i, epsilon * lambda_max)
```

기본값 `epsilon = 1e-3`. 이를 통해 수치적으로 안정적인 역행렬을 보장한다.

### 4.4 버그 수정 사항 (원래 스켈레톤 대비)

| # | 항목 | 수정 전 | 수정 후 |
|---|------|---------|---------|
| 1 | 잔차 방향 | `transed_mean_A - mean_B` | `mean_B - transed_mean_A` (GTSAM 규약) |
| 2 | derivative_scale 부호 | `d1 * d2 / 2 * e_term` (음수) | `-d1 * d2 * e_term` (양수, PSD 헤시안) |
| 3 | 비용 비음수성 | `-d1 * e_term` (음수 가능) | `-d1 * (1 - e_term)` (항상 >= 0) |

---

## 5. 삭제된 파일

gtsam_points 라이브러리에 통합 완료 후, 프로젝트 로컬 NDT 파일은 삭제되었다:

```
[삭제됨] include/ndt/integrated_ndt_factor.hpp
[삭제됨] include/ndt/ndt_types.hpp
[삭제됨] include/ndt/impl/integrated_ndt_factor_impl.hpp
[삭제됨] src/integrated_ndt_factor.cpp
```

---

## 6. 관련 문서

| 문서 | 설명 |
|------|------|
| [uml.md](./uml.md) | UML 클래스/시퀀스 다이어그램 |
| [../archive/2026-02/ndt_implementation_ko.md](../archive/2026-02/ndt_implementation_ko.md) | 초기 구현 문서 (아카이브) |

---

**최종 수정일**: 2026-02-19
