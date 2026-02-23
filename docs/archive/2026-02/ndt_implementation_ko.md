# gtsam-points NDT Factor 구현 문서

## 목차
1. [개요](#개요)
2. [NDT 알고리즘 이론](#ndt-알고리즘-이론)
3. [gtsam-points 아키텍처 분석](#gtsam-points-아키텍처-분석)
4. [구현 과정](#구현-과정)
5. [GICP/VGICP와의 차이점](#gicpvgicp와의-차이점)
6. [코드 구조](#코드-구조)
7. [테스트 및 검증](#테스트-및-검증)

---

## 개요

본 문서는 gtsam-points 라이브러리에 NDT (Normal Distributions Transform) 매칭 코스트 팩터를 구현한 과정을 기록합니다.

### 구현 목표
- **기존 패턴 준수**: VGICP 팩터의 구조와 스타일을 따름
- **Voxel 기반 대응 관계**: `GaussianVoxelMapCPU`를 활용한 효율적인 검색
- **해석적 Jacobian**: Lie 대수를 사용한 정확한 미분 계산
- **병렬 처리**: OpenMP 및 TBB를 통한 성능 최적화

### 참고 자료
- **논문**: 
  - Biber & Straßer, "The Normal Distributions Transform: A New Approach to Laser Scan Matching", IROS2003
  - Magnusson, "The Three-Dimensional Normal-Distributions Transform", PhD thesis, 2009
- **참조 구현**:
  - koide3/ndt_omp (https://github.com/koide3/ndt_omp)
  - tier4/ndt_omp (https://github.com/tier4/ndt_omp)
  - gtsam-points의 VGICP 팩터

---

## NDT 알고리즘 이론

### 기본 개념

NDT는 점군을 복셀(voxel) 공간으로 분할하고, 각 복셀 내 점들을 정규 분포로 모델링합니다.

```
각 복셀 V에 대해:
- 평균 (mean): μ = (1/N) Σ p_i
- 공분산 (covariance): Σ = (1/N) Σ (p_i - μ)(p_i - μ)^T
```

### 매칭 코스트

두 점군 P_source와 P_target을 정렬할 때, 소스 점 p를 변환 T로 이동시킨 후, 해당 위치의 타겟 복셀의 정규 분포와의 적합도를 측정합니다:

```
d = T(p) - μ_target
error = d^T * Σ^(-1) * d  (Mahalanobis 거리)
```

전체 매칭 스코어는 모든 점에 대한 누적:

```
E(T) = Σ_i [ -d1 * exp(-d2/2 * error_i) ]
```

여기서 d1, d2는 outlier_ratio와 resolution으로부터 계산되는 가우시안 파라미터입니다 (Magnusson 2009, Eq. 6.8-6.21).

### 정규화 (Regularization)

공분산 행렬이 특이(singular)하거나 ill-conditioned인 경우를 방지하기 위해 고유값 클램핑을 적용:

```
λ_i_regularized = max(λ_i, ε * λ_max)
```

기본값으로 ε = 1e-3 사용.

---

## gtsam-points 아키텍처 분석

### 계층 구조

```
IntegratedMatchingCostFactor (gtsam::NonlinearFactor 상속)
    ↑
    ├─ IntegratedGICPFactor_<TargetFrame, SourceFrame>
    ├─ IntegratedVGICPFactor_<SourceFrame>
    └─ IntegratedNDTFactor_<SourceFrame>  ← 신규 구현
```

### 핵심 인터페이스

모든 매칭 코스트 팩터는 다음 두 메서드를 구현해야 합니다:

#### 1. `update_correspondences()`
```cpp
virtual void update_correspondences(const Eigen::Isometry3d& delta) const;
```
- **목적**: 현재 변환 delta에서 소스-타겟 대응 관계 갱신
- **NDT 구현**: 
  - 소스 점을 delta로 변환
  - 복셀 좌표 계산
  - DIRECT1/7/27 모드에 따라 인근 복셀 검색
  - 최소 Mahalanobis 거리를 가진 복셀 선택

#### 2. `evaluate()`
```cpp
virtual double evaluate(
  const Eigen::Isometry3d& delta,
  Eigen::Matrix<double, 6, 6>* H_target,
  Eigen::Matrix<double, 6, 6>* H_source,
  Eigen::Matrix<double, 6, 6>* H_target_source,
  Eigen::Matrix<double, 6, 1>* b_target,
  Eigen::Matrix<double, 6, 1>* b_source) const;
```
- **목적**: 에러 계산 및 Jacobian 행렬 구성
- **반환값**: 총 에러 (모든 점의 누적)
- **출력**: Hessian (H) 및 gradient (b) 행렬

### VGICP vs NDT 차이점

| 항목 | VGICP | NDT |
|------|-------|-----|
| **소스 점 정보** | 공분산 필요 (`frame::cov`) | 점 위치만 필요 |
| **융합 공분산** | `Σ_target + T * Σ_source * T^T` | `Σ_target`만 사용 |
| **캐싱 전략** | Mahalanobis 행렬 캐싱 (FULL/COMPACT/NONE) | 복셀 역공분산 lazy 계산 |
| **대응 관계** | 가장 가까운 복셀 (단일) | 인근 복셀 중 최소 거리 (DIRECT7) |

---

## 구현 과정

### Wave 1: 데이터 구조 확장

#### 1-1. GaussianVoxel 확장

파일: `include/gtsam_points/types/gaussian_voxelmap_cpu.hpp`

기존 `GaussianVoxel` 구조체에 NDT 전용 필드 추가:

```cpp
struct GaussianVoxel {
  // 기존 필드
  Eigen::Vector4d mean;
  Eigen::Matrix4d cov;
  
  // NDT 전용 추가 필드
  Eigen::Matrix4d inv_cov;          // 역공분산 (캐시)
  bool inv_cov_valid;                // 역공분산 유효성 플래그
  double gauss_d1, gauss_d2;         // NDT 가우시안 파라미터
  
  // 메서드
  void compute_inverse_covariance(double regularization_epsilon);
  static void compute_ndt_params(double resolution, double outlier_ratio, 
                                   double& d1, double& d2);
};
```

**`compute_inverse_covariance()` 구현**:
1. Eigen의 `SelfAdjointEigenSolver`로 고유값 분해
2. 고유값 클램핑: `λ_i = max(λ_i, ε * λ_max)`
3. 정규화된 공분산 재구성
4. 역행렬 계산 후 `inv_cov`에 저장
5. `inv_cov_valid = true` 설정

**`compute_ndt_params()` 구현**:
Magnusson 2009의 공식 (Eq. 6.17-6.21) 적용:
```cpp
c1 = 10.0 * (1.0 - outlier_ratio);
c2 = outlier_ratio / (resolution * resolution * resolution);
d3 = -log(c2);
d1 = -log(c1 + c2) - d3;
d2 = -2.0 * log((-log(c1 * exp(-0.5) + c2) - d3) / d1);
```

### Wave 2: NDT Factor 구조 정의

#### 2-1. 헤더 파일 작성

파일: `include/gtsam_points/factors/integrated_ndt_factor.hpp`

**NDTSearchMode 열거형**:
```cpp
enum class NDTSearchMode {
  DIRECT1,   // 현재 복셀만 (가장 빠름)
  DIRECT7,   // 현재 + 6개 면 인접 복셀 (기본값, 균형)
  DIRECT27   // 현재 + 26개 인접 복셀 (3x3x3 큐브, 가장 정확)
};
```

**클래스 구조**:
```cpp
template <typename SourceFrame = gtsam_points::PointCloud>
class IntegratedNDTFactor_ : public IntegratedMatchingCostFactor {
public:
  // 생성자 (Binary와 Unary 버전)
  IntegratedNDTFactor_(gtsam::Key target_key, gtsam::Key source_key, ...);
  IntegratedNDTFactor_(const gtsam::Pose3& fixed_target_pose, 
                        gtsam::Key source_key, ...);
  
  // 설정 메서드
  void set_num_threads(int n);
  void set_resolution(double res);
  void set_outlier_ratio(double ratio);
  void set_regularization_epsilon(double eps);
  void set_search_mode(NDTSearchMode mode);
  
  // 쿼리 메서드
  int num_inliers() const;
  double inlier_fraction() const;
  
private:
  // 핵심 구현 (순수 가상 함수 오버라이드)
  void update_correspondences(const Eigen::Isometry3d& delta) const override;
  double evaluate(...) const override;
  
  // 멤버 변수
  int num_threads;
  double resolution, outlier_ratio, regularization_epsilon;
  NDTSearchMode search_mode;
  mutable std::vector<const GaussianVoxel*> correspondences;
  mutable double gauss_d1, gauss_d2;
  std::shared_ptr<const GaussianVoxelMapCPU> target_voxels;
  std::shared_ptr<const SourceFrame> source;
};
```

### Wave 3: 핵심 로직 구현

#### 3-1. 대응 관계 갱신

파일: `include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp`

**알고리즘 흐름**:

1. **NDT 파라미터 계산**:
```cpp
GaussianVoxel::compute_ndt_params(resolution, outlier_ratio, gauss_d1, gauss_d2);
```

2. **검색 모드별 오프셋 생성**:
```cpp
std::vector<Eigen::Vector3i> neighbor_offsets;
switch (search_mode) {
  case DIRECT7:
    offsets = [(0,0,0), (±1,0,0), (0,±1,0), (0,0,±1)];
    break;
  // ...
}
```

3. **각 소스 점에 대한 대응 관계 찾기** (병렬 처리):
```cpp
for each source point i:
  pt_transformed = delta * source_point[i]
  voxel_coord = target_voxels->voxel_coord(pt_transformed)
  
  best_voxel = nullptr
  min_distance = infinity
  
  for each offset in neighbor_offsets:
    neighbor_coord = voxel_coord + offset
    voxel = lookup_voxel(neighbor_coord)
    
    if voxel exists:
      // Lazy 역공분산 계산
      if not voxel->inv_cov_valid:
        voxel->compute_inverse_covariance(regularization_epsilon)
      
      // Mahalanobis 거리 계산
      diff = pt_transformed - voxel->mean
      distance = diff^T * voxel->inv_cov * diff
      
      if distance < min_distance:
        min_distance = distance
        best_voxel = voxel
  
  correspondences[i] = best_voxel
```

#### 3-2. 에러 및 Jacobian 계산

**에러 계산**:
```cpp
residual = transed_source_point - target_voxel->mean
residual_3d = residual.head<3>()  // 동차 좌표 → 3D
inv_cov_3d = target_voxel->inv_cov.topLeftCorner<3,3>()
mahalanobis_dist = residual_3d^T * inv_cov_3d * residual_3d
error = 0.5 * mahalanobis_dist  // GTSAM least-squares 형식
```

**Jacobian 계산** (Lie 대수):

Target 자코비안 (타겟 프레임에 대한 미분):
```cpp
J_target = [ -SO3::Hat(transed_point), I_3x3 ]  // 4x6 행렬
```

Source 자코비안 (소스 프레임에 대한 미분):
```cpp
J_source = [ delta.R * SO3::Hat(source_point), -delta.R ]  // 4x6 행렬
```

Mahalanobis 가중치 적용:
```cpp
J_target_weighted = J_target^T * inv_cov_B  // 6x4
J_source_weighted = J_source^T * inv_cov_B  // 6x4
```

Hessian 및 gradient 누적:
```cpp
H_target += J_target_weighted * J_target
H_source += J_source_weighted * J_source
H_target_source += J_target_weighted * J_source
b_target += J_target_weighted * residual
b_source += J_source_weighted * residual
```

#### 3-3. 병렬 처리

**OpenMP 경로**:
```cpp
#pragma omp parallel for num_threads(num_threads) schedule(guided, 8)
for (int i = 0; i < num_points; i++) {
  perpoint_task(i);
}
```

**TBB 경로**:
```cpp
tbb::parallel_for(
  tbb::blocked_range<int>(0, num_points, 8),
  [&](const tbb::blocked_range<int>& range) {
    for (int i = range.begin(); i < range.end(); i++) {
      perpoint_task(i);
    }
  }
);
```

병렬 리덕션은 `scan_matching_reduce_omp/tbb` 유틸리티 함수를 통해 Hessian 및 gradient를 안전하게 병합합니다.

### Wave 4: 빌드 시스템 통합

#### 4-1. CMakeLists.txt 수정

파일: `CMakeLists.txt`

```cmake
add_library(gtsam_points SHARED
  # ... 기존 파일들 ...
  src/gtsam_points/factors/integrated_vgicp_factor.cpp
  src/gtsam_points/factors/integrated_ndt_factor.cpp  # ← 추가
  src/gtsam_points/factors/integrated_loam_factor.cpp
  # ...
)
```

#### 4-2. 템플릿 인스턴스화

파일: `src/gtsam_points/factors/integrated_ndt_factor.cpp`

```cpp
#include <gtsam_points/types/point_cloud.hpp>
#include <gtsam_points/factors/integrated_ndt_factor.hpp>
#include <gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp>

// PointCloud 인스턴스화
template class gtsam_points::IntegratedNDTFactor_<gtsam_points::PointCloud>;

// DummyFrame 인스턴스화 (테스트용)
#include <gtsam_points/types/dummy_frame.hpp>
template class gtsam_points::IntegratedNDTFactor_<gtsam_points::DummyFrame>;
```

---

## GICP/VGICP와의 차이점

### 1. 알고리즘적 차이

| 특성 | GICP | VGICP | NDT |
|------|------|-------|-----|
| **이론적 기반** | Distribution-to-Distribution | GICP + Voxel 가속 | 정규 분포 변환 |
| **소스 점 요구사항** | 점 + 공분산 | 점 + 공분산 | 점만 |
| **타겟 표현** | KdTree (GICP) / Voxel (VGICP) | GaussianVoxelMap | GaussianVoxelMap |
| **매칭 코스트** | `(d^T)(Σ_A + Σ_B)^(-1)(d)` | `(d^T)(Σ_A + R*Σ_B*R^T)^(-1)(d)` | `(d^T)(Σ_B)^(-1)(d)` |
| **대응 관계** | 최근접 이웃 | 현재 복셀 | 인근 복셀 중 최적 (DIRECT7) |

### 2. 구현 복잡도

**VGICP**:
- 융합 공분산 계산 필요
- Mahalanobis 행렬 캐싱 전략 (FULL/COMPACT/NONE)
- 더 많은 메모리 사용

**NDT**:
- 역공분산만 캐싱 (더 단순)
- Lazy 계산 (필요시에만 inv_cov 계산)
- 메모리 효율적

### 3. 성능 특성

**GICP**: O(N * log M) - KdTree 검색  
**VGICP**: O(N) - 해시맵 기반 복셀 접근  
**NDT**: O(N * k) - k = 1, 7, 27 (검색 모드에 따라)

### 4. 적용 사례

- **GICP**: 고밀도 점군, 정밀한 정렬 필요
- **VGICP**: 대규모 점군, 실시간 SLAM
- **NDT**: 저밀도 점군, 노이즈가 많은 환경 (LiDAR SLAM)

---

## 코드 구조

### 파일 구성

```
gtsam_points/
├── include/gtsam_points/
│   ├── types/
│   │   └── gaussian_voxelmap_cpu.hpp  (수정: GaussianVoxel 확장)
│   └── factors/
│       ├── integrated_ndt_factor.hpp  (신규: 클래스 선언)
│       └── impl/
│           └── integrated_ndt_factor_impl.hpp  (신규: 템플릿 구현)
├── src/gtsam_points/
│   └── factors/
│       └── integrated_ndt_factor.cpp  (신규: 템플릿 인스턴스화)
└── CMakeLists.txt  (수정: integrated_ndt_factor.cpp 추가)
```

### 주요 클래스 다이어그램

```
                 gtsam::NonlinearFactor
                          ↑
                          |
           IntegratedMatchingCostFactor
              ↑                    ↑
              |                    |
   ┌──────────┴────────┬──────────┴────────┐
   |                   |                   |
IntegratedGICPFactor  IntegratedVGICPFactor  IntegratedNDTFactor
   |                   |                   |
   |                   └─ GaussianVoxelMapCPU
   └─ KdTree                               └─ GaussianVoxelMapCPU
```

### 데이터 흐름

```
1. 팩터 생성
   IntegratedNDTFactor(target_key, source_key, target_voxels, source)
        ↓
2. GTSAM 최적화 루프
   optimizeOneIteration()
        ↓
3. 팩터 선형화
   linearize(values)
        ↓
4. 대응 관계 갱신
   update_correspondences(delta)
     - 소스 점 변환
     - 복셀 검색 (DIRECT7)
     - Mahalanobis 거리 최소화
        ↓
5. 에러 및 Jacobian 계산
   evaluate(delta, H_target, H_source, ...)
     - 병렬 처리 (OpenMP/TBB)
     - 에러 누적
     - Hessian/gradient 구성
        ↓
6. HessianFactor 반환 → GTSAM 솔버
```

---

## 테스트 및 검증

### 1. 단위 테스트 (예정)

파일: `src/test/test_ndt.cpp`

**테스트 항목**:
- [ ] Binary factor 생성 및 평가
- [ ] Unary factor 생성 및 평가
- [ ] DIRECT1/7/27 모드 검증
- [ ] 병렬 처리 (NONE/OMP/TBB) 일관성 테스트
- [ ] Numerical Jacobian 검증 (analytical vs numerical, 오차 < 1e-5)
- [ ] 수렴 테스트 (회전 < 0.5°, 이동 < 0.02m)

### 2. Jacobian 검증 방법

GTSAM의 `numericalDerivative` 함수를 사용하여 해석적 Jacobian과 수치적 Jacobian을 비교:

```cpp
// 해석적 Jacobian 계산
Matrix H_analytical = factor->evaluateError(pose, H_target, H_source);

// 수치적 Jacobian 계산
Matrix H_numerical_target = numericalDerivative11(
  [&](const Pose3& p) { return factor->evaluateError(p, source_pose); },
  target_pose
);

// 오차 검증
EXPECT(assert_equal(H_analytical, H_numerical_target, 1e-5));
```

### 3. 통합 테스트 (예정)

**시나리오**:
1. KITTI 데이터셋 연속 프레임 정렬
2. VGICP vs NDT 정확도 비교
3. 실행 시간 벤치마크
4. 메모리 사용량 프로파일링

### 4. 빌드 검증

```bash
cd gtsam_points/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON
make -j$(nproc)
ctest -R test_ndt
```

---

## 사용 예시

### 기본 사용법

```cpp
#include <gtsam_points/factors/integrated_ndt_factor.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>

// 타겟 복셀맵 생성
auto target_voxels = std::make_shared<GaussianVoxelMapCPU>(1.0);  // 1m 해상도
target_voxels->insert(*target_cloud);

// NDT 팩터 생성
auto ndt_factor = gtsam::make_shared<IntegratedNDTFactor>(
  X(0),  // target key
  X(1),  // source key
  target_voxels,
  source_cloud
);

// 파라미터 설정
ndt_factor->set_num_threads(8);
ndt_factor->set_search_mode(NDTSearchMode::DIRECT7);
ndt_factor->set_outlier_ratio(0.55);
ndt_factor->set_regularization_epsilon(1e-3);

// GTSAM 그래프에 추가
graph.add(ndt_factor);

// 최적화
LevenbergMarquardtOptimizer optimizer(graph, initial_values);
Values result = optimizer.optimize();
```

### 고급 설정

```cpp
// DIRECT1 모드 (가장 빠름, 복셀 하나만 검색)
ndt_factor->set_search_mode(NDTSearchMode::DIRECT1);

// DIRECT27 모드 (가장 정확, 3x3x3 큐브 검색)
ndt_factor->set_search_mode(NDTSearchMode::DIRECT27);

// Outlier ratio 조정 (더 많은 outlier 가정)
ndt_factor->set_outlier_ratio(0.7);  // 70% outlier

// 더 강한 정규화 (ill-conditioned 공분산 대응)
ndt_factor->set_regularization_epsilon(1e-2);

// 대응 관계 통계 확인
std::cout << "Inliers: " << ndt_factor->num_inliers() << std::endl;
std::cout << "Inlier fraction: " << ndt_factor->inlier_fraction() << std::endl;
```

---

## 향후 작업

### 우선순위 1 (필수)
- [ ] 단위 테스트 작성 (`test_ndt.cpp`)
- [ ] Numerical Jacobian 검증 통과
- [ ] 빌드 시스템 검증 (모든 플랫폼)

### 우선순위 2 (권장)
- [ ] KITTI 벤치마크 실행
- [ ] VGICP와 성능 비교 문서화
- [ ] 메모리 프로파일링

### 우선순위 3 (개선)
- [ ] GPU 버전 구현 (`IntegratedNDTFactorGPU`)
- [ ] Colored NDT (intensity 활용)
- [ ] Continuous-time NDT (CT-NDT)

---

## 참고 문헌

1. Biber, P., & Straßer, W. (2003). "The normal distributions transform: A new approach to laser scan matching." *Proceedings 2003 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2003)*.

2. Magnusson, M. (2009). "The three-dimensional normal-distributions transform: an efficient representation for registration, surface analysis, and loop detection." *PhD thesis, Örebro University*.

3. Koide, K., Yokozuka, M., Oishi, S., & Banno, A. (2021). "Voxelized GICP for Fast and Accurate 3D Point Cloud Registration." *2021 IEEE International Conference on Robotics and Automation (ICRA)*.

4. koide3/ndt_omp. GitHub repository: https://github.com/koide3/ndt_omp

5. tier4/ndt_omp. GitHub repository: https://github.com/tier4/ndt_omp

6. GTSAM Documentation. https://gtsam.org/

---

## 변경 이력

| 날짜 | 버전 | 변경 내용 | 작성자 |
|------|------|-----------|--------|
| 2026-02-13 | 1.0 | 최초 작성 - NDT 팩터 구현 완료 | Sisyphus |

---

**문서 종료**
