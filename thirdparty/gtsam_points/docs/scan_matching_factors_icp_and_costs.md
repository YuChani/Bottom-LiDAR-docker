# Scan Matching Factors (ICP 계열) — 구조/수식/사용법

이 문서는 **gtsam_points의 Scan Matching Factors** 중 특히 **ICP 계열(ICP/GICP/VGICP/LOAM/CT-ICP)**을 중심으로,

1) 팩터가 GTSAM에 어떻게 붙는지(구조)
2) 각 팩터의 **cost function(목적함수) 수식**
3) 코드가 **정말 그 수식을 그대로 구현했는지(동일성 여부)**
4) C++ 사용법

을 정리합니다.

> 근거: 이 저장소의 헤더/구현을 직접 확인한 내용만 기술합니다.
> - 공통 베이스: `include/gtsam_points/factors/integrated_matching_cost_factor.hpp`
> - ICP 구현: `include/gtsam_points/factors/impl/integrated_icp_factor_impl.hpp`
> - GICP 구현: `include/gtsam_points/factors/impl/integrated_gicp_factor_impl.hpp`
> - VGICP 구현: `include/gtsam_points/factors/impl/integrated_vgicp_factor_impl.hpp`
> - LOAM 구현: `include/gtsam_points/factors/impl/integrated_loam_factor_impl.hpp`
> - CT-ICP 구현: `include/gtsam_points/factors/impl/integrated_ct_icp_factor_impl.hpp`

---

## 0) 용어/기호

- 타깃(맵/기준): B, 소스(정합할 스캔): A
- 포인트: \(p_A\), \(p_B\) (3D). 코드에서는 homogeneous 4D(`Eigen::Vector4d`)를 많이 사용하며 마지막 성분은 보통 1(포인트) 또는 0(노멀)입니다.
- 상대변환(타깃→소스 or 소스→타깃): 여기서는 코드 기준으로 `delta`는 **target과 source 사이의 변환**으로 사용되며,
  - ICP/GICP/VGICP 구현에서 `transed_mean_A = delta * mean_A` 형태로 **소스 포인트를 타깃 좌표계로 변환**하고
  - residual을 `mean_B - transed_mean_A`로 둡니다.

---

## 1) “Factor” 관점에서 본 공통 구조

### 1.1 IntegratedMatchingCostFactor: 스캔매칭 팩터의 공통 인터페이스

`gtsam_points::IntegratedMatchingCostFactor`는 `gtsam::NonlinearFactor`를 상속하고,

- **키 1개(unary)**: 고정된 target pose vs source pose
- **키 2개(binary)**: target pose vs source pose

두 형태를 지원합니다.

근거:
- 생성자/추상 메서드 정의: `include/gtsam_points/factors/integrated_matching_cost_factor.hpp`
  - `update_correspondences(const Eigen::Isometry3d& delta) const = 0;`
  - `evaluate(const Eigen::Isometry3d& delta, ... H/b ...) const = 0;`
  - `error(values)`, `linearize(values)`를 override

핵심 설계:

1) **correspondence 업데이트**(KNN/voxel lookup 등)와
2) **에러/헤시안 누적**(Gauss-Newton 형태)

을 한 팩터 내부에서 수행합니다.

### 1.2 선형화(linearize) 스타일: “포인트들을 reduce 해서 HessianFactor를 만든다”

ICP/GICP/VGICP/LOAM은 per-point(또는 per-feature) 오차를 더해

- \(H\) (6x6 블록들)과
- \(b\) (6x1)

를 누적한 뒤, 내부적으로 `gtsam::HessianFactor` 형태로 반환합니다.

병렬 reduce는 다음에 구현되어 있습니다.

- OpenMP: `include/gtsam_points/factors/impl/scan_matching_reduction.hpp` (`scan_matching_reduce_omp`)
- TBB: 같은 파일의 `scan_matching_reduce_tbb`

즉, 스캔 매칭 팩터는 GTSAM의 “일반적인 NonlinearFactor”처럼 residual vector를 직접 반환하기보다는,
**Gauss-Newton에 필요한 누적치(H, b)를 직접 만들고 HessianFactor를 반환**하는 형태입니다.

---

## 2) IntegratedICPFactor (Point-to-Point ICP)

### 2.1 목적함수(정석 ICP)

가장 표준적인 point-to-point ICP의 목적함수는

\[
\min_T \sum_i \lVert p_{B,\pi(i)} - T\,p_{A,i} \rVert^2
\]

여기서 \(\pi(i)\)는 최근접 대응점(일반적으로 KD-tree)입니다.

### 2.2 코드에서 실제로 계산하는 cost

**코드의 residual 정의**(대응점 존재 시):

- `transed_mean_A = delta * mean_A`
- `residual = mean_B - transed_mean_A`
- `error = residual.transpose() * residual`

근거:
- `include/gtsam_points/factors/impl/integrated_icp_factor_impl.hpp`
  - residual 구성: 207~209
  - error 정의: 215

> 주의: `mean_*`가 4D이지만 마지막 성분은 포인트면 1이고, 변환 후에도 1이므로 `mean_B - transed_mean_A`의 4번째 성분은 0이 됩니다.
> 따라서 `residual^T residual`은 사실상 3D norm과 동일하게 동작합니다.

### 2.3 대응점(correspondence) 생성

소스 포인트를 `delta`로 변환한 뒤, target KD-tree에서 1-NN을 찾고 거리 임계값을 적용합니다.

- KNN: `target_tree->knn_search(..., 1, ..., max_correspondence_distance_sq)`
- trimming: `k_sq_dist > max_correspondence_distance_sq`면 outlier(-1)

근거:
- `integrated_icp_factor_impl.hpp` 146~157

### 2.4 “수식 그대로 구현인가?”

**Point-to-point ICP는 “수식 그대로” 구현**되어 있다고 봐도 됩니다.

- (1) 대응점은 1-NN으로 생성
- (2) cost는 대응점 간 유클리드 거리 제곱합

---

## 3) IntegratedPointToPlaneICPFactor (Point-to-Plane ICP)

### 3.1 정석 point-to-plane ICP 목적함수

일반적으로 point-to-plane ICP는

\[
\min_T \sum_i \big(n_{B,\pi(i)}^\top (p_{B,\pi(i)} - T p_{A,i})\big)^2
\]

처럼 **법선 방향으로의 스칼라 투영**을 residual로 씁니다.

### 3.2 코드에서 실제로 계산하는 residual

이 구현은 `IntegratedICPFactor_` 내부에서 `use_point_to_plane=true`일 때 다음을 수행합니다.

- `residual = mean_B - transed_mean_A`
- `residual = normal_B.array() * residual.array()`  (성분별 곱)
- `error = residual.transpose() * residual`

근거:
- `include/gtsam_points/factors/impl/integrated_icp_factor_impl.hpp` 210~215

즉,

\[
e_i = \lVert n \odot (p_B - T p_A) \rVert^2
= (n_x \Delta x)^2 + (n_y \Delta y)^2 + (n_z \Delta z)^2
\]

형태이며, 이는

\[
\big(n^\top (p_B - T p_A)\big)^2
\]

와 **일반적으로 동일하지 않습니다**.

### 3.3 “수식 그대로 구현인가?” (중요 결론)

**아니오.** 이 구현은 “정석 point-to-plane ICP의 스칼라 투영 residual”을 그대로 쓰지 않고,
**노멀 벡터를 성분별 가중치(diagonal weighting)**처럼 적용한 형태입니다.

따라서 논문/교과서의 point-to-plane ICP 수식과 1:1로 동일한 구현이라고 말하면 안 됩니다.

> 참고로 CT-ICP 구현은 `dot(residual, normal)` 형태를 사용합니다(아래 6장).

---

## 4) IntegratedGICPFactor (Generalized ICP)

### 4.1 정석 GICP 목적함수(대표 형태)

GICP는 대응점 \((p_A, C_A)\), \((p_B, C_B)\)에 대해

\[
e_i = (p_B - T p_A)^\top \,(C_B + R C_A R^\top)^{-1}\, (p_B - T p_A)
\]

의 합을 최소화하는 형태가 흔합니다.

### 4.2 코드에서 실제로 계산하는 cost

- residual: `residual = mean_B - delta*mean_A`
- fused covariance: `RCR = cov_B + delta.matrix() * cov_A * delta.matrix().transpose()`
- mahalanobis: `mahalanobis.topLeftCorner<3,3>() = RCR.topLeftCorner<3,3>().inverse()`
- error: `residual.transpose() * mahalanobis * residual`

근거:
- `include/gtsam_points/factors/impl/integrated_gicp_factor_impl.hpp`
  - fused cov 계산/캐시: 177~191
  - error 계산: 266

### 4.3 캐시 모드(FusedCovCacheMode)

`FusedCovCacheMode::{FULL, COMPACT, NONE}`로

- FULL: 4x4(double) 형태를 캐시
- COMPACT: 3x3 상삼각을 float로 압축 캐시
- NONE: linearization point에서 매번 계산

근거:
- enum 정의: `include/gtsam_points/factors/integrated_gicp_factor.hpp` 17~24
- 캐시 로직: `integrated_gicp_factor_impl.hpp` 151~196 및 250~264

### 4.4 “수식 그대로 구현인가?”

**대체로 예.** 흔히 알려진 GICP의 D2D Mahalanobis 거리 형태를 그대로 계산합니다.

---

## 5) IntegratedVGICPFactor (Voxelized GICP)

### 5.1 목적함수 개념

VGICP는 target을 voxel 단위로 요약해 각 voxel에

- mean \(\mu_v\)
- covariance \(C_v\)

를 두고, 소스 포인트가 속한 voxel과의 D2D 비용(사실상 point-to-distribution)을 누적합니다.

### 5.2 코드에서의 대응점: “최근접점”이 아니라 “voxel lookup”

`update_correspondences`에서

1) \(p_A\)를 `delta`로 변환
2) 그 위치의 voxel 좌표를 구해
3) voxel index를 lookup해서 해당 voxel 포인터를 correspondence로 저장

근거:
- `include/gtsam_points/factors/impl/integrated_vgicp_factor_impl.hpp` 115~135

### 5.3 코드에서 실제로 계산하는 cost

GICP와 동일한 형태의 Mahalanobis 비용을 voxel mean/cov로 계산합니다.

- residual: `mean_B(voxel.mean) - delta*mean_A`
- fused covariance: `cov_B(voxel.cov) + R cov_A R^T`
- error: `residual^T * mahalanobis * residual`

근거:
- `integrated_vgicp_factor_impl.hpp` 201~226

### 5.4 “수식 그대로 구현인가?”

**예(개념적으로).**

- 차이는 correspondence가 NN이 아니라 voxel association이라는 점
- 비용 자체는 GICP와 같은 형태의 fused covariance Mahalanobis 거리

---

## 6) IntegratedCT_ICPFactor (Continuous-Time ICP)

CT-ICP는 한 스캔 내부에서 pose가 시간에 따라 변한다는 가정으로, **스캔 시작/끝 pose 2개**를 변수로 둡니다.

### 6.1 팩터가 연결하는 변수(키)

- `gtsam::KeyVector{source_t0_key, source_t1_key}`

근거:
- `include/gtsam_points/factors/impl/integrated_ct_icp_factor_impl.hpp` 25

### 6.2 시간 보간(스캔 내 pose 테이블)

스캔 포인트의 `time`을 기반으로 time table을 만들고, t0→t1의 상대변환을 Log/Exp로 보간해 각 time bin의 pose를 만듭니다.

근거:
- time_table/time_indices 구축: `integrated_ct_icp_factor_impl.hpp` 41~55
- 보간 및 도함수(derivatives) 계산: 202~229

### 6.3 코드에서 실제로 계산하는 cost (point-to-plane 형태)

`error(values)`에서 (correspondence 존재 시)

- `transed_source_pt = pose.transformFrom(source_pt)`
- `residual = transed_source_pt - target_pt`
- `error_scalar = dot(residual, target_normal)`
- `return error_scalar^2`

근거:
- `integrated_ct_icp_factor_impl.hpp` 117~122

### 6.4 “수식 그대로 구현인가?”

**예(정석 point-to-plane residual 형태).**

CT-ICP는 dot(n, residual)을 사용하므로, 3장에서 지적한 “성분별 곱”과 달리
일반적인 point-to-plane ICP의 스칼라 residual 구조에 부합합니다.

---

## 7) IntegratedLOAMFactor (LOAM 스타일: point-to-edge + point-to-plane)

`IntegratedLOAMFactor_`는 내부적으로

- `IntegratedPointToEdgeFactor_`
- `IntegratedPointToPlaneFactor_`

두 팩터를 합산하는 구조입니다.

근거:
- wrapper: `include/gtsam_points/factors/impl/integrated_loam_factor_impl.hpp` 87~91, 461~477

### 7.1 Point-to-edge residual

대응점 2개(x_j, x_l)를 잡고, 변환된 점 x_i가 그 선분에 만드는 **선까지의 거리**를 cross product로 계산합니다.

- `c_inv = 1 / ||x_j - x_l||`
- `residual = ( (x_i - x_j) × (x_i - x_l) ) * c_inv`
- `error = ||residual||^2`

근거:
- `integrated_loam_factor_impl.hpp` 330~336

이 형태는 LOAM 류 구현에서 흔히 쓰는 point-to-line 거리 기반 잔차와 정합합니다.

### 7.2 Point-to-plane residual

여기는 대응점 3개로 평면 법선을 만들고,

- normal = (x_j-x_l) × (x_j-x_m) 를 정규화
- residual = x_j - T x_i
- `plane_residual = residual.array() * normal.array()`
- `error = ||plane_residual||^2`

근거:
- `integrated_loam_factor_impl.hpp` 151~164

즉, 여기서도 point-to-plane을 **dot(n, residual)**이 아니라 **성분별 곱** 형태로 계산합니다.

### 7.3 “수식 그대로 구현인가?”

- Point-to-edge: **대체로 예** (표준적인 point-to-line 거리 잔차 형태)
- Point-to-plane: **아니오** (dot 기반이 아니라 성분별 곱 기반)

---

## 8) 공통 사용법 (C++): “팩터를 그래프에 넣고 최적화”

### 8.1 최소 예시: IntegratedICPFactor

아래는 `src/example/basic_scan_matching.cpp`의 핵심만 남긴 형태입니다.

```cpp
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose3.h>

#include <gtsam_points/util/read_points.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/factors/integrated_icp_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>

int main() {
  auto target_pts = gtsam_points::read_points("data/kitti_00/000000.bin");
  auto source_pts = gtsam_points::read_points("data/kitti_00/000001.bin");

  auto target = std::make_shared<gtsam_points::PointCloudCPU>(target_pts);
  auto source = std::make_shared<gtsam_points::PointCloudCPU>(source_pts);

  gtsam::Values init;
  init.insert(0, gtsam::Pose3());
  init.insert(1, gtsam::Pose3());

  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
    0, gtsam::Pose3(), gtsam::noiseModel::Isotropic::Precision(6, 1e6));

  auto icp = gtsam::make_shared<gtsam_points::IntegratedICPFactor>(0, 1, target, source);
  icp->set_max_correspondence_distance(5.0);
  graph.add(icp);

  gtsam_points::LevenbergMarquardtExtParams params;
  params.set_verbose();
  gtsam_points::LevenbergMarquardtOptimizerExt opt(graph, init, params);
  const auto& result = opt.optimize();

  std::cout << result.at<gtsam::Pose3>(1).matrix() << std::endl;
  return 0;
}
```

### 8.2 GICP 사용 시 필수: covariances

GICP/VGICP는 cov가 필요합니다(없으면 abort).

- GICP: target/source 모두 `frame::has_covs` 필요
  - 근거: `integrated_gicp_factor_impl.hpp` 37~45
- VGICP: source에 cov 필요
  - 근거: `integrated_vgicp_factor_impl.hpp` 37~40

따라서 프레임에 cov를 미리 추가해야 합니다(예: `gtsam_points::estimate_covariances(...)`).

### 8.3 CT-ICP 사용 시 필수: source times + target normals

CT-ICP는

- source: points + times
- target: points (+ linearize 시 normals)

를 요구합니다.

근거:
- source times 요구: `integrated_ct_icp_factor_impl.hpp` 36~38
- linearize 시 target normals 요구: 132~136

---

## 9) 결론: “논문/정석 ICP 수식과 동일한가?” 요약표

| Factor | 코드의 핵심 비용 형태 | 정석 수식과 동일? | 근거 파일 |
|---|---|---:|---|
| IntegratedICPFactor (p2p) | \(\sum ||p_B - T p_A||^2\) | 예 | `integrated_icp_factor_impl.hpp` |
| IntegratedPointToPlaneICPFactor (p2plane) | \(\sum ||n \odot (p_B - T p_A)||^2\) | **아니오** | `integrated_icp_factor_impl.hpp` |
| IntegratedGICPFactor | \(\sum r^T (C_B + R C_A R^T)^{-1} r\) | 예(대표형) | `integrated_gicp_factor_impl.hpp` |
| IntegratedVGICPFactor | voxel mean/cov에 대한 Mahalanobis | 예(개념적으로) | `integrated_vgicp_factor_impl.hpp` |
| IntegratedLOAMFactor (edge) | point-to-line 거리 기반 | 대체로 예 | `integrated_loam_factor_impl.hpp` |
| IntegratedLOAMFactor (plane) | \(\sum ||n \odot (p - p_{plane})||^2\) | **아니오** | `integrated_loam_factor_impl.hpp` |
| IntegratedCT_ICPFactor | \(\sum (n^T (T(t)p - q))^2\) | 예 | `integrated_ct_icp_factor_impl.hpp` |
