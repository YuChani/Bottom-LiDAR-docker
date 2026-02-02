# Scan Matching Factors 튜토리얼 (ICP/GICP/VGICP/LOAM/Colored/CT/Expression/GPU)

이 문서는 **gtsam_points**의 scan matching factor들(ICP 계열)을 대상으로,

- 수식(기호) ↔ 코드 변수 ↔ 구현(**file:line**)을 **1:1 매핑**하고
- “iteration(반복)”이 의미하는 바를
  - (A) optimizer outer iteration
  - (B) LM 내부 lambda 탐색(inner try)
  - (C) factor correspondence update
  로 **구분해서** 설명합니다.

> 주의: 이 문서는 저장소 코드만 근거로 작성합니다(추측 금지).

---

## 0. 전제/공통 기호

### 0.1 Target/Source, 포즈/변환

- target frame: B (맵/기준)
- source frame: A (정합할 스캔)
- 상대변환: `delta`

코드에서 `delta`는 **source 포인트를 target 좌표계로 변환**하는 형태로 사용됩니다.

- 예: ICP에서 `transed_mean_A = delta * mean_A` (source → target) 후 `residual = mean_B - transed_mean_A`
  - 근거: `include/gtsam_points/factors/impl/integrated_icp_factor_impl.hpp:207-209`

### 0.2 공통 residual 표기

여기서

- (homogeneous) source point: \(p_{A,i}\) (`mean_A`, `frame::point(*source, i)`)
- target correspondence: \(p_{B,\pi(i)}\) (`mean_B`, `frame::point(*target, target_index)`)
- (변환된 소스): \(\tilde p_{A,i} = \mathrm{delta}\, p_{A,i}\) (`transed_mean_A`)

ICP/GICP/VGICP류는 우선

\[
r_i = p_{B,\pi(i)} - \tilde p_{A,i}
\]

형태의 residual을 구성하는 것이 공통입니다.

---

## 1. “iteration”을 정확히 나누기

### 1.1 LM optimizer: outer iterate vs inner tryLambda

#### (A) Outer iteration에서의 linearize 호출

LM optimizer 확장 구현은 `linearize()`에서 **GPU hook의 linearize를 먼저 호출**한 뒤, GTSAM graph linearize를 수행합니다.

- `linearization_hook_->linearize(state_->values);` 후 `graph_.linearize(state_->values);`
  - 근거: `src/gtsam_points/optimizers/levenberg_marquardt_ext.cpp:141-145`

이 시점에서, 일반적인 nonlinear factor의 `linearize(values)`가 호출되며(또는 GPU hook이 관리하는 factor라면 hook이 미리 batch linearize 수행),
scan matching factor들에서 **correspondence update가 “필요하면” 수행**됩니다(각 factor 섹션 참고).

#### (B) Inner lambda 탐색(tryLambda)에서는 relinearize 없이 error만 반복

`tryLambda()`는 damped system을 풀어 `newValues`를 만든 뒤, **hook error + graph error만 계산**합니다.

- `linearization_hook_->error(newValues);` 후 `newError = calc_error(graph_, newValues);`
  - 근거: `src/gtsam_points/optimizers/levenberg_marquardt_ext.cpp:243-248`

즉 LM의 내부 lambda 탐색(여러 번의 damping/lambda 조정)은 **대응점 갱신/linearize를 다시 수행하지 않고**,
현재 linearization 기준에서 **error 평가만** 반복합니다.

### 1.2 ISAM2Ext: relinearize 직전에 GPU hook batch linearize

ISAM2 확장 구현에서도 linearization 단계에서 GPU hook가 먼저 수행됩니다.

- `linearization_hook->clear();`
- `linearization_hook->add(nonlinearFactors_);`
- `linearization_hook->linearize(theta_);`
- 이후 `nonlinearFactors_.linearize(theta_)`

근거: `src/gtsam_points/optimizers/isam2_ext.cpp:234-242`

### 1.3 GPU hook(NonlinearFactorSetGPU): batch linearize/error

GPU factor들은 hook에 의해 모아져 batch로 실행됩니다.

#### (A) batch linearize

1) 각 factor가 CPU input buffer에 linearization point를 기록
2) input buffer H2D
3) 각 factor가 `issue_linearize(...)`
4) 각 factor `sync()`
5) output buffer D2H
6) 각 factor가 `store_linearized(...)`

근거: `src/gtsam_points/cuda/nonlinear_factor_set_gpu.cpp:64-139`

#### (B) batch error evaluation

1) 각 factor가 CPU input buffer에 evaluation point를 기록
2) H2D
3) 각 factor가 `issue_compute_error(...)`
4) sync
5) D2H
6) 각 factor가 `store_computed_error(...)`

근거: `src/gtsam_points/cuda/nonlinear_factor_set_gpu.cpp:141-218`

---

## 2. Factor별: correspondence update / residual / Jacobian(선형화)

### 2.1 IntegratedICPFactor (Point-to-Point)

#### Correspondence update (1-NN KD-tree)

- tolerance gating(회전/이동 변화가 작으면 갱신 생략):
  - `diff = delta.inverse() * last_correspondence_point`,
    `diff_rot < tol_rot && diff_trans < tol_trans` 이면 `do_update=false`
  - 근거: `include/gtsam_points/factors/impl/integrated_icp_factor_impl.hpp:128-141`
- KD-tree 1NN + 거리 제한:
  - `target_tree->knn_search(..., max_correspondence_distance_sq)`
  - `num_found==0 || k_sq_dist > max...`이면 correspondence=-1
  - 근거: `.../integrated_icp_factor_impl.hpp:146-157`

#### Residual / error

- `transed_mean_A = delta * mean_A`
- `residual = mean_B - transed_mean_A`
- `error = residual.transpose() * residual`

근거: `.../integrated_icp_factor_impl.hpp:204-216`

#### Jacobian / Hessian 누적

- `J_target`, `J_source`를 구성 후
- `H += J^T J`, `b += J^T residual` 형태로 누적

근거: `.../integrated_icp_factor_impl.hpp:220-238`

---

### 2.2 IntegratedPointToPlaneICPFactor (주의: dot(n,r) 아님)

이 구현은 통상적인 point-to-plane ICP(스칼라 residual \(n^\top r\))가 아니라,
residual 벡터에 **성분별로 normal을 곱하는(diagonal weighting)** 형태입니다.

#### Residual

- `residual = mean_B - transed_mean_A`
- `residual = normal_B.array() * residual.array();`

근거: `include/gtsam_points/factors/impl/integrated_icp_factor_impl.hpp:207-213`

즉

\[
e_i = \lVert n \odot (p_B - T p_A) \rVert^2
\]

입니다.

#### Jacobian

- `J_target = normal_B.asDiagonal() * J_target`
- `J_source = normal_B.asDiagonal() * J_source`

근거: `.../integrated_icp_factor_impl.hpp:228-232`

---

### 2.3 IntegratedGICPFactor

#### Correspondence update

- tolerance gating + 1NN KD-tree:
  - `do_update` 계산: `...:135-143`
  - `target_tree->knn_search(..., max_correspondence_distance_sq)`로 correspondence 설정: `...:162-170`

근거: `include/gtsam_points/factors/impl/integrated_gicp_factor_impl.hpp:135-170`

#### Fused covariance / Mahalanobis

- `RCR = cov_B + delta.matrix() * cov_A * delta.matrix().transpose()`
- `mahalanobis.topLeftCorner<3,3>() = RCR.topLeftCorner<3,3>().inverse()`

근거: `.../integrated_gicp_factor_impl.hpp:177-191`

#### Error

- `error = residual.transpose() * mahalanobis * residual;` (주석: `r^T * M * r`)

근거: `.../integrated_gicp_factor_impl.hpp:247-268`

#### Jacobian/Hessian 누적

- `J_target_mahalanobis = J_target^T * M`, `J_source_mahalanobis = J_source^T * M`
- `H += (J^T M) J`, `b += (J^T M) r`

근거: `.../integrated_gicp_factor_impl.hpp:271-287`

---

### 2.4 IntegratedVGICPFactor (CPU, voxel correspondence)

#### Correspondence update = voxel lookup

- `pt = delta * point(source,i)`
- `coord = target_voxels->voxel_coord(pt)`
- `voxel_id = lookup_voxel_index(coord)`
- 없으면 `correspondences[i]=nullptr`, 있으면 voxel 포인터 저장

근거: `include/gtsam_points/factors/impl/integrated_vgicp_factor_impl.hpp:114-135`

#### Mahalanobis

- `RCR = voxel->cov + delta.matrix() * cov_A * delta.matrix().transpose()`
- `mahalanobis.topLeftCorner<3,3>() = RCR.topLeftCorner<3,3>().inverse()`

근거: `.../integrated_vgicp_factor_impl.hpp:137-147`

#### Error / 누적

- `residual = mean_B(voxel.mean) - delta*mean_A`
- `error = residual^T * M * residual`
- `H,b` 누적은 GICP와 동일 패턴

근거: `.../integrated_vgicp_factor_impl.hpp:195-257`

---

### 2.5 IntegratedVGICPFactorGPU (GPU)

#### Hook 기반 linearize/error

GPU factor는 hook(NonlinearFactorSetGPU)이 호출하는 entry-point들을 구현하고,
GPU에서 계산된 결과를 factor 내부에 **캐시**한 뒤 `linearize()`에서 그 캐시를 사용합니다.

- `linearize(values)`는 `linearization_result` 캐시를 우선 사용하며, 없으면 sync linearize로 fallback
  - 근거: `src/gtsam_points/factors/integrated_vgicp_factor_gpu.cpp:185-197`
- hook가 호출:
  - `set_linearization_point`: input buffer에 `calc_delta(values)` 기록
    - 근거: `.../integrated_vgicp_factor_gpu.cpp:218-223`
  - `issue_linearize`: `reset_inliers(...)` 후 `issue_linearize(...)`
    - 근거: `...:230-237`
  - `store_linearized`: 결과 캐시 + `update_inliers(num_inliers)`
    - 근거: `...:239-245`
  - `issue_compute_error` / `store_computed_error`로 error 캐시
    - 근거: `...:247-268`

#### Inlier 업데이트 “항상” 하지 않음(큰 pose 변화 때만)

- `reset_inliers(...)`에서
  - `force_update || source_inliers==nullptr || large_displacement(...)`이면 inlier set 재생성
  - 아니면 `inlier_evaluation_point_gpu=nullptr`
- `update_inliers(...)`는 `inlier_evaluation_point_gpu==nullptr`면 즉시 return

근거: `src/gtsam_points/factors/integrated_vgicp_derivatives_inliers.cu:46-67`

---

### 2.6 IntegratedLOAMFactor (edge + plane)

LOAM factor는 내부적으로 edge/plane 두 팩터를 합산합니다.

- `edge_factor->update_correspondences(delta);`
- `plane_factor->update_correspondences(delta);`
- `validate_correspondences();`

근거: `include/gtsam_points/factors/impl/integrated_loam_factor_impl.hpp:444-449`

#### Error/Hessian 합산

- `error = edge.evaluate(...)`
- plane의 `H,b`를 따로 계산해 더함

근거: `.../integrated_loam_factor_impl.hpp:452-477`

#### Correspondence validation (scan line reject)

- edge: 두 target 점의 수직각(v_theta)이 너무 비슷하면 reject
- plane: 세 점 모두 같은 scan line에 가까우면 reject

근거: `.../integrated_loam_factor_impl.hpp:487-529`

---

### 2.7 IntegratedColorConsistencyFactor (photometric)

#### Correspondence update (KD-tree on XYZ+intensity)

- `pt = delta * point(source,i); pt[3]=intensity(source,i)`
- `target_tree->knn_search(pt.data(), 1, ...)`

근거: `include/gtsam_points/factors/impl/integrated_color_consistency_factor_impl.hpp:101-129`

#### Photometric residual

코드는 target normal로 source점을 평면에 projection하고, intensity gradient로 1차 근사를 사용합니다.

- `projected = transed_A - (transed_A-mean_B).dot(normal_B) * normal_B`
- `offset = projected - mean_B`
- `error_photo = intensity_B + gradient_B.dot(offset) - intensity_A`
- `err = error_photo * photometric_term_weight * error_photo`

근거: `.../integrated_color_consistency_factor_impl.hpp:175-193`

#### Jacobian/Hessian 누적

- `J_projected_transed = I - normal_B * normal_B^T` (단, (3,3)=0)
- `H += J^T * w * J`, `b += J^T * w * error_photo`

근거: `.../integrated_color_consistency_factor_impl.hpp:206-221`

---

### 2.8 IntegratedColoredGICPFactor (geom + photo 혼합)

#### Correspondence update

- ColorConsistency와 동일하게 (XYZ+intensity)로 1NN
- tolerance gating도 동일 패턴

근거: `include/gtsam_points/factors/impl/integrated_colored_gicp_factor_impl.hpp:101-129`

#### Mahalanobis(4x4) 구성의 특징

- `RCR = target_cov + delta * cov_A * delta^T`
- `RCR(3,3)=1.0`로 설정 후 inverse
- inverse 후 `mahalanobis(3,3)=0.0`

근거: `.../integrated_colored_gicp_factor_impl.hpp:134-139`

#### Error

- `geometric_term_weight = 1 - photometric_term_weight`
- geometric: `residual_geom = transed_A - mean_B`
  - `error_geom = residual_geom^T * geometric_term_weight * mahalanobis[i] * residual_geom`
- photo: `residual_photo = intensity_B + gradient_B.dot(offset) - intensity_A`
  - `error_photo = residual_photo * photometric_term_weight * residual_photo`

근거: `.../integrated_colored_gicp_factor_impl.hpp:174-252`

---

### 2.9 IntegratedCT_ICPFactor (continuous-time point-to-plane)

#### Error (dot 기반 point-to-plane)

- `transed_source_pt = pose.transformFrom(source_pt)`
- `residual = transed_source_pt - target_pt`
- `error_scalar = dot(residual, target_normal)`
- 반환은 `error_scalar^2`

근거: `include/gtsam_points/factors/impl/integrated_ct_icp_factor_impl.hpp:92-122`

#### Linearize 시 매번 pose/ correspondence 갱신

- `update_poses(values); update_correspondences();`

근거: `.../integrated_ct_icp_factor_impl.hpp:132-140`

---

### 2.10 IntegratedCT_GICPFactor (continuous-time + Mahalanobis)

#### Linearize 시 매번 correspondence 갱신

- `this->update_poses(values); this->update_correspondences();`

근거: `include/gtsam_points/factors/impl/integrated_ct_gicp_factor_impl.hpp:103-106`

#### Correspondence update

- time-index에 해당하는 pose로 source point를 변환 후 1NN
- 거리 제한 밖이면 `mahalanobis=0`
- 안이면 `RCR = cov_B + pose * cov_A * pose^T`, `mahalanobis.block<3,3>() = RCR.block<3,3>().inverse()`

근거: `.../integrated_ct_gicp_factor_impl.hpp:171-199`

#### Error

- `error = residual.transpose() * mahalanobis[i] * residual`

근거: `.../integrated_ct_gicp_factor_impl.hpp:88-92`

---

### 2.11 Expression ICP (experimental)

#### ICPFactorExpr: correspondence update 시점

Expression 기반 ICP factor는 `unwhitenedError(values, H)`에서

- `target_index < 0 || H`일 때 correspondence를 갱신합니다.
  - “첫 호출” 또는 “linearization(=H 요청)” 때 갱신

근거: `src/gtsam_points/factors/experimental/expression_icp_factor.cpp:37-45`

#### Residual expression

- `target_point - transformFrom(delta, source)`

근거: `.../expression_icp_factor.cpp:56-62`

#### IntegratedICPFactorExpr: graph를 linearize한 뒤 HessianFactor로 변환

- `auto linearized = graph->linearize(values);`
- `auto hessian = linearized->hessian(ordering);`
- Hessian blocks로 `gtsam::HessianFactor` 생성

근거: `.../expression_icp_factor.cpp:71-85`

---

### 2.12 Continuous-time ICP (expression, experimental)

#### CTICPFactorExpr: correspondence update 시점

- `target_index < 0 || H`일 때 correspondence 갱신

근거: `src/gtsam_points/factors/experimental/continuous_time_icp_factor.cpp:59-78`

---

## 3. Wiring(사용 패턴) 체크리스트

### 3.1 (CUDA) GPU factor를 쓰려면 hook 등록이 필요

데모 코드에서 CUDA 빌드 시 hook를 등록합니다.

- `LinearizationHook::register_hook([] { return create_nonlinear_factor_set_gpu(); });`

근거: `src/demo/demo_matching_cost_factors.cpp:50-53`

테스트에서도 동일 패턴:

- `LinearizationHook::register_hook(...)`

근거: `src/test/test_matching_cost_factors.cpp:97-100`

### 3.2 MatchingCostFactorDemo: factor 선택과 공통 knobs

`demo_matching_cost_factors.cpp`의 `create_factor(...)`는 factor별로 다음 설정을 적용합니다.

- ICP / ICP_PLANE / GICP:
  - `set_correspondence_update_tolerance(rot, trans)`
  - `set_num_threads(num_threads)`
- VGICP:
  - `set_num_threads(num_threads)`
- VGICP_GPU:
  - `IntegratedVGICPFactorGPU(...)` 생성

근거: `src/demo/demo_matching_cost_factors.cpp:197-231`

### 3.3 Colored registration demo

`demo_colored_registration.cpp`에서

- ColoredICP = (PointToPlaneICP) + (ColorConsistency) 두 factor 조합
- ColoredGICP = 단일 factor

근거: `src/demo/demo_colored_registration.cpp:117-129`

---

## 4. Build / Verification (환경 재현)

### 4.1 GTSAM 버전 요구

이 리포의 CMake는 GTSAM 4.2를 required로 지정합니다.

- `find_package(GTSAM 4.2 REQUIRED)`
- `find_package(GTSAM_UNSTABLE 4.2 REQUIRED)`

근거: `CMakeLists.txt:32-34`

또한 README는 “Tested … with **GTSAM 4.3a0**” 및 “base GTSAM version changed… rebuild and install **GTSAM 4.3a0**”를 명시합니다.

근거: `README.md:1-2`, `README.md:18-20`

#### (현재 환경에서의 실제 configure 실패 로그)

CPU only configure를 시도하면, 시스템에 설치된 GTSAM이 **4.1.1**이라서(요구: **4.2**) CMake가 중단됩니다.

근거(실행 결과):

```text
$ cmake -S . -B build-cpu -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON -DBUILD_DEMO=OFF -DBUILD_WITH_CUDA=OFF
...
Could not find a configuration file for package "GTSAM" that is compatible
with requested version "4.2".

The following configuration files were considered but not accepted:

  /usr/lib/cmake/GTSAM/GTSAMConfig.cmake, version: 4.1.1
  /lib/cmake/GTSAM/GTSAMConfig.cmake, version: 4.1.1
```

### 4.2 (README 기반) GTSAM 4.3a0 설치 & gtsam_points 빌드

README에 있는 명령을 그대로 따릅니다.

근거(전체 섹션): `README.md:Installation > Install from source`

### 4.3 재현 가능한 빌드/테스트 커맨드 예시

```bash
# CPU only
cmake -S . -B build-cpu -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON -DBUILD_DEMO=OFF -DBUILD_WITH_CUDA=OFF
cmake --build build-cpu -j
ctest --test-dir build-cpu

# CUDA on
cmake -S . -B build-cuda -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON -DBUILD_DEMO=OFF -DBUILD_WITH_CUDA=ON
cmake --build build-cuda -j
ctest --test-dir build-cuda
```

---

## 5. 참고: 기존 문서

이미 `docs/scan_matching_factors_icp_and_costs.md`에 ICP/GICP/VGICP/LOAM/CT-ICP 중심 설명이 있습니다.
본 문서는 여기에 더해

- optimizer iteration 의미(outer/inner)
- GPU hook/batching 구조
- colored/expression/CT-GICP 등

을 “file:line 근거 중심”으로 보강한 버전입니다.
