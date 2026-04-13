# NDT 가평균/가분산 계산 방식 상세 분석

## 1. 문서 목적

이 문서는 Bottom-LiDAR-docker 프로젝트의 NDT 구현에서 `GaussianVoxel`에 저장되는
**가평균(pseudo-mean)** 과 **가분산(pseudo-covariance)** 이 어떻게 계산되는지를
소스 코드 수준에서 추적하고 수학적으로 해설한다.

특히 다음 세 가지 물음에 답한다.

1. 각 복셀의 `mean`과 `cov`는 어디서, 어떻게 만들어지는가?
2. CPU 경로와 GPU 경로의 수식이 서로 같은가?
3. 이 방식이 "전통적 NDT의 공간 공분산"과 어떻게 다른가?

---

## 2. 전체 파이프라인 개요

NDT 가평균/가분산 계산은 크게 5단계로 이루어진다.

### 2.1 파이프라인 진입점 (main.cpp)

실제 파이프라인의 시작은 `src/main.cpp`의 프레임 로딩 루프이다 (줄 245-291).

```cpp
// src/main.cpp, 줄 245-291 (핵심부만 발췌)

// [Step 0] float → double 변환 + homogeneous 좌표 (x,y,z) → (x,y,z,1)
std::vector<Eigen::Vector4d> points(points_f.size());
std::transform(points_f.begin(), points_f.end(), points.begin(), [](const Eigen::Vector3f& p) {
  return (Eigen::Vector4d() << p.cast<double>(), 1.0).finished();      // L248-251
});

// [Step 1] 포인트별 공분산 추정 (Stage 1 진입)
auto covs = gtsam_points::estimate_covariances(points);                // L257

// [Step 2] PointCloud 프레임 구성
auto frame = std::make_shared<gtsam_points::PointCloudCPU>();          // L261
frame->add_points(points);                                              // L266
frame->add_covs(covs);                                                  // L267

// [Step 3] GaussianVoxelMap 생성 + 삽입 (Stage 2 진입)
auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(resolution);  // L289
voxelmap->insert(*frame);   // → IncrementalVoxelMap::insert()         // L290
```

이 코드가 아래 Stage 다이어그램의 시작점이다.

### 2.2 Stage 다이어그램

```
[입력 포인트 클라우드]
        |
        v
[main.cpp L248-251] float→double, homogeneous 좌표 변환
        |
        v
[Stage 1] 포인트별 공분산 추정 (KNN 기반)
  main.cpp L257 → covariance_estimation.cpp
        |
        v  포인트마다 (pos, cov) 쌍 보유
        |
[main.cpp L266-267] frame->add_points() / add_covs()
        |
        v
[Stage 2] GaussianVoxel 집계 (CPU)
  main.cpp L290 → IncrementalVoxelMap::insert()
                 → GaussianVoxel::add() / finalize()
        |
[Stage 3] GPU 집계 경로 (선택적)
  gaussian_voxelmap_gpu.cu
  → accumulate_points_kernel / finalize_voxels_kernel
        |
        v  복셀마다 (mean, cov) 확정
        |
[Stage 4] NDT Factor에서 역공분산 정규화
  integrated_ndt_factor.hpp
  → compute_ndt_inverse_covariance()
        |
        v  복셀마다 (mean, inv_cov) 확정
        |
[Stage 5] NDT/LightNDT Factor에서 비용 함수 평가 (Section 7)
  integrated_ndt_factor_impl.hpp / integrated_light_ndt_factor_impl.hpp
  → update_correspondences() + evaluate()
```

### 2.3 각 Stage 핵심 요약

| Stage | 수행 연산 | 결과 |
|-------|-----------|------|
| 1 | KNN 이웃으로 포인트별 공분산 $\Sigma_i$ 계산 | 각 포인트에 `cov` 부착 |
| 2/3 | 복셀 내 포인트들의 위치와 공분산을 합산 후 N으로 나눔 | 복셀에 `mean`, `cov` 저장 |
| 4 | 고유값 분해 + 클램핑으로 역공분산 $\Sigma^{-1}$ 생성 | NDT 비용 함수에 사용 |
| 5 | 가평균/가분산을 사용한 대응점 탐색 + 비용/Hessian 계산 | 최적화 반복에 사용 (Section 7) |

---

## 3. Stage 1: 포인트별 공분산 추정 (Per-point Covariance Estimation)

### 3.1 소스 위치

`thirdparty/gtsam_points/src/gtsam_points/features/covariance_estimation.cpp`, 줄 18-77

### 3.2 핵심 알고리즘

각 포인트 $p$ 에 대해 KdTree로 k개의 최근접 이웃 $\{p_1, \ldots, p_k\}$ 를 탐색한 뒤,
다음 누적식으로 평균과 공분산을 계산한다.

**누적 단계** (줄 33-43):

$$
S_{\mu} = \sum_{j=1}^{k} p_j, \quad S_{\Sigma} = \sum_{j=1}^{k} p_j p_j^T
$$

**평균 계산**:

$$
\mu_i = \frac{S_{\mu}}{k}
$$

**공분산 계산** (편향 추정량, $k$로 나눔):

$$
\Sigma_i = \frac{S_{\Sigma}}{k} - \mu_i \mu_i^T = \frac{S_{\Sigma} - \mu_i \cdot S_{\mu}^T}{k}
$$

> 주의: 비편향 추정량은 $k-1$로 나누지만, 이 구현은 $k$로 나누는 **편향 추정량**을 사용한다.

코드로 표현하면 다음과 같다.

```cpp
// covariance_estimation.cpp, 줄 33-43
Vector4d sum_points = Vector4d::Zero();
Matrix4d sum_covs   = Matrix4d::Zero();

for (int j = 0; j < num_found; j++) {
  const auto& pt = points.points[indices[j]];
  sum_points += pt;
  sum_covs   += pt * pt.transpose();
}

Vector4d mean = sum_points / num_found;
Matrix4d cov  = (sum_covs - mean * sum_points.transpose()) / num_found;
```

### 3.3 EIG 정규화 (선택적)

`CovarianceEstimationParams::EIG` 방식이 활성화되면, 고유값 분해 후 **고유값을 완전히 교체**한다
(줄 48-54). 단순 클램핑이 아님에 주의한다.

```cpp
// covariance_estimation.cpp, 줄 48-54
case CovarianceEstimationParams::EIG: {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig;
  eig.computeDirect(cov.block<3, 3>(0, 0));

  covs[i].setZero();
  covs[i].block<3, 3>(0, 0) = eig.eigenvectors()
      * params.eigen_values.asDiagonal()
      * eig.eigenvectors().inverse();
}
```

여기서 `params.eigen_values`의 기본값은 `(1e-3, 1.0, 1.0)` 이다. 즉:

$$
\Sigma_i^{\text{EIG}} = V \, \mathrm{diag}(10^{-3},\; 1.0,\; 1.0) \, V^{-1}
$$

원래 고유값 $(\lambda_1, \lambda_2, \lambda_3)$은 **완전히 무시**되고, 고유벡터(방향)만 보존된다.
결과적으로 모든 포인트의 공분산은 **평면형 타원체(plane-like ellipsoid)** 로 강제된다:

- **법선 방향** (최소 고유값 방향): 분산 $10^{-3}$ → 매우 납작
- **접선 방향** (나머지 두 축): 분산 $1.0$ → 단위 분산

이 처리가 GICP/VGICP 스타일의 핵심이다. KNN으로 구한 고유벡터에서 **방향 정보만 추출**하고,
크기(eigenvalue)는 고정값으로 대체하여 모든 포인트가 동일한 스케일의 평면 불확실성을 갖게 한다.

> **주의**: `covs[i].setZero()`로 4×4 행렬 전체를 먼저 0으로 초기화한 뒤 3×3 블록만 채우므로,
> 4번째 행/열은 항상 0이 된다 (homogeneous 좌표의 w 성분에 해당).

이 단계를 마치면 각 포인트는 자신만의 $\Sigma_i$ 를 보유한 상태가 된다.

---

## 4. Stage 2: GaussianVoxel 가평균/가분산 계산 (CPU)

### 4.1 소스 위치

- 구조체 정의: `thirdparty/gtsam_points/include/gtsam_points/types/gaussian_voxelmap_cpu.hpp`, 줄 13-52
- `add()` / `finalize()`: `thirdparty/gtsam_points/src/gtsam_points/types/gaussian_voxelmap_cpu.cpp`, 줄 23-47

### 4.2 GaussianVoxel 구조체

```cpp
// gaussian_voxelmap_cpu.hpp, 줄 13-52
struct GaussianVoxel {
  bool     finalized  = false;
  size_t   num_points = 0;
  Vector4d mean       = Vector4d::Zero();   // 가평균
  Matrix4d cov        = Matrix4d::Zero();   // 가분산
  double   intensity  = 0.0;
};
```

생성자(줄 21)에서 모든 필드가 0으로 초기화된다. `size()`는 항상 1을 반환한다.

**Homogeneous 좌표 (4D) 처리**:

CPU 경로에서 포인트 위치는 `Vector4d`로 표현된다. 실제 3D 좌표 $(x, y, z)$에 $w = 1$을 붙여
$(x, y, z, 1)$ 형태의 4D homogeneous 좌표를 사용한다 (`main.cpp` L248-251).
이에 대응하여 공분산은 `Matrix4d` (4×4)로 저장되지만, **실질적인 데이터는 좌상 3×3 블록**에만 있다.
EIG 정규화에서 `covs[i].setZero()` 후 `block<3,3>(0,0)`만 채우므로,
4번째 행/열은 항상 0이다 (`cov(3,3) = 0`).

이 설계는 SE(3) 변환 행렬(4×4)과의 연산 호환성을 위한 것이다. $T \cdot p$로 회전+평행이동을
한 번에 적용할 수 있다. GPU 경로에서는 `Vector3f` / `Matrix3f` (3D)를 직접 사용하므로
이 4D 패딩이 없다.

### 4.3 add(): 누적

포인트 클라우드의 포인트 $i$가 어떤 복셀에 삽입될 때 호출된다.

```cpp
// gaussian_voxelmap_cpu.cpp, 줄 23-37
void GaussianVoxel::add(const Setting& setting, const PointCloud& points, size_t i) {
  if (finalized) {
    this->finalized  = false;
    this->mean      *= num_points;   // 합산값으로 복원
    this->cov       *= num_points;   // 합산값으로 복원
  }
  num_points++;
  this->mean += points.points[i];   // 위치 누적
  this->cov  += points.covs[i];     // 포인트별 공분산 누적
  // ... (intensity 처리 생략)
}
```

**un-finalize 트릭** (줄 24-28): 이미 `finalize()`가 호출된 복셀에 새 포인트가 들어오면,
저장된 평균/공분산에 `num_points`를 다시 곱해 합산값을 복원한 뒤 계속 누적한다.

복셀에 $N$개의 포인트 $p_1, \ldots, p_N$ 이 누적된 시점에서:

$$
\text{mean (sum)} = \sum_{i=1}^{N} p_i, \quad \text{cov (sum)} = \sum_{i=1}^{N} \Sigma_i
$$

### 4.4 finalize(): 나눗셈

```cpp
// gaussian_voxelmap_cpu.cpp, 줄 39-47
void GaussianVoxel::finalize() {
  if (finalized) return;
  mean /= num_points;   // 가평균
  cov  /= num_points;   // 가분산
  finalized = true;
}
```

**가평균 (pseudo-mean)**:

$$
\bar{\mu} = \frac{1}{N} \sum_{i=1}^{N} p_i
$$

**가분산 (pseudo-covariance)**:

$$
\bar{\Sigma} = \frac{1}{N} \sum_{i=1}^{N} \Sigma_i
$$

이 두 수식에서 핵심을 짚어야 한다. $\bar{\Sigma}$ 는 복셀 내 포인트들의 **공간적 분포**로 계산한 공분산이
아니라, **각 포인트의 로컬 이웃 공분산을 단순 평균**한 값이다. 그래서 "가분산"이라 부른다.

### 4.5 IncrementalVoxelMap::insert(): 복셀 생성부터 finalize까지

`GaussianVoxel::add()`와 `finalize()`는 개별 복셀 수준의 연산이다.
이들을 호출하는 상위 흐름은 `IncrementalVoxelMap::insert()`에 있다.

**소스 위치**: `thirdparty/gtsam_points/include/gtsam_points/ann/impl/incremental_voxelmap_impl.hpp`, 줄 31-68

```cpp
// incremental_voxelmap_impl.hpp, 줄 31-68
template <typename VoxelContents>
void IncrementalVoxelMap<VoxelContents>::insert(const PointCloud& points) {
  // [Phase 1] 포인트별 복셀 삽입
  for (size_t i = 0; i < points.size(); i++) {
    const Eigen::Vector3i coord = fast_floor(points.points[i] * inv_leaf_size).head<3>();

    auto found = voxels.find(coord);
    if (found == voxels.end()) {
      // 새 복셀 생성
      auto voxel = std::make_shared<...>(VoxelInfo(coord, lru_counter), VoxelContents());
      found = voxels.emplace_hint(found, coord, flat_voxels.size());
      flat_voxels.emplace_back(voxel);
    }

    auto& [info, voxel] = *flat_voxels[found->second];
    info.lru = lru_counter;
    voxel.add(voxel_setting, points, i);   // ← GaussianVoxel::add() 호출
  }

  // [Phase 2] LRU 퇴거 (주기적)
  if ((++lru_counter) % lru_clear_cycle == 0) {
    // lru_horizon 이전에 마지막으로 접근된 복셀 제거
    flat_voxels.erase(
      std::remove_if(flat_voxels.begin(), flat_voxels.end(), [&](auto& v) {
        return v->first.lru + lru_horizon < lru_counter;
      }),
      flat_voxels.end()
    );
    // 해시맵 재구성
    voxels.clear();
    for (size_t i = 0; i < flat_voxels.size(); i++)
      voxels[flat_voxels[i]->first.coord] = i;
  }

  // [Phase 3] 전체 복셀 finalize
  for (auto& voxel : flat_voxels) {
    voxel->second.finalize();   // ← GaussianVoxel::finalize() 호출
  }
}
```

**핵심 흐름 정리**:

1. **Phase 1 — 포인트별 삽입**: 프레임의 모든 포인트를 순회하며, 각 포인트의 3D 좌표를 복셀 해상도로 양자화하여 복셀 좌표를 구한다. 해당 복셀이 없으면 새로 생성하고, `voxel.add()`를 호출해 mean/cov 합산값에 누적한다.

2. **Phase 2 — LRU 퇴거**: `lru_clear_cycle`마다 오래된 복셀을 삭제한다. `lru_horizon` 이전에 마지막으로 접근(add)된 복셀이 퇴거 대상이다. 이는 슬라이딩 윈도우 방식의 점진적 맵 관리를 위한 것이다.

3. **Phase 3 — 일괄 finalize**: **모든 포인트 삽입이 끝난 후**, 맵의 **전체** 복셀에 대해 `finalize()`를 호출한다. 이미 `finalized == true`인 복셀은 내부에서 `if (finalized) return;`으로 즉시 반환된다.

> **중요**: `finalize()`는 개별 포인트 삽입 직후가 아니라, 한 프레임의 모든 포인트가 삽입된 뒤 일괄 호출된다.
> 또한 `add()` 내부의 un-finalize 트릭 덕분에, 이미 finalize된 복셀에 새 포인트가 들어오면
> 합산값으로 복원 → 누적 → 다시 finalize 흐름이 정상적으로 동작한다.

---

## 5. Stage 3: GPU 구현

### 5.1 소스 위치

`thirdparty/gtsam_points/src/gtsam_points/types/gaussian_voxelmap_gpu.cu`, 줄 92-176

### 5.2 accumulate_points_kernel (줄 92-152)

GPU 커널에서는 여러 스레드가 동시에 같은 복셀에 쓸 수 있으므로 `atomicAdd`를 사용한다.

```cuda
// gaussian_voxelmap_gpu.cu, 줄 92-152
atomicAdd(&num_points, 1);

for (int j = 0; j < 3; j++) {
  atomicAdd(voxel_mean.data() + j, mean[j]);   // 위치 누적
}
for (int j = 0; j < 9; j++) {
  atomicAdd(voxel_cov.data() + j, cov.data()[j]);  // 공분산 누적
}
```

수학적 연산은 CPU의 `add()`와 동일하다. 차이는 타입뿐이다.

| 항목 | CPU | GPU |
|------|-----|-----|
| 위치 벡터 | `Vector4d` (double) | `Vector3f` (float) |
| 공분산 행렬 | `Matrix4d` (double) | `Matrix3f` (float) |
| 동기화 | 단일 스레드 | `atomicAdd` |
| intensity | 합산 후 평균 | **`atomicMax`** (최대값) |

> **intensity 처리의 CPU/GPU 차이**: CPU 경로에서는 intensity를 합산 후 `finalize()`에서 N으로 나누어
> 평균을 구한다. 반면 GPU 경로에서는 `atomicMax`를 사용하여 복셀 내 포인트 중 **최대 intensity** 를
> 저장한다. mean/cov 계산에는 영향 없지만, intensity 기반 후처리에서 결과가 달라질 수 있다.

### 5.3 finalize_voxels_kernel (줄 154-176)

```cuda
// gaussian_voxelmap_gpu.cu, 줄 154-176
voxel_mean /= num_points;   // CPU finalize()와 동일
voxel_covs /= num_points;   // CPU finalize()와 동일
```

수식은 CPU와 완전히 같다.

$$
\bar{\mu}_{GPU} = \frac{1}{N} \sum_{i=1}^{N} p_i, \quad \bar{\Sigma}_{GPU} = \frac{1}{N} \sum_{i=1}^{N} \Sigma_i
$$

GPU는 수치 정밀도(float vs double)만 다를 뿐, 동일한 가평균/가분산 정의를 따른다.

---

## 6. Stage 4: NDT Factor에서의 역공분산 정규화

### 6.1 소스 위치

`include/ndt/integrated_ndt_factor.hpp`, 줄 45-55

### 6.2 compute_ndt_inverse_covariance()

복셀의 가분산 $\bar{\Sigma}$는 그대로 역행렬을 구하면 수치 불안정 문제가 생길 수 있다.
작은 고유값이 0에 가까우면 역행렬이 폭발하기 때문이다. 이를 막기 위해 고유값 클램핑을 적용한다.

```cpp
// integrated_ndt_factor.hpp, 줄 45-55
inline Eigen::Matrix4d compute_ndt_inverse_covariance(
    const Eigen::Matrix4d& cov, double regularization_epsilon = 1e-3) {

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(cov);
  Eigen::Vector4d eigenvalues = solver.eigenvalues();

  double lambda_max = eigenvalues.maxCoeff();
  Eigen::Vector4d clamped = eigenvalues.array()
                              .max(regularization_epsilon * lambda_max)
                              .matrix();

  Eigen::Matrix4d cov_reg = eigenvectors
                            * clamped.asDiagonal()
                            * eigenvectors.transpose();
  return cov_reg.inverse();
}
```

### 6.3 수식 해설

고유값 분해:

$$
\bar{\Sigma} = V \, \mathrm{diag}(\lambda_1, \ldots, \lambda_4) \, V^T
$$

고유값 클램핑:

$$
\lambda_i^{\text{clamped}} = \max\!\left(\lambda_i,\; \varepsilon \cdot \lambda_{\max}\right), \quad \varepsilon = 10^{-3}
$$

정규화된 공분산 재조합:

$$
\bar{\Sigma}_{\text{reg}} = V \, \mathrm{diag}\!\left(\lambda_1^{\text{clamped}}, \ldots, \lambda_4^{\text{clamped}}\right) V^T
$$

역공분산:

$$
\bar{\Sigma}_{\text{reg}}^{-1} = V \, \mathrm{diag}\!\left(\frac{1}{\lambda_1^{\text{clamped}}}, \ldots, \frac{1}{\lambda_4^{\text{clamped}}}\right) V^T
$$

이 역공분산이 NDT 비용 함수의 마할라노비스 거리 항에 들어간다.

$$
E = -d_1 \left(1 - \exp\!\left(-\frac{d_2}{2} \, r^T \bar{\Sigma}_{\text{reg}}^{-1} r\right)\right)
$$

여기서 $r = \bar{\mu}_{\text{target}} - T \cdot \bar{\mu}_{\text{source}}$ 는 잔차 벡터다.

### 6.4 inv_cov_cache: 역공분산 캐싱 메커니즘

실제 NDT Factor 구현에서는 매 대응점 탐색(linearization)마다 모든 복셀의 역공분산을 재계산하지 않는다.
대신 **최초 1회만 계산 후 캐싱**하는 최적화가 적용되어 있다.

**소스 위치**: `include/ndt/impl/integrated_ndt_factor_impl.hpp`, 줄 135-143

```cpp
// integrated_ndt_factor_impl.hpp, 줄 135-143
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

**동작 방식**:

1. `update_correspondences()`가 처음 호출될 때 `inv_cov_cached == false`이다.
2. 타겟 복셀맵의 **모든 복셀**에 대해 `compute_ndt_inverse_covariance()`를 호출하여
   정규화된 역공분산 $\bar{\Sigma}_{\text{reg}}^{-1}$을 계산한다.
3. 결과를 `inv_cov_cache[voxel_index]`에 저장하고 `inv_cov_cached = true`로 플래그를 설정한다.
4. 이후 linearization에서는 캐시된 역공분산을 인덱스로 즉시 조회한다.

**캐시 무효화**: `regularization_epsilon`이 변경되면 캐시가 무효화되어야 하지만,
현재 구현에서는 생성 시 고정되므로 실질적으로 무효화가 발생하지 않는다.

**LightNDT에도 동일 패턴 적용**: `integrated_light_ndt_factor_impl.hpp` (줄 131-138)에도
같은 `inv_cov_cache` 패턴이 사용된다.

이 캐싱은 고유값 분해(eigendecomposition)가 $O(\text{num\_voxels})$회 필요한 비싼 연산이기 때문에
중요하다. 캐시 없이 매 linearization마다 재계산하면 최적화 반복 횟수에 비례해 비용이 증가한다.

---

## 7. NDT/LightNDT Factor에서의 가평균/가분산 활용

앞 섹션들에서 가평균/가분산이 **어떻게 계산되는지**를 다뤘다면,
이 섹션에서는 사용자가 구현한 NDT Factor와 LightNDT Factor에서
가평균/가분산이 **어떻게 소비되는지**를 추적한다.

### 7.1 소스 파일 위치

| 파일 | 역할 | 줄 수 |
|------|------|-------|
| `include/ndt/integrated_ndt_factor.hpp` | NDT Factor 헤더 (클래스 선언, `NdtCorrespondence`, `compute_ndt_inverse_covariance()`, `compute_ndt_params()`) | 251 |
| `include/ndt/impl/integrated_ndt_factor_impl.hpp` | NDT Factor 템플릿 구현 (`update_correspondences()`, `evaluate()`) | 321 |
| `include/ndt/integrated_light_ndt_factor.hpp` | LightNDT Factor 헤더 | 102 |
| `include/ndt/impl/integrated_light_ndt_factor_impl.hpp` | LightNDT Factor 템플릿 구현 | 260 |
| `src/ndt/integrated_ndt_factor.cpp` | NDT 명시적 템플릿 인스턴스화 | 11 |
| `src/ndt/integrated_light_ndt_factor.cpp` | LightNDT 명시적 템플릿 인스턴스화 | 11 |

### 7.2 NdtCorrespondence: 가평균/가분산 캐싱 구조체

NDT Factor는 `GaussianVoxel`을 직접 참조하지 않고, 대응점 탐색 시점에 필요한 값만 추출하여
`NdtCorrespondence` 구조체에 캐싱한다.

```cpp
// integrated_ndt_factor.hpp, 줄 29-38
struct NdtCorrespondence {
  Eigen::Vector4d mean;      // ← voxel.mean (가평균)
  Eigen::Matrix4d inv_cov;   // ← compute_ndt_inverse_covariance(voxel.cov) (가분산의 역행렬)
  double one_over_Z;         // ← 1/sqrt((2π)^3 * |Σ₃ₓ₃|) (NDT only)
  bool valid;
};
```

**가평균 → `mean`**: `GaussianVoxel::mean` (복셀 내 포인트 위치의 산술 평균)이 그대로 복사된다.

**가분산 → `inv_cov`**: `GaussianVoxel::cov` (복셀 내 포인트별 공분산의 평균)에
`compute_ndt_inverse_covariance()`를 적용한 결과이다. 즉 가분산 $\bar{\Sigma}$의
고유값 클램핑 정규화 + 역행렬이 한 번에 적용된 $\bar{\Sigma}_{\text{reg}}^{-1}$이다.

**`one_over_Z`**: NDT Factor 전용. 가분산의 3×3 블록 행렬식에서 계산한 정규화 상수이다.
LightNDT는 이 값을 사용하지 않는다.

### 7.3 update_correspondences(): 대응점 탐색 (NDT/LightNDT 공통)

두 Factor의 `update_correspondences()`는 구조가 동일하다.
차이점은 NDT가 `one_over_Z`를 추가로 캐싱하는 것뿐이다.

**소스 위치**:
- NDT: `integrated_ndt_factor_impl.hpp`, 줄 94-203
- LightNDT: `integrated_light_ndt_factor_impl.hpp`, 줄 90-195

```
[입력]                           [출력]
소스 포인트 클라우드              correspondences[i] 배열
현재 변환 delta (T)              (mean, inv_cov, valid)
타겟 복셀맵 (GaussianVoxelMapCPU)
```

**흐름**:

```
1. inv_cov_cache 구축 (최초 1회, Section 6.4 참조)
   → 모든 타겟 복셀의 cov에 대해 compute_ndt_inverse_covariance() 호출
   → inv_cov_cache[voxel_index] 에 저장

2. 각 소스 포인트 i에 대해 (병렬):
   a. 변환: pt = delta * source.points[i]
   b. 복셀 좌표: coord = voxel_coord(pt)
   c. 이웃 탐색 (DIRECT1/7/27):
      for each neighbor_offset:
        neighbor_coord = coord + offset
        voxel_id = lookup_voxel_index(neighbor_coord)
        if not found: skip

        ┌─────────────────────────────────────────────────┐
        │  가평균 사용: diff = pt - voxel.mean             │
        │  가분산 사용: mahal = diff^T * inv_cov * diff    │
        └─────────────────────────────────────────────────┘

        if mahal < min_mahalanobis:
          best_voxel = this voxel
          best_inv_cov = inv_cov_cache[voxel_id]

   d. 캐싱:
      correspondences[i].mean    = best_voxel->mean     ← 가평균
      correspondences[i].inv_cov = best_inv_cov          ← 가분산^{-1}
      correspondences[i].one_over_Z = ...                ← NDT only
      correspondences[i].valid   = true
```

**코드 핵심부** (NDT, 줄 149-182):

```cpp
Eigen::Vector4d pt = delta * frame::point(*source, i);
Eigen::Vector3i coord = target_voxels->voxel_coord(pt);

// 이웃 복셀 중 마할라노비스 거리가 최소인 복셀 선택
for (const auto& offset : neighbor_offsets) {
  Eigen::Vector3i neighbor_coord = coord + offset;
  const auto voxel_id = target_voxels->lookup_voxel_index(neighbor_coord);
  if (voxel_id < 0) continue;

  const auto& voxel = target_voxels->lookup_voxel(voxel_id);
  const Eigen::Matrix4d& inv_cov = inv_cov_cache[voxel_id];

  Eigen::Vector4d diff = pt - voxel.mean;                    // 가평균과의 차이
  double mahalanobis_dist = diff.transpose() * inv_cov * diff; // 가분산 기반 거리

  if (mahalanobis_dist < min_mahalanobis) {
    best_voxel = &voxel;
    best_inv_cov = inv_cov;
  }
}

// 최선 복셀의 가평균/가분산 캐싱
correspondences[i].mean = best_voxel->mean;
correspondences[i].inv_cov = best_inv_cov;
// NDT only: 정규화 상수 계산 (가분산의 3×3 행렬식 사용)
const Eigen::Matrix3d cov3 = best_voxel->cov.topLeftCorner<3, 3>();
const double det3 = std::max(cov3.determinant(), 1e-12);
correspondences[i].one_over_Z = 1.0 / std::sqrt(std::pow(2.0 * M_PI, 3) * det3);
```

> **LightNDT 차이점**: `one_over_Z` 계산이 없다. 그 외 대응점 탐색 로직은 동일하다.

### 7.4 NDT Factor: evaluate() — Gaussian NDT 비용 함수

**소스 위치**: `integrated_ndt_factor_impl.hpp`, 줄 217-319

NDT Factor는 Magnusson 2009의 Gaussian NDT 비용 함수를 사용한다.
가평균과 가분산(의 역행렬)이 비용, Jacobian, Hessian 모두에 관여한다.

**비용 함수** (줄 243-282):

```cpp
// d1, d2: 복셀별 NDT 가우시안 파라미터 (one_over_Z → 가분산 행렬식에서 유도)
double d1, d2;
compute_ndt_params(resolution, outlier_ratio, corr.one_over_Z, d1, d2);

const auto& mean_A = frame::point(*source, i);   // 소스 포인트
const auto& mean_B = corr.mean;                  // ← 가평균
const auto& inv_cov_B = corr.inv_cov;            // ← 가분산^{-1}

Eigen::Vector4d transed_mean_A = delta * mean_A;
Eigen::Vector4d residual = mean_B - transed_mean_A;   // 가평균 - 변환된 소스

double mahalanobis_dist = residual.transpose() * inv_cov_B * residual;
double e_term = std::exp(-d2 / 2.0 * mahalanobis_dist);
double cost = -d1 * (1 - e_term);   // 비음수 NDT 비용
```

수식으로 표현하면:

$$
r_i = \bar{\mu}_B - T \cdot p_A, \quad m_i = r_i^T \bar{\Sigma}_{\text{reg},B}^{-1} r_i
$$

$$
E_i = -d_1 \left(1 - \exp\!\left(-\frac{d_2}{2} \, m_i\right)\right)
$$

여기서 $\bar{\mu}_B$가 **가평균**, $\bar{\Sigma}_{\text{reg},B}^{-1}$이 **가분산의 정규화된 역행렬**이다.

**d1/d2 파라미터와 가분산의 관계**:

NDT Factor에서 `d1`, `d2`는 **복셀별로** 달라진다. `compute_ndt_params()`의 `one_over_Z` 인자가
가분산의 3×3 행렬식에서 계산되기 때문이다:

$$
\frac{1}{Z_i} = \frac{1}{\sqrt{(2\pi)^3 \cdot |\bar{\Sigma}_{3 \times 3}|}}
$$

즉, 가분산의 크기(행렬식)가 d1/d2를 통해 비용 함수의 **가중치 스케일**에도 영향을 준다.

**Jacobian** (줄 289-295):

```cpp
// target Jacobian: residual의 target 포즈에 대한 미분 (4×6)
J_target.block<3,3>(0,0) = -SO3::Hat(transed_mean_A.head<3>());  // rotation
J_target.block<3,3>(0,3) = I₃;                                    // translation

// source Jacobian: residual의 source 포즈에 대한 미분 (4×6)
J_source.block<3,3>(0,0) = R * SO3::Hat(mean_A.head<3>());
J_source.block<3,3>(0,3) = -R;
```

Jacobian은 가평균이 아닌 **포인트 위치**와 **변환 행렬**로 구성된다.
가분산은 Jacobian 자체에는 관여하지 않지만, Hessian에서 가중치 행렬로 곱해진다.

**Hessian (Gauss-Newton H1 근사)** (줄 297-306):

```cpp
double weight = -d1 * d2 * e_term;   // 양수 (PSD 보장)

H_target        += weight * J_target^T * inv_cov_B * J_target;
H_source        += weight * J_source^T * inv_cov_B * J_source;
H_target_source += weight * J_target^T * inv_cov_B * J_source;

b_target += weight * J_target^T * inv_cov_B * residual;
b_source += weight * J_source^T * inv_cov_B * residual;
```

수식:

$$
H \mathrel{+}= w_i \cdot J_i^T \, \bar{\Sigma}_{\text{reg},B}^{-1} \, J_i, \quad
b \mathrel{+}= w_i \cdot J_i^T \, \bar{\Sigma}_{\text{reg},B}^{-1} \, r_i
$$

$$
w_i = -d_1 \cdot d_2 \cdot \exp\!\left(-\frac{d_2}{2} \, m_i\right)
$$

가분산의 역행렬 $\bar{\Sigma}_{\text{reg}}^{-1}$이 **정보 행렬(information matrix)** 역할을 한다.
이 행렬이 Hessian에 곱해지므로, 가분산이 작은 방향(확실한 방향)은 Hessian 기여가 크고,
가분산이 큰 방향(불확실한 방향)은 기여가 작다. 이것이 NDT 정합의 핵심 원리다.

### 7.5 LightNDT Factor: evaluate() — 순수 마할라노비스 비용 함수

**소스 위치**: `integrated_light_ndt_factor_impl.hpp`, 줄 197-258

LightNDT는 Magnusson의 Gaussian 파라미터(d1, d2)와 지수 함수를 **제거**하고,
순수 마할라노비스 거리를 비용으로 사용한다.

**비용 함수** (줄 221-228):

```cpp
const auto& mean_B = corr.mean;        // ← 가평균
const auto& inv_cov_B = corr.inv_cov;  // ← 가분산^{-1}

Eigen::Vector4d transed_mean_A = delta * mean_A;
Eigen::Vector4d residual = mean_B - transed_mean_A;

double cost = residual.transpose() * inv_cov_B * residual;  // 순수 마할라노비스
```

수식:

$$
E_i^{\text{Light}} = r_i^T \, \bar{\Sigma}_{\text{reg},B}^{-1} \, r_i
$$

NDT의 $-d_1(1 - e^{-d_2/2 \cdot m})$와 비교하면, **지수 포장 없이 마할라노비스 거리 자체**가 비용이다.

**Hessian (줄 241-248)**:

```cpp
// weight = 1 (암묵적, 별도 weight 변수 없음)
Eigen::Matrix<double, 6, 4> J_target_weighted = J_target.transpose() * inv_cov_B;
Eigen::Matrix<double, 6, 4> J_source_weighted = J_source.transpose() * inv_cov_B;

H_target        += J_target_weighted * J_target;
H_source        += J_source_weighted * J_source;
H_target_source += J_target_weighted * J_source;
b_target        += J_target_weighted * residual;
b_source        += J_source_weighted * residual;
```

수식:

$$
H \mathrel{+}= J_i^T \, \bar{\Sigma}_{\text{reg},B}^{-1} \, J_i, \quad
b \mathrel{+}= J_i^T \, \bar{\Sigma}_{\text{reg},B}^{-1} \, r_i
$$

NDT와 비교하면 $w_i = 1$인 가중 최소제곱 문제와 동일하다.
`J_weighted = J^T * inv_cov`를 먼저 계산하여 행렬 곱셈 횟수를 줄이는 최적화가 적용되어 있다.

### 7.6 NDT vs LightNDT: 동일한 가평균/가분산, 다른 비용 함수

두 Factor는 **동일한 가평균/가분산**을 입력으로 받지만, 비용 함수 구조가 다르다.

| 항목 | NDT Factor | LightNDT Factor |
|------|-----------|----------------|
| **소스** | `integrated_ndt_factor_impl.hpp` | `integrated_light_ndt_factor_impl.hpp` |
| **가평균 사용** | `corr.mean` (잔차 계산) | `corr.mean` (잔차 계산) — 동일 |
| **가분산 사용** | `corr.inv_cov` (Mahalanobis + Hessian) | `corr.inv_cov` (Mahalanobis + Hessian) — 동일 |
| **비용 함수** | $-d_1(1 - e^{-d_2 m/2})$ | $m = r^T \Sigma^{-1} r$ |
| **d1/d2 파라미터** | 복셀별 계산 (`one_over_Z` → 가분산 행렬식) | 없음 |
| **weight** | $w = -d_1 d_2 e^{-d_2 m/2}$ (거리에 따라 감쇠) | $w = 1$ (상수) |
| **Outlier 처리** | 지수 함수가 먼 포인트의 기여를 자동 감쇠 | 없음 (먼 포인트도 동일 가중치) |
| **Hessian** | $w \cdot J^T \Sigma^{-1} J$ | $J^T \Sigma^{-1} J$ |
| **PSD 보장** | $w > 0$ 이므로 H1만으로 PSD 보장 | 항상 PSD ($J^T A J$ 형태) |
| **resolution 필요** | 예 (d1/d2 계산에 사용) | 아니오 |
| **outlier_ratio 필요** | 예 (d1/d2 계산에 사용) | 아니오 |
| **수렴 특성** | 먼 거리에서 gradient 작음 → 초기값 민감 | 먼 거리에서도 gradient 큼 → 빠른 수렴 |

**핵심 차이**: NDT의 지수 함수 $e^{-d_2 m/2}$는 마할라노비스 거리가 클수록 weight를 0에 가깝게
만들어 outlier의 영향을 줄인다. LightNDT는 이 보호 장치가 없지만, 대신 gradient가 거리에
비례하므로 큰 오차를 빠르게 줄일 수 있다.

**가분산이 비용 함수에 미치는 영향** (두 Factor 공통):

가분산 $\bar{\Sigma}$의 고유벡터가 정보 행렬 $\bar{\Sigma}_{\text{reg}}^{-1}$의 방향을 결정한다.
예를 들어 평면을 구성하는 복셀의 가분산이 법선 방향으로 작고 접선 방향으로 크다면:
- 법선 방향: $\sigma^2_n \approx 10^{-3}$ → $1/\sigma^2_n \approx 10^3$ → **강한 구속**
- 접선 방향: $\sigma^2_t \approx 1.0$ → $1/\sigma^2_t = 1.0$ → **약한 구속**

이는 "평면 위에서 미끄러지는 것은 허용하되, 평면을 관통하는 이동은 크게 벌점을 주는" 효과를 낸다.
이 방향별 구속력이 EIG 정규화(Section 3.3)에서 고유값을 `(1e-3, 1.0, 1.0)`으로 강제한 결과와
정확히 일치한다.

---

## 8. GaussianVoxel(NDT) vs IncrementalCovarianceVoxelMap(GICP) 비교

### 8.1 두 방식의 공분산 계산 방식 차이

| 항목 | GaussianVoxel (NDT) | IncrementalCovarianceVoxelMap (GICP) |
|------|---------------------|--------------------------------------|
| 소스 파일 | `gaussian_voxelmap_cpu.cpp` | `incremental_covariance_voxelmap.cpp` |
| 공분산 정의 | 포인트별 KNN 공분산의 평균 | 복셀 내 포인트들의 공간적 분포 공분산 |
| 수식 | $\bar{\Sigma} = \frac{1}{N}\sum_i \Sigma_i$ | $\Sigma = \frac{1}{N-1}\sum_i (p_i - \mu)(p_i - \mu)^T$ |
| 추정 방식 | 편향 평균 (가분산) | 비편향 추정량 (표본 공분산) |
| 의미 | 이웃 구조의 평균적 불확실성 | 복셀 내 점들의 공간 분포 |
| Factor | NDT Factor | GICP Factor |

### 8.2 수식 비교

**NDT 가분산** (복셀 내 포인트별 공분산의 평균):

$$
\bar{\Sigma}_{\text{NDT}} = \frac{1}{N} \sum_{i=1}^{N} \Sigma_i^{\text{KNN}}
$$

**GICP 공분산** (복셀 내 포인트들의 공간 분포):

$$
\Sigma_{\text{GICP}} = \frac{1}{N-1} \sum_{i=1}^{N} (p_i - \mu)(p_i - \mu)^T
$$

두 방식은 목적이 다르다. NDT는 포인트 주변 이웃의 기하학적 구조를 반영한 불확실성을 쌓고,
GICP는 복셀이라는 공간 단위 안에서 포인트들이 얼마나 퍼져 있는지를 직접 측정한다.

---

## 9. 핵심 인사이트: 전통적 NDT vs VGICP-style 가평균

### 9.1 전통적 NDT (Magnusson 2009)

전통적 NDT는 복셀 안에 들어온 포인트들의 **공간 분포**를 가우시안으로 모델링한다.

$$
\mu_{\text{trad}} = \frac{1}{N} \sum_{i=1}^{N} p_i
$$

$$
\Sigma_{\text{trad}} = \frac{1}{N} \sum_{i=1}^{N} (p_i - \mu_{\text{trad}})(p_i - \mu_{\text{trad}})^T
$$

이 공분산은 복셀 안 포인트들이 어디에 모여 있는지, 어느 방향으로 늘어서 있는지를 직접 담는다.
LiDAR 스캔에서 벽면이나 평면 구조는 이 방식으로 편평한 타원형 가우시안이 된다.

### 9.2 이 코드베이스의 방식: VGICP-style

이 코드베이스는 전통적 NDT가 아니다. 단계를 나눠 보면 명확해진다.

- **Stage 1**: KNN으로 각 포인트의 **로컬 이웃 공분산** $\Sigma_i$ 를 계산한다.
  이 $\Sigma_i$ 는 포인트 $p_i$ 주변 k개 이웃의 분포를 나타낸다.
- **Stage 2**: 복셀에 들어온 포인트들의 $\Sigma_i$를 산술 평균한다.
  이 평균값이 "가분산"이다.

결과적으로 `GaussianVoxel::cov`는 "이 복셀을 구성하는 포인트들이 각각 속한 이웃 구조의 평균 불확실성"을
나타내는 값이지, 복셀 내 포인트들의 공간 산포를 직접 측정한 값이 아니다.

이 접근은 **VGICP(Voxelized GICP)** 에서 유래한 방식으로,
GICP의 포인트별 공분산 아이디어를 복셀 단위로 집계하는 형태다. NDT의 비용 함수 틀을 유지하면서
VGICP의 공분산 집계 방식을 결합했기 때문에, 가평균/가분산이라는 이름이 붙는다.

### 9.3 "가(假)"의 의미

"가평균", "가분산"에서 "가(假)"는 진짜가 아닌, 그럴듯한 대리값이라는 뜻이다.

- **가평균**: 복셀 내 포인트 위치의 산술 평균. 이 자체는 전통적 NDT와 동일하다.
- **가분산**: 진짜 공간 공분산이 아니라 포인트별 공분산을 평균한 대리값.
  이 값이 전통적 NDT와 결정적으로 다른 지점이다.

---

## 10. 요약 테이블

### 10.1 각 Stage 수식 요약

| Stage | 파일 | 핵심 수식 | 비고 |
|-------|------|-----------|------|
| 1 (포인트별 공분산) | `covariance_estimation.cpp:33-43` | $\Sigma_i = \frac{1}{k}S_\Sigma - \mu_i\mu_i^T$ | 편향 추정, KNN |
| 2 (CPU 집계) | `gaussian_voxelmap_cpu.cpp:23-47` | $\bar{\mu} = \frac{1}{N}\sum p_i$, $\bar{\Sigma} = \frac{1}{N}\sum \Sigma_i$ | un-finalize 트릭 포함 |
| 3 (GPU 집계) | `gaussian_voxelmap_gpu.cu:92-176` | 동일 수식, float 타입, atomicAdd | CPU와 수학적으로 동일 |
| 4 (역공분산) | `integrated_ndt_factor.hpp:45-55` | $\bar{\Sigma}_{\text{reg}}^{-1}$ via eigenvalue clamp | $\varepsilon = 10^{-3}$ |
| 5 (NDT Factor 활용) | `integrated_ndt_factor_impl.hpp` | $E = -d_1(1 - e^{-d_2 m/2})$, $H += w \cdot J^T \Sigma^{-1} J$ | Section 7.4 참조 |
| 5 (LightNDT Factor 활용) | `integrated_light_ndt_factor_impl.hpp` | $E = r^T \Sigma^{-1} r$, $H += J^T \Sigma^{-1} J$ | Section 7.5 참조 |

### 10.2 NDT vs GICP 공분산 철학 비교

| 관점 | NDT (이 코드) | GICP |
|------|--------------|------|
| 공분산 출처 | 포인트별 KNN 결과를 복셀에서 평균 | 복셀 내 포인트의 공간 산포 직접 계산 |
| 자유도 | 포인트 수 k (KNN 이웃) | 복셀 내 포인트 수 N |
| 편향 여부 | 편향 (1/k) | 비편향 (1/(N-1)) |
| 이름 | 가분산 (pseudo-covariance) | 표본 공분산 (sample covariance) |
| 의미 | 이웃 구조의 평균 불확실성 | 복셀 내 공간 분포 |

### 10.3 전통적 NDT와의 결정적 차이

| 항목 | 전통적 NDT | 이 코드베이스 |
|------|-----------|--------------|
| $\Sigma_{\text{voxel}}$ 계산 | 복셀 내 포인트 공간 분포 | 포인트별 KNN 공분산의 평균 |
| 공분산이 반영하는 것 | 복셀 내 점들의 배치 형태 | 각 점 주변 이웃의 기하 구조 |
| 분류 | 전통적 NDT | VGICP-style NDT |

---

최종 수정일: 2026-03-31
