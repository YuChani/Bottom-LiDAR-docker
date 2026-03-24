# Voxel 구조 및 NDT/LightNDT 수학 해설

이 문서는 현재 코드 기준으로 voxel 관련 구조가 어떻게 이루어져 있는지, 그리고 그 voxel 정보가 NDT / LightNDT factor 안에서 어떤 수학적 의미로 쓰이는지를 자세히 설명한다.

함께 생성한 구조도 파일은 아래 경로에 있다.

- `artifacts/uml/voxel_structure.prisma`

이 `.prisma` 파일은 VSCode의 **Prisma Generate UML** 뷰어에서 열 수 있도록 만든 구조도용 스키마다.

---

## 1. 큰 흐름: 이 코드에서 voxel은 어떻게 만들어지고 쓰이는가

현재 구현의 전체 흐름은 아래처럼 이해하면 된다.

```text
PointCloud / PointCloudCPU
  -> GaussianVoxelMapCPU::insert()
  -> IncrementalVoxelMap<GaussianVoxel> 내부에 voxel 생성
  -> 각 voxel이 mean / cov / intensity를 가짐
  -> NDT / LightNDT / VGICP factor가 GaussianVoxelMapCPU를 조회
  -> source point마다 대응 voxel을 고름
  -> 그 결과를 비용함수(cost)와 Hessian/gradient 계산에 사용
```

즉 voxel은 단순한 격자 셀이 아니라,

- **좌표계상 위치(mean)**
- **분산 구조(covariance)**
- **해당 voxel에 들어온 점 개수(num_points)**
- **부가 intensity 정보**

를 담고 있는 **확률적/통계적 셀**이다.

---

## 2. PointCloud 계층

### 2.1 `PointCloud`

정의 파일:

- `thirdparty/gtsam_points/include/gtsam_points/types/point_cloud.hpp`

`PointCloud`는 가장 기본적인 point cloud 타입이다. 이 타입은 스스로 데이터를 소유하는 구조가 아니라, 아래 같은 raw pointer를 들고 있는 **비소유(non-owning) 인터페이스**에 가깝다.

- `points`
- `normals`
- `covs`
- `intensities`
- GPU용 pointer들

즉 이 타입은 “점군이라는 데이터 뭉치가 최소한 어떤 필드들을 가질 수 있는가”를 정의하는 기반 타입이다.

---

### 2.2 `PointCloudCPU : PointCloud`

정의 파일:

- `thirdparty/gtsam_points/include/gtsam_points/types/point_cloud_cpu.hpp`

`PointCloudCPU`는 `PointCloud`를 상속하고, 실제 저장소를 가진다.

대표 저장 필드:

- `times_storage`
- `points_storage`
- `normals_storage`
- `covs_storage`
- `intensities_storage`

즉 `PointCloud`가 포인터 인터페이스라면, `PointCloudCPU`는 그 포인터들이 실제로 가리키는 메모리를 가진 구현체다.

이 코드에서 voxelmap에 `insert()`되는 frame은 대부분 결국 이 계층에서 온다고 보면 된다.

---

## 3. Voxel 계층

### 3.1 `GaussianVoxel`

정의 파일:

- `thirdparty/gtsam_points/include/gtsam_points/types/gaussian_voxelmap_cpu.hpp`

구현 파일:

- `thirdparty/gtsam_points/src/gtsam_points/types/gaussian_voxelmap_cpu.cpp`

이 struct가 이 코드에서 가장 핵심적인 voxel 단위다. 필드는 아래와 같다.

- `bool finalized`
- `size_t num_points`
- `Eigen::Vector4d mean`
- `Eigen::Matrix4d cov`
- `double intensity`

여기서 중요한 점은 **voxel 하나가 곧 Gaussian 하나**라는 것이다.

즉 이 voxel은 단지 “여기 셀이 찼다”는 정보만 갖지 않고,

- 평균 위치 `mean`
- 공분산 `cov`

를 가지므로, **local distribution(지역 정규분포)**를 나타내는 셀이다.

---

### 3.2 `GaussianVoxel::add()`와 `finalize()`

구현 위치:

- `gaussian_voxelmap_cpu.cpp:23-47`

`add()`는 voxel에 들어온 점을 누적한다.

실제로 하는 일은:

```cpp
num_points++;
mean += points.points[i];
cov += points.covs[i];
```

그리고 intensity가 있으면 최대 intensity를 유지한다.

이후 `finalize()`에서

```cpp
mean /= num_points;
cov /= num_points;
```

를 수행한다.

즉 현재 구현은 voxel 안에 들어온 점들의 합을 쌓아두었다가, 마지막에 평균화해서

- 평균 위치
- 평균 공분산

를 만들어낸다.

이건 “point set -> local Gaussian” 변환으로 보면 된다.

---

### 3.3 `VoxelInfo`

정의 파일:

- `thirdparty/gtsam_points/include/gtsam_points/ann/incremental_voxelmap.hpp`

`VoxelInfo`는 voxel의 메타정보만 가진다.

- `lru`
- `coord`

즉 Gaussian 자체가 아니라,

- 이 voxel이 grid 상 어디 있는지
- LRU 기반으로 얼마나 최근에 쓰였는지

같은 관리 정보를 담는다.

---

### 3.4 `IncrementalVoxelMap<VoxelContents>`

정의 파일:

- `thirdparty/gtsam_points/include/gtsam_points/ann/incremental_voxelmap.hpp`

이 템플릿은 voxel container + lookup + neighbor search 구조다.

핵심 필드는:

- `double inv_leaf_size`
- `std::vector<Eigen::Vector3i> offsets`
- `size_t lru_horizon`
- `size_t lru_clear_cycle`
- `size_t lru_counter`
- `std::vector<std::shared_ptr<std::pair<VoxelInfo, VoxelContents>>> flat_voxels`
- `std::unordered_map<Eigen::Vector3i, size_t, XORVector3iHash> voxels`

여기서 중요한 것은 `flat_voxels` 구조다.

즉 실제 voxelmap은 개념적으로 아래처럼 저장된다.

```text
flat_voxels[k] = (VoxelInfo, GaussianVoxel)
```

즉 각 voxel은

- grid 좌표/관리 정보 (`VoxelInfo`)
- 통계 셀 내용 (`GaussianVoxel`)

의 pair로 저장된다.

그리고 `voxels` 해시맵은

```text
coord -> index in flat_voxels
```

역할을 한다.

---

### 3.5 `GaussianVoxelMapCPU : GaussianVoxelMap, IncrementalVoxelMap<GaussianVoxel>`

정의 파일:

- `thirdparty/gtsam_points/include/gtsam_points/types/gaussian_voxelmap_cpu.hpp`

구현 파일:

- `thirdparty/gtsam_points/src/gtsam_points/types/gaussian_voxelmap_cpu.cpp`

이 타입이 실제 CPU 기반 Gaussian voxel map 구현이다.

즉 구조적으로 보면:

- `GaussianVoxelMap`의 추상 인터페이스를 만족하고
- 실제 내부 저장/검색은 `IncrementalVoxelMap<GaussianVoxel>`를 사용한다.

핵심 메서드:

- `voxel_resolution()`
- `voxel_coord()`
- `lookup_voxel_index()`
- `lookup_voxel()`
- `insert()`

수학적으로 가장 중요한 함수는 `voxel_coord()`이다.

```cpp
return fast_floor(x * inv_leaf_size).head<3>();
```

즉 point `x`를 voxel size로 나눈 뒤 floor를 취해서 integer voxel coordinate를 만든다.

만약 voxel resolution이 `r`이면,

```math
coord = floor(x / r)
```

와 같은 역할이다.

---

## 4. 저장/로드용 compact 구조

### `GaussianVoxelData`

정의 파일:

- `thirdparty/gtsam_points/include/gtsam_points/types/gaussian_voxel_data.hpp`

이 struct는 serialization 전용 compact 표현이다.

여기서는 `GaussianVoxel`의 정보를 더 compact한 형식으로 저장한다.

- `coord`
- `num_points`
- `mean` (3D)
- `cov` (대칭행렬 6개 성분)
- `intensity`

그리고 `uncompact()`는 다시

```cpp
shared_ptr<pair<VoxelInfo, GaussianVoxel>>
```

형태로 복원한다.

즉 저장 파일 관점에서는 `GaussianVoxelData`, 메모리 관점에서는 `(VoxelInfo, GaussianVoxel)`가 대응된다.

---

## 5. NDT에서 voxel이 어떻게 사용되는가

### 5.1 `NdtCorrespondence`

정의 파일:

- `thirdparty/gtsam_points/include/gtsam_points/factors/integrated_ndt_factor.hpp`

NDT는 voxel pointer를 직접 오래 들고 다니지 않고, source point마다 대응 voxel에서 필요한 값만 캐시한다.

필드:

- `mean`
- `inv_cov`
- `one_over_Z`
- `valid`

즉 source point 하나당 “내가 현재 대응시키기로 한 voxel Gaussian 정보”를 복사해서 들고 있는 구조다.

---

### 5.2 correspondence search

구현 파일:

- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:145-183`

각 source point에 대해 하는 일은 다음과 같다.

1. source point `p_i`를 현재 pose `delta`로 변환한다.
2. `target_voxels->voxel_coord(pt)`로 현재 voxel 좌표를 구한다.
3. `DIRECT1`, `DIRECT7`, `DIRECT27`에 따라 neighbor offsets를 순회한다.
4. 각 candidate voxel에 대해 Mahalanobis distance를 계산한다.
5. 가장 작은 voxel을 best match로 고른다.
6. 그 voxel의 `mean`, `inv_cov`, `one_over_Z`를 `NdtCorrespondence`에 저장한다.

즉 correspondence는 “최근접 voxel center”가 아니라,

```math
m = (p - \mu)^T \Sigma^{-1} (p - \mu)
```

가 가장 작은 voxel을 고르는 방식이다.

이게 NDT의 핵심이다. point-to-point nearest neighbor가 아니라, **point-to-distribution match**를 한다.

---

### 5.3 `inv_cov`의 의미

NDT correspondence cache에는 voxel의 regularized inverse covariance가 들어간다.

현재 구현:

```cpp
inv_cov_cache[v] = compute_ndt_inverse_covariance(voxel.cov, regularization_epsilon);
```

이 inverse covariance는 Mahalanobis distance 계산과 Hessian weighting에 들어간다.

즉 voxel covariance가 클수록 그 방향 차이에 덜 민감하고,
작을수록 더 민감하다.

이는 직관적으로,

- 넓은 분포 방향: 덜 중요
- 날카로운 분포 방향: 더 중요

라는 weighting이다.

---

### 5.4 `one_over_Z`, `d1`, `d2`

`one_over_Z`는 현재 코드에서

```cpp
1 / sqrt((2π)^3 det(Σ_3x3))
```

로 계산된다.

이 값은 Gaussian normalization factor의 reciprocal이다.

이후 `compute_ndt_params()`에서

```cpp
c1 = (1 - o) * one_over_Z
c2 = o / V
```

를 기반으로 `d1`, `d2`를 계산한다.

여기서

- `o` = outlier ratio
- `V = resolution^3`

이다.

이 `d1`, `d2`는 결국 NDT score의 shape를 정한다.

---

## 6. 현재 NDT의 수학적 형태

현재 구현의 core는 아래다.

```cpp
mahalanobis_dist = residual.transpose() * inv_cov_B * residual;
exponent = -d2 * mahalanobis_dist / 2.0;
e_term = exp(exponent);
cost = -d1 * (1 - e_term);
weight = -d1 * d2 * e_term;
```

즉 source point 하나에 대한 residual은

```math
r_i = \, \mu_i - (R p_i + t)
```

Mahalanobis distance는

```math
m_i = r_i^T \Sigma_i^{-1} r_i
```

그리고 NDT score 기반 항은

```math
e_i = \exp\left(-\frac{d_2}{2} m_i\right)
```

현재 optimizer에 들어가는 cost는

```math
cost_i = -d_1 (1 - e_i)
```

이다.

중요한 점은 이 cost가 원래 score 최대화를 **generic LM이 다루는 양수 error 최소화**로 재표현한 형태라는 것이다.

즉 정합이 잘 맞으면 `e_i -> 1`, 따라서 `cost_i -> 0`이 된다.

---

## 7. LightNDT는 무엇이 다른가

구현 파일:

- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_light_ndt_factor_impl.hpp`

LightNDT는 correspondence search는 거의 같은데, 최종 cost가 다르다.

현재 구현은

```cpp
cost = residual.transpose() * inv_cov_B * residual;
```

즉 그냥 quadratic Mahalanobis LSQ다.

수학적으로는

```math
cost_i = r_i^T \Sigma_i^{-1} r_i
```

이고,

NDT처럼

- `one_over_Z`
- `d1`, `d2`
- `exp()`

가 들어가지 않는다.

즉 요약하면:

- **NDT**: voxel Gaussian + exp-weighted score
- **LightNDT**: voxel Gaussian + simple quadratic LSQ

그래서 LightNDT는 generic LM에서 iteration이 더 적게 나오는 경향이 있다.

---

## 8. VGICP와의 차이

VGICP는 NDT/LightNDT와 비슷하게 `GaussianVoxelMapCPU`를 target으로 쓰지만, correspondence cache가 다르다.

NDT/LightNDT:

- `NdtCorrespondence`를 저장
- 즉 voxel에서 필요한 값을 복사해둠

VGICP:

- `const GaussianVoxel*`를 직접 저장

그리고 VGICP는 target voxel covariance와 source point covariance를 합쳐서 비용을 만든다.

즉 voxel을 “target uncertainty”로 쓰는 방식은 비슷하지만, 수학적 backend는 다르다.

---

## 9. 현재 voxel structure를 보는 가장 정확한 관점

이 코드에서 voxel은 단순히 occupancy cell이 아니다.

더 정확히 말하면,

> **GaussianVoxelMapCPU는 3차원 공간을 격자로 나누고, 각 칸에 GaussianVoxel이라는 local distribution 추정치를 저장하는 구조**

다.

그리고 NDT/LightNDT/VGICP는 이 local distribution을 참조해서 source point와 target voxel 사이의 정합 비용을 만든다.

즉 핵심 역할 분담은 아래와 같다.

### 데이터 계층

- `PointCloud` / `PointCloudCPU`

### 통계 voxel 계층

- `GaussianVoxel`
- `VoxelInfo`
- `IncrementalVoxelMap<GaussianVoxel>`
- `GaussianVoxelMapCPU`

### factor 계층

- `IntegratedNDTFactor`
- `IntegratedLightNDTFactor`
- `IntegratedVGICPFactor`

### 수학적 해석

- NDT: score-based Gaussian matching
- LightNDT: voxel correspondence + LSQ backend
- VGICP: voxel covariance + point covariance fusion

---

## 10. 실제 코드에서 가장 중요한 연결점

실전에서 voxel 구조를 읽을 때 핵심 연결점은 아래 네 군데다.

1. `PointCloudCPU`가 실제 points/covs를 소유한다.
2. `GaussianVoxel::add/finalize`가 그 점군을 local Gaussian으로 바꾼다.
3. `GaussianVoxelMapCPU::voxel_coord / lookup_voxel_index / lookup_voxel`가 spatial query를 담당한다.
4. NDT/LightNDT factor가 그 voxel Gaussian을 대응점 cost로 바꾼다.

즉 한 줄로 요약하면,

> **현재 코드의 voxel 구조는 “점군을 local Gaussian 집합으로 바꾼 뒤, 그 Gaussian들을 정합 비용 함수의 기본 단위로 쓰는 구조”다.**

---

## 11. 같이 보면 좋은 파일

- 구조도 파일: `artifacts/uml/voxel_structure.prisma`
- PointCloud base: `thirdparty/gtsam_points/include/gtsam_points/types/point_cloud.hpp`
- PointCloudCPU: `thirdparty/gtsam_points/include/gtsam_points/types/point_cloud_cpu.hpp`
- Gaussian voxel/map: `thirdparty/gtsam_points/include/gtsam_points/types/gaussian_voxelmap.hpp`
- Gaussian voxel CPU 구현: `thirdparty/gtsam_points/include/gtsam_points/types/gaussian_voxelmap_cpu.hpp`
- Gaussian voxel CPU 구현체: `thirdparty/gtsam_points/src/gtsam_points/types/gaussian_voxelmap_cpu.cpp`
- Incremental voxel map: `thirdparty/gtsam_points/include/gtsam_points/ann/incremental_voxelmap.hpp`
- NDT factor: `thirdparty/gtsam_points/include/gtsam_points/factors/integrated_ndt_factor.hpp`
- NDT factor impl: `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp`
- LightNDT factor impl: `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_light_ndt_factor_impl.hpp`
- 테스트 예시: `thirdparty/gtsam_points/src/test/test_ndt.cpp`

---

## 12. 줄 단위(line-level) 매핑

이 절은 “어떤 설명이 실제로 코드 어느 줄에 대응하는가”를 바로 따라가도록 정리한 section이다.

### 12.1 PointCloud와 PointCloudCPU

- `PointCloud` 기본 필드
  - `point_cloud.hpp:103-118`
  - `num_points`, `points`, `normals`, `covs`, `intensities`, GPU 포인터들이 정의됨

- `PointCloudCPU`의 실제 storage
  - `point_cloud_cpu.hpp:100-107`
  - `times_storage`, `points_storage`, `normals_storage`, `covs_storage`, `intensities_storage`가 실제 데이터 저장소 역할

수학적 해석:

- NDT/VGICP/LightNDT는 point를 항상 homogeneous 4D 벡터로 다룬다.
- 점 하나는 실질적으로 `p_i = [x, y, z, 1]^T`
- 공분산은 `Σ_i ∈ R^{4x4}` 형태로 들어오지만, 실제 공간 의미는 상단 3x3이 중심이다.

### 12.2 GaussianVoxel 생성과 finalize

- `gaussian_voxelmap_cpu.cpp:23-37`
  - `GaussianVoxel::add()`
  - `mean += points.points[i]`
  - `cov += points.covs[i]`
  - intensity는 `max`로 누적

- `gaussian_voxelmap_cpu.cpp:39-47`
  - `GaussianVoxel::finalize()`
  - `mean /= num_points`
  - `cov /= num_points`

수학적 해석:

현재 구현은 voxel 안의 점 집합에 대해 “샘플 좌표 분산”을 새로 계산하는 구조가 아니다. 대신 입력 point가 이미 가지고 있는 공분산 `Σ_point`를 voxel 단위로 평균화한다.

즉 voxel covariance는 대략

```math
\Sigma_{voxel} = \frac{1}{N} \sum_{k=1}^{N} \Sigma_k
```

처럼 형성된다.

즉 이 코드의 GaussianVoxel은 “점 위치 집합으로부터 추정한 sample covariance voxel”이라기보다,

> **각 point가 갖는 local uncertainty를 voxel 단위로 평균화한 local Gaussian**

으로 보는 게 더 정확하다.

### 12.3 IncrementalVoxelMap 삽입/삭제/정리

- `incremental_voxelmap_impl.hpp:31-68`
  - `insert()`가 실제 삽입 경로
  - `coord = fast_floor(points.points[i] * inv_leaf_size).head<3>()`
  - voxel 없으면 생성
  - `voxel.add(...)`
  - 마지막에 모든 voxel `finalize()`

- `incremental_voxelmap_impl.hpp:49-61`
  - LRU 기반 voxel 삭제와 rehash

- `incremental_voxelmap_impl.hpp:95-140`
  - neighbor offsets 생성 (`1`, `7`, `19`, `27`)

- `incremental_voxelmap_impl.hpp:195-228`
  - `voxel_data()`가 voxel map을 다시 `PointCloudCPU`로 materialize

수학적 해석:

삽입 시 voxel 좌표는

```math
coord = \lfloor x / r \rfloor
```

이고, 여기서 `r`은 voxel resolution이다.

즉 공간을 정육면체 grid로 양자화한 뒤, 각 칸에 local Gaussian을 쌓는 구조다.

### 12.4 GaussianVoxelMapCPU 조회

- `gaussian_voxelmap_cpu.cpp:55-57`
  - `voxel_resolution() = leaf_size()`

- `gaussian_voxelmap_cpu.cpp:59-61`
  - `voxel_coord()` 구현

- `gaussian_voxelmap_cpu.cpp:63-69`
  - `lookup_voxel_index(coord)`

- `gaussian_voxelmap_cpu.cpp:71-73`
  - `lookup_voxel(voxel_id)`

즉 factor 쪽에서는 결국

```text
point -> voxel_coord -> lookup_voxel_index -> lookup_voxel
```

흐름으로 target voxel Gaussian을 가져온다.

### 12.5 NDT correspondence 구축

- `integrated_ndt_factor_impl.hpp:138-143`
  - 모든 voxel에 대해 `inv_cov_cache[v] = compute_ndt_inverse_covariance(...)`

- `integrated_ndt_factor_impl.hpp:149-180`
  - source point를 현재 pose로 변환
  - `voxel_coord(pt)`
  - neighbor voxel들을 순회
  - 각 voxel에 대해 Mahalanobis distance 계산
  - 최소 거리 voxel 선택
  - `mean`, `inv_cov`, `one_over_Z`, `valid` 저장

수학적 해석:

각 candidate voxel에 대해 계산하는 양은

```math
m_{iv} = (p'_i - \mu_v)^T \Sigma_v^{-1} (p'_i - \mu_v)
```

이다.

그리고 correspondence는

```math
v^* = \arg\min_v m_{iv}
```

를 만족하는 voxel이다.

즉 NDT correspondence는 nearest Euclidean center가 아니라, **가장 잘 맞는 Gaussian**을 고르는 것이다.

### 12.6 `one_over_Z`, `d1`, `d2`

- `integrated_ndt_factor_impl.hpp:178-180`
  - `det3 = det(cov.topLeftCorner<3,3>())`
  - `one_over_Z = 1 / sqrt((2π)^3 det3)`

- `integrated_ndt_factor.hpp:87-104`
  - `compute_ndt_params(resolution, outlier_ratio, one_over_Z, d1, d2)`

수학적 해석:

Gaussian normalization의 3D 부분은

```math
Z_i = \sqrt{(2\pi)^3 |\Sigma_i|}
```

이고, 현재 코드는 그 reciprocal인

```math
\frac{1}{Z_i}
```

를 저장한다.

그 다음

```math
c_1 = (1-o) \frac{1}{Z_i}, \quad c_2 = \frac{o}{V}, \quad V = resolution^3
```

를 바탕으로 Magnusson식 `d1`, `d2`를 만든다.

즉 `d1`, `d2`는 voxel Gaussian의 정규화와 outlier model이 함께 섞인 shape parameter다.

### 12.7 현재 NDT cost와 weight

- `integrated_ndt_factor_impl.hpp:263-282`
  - `mahalanobis_dist`
  - `exponent = -d2 * mahalanobis_dist / 2`
  - `e_term = exp(exponent)`
  - `cost = -d1 * (1 - e_term)`

- `integrated_ndt_factor_impl.hpp:297-306`
  - `weight = -d1 * d2 * e_term`
  - Hessian/gradient 누적

수학적 해석:

각 source point에 대해 현재 구현은

```math
r_i = \mu_i - (R p_i + t)
```

```math
m_i = r_i^T \Sigma_i^{-1} r_i
```

```math
e_i = \exp\left(-\frac{d_2}{2} m_i\right)
```

```math
cost_i = -d_1 (1 - e_i)
```

```math
weight_i = -d_1 d_2 e_i
```

로 동작한다.

이때 Hessian 근사는

```math
H_i \approx weight_i J_i^T \Sigma_i^{-1} J_i
```

형태다.

즉 이 구현은 “voxel Gaussian + score-based nonlinear weighting”을 쓰는 구조다.

### 12.8 LightNDT cost

- `integrated_light_ndt_factor_impl.hpp:145-174`
  - correspondence search는 거의 NDT와 동일

- `integrated_light_ndt_factor_impl.hpp:221-250`
  - `cost = residual^T inv_cov residual`
  - Hessian/gradient는 pure quadratic accumulation

수학적 해석:

LightNDT는 correspondence는 그대로 두고,

```math
cost_i = r_i^T \Sigma_i^{-1} r_i
```

만 푼다.

즉 `d1`, `d2`, `e_term`, `exp()`가 빠진다.

그래서 optimizer 입장에서는 LightNDT가 훨씬 “표준 weighted LSQ”에 가깝고, iteration이 적어지기 쉽다.

### 12.9 VGICP line-level 연결

- `integrated_vgicp_factor_impl.hpp:134-142`
  - transformed source point로 voxel lookup
  - correspondence는 `const GaussianVoxel*`

- `integrated_vgicp_factor_impl.hpp:150-160`
  - `RCR = target_voxel->cov + R * cov_source * R^T`
  - fused covariance inverse 계산

수학적 해석:

VGICP는 target voxel covariance와 source point covariance를 합쳐서

```math
\Sigma_{fused} = \Sigma_{target} + R \Sigma_{source} R^T
```

형태로 사용한다.

즉 NDT/LightNDT가 target voxel Gaussian만 직접 쓰는 데 비해, VGICP는 source uncertainty까지 결합한다.

## 13. 구조도를 더 정확히 읽는 방법

`artifacts/uml/voxel_structure.prisma`에서 가장 중요한 relation은 아래처럼 보면 된다.

- `PointCloudCPU -> PointCloud`
  - 데이터 소유 관계
- `GaussianVoxelMapCPU -> GaussianVoxelMap`
  - 추상 인터페이스 구현 관계
- `GaussianVoxelMapCPU -> IncrementalVoxelMapGaussianVoxel`
  - 실제 저장/검색 엔진 사용 관계
- `IncrementalVoxelMapGaussianVoxel -> GaussianVoxel / VoxelInfo`
  - voxel content와 메타정보 보유 관계
- `IntegratedNDTFactor / IntegratedLightNDTFactor / IntegratedVGICPFactor -> GaussianVoxelMapCPU`
  - voxelized target map 소비 관계

즉 구조도의 핵심은

> **voxel map은 target side local Gaussian 데이터베이스이고, factor들은 그 데이터베이스를 query해서 cost를 만든다**

로 읽으면 된다.
