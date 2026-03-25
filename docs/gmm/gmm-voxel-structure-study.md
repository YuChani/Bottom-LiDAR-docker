# gtsam_points 복셀 구조와 GMM 확장 — 학습 가이드

> **관련 문서**:
> - `docs/gmm/gmm-voxelmap-design.md`
> - `docs/gmm/mixture-ndt-factor-design.md`
> - `docs/gmm/armadillo-dependency.md`
>
> **날짜**: 2026-03-25

---

## 목차

1. [gtsam_points 복셀 생성 구조](#1-gtsampoints-복셀-생성-구조)
2. [GMM 복셀 생성 구조](#2-gmm-복셀-생성-구조)
3. [IntegratedMixtureLightNDTFactor vs IntegratedLightNDTFactor 구현 비교](#3-integratedmixturelightndtfactor-vs-integratedlightndtfactor-구현-비교)
4. [외부 의존성: Armadillo](#4-외부-의존성-armadillo)

---

## 1. gtsam_points 복셀 생성 구조

이 섹션은 `gtsam_points` 라이브러리가 포인트 클라우드를 어떻게 복셀 단위로 조직화하는지 설명한다. 핵심은 `IncrementalVoxelMap<T>` 템플릿으로, 이 하나의 제네릭 구조 위에 `GaussianVoxel`과 `GMMVoxel`이 모두 올라탄다.

---

### 1.1 IncrementalVoxelMap\<T\> 템플릿 아키텍처

**파일**: `thirdparty/gtsam_points/include/gtsam_points/ann/incremental_voxelmap.hpp`

`IncrementalVoxelMap<T>`는 복셀 내부에 저장할 콘텐츠 타입 `T`를 템플릿 파라미터로 받는다. 이 설계 덕분에 동일한 공간 해싱, LRU 관리, 이웃 탐색 로직을 `GaussianVoxel`과 `GMMVoxel` 양쪽에서 재사용할 수 있다.

```
IncrementalVoxelMap<T>
        |
        |--- T = GaussianVoxel  -->  GaussianVoxelMapCPU
        |
        |--- T = GMMVoxel       -->  GMMVoxelMapCPU
```

클래스의 공개 인터페이스를 구성하는 주요 메서드:

| 메서드 | 설명 |
|--------|------|
| `insert(frame, T_frame)` | 포인트 클라우드를 변환하여 복셀맵에 삽입 |
| `knn_search(query, k, indices, dists)` | 가장 가까운 K개의 가우시안/GMM 성분 탐색 |
| `set_neighbor_voxel_mode(mode)` | 이웃 탐색 반경 설정 (1/7/19/27) |
| `point(i)`, `cov(i)` | 전역 인덱스 `i`로 포인트/공분산 접근 |

---

### 1.2 이중 저장 구조: 해시맵 + 플랫 벡터

`IncrementalVoxelMap<T>` 내부에는 두 가지 자료구조가 공존한다.

```
복셀 조회 (O(1))                 복셀 순회 (캐시 친화적)
       |                                  |
       v                                  v
unordered_map<Vector3i,          vector<shared_ptr<
  size_t,                          pair<VoxelInfo,
  XORVector3iHash>                      VoxelContents>>>
       |                                  |
  voxels[coord] = idx            flat_voxels[idx]
```

- `voxels`: 정수 좌표 `Vector3i` → `flat_voxels` 인덱스로의 해시맵. 복셀 존재 여부 확인과 좌표 기반 조회에 사용된다.
- `flat_voxels`: 실제 데이터를 담는 플랫 벡터. 전체 복셀 순회 시 캐시 지역성이 좋다.

각 원소는 `shared_ptr<pair<VoxelInfo, VoxelContents>>`이다. `VoxelInfo`는 LRU 관리용 메타데이터를 담는다.

```cpp
struct VoxelInfo {
    size_t   lru;    // 마지막으로 접근된 시각 (삽입 카운터 기준)
    Vector3i coord;  // 정수 좌표 (역방향 조회용)
};
```

`VoxelContents`가 `GaussianVoxel`이면 단일 가우시안, `GMMVoxel`이면 GMM 성분 리스트를 저장한다.

---

### 1.3 복셀 좌표 계산: fast_floor()

**파일**: `thirdparty/gtsam_points/include/gtsam_points/util/fast_floor.hpp`

포인트를 어느 복셀에 넣을지 결정하려면 3D 좌표를 정수 격자 좌표로 변환해야 한다.

$$\text{coord} = \lfloor \mathbf{p}_{xyz} \cdot \frac{1}{\ell} \rfloor$$

여기서 $\ell$은 복셀 한 변의 길이(leaf size)다. 곱셈이 나눗셈보다 빠르므로 `inv_leaf_size = 1.0 / leaf_size`를 미리 계산해 저장한다.

코드로는 다음과 같다.

```cpp
// 내부적으로 이렇게 동작
coord = fast_floor(pt.head<3>() * inv_leaf_size).cast<int>();
```

`fast_floor()`는 `std::floor()`보다 빠르다. `std::floor()`는 음수 처리를 위해 분기나 특수 명령을 쓰지만, `fast_floor()`는 double을 int로 단순 truncation한 뒤 음수 값에 -1을 보정하는 방식을 쓴다.

```
예시: leaf_size = 0.5, inv_leaf_size = 2.0

  포인트 (0.3, -0.1, 1.7)
  * 2.0 → (0.6, -0.2, 3.4)
  fast_floor → (0, -1, 3)     ← 복셀 좌표
```

이 좌표가 해시맵의 키가 된다.

---

### 1.4 공간 해싱: XORVector3iHash

**파일**: `thirdparty/gtsam_points/include/gtsam_points/util/vector3i_hash.hpp`

`unordered_map`의 해시 함수로 `XORVector3iHash`를 사용한다. `Vector3i`의 세 정수 좌표를 XOR-mixing으로 결합한다.

```cpp
struct XORVector3iHash {
    size_t operator()(const Vector3i& x) const {
        size_t hx = hash(x[0]);
        size_t hy = hash(x[1]);
        size_t hz = hash(x[2]);
        return hx
             ^ ((hy << 16) | (hy >> 16))
             ^ ((hz << 32) | (hz >> 32));
    }
};
```

각 축의 해시를 서로 다른 비트 위치로 시프트하여 XOR하면, 좌표 값의 비트가 고르게 퍼져 해시 충돌이 줄어든다. 특히 공간 데이터는 인접 좌표끼리 유사한 값을 가지므로 이런 mixing이 중요하다.

---

### 1.5 GaussianVoxel의 add() / finalize()

**파일**: `thirdparty/gtsam_points/include/gtsam_points/types/gaussian_voxelmap_cpu.hpp`

`GaussianVoxel`은 복셀 내 모든 포인트를 단일 가우시안 분포로 요약한다.

**add() — 포인트 축적 (Welford 스타일)**

```cpp
void add(const Setting&, const Vector4d& pt, ...) {
    num_points++;
    mean += pt;             // 합산 (아직 나누지 않음)
    cov  += pt * pt.transpose();  // 외적 합산
}
```

`mean`은 정규화 전까지 포인트들의 합이다. `pt`의 w 성분은 1이므로 `mean.w = num_points`가 된다.

**finalize() — 정규화**

```cpp
void finalize() {
    mean /= num_points;
    cov   = cov / num_points - mean * mean.transpose();
    finalized = true;
}
```

표본 공분산 공식을 풀어 쓰면:

$$\Sigma = \frac{1}{N}\sum_{i} \mathbf{p}_i \mathbf{p}_i^T - \bar{\mathbf{p}}\bar{\mathbf{p}}^T$$

이것이 `cov/num_points - mean*mean^T`다. finalize 후 `mean.w = 1` (동차 좌표), `size()` 는 항상 1을 반환한다.

**knn_search() — 단순 유클리드 거리**

```cpp
double dist = (pt - mean).squaredNorm();
```

`pt.w = 1`, `mean.w = 1` 이므로 `(pt - mean).w = 0`이 자연스럽게 성립한다. diff의 w 성분을 0으로 강제할 필요가 없다.

---

### 1.6 이웃 탐색: neighbor_offsets(1/7/19/27)

포인트를 가장 가까운 가우시안에 대응시킬 때, 포인트가 속한 복셀 하나만 확인하면 경계면 근처에서 틀린 대응이 나올 수 있다. 이를 위해 이웃 복셀도 함께 조회한다.

`set_neighbor_voxel_mode(mode)`는 내부적으로 `neighbor_offsets(mode)`를 호출해 오프셋 집합을 생성한다.

```
mode=1  → 중심 복셀만 (오프셋 1개)
         [0,0,0]

mode=7  → 중심 + 6면 (오프셋 7개)
         [0,0,0], [±1,0,0], [0,±1,0], [0,0,±1]

mode=19 → 7 + 12 엣지 (오프셋 19개)
         + [±1,±1,0], [±1,0,±1], [0,±1,±1]

mode=27 → 전체 3×3×3 (오프셋 27개)
         모든 조합 [-1,0,1]^3
```

각 소스 포인트에 대해 이 오프셋들을 해당 포인트의 복셀 좌표에 더해 이웃 복셀 좌표들을 생성하고, 해시맵에서 각각 조회한다.

---

### 1.7 LRU 캐시 퇴출(Eviction) 메커니즘

온라인 SLAM에서는 로봇이 이동하면서 이전에 본 영역의 복셀이 더 이상 필요 없어진다. 메모리를 무한정 늘릴 수 없으므로 오래된 복셀을 주기적으로 제거한다.

관련 파라미터:

| 파라미터 | 역할 |
|---------|------|
| `lru_counter` | 지금까지 삽입한 포인트 수 (단조증가) |
| `lru_horizon` | 이 값보다 오래된 복셀을 제거 |
| `lru_clear_cycle` | 몇 번의 insert마다 LRU 청소를 실행할지 |

동작 흐름:

```
insert() 호출마다:
  lru_counter++
  접근된 복셀의 voxelinfo.lru = lru_counter

매 lru_clear_cycle 번째 insert에서:
  flat_voxels를 순회
  if voxelinfo.lru < lru_counter - lru_horizon:
      voxels.erase(coord)
      flat_voxels에서 제거 (swap-and-pop)
```

`lru_horizon`이 클수록 더 많은 복셀이 유지된다. 이 값은 사용 환경에 맞춰 튜닝해야 한다.

---

### 1.8 전역 인덱싱: calc_index()의 비트 인코딩

`flat_voxels`의 각 복셀에는 여러 포인트(또는 GMM 성분)가 담길 수 있다. 이를 단일 `size_t` 인덱스로 표현하기 위해 비트 인코딩을 사용한다.

```cpp
// point_id_bits = 32
size_t calc_index(size_t voxel_id, size_t point_id) {
    return (voxel_id << 32) | point_id;
}

size_t voxel_id(size_t i) { return i >> 32; }
size_t point_id(size_t i) { return i & 0xFFFFFFFF; }
```

```
bit 63 ..... bit 32 | bit 31 ..... bit 0
   voxel_id (32bit) | point_id (32bit)
```

이 인코딩으로 복셀 최대 개수는 $2^{32} \approx 43$억 개, 복셀당 포인트/성분 최대 수도 $2^{32}$개다. 실제 사용에서는 충분히 큰 범위다.

`IncrementalVoxelMap`의 `point(i)` 같은 접근자는 이 인덱스를 분해해 `flat_voxels[voxel_id(i)]->second`에 접근한 뒤, `frame::traits<VoxelContents>::point(voxel, point_id(i))`를 호출한다.

---

### 1.9 frame::traits\<T\> 특수화 체인

`gtsam_points`는 다양한 포인트 클라우드 타입을 통일된 인터페이스로 다루기 위해 `frame::traits<T>` 특수화 패턴을 쓴다. 이는 C++ 정책(policy) 기반 설계의 변형이다.

`GaussianVoxelMapCPU`를 통한 호출 체인:

```
frame::traits<GaussianVoxelMapCPU>::point(map, i)
        |
        v
IncrementalVoxelMap<GaussianVoxel>::point(i)
        |
        v
frame::traits<GaussianVoxel>::point(voxel, point_id)
        |
        v
return voxel.mean  (const Vector4d&)
```

각 계층별 특수화:

1. `traits<GaussianVoxel>`: `point()` 가 `const Vector4d&`를 반환한다. `mean`이 복셀 객체 내부에 저장되어 있으므로 참조 반환이 안전하다.

2. `traits<IncrementalVoxelMap<VoxelContents>>`: 제네릭 템플릿. `ivox.point(i)`를 거쳐 최종적으로 `frame::point(flat_voxels[voxel_id]->second, point_id)`를 호출한다. 반환 타입은 `const&`.

3. `traits<GaussianVoxelMapCPU>`: `IncrementalVoxelMap` 계층으로 위임한다.

`GMMVoxel`의 경우 이 체인이 다르게 동작한다. 자세한 내용은 [2.8절](#28-frametrait-specialization과-dangling-reference-방지)에서 다룬다.

---

### 1.10 GaussianVoxelMapCPU: GaussianVoxelMap + IncrementalVoxelMap\<GaussianVoxel\>

**파일**: `thirdparty/gtsam_points/include/gtsam_points/types/gaussian_voxelmap_cpu.hpp`

```cpp
class GaussianVoxelMapCPU
    : public GaussianVoxelMap,                    // 추상 인터페이스
      public IncrementalVoxelMap<GaussianVoxel>   // 실제 구현
{};
```

`GaussianVoxelMap`은 순수 가상 함수로 구성된 추상 인터페이스다. factor 코드가 구체 타입에 의존하지 않고 이 인터페이스만 알아도 동작하도록 설계되었다.

`IncrementalVoxelMap<GaussianVoxel>`이 실제 공간 해싱, LRU, 삽입 로직을 제공한다.

편의 메서드들:

- `voxel_coord(i)`: 전역 인덱스 `i`로 복셀 좌표 반환
- `lookup_voxel_index(coord)`: 정수 좌표로 flat_voxels 인덱스 반환
- `lookup_voxel(coord)`: 정수 좌표로 `GaussianVoxel` 참조 반환

`insert()`는 `IncrementalVoxelMap::insert()`로 위임하며, 삽입 즉시 각 복셀의 `finalize()`가 호출된다(지연 없음).

---

## 2. GMM 복셀 생성 구조

이 섹션은 `GMMVoxel`이 단일 가우시안 대신 가우시안 혼합 모델(GMM)을 어떻게 유지하는지, 그리고 `GMMVoxelMapCPU`가 `GaussianVoxelMapCPU`와 어떻게 다른지 설명한다.

---

### 2.1 GMMVoxel 구조 개요

**파일**: `include/gmm/gmm_voxelmap_cpu.hpp`

```cpp
struct GMMVoxel {
    // --- 축적 단계 ---
    vector<Vector4d>  reservoir_;         // 저수지 샘플 (최대 C개)
    size_t            total_points_seen_; // 지금까지 본 포인트 수
    bool              dirty_;             // finalize가 필요한지 여부
    mt19937           rng_;               // 난수 생성기 (seed=42)

    // --- finalize 결과 ---
    vector<GMMComponent> components_;     // 적합된 GMM 성분들
    Setting              cached_setting_; // finalize 시 사용된 설정
};
```

`GMMComponent`는 하나의 가우시안 성분을 나타낸다.

```cpp
struct GMMComponent {
    Vector4d mean;    // 평균 (w=0)
    Matrix4d cov;     // 공분산 (3×3 유효, 4행4열은 0)
    double   weight;  // 혼합 가중치 π_k (합 = 1)
};
```

`Setting`은 EM 알고리즘의 하이퍼파라미터 모음이다.

```cpp
struct Setting {
    int    max_components          = 3;
    int    max_em_iterations       = 20;
    double convergence_tol         = 1e-4;
    double covariance_regularization = 1e-3;
    double min_weight_threshold    = 0.01;
    int    reservoir_capacity      = 256;
};
```

---

### 2.2 Algorithm R 저수지 샘플링 (Vitter, 1985)

복셀 하나에 수천 개의 포인트가 들어올 수도 있는데, EM 알고리즘은 $O(K \times N \times D^2 \times \text{iters})$ 비용이 든다. 따라서 각 복셀마다 포인트를 최대 `reservoir_capacity`(기본 256)개로 제한한다.

단순히 처음 256개를 쓰면 공간적으로 편향된 샘플이 된다. 저수지 샘플링은 $N$번째 포인트까지 **모든 포인트가 동일한 확률**로 최종 샘플에 포함되도록 보장한다.

**Algorithm R** (Vitter, 1985):

```
C = reservoir_capacity (예: 256)
N = total_points_seen_

포인트 p가 들어올 때:
  N++
  if N <= C:
      reservoir_.push_back(p)    // 아직 저수지가 안 찼으면 그냥 추가
  else:
      j = Uniform(0, N-1)        // 0 이상 N-1 이하 균일 정수
      if j < C:
          reservoir_[j] = p      // j번째 슬롯을 새 포인트로 교체
      // else: 버림
```

확률 분석: $N$번째 포인트가 최종 샘플에 포함될 확률은 $\frac{C}{N}$이다. 이전 포인트들도 각각 $\frac{C}{N}$의 확률을 유지한다. 증명은 귀납법으로 쉽게 확인할 수 있다.

중요한 세부사항: `add()` 내부에서 `pt(3) = 0.0`으로 강제한다. GMM 성분의 mean도 w=0이므로, knn_search 시 diff(3) 처리와 일관성을 맞추기 위함이다.

---

### 2.3 2단계 지연 패턴: insert → finalize_all

`GaussianVoxel`은 `add()` 후 즉시 `finalize()`가 호출된다. 반면 `GMMVoxel`은 EM 비용 때문에 지연 패턴을 쓴다.

```
1단계: 포인트 축적
  GMMVoxelMapCPU::insert(frame)
        |
        v
  IncrementalVoxelMap::insert() 로직
        |
        v
  각 포인트에 대해 GMMVoxel::add() 호출
        |
        v
  dirty_ = true  (finalize 필요 표시)
  needs_finalize_ = true  (맵 수준 플래그)
  [finalize() 는 호출하지 않음!]

2단계: 일괄 EM 적합
  GMMVoxelMapCPU::finalize_all()
        |
        v
  #pragma omp parallel for
  flat_voxels 전체 순회
        |
        v
  dirty_ == true 인 복셀마다 GMMVoxel::finalize() 호출
        |
        v
  needs_finalize_ = false
```

`finalize_all()`은 두 경로로 트리거될 수 있다.

1. **명시적 호출**: `main.cpp`의 `ensure_gmm_voxelmaps()`가 삽입 후 직접 호출
2. **지연 트리거**: `knn_search()` 진입 시 `needs_finalize_`가 true면 자동 호출

그런데 `IntegratedMixtureLightNDTFactor`의 `update_correspondences()`는 `knn_search()`를 우회하고 `lookup_voxel().components()`에 직접 접근한다. 따라서 지연 트리거가 작동하지 않고, 명시적 `finalize_all()` 호출이 반드시 필요하다.

`needs_finalize_`는 `mutable bool`로 선언되어 있어 `const` 메서드에서도 변경할 수 있다.

---

### 2.4 fit_gmm() Cold-start vs Warm-start

**파일**: `src/gmm/mixture_em_backend.cpp`

`GMMVoxel::finalize()`는 내부적으로 `fit_gmm()`을 호출한다. 복셀에 기존 성분이 있는지에 따라 초기화 전략이 갈린다.

**Cold-start** (처음 적합 또는 기존 성분 없음):

```
gmm_full::learn(
    data,
    K = max_components,
    eucl_dist,
    random_spread,   // k-means++ 스타일 초기화
    20,              // k-means 사전 반복 수
    max_iters,
    tol
)
```

`random_spread`는 첫 번째 중심을 무작위로 선택한 후, 나머지 중심들을 거리 비례 확률로 선택하는 k-means++ 방식이다. 20번의 k-means 반복 후 EM을 시작한다.

**Warm-start** (기존 성분 존재):

```
// 이전 공분산에서 정규화 항 제거 (이중 정규화 방지)
for each component k:
    cov_diag -= epsilon
    cov_diag = max(cov_diag, 1e-6)  // 음수 방지

// Armadillo 모델에 이전 파라미터 주입
gmm.set_params(previous_means, previous_covs, previous_weights)

// k-means 없이 바로 EM
gmm_full::learn(
    data,
    K,
    eucl_dist,
    keep_existing,  // 주입된 파라미터 유지
    0,              // k-means 사전 반복 없음
    max_iters,
    tol
)
```

De-regularization이 필요한 이유: 저장된 공분산에는 이미 `covariance_regularization`(기본 1e-3)이 더해져 있다. warm-start 전에 이를 빼지 않으면 `extract_result()`에서 또 더해져 이중 정규화가 발생한다.

**Warm-start 실패 처리**: warm-start가 실패하면 cold-start로 자동 재시도한다.

---

### 2.5 extract_result() 후처리 파이프라인

EM 적합이 끝나면 Armadillo 형식을 Eigen 형식으로 변환하고 후처리한다.

**3단계 파이프라인**:

```
[1단계] Armadillo 3D → Eigen 4D 변환
  arma::vec mean3 = gmm.means.col(k)
  Vector4d mean4 = (mean3[0], mean3[1], mean3[2], 0.0)  // w=0

  arma::mat cov3 = gmm.fcovs.slice(k)  // 3×3
  Matrix4d cov4 = 0                    // 4×4 영행렬
  cov4.topLeftCorner<3,3>() = cov3     // 왼쪽 위 3×3만 복사

[2단계] 공분산 정규화
  cov4(0,0) += epsilon
  cov4(1,1) += epsilon
  cov4(2,2) += epsilon
  // 4×4 (3,3) 원소는 건드리지 않음

[3단계] 가중치 가지치기 및 정규화
  for each component k:
      if weight_k < min_weight_threshold:
          remove component k

  if all pruned:
      fallback: 샘플들의 가중 평균으로 단일 성분 생성

  // 남은 성분들의 가중치 재정규화
  total = sum(weight_k)
  for each k: weight_k /= total
```

---

### 2.6 폴백 체인 (Edge Cases)

복셀에 포인트가 거의 없거나 EM이 실패하는 상황을 위한 폴백 체인:

```
N = 0
  └─> components_ = {}  (빈 결과, knn_search에서 제외)

N = 1
  └─> single_component_fallback()
      mean = p_0, cov = epsilon * I_3

EM 수렴 실패
  └─> single_component_fallback()

모든 성분 가지치기됨
  └─> weighted_mean_fallback()
      mean = Σ(π_k * mean_k) / Σπ_k (단일 성분)

Warm-start 실패
  └─> Cold-start 재시도
      실패 시 single_component_fallback()

K > max_components
  └─> Cold-start (K를 max_components로 제한)
```

`single_component_fallback()`은 $\epsilon I_3$ 공분산을 써서 거의 점에 가까운 가우시안을 만든다. factor 계산 시 이 복셀은 높은 Mahalanobis 거리를 가져 사실상 대응에서 제외된다.

---

### 2.7 GMMVoxelMapCPU의 다중 상속 설계

**파일**: `include/gmm/gmm_voxelmap_cpu.hpp`

```cpp
class GMMVoxelMapCPU
    : public GaussianVoxelMap,              // 추상 인터페이스
      public IncrementalVoxelMap<GMMVoxel>  // 실제 구현
{
    mutable bool needs_finalize_ = false;

public:
    void insert(const Frame& frame, const SE3& T = SE3());
    void finalize_all();
    void knn_search(...) override;
};
```

`GaussianVoxelMap`을 상속하는 이유: `IntegratedMixtureLightNDTFactor`가 `GaussianVoxelMap*`으로 target voxelmap을 받으므로, `dynamic_pointer_cast<GMMVoxelMapCPU>`로 다운캐스트할 수 있어야 한다.

`IncrementalVoxelMap<GMMVoxel>`을 상속하는 이유: 공간 해싱, LRU, 이웃 탐색 로직을 그대로 재사용하기 위해서다.

`insert()` 오버라이드의 핵심 차이:

```
GaussianVoxelMapCPU::insert()
    → IncrementalVoxelMap::insert()
    → 각 포인트마다 GaussianVoxel::add() 후 즉시 finalize()

GMMVoxelMapCPU::insert()
    → IncrementalVoxelMap::insert() (add만 호출)
    → finalize() 는 호출하지 않음
    → needs_finalize_ = true
```

---

### 2.8 frame::traits\<GMMVoxel\> 특수화와 dangling reference 방지

`GMMVoxel`의 traits 특수화는 `GaussianVoxel`과 중요한 차이가 있다.

**문제**: `GMMVoxel::point(k)`는 `components_[k].mean`을 반환해야 하는데, `components_`는 `vector`다. 제네릭 `traits<IncrementalVoxelMap<VoxelContents>>`는 `const Vector4d&`를 반환하도록 구현되어 있다.

만약 `traits<GMMVoxel>::point()`가 값(value)을 반환하면, 이 값은 임시 객체다. `traits<IncrementalVoxelMap<GMMVoxel>>`의 제네릭 구현이 이 임시 객체에 `const&`를 만들면 곧바로 dangling reference가 된다.

**해결**: `traits<IncrementalVoxelMap<GMMVoxel>>`를 **명시적으로 특수화**하여 값 반환으로 오버라이드한다.

```
traits<GMMVoxel>::point(voxel, k)
    → return voxel.components_[k].mean  (Vector4d by value)
                                             |
                                             v
traits<IncrementalVoxelMap<GMMVoxel>>::point(ivox, i)
    [명시적 특수화 — 제네릭 템플릿 아님]
    → return frame::point(flat_voxels[voxel_id(i)]->second, point_id(i))
    → 반환 타입: Vector4d (by value, decltype(auto)로 전파)

traits<GMMVoxelMapCPU>::point(map, i)
    → decltype(auto) 사용 → 값 의미론 그대로 전파
```

제네릭 템플릿을 그대로 쓰면:

```
제네릭 traits<IncrementalVoxelMap<GMMVoxel>>::point()
    → const Vector4d& ref = frame::point(voxel, k)
                                   ↑
                          임시 Vector4d를 가리키는 dangling reference!
```

명시적 특수화를 통해 이 문제를 컴파일 타임에 막는다.

---

### 2.9 GaussianVoxel vs GMMVoxel 비교 테이블

| 항목 | GaussianVoxel | GMMVoxel |
|------|--------------|---------|
| 복셀당 분포 수 | 1 (단일 가우시안) | K (최대 max_components, 기본 3) |
| 포인트 축적 방식 | 누적합 (Welford) | 저수지 샘플링 (Algorithm R) |
| 축적 메모리 | `num_points`, `mean`, `cov` 합산 | `reservoir_` 벡터 (최대 256×32bytes) |
| finalize 비용 | $O(1)$ (나누기만) | $O(K \times N \times D^2 \times \text{iters})$ EM |
| finalize 타이밍 | `add()` 직후 즉시 | `finalize_all()`로 지연 (일괄 처리) |
| 병렬화 | 없음 (단순 연산) | OpenMP parallel for |
| mean의 w 값 | w=1 (동차 좌표) | w=0 (3D 방향 벡터 관례) |
| diff(3) 처리 | 불필요 (자연히 0) | 필수 (`diff(3) = 0` 강제) |
| 메모리/복셀 | ~160 bytes | ~160×K + reservoir×32 bytes |
| Setting 구조체 | 빈 구조체 `{}` | 6개 하이퍼파라미터 |
| `size()` 반환값 | 항상 1 | `components_.size()` (0~K) |
| traits::point() 반환 | `const Vector4d&` (참조) | `Vector4d` (값, dangling ref 방지) |
| knn_search 기준 | $(p - \mu)^2$ 유클리드 | Mahalanobis 거리 $\Delta^T \Sigma^{-1} \Delta$ |
| warm-start 지원 | 없음 | 있음 (기존 성분 재활용) |

---

## 3. IntegratedMixtureLightNDTFactor vs IntegratedLightNDTFactor 구현 비교

이 섹션은 두 factor의 구현 차이를 코드 수준에서 비교하고, SE(3) Jacobian 유도 과정을 완전히 설명한다.

---

### 3.1 공통 기반: IntegratedMatchingCostFactor

두 factor 모두 `IntegratedMatchingCostFactor`를 상속한다. 이 기반 클래스는 Template Method 패턴으로 구현되어 있다.

```
IntegratedMatchingCostFactor
        |
        |--- linearize(values)
        |        |
        |        v
        |    delta = T_target^{-1} * T_source
        |    update_correspondences(delta)  [가상 함수]
        |    evaluate(delta, H, b)          [가상 함수]
        |    return HessianFactor(...)
        |
        |--- IntegratedLightNDTFactor
        |       override: update_correspondences()
        |       override: evaluate()
        |
        +--- IntegratedMixtureLightNDTFactor
                override: update_correspondences()
                override: evaluate()
```

`linearize()`는 GTSAM 옵티마이저가 각 반복에서 호출한다. 결과로 `HessianFactor`를 반환하고, 옵티마이저는 이를 Cholesky 분해해 포즈 업데이트 $\Delta\xi$를 구한다.

**대응 재계산 조건**: `delta`가 이전 값과 충분히 달라졌을 때만 `update_correspondences()`를 호출한다. 포즈 변화가 미미하면 이전 대응을 재사용해 계산 비용을 줄인다.

---

### 3.2 복셀맵 타입과 대응 구조체 차이

| 측면 | IntegratedLightNDTFactor | IntegratedMixtureLightNDTFactor |
|------|--------------------------|--------------------------------|
| 복셀맵 타입 | `GaussianVoxelMapCPU` | `GMMVoxelMapCPU` |
| 대응 구조체 | `NdtCorrespondence` | `MixtureNdtCorrespondence` |
| 정규화 계수 | `one_over_z` ($1/Z$) | `pi_k` ($\pi_k$, 혼합 가중치) |
| 대응 수 | 소스 포인트당 1개 | 소스 포인트당 최대 $K$개 |

```cpp
struct NdtCorrespondence {
    Vector4d  mean;       // 복셀 평균 (w=1)
    Matrix4d  inv_cov;    // 역공분산
    double    one_over_z; // 정규화 인수
    size_t    voxel_id;   // 복셀 인덱스
};

struct MixtureNdtCorrespondence {
    Vector4d  mean;       // 성분 평균 (w=0)
    Matrix4d  inv_cov;    // 역공분산
    double    pi_k;       // 혼합 가중치
    size_t    voxel_id;   // 복셀 인덱스
    int       component;  // 성분 인덱스 k
};
```

---

### 3.3 update_correspondences() 코드 수준 비교

**IntegratedLightNDTFactor**:

```
각 소스 포인트 p_i에 대해:
  1. T * p_i 로 포인트 변환
  2. 변환된 포인트의 복셀 좌표 계산
  3. 이웃 복셀(neighbor_offsets 기반) 순회
  4. 각 복셀의 단일 GaussianVoxel 가져옴
  5. 유클리드 거리 (p - mean)^2 계산
  6. 최솟값인 복셀 선택
  7. NdtCorrespondence{ mean, inv_cov, one_over_z } 저장
```

**IntegratedMixtureLightNDTFactor**:

```
각 소스 포인트 p_i에 대해:
  1. T * p_i 로 포인트 변환
  2. 변환된 포인트의 복셀 좌표 계산
  3. 이웃 복셀 순회
  4. 각 복셀의 GMMVoxel에서 모든 성분 K개 순회
  5. 각 성분에 대해 Mahalanobis 거리 계산:
       diff = pt - mean_k,  diff(3) = 0
       d^2 = diff^T * inv_cov_k * diff
  6. 전체 이웃 복셀 × K 성분 중 d^2 최솟값 선택 (argmin)
  7. MixtureNdtCorrespondence{ mean_k, inv_cov_k, pi_k } 저장
```

핵심 차이: LightNDT는 이웃 복셀 수준에서 argmin을 찾지만, MixtureLightNDT는 **복셀 × 성분** 전체에서 argmin을 찾는다. 성분 수 K=3, 이웃 수 7이면 21개 후보 중 최선을 고른다.

`diff(3) = 0` 처리가 중요하다. `pt.w = 1`, `mean_k.w = 0`이므로 `diff.w = 1`이 되는데, 이를 0으로 강제하지 않으면 Mahalanobis 거리 계산 시 w 성분이 오염된다. LightNDT는 `mean.w = 1`이므로 `diff.w = 0`이 자연히 성립해 이 처리가 필요 없다.

---

### 3.4 evaluate() 코드 수준 비교

**IntegratedLightNDTFactor**:

$$\text{cost} = \sum_i \frac{1}{Z_i} \mathbf{r}_i^T \Sigma_i^{-1} \mathbf{r}_i$$

```
각 대응 c_i에 대해:
  r = T * p_source - c_i.mean   (잔차)
  r(3) = 0                      (LightNDT는 불필요하지만 안전하게 처리)
  H += (1/Z) * J^T * inv_cov * J
  b += (1/Z) * J^T * inv_cov * r
  cost += (1/Z) * r^T * inv_cov * r
```

**IntegratedMixtureLightNDTFactor**:

$$\text{cost} = \sum_i \pi_{k(i)} \cdot \mathbf{r}_i^T \Sigma_{k(i)}^{-1} \mathbf{r}_i$$

```
각 대응 c_i에 대해:
  r = T * p_source - c_i.mean   (잔차)
  r(3) = 0                      (mean.w=0, pt.w=1 → r.w=1이 되므로 반드시 필요)
  H += pi_k * J^T * inv_cov * J
  b += pi_k * J^T * inv_cov * r
  cost += pi_k * r^T * inv_cov * r
```

두 factor의 Hessian 기여 차이를 도식화하면:

```
LightNDT:         (1/Z) × [J^T Σ^{-1} J]
MixtureLightNDT:   π_k  × [J^T Σ^{-1} J]
```

`r(3) = 0` 처리가 MixtureLightNDT에서 필수인 이유: 잔차의 4번째 성분이 0이 아니면 `inv_cov × r`이 오염된다. `inv_cov`의 4행4열은 0이지만, 수치 오차와 코드 안정성을 위해 명시적으로 0을 대입한다.

---

### 3.5 inv_cov_cache: 1D vs 2D

역공분산 계산은 비싸므로 캐시한다.

**IntegratedLightNDTFactor**:

```cpp
vector<Matrix4d> inv_cov_cache;
// 인덱싱: inv_cov_cache[voxel_id]
// 크기: V (복셀 수)
```

**IntegratedMixtureLightNDTFactor**:

```cpp
vector<vector<Matrix4d>> inv_cov_cache;
// 인덱싱: inv_cov_cache[voxel_id][k]
// 크기: V × K
```

메모리 사용량 추정 (복셀 수 10,000, K=3):

$$10{,}000 \times 3 \times 128 \text{ bytes} = 3.84 \text{ MB}$$

`Matrix4d`는 $4 \times 4 \times 8 = 128$ bytes다. 실용적인 크기지만, K나 복셀 수가 크게 늘면 주의해야 한다.

`update_correspondences()` 시 `inv_cov_cache`를 먼저 채우고, `evaluate()` 시 재사용한다. 같은 복셀에 대해 여러 소스 포인트가 대응되더라도 역공분산은 한 번만 계산한다.

---

### 3.6 SE(3) Jacobian 유도 (완전한 유도 과정)

두 factor 모두 동일한 SE(3) Jacobian을 사용한다. 잔차 $\mathbf{r} = T \mathbf{p} - \boldsymbol{\mu}$ 에 대한 Jacobian이다.

**표기법 정의**:
- $T \in SE(3)$: $4 \times 4$ 변환 행렬 (회전 $R$ + 평행이동 $\mathbf{t}$)
- $\mathbf{p} \in \mathbb{R}^4$: 소스 포인트 (동차 좌표, $p_w = 1$)
- $\boldsymbol{\mu} \in \mathbb{R}^4$: 타겟 평균
- $\xi \in \mathfrak{se}(3)$: 李 대수 원소 (6차원 트위스트: $[\boldsymbol{\omega}^\top, \mathbf{v}^\top]^\top$)

**잔차의 선형화**:

작은 교란 $\delta\xi$에 대해 $T' = \exp(\hat{\delta\xi}) \cdot T$라 하면:

$$T' \mathbf{p} \approx T\mathbf{p} + \frac{\partial (T\mathbf{p})}{\partial \xi} \delta\xi$$

**타겟 포즈에 대한 Jacobian** $J_{\text{target}} \in \mathbb{R}^{4 \times 6}$:

타겟 포즈가 변할 때 잔차 $\mathbf{r} = T_t^{-1}(T_s \mathbf{p}_s) - \boldsymbol{\mu}$의 변화율.

$T_t^{-1} = [R_t^\top, -R_t^\top \mathbf{t}_t; \mathbf{0}^\top, 1]$이고, 타겟 포즈 교란 $\delta\xi_t$에 대해:

$$J_{\text{target}} = \begin{bmatrix} -[T_t \mathbf{p}_t]_\times & I_3 \\ 0 & 0 \end{bmatrix} \in \mathbb{R}^{4 \times 6}$$

여기서 $[T_t \mathbf{p}_t]_\times$는 벡터의 반대칭 행렬(skew-symmetric matrix):

$$[\mathbf{v}]_\times = \begin{bmatrix} 0 & -v_3 & v_2 \\ v_3 & 0 & -v_1 \\ -v_2 & v_1 & 0 \end{bmatrix}$$

**소스 포즈에 대한 Jacobian** $J_{\text{source}} \in \mathbb{R}^{4 \times 6}$:

소스 포인트 $\mathbf{p}_s$에 변환 $T_s$를 적용할 때의 Jacobian:

$$J_{\text{source}} = \begin{bmatrix} R_s [\mathbf{p}_s]_\times & -R_s \\ 0 & 0 \end{bmatrix} \in \mathbb{R}^{4 \times 6}$$

$R_s$는 소스 포즈의 회전 행렬이다.

**Hessian과 gradient 기여**:

잔차 $\mathbf{r}$, 역공분산 $\Sigma^{-1}$, 가중치 $w$ (LightNDT: $1/Z$, MixtureLightNDT: $\pi_k$)에 대해:

$$H_{tt} \mathrel{+}= w \cdot J_t^\top \Sigma^{-1} J_t$$
$$H_{ss} \mathrel{+}= w \cdot J_s^\top \Sigma^{-1} J_s$$
$$H_{ts} \mathrel{+}= w \cdot J_t^\top \Sigma^{-1} J_s$$
$$\mathbf{b}_t \mathrel{+}= w \cdot J_t^\top \Sigma^{-1} \mathbf{r}$$
$$\mathbf{b}_s \mathrel{+}= w \cdot J_s^\top \Sigma^{-1} \mathbf{r}$$

GTSAM은 이를 합쳐 블록 행렬 형태의 `HessianFactor`를 구성한다.

$$\begin{bmatrix} H_{tt} & H_{ts} \\ H_{ts}^\top & H_{ss} \end{bmatrix} \begin{bmatrix} \Delta\xi_t \\ \Delta\xi_s \end{bmatrix} = -\begin{bmatrix} \mathbf{b}_t \\ \mathbf{b}_s \end{bmatrix}$$

**Cholesky 분해를 통한 업데이트**:

$$\begin{bmatrix} \Delta\xi_t \\ \Delta\xi_s \end{bmatrix} = -H^{-1} \mathbf{b}$$

$$T'_s = \exp(\widehat{\Delta\xi_s}) \cdot T_s$$

이 과정이 Levenberg-Marquardt 옵티마이저의 각 반복에서 반복된다.

---

### 3.7 main.cpp에서 GTSAM optimizer까지의 전체 파이프라인 추적

최적화가 실행되는 전체 경로를 단계별로 추적한다.

```
main.cpp::run_optimization()
    |
    | [1] 복셀맵 생성 및 포인트 삽입
    v
GMMVoxelMapCPU::insert(target_frame, T_target)
    |
    v
IncrementalVoxelMap::insert()
    |--- 각 포인트 → fast_floor() → coord
    |--- coord 없으면 새 복셀 생성
    |--- GMMVoxel::add(pt) → reservoir sampling
    |--- lru 갱신
    v
needs_finalize_ = true

    |
    | [2] GMM 사전 적합 (명시적)
    v
ensure_gmm_voxelmaps() 또는 finalize_all()
    |
    v
#pragma omp parallel for flat_voxels
    |--- GMMVoxel::finalize()
    |     |--- fit_gmm(reservoir, setting)
    |     |     |--- cold/warm start
    |     |     |--- gmm_full::learn() [Armadillo EM]
    |     |     |--- extract_result()
    |     v
    |   components_ 채워짐, dirty_ = false
    v
needs_finalize_ = false

    |
    | [3] Factor 생성
    v
create_factor("MixtureLightNDT", target_key, source_key,
              target_voxels, source_frame)
    |
    v
IntegratedMixtureLightNDTFactor(
    target_key, source_key,
    dynamic_pointer_cast<GMMVoxelMapCPU>(target_voxels),
    source_frame
)

    |
    | [4] GTSAM graph에 추가
    v
graph.add(factor)
initial_values.insert(target_key, T_target_init)
initial_values.insert(source_key, T_source_init)

    |
    | [5] LM 최적화
    v
LevenbergMarquardtOptimizer::optimize()
    |
    | [각 반복 iteration]
    v
factor.linearize(current_values)
    |
    |--- delta = T_target^{-1} * T_source (상대 변환)
    |
    |--- update_correspondences(delta)
    |     |--- 각 소스 포인트 변환: p' = delta * p_s
    |     |--- 이웃 복셀 순회
    |     |--- GMMVoxel::components() 에서 모든 성분 접근
    |     |--- Mahalanobis 거리로 argmin 성분 선택
    |     |--- MixtureNdtCorrespondence 저장
    |     |--- inv_cov_cache[v][k] 채우기
    |
    |--- evaluate(delta, &H_t, &H_ts, &H_s, &b_t, &b_s)
    |     |--- 각 대응에 대해:
    |     |     r = T * p_s - mean_k, r(3) = 0
    |     |     J_t, J_s 계산 (SE3 Jacobian)
    |     |     H += pi_k * J^T * inv_cov * J
    |     |     b += pi_k * J^T * inv_cov * r
    |     |     cost += pi_k * r^T * inv_cov * r
    |
    v
HessianFactor(target_key, source_key,
              H_t, H_ts, b_t, H_s, b_s, cost)

    |
    |--- Cholesky: H Δξ = -b
    |--- T'_t = exp(Δξ_t) * T_t
    |--- T'_s = exp(Δξ_s) * T_s
    |--- 수렴 판정
    v
최적화된 T_target, T_source 반환
```

**`lookup_voxel().components()` 직접 접근 이유**:

`update_correspondences()`는 `knn_search()` 대신 `lookup_voxel(coord).components()`에 직접 접근한다. `knn_search()`는 내부적으로 유클리드 거리를 쓰지만, factor는 Mahalanobis 거리로 최선의 성분을 직접 골라야 하기 때문이다. 이 때문에 `knn_search()`의 지연 `finalize_all()` 트리거가 작동하지 않으므로, 명시적 `finalize_all()` 호출이 필수다.

---

## 4. 외부 의존성: Armadillo

---

### 4.1 Armadillo 개요

[Armadillo](http://arma.sourceforge.net/)는 C++ 선형 대수 라이브러리다. LAPACK, BLAS, ARPACK 등의 저수준 Fortran 라이브러리를 래핑하여 Eigen과 유사한 표현식 템플릿 문법을 제공한다.

이 프로젝트에서 Armadillo를 쓰는 이유는 단 하나다: `arma::gmm_full` 클래스. 이 클래스는 가우시안 혼합 모델의 EM 알고리즘 구현을 제공하며, 다음 기능들이 필요하다.

| 기능 | Armadillo API | 용도 |
|------|--------------|------|
| EM 적합 | `gmm_full::learn()` | cold/warm start EM |
| 파라미터 주입 | `gmm_full::set_params()` | warm-start 초기화 |
| 평균 추출 | `gmm.means.col(k)` | 성분 평균 가져오기 |
| 공분산 추출 | `gmm.fcovs.slice(k)` | full 공분산 행렬 |
| 가중치 추출 | `gmm.hefts(k)` | 혼합 가중치 |

Eigen에는 GMM EM 구현이 없으므로, 별도 라이브러리가 필요했다. Armadillo의 `gmm_full`은 full 공분산 행렬을 지원하는 완성도 높은 구현이다.

---

### 4.2 시스템 패키지 설치 방식 (apt)

Armadillo는 GitHub에서 클론하는 방식이 아니라 시스템 패키지로 설치된다.

```bash
apt install libarmadillo-dev
```

Docker 컨테이너 내 설치 버전: **10.8.2**

이 패키지를 설치하면 다음이 제공된다.

```
/usr/include/armadillo          (헤더 파일)
/usr/include/armadillo_bits/    (세부 헤더들)
/usr/lib/x86_64-linux-gnu/libarmadillo.so   (공유 라이브러리)
/usr/lib/cmake/armadillo/       (CMake 패키지 파일)
```

`libarmadillo-dev`는 내부적으로 LAPACK과 BLAS 구현에 의존한다. Ubuntu에서는 보통 `liblapack-dev`, `libblas-dev` (또는 OpenBLAS) 가 함께 설치된다.

---

### 4.3 CMakeLists.txt에서의 연결

```cmake
# Armadillo 패키지 탐색
find_package(Armadillo REQUIRED)

# 타겟에 인클루드 디렉토리와 라이브러리 연결
target_include_directories(gmm_library PRIVATE ${ARMADILLO_INCLUDE_DIRS})
target_link_libraries(gmm_library PRIVATE ${ARMADILLO_LIBRARIES})
```

`find_package(Armadillo REQUIRED)`는 `/usr/lib/cmake/armadillo/` 아래의 CMake 패키지 파일을 읽어 `ARMADILLO_INCLUDE_DIRS`와 `ARMADILLO_LIBRARIES` 변수를 설정한다.

`REQUIRED` 키워드 덕분에 Armadillo가 설치되지 않으면 CMake 구성 단계에서 즉시 오류가 발생한다.

---

### 4.4 프로젝트에서의 사용 범위

Armadillo는 프로젝트 전체에서 단 하나의 파일에서만 사용된다.

**파일**: `src/gmm/mixture_em_backend.cpp`

다른 파일들은 Armadillo 헤더를 인클루드하지 않는다. `gmm_voxelmap_cpu.hpp`, `gmm_component.hpp`, factor 코드 모두 Eigen만 사용한다.

사용 패턴:

```cpp
#include <armadillo>

// 데이터 변환: Eigen 3D → Armadillo (N×3 행렬)
arma::mat data(reservoir.size(), 3);
for (size_t i = 0; i < reservoir.size(); i++) {
    data(i, 0) = reservoir[i](0);
    data(i, 1) = reservoir[i](1);
    data(i, 2) = reservoir[i](2);
}

// GMM 모델 선언
arma::gmm_full gmm;

// EM 적합 (cold-start)
bool success = gmm.learn(
    data.t(),          // Armadillo는 열 우선(column-major)이므로 전치
    K,                 // 성분 수
    arma::eucl_dist,   // 거리 메트릭
    arma::random_spread,  // 초기화 전략
    20,                // k-means 사전 반복
    max_iters,
    tol,
    false              // verbose 출력 비활성화
);

// 결과 추출
for (int k = 0; k < K; k++) {
    arma::vec  mean3 = gmm.means.col(k);  // 3×1
    arma::mat  cov3  = gmm.fcovs.slice(k); // 3×3
    double     pi_k  = gmm.hefts(k);
    // ... Eigen 4D로 변환 ...
}
```

이 파일 외부에서 Armadillo가 보이지 않도록 구현을 캡슐화한 것은 좋은 설계다. `mixture_em_backend.cpp`를 다른 EM 구현으로 교체하더라도 나머지 코드는 변경이 없다.

---

*이 문서는 gtsam_points 복셀 구조와 GMM 확장에 관한 학습 세션을 정리한 것입니다. 코드 참조는 모두 `thirdparty/gtsam_points/` 및 `src/`, `include/gmm/` 하위 경로를 기준으로 합니다.*
