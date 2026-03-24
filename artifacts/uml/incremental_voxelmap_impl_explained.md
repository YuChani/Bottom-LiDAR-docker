# incremental_voxelmap_impl.hpp 수학·구조 해설

이 문서는 현재 코드 기준으로 `thirdparty/gtsam_points/include/gtsam_points/ann/impl/incremental_voxelmap_impl.hpp`가 실제로 어떤 구조의 voxel map을 만들고, 어떤 수학으로 점을 voxel에 넣고, 어떤 방식으로 이웃 검색을 수행하는지를 자세히 설명한다.

핵심 결론부터 말하면, 이 파일은 단순한 보조 함수 모음이 아니라:

- **증분 삽입 가능한 voxel 기반 최근접 탐색 자료구조의 실제 구현체**이고
- `GaussianVoxelMapCPU` 같은 상위 타입은 이 구현을 감싸는 **typed wrapper**이며
- 실제 voxel 생성은 `insert()` 안에서, 실제 최근접 탐색은 `knn_search()` 안에서 일어난다.

---

## 1. 이 파일이 맡는 역할

중심 파일:

- `thirdparty/gtsam_points/include/gtsam_points/ann/incremental_voxelmap.hpp`
- `thirdparty/gtsam_points/include/gtsam_points/ann/impl/incremental_voxelmap_impl.hpp`

이 둘의 관계는 다음처럼 이해하면 된다.

- `incremental_voxelmap.hpp`
  - 클래스 선언
  - 멤버 변수 정의
  - 인덱스 인코딩 함수 정의
- `incremental_voxelmap_impl.hpp`
  - 실제 동작 구현
  - `insert()`
  - `knn_search()`
  - `neighbor_offsets()`
  - `voxel_data()`

즉 이 코드는 “voxel map이라는 개념”을 선언만 해둔 것이 아니라, **점들을 voxel로 양자화해서 저장하고, 주변 voxel만 검사해 KNN을 수행하는 전체 알고리즘**을 담고 있다.

---

## 2. 큰 구조: 누가 실제 owner인가

현재 코드에서 중요한 타입 계층은 다음과 같다.

### 2.1 `IncrementalVoxelMap<VoxelContents>`

정의:

- `thirdparty/gtsam_points/include/gtsam_points/ann/incremental_voxelmap.hpp`

이 템플릿이 실제 owner다.

이 타입이 직접 들고 있는 것:

- voxel size 관련 상태
- neighbor voxel offset 목록
- LRU cache 상태
- voxel hash map
- voxel 배열

즉 “실제 voxel map 구현”은 이 템플릿이 한다.

### 2.2 `GaussianVoxelMapCPU`

정의:

- `thirdparty/gtsam_points/include/gtsam_points/types/gaussian_voxelmap_cpu.hpp`

구현:

- `thirdparty/gtsam_points/src/gtsam_points/types/gaussian_voxelmap_cpu.cpp`

이 타입은 다음처럼 정의되어 있다.

```cpp
class GaussianVoxelMapCPU : public GaussianVoxelMap, public IncrementalVoxelMap<GaussianVoxel>
```

즉 `GaussianVoxelMapCPU`는:

- `VoxelContents = GaussianVoxel`인
- `IncrementalVoxelMap`의 concrete specialization이다.

그리고 `GaussianVoxelMapCPU::insert()`는 사실상 아래 한 줄 위임이다.

```cpp
void GaussianVoxelMapCPU::insert(const PointCloud& frame) {
  IncrementalVoxelMap<GaussianVoxel>::insert(frame);
}
```

따라서 **실제로 voxel이 만들어지는 곳은 `GaussianVoxelMapCPU`가 아니라 `IncrementalVoxelMap<VoxelContents>::insert()`**라고 보는 것이 정확하다.

---

## 3. 자료구조

### 3.1 `VoxelInfo`

위치:

- `incremental_voxelmap.hpp:13-26`

정의 필드:

- `size_t lru`
- `Eigen::Vector3i coord`

역할:

- `coord`: 이 voxel이 정수 격자 공간에서 어느 칸인지
- `lru`: 최근에 삽입 시 사용된 시각(counter)

즉 `VoxelInfo`는 voxel의 **통계 내용**이 아니라 **관리 메타데이터**다.

---

### 3.2 실제 저장소: `flat_voxels`

위치:

- `incremental_voxelmap.hpp:129`

정의:

```cpp
std::vector<std::shared_ptr<std::pair<VoxelInfo, VoxelContents>>> flat_voxels;
```

이 의미는 다음과 같다.

```text
flat_voxels[k] = (VoxelInfo, VoxelContents)
```

즉 voxel 하나는 내부적으로:

- 관리 정보 `VoxelInfo`
- 실제 데이터 `VoxelContents`

의 pair로 저장된다.

여기서 `VoxelContents`는 템플릿 파라미터다. 현재 `GaussianVoxelMapCPU`에서는 `GaussianVoxel`가 들어간다.

---

### 3.3 빠른 조회용 해시: `voxels`

위치:

- `incremental_voxelmap.hpp:130`

정의:

```cpp
std::unordered_map<Eigen::Vector3i, size_t, XORVector3iHash> voxels;
```

의미:

```text
coord -> index in flat_voxels
```

즉 역할 분담은 아래와 같다.

- `voxels`
  - 정수 voxel 좌표로 빠르게 검색
- `flat_voxels`
  - 실제 voxel 순회/접근/저장

이 구조는 매우 중요하다. 왜냐하면 unordered_map에 모든 payload를 직접 넣지 않고, 해시는 **인덱스만**, 실데이터는 **연속 배열 비슷한 구조**에 따로 저장해두기 때문이다.

---

### 3.4 LRU 관련 상태

위치:

- `incremental_voxelmap.hpp:124-126`

필드:

- `lru_horizon`
- `lru_clear_cycle`
- `lru_counter`

의미:

- `lru_counter`: 현재 삽입 배치 번호 같은 역할
- `lru_clear_cycle`: 몇 번 insert마다 오래된 voxel 정리할지
- `lru_horizon`: 얼마나 오래 안 쓰인 voxel을 stale로 볼지

즉 이 voxel map은 무한히 커지는 구조가 아니라, **삽입 기반 LRU 정리 정책**을 가진다.

---

## 4. PointCloud가 어떤 형식으로 들어오는가

관련 위치:

- `thirdparty/gtsam_points/include/gtsam_points/types/point_cloud.hpp:103-109`
- `thirdparty/gtsam_points/src/gtsam_points/types/point_cloud_cpu.cpp:123-131`

`PointCloud`의 핵심 필드는 아래다.

```cpp
Eigen::Vector4d* points;   // (x, y, z, 1)
Eigen::Vector4d* normals;  // (nx, ny, nz, 0)
Eigen::Matrix4d* covs;
double* intensities;
```

즉 점 하나는 homogeneous form으로 저장되고, covariance는 `Matrix4d`로 저장되지만 실질적인 3D 공간 공분산은 상단 `3x3` block에 들어간다.

`PointCloudCPU::add_covs()`는 3x3 또는 4x4 covariance를 받아 top-left block에 복사한다.

따라서 voxel 쪽에서 보는 입력은 본질적으로:

- 위치 벡터 `p_i`
- 점별 covariance `Σ_i`
- optional intensity `I_i`

라고 보면 된다.

---

## 5. 좌표 양자화 수학: 점을 voxel 좌표로 보내는 방법

관련 위치:

- `incremental_voxelmap_impl.hpp:34`
- `incremental_voxelmap_impl.hpp:73`
- `thirdparty/gtsam_points/include/gtsam_points/util/fast_floor.hpp:12-14`

핵심 식은 이거다.

```cpp
coord = fast_floor(point * inv_leaf_size).head<3>()
```

여기서:

- `leaf_size = voxel 크기`
- `inv_leaf_size = 1 / leaf_size`

이므로 수학적으로는

```text
coord_x = floor(x / leaf_size)
coord_y = floor(y / leaf_size)
coord_z = floor(z / leaf_size)
```

와 같다.

즉 3차원 연속 공간의 점 `p = (x, y, z)`를 정수 격자 셀 `coord = (i, j, k)`로 보내는 과정이다.

### 5.1 왜 `fast_floor`를 쓰는가

`fast_floor()`는 다음 식으로 동작한다.

```cpp
const Eigen::Array4i ncoord = pt.cast<int>();
return ncoord - (pt < ncoord.cast<double>()).cast<int>();
```

이건 단순 cast가 truncation이라는 점을 보정해서, 음수에서도 진짜 floor와 같은 결과를 주도록 만든 빠른 구현이다.

예를 들어:

- `pt = 1.7` → `floor = 1`
- `pt = -0.2` → `floor = -1`

이 규칙이 삽입과 검색에서 동일하게 쓰이므로, “어느 점이 어느 voxel에 속하는가”의 기준이 일관된다.

---

## 6. `insert()`가 실제로 하는 일

관련 위치:

- `incremental_voxelmap_impl.hpp:31-68`

이 함수가 사실상 이 파일의 핵심이다.

### 6.1 한 줄 요약

`insert(points)`는 입력 점군의 각 점을 voxel 좌표로 양자화하고, voxel이 없으면 새로 만들고, 있으면 기존 voxel에 누적한 뒤, 오래된 voxel을 정리하고, 마지막으로 모든 voxel을 finalize한다.

---

### 6.2 단계 1: 각 점의 voxel 좌표 계산

코드:

```cpp
for (size_t i = 0; i < points.size(); i++) {
  const Eigen::Vector3i coord = fast_floor(points.points[i] * inv_leaf_size).template head<3>();
```

수학적으로는:

```text
v_i = floor(p_i / leaf_size)
```

즉 각 점 `p_i`를 자신이 속할 voxel index `v_i`로 변환한다.

---

### 6.3 단계 2: voxel 검색 또는 생성

코드:

```cpp
auto found = voxels.find(coord);
if (found == voxels.end()) {
  auto voxel = std::make_shared<std::pair<VoxelInfo, VoxelContents>>(VoxelInfo(coord, lru_counter), VoxelContents());
  found = voxels.emplace_hint(found, coord, flat_voxels.size());
  flat_voxels.emplace_back(voxel);
}
```

이 부분이 **실제 voxel 생성 지점**이다.

생성되는 것은 정확히 말하면:

```text
(VoxelInfo(coord, current_lru), empty VoxelContents)
```

이다.

즉 이 구현에서 “voxel 생성”은 다음 세 층이 동시에 일어난다.

1. 공간 좌표 `coord` 결정
2. 메타데이터 `VoxelInfo` 생성
3. 내용 객체 `VoxelContents()` 생성

그리고:

- `voxels[coord] = flat_voxels.size()` 로 hash 등록
- `flat_voxels.push_back(...)` 로 실제 저장

이 된다.

---

### 6.4 단계 3: voxel 내용 업데이트

코드:

```cpp
auto& [info, voxel] = *flat_voxels[found->second];
info.lru = lru_counter;
voxel.add(voxel_setting, points, i);
```

의미:

- `info.lru = lru_counter`
  - 이 insert batch에서 이 voxel이 사용되었음을 기록
- `voxel.add(...)`
  - 실제 점/공분산/intensity 누적

중요한 점은 `IncrementalVoxelMap`이 **VoxelContents의 내부 수학을 모른다**는 것이다.
즉 이 템플릿은:

- voxel 좌표 관리
- 생성/검색/정리/순회

만 담당하고,

- “점 하나를 voxel 내용에 어떻게 반영할지”는 `VoxelContents::add()`에 위임한다.

이 구조 덕분에 같은 `IncrementalVoxelMap`이:

- `GaussianVoxel`
- `FlatContainer`
- `IncrementalCovarianceContainer`

같은 다른 voxel payload 타입에도 재사용될 수 있다.

---

### 6.5 단계 4: LRU 기반 삭제

코드:

```cpp
if ((++lru_counter) % lru_clear_cycle == 0) {
  auto remove_counter = std::remove_if(..., [&](...) {
    return voxel->first.lru + lru_horizon < lru_counter;
  });
  flat_voxels.erase(remove_counter, flat_voxels.end());

  voxels.clear();
  for (size_t i = 0; i < flat_voxels.size(); i++) {
    voxels[flat_voxels[i]->first.coord] = i;
  }
}
```

삭제 조건은 수학적으로 다음과 같다.

```text
voxel is stale if last_used + horizon < current_counter
```

즉 어떤 voxel이 최근 `lru_horizon`번의 insert batch 동안 한 번도 다시 삽입되지 않았다면 제거한다.

### 6.5.1 중요한 구현 포인트

`lru_counter`는 **점마다 증가하지 않고 insert 호출마다 1 증가**한다.

즉 이 구조의 시간 개념은:

- “몇 개의 점을 넣었는가”가 아니라
- “몇 번의 batch 삽입이 지나갔는가”

이다.

### 6.5.2 왜 재해시가 필요한가

`flat_voxels`에서 erase가 일어나면 배열 인덱스가 바뀐다.

그런데 `voxels`는:

```text
coord -> index in flat_voxels
```

이므로, 삭제 뒤에는 반드시 다시 index를 맞춰야 한다. 그래서 `voxels.clear()` 후 전체 재구성을 한다.

---

### 6.6 단계 5: finalize

코드:

```cpp
for (auto& voxel : flat_voxels) {
  voxel->second.finalize();
}
```

이건 매우 중요하다.

이 의미는:

- 새로 touched된 voxel만 finalize하는 것이 아니라
- 현재 남아 있는 모든 voxel에 대해 finalize를 호출한다

는 뜻이다.

즉 `insert()`는 단순 append가 아니라, insert batch 하나를 마칠 때마다 voxel 통계를 “읽기 좋은 상태”로 다시 정리하는 단계까지 포함한다.

---

## 7. `GaussianVoxel`의 수학

관련 위치:

- `thirdparty/gtsam_points/include/gtsam_points/types/gaussian_voxelmap_cpu.hpp:12-52`
- `thirdparty/gtsam_points/src/gtsam_points/types/gaussian_voxelmap_cpu.cpp:23-47`

`GaussianVoxel` 필드:

- `finalized`
- `num_points`
- `mean`
- `cov`
- `intensity`

### 7.1 `add()`가 하는 일

핵심 코드:

```cpp
if (finalized) {
  mean *= num_points;
  cov *= num_points;
  finalized = false;
}

num_points++;
mean += points.points[i];
cov += points.covs[i];
```

즉 finalize된 평균값이 이미 저장되어 있으면, 먼저 합(sum) 상태로 되돌린 다음 새 샘플을 더한다.

수학적으로 보면, voxel 안에 들어온 점 인덱스를 `S`라고 할 때:

```text
mean_sum = Σ p_i
cov_sum  = Σ Σ_i
num_points = |S|
```

를 유지하다가,

`finalize()`에서

```text
mean = (1 / |S|) Σ p_i
cov  = (1 / |S|) Σ Σ_i
```

를 만든다.

### 7.2 중요한 해석

이 구현의 `cov`는 “voxel 안 점들의 scatter covariance”를 직접 계산한 것이 아니다.

즉 여기서는

```text
cov = average of per-point covariances
```

다.

따라서 이 파일 하나만 놓고 보면, voxel covariance는

- 점들 위치의 분산을 새로 계산한 값이 아니라
- 입력 point cloud가 이미 갖고 있던 per-point covariance들을 평균낸 값

이다.

이 차이는 문서에서 꼭 강조할 필요가 있다.

### 7.3 intensity는 평균이 아니다

코드:

```cpp
this->intensity = std::max(this->intensity, frame::intensity(points, i));
```

즉 voxel intensity는 평균 intensity가 아니라 **최대 intensity**다.

---

## 8. `knn_search()`가 하는 일

관련 위치:

- `incremental_voxelmap_impl.hpp:71-92`
- `thirdparty/gtsam_points/include/gtsam_points/ann/knn_result.hpp`

### 8.1 query의 voxel 중심 계산

코드:

```cpp
const Eigen::Vector4d query = (Eigen::Vector4d() << pt[0], pt[1], pt[2], 1.0).finished();
const Eigen::Vector3i center = fast_floor(query * inv_leaf_size).template head<3>();
```

즉 query point도 insert 때와 같은 규칙으로 voxel 좌표로 보낸다.

수학적으로:

```text
center = floor(q / leaf_size)
```

---

### 8.2 전역 인덱스 transform 준비

코드:

```cpp
size_t voxel_index = 0;
const auto index_transform = [&](const size_t point_index) { return calc_index(voxel_index, point_index); };
```

이 transform은 각 voxel 내부 local index를 전역 index로 바꿔준다.

---

### 8.3 `KnnResult`의 역할

`KnnResult`는:

- k개 neighbor 버퍼 유지
- 현재 worst distance보다 나쁜 후보 제거
- 정렬된 상태 삽입
- local index를 global index로 변환

을 담당한다.

`push(index, distance)`의 핵심 의미는 다음과 같다.

```text
if candidate is better than current worst:
  insert into sorted neighbor list
  save transformed global index
```

즉 실제 KNN 전체 조립은 `IncrementalVoxelMap`과 `KnnResult`가 같이 한다.

---

### 8.4 이웃 voxel만 순회

코드:

```cpp
for (const auto& offset : offsets) {
  const Eigen::Vector3i coord = center + offset;
  const auto found = voxels.find(coord);
  if (found == voxels.end()) {
    continue;
  }

  voxel_index = found->second;
  const auto& voxel = flat_voxels[voxel_index]->second;
  voxel.knn_search(query, result);
}
```

즉 이 구현은 전체 voxel map 전체를 다 보지 않는다. 오직:

```text
center voxel + selected neighbor offsets
```

만 본다.

그래서 이 자료구조의 철학은:

- coarse step: voxel quantization으로 공간 범위 축소
- fine step: 인접 voxel 내부에서만 실제 거리 비교

라고 요약할 수 있다.

---

## 9. `neighbor_offsets()`의 의미

관련 위치:

- `incremental_voxelmap_impl.hpp:96-140`

지원 모드:

- `1`: 자기 voxel만
- `7`: 중심 + 6개 face neighbor
- `19`: 3x3x3 중 코너 제외
- `27`: 3x3x3 전체

### 9.1 왜 이런 모드가 필요한가

voxel 하나만 보면 빠르지만, query가 voxel 경계 근처에 있을 때 진짜 최근접점이 옆 voxel에 있을 수 있다.

그래서 탐색 범위를 넓히면 정확도는 올라갈 수 있고, 대신 비용도 증가한다.

즉:

- mode 1: 가장 빠름
- mode 7: 보통의 타협점
- mode 19/27: 더 넓은 탐색

이다.

### 9.2 기본값

생성자:

```cpp
IncrementalVoxelMap<VoxelContents>::IncrementalVoxelMap(double leaf_size)
: ...,
  offsets(neighbor_offsets(7)) {}
```

즉 generic incremental voxel map의 기본 탐색 범위는 7-neighborhood다.

다만 `GaussianVoxelMapCPU` 생성자에서는:

```cpp
offsets = neighbor_offsets(1);
```

를 다시 설정하므로, `GaussianVoxelMapCPU` 쪽은 기본적으로 center voxel만 쓰게 된다.

이 차이도 중요하다.

---

## 10. packed index 구조

관련 위치:

- `incremental_voxelmap.hpp:84-86`
- `incremental_voxelmap.hpp:119-120`

정의:

```cpp
global_index = (voxel_id << 32) | point_id
```

복원:

```cpp
voxel_id(i) = i >> 32
point_id(i) = i & ((1ull << 32) - 1)
```

이 구조 덕분에 외부에서는 “평평한 하나의 index”만 넘겨도 내부적으로는:

- 어느 voxel인지
- 그 voxel 내부 몇 번째 점인지

를 복구할 수 있다.

### 10.1 주의할 점

여기서 `voxel_id`는 공간 좌표 `(i,j,k)`가 아니다.

정확히는:

```text
voxel_id = index in flat_voxels
```

이다.

그래서 LRU 삭제 후 `flat_voxels`가 재정렬되면, 예전 전역 index를 장기적으로 안정적인 식별자로 보면 안 된다.

### 10.2 코드 주석의 미묘한 점

`incremental_voxelmap.hpp`의 `voxel_id()` / `point_id()` 옆 주석은 서로 뒤바뀐 표현처럼 보인다.

실제 동작은:

- `voxel_id()`가 voxel id 추출
- `point_id()`가 point id 추출

이다.

---

## 11. `voxel_data()`는 무엇을 반환하는가

관련 위치:

- `incremental_voxelmap_impl.hpp:195-228`

이 함수는 현재 voxel map의 내용을 `PointCloudCPU`로 평탄화해서 반환한다.

즉 개념적으로는:

```text
for every voxel
  for every representative point inside that voxel
    append point / normal / cov / intensity to flat point cloud
```

이다.

`GaussianVoxel`는 `size() == 1`이므로, Gaussian voxel map에서는 voxel 하나당 대표점 하나가 나온다.

따라서 `voxel_data()`는 “원래 입력 점군 복원”이 아니라 **현재 voxel representation을 point cloud 인터페이스로 노출한 것**이라고 보는 게 맞다.

---

## 12. 실행 흐름을 한 번에 보면

### 12.1 삽입

```text
PointCloud 입력
  -> 각 점 p_i를 floor(p_i / leaf_size)로 정수 voxel 좌표화
  -> coord로 voxels 해시 조회
  -> 없으면 (VoxelInfo, VoxelContents) 생성
  -> 해당 voxel에 voxel.add(...)
  -> 주기적으로 오래된 voxel 삭제
  -> flat_voxels 인덱스 재구성
  -> 모든 voxel finalize()
```

### 12.2 검색

```text
query q 입력
  -> center = floor(q / leaf_size)
  -> offsets에 정의된 neighbor voxel만 순회
  -> 존재하는 voxel에 대해서만 voxel.knn_search(q, result)
  -> local index를 (voxel_id, point_id) packed index로 변환
  -> 최종 KNN 결과 반환
```

---

## 13. 이 구조가 NDT / LightNDT / VGICP와 연결되는 방식

이 incremental voxel map은 직접 최적화 cost를 계산하지는 않는다. 대신 **빠르게 local Gaussian 또는 local representative를 찾는 공간 인덱스** 역할을 한다.

현재 코드 기준으로 downstream 의미는 대략 다음과 같다.

- `NDT`
  - source point를 voxel로 보낸 뒤, 대응 voxel의 mean/cov를 사용
- `LightNDT`
  - 대응 구조는 비슷하지만 cost는 더 단순한 quadratic LSQ 형태
- `VGICP`
  - voxel 단위 대표 공분산/대표점을 이용해 대응 비용 계산

즉 이 파일은 factor가 직접 보는 cost 함수 파일은 아니지만, **그 cost 함수들이 의존하는 target-side spatial/statistical representation을 만드는 기반**이다.

---

## 14. 구현상 꼭 기억해야 할 포인트

1. **실제 voxel 생성 줄은 `incremental_voxelmap_impl.hpp:38`이다.**
   - 여기서 `(VoxelInfo, VoxelContents)`가 만들어진다.

2. **`GaussianVoxelMapCPU::insert()`는 wrapper다.**
   - 진짜 동작은 `IncrementalVoxelMap<GaussianVoxel>::insert()`에 있다.

3. **`cov`는 voxel 내부 위치 scatter covariance가 아니라 point covariance 평균이다.**

4. **intensity는 평균이 아니라 최대값이다.**

5. **LRU는 검색 기준이 아니라 삽입 기준으로 갱신된다.**
   - `knn_search()`는 `lru`를 갱신하지 않는다.

6. **global index는 영구 식별자가 아니다.**
   - `flat_voxels`가 재압축되면 index가 바뀔 수 있다.

7. **generic default neighbor mode는 7이지만, `GaussianVoxelMapCPU`는 1로 재설정한다.**

---

## 15. 핵심 요약

`incremental_voxelmap_impl.hpp`는 이 프로젝트에서 voxel map의 “실제 엔진”이다.

수학적으로는:

- 점을 `floor(p / leaf_size)`로 정수 격자에 보낸다.
- 같은 격자 칸에 들어온 점들을 하나의 voxel payload에 누적한다.
- `GaussianVoxel`의 경우 그 payload는 평균 위치와 평균 공분산을 갖는 local Gaussian이 된다.
- 검색 시에는 전체 공간이 아니라 인접 voxel 집합만 탐색해서 KNN을 수행한다.

구조적으로는:

- `voxels`: 좌표 -> 배열 인덱스
- `flat_voxels`: 실제 voxel payload 배열
- `VoxelInfo`: 메타데이터
- `VoxelContents`: 실제 통계/점 저장

로 분리되어 있어서, 빠른 조회와 다양한 voxel payload 타입 재사용이 가능하다.

---

## 16. 참고 파일

- `thirdparty/gtsam_points/include/gtsam_points/ann/incremental_voxelmap.hpp`
- `thirdparty/gtsam_points/include/gtsam_points/ann/impl/incremental_voxelmap_impl.hpp`
- `thirdparty/gtsam_points/include/gtsam_points/util/fast_floor.hpp`
- `thirdparty/gtsam_points/include/gtsam_points/types/gaussian_voxelmap_cpu.hpp`
- `thirdparty/gtsam_points/src/gtsam_points/types/gaussian_voxelmap_cpu.cpp`
- `thirdparty/gtsam_points/include/gtsam_points/types/point_cloud.hpp`
- `thirdparty/gtsam_points/src/gtsam_points/types/point_cloud_cpu.cpp`
- `thirdparty/gtsam_points/include/gtsam_points/ann/knn_result.hpp`
