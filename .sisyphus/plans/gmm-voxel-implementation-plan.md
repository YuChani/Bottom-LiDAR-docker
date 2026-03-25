# GMM Voxel + MixtureLightNDT: 단계별 구현 계획

## 요약

이 계획은 **GMM (Gaussian Mixture Model) 복셀 시스템**과 이에 대응하는 **MixtureLightNDT 정합 팩터**를 5개 단계에 걸쳐 7개의 새 파일과 3개의 수정 파일로 구현합니다. 각 단계는 명확한 TDD 경계를 가진 원자적 커밋 단위로 설계되었습니다.

**의존성 체인**: Phase 1 → Phase 2 → Phase 2.5 → Phase 3 → Phase 3.5

---

## 프로젝트 파일 구조

새로 생성하는 GMM 관련 파일은 `thirdparty/gtsam_points/` 가 아닌 프로젝트 루트의 `include/gmm/` + `src/gmm/` 에 배치합니다.

### 디렉토리 레이아웃

```
include/
  gmm/
    gmm_voxel.hpp                                ← GMMVoxel 구조체
    gmm_voxelmap_cpu.hpp                         ← GMMVoxelMapCPU 클래스
    mixture_em_backend.hpp                       ← EM Backend (Armadillo)
    integrated_mixture_light_ndt_factor.hpp       ← MixtureLightNDT factor
    impl/
      gmm_voxelmap_cpu_impl.hpp
      integrated_mixture_light_ndt_factor_impl.hpp
src/
  gmm/
    gmm_voxelmap_cpu.cpp
    mixture_em_backend.cpp
    integrated_mixture_light_ndt_factor.cpp
    test/
      test_gmm_voxelmap.cpp
```

### 선택 이유 (vs. `thirdparty/gtsam_points/` 배치)

1. **upstream 분리**: `thirdparty/gtsam_points/` 는 upstream 라이브러리입니다. 여기에 프로젝트 고유 파일을 추가하면 upstream 업데이트 시 merge conflict 가 발생합니다.
2. **프로젝트 고유 코드 구분**: GMM 복셀 시스템은 이 프로젝트만의 확장이므로, thirdparty 와 명확히 분리해야 합니다.
3. **CMake include path 이미 등록**: `CMakeLists.txt` 에 `include_directories(include)` 가 이미 등록되어 있어, `include/gmm/` 하위 파일은 별도 설정 없이 사용 가능합니다.
4. **include 컨벤션**: 프로젝트 소유 헤더는 quoted include `"gmm/foo.hpp"`, thirdparty 헤더는 angle-bracket include `<gtsam_points/...>` 를 사용합니다.

### CMake 등록

새 `.cpp` 소스 파일은 프로젝트 루트 `CMakeLists.txt` 의 `add_executable(lidar_registration_benchmark ...)` 블록에 명시적으로 추가합니다:

```cmake
add_executable(lidar_registration_benchmark
    src/main.cpp
    src/loam_feature.cpp
    src/featureExtraction.cpp
    # GMM Voxel
    src/gmm/gmm_voxelmap_cpu.cpp
    src/gmm/mixture_em_backend.cpp
    src/gmm/integrated_mixture_light_ndt_factor.cpp
)
```

Armadillo 링킹도 프로젝트 루트 `CMakeLists.txt` 에서 처리합니다:

```cmake
find_package(Armadillo REQUIRED)
target_link_libraries(lidar_registration_benchmark
    ...
    ${ARMADILLO_LIBRARIES}
)
target_include_directories(lidar_registration_benchmark PRIVATE ${ARMADILLO_INCLUDE_DIRS})
```

---

## Phase 1: GMMVoxel 구조체 + GMMVoxelMapCPU 셸 (EM 없음)

**목표**: `GMMVoxel` VoxelContents 타입을 정의하고 `add()`에 저수지 샘플링을, `finalize()`에 스텁을 구현한 뒤, `GaussianVoxelMap`을 상속하는 `GMMVoxelMapCPU`에 연결합니다. 컴파일 통과, 기존 테스트 영향 없음.

**복잡도**: 중간  
**의존성**: 없음  
**예상 소요 시간**: ~3시간

### 생성할 파일

#### 1a. `include/gmm/gmm_voxelmap_cpu.hpp`

**핵심 내용:**

```cpp
namespace gtsam_points {

// One Gaussian component of a GMM
struct GMMComponent {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector4d mean = Eigen::Vector4d::Zero();   // homogeneous (w=0)
  Eigen::Matrix4d cov  = Eigen::Matrix4d::Zero();   // 4x4, only 3x3 used
  double weight = 0.0;
};

struct GMMVoxel {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<GMMVoxel>;
  using ConstPtr = std::shared_ptr<const GMMVoxel>;

  struct Setting {
    int max_components = 3;
    int max_em_iterations = 20;
    double convergence_tol = 1e-4;
    double covariance_regularization = 1e-3;
    double min_weight_threshold = 0.01;
    int reservoir_capacity = 256;
  };

  GMMVoxel();

  // VoxelContents contract
  size_t size() const;                                              // returns num_components (K after finalize, 0 before)
  void add(const Setting& setting, const PointCloud& points, size_t i);  // reservoir sampling
  void finalize();                                                  // stub: single Gaussian fallback

  template <typename Result>
  void knn_search(const Eigen::Vector4d& pt, Result& result) const; // search all K components

  // GMM-specific accessors
  const std::vector<GMMComponent>& components() const { return components_; }
  const std::vector<Eigen::Vector4d>& reservoir() const { return reservoir_; }
  bool is_dirty() const { return dirty_; }

private:
  // Reservoir sampling state
  std::vector<Eigen::Vector4d> reservoir_;
  size_t total_points_seen_ = 0;
  bool dirty_ = false;            // true if new points added since last finalize
  std::mt19937 rng_{42};          // per-voxel RNG for reservoir

  // Finalized GMM components
  std::vector<GMMComponent> components_;
  Setting cached_setting_;        // setting snapshot from last finalize
};

// frame::traits specialization — component index exposure
// size() = K, point(v, k) = k-th mean, cov(v, k) = k-th cov
template <>
struct frame::traits<GMMVoxel> {
  static int   size          (const GMMVoxel& v)          { return v.size(); }
  static bool  has_points    (const GMMVoxel& v)          { return v.size() > 0; }
  static bool  has_normals   (const GMMVoxel& v)          { return false; }
  static bool  has_covs      (const GMMVoxel& v)          { return v.size() > 0; }
  static bool  has_intensities(const GMMVoxel& v)         { return false; }
  static Eigen::Vector4d point (const GMMVoxel& v, size_t k) { return v.components()[k].mean; }
  static Eigen::Vector4d normal(const GMMVoxel& v, size_t k) { return Eigen::Vector4d::Zero(); }
  static Eigen::Matrix4d cov   (const GMMVoxel& v, size_t k) { return v.components()[k].cov; }
  static double intensity      (const GMMVoxel& v, size_t k) { return 0.0; }
};

// GMMVoxelMapCPU: sibling of GaussianVoxelMapCPU (NOT a subclass of it)
class GMMVoxelMapCPU
  : public GaussianVoxelMap,
    public IncrementalVoxelMap<GMMVoxel> {
public:
  using Ptr = std::shared_ptr<GMMVoxelMapCPU>;
  using ConstPtr = std::shared_ptr<const GMMVoxelMapCPU>;

  GMMVoxelMapCPU(double resolution);
  virtual ~GMMVoxelMapCPU();

  // GaussianVoxelMap pure virtuals
  virtual double voxel_resolution() const override;
  virtual void insert(const PointCloud& frame) override;
  virtual void save_compact(const std::string& path) const override;

  // GMM-specific access
  GMMVoxel::Setting& gmm_setting() { return voxel_insertion_setting(); }
  Eigen::Vector3i voxel_coord(const Eigen::Vector4d& x) const;
  int lookup_voxel_index(const Eigen::Vector3i& coord) const;
  const GMMVoxel& lookup_voxel(int voxel_id) const;
  size_t num_voxels() const;
};

} // namespace gtsam_points
```

#### 1b. `src/gmm/gmm_voxelmap_cpu.cpp`

**핵심 구현 세부사항:**

- **`GMMVoxel()` 생성자**: `reservoir_` 비어있음, `components_` 비어있음, `dirty_ = false`로 초기화.

- **`GMMVoxel::add()`**: 제한된 저수지 샘플링 (Algorithm R):
  ```
  total_points_seen_++
  if reservoir_.size() < capacity:
      reservoir_.push_back(points.points[i])
  else:
      j = uniform_int(0, total_points_seen_ - 1)
      if j < capacity:
          reservoir_[j] = points.points[i]
  dirty_ = true
  ```
  참고: (`GaussianVoxel::add`와 달리) `points.covs`가 필요하지 않습니다. 원시 포인트만 사용합니다.

- **`GMMVoxel::finalize()` (Phase 1 스텁)**: `!dirty_`이면 반환. `reservoir_.size() < 3`이면 저수지 포인트의 평균/공분산으로 단일 컴포넌트 GMM을 생성합니다. `dirty_ = false`로 설정. 이것은 임시 폴백이며, Phase 2.5에서 실제 EM으로 교체됩니다.

- **`GMMVoxel::knn_search()`**: 모든 `components_`를 순회하며 `sq_dist = (pt - comp.mean).squaredNorm()`을 계산하고, 각 컴포넌트 인덱스 `k`에 대해 `result.push(k, sq_dist)`를 호출합니다.

- **`GMMVoxelMapCPU` 생성자**: `IncrementalVoxelMap<GMMVoxel>(resolution)`을 호출한 후, `offsets = neighbor_offsets(1)` (`GaussianVoxelMapCPU`와 동일).

- **`save_compact()`**: Phase 1 스텁, "save_compact not yet implemented for GMMVoxelMapCPU" 경고를 로깅합니다.

- **위임 메서드**: `voxel_resolution()` → `leaf_size()`, `insert()` → `IncrementalVoxelMap::insert()`, `voxel_coord/lookup_voxel_index/lookup_voxel/num_voxels`는 `GaussianVoxelMapCPU`와 동일한 패턴입니다.

### 수정할 파일

#### 1c. `CMakeLists.txt` (프로젝트 루트)
- `add_executable(lidar_registration_benchmark ...)` 블록에 `src/gmm/gmm_voxelmap_cpu.cpp`를 추가합니다.
- Armadillo 링킹은 아직 필요 없습니다 (Phase 1에서는 EM을 사용하지 않음).

### 테스트 전략 (TDD)

#### 1d. `src/gmm/test/test_gmm_voxelmap.cpp` (새 파일)

구현 **전에** 작성할 테스트:

```
TEST(GMMVoxel, DefaultConstructEmpty)
  — size() == 0, components() 비어있음, reservoir() 비어있음

TEST(GMMVoxel, ReservoirAccumulates)
  — capacity=10으로 Setting 생성, 5개 포인트 추가 → reservoir.size() == 5
  — 20개 더 추가 → reservoir.size() == 10 (상한 적용)

TEST(GMMVoxel, FinalizeStubSingleComponent)
  — 알려진 분포에서 10개 포인트 추가, finalize()
  — size() == 1, component[0].weight ≈ 1.0
  — component[0].mean ≈ 10개 포인트의 실제 평균

TEST(GMMVoxel, DirtyFlagBehavior)
  — add()가 dirty=true로 설정, finalize()가 dirty=false로 설정
  — add() 없이 두 번째 finalize()는 no-op

TEST(GMMVoxelMapCPU, InstantiatesAndInserts)
  — GMMVoxelMapCPU(1.0) 생성, KITTI 프레임 삽입
  — num_voxels() > 0
  — dynamic_pointer_cast<GaussianVoxelMap> 성공

TEST(GMMVoxelMapCPU, IncrementalVoxelMapInstantiates)
  — IncrementalVoxelMap<GMMVoxel> 템플릿 컴파일 및 동작 확인
  — knn_search가 유효한 인덱스를 반환

TEST(GMMVoxelMapCPU, ExistingFactorsUnaffected)
  — GaussianVoxelMapCPU가 여전히 동작하는지 확인 (회귀 방지)
```

### 검증 기준
- [x] `cmake --build .`가 새로운 경고 없이 성공
- [x] 모든 기존 테스트 통과 (`test_voxelmap`, `test_matching_cost_factors`)
- [x] 새로운 `test_gmm_voxelmap` 테스트 전부 통과
- [x] `IncrementalVoxelMap<GMMVoxel>` 인스턴스화 및 데이터 삽입 성공

### 수학적 기초

#### 1. Algorithm R (Vitter, 1985), 균일 저수지 샘플링

**주장**: 스트림에서 n개 항목을 본 후, 각 항목은 용량 C인 저수지에 정확히 $\min(C, n)/n$의 확률로 존재합니다.

**n에 대한 귀납법 증명**:

*기저 사례*: $n \le C$일 때 모든 항목이 저장됩니다. 각각 $C/C = 1 = \min(C,n)/n$의 확률을 가집니다 ✓ ($\min(C,n) = n$이므로 자명).

*귀납 단계*: n개 항목 후($n \ge C$) 저수지의 각 항목이 $C/n$ 확률을 가진다고 가정합니다. 항목 $n+1$이 도착하면:
- 항목 $n+1$은 $C/(n+1)$ 확률로 포함됩니다 ($j \in \{0, \ldots, n\}$을 균등하게 생성하고, $j < C$일 때만 포함).
- 포함되면, 균일하게 무작위 선택된 저수지 슬롯을 교체합니다. 기존 항목은 $(1 - 1/C)$ 확률로 생존합니다.
- 이전에 저장된 항목의 경우, 단계 $n+1$ 이후 저수지에 있을 확률:

$$
P(\text{survive}) = P(\text{in reservoir after } n) \times P(\text{not evicted})
$$

$$
= \frac{C}{n} \times \left[1 - \frac{C}{n+1} \times \frac{1}{C}\right]
= \frac{C}{n} \times \left[1 - \frac{1}{n+1}\right]
= \frac{C}{n} \times \frac{n}{n+1}
= \frac{C}{n+1} \checkmark
$$

기존 항목과 새 항목 모두 $C/(n+1) = \min(C, n+1)/(n+1)$의 확률을 가집니다. 귀납법에 의해 불변량은 모든 n에 대해 성립합니다. $\blacksquare$

**구현 참고**: 복셀 단위 `std::mt19937 rng_{42}`가 균일 랜덤 인덱스 j를 생성합니다. 결정적 시딩은 디버깅 재현성을 보장하지만, 프로덕션에서는 `std::random_device` 시딩을 고려할 수 있습니다.

#### 2. 충분통계량 대신 저수지를 사용하는 이유

**단일 가우시안**의 경우, 충분통계량은 $\{\sum x_i, \sum x_i \cdot x_i^T, N\}$이며 점진적으로 누적할 수 있습니다. `GaussianVoxel`이 정확히 이 방식을 사용합니다: Welford 방법을 통한 누적 평균 및 공분산.

**K-컴포넌트 GMM**의 경우, 충분통계량은:
- 컴포넌트별: $\{\sum_i \gamma_{ik} \cdot x_i,\; \sum_i \gamma_{ik} \cdot x_i \cdot x_i^T,\; \sum_i \gamma_{ik}\}$, $k=1 \ldots K$

이것은 **책임도(responsibilities)** $\gamma_{ik} = P(z_i=k \mid x_i, \theta)$를 필요로 하며, 이는 현재 모델 파라미터 $\theta$에 의존합니다. 그런데 $\theta$가 바로 우리가 추정하려는 것이므로, 닭과 달걀 문제가 발생합니다.

따라서 GMM 충분통계량을 점진적으로 누적할 수 없습니다. 원시 포인트를 보존하고 EM 배치 피팅을 수행해야 합니다. 저수지 샘플링은 스트리밍으로 들어오는 포인트 수에 관계없이 복셀당 메모리를 $O(C)$로 제한합니다.

**메모리 한계**: $C=256$이고 `sizeof(Eigen::Vector4d)=32` 바이트일 때, 각 복셀은 저수지에 최대 $256 \times 32 = 8$ KB를 사용합니다. 10,000개 복셀의 경우: ~80 MB 총량으로, 허용 가능한 수준입니다.

#### 3. 스텁 finalize() 수학, 단일 컴포넌트 폴백

저수지에 N개 포인트 $\{x_1, \ldots, x_N\}$이 있을 때, 단일 컴포넌트 폴백은 다음을 계산합니다:

- **평균**:

$$
\mu = \frac{1}{N} \sum_{i=1}^{N} x_i
$$

- **공분산**:

$$
\Sigma = \frac{1}{N} \sum_{i=1}^{N} (x_i - \mu)(x_i - \mu)^T + \varepsilon I
$$

**$\varepsilon$-정규화가 필요한 이유**: 표본 공분산 행렬의 랭크는 $\min(N-1, d)$이며, 여기서 $d$는 차원(3D 포인트의 경우 $d=3$)입니다. $N \le d$ (즉, 3D에서 $N \le 3$)일 때 행렬은 랭크 결핍입니다:
- $N=1$: 랭크 0 ($\Sigma = 0$)
- $N=2$: 랭크 1 (포인트가 직선을 정의, 2개 방향에서 공분산 붕괴)
- $N=3$: 랭크 2 (포인트가 평면을 정의, 법선 방향에서 공분산 붕괴)

$\varepsilon I$를 추가하면 모든 고유값이 $\lambda_i \ge \varepsilon > 0$이 되어 $\Sigma$가 양의 정부호이고 가역이 됩니다. 이는 NDT 팩터의 마할라노비스 거리 계산에 필수입니다. $\varepsilon$ = `covariance_regularization` (기본값 1e-3)을 사용합니다.

### 위험 요소
- **`frame::traits` 반환 타입**: `GaussianVoxel` traits는 `const Eigen::Vector4d&` (멤버에 대한 참조)를 반환합니다. `GMMVoxel` traits는 컴포넌트 벡터에 인덱싱될 수 있으므로 **값으로** 반환해야 합니다. `frame::point()` 자유 함수가 `decltype(auto)`를 사용하여 두 경우 모두 처리하므로 문제없습니다.
- **저수지 RNG 스레드 안전성**: `add()`는 `IncrementalVoxelMap::insert()` 내부에서 포인트별로 호출되며, 포인트를 순차적으로 순회합니다 (각 포인트는 하나의 복셀에 매핑되고, 같은 복셀의 `add()`가 동시에 호출되지 않음). 따라서 복셀별 `mt19937`은 안전합니다.

---

## Phase 2: EM 백엔드 (Armadillo `gmm_full`)

**목표**: 3D 포인트 버퍼를 받아 GMM 컴포넌트를 생성하는 독립 EM 피팅 모듈을 구현합니다. 합성 데이터로 격리 테스트합니다.

**복잡도**: 중간  
**의존성**: Phase 1 (`GMMComponent` 구조체 필요)  
**예상 소요 시간**: ~2시간

### 생성할 파일

#### 2a. `include/gmm/mixture_em_backend.hpp`

```cpp
namespace gtsam_points {

struct GMMFitResult {
  std::vector<GMMComponent> components;
  bool converged = false;
  int iterations_run = 0;
};

struct GMMFitParams {
  int max_components = 3;
  int max_em_iterations = 20;
  double convergence_tol = 1e-4;
  double covariance_regularization = 1e-3;
  double min_weight_threshold = 0.01;
};

// Fit a GMM to 3D points using Armadillo's gmm_full
// Returns empty result if N < 2
GMMFitResult fit_gmm(
    const std::vector<Eigen::Vector4d>& points,  // homogeneous 4D (w=0)
    const GMMFitParams& params);

// Warm-start version: uses previous components as initialization
GMMFitResult fit_gmm(
    const std::vector<Eigen::Vector4d>& points,
    const GMMFitParams& params,
    const std::vector<GMMComponent>& initial_components);

} // namespace gtsam_points
```

#### 2b. `src/gmm/mixture_em_backend.cpp`

**핵심 구현 세부사항:**

- **데이터 레이아웃 변환**: `std::vector<Eigen::Vector4d>` → `arma::mat(3, N)`, 각 Vector4d에서 `.head<3>()`을 추출하여 열로 패킹.

- **K 선택**: `K_actual = min(params.max_components, N)`. $N < 2$이면 단일 컴포넌트 폴백(모든 포인트의 평균 + 공분산만) 반환.

- **`arma::gmm_full` 호출**:
  ```cpp
  arma::gmm_full model;
  bool ok = model.learn(data, K_actual,
      arma::eucl_dist, arma::random_spread,
      /*km_iter=*/20, /*em_iter=*/params.max_em_iterations,
      /*var_floor=*/1e-6, /*print_mode=*/false);
  ```

- **후처리 루프** (각 컴포넌트 k = 0..K-1에 대해):
  - 평균 추출: `model.means.col(k)` → `Eigen::Vector4d(x, y, z, 0)`
  - 공분산 추출: `model.fcovs.slice(k)` → `Eigen::Matrix4d` (3x3을 좌상단에 복사, 나머지는 0)
  - **정규화**: `cov3x3 += params.covariance_regularization * I3` (Armadillo의 `var_floor`는 개별 대각 요소만 클램핑하므로, 거의 평면인 클러스터에는 균일한 $\varepsilon \cdot I$가 필요)
  - 가중치 추출: `model.hefts(k)`

- **미소 가중치 제거**: `weight < params.min_weight_threshold`인 컴포넌트 제거. 나머지 가중치를 합 = 1로 재정규화.

- **EM 실패 시 폴백** (`ok == false`): 표본 평균/공분산으로 단일 컴포넌트 반환.

- **웜 스타트 변형**: `learn()` 호출 전에 `model.means`, `model.fcovs`, `model.hefts`를 설정하고 `arma::random_spread` 대신 `arma::keep_existing`을 사용.

### 수정할 파일

#### 2c. `CMakeLists.txt` (프로젝트 루트)
- `add_executable()` 블록에 `src/gmm/mixture_em_backend.cpp` 추가
- Armadillo 링킹 추가:
  ```cmake
  find_package(Armadillo REQUIRED)
  target_link_libraries(gtsam_points PRIVATE ${ARMADILLO_LIBRARIES})
  target_include_directories(gtsam_points PRIVATE ${ARMADILLO_INCLUDE_DIRS})
  ```

### 테스트 전략 (TDD)

#### 2d. `test_gmm_voxelmap.cpp`의 테스트 (Phase 1의 기존 파일에 추가)

```
TEST(GMMFit, SingleCluster)
  — N(mu=[1,2,3], sigma=0.5*I)에서 200개 포인트 생성
  — max_components=3으로 fit_gmm
  — 결과: 1개 컴포넌트 (나머지 제거됨), mean ≈ [1,2,3], cov ≈ 0.25*I

TEST(GMMFit, BimodalSeparation)
  — N([0,0,0], 0.1*I)에서 100개 + N([5,0,0], 0.1*I)에서 100개 포인트 생성
  — max_components=3으로 fit_gmm
  — 결과: 2개 컴포넌트, means ≈ {[0,0,0], [5,0,0]}, weights ≈ {0.5, 0.5}

TEST(GMMFit, TrimodalSeparation)
  — 잘 분리된 3개 클러스터
  — 결과: 3개 컴포넌트, weights ≈ 1/3씩

TEST(GMMFit, TooFewPointsFallback)
  — 1개 포인트 → 단일 컴포넌트, cov = regularization * I

TEST(GMMFit, CovariancePD)
  — 모든 결과에 대해, 각 컴포넌트의 3x3 공분산의 모든 고유값 > 0 확인

TEST(GMMFit, WeightSumToOne)
  — 모든 컴포넌트 가중치의 합 ≈ 1.0 (1e-6 이내)

TEST(GMMFit, WarmStartConvergesFaster)
  — 한 번 피팅 후, 웜 스타트로 다시 피팅
  — 웜 스타트가 더 적은 반복 횟수를 사용해야 함 (또는 최소한 발산하지 않아야 함)
```

### 검증 기준
- [x] `find_package(Armadillo)`가 CMake에서 성공
- [x] 모든 합성 GMM 테스트 통과
- [x] 이봉 데이터가 올바른 평균으로 2개 컴포넌트 생성 (0.3 허용 오차 이내)
- [x] 모든 공분산 행렬이 양의 정부호
- [x] 가중치 합 ≈ 1.0

### 수학적 기초

#### 1. GMM 밀도 정의

K-컴포넌트 가우시안 혼합 모델은 다음의 확률 밀도를 정의합니다:

$$
p(x|\theta) = \sum_{k=1}^{K} \pi_k \cdot \mathcal{N}(x | \mu_k, \Sigma_k)
$$

여기서 $\theta = \{\pi_k, \mu_k, \Sigma_k\}_{k=1}^{K}$이고, 다변량 가우시안 밀도는:

$$
\mathcal{N}(x|\mu,\Sigma) = (2\pi)^{-d/2} |\Sigma|^{-1/2} \exp\left(-\frac{1}{2}(x-\mu)^T \Sigma^{-1} (x-\mu)\right)
$$

혼합 가중치는 다음을 만족합니다: $\pi_k \ge 0$, $\sum_{k=1}^{K} \pi_k = 1$.

#### 2. 로그 우도

관측된 N개 포인트 $\{x_1, \ldots, x_N\}$이 주어졌을 때, 로그 우도는:

$$
L(\theta) = \sum_{i=1}^{N} \log\left[ \sum_{k=1}^{K} \pi_k \cdot \mathcal{N}(x_i | \mu_k, \Sigma_k) \right]
$$

$L(\theta)$의 직접 최대화는 합의 로그에 닫힌 형태의 최적값이 없으므로 다루기 어렵습니다. EM은 잠재 변수를 도입하여 이를 해결합니다.

#### 3. E-단계 유도

각 포인트 $x_i$에 대해 잠재 지시자 $z_i \in \{1,\ldots,K\}$를 도입합니다. $z_i = k$는 "포인트 i가 컴포넌트 k에 의해 생성되었음"을 의미합니다.

**베이즈 정리**에 의해:

$$
\gamma_{ik} = P(z_i = k | x_i, \theta)
= \frac{\mathcal{N}(x_i | \mu_k, \Sigma_k) \cdot \pi_k}{\sum_{j=1}^{K} \mathcal{N}(x_i | \mu_j, \Sigma_j) \cdot \pi_j}
$$

이것은 **책임도(responsibility)**이며, 컴포넌트 k가 포인트 $x_i$를 생성했을 사후 확률입니다. 각 $\gamma_{ik} \in [0,1]$이고 각 i에 대해 $\sum_k \gamma_{ik} = 1$입니다.

**수치 안정성**: $\mathcal{N}(x_i|\mu_k,\Sigma_k)$를 직접 계산하면 큰 마할라노비스 거리에서 언더플로가 발생합니다. 구현은 로그 공간을 사용합니다: $\log \mathcal{N}(x_i|\mu_k,\Sigma_k) = -(d/2)\log(2\pi) - \frac{1}{2} \log|\Sigma_k| - \frac{1}{2} d_{ik}^2$를 계산한 다음, 분모에 log-sum-exp 트릭을 적용합니다.

#### 4. M-단계 유도

EM은 **기대 완전 데이터 로그 우도** (Q-함수)를 최대화합니다:

$$
Q(\theta|\theta^{\text{old}}) = \sum_{i=1}^{N} \sum_{k=1}^{K} \gamma_{ik} \left[\log \pi_k + \log \mathcal{N}(x_i|\mu_k, \Sigma_k)\right]
$$

편미분을 취하고 0으로 놓으면:

**유효 카운트**: $N_k = \sum_{i=1}^{N} \gamma_{ik}$ (컴포넌트 k에 할당된 포인트의 소프트 카운트)

**가중치** ($\sum \pi_k = 1$에 대한 라그랑주 승수를 통해):

$$
\frac{\partial}{\partial \pi_k} \left[Q + \lambda\left(\sum_k \pi_k - 1\right)\right] = \frac{\sum_i \gamma_{ik}}{\pi_k} + \lambda = 0
$$

$\Rightarrow \pi_k = -\sum_i \gamma_{ik}/\lambda$. k에 대해 합산: $\sum_k \pi_k = -N/\lambda = 1$이므로 $\lambda = -N$.

$$
\Rightarrow \pi_k = \frac{N_k}{N}
$$

**평균** ($\partial Q/\partial \mu_k = 0$으로 설정):

$$
\frac{\partial Q}{\partial \mu_k} = \sum_i \gamma_{ik} \cdot \Sigma_k^{-1}(x_i - \mu_k) = 0
$$

$$
\Rightarrow \mu_k = \frac{1}{N_k} \sum_{i=1}^{N} \gamma_{ik} x_i
$$

**공분산** ($\partial Q/\partial \Sigma_k = 0$으로 설정, $\partial/\partial \Sigma^{-1} [-\frac{1}{2} \log|\Sigma| - \frac{1}{2} x^T \Sigma^{-1} x]$ 사용):

$$
\Sigma_k = \frac{1}{N_k} \sum_{i=1}^{N} \gamma_{ik} (x_i - \mu_k)(x_i - \mu_k)^T
$$

#### 5. 단조 수렴 보장

**정리** (Dempster, Laird, Rubin 1977): EM은 $L(\theta^{t+1}) \ge L(\theta^{t})$를 보장합니다.

*증명 개요*: 오목 로그 함수에 젠센 부등식을 적용하면:

$$
L(\theta) - L(\theta^{\text{old}}) = \sum_i \log\left[\sum_k \gamma_{ik} \cdot \frac{\pi_k \mathcal{N}(x_i|\mu_k,\Sigma_k)}{\pi_k^{\text{old}} \mathcal{N}(x_i|\mu_k^{\text{old}},\Sigma_k^{\text{old}})}\right]
$$

$$
\ge \sum_i \sum_k \gamma_{ik} \log\left[\frac{\pi_k \mathcal{N}(x_i|\mu_k,\Sigma_k)}{\pi_k^{\text{old}} \mathcal{N}(x_i|\mu_k^{\text{old}},\Sigma_k^{\text{old}})}\right]
$$

M-단계가 우변을 최대화하여 $\ge 0$임을 보장합니다. 따라서 $L$은 비감소입니다. $L$이 위로 유계이므로 (잘 조건화된 $\Sigma_k$에 대해), EM은 지역 최대값 또는 안장점으로 수렴합니다.

**주의**: EM은 *지역* 최적해로만 수렴합니다. 우리 사용 사례(3개 이하 컴포넌트, 256개 이하 포인트)에서는 지역 최적해가 보통 충분하며, 특히 웜 스타트 초기화와 함께 사용할 때 그렇습니다.

#### 6. 수렴 기준

상대 로그 우도 변화가 허용 오차 아래로 떨어지면 수렴을 선언합니다:

$$
\frac{|L(\theta^{t+1}) - L(\theta^{t})|}{|L(\theta^{t})|} < \varepsilon
$$

기본 $\varepsilon = 10^{-4}$. 또한 `max_em_iterations` = 20을 하드 스톱으로 설정합니다. 실제로 대부분의 피팅은 잘 분리된 클러스터에 대해 5~15회 반복에서 수렴합니다.

#### 7. 웜 스타트 이론

이전 컴포넌트 $\theta^{\text{old}}$를 사용할 수 있는 경우 (이전 `finalize()` 호출에서), 무작위 또는 K-means 초기화 대신 $\theta^{\text{old}}$로 EM을 초기화합니다.

**작동 원리**: 저수지가 $\delta$ 비율의 포인트만큼 변경되었다면 (이전 포인트가 새 포인트로 교체), 진정한 기저 혼합은 파라미터 공간에서 최대 $O(\delta)$만큼 이동합니다 (매끄러운 밀도 변화 가정). EM은 파라미터 공간에서 **수렴 영역(basin of attraction)** 내의 지역 최적값으로 수렴하며, $\delta$가 작을 때 $\theta^{\text{old}}$에서 시작하면 같은 영역 내에 머물게 됩니다.

**경험적 이점**: 웜 스타트는 일반적으로 반복 횟수를 15~20회(콜드 스타트)에서 3~8회로 줄여, 50~80% 감소를 보입니다. 이는 프레임 간에 소수의 저수지 포인트만 변경되는 점진적 복셀 업데이트에서 매우 중요합니다.

#### 8. Armadillo `gmm_full::learn()` 매핑

Armadillo의 `arma::gmm_full`은 위에서 설명한 전체 공분산 EM 알고리즘을 구현합니다. 주요 파라미터 매핑:

| 우리의 개념 | Armadillo API | 참고 |
|---|---|---|
| 입력 데이터 | `arma::mat(3, N)` column-major | 각 `Vector4d`에서 `.head<3>()` 추출 |
| K-means 초기화 | `eucl_dist` 플래그 | 초기 K-means 클러스터링에 유클리드 거리 사용 |
| 마할라노비스 초기화 | `maha_dist` 플래그 | 마할라노비스 거리 사용; 클러스터 스케일이 다를 때 더 좋지만 초기 공분산 추정 필요 (더 느림) |
| K-means 반복 | `km_iter=20` | 컴포넌트 평균 시딩을 위한 EM 전 클러스터링 |
| EM 반복 | `em_iter=max_em_iterations` | 우리의 convergence_tol은 Armadillo 내부 기준에 매핑 |
| 대각 하한 | `var_floor=1e-6` | 개별 대각 요소만 클램핑 (§9 참조) |
| 웜 스타트 | `keep_existing` 플래그 | K-means를 건너뛰고 사전 설정된 `model.means/fcovs/hefts`에서 EM 시작 |

`maha_dist`가 아닌 `eucl_dist`를 사용하는 이유: (a) 저수지 샘플링 후 클러스터가 대략 등방적이므로, (b) `eucl_dist`가 더 빠르고 초기화에 수치적으로 더 안정적이며, (c) EM이 후속 반복에서 초기화 편향을 교정합니다.

#### 9. 공분산 정규화

**문제**: Armadillo의 `var_floor`는 각 대각 요소를 개별적으로 클램핑합니다: $\Sigma_{ii} \leftarrow \max(\Sigma_{ii}, \text{var\_floor})$. 이것은 거의 평면인 클러스터에 **불충분**합니다:
- 평면 위의 포인트를 생각해봅시다: $\Sigma$는 고유값 $(\lambda_1, \lambda_2, \varepsilon_{\text{small}})$을 가지며 $\varepsilon_{\text{small}} \approx 0$입니다.
- 대각 요소들은 모두 var_floor보다 클 수 있지만 (평면이 축 정렬이 아니므로), 최소 고유값은 여전히 $\approx 0$입니다.
- 결과: $\Sigma^{-1}$가 거대한 고유값을 가짐 → 마할라노비스 거리 폭발 → 수치 불안정성.

**우리의 해결책**: Armadillo 반환 후 다음을 적용합니다:

$$
\Sigma_k \leftarrow \Sigma_k + \varepsilon \cdot I_3
$$

이것은 $\lambda_{\min}(\Sigma_k) \ge \varepsilon > 0$을 보장합니다. 원래 $\Sigma_k$의 임의의 고유값 $\lambda$에 대해, 정규화된 고유값은 $\lambda + \varepsilon \ge \varepsilon$이기 때문입니다.

**$\varepsilon$ 선택**: `covariance_regularization` = 1e-3 (기본값)을 사용합니다. 이는 임의의 방향에서 최소 표준편차가 $\sqrt{10^{-3}} \approx 3.2$ cm임을 의미하며, 센서 노이즈가 ~2~5 cm인 LiDAR 데이터에 물리적으로 타당합니다.

#### 10. 컴포넌트 가지치기

EM 수렴 후, 무시할 수 있는 가중치를 가진 컴포넌트를 제거합니다:

1. $\pi_k < \tau$인 컴포넌트 k 제거 (기본 $\tau$ = `min_weight_threshold` = 0.01, 즉 포인트의 < 1%)
2. 재정규화: $\pi_k' = \pi_k / \sum_{j \in \text{kept}} \pi_j$

**수학적 정당화**: $\pi_k = 0.01$인 컴포넌트는 임의의 점에서 혼합 밀도에 최대 1%를 기여합니다. 이를 제거하면 밀도가 최대 다음만큼 변합니다:
- 절대 변화: $|p(x|\theta) - p(x|\theta')| \le \pi_k \cdot \max_x \mathcal{N}(x|\mu_k,\Sigma_k) = \tau \cdot (2\pi)^{-d/2} |\Sigma_k|^{-1/2}$
- 전형적인 $\Sigma_k$ ($\sigma \approx 0.1$m)의 경우: 최대 밀도 $\approx 25\;\text{m}^{-3}$이므로 절대 변화 $\le 0.25\;\text{m}^{-3}$으로 무시 가능.

재정규화는 혼합 속성(가중치 합 = 1)을 보존하고, 가지치기된 모델이 유효한 확률 분포로 유지되도록 합니다.

### 위험 요소
- **Armadillo `learn()` 확률적**: 무작위 초기화로 인해 실행마다 결과가 달라질 수 있습니다. 테스트는 넉넉한 허용 오차(예: 평균 0.5 이내, 가중치 0.15 이내)를 사용하거나 `arma::arma_rng::set_seed(42)`로 RNG 시드를 고정해야 합니다.
- **`var_floor` 동작**: Armadillo의 `var_floor`는 개별 대각 요소만 하한을 적용하며 최소 고유값에는 적용하지 않습니다. 평면/선형 클러스터의 경우 비대각 요소로 인해 공분산이 여전히 거의 특이할 수 있습니다. 명시적 `+= epsilon*I` 정규화가 이를 처리합니다.
- **Armadillo Eigen 상호 운용**: 데이터 복사가 필요합니다 (Eigen과 Armadillo 메모리 레이아웃 간 제로 카피 불가). 복셀당 256포인트에서는 이 비용이 무시할 수 있습니다.

---

## Phase 2.5: EM을 GMMVoxel::finalize()에 연결

**목표**: 스텁 `finalize()`를 실제 EM으로 교체합니다. 전체 파이프라인을 검증합니다: `포인트 삽입 → 저수지 → EM → 컴포넌트`.

**복잡도**: 소규모  
**의존성**: Phase 1 + Phase 2  
**예상 소요 시간**: ~1시간

### 수정할 파일

#### 2.5a. `src/gmm/gmm_voxelmap_cpu.cpp`

`GMMVoxel::finalize()` 스텁을 다음으로 교체:

```cpp
void GMMVoxel::finalize() {
  if (!dirty_) return;

  // Convert Setting → GMMFitParams
  GMMFitParams params;
  params.max_components = cached_setting_.max_components;
  params.max_em_iterations = cached_setting_.max_em_iterations;
  params.convergence_tol = cached_setting_.convergence_tol;
  params.covariance_regularization = cached_setting_.covariance_regularization;
  params.min_weight_threshold = cached_setting_.min_weight_threshold;

  GMMFitResult result;
  if (components_.empty()) {
    result = fit_gmm(reservoir_, params);                    // cold start
  } else {
    result = fit_gmm(reservoir_, params, components_);       // warm start (incremental)
  }

  components_ = std::move(result.components);
  dirty_ = false;
}
```

또한 `GMMVoxel::add()`에 추가:
```cpp
cached_setting_ = setting;  // snapshot for finalize()
```

#### 2.5b. `include/gmm/gmm_voxelmap_cpu.hpp`
- `#include "gmm/mixture_em_backend.hpp"` 추가

### 테스트 전략 (TDD)

```
TEST(GMMVoxel, FinalizeProducesValidGMM)
  — Setting{max_components=3, ...} 생성
  — 200개 포인트 추가: 클러스터 A에서 100개, 클러스터 B에서 100개
  — finalize()
  — size() == 2 (노이즈가 세 번째를 만들 경우 3일 수도 있음)
  — 모든 컴포넌트 가중치 > 0, 합 ≈ 1.0
  — 모든 컴포넌트 공분산이 양의 정부호

TEST(GMMVoxelMapCPU, InsertAndFinalizeRealData)
  — GMMVoxelMapCPU(1.0) 생성, max_components=3 설정
  — KITTI 프레임 삽입
  — 샘플링된 복셀에 대해: size()가 {1,2,3} 중 하나, 가중치 합 ≈ 1.0, 공분산 양의 정부호

TEST(GMMVoxel, IncrementalWarmStart)
  — 클러스터 A에서 100개 포인트 추가, finalize() → 1개 컴포넌트
  — 클러스터 B에서 100개 포인트 추가, finalize() → 2개 컴포넌트
  — 웜 스타트 사용 확인 (두 번째 finalize 전에 components_가 비어있지 않았음)
```

### 검증 기준
- [x] `GMMVoxelMapCPU` 삽입 + finalize가 유효한 컴포넌트 생성
- [x] 모든 복셀에서 가중치 합 ≈ 1.0
- [x] 모든 공분산 행렬이 양의 고유값을 가짐
- [x] 점진적 웜 스타트가 품질을 저하시키지 않음
- [x] 기존 테스트 여전히 통과

### 수학적 기초

#### 1. Dirty 플래그 의미론, 분포 이동 감지

`dirty_` 플래그는 경량 **분포 이동 감지기**를 구현합니다. 형식적으로:

시점 t에서의 저수지 상태를 $R_t$, 피팅된 GMM 파라미터를 $\theta_t = \text{EM}(R_t)$라 합시다. 새 포인트가 도착하면 저수지는 $R_{t+1} = \text{ReservoirSample}(R_t \cup \text{new\_points})$가 됩니다.

이전 파라미터 $\theta_t$는 $R_{t+1} = R_t$ (변화 없음)인 경우에만 유효합니다. `add()`가 저수지를 수정하면 (포인트 하나만 교체되더라도), $\theta_t$는 더 이상 $R_{t+1}$에 대한 지역 최적값이 아닐 수 있습니다. dirty 플래그가 이를 추적합니다:

- `add()`가 저수지 수정 → `dirty_ = true` ($\theta$가 오래됨)
- `finalize()`가 EM 실행 → `dirty_ = false` ($\theta$가 새로움)
- `!dirty_`일 때 `finalize()` → no-op ($\theta$가 여전히 유효, 비용이 큰 EM 건너뜀)

이를 통해 `insert()`가 호출되었지만 이 복셀에 포인트가 들어오지 않은 경우나, 중간에 삽입 없이 `finalize()`가 여러 번 호출된 경우의 불필요한 EM 계산을 방지합니다.

#### 2. 웜 스타트 수렴 분석

$R' = R_{t+1}$을 새 저수지라 하고, $R_t$와 $\delta$ 비율의 포인트만큼 다르다 합시다 ($\delta = |R' \triangle R_t| / |R_t|$, 여기서 $\triangle$은 대칭 차집합).

**KL 발산 한계**: $p_t(x)$와 $p_{t+1}(x)$가 $R_t$와 $R'$에 대한 진정한 혼합 밀도라면:

$$
\text{KL}(p_{t+1} \| p_t) = O(\delta)
$$

$\delta$가 작을 때 성립합니다. 이는 혼합 밀도가 충분통계량의 매끄러운 함수이고, $\delta$ 비율의 포인트를 변경하면 경험적 충분통계량이 최대 $O(\delta)$만큼 이동하기 때문입니다.

**수렴 영역**: EM은 파라미터 공간에서 "수렴 영역" 내의 가장 가까운 지역 최대값으로 수렴합니다. $\theta^{\text{old}} = \theta_t$가 이미 $R_t$에 대한 최적값 근처에 있고, $R'$가 $R_t$에 가까우면 ($\delta$ 작음), $R'$에 대한 최적값은 같은 영역 내에 위치합니다. 따라서 $\theta^{\text{old}}$에서 웜 스타트하면 올바른 지역 최적값으로 수렴합니다.

**웜 스타트가 실패하는 경우**: $\delta > \sim 50\%$ (저수지의 절반 이상 교체)이면, 분포가 충분히 이동하여 $\theta^{\text{old}}$가 더 이상 올바른 영역에 있지 않을 수 있습니다. 이 경우 웜 스타트는 차선의 지역 최대값으로 수렴할 수 있습니다. 그러나 프레임이 상당히 겹치는 점진적 LiDAR 처리에서 $\delta$는 보통 < 20%입니다.

#### 3. 온라인/점진적 EM을 사용하지 않는 이유

검토 후 기각된 대안적 접근법:

**온라인 EM (Cappé & Moulines, 2009)**: 확률적 근사를 사용하여 각 새 포인트로 파라미터를 업데이트합니다:

$$
\theta_{n+1} = \theta_n + \eta_n \cdot \nabla Q(\theta_n; x_n)
$$

우리 사용 사례에서의 문제점:
- **스텝 사이즈 스케줄링**: $\eta_n$의 세심한 튜닝 필요 (예: $\eta_n = n^{-\alpha}$, $\alpha \in (0.5, 1]$). 너무 공격적 → 불안정. 너무 보수적 → 느린 적응.
- **포인트 교체**: 저수지 샘플링은 이전 포인트를 *교체*합니다. 온라인 EM에는 교체된 포인트를 "학습 취소"하는 메커니즘이 없습니다. 중요도 가중 보정이 필요합니다.
- **소규모 N**: 복셀당 256포인트만 있으므로 배치 EM이 마이크로초 단위로 실행됩니다. 온라인 EM의 포인트별 오버헤드는 속도 향상을 제공하지 않습니다.

**점진적 EM (Neal & Hinton, 1998)**: 포인트별 충분통계량을 유지하고 느긋하게 업데이트합니다. 복잡도: 책임도에 $O(NK)$ 메모리 vs 저수지 $O(N)$ + 컴포넌트 $O(K)$.

**우리의 선택**: 웜 스타트를 가진 배치 EM이 우리 규모($N=256$, $K \le 3$, ~10회 반복)에서 가장 단순하고 올바른 접근법입니다. finalize당 총 비용: ~$256 \times 3 \times 10 = 7{,}680$ 부동소수점 연산으로 무시할 수 있습니다.

### 위험 요소
- **EM 성능**: 10,000+ 복셀 x 256 저수지 포인트 x 20 EM 반복은 느릴 수 있습니다. 프로파일링이 필요합니다. 프레임당 > 5초이면 다음을 고려합니다: (a) `max_em_iterations`를 10으로 줄이기, (b) 복셀 간 `finalize()` 호출을 병렬화.
- **포인트가 매우 적은 복셀**: 복셀의 저수지에 `max_components`보다 적은 포인트가 있으면, `fit_gmm`이 더 적은 컴포넌트로 우아하게 폴백해야 합니다.

---

## Phase 3: MixtureLightNDT 팩터

**목표**: 다중 컴포넌트 GMM 복셀을 정합에 사용하는 `IntegratedMixtureLightNDTFactor_`를 구현합니다. 각 소스 포인트는 이웃 복셀의 컴포넌트들 중 최적의 컴포넌트(최소 마할라노비스)와 매칭됩니다.

**복잡도**: 대규모  
**의존성**: Phase 2.5  
**예상 소요 시간**: ~4시간

### 생성할 파일

#### 3a. `include/gmm/integrated_mixture_light_ndt_factor.hpp`

**`integrated_light_ndt_factor.hpp` 구조를 미러링합니다. 주요 차이점:**

```cpp
namespace gtsam_points {

// Multi-component correspondence: best single component from GMM
struct MixtureNdtCorrespondence {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector4d mean = Eigen::Vector4d::Zero();
  Eigen::Matrix4d inv_cov = Eigen::Matrix4d::Zero();
  double weight = 0.0;        // NEW: component weight pi_k
  bool valid = false;
};

template <typename SourceFrame = gtsam_points::PointCloud>
class IntegratedMixtureLightNDTFactor_ : public gtsam_points::IntegratedMatchingCostFactor {
public:
  // Same constructor pattern as LightNDT, but casts to GMMVoxelMapCPU
  IntegratedMixtureLightNDTFactor_(
    gtsam::Key target_key, gtsam::Key source_key,
    const GaussianVoxelMap::ConstPtr& target_voxels,
    const std::shared_ptr<const SourceFrame>& source);

  // Unary version
  IntegratedMixtureLightNDTFactor_(
    const gtsam::Pose3& fixed_target_pose, gtsam::Key source_key,
    const GaussianVoxelMap::ConstPtr& target_voxels,
    const std::shared_ptr<const SourceFrame>& source);

  // Same setter API as LightNDT
  void set_num_threads(int n);
  void set_regularization_epsilon(double eps);
  void set_correspondence_update_tolerance(double angle, double trans);
  void set_search_mode(NDTSearchMode mode);

  int num_inliers() const;
  double inlier_fraction() const;

private:
  virtual void update_correspondences(const Eigen::Isometry3d& delta) const override;
  virtual double evaluate(...) const override;

  std::shared_ptr<const GMMVoxelMapCPU> target_voxels;  // dynamic_pointer_cast target
  std::shared_ptr<const SourceFrame> source;

  // Per-voxel inverse covariance cache: [voxel_id][component_k]
  mutable std::vector<std::vector<Eigen::Matrix4d>> inv_cov_cache;
  mutable bool inv_cov_cached = false;

  mutable std::vector<MixtureNdtCorrespondence> correspondences;
};

using IntegratedMixtureLightNDTFactor = IntegratedMixtureLightNDTFactor_<>;
}
```

#### 3b. `include/gmm/impl/integrated_mixture_light_ndt_factor_impl.hpp`

**`update_correspondences()`, LightNDT와의 핵심 차이점:**

```
For each source point i:
  q_i = delta * source_point[i]
  coord = voxel_coord(q_i)

  best_corr = invalid
  min_mahalanobis = INF

  for each offset in neighbor_offsets:
    voxel_id = lookup(coord + offset)
    if voxel_id < 0: continue
    voxel = lookup_voxel(voxel_id)

    for k = 0 .. voxel.size() - 1:            // iterate ALL components
      inv_cov = inv_cov_cache[voxel_id][k]
      diff = q_i - voxel.components()[k].mean
      m = diff^T * inv_cov * diff

      if m < min_mahalanobis:
        min_mahalanobis = m
        best_corr = {mean=comp.mean, inv_cov=inv_cov, weight=comp.weight, valid=true}

  correspondences[i] = best_corr
```

**`evaluate()`, 가중치 스케일 L2 마할라노비스:**

각 대응의 비용: $\pi_k \cdot r^T \cdot \Sigma_k^{-1} \cdot r$

여기서 $\pi_k$는 컴포넌트 가중치입니다. 야코비안 누적은 `weight = corr.weight`를 스칼라 승수로 사용하는 LightNDT 패턴을 따릅니다.

#### 3c. `src/gmm/integrated_mixture_light_ndt_factor.cpp`

명시적 템플릿 인스턴스화 (`integrated_light_ndt_factor.cpp`를 미러링).

### 수정할 파일

#### 3d. `CMakeLists.txt` (프로젝트 루트)
- `add_executable()` 블록에 `src/gmm/integrated_mixture_light_ndt_factor.cpp` 추가

### 테스트 전략 (TDD)

```
TEST(MixtureLightNDT, ConstructsFromGMMVoxelMap)
TEST(MixtureLightNDT, RejectsNonGMMVoxelMap)
TEST(MixtureLightNDT, ErrorDecreasesWithOptimization)
TEST(MixtureLightNDT, NumericalGradientCheck)
TEST(MixtureLightNDT, IdentityPoseZeroResidual)
TEST(MixtureLightNDT, InlierFractionReasonable)
```

### 검증 기준
- [x] 팩터가 컴파일 및 링크됨
- [x] 생성자에서 `dynamic_pointer_cast<GMMVoxelMapCPU>` 성공
- [x] 수치 그래디언트 검사 통과 (해석적 vs 수치적 1e-4 이내)
- [x] 최적화 중 오차 감소
- [x] 스레드 안전 병렬 평가 (OMP)
- [x] 기존 팩터 테스트에 영향 없음

### 수학적 기초

#### 1. 비용 함수 유도

**출발점, 표준 NDT 비용**: 분포 $\mathcal{N}(\mu, \Sigma)$를 가진 타겟 복셀과 $T \in SE(3)$로 변환된 소스 포인트 $\{p_i\}$가 주어졌을 때:

$$
C_{\text{NDT}}(T) = \sum_i \exp\left(-\frac{1}{2} d_i^T \Sigma^{-1} d_i\right), \quad d_i = \mu - T \cdot p_i
$$

이것은 가우시안 커널의 합으로 매끄럽지만 최적값에서 멀리 떨어진 곳에 평탄 영역이 있어 비볼록합니다.

**LightNDT 단순화**: 지수 함수를 제거합니다:

$$
C_{\text{LightNDT}}(T) = \sum_i d_i^T \Sigma^{-1} d_i \quad \text{(순수 마할라노비스 거리)}
$$

이것은 $d_i$에 대해 **이차적**이므로 Gauss-Newton 최적화에 이상적입니다:
- 그래디언트가 $d_i$에 대해 선형 → 잘 조건화됨
- 헤시안이 상수 (이차 잔차항 불필요)
- 평탄 영역 없음, 비용이 거리에 따라 이차적으로 증가

**MixtureLightNDT 확장**: 복셀당 K개 컴포넌트를 가진 GMM 복셀의 경우:

$$
C_{\text{MixtureLightNDT}}(T) = \sum_i \pi_{k^*(i)} \cdot d_{ik^*(i)}^T \Sigma_{k^*(i)}^{-1} d_{ik^*(i)}
$$

여기서 $k^*(i) = \arg\min_k d_{ik}^T \Sigma_k^{-1} d_{ik}$ (포인트별 최적 컴포넌트 선택)이고 $d_{ik} = \mu_k - T \cdot p_i$입니다.

**모든 컴포넌트를 합산하지 않고 argmin (최적 컴포넌트)을 사용하는 이유**:
- (a) **포인트별 비용을 이차로 유지**: 고정된 대응에서 각 포인트가 단일 이차항을 기여 → 표준 Gauss-Newton 직접 적용 가능
- (b) **log-sum-exp 복잡성 회피**: 적절한 혼합 우도 $\log \sum_k \pi_k \mathcal{N}(d_{ik})$는 log-sum-exp를 필요로 하며, 복잡한 헤시안을 가진 비이차 비용 생성
- (c) **희소 대응이 더 빠름**: 대응 선택 후 반복당 포인트당 하나의 컴포넌트만 평가, 전체 혼합의 K개 컴포넌트 대비
- (d) **경험적으로 동등**: 잘 분리된 컴포넌트의 경우, 최적 컴포넌트가 혼합 우도를 지배(>90% 책임도)하므로 argmin 근사의 오차는 무시할 수 있음

#### 2. 가중 마할라노비스($\pi_k$ 인자)를 사용하는 이유

가중치 $\pi_k$가 각 대응의 기여를 스케일링합니다:

$$
C_i = \pi_{k^*} \cdot r_i^T \Sigma_{k^*}^{-1} r_i
$$

**$\pi_k$ 없이**: 모든 컴포넌트가 동등하게 신뢰됩니다. $\pi_k = 0.01$ (포인트의 1% 피팅)인 노이즈 컴포넌트가 $\pi_k = 0.9$ (포인트의 90% 피팅)인 지배적 컴포넌트와 정합 비용에 동일한 영향을 미칩니다. 이렇게 하면 이상치/노이즈 컴포넌트가 자세 추정을 왜곡합니다.

**$\pi_k$ 포함**: 지배적 컴포넌트가 비용에 ~90배 더 기여 → 허위 컴포넌트의 자연스러운 하향 가중치. 이것은 가중치가 각 측정의 신뢰도를 반영하는 **가중 최소 제곱법**과 유사합니다.

**소프트 EM과의 연결**: 전체 혼합 모델에서 사후 책임도 $\gamma_{ik} \propto \pi_k \mathcal{N}(x_i|\mu_k,\Sigma_k)$는 자연스럽게 $\pi_k$를 사전 확률로 포함합니다. 우리의 $\pi_k$ 가중치는 하드 할당(argmin) 단순화 하에서 이 사후 가중치를 근사합니다.

#### 3. SE(3)에 대한 야코비안 유도

**매개변수화**: $T = (R, t) \in SE(3)$, 리 대수 $\xi = (\omega, v) \in \mathfrak{se}(3)$를 통한 섭동, 여기서 $\omega \in \mathbb{R}^3$ (회전)이고 $v \in \mathbb{R}^3$ (병진). 섭동은 $T(\xi) = \exp(\hat{\xi}) \cdot T_0$으로 작용하며, 좌측 섭동 규약을 따릅니다.

**잔차**: $r_i = \mu_k - (R \cdot p_i + t) \in \mathbb{R}^3$

**야코비안** $\partial r_i / \partial \xi$: 좌측 섭동 하에서, 항등식 $\partial(R \cdot p)/\partial\omega = -R \cdot [p]_\times$ (여기서 $[p]_\times$는 반대칭/hat 행렬)를 사용:

$$
\frac{\partial r_i}{\partial \xi} = \left[\frac{\partial r_i}{\partial \omega} \;\middle|\; \frac{\partial r_i}{\partial v}\right]
= \left[R \cdot [p_i]_\times \;\middle|\; -I_{3 \times 3}\right]
$$

($3 \times 6$ 행렬이지만 동차 좌표를 위해 $4 \times 6$에 제로 패딩 사용)

여기서:

$$
[p_i]_\times = \begin{pmatrix} 0 & -p_z & p_y \\ p_z & 0 & -p_x \\ -p_y & p_x & 0 \end{pmatrix} \quad (\text{SO(3)::Hat() 연산})
$$

**비용 야코비안**: 비용 $C_i = \pi_k \cdot r_i^T \Sigma_k^{-1} r_i$에 대해:

$$
\frac{\partial C_i}{\partial \xi} = 2 \cdot \pi_k \cdot r_i^T \Sigma_k^{-1} \cdot \frac{\partial r_i}{\partial \xi} \quad (1 \times 6 \text{ 행 벡터})
$$

**Gauss-Newton 헤시안 근사**: 이차 항을 생략하면:

$$
H \approx 2 \cdot \sum_i \pi_{k^*(i)} \cdot J_i^T \Sigma_{k^*(i)}^{-1} J_i \quad (6 \times 6 \text{ 행렬})
$$

여기서 $J_i = \partial r_i / \partial \xi$. 이것은 **구성에 의해 양의 준정부호**입니다 (양의 준정부호 행렬의 합). Gauss-Newton이 하강 방향을 생성함을 보장합니다. Levenberg-Marquardt 감쇠 $H + \lambda I$가 이를 양의 정부호로 만듭니다.

#### 4. 고정 대응 안정성

대응(각 소스 포인트가 어떤 컴포넌트 $k^*$와 매칭되는지)은 업데이트 사이에 **고정**됩니다. 자세 변화가 허용 오차를 초과하면 업데이트됩니다: $\Delta\text{angle} > \theta_{\text{tol}}$ 또는 $\Delta\text{trans} > t_{\text{tol}}$.

**업데이트 사이**: 비용 함수는 고정된 이차항들의 합입니다 (각 포인트가 고정된 $\mu_k$, $\Sigma_k^{-1}$을 가짐). 이것은 자세 파라미터에 대한 **매끄러운 이차 함수** → Gauss-Newton / Levenberg-Marquardt에 완벽하게 적합.

**대응 업데이트 경계에서**: 대응이 전환될 때 (포인트 i가 컴포넌트 k에서 k'로 변경):
- **$C^0$ 연속** (값 연속): 전환 경계에서 $d_{ik}^T \Sigma_k^{-1} d_{ik} = d_{ik'}^T \Sigma_{k'}^{-1} d_{ik'}$ (argmin 정의에 의해)
- **$C^1$ 연속 아님** (그래디언트 점프 가능): $\mu_k \neq \mu_{k'}$이고 $\Sigma_k^{-1} \neq \Sigma_{k'}^{-1}$이므로 야코비안 $\partial C_i/\partial \xi$가 다름

이 **$C^0$이지만 $C^1$이 아닌** 동작은 ICP의 대응 전환 동작과 동일하며, 반복 정합에서 표준적입니다. LM의 신뢰 영역 메커니즘이 그래디언트 불연속성을 우아하게 처리합니다. 스텝이 비용을 증가시키는 대응 전환을 야기하면 LM이 신뢰 영역을 축소합니다.

#### 5. 역공분산 캐싱

**$\Sigma^{-1}$ 계산 비용**: $3 \times 3$ SPD 행렬의 행렬 역행렬은 $O(d^3) = O(27)$ flops. evaluate() 호출마다 순진하게 계산하면:

평가당 총량: $K \times V \times O(d^3)$, $K=3$ 컴포넌트, $V=10{,}000$ 복셀 → ~810,000 역행렬 per 팩터 평가.

**캐싱 전략**: 대응 업데이트 시점에 (복셀, 컴포넌트) 쌍마다 inv_cov를 한 번 계산합니다. 업데이트 사이(보통 3~5 최적화 반복)에는 캐시된 값을 사용 → 상각 $O(1)$ per evaluate().

**캐시 무효화**: 대응이 업데이트될 때만 (이때 매칭 복셀/컴포넌트도 변경될 수 있음). 캐시는 각 업데이트 시 완전히 재구축되며, 점진적 무효화보다 단순합니다.

**메모리 비용**: $V \times K \times \text{sizeof}(\text{Matrix4d})$, Matrix4d = $4 \times 4 \times 8$ 바이트 = 128 바이트.
$V=10{,}000$, $K=3$일 때: $10{,}000 \times 3 \times 128 = 3.84$ MB로, 포인트 클라우드 데이터 자체(일반적인 LiDAR 프레임에 ~10~100 MB)에 비해 무시할 수 있습니다.

### 위험 요소
- **수치 그래디언트 검사 민감도**: 가중 마할라노비스 비용이 컴포넌트 경계 근처에서 급격한 곡률을 가질 수 있습니다. 더 작은 유한 차분 스텝(1e-6)과 더 느슨한 허용 오차(1e-3)를 사용합니다.
- **최적화 중 컴포넌트 전환**: 최근접 컴포넌트 매칭에 고유합니다. LM에서 허용 가능하며, 대응은 반복마다 업데이트됩니다.
- **이웃 검색에서 빈 복셀**: 우아하게 건너뜁니다.

### 설계 결정: `pi_k * Mahalanobis`를 사용하고 `log-likelihood`를 사용하지 않는 이유

$\pi_k \cdot r^T \Sigma_k^{-1} r$ (가중 마할라노비스)를 사용하면 비용 함수가 **이차적**으로 유지되어, 잘 동작하는 Gauss-Newton 헤시안을 생성합니다. 전체 log-GMM-likelihood는 log-sum-exp와 더 복잡한 헤시안 근사를 필요로 합니다.

---

## Phase 3.5: 벤치마크 통합 + 검증

**목표**: `MixtureLightNDT`를 메인 벤치마크에 연결하고, 헤드리스 비교를 실행하여 R/t 오차가 LightNDT와 경쟁적임을 검증합니다.

**복잡도**: 소규모  
**의존성**: Phase 3  
**예상 소요 시간**: ~1.5시간

### 수정할 파일

#### 3.5a. `src/main.cpp`

**변경 사항 (4곳):**

1. **include 추가** (상단):
   ```cpp
   #include "gmm/gmm_voxelmap_cpu.hpp"
   #include "gmm/integrated_mixture_light_ndt_factor.hpp"
   ```

2. **팩터 타입 추가** (생성자 내):
   ```cpp
   factor_types.push_back("MixtureLightNDT"); // 7: GMM-based LightNDT
   ```

3. **gmm_voxelmaps 저장소 + 생성 루프 추가**

4. **팩터 생성 분기 추가** (`create_factor()` 내):
   ```cpp
   else if (factor_types[factor_type] == std::string("MixtureLightNDT"))
   {
     auto factor = gtsam::make_shared<gtsam_points::IntegratedMixtureLightNDTFactor_<gtsam_points::PointCloud>>(
       target_key, source_key, target_voxelmap, source);
     factor->set_num_threads(num_threads);
     factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT7);
     factor->set_regularization_epsilon(1e-3);
     factor->set_correspondence_update_tolerance(correspondence_update_tolerance_rot,
                                                  correspondence_update_tolerance_trans);
     return factor;
   }
   ```

5. **gmm_voxelmaps를 최적화에 연결**, MixtureLightNDT가 선택되었을 때 `gmm_voxelmaps` 전달.

### 테스트 전략

```bash
# Build and run headless benchmark
docker exec -it bottom-lidar bash
cd /root/workdir && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON
make -j$(nproc)
./lidar_registration_demo --headless
```

### 수학적 기초

#### 1. 정합 오차 메트릭

**회전 오차**: 추정된 회전 $R_{\text{est}}$와 정답 $R_{\text{gt}}$가 주어졌을 때:

$$
e_R = \arccos\left( \frac{\text{tr}(R_{\text{est}}^T \cdot R_{\text{gt}}) - 1}{2} \right) \quad [\text{radians}]
$$

이것은 SO(3) 위의 **측지 거리**로, 회전 $R_{\text{est}}^T \cdot R_{\text{gt}}$ (상대 회전 오차)의 각도입니다. arccos 인자는 수치 안전을 위해 $[-1, 1]$로 클리핑됩니다 (부동소수점 오류로 $\text{tr}(\cdot)$이 $[1, 3]$ 밖으로 약간 벗어날 수 있음).

**병진 오차**: 미터 단위 유클리드 거리:

$$
e_t = \|t_{\text{est}} - t_{\text{gt}}\|_2
$$

**결합 RPE를 사용하지 않는 이유**: $\sqrt{e_R^2 + e_t^2}$ 같은 결합 메트릭은 단위 정규화가 필요합니다 (라디안과 미터는 비교 불가). $e_R$과 $e_t$를 별도로 보고하는 것이 정합 문헌의 표준입니다 (예: KITTI odometry benchmark).

#### 2. 통계적 검증

**프로토콜**: $N \ge 10$ 프레임 쌍을 실행하여 오차 샘플 $\{e_R^{(j)}, e_t^{(j)}\}_{j=1}^{N}$을 얻습니다. 다음을 계산:
- $\text{mean}(e_R) \pm \text{std}(e_R)$ 및 $\text{mean}(e_t) \pm \text{std}(e_t)$

**경쟁력 기준**: MixtureLightNDT가 LightNDT와 "경쟁적"이려면:
- $\text{mean}(e_{R,\text{Mixture}}) \le 1.2 \times \text{mean}(e_{R,\text{LightNDT}})$ (회전 20% 이내)
- $\text{mean}(e_{t,\text{Mixture}}) \le 1.2 \times \text{mean}(e_{t,\text{LightNDT}})$ (병진 20% 이내)

20% 마진은 다음을 고려합니다: (a) GMM 피팅 오버헤드로 인한 약간의 노이즈, (b) 다중 컴포넌트 매칭으로 인한 다른 수렴 영역. MixtureLightNDT가 단일 가우시안 복셀이 잘 맞지 않는 구조화된 환경(벽, 바닥면)에서는 실제로 LightNDT를 *개선*할 것으로 기대합니다.

**수렴 속도 비교**: 허용 오차에 도달하기까지의 반복 횟수도 비교합니다. MixtureLightNDT는 팩터 평가당 더 많은 반복(다중 컴포넌트 검색)이 필요할 수 있지만, 총 반복은 더 적을 수 있습니다(더 나은 대응).

### 검증 기준
- [x] MixtureLightNDT를 포함하여 `--headless`가 완료까지 실행
- [x] MixtureLightNDT R/t 오차 <= LightNDT (또는 20% 이내)
- [x] 크래시, NaN 오류, 어설션 실패 없음
- [x] 이전 모든 테스트 여전히 통과

---

## 병렬화 가능 작업 요약

| 단계 | 작업 | 병렬 가능 대상 |
|-------|------|----------------|
| 1a | GMMVoxel 구조체 + GMMVoxelMapCPU | - |
| 1b | Phase 1 테스트 | 1a (TDD: 테스트 먼저 작성) |
| 2a | EM 백엔드 헤더 + 구현 | 1a (독립 모듈) |
| 2b | EM 백엔드 테스트 | 2a |
| **2.5** | **EM → finalize 연결** | **1a + 2a 완료 대기 필요** |
| 3a | 팩터 헤더 + 구현 | 2.5 (GMMVoxelMapCPU 완료 필요) |
| 3b | 팩터 테스트 | 3a |
| 3.5 | 벤치마크 통합 | 3a |

---

## 전체 커밋 시퀀스 (10개 원자적 커밋)

| # | 커밋 메시지 | 파일 |
|---|---|---|
| 1 | `feat(types): add GMMVoxel struct with reservoir sampling and stub finalize` | `gmm_voxelmap_cpu.hpp`, `gmm_voxelmap_cpu.cpp`, `CMakeLists.txt` |
| 2 | `test(types): add GMMVoxel and GMMVoxelMapCPU unit tests` | `test_gmm_voxelmap.cpp` |
| 3 | `build(docker): add libarmadillo-dev to Dockerfile` | Dockerfile |
| 4 | `feat(types): add Armadillo EM backend for GMM fitting` | `mixture_em_backend.hpp`, `mixture_em_backend.cpp`, `CMakeLists.txt` |
| 5 | `test(types): add GMM fitting unit tests with synthetic data` | `test_gmm_voxelmap.cpp` |
| 6 | `feat(types): connect EM backend to GMMVoxel::finalize() with warm start` | `gmm_voxelmap_cpu.hpp`, `gmm_voxelmap_cpu.cpp` |
| 7 | `test(types): add GMMVoxel finalize integration tests` | `test_gmm_voxelmap.cpp` |
| 8 | `feat(factors): add MixtureLightNDT factor with multi-component correspondence` | factor hpp/impl/cpp, `CMakeLists.txt` |
| 9 | `test(factors): add MixtureLightNDT factor tests with gradient check` | test file |
| 10 | `feat(benchmark): integrate MixtureLightNDT into headless benchmark` | `src/main.cpp` |

---

## 부록: Momus 지적 사항

### `frame::traits<GMMVoxel>::point()`의 댕글링 레퍼런스 위험

**문제**: `IncrementalVoxelMap<T>`는 `frame::traits<VoxelContents>::point()`를 사용하여 복셀 데이터에 접근합니다. `GaussianVoxel`의 경우 `const Eigen::Vector4d&` (멤버 변수에 대한 참조)를 반환하며, 복셀이 참조보다 수명이 길기 때문에 안전합니다. `GMMVoxel`의 경우, `point(v, k)`는 `std::vector<GMMComponent>`에 인덱싱하므로 `Eigen::Vector4d`를 **값으로** 반환합니다. 참조할 단일 멤버가 없기 때문입니다.

만약 `IncrementalVoxelMap`이 반환값을 `const auto&` (참조)로 저장하면, 값 반환은 즉시 파괴되는 임시 객체에 바인딩됩니다 → **댕글링 레퍼런스**.

**분석**: 안전성은 `IncrementalVoxelMap`이 반환값을 어떻게 사용하느냐에 달려 있습니다:
1. `auto x = frame::point(v, k)`, **안전**: 값을 복사
2. `const auto& x = frame::point(v, k)`, **C++에서 안전**: const 참조가 임시 객체의 수명을 참조의 스코프까지 연장
3. `decltype(auto) x = frame::point(v, k)`, **안전**: 함수가 값으로 반환하므로 `Eigen::Vector4d` (값 타입)로 추론되어 x는 복사본
4. `auto&& x = frame::point(v, k)`, **안전**: 범용 참조가 임시 객체의 수명을 연장

**유일하게 위험한 경우**는 원시 포인터에 저장하거나 참조를 외부 스코프에 반환하는 것입니다. `IncrementalVoxelMap`이 같은 표현식이나 로컬 스코프 내에서 반환값을 사용하므로 댕글링 레퍼런스는 발생하지 않습니다.

**해결책**: 우리의 `frame::traits<GMMVoxel>::point()`는 `Eigen::Vector4d` (값으로)를 반환합니다. 이 계약을 문서화하기 위한 컴파일 타임 어설션을 추가합니다:

```cpp
static_assert(!std::is_reference_v<decltype(frame::traits<GMMVoxel>::point(std::declval<const GMMVoxel&>(), 0))>,
              "GMMVoxel::point() must return by value, not reference");
```

추가로 `test_gmm_voxelmap.cpp`에 단위 테스트를 추가합니다:
```
TEST(GMMVoxel, PointReturnsByValue)
  — 반환된 Vector4d를 수정해도 컴포넌트의 mean에 영향을 미치지 않는지 확인
  — 앨리어싱 / 참조 반환이 없음을 증명
```

---

## 파일 요약

| # | 경로 | 단계 | 작업 |
|---|------|-------|------|
| 1 | `include/gmm/gmm_voxelmap_cpu.hpp` | 1 | 생성 |
| 2 | `src/gmm/gmm_voxelmap_cpu.cpp` | 1+2.5 | 생성 후 수정 |
| 3 | `include/gmm/mixture_em_backend.hpp` | 2 | 생성 |
| 4 | `src/gmm/mixture_em_backend.cpp` | 2 | 생성 |
| 5 | `include/gmm/integrated_mixture_light_ndt_factor.hpp` | 3 | 생성 |
| 6 | `include/gmm/impl/integrated_mixture_light_ndt_factor_impl.hpp` | 3 | 생성 |
| 7 | `src/gmm/integrated_mixture_light_ndt_factor.cpp` | 3 | 생성 |
| 8 | `CMakeLists.txt` (프로젝트 루트) | 1,2,3 | 수정 (3회) |
| 9 | `src/main.cpp` | 3.5 | 수정 |
| 10 | `src/gmm/test/test_gmm_voxelmap.cpp` | 1,2,2.5,3 | 생성 후 확장 |
