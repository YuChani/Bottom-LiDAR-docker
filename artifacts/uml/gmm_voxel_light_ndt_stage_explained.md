# GMM Voxel Map → Mixture LightNDT 구현 순서에 대한 수학 검증 및 코드 분석

이 문서는 현재 `Bottom-LiDAR-docker` 코드 기준으로, 왜 앞으로의 구현 순서를

```text
GMM voxel map 타입 추가
-> voxel 내부 build / EM fitting 안정화
-> Mixture LightNDT factor 연결
-> 그 다음 full Mixture NDT 확장
```

로 잡는 것이 수학적으로 맞고, 아키텍처적으로도 안전한지 설명하기 위한 문서다.

핵심 목표는 두 가지다.

1. 왜 이 순서가 맞는지 수학적으로 납득 가능하게 정리하는 것
2. 현재 코드에서 정확히 어디를 바꿔야 하는지 단계별로 연결해 주는 것

관련 선행 문서:

- `artifacts/uml/mixture_model_ndt_design_explained.md`
- `artifacts/uml/mixture_model_ndt_flow.prisma`
- `artifacts/uml/incremental_voxelmap_impl_explained.md`

---

## 1. 먼저 결론

현재 repo는 **voxel 하나당 Gaussian 하나**를 전제로 움직인다. 따라서 GMM을 하려면 factor를 먼저 바꾸는 것이 아니라, **map 표현을 먼저 바꿔야 한다.**

그래서 가장 안전한 순서는 아래다.

1. `GMMVoxelMapCPU` 같은 새 map 타입 추가
2. voxel 내부에서 `($\pi_k$, $\mu_k$, $\Sigma_k$)`를 만드는 build / EM fitting 구현
3. 그 결과를 소비하는 `MixtureLightNDT` factor 구현
4. 이후 필요하면 full Mixture NDT로 확장

이 순서가 맞는 이유는 간단하다.

- mixture의 본질은 **map 내부 분포 표현**에 있기 때문이고
- 현재 factor들은 모두 **이미 만들어진 single Gaussian voxel map** 위에서만 동작하기 때문이다.

---

## 2. 현재 기준선: 이 repo의 voxel 표현은 single-Gaussian이다

현재 기준 타입은 아래 파일들에서 확인된다.

- `thirdparty/gtsam_points/include/gtsam_points/types/gaussian_voxelmap_cpu.hpp`
- `thirdparty/gtsam_points/src/gtsam_points/types/gaussian_voxelmap_cpu.cpp`

현재 `GaussianVoxel`는 다음만 가진다.

- `mean`
- `cov`
- `num_points`
- `intensity`
- `finalized`

즉 voxel 하나가 가지는 분포는:

$$
\text{voxel} \Rightarrow (\mu, \Sigma)
$$

하나뿐이다.

여기에는:

- component 배열
- component weight
- component 개수

같은 mixture 정보가 없다.

또한 `GaussianVoxelMapCPU::lookup_voxel_index()` 와 `lookup_voxel()`는 결국 **coord당 voxel 하나**, **voxel id당 Gaussian 하나**만 반환한다. 즉 현재 target map API 자체가 이미 single-result 구조다.

이 말은 곧, 지금의 factor들은 "single Gaussian per voxel"을 당연한 전제로 삼고 있다는 뜻이다.

---

## 3. 수학적 이유 1: 현재 insert/finalize 경로는 멀티모달 정보를 보존하지 않는다

핵심 파일:

- `thirdparty/gtsam_points/include/gtsam_points/ann/impl/incremental_voxelmap_impl.hpp`
- `thirdparty/gtsam_points/src/gtsam_points/types/gaussian_voxelmap_cpu.cpp`

현재 insert 흐름은 아래와 같다.

```text
점 입력
  -> voxel coord 계산
  -> 해당 voxel에 add()
  -> 모든 voxel finalize()
```

코드상 핵심은:

- `coord = fast_floor(points.points[i] * inv_leaf_size)`
- `voxel.add(...)`
- 마지막에 `voxel->second.finalize()`

이다.

그리고 `GaussianVoxel::add()` / `finalize()`는 실질적으로 다음을 한다.

$$
\mu = \frac{1}{n} \sum_i x_i
$$

그리고 현재 코드 구현은 공분산도 voxel 내부 sample covariance를 새로 계산한다기보다, 입력 점이 이미 갖고 있는 covariance들을 평균내는 식으로 summary를 만든다.

즉 중요한 점은:

- voxel 안 raw sample 구조는 유지되지 않고
- 마지막에는 single summary로 collapse된다는 것이다.

그래서 voxel 내부에 두 개 이상의 모드가 있더라도, 현재 경로에서는 결국 하나의 `mean/cov`로 눌려 버린다.

이것이 바로 **mixture를 하려면 map build 단계가 먼저 바뀌어야 하는 이유**다.

---

## 4. 수학적 이유 2: GMM의 핵심은 factor가 아니라 component 추정이다

GMM에서 먼저 필요한 것은 cost 함수가 아니라 **component 그 자체**다.

즉 voxel 내부에 실제로 아래가 있어야 한다.

$$
\{(\pi_k, \mu_k, \Sigma_k)\}_{k=1}^{K}
$$

여기서:

- $\pi_k$: component weight
- $\mu_k$: component mean
- $\Sigma_k$: component covariance

이다.

이 component들이 아직 만들어지지도 않았는데 factor만 먼저 mixture처럼 바꾸면, 그건 수학적으로 target density가 없는 상태에서 likelihood만 흉내 내는 것이 된다.

즉 올바른 순서는 항상:

```text
sample -> component 추정 -> mixture density 확정 -> registration cost 정의
```

이다.

이 순서가 뒤집히면 mixture registration이 아니라 단순 heuristic soft association이 되기 쉽다.

---

## 5. EM이 핵심이고, KL은 선택 단계다

여기서 자주 헷갈리는 부분이 있다.

### 5.1 EM의 역할

EM은 mixture 추정의 본체다.

voxel 안 샘플이 $x_i$일 때, component responsibility는 보통:

$$
\gamma_{ik} = \frac{\pi_k \mathcal{N}(x_i \mid \mu_k, \Sigma_k)}{\sum_j \pi_j \mathcal{N}(x_i \mid \mu_j, \Sigma_j)}
$$

로 계산된다.

그 다음:

$$
N_k = \sum_i \gamma_{ik}
$$

$$
\pi_k = \frac{N_k}{N}, \quad
\mu_k = \frac{1}{N_k}\sum_i \gamma_{ik} x_i
$$

$$
\Sigma_k = \frac{1}{N_k}\sum_i \gamma_{ik}(x_i-\mu_k)(x_i-\mu_k)^T
$$

처럼 파라미터를 업데이트한다.

즉 EM은 "voxel 내부 샘플들을 실제 mixture component로 바꾸는 핵심 수학"이다.

### 5.2 KL divergence의 역할

반면 KL divergence는 이번 1차 구현에서 **필수 핵심**이 아니다.

KL은 보통:

- 너무 비슷한 component를 병합하거나
- component reduction / pruning을 하거나
- merge criterion을 정하는 보조 규칙

으로 쓰는 쪽이 자연스럽다.

즉 현재 실험 목표에서는:

```text
EM = 필수
KL merge = 선택
```

라고 보는 것이 맞다.

따라서 첫 구현에서는 KL을 빼고,

- initializer
- EM
- covariance floor
- tiny-weight pruning

정도만 두는 것이 더 안전하다.

---

## 6. 왜 1단계는 새 GMM voxel map 타입 추가인가

현재 코드의 핵심 타입 경계는 아래다.

- `GaussianVoxel`
- `GaussianVoxelMapCPU`
- `IntegratedLightNDTFactor`

현재 factor 계층은 `GaussianVoxelMapCPU`와 `NdtCorrespondence` 같은 single-Gaussian 구조를 직접 사용한다. 즉 기존 클래스 안으로 억지로 component vector를 밀어 넣는 식은 현재 아키텍처와 잘 맞지 않는다.

그래서 가장 자연스러운 1단계는 sibling path를 만드는 것이다.

예:

- `GMMVoxel` 또는 `GaussianMixtureVoxel`
- `GMMVoxelMapCPU` 또는 `GaussianMixtureVoxelMapCPU`
- `MixtureComponent`

이 방식의 장점은:

- 기존 경로를 유지할 수 있고
- single Gaussian baseline과 바로 비교 가능하며
- 현재 benchmark / factor 구조를 깨뜨리지 않는다는 것이다.

즉 "map 타입 추가"가 첫 단계인 이유는 단순한 취향이 아니라, 현재 코드의 타입 불변식과 의존성 구조 때문이라고 볼 수 있다.

---

## 7. 왜 2단계는 voxel 내부 build/EM 안정화인가

이 단계에서 해야 하는 일은 registration이 아니다. **GMM voxel 자체가 제대로 만들어지는지 검증하는 것**이다.

필요한 내부 구성은 대략 아래다.

```text
voxel
  -> sample reservoir / raw sample buffer
  -> initializer
  -> EM fitter
  -> optional merge/prune
  -> finalized components
```

이 단계에서 검증해야 할 수학 포인트는 다음이다.

1. component weight가 0보다 크고 합이 1에 가깝게 유지되는가
2. covariance가 positive definite 또는 최소한 invertible regularized form을 유지하는가
3. bimodal voxel에서 single Gaussian보다 log-likelihood가 개선되는가
4. sample 수가 적은 voxel은 single fallback이 잘 되는가

즉 이 단계는 "GMM을 만들 수 있는가"를 검증하는 단계이지, 아직 "정합이 잘 되는가"를 검증하는 단계가 아니다.

이 분리가 중요하다. 그래야 실패했을 때 원인이

- map fitting 문제인지
- registration factor 문제인지

분리된다.

---

## 8. 왜 3단계는 Mixture LightNDT가 가장 안전한 첫 factor인가

핵심 파일:

- `thirdparty/gtsam_points/include/gtsam_points/factors/integrated_light_ndt_factor.hpp`
- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_light_ndt_factor_impl.hpp`
- `thirdparty/gtsam_points/include/gtsam_points/factors/integrated_matching_cost_factor.hpp`
- `thirdparty/gtsam_points/src/gtsam_points/factors/integrated_matching_cost_factor.cpp`

현재 `IntegratedLightNDTFactor`는 구조가 단순하다.

현재 cost는 essentially:

$$
r = \mu_B - (Rp+t)
$$

$$
\text{cost} = r^T \Sigma_B^{-1} r
$$

이다.

즉 현재 LightNDT는:

- target voxel Gaussian 하나 선택
- residual 계산
- quadratic Mahalanobis cost 계산

이라는 매우 해석하기 쉬운 구조다.

이걸 mixture로 확장하면 첫 버전은 이렇게 잡을 수 있다.

```text
source point p
  -> p' = Rp+t
  -> target voxel의 mixture components 읽기
  -> responsibility 또는 effective component weights 계산
  -> weighted Mahalanobis quadratic로 local cost 구성
```

즉 full Mixture NDT보다 훨씬 단순하게 시작할 수 있다.

또한 base class `IntegratedMatchingCostFactor`는 이미:

- `update_correspondences(delta)`
- `evaluate(delta, H, b)`

라는 확장 지점을 제공하고 있으므로, **새 mixture factor를 current LightNDT factor의 sibling으로 추가하기에 적합하다.**

---

## 9. 왜 correspondence freeze가 중요하고, 왜 LightNDT에서 먼저 검증해야 하는가

현재 base factor는 `error()`와 `linearize()`에서 correspondence 갱신에 의존한다. mixture로 가면 여기서 문제가 더 커진다.

왜냐하면 component responsibility가 optimization step마다 크게 바뀌면:

- best mode가 뒤집히고
- cost surface가 불연속적으로 보이고
- LM / GN 선형화가 불안정해질 수 있기 때문이다.

그래서 첫 실험에서는:

- `update_correspondences()`에서 component set / responsibility를 정하고
- `evaluate()`에서는 그걸 freeze한 채 local quadratic approximation을 하는 쪽이 안전하다.

이 철학은 현재 repo의 correspondence caching 구조와도 잘 맞는다.

즉 MixtureLightNDT는 단순해서 좋은 것이 아니라, **frozen responsibility 실험을 가장 먼저 붙이기 좋은 factor**라서 좋은 것이다.

---

## 10. 왜 full Mixture NDT는 그 다음이어야 하는가

full mixture NDT로 가면 cost는 더 이상 단순 quadratic이 아니다.

자연스러운 형태는 보통:

$$
p(x) = \sum_k \pi_k \mathcal{N}(x \mid \mu_k, \Sigma_k)
$$

$$
\text{cost}(x) = -\log \sum_k \pi_k \mathcal{N}(x \mid \mu_k, \Sigma_k)
$$

가 된다.

이 단계로 가면 필요해지는 것이 많다.

- log-sum-exp 수치 안정화
- responsibility gradient 해석
- Hessian 근사 전략
- component switching에 대한 안정성 처리

즉 map fitting이 아직 안정화되지 않은 상태에서 full Mixture NDT까지 한 번에 가면, 어디서 문제가 났는지 분리하기 어렵다.

그래서 full Mixture NDT는 **Mixture LightNDT가 먼저 안정화된 뒤** 가는 것이 맞다.

---

## 11. 단계별로 무엇을 검증해야 하는가

### 11.1 1단계: GMM voxel map 타입 추가

검증 질문:

- 기존 `GaussianVoxelMapCPU`를 안 깨고 sibling path가 분리되었는가
- voxel 하나가 여러 component를 담을 수 있는가
- lookup API가 mixture voxel을 반환할 수 있는가

### 11.2 2단계: voxel 내부 build / EM fitting

검증 질문:

- component weight가 정상적인가
- covariance가 invertible한가
- single Gaussian보다 더 높은 voxel likelihood를 주는가
- raw sample multimodality가 실제로 반영되는가

### 11.3 3단계: Mixture LightNDT factor

검증 질문:

- source point와 target component 사이 responsibility가 안정적인가
- frozen correspondence로 optimizer가 흔들리지 않는가
- 기존 LightNDT보다 multimodal region에서 더 낫게 동작하는가

### 11.4 4단계: full Mixture NDT

검증 질문:

- mixture likelihood 계산이 수치적으로 안정적인가
- quadratic 근사보다 실제 score-shaped objective가 개선을 주는가
- iteration / runtime / robustness trade-off가 납득 가능한가

즉 이 문서에서 말하는 "올바른 순서"는 단지 구현 편의성이 아니라, **검증 실험의 인과관계를 분리하는 순서**이기도 하다.

---

## 12. 현재 코드에서 각 단계의 확장 지점

### 12.1 voxel/map 계층

- `thirdparty/gtsam_points/include/gtsam_points/types/gaussian_voxelmap_cpu.hpp`
- `thirdparty/gtsam_points/src/gtsam_points/types/gaussian_voxelmap_cpu.cpp`
- `thirdparty/gtsam_points/include/gtsam_points/ann/incremental_voxelmap.hpp`
- `thirdparty/gtsam_points/include/gtsam_points/ann/impl/incremental_voxelmap_impl.hpp`

여기가 새 `GMMVoxel`, `GMMVoxelMapCPU`, sample reservoir, finalize path를 추가할 자리다.

### 12.2 factor 계층

- `thirdparty/gtsam_points/include/gtsam_points/factors/integrated_light_ndt_factor.hpp`
- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_light_ndt_factor_impl.hpp`
- `thirdparty/gtsam_points/include/gtsam_points/factors/integrated_matching_cost_factor.hpp`

여기가 `MixtureLightNDTFactor`를 current factor의 sibling path로 추가할 자리다.

### 12.3 benchmark 연결

- `src/main.cpp`

여기가 새 factor type을 등록하고 기존 NDT / LightNDT와 A/B 비교 실험을 붙일 자리다.

즉 코드 구조상으로도:

```text
map 먼저
-> factor 나중
```

이 자연스럽다.

---

## 13. 명시적으로 바로잡아야 할 오해

1. **"KL -> EM이 정석이다"**
   - 아니다. 지금 실험에서는 EM이 핵심이고 KL은 선택적 merge/prune 기준이다.

2. **"factor만 먼저 바꾸면 mixture가 된다"**
   - 아니다. mixture의 본질은 map-level density 표현이다.

3. **"주변 voxel 여러 개를 soft하게 섞으면 true mixture voxel이다"**
   - 아니다. 그건 factor-level soft association일 뿐이다.

4. **"처음부터 full Mixture NDT로 가는 것이 맞다"**
   - 아니다. LightNDT-style quadratic path가 먼저 안정화되어야 한다.

---

## 14. 최종 정리

이 repo에서 GMM voxel 기반 registration을 가장 안전하게 실험하는 순서는 다음과 같다.

```text
1. 새 GMM voxel map 타입 추가
2. voxel 내부 EM fitting 안정화
3. Mixture LightNDT factor 연결
4. known-transform / synthetic / multimodal scene 검증
5. 이후 full Mixture NDT 확장
6. 필요할 때만 KL merge/split 추가
```

이 순서가 맞는 이유는:

- 수학적으로는 **component가 먼저 존재해야 mixture likelihood를 정의할 수 있기 때문**이고
- 코드 구조상으로는 **현재 factor가 single-Gaussian map 위에 서 있기 때문**이며
- 실험 설계상으로는 **map fitting 문제와 registration 문제를 분리해서 검증할 수 있기 때문**이다.

즉 이 순서는 단순한 구현 편의성이 아니라, 현재 코드와 수학 모델을 함께 고려했을 때 가장 낭비가 적은 실험 순서다.

---

## 15. 관련 코드 및 문서

- `artifacts/uml/mixture_model_ndt_design_explained.md`
- `artifacts/uml/mixture_model_ndt_flow.prisma`
- `artifacts/uml/incremental_voxelmap_impl_explained.md`
- `thirdparty/gtsam_points/include/gtsam_points/types/gaussian_voxelmap_cpu.hpp`
- `thirdparty/gtsam_points/src/gtsam_points/types/gaussian_voxelmap_cpu.cpp`
- `thirdparty/gtsam_points/include/gtsam_points/ann/incremental_voxelmap.hpp`
- `thirdparty/gtsam_points/include/gtsam_points/ann/impl/incremental_voxelmap_impl.hpp`
- `thirdparty/gtsam_points/include/gtsam_points/factors/integrated_light_ndt_factor.hpp`
- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_light_ndt_factor_impl.hpp`
- `thirdparty/gtsam_points/include/gtsam_points/factors/integrated_matching_cost_factor.hpp`
- `thirdparty/gtsam_points/src/gtsam_points/factors/integrated_matching_cost_factor.cpp`
- `src/main.cpp`

---

## 16. 이번 대화에서 추가로 확정된 작업 목표

이번 대화를 통해 작업 목표는 더 좁혀졌다.

처음에는 mixture model을 넣는 큰 방향 자체를 탐색하고 있었지만, 지금은 다음처럼 구체화되었다.

```text
최종 큰 목표:
  GMM voxel 구조를 만들고,
  그 위에 registration을 얹는다.

첫 실험 목표:
  source는 point로 유지하고,
  target만 GMM voxel map으로 바꾼 뒤,
  LightNDT 스타일 factor로 정합을 수행한다.
```

즉 현재 우선순위는 **full Mixture NDT**나 **mixture-to-mixture registration**이 아니라,

- `GMM voxel map`
- `MixtureLightNDT (point-to-fixed-GMM)`

를 먼저 안정화하는 것이다.

---

## 17. 이번 대화에서 바로잡힌 오해

이번 대화에서는 다음 오해들이 정리되었다.

### 17.1 "voxel만 multi-Gaussian으로 바꾸면 LightNDT는 그대로 써도 된다"

이건 정확하지 않다.

현재 `IntegratedLightNDTFactor_`는:

- `GaussianVoxelMapCPU`
- `NdtCorrespondence`
- `inv_cov_cache`

모두가 **단일 Gaussian correspondence**를 전제로 한다.

즉 현재 구조는:

```text
source point 1개
  -> best voxel 1개
  -> mean 1개 / inv_cov 1개
```

이다.

그래서 target voxel이 여러 Gaussian을 가지게 되면 최소한 아래는 같이 바뀌어야 한다.

- map API
- correspondence cache
- `update_correspondences()`
- `evaluate()`

반면 그대로 둘 수 있는 것은 다음이다.

- source를 point로 쓰는 구조
- `IntegratedMatchingCostFactor`의 상위 흐름
- GTSAM factor graph 연결
- LM / GN optimizer 자체

즉 정확한 표현은:

```text
voxel layer + factor layer는 새로 만들고,
optimizer layer는 대부분 유지한다.
```

### 17.2 "KL divergence -> EM이 정석 파이프라인이다"

이것도 정정되었다.

이번 실험에서의 올바른 이해는 다음이다.

- **EM은 본체**
- **KL divergence는 optional merge/prune 기준**

즉 첫 구현은:

- initializer
- EM
- covariance regularization
- tiny-weight pruning

정도로 출발하고, KL 기반 merge/split은 나중에 붙이는 것이 더 맞다.

### 17.3 "Robust Point Set Registration Using Gaussian Mixture Models의 mixture-to-mixture를 바로 따라가면 된다"

그 논문은 **좋은 상위 개념 레퍼런스**이지만, 지금 이 repo의 첫 구현 순서를 바꿀 정도의 직접 구현 기준은 아니다.

그 논문 방식은:

- source도 mixture로 표현해야 하고
- component pair 상태를 다뤄야 하며
- objective도 distribution-overlap 형태로 바뀌고
- gradient / Hessian / 수치안정성이 더 복잡해진다.

따라서 지금은:

```text
참고는 mixture-to-mixture
구현은 point-to-fixed-GMM
```

이 맞다.

---

## 18. secedo EM 재사용에 대한 최종 판단

사용자는 `https://github.com/ratschlab/secedo/blob/main/expectation_maximization.cpp`를 EM backend로 쓰는 전제를 제시했다.

이 전제도 검토 결과가 정리되었다.

### 18.1 그대로 drop-in으로 쓸 수는 없다

확인 결과 `secedo`의 `expectation_maximization.cpp`는:

- `PosData`
- `id_to_pos`
- `theta`
- `logger()`
- `sum()`

같은 자체 타입/유틸리티에 묶인 **2-cluster 전용 구현**이다.

즉 이것은 LiDAR GMM voxel용 범용 EM 한 파일이 아니다.

### 18.2 그렇다고 완전히 버릴 필요도 없다

이 코드는 그대로 공용 API에 노출하는 것이 아니라, **내부 EM backend**로 감싸면 재사용 가능하다.

즉 구조는 다음처럼 잡는 것이 맞다.

```text
GMMVoxelMapCPU
  -> 내부적으로 mixture_em_backend 호출
  -> 최종 MixtureComponent(weight, mean, cov) 생성

MixtureLightNDTFactor
  -> mixture_em_backend를 모름
  -> 최종 component들만 읽음
```

즉 EM은 map build 단계 안에 숨기고, factor에는 절대 노출하면 안 된다.

### 18.3 이 전제를 따르면 파일 수는 늘어난다

기존에 생각했던 "신규 5개 + 기존 2개 수정"보다, `secedo` EM을 감싸는 내부 backend 계층이 들어가므로 다음이 더 맞다.

```text
신규 7개 + 기존 2개 수정
```

권장 신규 파일은 아래와 같다.

#### public path

- `include/gmm/gmm_voxelmap_cpu.hpp`
- `src/gmm/gmm_voxelmap_cpu.cpp`
- `include/gmm/integrated_mixture_light_ndt_factor.hpp`
- `include/gmm/impl/integrated_mixture_light_ndt_factor_impl.hpp`
- `src/gmm/integrated_mixture_light_ndt_factor.cpp`

#### internal EM backend path

- `include/gmm/mixture_em_backend.hpp`
- `src/gmm/mixture_em_backend.cpp`

#### modified existing files

- `CMakeLists.txt` (프로젝트 루트)
- `src/main.cpp`

즉 지금 기준으로는 **public path 5개 + internal backend 2개 + 기존 연결 2개 수정**이 가장 자연스럽다.

---

## 19. 현재 시점에서의 구현 순서 최종판

이번 대화 전체를 종합하면, 앞으로의 실제 구현 순서는 아래처럼 고정하는 것이 맞다.

```text
1. GMM voxel map 타입 추가
2. 내부 EM backend 연결 및 voxel build/finalize 안정화
3. voxel 단위 GMM fitting 품질 검증
4. MixtureLightNDT factor 추가
5. hard-assignment 또는 frozen-responsibility 기반의 첫 registration 실험
6. synthetic / known-transform / multimodal case 검증
7. 필요할 때만 KL merge/split 추가
8. 이후 full Mixture NDT 검토
9. 가장 마지막에만 mixture-to-mixture 검토
```

여기서 중요한 것은 순서 자체보다, **검증의 인과관계를 분리하는 것**이다.

즉 실패했을 때 원인을 다음처럼 분리할 수 있어야 한다.

- GMM map fitting 문제
- correspondence/cache 문제
- MixtureLightNDT cost 문제
- optimizer/linearization 문제

이 분리가 안 되면 디버깅 비용이 폭증한다.

---

## 20. 지금까지의 핵심 결론 요약

이번 대화에서 최종적으로 확정된 핵심 결론은 아래와 같다.

1. 현재 repo는 처음부터 끝까지 **single-Gaussian per voxel** 구조다.
2. 진짜 mixture model을 하려면 **map-level representation**을 먼저 바꿔야 한다.
3. 첫 실험은 **source point 유지 + target GMM voxel + MixtureLightNDT** 가 맞다.
4. **voxel만 바꾸면 안 되고**, factor의 correspondence/cost/cache도 함께 바꿔야 한다.
5. 하지만 **optimizer 자체는 대부분 유지 가능**하다.
6. **EM은 필수**, **KL은 선택**이다.
7. `secedo` EM은 직접 drop-in이 아니라 **내부 backend로 감싸서** 써야 한다.
8. `Robust Point Set Registration Using Gaussian Mixture Models`의 mixture-to-mixture는 **좋은 상위 참고 문헌**이지만, **지금 바로 구현할 1단계는 아니다.**
9. 지금 당장의 가장 올바른 구현 방향은:

```text
GMM voxel map
-> MixtureLightNDT
-> 이후 full Mixture NDT
-> 마지막에 mixture-to-mixture 여부 검토
```

이다.

---

## 21. 보완: 1단계 전 반드시 해결해야 할 설계 결정

코딩을 시작하기 전에 다음 설계 결정이 확정되어야 한다. 이 결정들이 이후 모든 코드 구조를 결정하기 때문이다.

### 21.1 결정 1: GMMVoxelMapCPU의 타입 계층 — 상속 vs. 별도 경로

현재 NDT factor들은 생성자에서 `GaussianVoxelMap::ConstPtr`를 받은 뒤 내부에서 다음 캐스트를 수행한다.

```cpp
auto target = std::dynamic_pointer_cast<const GaussianVoxelMapCPU>(target_voxelmap);
```

따라서 `GMMVoxelMapCPU`가 이 기존 factor와 공존하려면 다음 중 하나를 선택해야 한다.

**선택지 A: GaussianVoxelMapCPU를 상속**

```text
GaussianVoxelMap (abstract)
  -> GaussianVoxelMapCPU
       -> GMMVoxelMapCPU  (상속)
```

장점: 기존 factor가 `dynamic_pointer_cast<GaussianVoxelMapCPU>`로 캐스트해도 성공한다.
단점: `GaussianVoxelMapCPU`의 single-Gaussian 멤버(`flat_voxels<GaussianVoxel>` 등)를 물려받게 되므로, 의미 없는 상태를 들고 다니게 된다.

**선택지 B: GaussianVoxelMap만 상속하고, 새 factor는 별도 캐스트**

```text
GaussianVoxelMap (abstract)
  ├── GaussianVoxelMapCPU      (기존)
  └── GMMVoxelMapCPU            (새 sibling)
```

장점: 타입이 깔끔하게 분리된다.
단점: `MixtureLightNDTFactor`는 `GMMVoxelMapCPU`로 직접 캐스트해야 하므로, 기존 factor에 GMM map을 넘기면 런타임 에러가 난다.

**권장**: 선택지 B. 새 factor(`MixtureLightNDTFactor`)는 처음부터 `GMMVoxelMapCPU`를 직접 받도록 설계하고, 기존 factor에 GMM map을 넘기는 경로는 아예 컴파일 타임에 차단하는 것이 더 안전하다.

### 21.2 결정 2: EM Backend — Armadillo vs. secedo 래핑 vs. 직접 구현

현재 컨테이너에서 확인된 EM 관련 가용 자원은 다음과 같다.

| 선택지 | 상태 | 장점 | 단점 |
|--------|------|------|------|
| Armadillo gmm_full | 시스템 설치됨 (v10.8.2) | 즉시 사용 가능, full-cov EM 내장, 검증된 구현 | Armadillo 타입과 Eigen 타입 변환 필요, 외부 의존성 추가 |
| secedo EM 래핑 | 미설치 (clone 필요) | Eigen 기반이라 타입 호환 쉬움 | 2-cluster 전용 구현, K>2로 일반화하려면 상당한 수정 필요 |
| 직접 구현 | 없음 | 완전한 제어권, Eigen 네이티브, 필요한 것만 구현 | 구현+디버깅 시간 |

**권장**: 첫 프로토타입은 **Armadillo gmm_full**로 빠르게 검증하고, 안정화 이후 **Eigen 네이티브 직접 구현**으로 교체하는 2단계 전략. secedo는 2-cluster 전용이라 일반 K-component 용도로는 수정 비용이 크다.

Armadillo 사용 시 빌드 연결:

```cmake
find_package(Armadillo REQUIRED)
target_link_libraries(gtsam_points PRIVATE ${ARMADILLO_LIBRARIES})
target_include_directories(gtsam_points PRIVATE ${ARMADILLO_INCLUDE_DIRS})
```

### 21.3 결정 3: Sample Reservoir 전략 — 전수 보관 vs. Bounded Buffer

EM fitting을 하려면 voxel 내부 raw 샘플이 필요하다. 현재 `GaussianVoxel::add()`는 sum만 누적하고 개별 점은 버린다.

**선택지 A: 전수 보관**

voxel 내부에 `std::vector<Eigen::Vector4d> raw_samples`를 유지한다.

장점: EM에 모든 샘플을 사용 가능.
단점: 메모리 폭증 (voxel당 수백에서 수천 점).

**선택지 B: Bounded Reservoir Sampling**

voxel 내부에 고정 크기 버퍼 (예: 최대 256개)를 두고, 새 점이 들어오면 reservoir sampling으로 대체한다.

장점: 메모리 제한됨, 통계적 대표성 유지.
단점: 점이 매우 많을 때 일부 정보 손실.

**선택지 C: Sufficient Statistics + 점진적 EM**

raw 샘플 저장 없이 sufficient statistics만 유지하며 online EM을 수행한다.

장점: 메모리 최소.
단점: 구현 복잡도 높음, 수렴 보장 어려움.

**권장**: 첫 구현은 **선택지 B (Bounded Reservoir, capacity=256)**. 메모리와 정확도의 합리적 타협점이며, 구현이 단순하다.

### 21.4 결정 4: Voxel당 최대 Component 수

EM의 K값을 어떻게 정할 것인가.

- **고정 K**: 모든 voxel이 동일한 K (예: K=3)
- **적응형 K**: BIC/AIC 기반으로 voxel마다 K를 다르게
- **단순 상한**: K_max=5, 실제로는 pruning으로 줄어듦

**권장**: 첫 구현은 **K_max=3 고정 + tiny-weight pruning**. 적응형 K는 이후 개선 사항.

---

## 22. 보완: GaussianVoxel::Setting 패턴 활용

현재 `GaussianVoxel`은 다음과 같은 내부 설정 구조를 가진다.

```cpp
struct GaussianVoxel {
  struct Setting {};  // 현재는 비어있음
  // ...
  void add(const Setting&, const PointCloud&, size_t i);
};
```

이 `Setting` struct는 `IncrementalVoxelMap` 템플릿이 `VoxelContents::add()`를 호출할 때 전달하는 파라미터다.

```cpp
// incremental_voxelmap_impl.hpp 내부
voxel.add(voxel_setting, points, i);
```

여기서 `voxel_setting`은 `IncrementalVoxelMap`의 멤버 변수이다.

```cpp
typename VoxelContents::Setting voxel_setting;
```

따라서 새 `GMMVoxel`의 `Setting`에 GMM 파라미터를 넣으면 기존 템플릿 구조를 그대로 활용할 수 있다.

```cpp
struct GMMVoxel {
  struct Setting {
    int max_components = 3;
    int max_em_iterations = 20;
    double convergence_tol = 1e-4;
    double covariance_regularization = 1e-3;
    double min_weight_threshold = 0.01;
    int reservoir_capacity = 256;
  };
  // ...
};
```

이 패턴을 사용하면 `IncrementalVoxelMap<GMMVoxel>`이 자동으로 `GMMVoxel::Setting`을 생성하고 `add()` 호출 시 전달한다. 별도 factory나 configuration 계층이 필요 없다.

---

## 23. 보완: frame::traits 특수화 요구사항

`IncrementalVoxelMap<VoxelContents>`의 `knn_search()`와 외부 코드가 voxel 데이터에 접근하려면 `frame::traits` 특수화가 필요하다.

현재 `GaussianVoxel`의 traits는 다음과 같다.

```cpp
namespace frame {
template <>
struct traits<GaussianVoxel> {
  static bool has_points(...) { return true; }
  static bool has_covs(...) { return true; }
  static bool has_normals(...) { return false; }
  static bool has_intensities(...) { return true; }
  static const Eigen::Vector4d& point(const GaussianVoxel& v, size_t) { return v.mean; }
  static const Eigen::Matrix4d& cov(const GaussianVoxel& v, size_t) { return v.cov; }
  static double intensity(const GaussianVoxel& v, size_t) { return v.intensity; }
};
}
```

여기서 `size_t` 인자는 voxel 내부 점 인덱스인데, `GaussianVoxel`는 `size()==1`이므로 항상 0이다.

`GMMVoxel`의 traits는 두 가지 설계가 가능하다.

**선택지 A: dominant component만 노출 (호환성 우선)**

weight가 가장 큰 component의 mean/cov만 반환한다.
`knn_search()`가 기존과 동일하게 동작하지만, mixture 정보는 traits를 통해서는 접근 불가.

**선택지 B: component 인덱스를 size_t로 노출**

`size()`가 component 수를 반환하고, `point(v, k)`가 k번째 component의 mean을 반환한다.
`knn_search()`가 모든 component를 후보로 검사하므로 mixture 정보가 자연스럽게 노출된다.

**권장**: 선택지 B. `size()`가 component 수를 반환하면 `knn_search()` 루프가 자동으로 모든 component를 후보에 넣는다. 추가 코드 변경 없이 기존 검색 로직이 mixture-aware해진다.

---

## 24. 보완: Incremental EM 전략

현재 `IncrementalVoxelMap::insert()`는 매 호출 끝에 모든 voxel에 대해 `finalize()`를 호출한다.

```cpp
for (auto& voxel : flat_voxels) {
  voxel->second.finalize();
}
```

이 구조에서 `GMMVoxel::finalize()`가 full EM을 수행한다면 다음 문제가 있다.

1. **비용**: 매 insert batch마다 전체 voxel에 대해 EM 반복이면 실시간 처리 불가능할 수 있음
2. **안정성**: 새 점이 몇 개 추가됐을 뿐인데 전체 EM을 다시 돌리면 component가 불필요하게 흔들릴 수 있음

### 24.1 해결 전략: Dirty Flag + Warm Start

```text
GMMVoxel::add():
  reservoir에 점 추가
  dirty = true

GMMVoxel::finalize():
  if (!dirty) return;            // 변경 없으면 skip
  if (first_time):
    full EM from scratch
  else:
    warm-start EM (기존 component를 초기값으로 사용, 적은 iteration)
  dirty = false
```

이 전략의 장점:

- 변경 없는 voxel은 `finalize()` 즉시 리턴 (비용 O(1))
- 변경된 voxel만 EM 수행 (touched voxel 비례 비용)
- warm start로 수렴이 빠름 (보통 2-5회 iteration)

### 24.2 Reservoir Overflow 정책

bounded reservoir가 가득 찼을 때 reservoir sampling을 적용한다.

```text
새 점 도착
  if reservoir.size() < capacity:
    reservoir.push_back(point)
  else:
    j = random_int(0, total_seen)
    if j < capacity:
      reservoir[j] = point
    total_seen++
```

이 방식은 통계적으로 uniform random subset을 유지한다.

---

## 25. 보완: 테스트 및 벤치마크 인프라 상세

### 25.1 기존 단위 테스트 인프라

`thirdparty/gtsam_points/src/test/` 디렉토리에 다음 테스트가 존재한다.

| 파일 | 역할 | GMM 활용 |
|------|------|----------|
| test_voxelmap.cpp | voxelmap 생성/삽입/조회 | GMMVoxelMap 테스트의 직접 템플릿 |
| test_alignment.cpp | ICP/GICP/VGICP/NDT 정합 | MixtureLightNDT 정합 테스트 참고 |
| test_matching_cost_factors.cpp | factor error/linearize | MixtureLightNDT factor 단위 테스트 참고 |

테스트 활성화 방법: `cmake -DBUILD_TESTS=ON ..`

### 25.2 기존 벤치마크 인프라

`src/main.cpp`의 `MatchingCostFactorDemo` 클래스는 headless 모드(--headless)에서 6개 factor를 순차 실행하며 각 factor별 R error, t error를 출력한다. 동일 입력 데이터로 A/B 비교가 가능하다.

GMM 벤치마크 추가 방법:

1. `factor_types` 벡터에 "MixtureLightNDT" 추가
2. `create_factor()` 메서드에 `else if` 분기 추가
3. 동일 headless 루프에서 기존 6개 + GMM 1개를 같이 실행
4. R/t error 직접 비교 가능

### 25.3 GMM 전용 검증 실험

기존 벤치마크 외에 GMM 특화 검증이 필요한 지점:

| 검증 대상 | 방법 | 성공 기준 |
|-----------|------|-----------|
| EM 수렴 | voxel별 log-likelihood 추적 | monotonic increase |
| Component 품질 | weight 합 검사, cov eigenvalue 검사 | 합 근사 1.0, 모든 eigenvalue 양수 |
| Bimodal 재현 | synthetic 2-plane voxel | K=2일 때 두 평면 분리 |
| Single fallback | 점 수 부족 voxel | K=1로 자동 fallback |
| 기존 경로 불변 | 기존 6개 factor R/t error | 변화 없음 |

---

## 26. 보완: IncrementalCovarianceVoxelMap 참고

현재 코드에 `incremental_covariance_voxelmap.hpp`가 존재한다. 이 타입은 voxel 내부에서 online covariance estimation을 수행하며, 기존 `GaussianVoxel`의 "입력 covariance 평균" 방식과 달리 voxel 내부 점 위치의 sample covariance를 직접 계산한다.

이 구현은 GMM 구현에 직접 사용되지는 않지만, 다음 두 가지 이유로 참고 가치가 있다.

1. `IncrementalVoxelMap<T>` 템플릿에 커스텀 `VoxelContents`를 끼우는 실제 예제
2. voxel 내부 통계 처리를 `add()`/`finalize()` 패턴으로 구현하는 방법

---

## 27. 보완: 파일별 구현 우선순위와 의존 관계

### 27.1 의존 관계 그래프

```text
[Phase 0] 설계 결정 확정 (섹션 21)
    |
    v
[Phase 1] GMMVoxel + GMMVoxelMapCPU 타입
    |  파일: gmm_voxelmap_cpu.hpp, gmm_voxelmap_cpu.cpp
    |  의존: 없음 (기존 코드 수정 없음)
    |
    v
[Phase 2] EM Backend
    |  파일: mixture_em_backend.hpp, mixture_em_backend.cpp
    |  의존: Phase 1 (GMMVoxel struct 정의)
    |
    v
[Phase 2.5] Voxel Build/Finalize 통합 + 단위 테스트
    |  파일: gmm_voxelmap_cpu.cpp 내 finalize() 연결, test_gmm_voxelmap.cpp
    |  의존: Phase 1 + Phase 2
    |  CMakeLists.txt 수정: gtsam_points에 새 소스 추가
    |
    v
[Phase 3] MixtureLightNDT Factor
    |  파일: integrated_mixture_light_ndt_factor.hpp, impl/, .cpp
    |  의존: Phase 2.5 (안정적인 GMMVoxelMap)
    |  CMakeLists.txt 수정: factor 소스 추가
    |
    v
[Phase 3.5] Benchmark 연결 + 검증
    |  파일: src/main.cpp 수정
    |  의존: Phase 3
    |
    v
[Phase 4] Full Mixture NDT (조건부)
    |  의존: Phase 3.5 안정화
    |
    v
[Phase 5] KL merge/split (선택)
[Phase 6] Mixture-to-Mixture (최종 검토)
```

### 27.2 파일 단위 작업 순서

| 순서 | 파일 | 내용 | 의존 |
|:---:|------|------|------|
| 1 | types/gmm_voxelmap_cpu.hpp | GMMVoxel, MixtureComponent, GMMVoxelMapCPU 선언 | 없음 |
| 2 | types/gmm_voxelmap_cpu.cpp | GMMVoxel::add(), GMMVoxelMapCPU::insert()/lookup 구현 | 1 |
| 3 | types/mixture_em_backend.hpp | EM backend 인터페이스 선언 | 1 |
| 4 | types/mixture_em_backend.cpp | EM 구현 (Armadillo 래핑 or 직접) | 3 |
| 5 | types/gmm_voxelmap_cpu.cpp 보완 | finalize()에서 EM backend 호출 연결 | 2, 4 |
| 6 | CMakeLists.txt (gtsam_points) | 신규 cpp 3개 등록 | 2, 4 |
| 7 | 단위 테스트 작성 + EM 검증 | voxel build 품질 확인 | 5, 6 |
| 8 | factors/integrated_mixture_light_ndt_factor.hpp | factor 선언 | 1 |
| 9 | factors/impl/integrated_mixture_light_ndt_factor_impl.hpp | update_correspondences, evaluate 구현 | 8 |
| 10 | factors/integrated_mixture_light_ndt_factor.cpp | 명시적 인스턴스화 | 9 |
| 11 | CMakeLists.txt 보완 | factor cpp 등록 | 10 |
| 12 | src/main.cpp 수정 | GMM factor type 등록, benchmark 연결 | 10, 11 |
| 13 | 통합 검증 | headless benchmark 실행, R/t error 비교 | 12 |

---

## 28. 보완: 예상 리스크와 완화 전략

| 리스크 | 심각도 | 완화 전략 |
|--------|:------:|-----------|
| EM이 voxel 내 점 수 부족으로 degenerate | 높음 | 점 수 < 3*K이면 자동으로 K=1 fallback |
| Covariance가 singular | 높음 | regularization epsilon 추가 (cov += eps * I) |
| 매 insert마다 EM 비용 폭증 | 중간 | dirty flag + warm start + bounded reservoir |
| Armadillo-Eigen 타입 변환 오버헤드 | 낮음 | Eigen::Map zero-copy 매핑 활용 |
| Component responsibility 불안정 | 중간 | frozen responsibility + LightNDT quadratic 먼저 |
| 기존 factor 경로 regression | 높음 | 기존 코드 수정 최소화, sibling path 전략 |
| LRU 삭제 시 GMMVoxel reservoir 메모리 해제 | 낮음 | shared_ptr 기반이므로 자동 해제 |

---

## 29. 최종 업데이트된 구현 순서 (섹션 19 보완)

이전 섹션 19의 구현 순서를 다음과 같이 보완한다.

```text
0. 필수 설계 결정 확정 (섹션 21)
   - 타입 계층: GaussianVoxelMap 직접 상속 (sibling path, 선택지 B)
   - EM backend: Armadillo 프로토타입 -> 이후 Eigen 네이티브
   - Reservoir: bounded 256
   - K_max: 3 고정 + pruning
   - frame::traits: component 인덱스 노출 방식 (선택지 B)

1. GMMVoxel + GMMVoxelMapCPU 타입 추가
   - MixtureComponent struct
   - GMMVoxel struct (Setting 포함)
   - GMMVoxelMapCPU class
   - frame::traits<GMMVoxel> 특수화
   - 검증: 컴파일 성공, 기존 경로 불변

2. EM Backend 구현 + Voxel Build/Finalize 연결
   - mixture_em_backend.hpp/.cpp
   - GMMVoxel::add() -> reservoir 축적
   - GMMVoxel::finalize() -> dirty check -> EM 호출 -> component 확정
   - 검증: synthetic bimodal 데이터로 component 분리 확인

3. Voxel 단위 GMM Fitting 품질 검증
   - 단위 테스트: weight 합, cov positive definite, log-likelihood 개선
   - single fallback 검증
   - 검증: test_gmm_voxelmap 전체 통과

4. MixtureLightNDT Factor 구현
   - MixtureNdtCorrespondence struct
   - update_correspondences(): component responsibility 계산 + freeze
   - evaluate(): frozen responsibility 기반 weighted Mahalanobis
   - 검증: 단위 테스트 + factor error/linearize 일관성

5. Benchmark 연결 + Registration 검증
   - src/main.cpp에 MixtureLightNDT 등록
   - headless mode로 기존 6개 factor와 A/B 비교
   - known-transform, synthetic, multimodal scene
   - 검증: R/t error가 LightNDT 이하

6. 선택적 확장 (안정화 후)
   - KL merge/split
   - Full Mixture NDT
   - Mixture-to-Mixture (최종)
```
