# Mixture Model 기반 NDT 설계 상세 정리

이 문서는 현재 `Bottom-LiDAR-docker` 코드 기준으로, 왜 현재 NDT / LightNDT / VGICP 경로가 **single-Gaussian voxel map** 구조인지, 그리고 사용자가 말한 **"한 voxel 안에 여러 Gaussian distribution이 들어가는 true mixture model"**을 구현하려면 어떤 방향으로 설계를 바꿔야 하는지를 상세하게 정리한 문서다.

이 문서의 목적은 두 가지다.

1. 현재 코드가 무엇을 하고 있는지 헷갈리지 않게 정리하는 것
2. 앞으로 mixture model을 구현할 때 잘못된 방향으로 빠지지 않도록 기준 설계를 세우는 것

함께 보는 구조도 파일은 아래 경로에 있다.

- `artifacts/uml/mixture_model_ndt_flow.prisma`

이 `.prisma` 파일은 VSCode의 **Prisma Generate UML** 뷰어에서 열 수 있도록 만든 mixture-model 설계 구조도다. 이 구조도는 현재 single-Gaussian 경로와 앞으로 추가할 mixture 경로를 **병렬 비교**할 수 있게 구성했다.

---

## 1. 먼저 결론

현재 코드의 NDT는 **mixture-model NDT가 아니다.**

왜냐하면 현재 target map의 실제 표현은 `GaussianVoxelMapCPU`이고, 이 맵은 **voxel 하나당 Gaussian 하나(mean 하나, covariance 하나)**만 저장하기 때문이다.

즉 현재 구조는:

```text
voxel 하나
  -> mean 1개
  -> cov 1개
  -> intensity 1개
  -> num_points 1개
```

이다.

반면 사용자가 말한 mixture model은:

```text
voxel 하나
  -> component 1: (weight, mean, cov)
  -> component 2: (weight, mean, cov)
  -> component 3: (weight, mean, cov)
  -> ...
```

처럼 **한 voxel 안에 여러 Gaussian component가 공존**하는 구조다.

따라서 이걸 구현하려면 단순히 factor에서 주변 voxel 여러 개를 soft하게 고르는 정도로는 부족하고, **map-level representation 자체를 새로 만들어야 한다.**

---

## 2. 현재 코드의 실제 구조

현재 핵심 타입은 아래와 같다.

- `GaussianVoxel`
- `GaussianVoxelMapCPU`
- `IntegratedNDTFactor`
- `IntegratedLightNDTFactor`
- `IntegratedVGICPFactor`

이 구조를 한 줄로 요약하면:

```text
PointCloud
  -> GaussianVoxelMapCPU::insert()
  -> voxel 하나당 Gaussian 하나 생성
  -> NDT / LightNDT / VGICP가 그 단일 Gaussian을 target correspondence로 사용
```

즉 현재 repo는 처음부터 끝까지 **single-Gaussian per voxel** 가정을 공유하고 있다.

---

## 3. 현재 voxel map이 single-Gaussian인 이유

### 3.1 `GaussianVoxel` 자체가 단일 Gaussian 구조다

정의 위치:

- `thirdparty/gtsam_points/include/gtsam_points/types/gaussian_voxelmap_cpu.hpp`

현재 `GaussianVoxel`는 아래 정보를 가진다.

- `mean`
- `cov`
- `num_points`
- `intensity`
- `finalized`

여기서 중요한 점은 **component 배열이 없다는 것**이다.

즉 이 voxel은:

- Gaussian이 1개인지
- 2개인지
- 5개인지

를 표현하지 못한다.

그냥 하나의 평균과 하나의 공분산만 가진다.

### 3.2 `size() == 1`이라는 사실

`GaussianVoxel::size() const { return 1; }`

이 줄의 의미는 단순한 구현 디테일이 아니다. 현재 프레임/voxel trait 계층에서 `GaussianVoxel`는 **항상 대표점 하나만 가진다**는 전제를 뜻한다.

즉 현재 voxel 하나는 내부적으로 여러 component를 가진 mixture container가 아니라, **대표 Gaussian 하나를 가진 local distribution cell**이다.

### 3.3 `GaussianVoxelMapCPU`의 lookup API도 단일 결과만 반환한다

관련 함수:

- `lookup_voxel_index(coord)`
- `lookup_voxel(voxel_id)`

이 API는 다음 식이다.

```text
coord -> voxel index 하나
voxel index -> GaussianVoxel 하나
```

즉 현재 map API 자체가 다음을 지원하지 않는다.

- voxel 안 component 목록 가져오기
- component 수 조회
- component별 weight/mean/cov 조회

따라서 mixture map으로 가려면 factor보다 먼저 **map API가 바뀌어야** 한다.

---

## 4. 현재 NDT가 하는 일

### 4.1 현재 correspondence 구조

`IntegratedNDTFactor`는 `NdtCorrespondence`를 사용한다.

현재 `NdtCorrespondence`는 아래 필드만 가진다.

- `mean`
- `inv_cov`
- `one_over_Z`
- `valid`

즉 source point 하나에 대해 target 쪽 Gaussian 하나만 저장한다.

### 4.2 현재 correspondence 선택 방식

현재 NDT는 대략 아래처럼 동작한다.

```text
source point p
  -> transform해서 q' = R p + t
  -> q'가 속한 voxel 좌표 계산
  -> DIRECT1 / DIRECT7 / DIRECT27 범위의 neighbor voxel 후보 검사
  -> 각 voxel mean/cov에 대해 Mahalanobis distance 계산
  -> 그중 가장 작은 voxel 하나를 best_voxel로 선택
  -> 그 voxel의 mean / inv_cov / one_over_Z를 correspondence에 저장
```

핵심은 **best voxel 하나만 남긴다**는 것이다.

즉 지금 NDT는:

- 여러 hypothesis를 유지하지 않고
- 여러 component를 동시에 적분하지 않고
- 하나의 Gaussian correspondence로 collapse한다.

### 4.3 현재 cost의 의미

현재 residual은 본질적으로 transformed source point와 target Gaussian mean 사이의 차이로 계산되고, Mahalanobis distance는

\[
r^T \Sigma^{-1} r
\]

형태를 따른다.

즉 현재 cost는 “source point가 선택된 target Gaussian 하나에 얼마나 잘 들어맞는가”를 본다.

이건 mixture likelihood가 아니라 **single Gaussian likelihood**다.

---

## 5. LightNDT가 mixture가 아닌 이유

`IntegratedLightNDTFactor`도 target map으로 `GaussianVoxelMapCPU`를 사용하고, correspondence도 `NdtCorrespondence`를 그대로 사용한다.

즉 LightNDT는:

- map 구조는 NDT와 같고
- correspondence 구조도 NDT와 같고
- 단지 score 함수가 더 단순한 quadratic Mahalanobis LSQ라는 점만 다르다.

따라서 LightNDT를 쓴다고 해서 mixture model이 들어가는 것은 아니다.

---

## 6. VGICP도 mixture map이 아닌 이유

`IntegratedVGICPFactor`도 target으로 `GaussianVoxelMapCPU`를 사용한다.

차이는 correspondence 캐시가 `NdtCorrespondence`가 아니라:

- `const GaussianVoxel*`

라는 점이다.

하지만 이 역시 source point 하나당 target voxel 하나를 잡는 구조다. 이후에는 source covariance와 target voxel covariance를 합쳐 fused covariance를 만들고 Mahalanobis를 계산한다.

즉 VGICP도 결국:

- voxel 하나
- target Gaussian 하나

를 기준으로 계산한다.

그래서 현재 repo 전체는 NDT/LightNDT/VGICP가 모두 같은 single-Gaussian voxel map 위에 서 있다고 보면 된다.

---

## 7. 사용자가 말한 mixture model은 정확히 무엇인가

사용자가 말한 mixture model은 정확히는 다음 개념에 가깝다.

### 7.1 single Gaussian voxel

현재 구조:

\[
\text{voxel} \Rightarrow (\mu, \Sigma)
\]

즉 voxel 하나가 하나의 Gaussian으로 대표된다.

### 7.2 mixture voxel

사용자가 원하는 구조:

\[
\text{voxel} \Rightarrow \{(\pi_k, \mu_k, \Sigma_k)\}_{k=1}^{K}
\]

여기서:

- \(\pi_k\): component weight
- \(\mu_k\): component mean
- \(\Sigma_k\): component covariance

이다.

즉 voxel 하나가 하나의 분포가 아니라 **가중합된 여러 Gaussian 분포의 집합**으로 표현된다.

### 7.3 왜 이런 구조가 필요한가

single Gaussian은 voxel 안의 여러 구조를 하나로 평균내어 collapse한다.

예를 들어 voxel 안에:

- 서로 다른 두 평면이 지나가거나
- 얇은 구조 둘이 겹치거나
- 멀티모달한 표면 샘플이 존재하면

하나의 mean/cov로는 그 구조를 제대로 표현하지 못한다.

이럴 때 mixture model은 “같은 voxel 안에도 실제로는 여러 모드가 있다”는 것을 표현할 수 있다.

---

## 8. map-level mixture와 factor-level mixture의 차이

이 부분이 가장 중요하다.

### 8.1 factor-level mixture

이건 map은 그대로 두고 factor만 바꾸는 방식이다.

예를 들면:

- 주변 voxel 여러 개를 보고
- 각각의 single Gaussian에 대해 score를 계산한 뒤
- softmax나 weighted sum으로 섞는 방식

이다.

이건 구현은 상대적으로 쉽지만, **한 voxel 안의 multimodality를 직접 표현하지는 못한다.**

즉 이건 “여러 voxel 중 어떤 voxel이 더 맞는가”를 부드럽게 처리하는 것이지, “voxel 내부에 여러 Gaussian이 있다”는 개념은 아니다.

### 8.2 map-level mixture

이건 target map 자체를 mixture model로 바꾸는 방식이다.

즉:

```text
voxel 하나
  -> component 1
  -> component 2
  -> component 3
```

처럼 환경 모델 자체가 multi-modal해진다.

이것이 사용자가 말한 “한 voxel 안에 여러 Gaussian distribution이 있는 방식”과 정확히 맞는다.

### 8.3 결론

사용자 의도와 일치하는 것은 **map-level mixture**다.

factor-level mixture는 구현 실험용 근사나 soft association에는 쓸 수 있지만, 그걸 true mixture voxel map이라고 부르면 안 된다.

---

## 9. 왜 현재 구조를 조금만 바꿔서는 안 되는가

현재 repo에서는 single-Gaussian 가정이 여러 층에 박혀 있다.

1. `GaussianVoxel`
   - voxel당 mean/cov 하나
2. `GaussianVoxelMapCPU`
   - coord당 voxel 하나
3. `lookup_voxel()` API
   - 결과 하나
4. `NdtCorrespondence`
   - correspondence당 mean/inv_cov 하나
5. `IntegratedNDTFactor`
   - best voxel 하나 선택
6. `IntegratedLightNDTFactor`
   - same path
7. `IntegratedVGICPFactor`
   - voxel pointer 하나 선택

즉 single-Gaussian 전제가 factor 하나에만 있는 것이 아니라:

- map type
- lookup API
- correspondence cache
- evaluation loop

전체에 걸쳐 있다.

그래서 mixture model은 patch가 아니라 **새 아키텍처 경로**로 가는 것이 맞다.

---

## 10. true mixture-model NDT를 위해 필요한 새 구조

가장 안전한 방향은 기존 구조를 유지한 채 새 타입을 병행 추가하는 것이다.

즉 기존:

- `GaussianVoxel`
- `GaussianVoxelMapCPU`
- `IntegratedNDTFactor`

는 건드리지 않고, 별도로 예를 들면:

- `GaussianComponent`
- `GaussianMixtureVoxel`
- `GaussianMixtureVoxelMapCPU`
- `IntegratedMixtureLightNDTFactor`
- `IntegratedMixtureNDTFactor`

같은 계층을 만드는 것이다.

이렇게 해야 기존 benchmark와 비교도 가능하고, 현재 working path도 깨지지 않는다.

---

## 11. 새 voxel 타입은 무엇을 가져야 하는가

### 11.1 component 수준 필드

component 하나는 최소한 아래를 가져야 한다.

- `weight` 또는 `pi`
- `mean`
- `cov`
- `inv_cov`
- `one_over_Z`
- `num_points`
- 필요하면 `intensity`

즉 수학적으로는:

\[
\mathcal{C}_k = (\pi_k, \mu_k, \Sigma_k)
\]

이다.

### 11.2 voxel 수준 필드

voxel은 아래를 가져야 한다.

- `std::vector<GaussianComponent>` 또는 bounded component array
- component 개수
- fitting state
- touched / finalized 상태

즉 voxel이 단일 Gaussian이 아니라 **component 집합의 컨테이너**가 된다.

---

## 12. mixture voxel map을 만들려면 insert 단계가 어떻게 바뀌어야 하는가

현재 `IncrementalVoxelMap<GaussianVoxel>::insert()`는 점들을 하나의 Gaussian summary로 collapse한다.

이 구조로는 multimodality가 이미 사라진다.

따라서 true mixture model을 하려면 voxel build가 다음 단계로 바뀌어야 한다.

```text
PointCloud 입력
  -> voxel coord 계산
  -> 해당 voxel의 raw sample buffer 또는 bounded reservoir에 점 추가
  -> voxel별 local clustering / split-init
  -> voxel 내부 EM 또는 유사 fitting 수행
  -> K개의 Gaussian component 생성
  -> prune / merge / finalize
```

즉 핵심은:

- 지금처럼 sum만 쌓는 것이 아니라
- voxel 내부 sample structure를 잠깐 보존해야 하고
- 그 샘플로부터 mixture component를 fitting해야 한다

는 점이다.

---

## 13. EM과 local fitting은 어떤 의미인가

### 13.1 EM이란

EM(Expectation-Maximization)은 latent assignment를 직접 모를 때 반복적으로 mixture parameter를 추정하는 방식이다.

간단히 말하면:

1. 각 점이 어떤 component에 속할 확률을 계산하고
2. 그 확률로 component의 weight / mean / covariance를 다시 추정한다

를 반복한다.

### 13.2 voxel 내부에서의 의미

voxel 안의 점 집합을 \(x_i\)라고 하면, component \(k\)에 대한 responsibility는 보통:

\[
\gamma_{ik} = \frac{\pi_k \mathcal{N}(x_i \mid \mu_k, \Sigma_k)}{\sum_j \pi_j \mathcal{N}(x_i \mid \mu_j, \Sigma_j)}
\]

로 계산된다.

그 다음 M-step에서는:

\[
N_k = \sum_i \gamma_{ik}
\]

\[
\pi_k = \frac{N_k}{N}
\]

\[
\mu_k = \frac{1}{N_k}\sum_i \gamma_{ik} x_i
\]

\[
\Sigma_k = \frac{1}{N_k}\sum_i \gamma_{ik}(x_i-\mu_k)(x_i-\mu_k)^T
\]

같은 식으로 업데이트한다.

즉 voxel 하나 안에서 “점들이 실제로 몇 개의 모드를 이루는가”를 추정하는 과정이다.

---

## 14. mixture NDT의 correspondence는 어떻게 바뀌어야 하는가

현재 NDT는 source point 하나당 best voxel 하나를 골라 `NdtCorrespondence` 하나를 만든다.

mixture NDT는 이렇게 하면 안 된다.

대신 source point를 transform한 점 \(q'\)에 대해, 해당 voxel의 각 component \(k\)에 대해 score를 계산해야 한다.

예를 들면:

\[
s_k = \log \pi_k - \frac{1}{2}(q' - \mu_k)^T \Sigma_k^{-1}(q' - \mu_k) - \frac{1}{2}\log |\Sigma_k|
\]

이 score를 바탕으로 responsibility를 만들 수 있다.

\[
\gamma_k = \text{softmax}(s_k)
\]

즉 correspondence는 더 이상:

- mean 하나
- inv_cov 하나

가 아니라,

- component 목록
- 각 component의 weight/responsibility

를 가져야 한다.

---

## 15. mixture cost는 어떻게 생각해야 하는가

### 15.1 single Gaussian일 때

현재 구조는 사실상:

\[
\text{cost}(q') \sim r^T \Sigma^{-1} r
\]

또는 NDT score 형태로 하나의 Gaussian에 대한 정합도를 본다.

### 15.2 mixture일 때

true mixture라면 likelihood는 보통:

\[
p(q') = \sum_k \pi_k \mathcal{N}(q' \mid \mu_k, \Sigma_k)
\]

가 된다.

따라서 비용은 자연스럽게:

\[
-\log \sum_k \pi_k \mathcal{N}(q' \mid \mu_k, \Sigma_k)
\]

형태가 된다.

이게 중요한 이유는, mixture는 “각 component cost를 따로 계산해서 그냥 더하는 것”과 같지 않기 때문이다.

즉:

- weighted sum of residuals
- sum of component costs

와

- negative log of weighted sum of likelihoods

는 다르다.

이 차이를 놓치면 mixture처럼 보이지만 실제로는 확률적으로 일관되지 않은 cost를 만들게 된다.

---

## 16. 왜 `log-sum-exp`가 필요한가

mixture likelihood를 로그 공간에서 계산하면 보통:

\[
\log \sum_k e^{a_k}
\]

형태가 나온다.

이걸 그냥 계산하면 수치적으로 overflow / underflow가 나기 쉽다.

그래서 보통:

\[
\log \sum_k e^{a_k} = m + \log \sum_k e^{a_k - m}
\]

여기서 \(m = \max_k a_k\)를 사용해 안정적으로 계산한다.

이게 바로 `log-sum-exp`다.

mixture NDT에서 component score들을 합칠 때 이 기법이 필요하다.

---

## 17. responsibility를 evaluate마다 다시 계산하면 안 되는 이유

이건 optimizer 관점에서 매우 중요하다.

현재 VGICP도 voxel correspondence discontinuity 때문에 correspondence를 freeze하고 Mahalanobis만 업데이트하는 경로를 가지고 있다.

mixture에서도 비슷한 문제가 더 강하게 나타난다. 만약 LM의 trial step마다:

- component 선택이 바뀌고
- responsibility가 크게 튀고
- best mode가 뒤집히면

cost surface가 매우 불연속적이거나 너무 급격하게 바뀌게 된다.

그래서 mixture factor를 설계할 때는 보통:

- `update_correspondences()` 시점에서 component set / responsibility를 정하고
- `evaluate()`에서는 그걸 freeze한 상태로 local quadratic approximation을 수행하는 쪽이 더 안전하다.

즉 현재 코드 철학으로 보면, **MixtureLightNDT는 frozen responsibility 형태로 먼저 구현하는 것이 가장 안전한 출발점**이다.

---

## 18. 권장 구현 순서

현재 코드 기준으로 가장 안전한 순서는 아래와 같다.

### 18.1 1단계: map 타입 추가

먼저 새 mixture voxel map 타입을 만든다.

예:

- `GaussianComponent`
- `GaussianMixtureVoxel`
- `GaussianMixtureVoxelMapCPU`

이 단계에서는 기존 `GaussianVoxelMapCPU`는 건드리지 않는다.

### 18.2 2단계: mixture map build 구현

voxel 내부 raw samples 또는 bounded reservoir를 바탕으로 component fitting을 구현한다.

이 단계의 핵심은:

- component initialization
- EM 또는 유사 update
- component pruning / merge
- finalize

이다.

### 18.3 3단계: MixtureLightNDT부터 구현

이게 가장 안전하다.

이유는:

- correspondence 구조는 바뀌지만
- score 수식이 상대적으로 단순해서
- mixture map이 제대로 동작하는지 확인하기 쉽기 때문이다.

### 18.4 4단계: MixtureNDT 구현

그 다음에 score-shaped NDT objective로 확장한다.

### 18.5 5단계: MixtureVGICP는 마지막

VGICP는 source covariance fusion까지 겹치므로 가장 복잡하다. 처음부터 같이 넣지 않는 편이 안전하다.

---

## 19. 하면 안 되는 것

이건 꼭 별도로 적어둘 필요가 있다.

### 19.1 주변 voxel 여러 개를 soft하게 섞는 것을 true mixture voxel이라고 부르면 안 된다

그건 factor-level mixture 또는 soft association이지, voxel 내부 mixture가 아니다.

### 19.2 기존 `mean/cov/num_points`만으로 mixture를 복원하려고 하면 안 된다

이미 단일 Gaussian으로 collapse된 통계만으로는 멀티모달 구조를 복원할 수 없다.

### 19.3 기존 NDT factor에 억지로 component vector만 추가해서 끝내면 안 된다

map API, lookup API, correspondence cache, evaluate path가 다 single-Gaussian 전제를 갖고 있기 때문이다.

### 19.4 NDT / LightNDT / VGICP를 한 번에 mixture로 바꾸면 안 된다

검증이 매우 어려워진다.

---

## 20. 용어 해설

### 20.1 GMM (Gaussian Mixture Model)

여러 개의 Gaussian 분포를 가중합으로 섞어서 하나의 확률분포를 표현하는 모델이다.

### 20.2 mixture component

mixture model을 이루는 개별 Gaussian 하나다. 각 component는 보통 weight, mean, covariance를 가진다.

### 20.3 weight / mixing coefficient

각 component가 전체 mixture에서 차지하는 비율이다. 보통 \(\pi_k\)로 표기한다.

### 20.4 responsibility

어떤 점이 특정 component에서 왔을 사후확률이다. soft assignment라고 생각하면 된다.

### 20.5 EM

mixture model의 latent assignment를 직접 모르기 때문에, component 확률과 파라미터를 번갈아 갱신하는 반복 추정 알고리즘이다.

### 20.6 Mahalanobis distance

공분산을 고려한 거리다.

\[
d_M^2 = r^T \Sigma^{-1} r
\]

### 20.7 map-level mixture

지도 표현 자체가 여러 Gaussian component를 포함하는 구조다.

### 20.8 factor-level mixture

지도는 그대로 두고 정합 factor나 likelihood 쪽에서 여러 가설을 섞는 구조다.

### 20.9 frozen correspondence

optimization step마다 correspondence를 다시 뒤집지 않고, 선형화 시점의 correspondence를 고정해서 쓰는 방식이다.

### 20.10 log-sum-exp

로그 공간에서 여러 likelihood를 수치적으로 안정하게 합치는 기법이다.

---

## 21. 최종 정리

현재 repo는:

- voxel 하나당 Gaussian 하나
- source point 하나당 target Gaussian 하나

라는 구조를 전제로 한다.

따라서 사용자가 말한 mixture model을 제대로 구현하려면:

1. **새 mixture voxel map 타입을 만들고**
2. **voxel 내부에서 component fitting을 수행하고**
3. **correspondence를 single Gaussian이 아니라 component 집합으로 바꾸고**
4. **mixture likelihood에 맞는 factor를 새로 만들어야 한다.**

즉 이건 단순한 factor tweak가 아니라, **map-level representation부터 바뀌는 새 경로**다.

이 결론을 기준으로 다음 구현을 진행하면, 예전에 제거한 잘못된 GMM-NDT처럼 “실제로는 mixture map이 아닌데 mixture처럼 보이는 경로”에 다시 매몰되지 않을 수 있다.

---

## 22. 관련 코드 위치

- `thirdparty/gtsam_points/include/gtsam_points/types/gaussian_voxelmap_cpu.hpp`
- `thirdparty/gtsam_points/src/gtsam_points/types/gaussian_voxelmap_cpu.cpp`
- `thirdparty/gtsam_points/include/gtsam_points/factors/integrated_ndt_factor.hpp`
- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp`
- `thirdparty/gtsam_points/include/gtsam_points/factors/integrated_light_ndt_factor.hpp`
- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_light_ndt_factor_impl.hpp`
- `thirdparty/gtsam_points/include/gtsam_points/factors/integrated_vgicp_factor.hpp`
- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_vgicp_factor_impl.hpp`
- `thirdparty/gtsam_points/include/gtsam_points/factors/integrated_matching_cost_factor.hpp`
- `src/main.cpp`

---

## 23. 구조도에서 어떻게 읽어야 하는가

이번에 같이 만든 UML 구조도는 현재 경로와 새 경로를 한 화면에서 비교하는 용도로 설계했다.

구조도에서 보는 핵심 축은 두 개다.

### 23.1 현재 경로

현재 경로는 아래처럼 읽으면 된다.

```text
PointCloud
  -> GaussianVoxelMapCPU
  -> GaussianVoxel
  -> IntegratedNDTFactor
```

즉 현재 repo는:

- point cloud를 voxel map에 넣고
- voxel 하나를 Gaussian 하나로 요약한 뒤
- factor가 그 single Gaussian을 correspondence로 소비한다.

### 23.2 새 mixture 경로

새 경로는 아래처럼 읽으면 된다.

```text
PointCloud
  -> GaussianMixtureVoxelMapCPU
  -> GaussianMixtureVoxel
  -> MixtureComponent[]
  -> IntegratedMixtureNDTFactor
```

그리고 그 중간에는 단순한 summary accumulation이 아니라:

```text
SampleReservoir
  -> MixtureInitializer
  -> ComponentMerger
  -> EMEstimator
```

가 들어간다.

즉 이 구조도는 “단일 Gaussian path”와 “mixture fitting path”가 어디서 갈라지는지를 보여주는 설계도다.

---

## 24. 새 mixture 경로에서 각 설계 요소의 역할

구조도에 나온 새 타입들의 역할은 아래처럼 보면 된다.

### 24.1 `GaussianMixtureVoxelMapCPU`

mixture target map의 최상위 컨테이너다.

역할:

- voxel resolution 관리
- coord -> mixture voxel lookup
- mixture voxel들의 집합 보관
- factor가 참조할 target map 역할 수행

즉 현재 `GaussianVoxelMapCPU`의 mixture 버전이다.

### 24.2 `GaussianMixtureVoxel`

voxel 하나를 나타내는 mixture container다.

역할:

- voxel 좌표에 속한 샘플 관리
- component fitting 결과 보관
- finalized mixture representation 제공

현재 `GaussianVoxel`와 달리, single mean/cov가 아니라 여러 component를 가진다.

### 24.3 `MixtureComponent`

mixture를 이루는 개별 Gaussian 성분이다.

역할:

- `weight`
- `mean`
- `cov`
- `inv_cov`
- `one_over_Z`

를 저장한다.

즉 registration에서 실제 likelihood 계산에 들어가는 최소 단위다.

### 24.4 `SampleReservoir`

voxel 안 raw sample들을 모아두는 저장소다.

왜 필요한가?

현재 single Gaussian 경로처럼 바로 sum/average만 하면 multimodality 정보가 사라지기 때문이다. mixture fitting을 하려면 voxel 안 샘플 구조를 어느 정도 유지해야 한다.

### 24.5 `MixtureInitializer`

EM 전에 component 초기값을 잡는 단계다.

역할:

- 초기 component 수 설정
- 초기 mean/cov seed 생성
- split 기반 초기화 또는 cluster 기반 초기화 수행

### 24.6 `ComponentMerger`

너무 비슷한 component들을 병합하거나 pruning하는 단계다.

여기서 KL divergence 같은 기준이 들어갈 수 있다. 즉 이 단계는 “mixture를 추정하는 본체”라기보다, component 구조를 정리하는 단계다.

### 24.7 `EMEstimator`

최종 mixture parameter를 추정하는 단계다.

역할:

- responsibility 계산
- weight/mean/covariance 반복 갱신
- convergence 판단

즉 mixture model fitting의 핵심이다.

### 24.8 `MixtureResponsibilityMatrix`

factor가 source point와 component 사이의 responsibility를 보관하는 캐시다.

중요한 점은 이걸 map이 아니라 **factor 쪽이 소유**한다는 것이다. 그래야 linearization point 기준으로 freeze된 responsibility를 유지할 수 있다.

### 24.9 `IntegratedMixtureNDTFactor`

mixture voxel map을 이용해 registration cost를 계산하는 새 factor다.

역할:

- voxel/component lookup
- component likelihood 계산
- responsibility freeze/update
- mixture-aware residual/Hessian/gradient 계산

즉 기존 `IntegratedNDTFactor`의 mixture-aware sibling path다.

---

## 25. 구현 순서를 구조도 기준으로 다시 쓰면

구조도 기준으로 구현 순서를 다시 쓰면 아래와 같다.

```text
PointCloud 입력
  -> GaussianMixtureVoxelMapCPU::insert()
  -> GaussianMixtureVoxel 내부 SampleReservoir 축적
  -> MixtureInitializer로 초기 component 생성
  -> ComponentMerger로 유사 component 정리
  -> EMEstimator로 최종 (pi_k, mu_k, Sigma_k) 추정
  -> finalized GaussianMixtureVoxelMapCPU 완성
  -> IntegratedMixtureNDTFactor가 component likelihood 계산
  -> MixtureResponsibilityMatrix에 responsibility 고정
  -> mixture-aware NDT cost 평가
  -> LM / GN / Newton 최적화
```

즉 현재 single path에서는 voxel build와 factor evaluation 사이에 “단일 Gaussian summary”만 있었지만, 새 path에서는 그 사이에 **초기화/병합/EM/freeze-responsibility** 단계가 추가된다.

---

## 26. 최종 설계 원칙

이번 구조도의 설계 원칙은 아래와 같다.

1. 기존 `GaussianVoxelMapCPU` 경로는 유지한다.
2. mixture는 새 sibling path로 추가한다.
3. map-level mixture와 factor-level cache를 분리한다.
4. KL divergence는 정리/병합 단계에 둔다.
5. EM은 최종 mixture parameter 추정 단계에 둔다.
6. optimizer는 기존 GTSAM/LM 계열 루프를 재사용하되, cost만 mixture-aware 하게 바꾼다.

이 원칙을 지키면 현재 코드와의 비교가 쉬워지고, 잘못된 “가짜 mixture” 경로로 새 구현이 흘러가는 것도 막을 수 있다.
