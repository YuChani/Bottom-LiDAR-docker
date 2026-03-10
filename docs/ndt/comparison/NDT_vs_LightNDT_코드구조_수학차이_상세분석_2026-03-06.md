# NDT vs LightNDT 코드 구조와 수학 차이 상세 분석

## 1. 문서 목적

이 문서는 현재 프로젝트 구현 기준으로 NDT와 LightNDT의 차이를 다음 3축에서 정리한다.

1. 코드 구조 차이 (클래스/상태/계산 경로)
2. 수학 구조 차이 (목적함수, 도함수, Hessian)
3. 왜 이 차이가 수렴 속도/안정성 차이로 이어지는지

핵심은 "무슨 알고리즘 이름이냐"보다, **LM에 어떤 선형화 정보를 어떤 품질로 넘기느냐**다.

---

## 2. 공통 기반 (같은 점)

두 구현은 다음 구조를 공유한다.

- 둘 다 `IntegratedMatchingCostFactor`를 상속한다.
- 둘 다 `update_correspondences(delta)` -> `evaluate(delta, ...)` 흐름을 쓴다.
- 둘 다 voxel 이웃 탐색 모드 `DIRECT1/7/27`을 사용한다.
- 둘 다 voxel 공분산 역행렬 캐시(`inv_cov_cache`)를 사용한다.

근거:

- 공통 error/linearize 흐름: `thirdparty/gtsam_points/src/gtsam_points/factors/integrated_matching_cost_factor.cpp:32`, `thirdparty/gtsam_points/src/gtsam_points/factors/integrated_matching_cost_factor.cpp:40`
- NDT correspondence 경로: `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:98`
- LightNDT correspondence 경로: `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_light_ndt_factor_impl.hpp:90`

즉, "대응점 찾는 파이프라인"은 거의 같고, 진짜 차이는 **cost/weight/선형화**에 있다.

---

## 3. 코드 구조 차이

## 3.1 기본 파라미터 상태

NDT는 추가 상태를 가진다.

- `resolution`, `outlier_ratio`, `gauss_d1`, `gauss_d2`
- `compute_ndt_params(resolution, outlier_ratio, d1, d2)`를 correspondence 갱신 시 계산

근거:

- `thirdparty/gtsam_points/include/gtsam_points/factors/integrated_ndt_factor.hpp:67`
- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:119`

반면 LightNDT는 `d1/d2`와 outlier 파라미터 개념이 없다.

- 기본 상태는 `regularization_epsilon`, `search_mode`, tolerance 중심
- 비용 계산에 지수 score 파라미터를 쓰지 않는다.

근거:

- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_light_ndt_factor_impl.hpp:20`

## 3.2 correspondence 단계

둘 다 동일한 절차를 갖는다.

1. 변환된 소스 포인트의 voxel 좌표 계산
2. `DIRECT1/7/27` 이웃 후보 순회
3. 최소 Mahalanobis voxel 선택
4. 해당 voxel의 mean/inv_cov 저장

근거:

- NDT: `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:121`~`185`
- LightNDT: `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_light_ndt_factor_impl.hpp:109`~`173`

정리: correspondence는 사실상 동형이다.

## 3.3 evaluate 단계의 분기점

진짜 차이는 `evaluate()` 내부 cost/Hessian 생성 방식이다.

- NDT: score 기반 비선형 cost + 가중치(weight) 곱
  - `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:260`~`289`
- LightNDT: 순수 Mahalanobis 이차 cost
  - `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_light_ndt_factor_impl.hpp:228`~`248`

---

## 4. 수학 구조 차이

## 4.1 NDT 목적함수

NDT 구현은 다음 형태다.

```text
m = r^T Sigma^{-1} r
score = -d1 * exp(-d2/2 * m)
cost = -d1 - score
```

구현 근거:

- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:245`
- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:261`
- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:264`

파라미터 생성:

```text
d1, d2 = f(resolution, outlier_ratio)
```

- `thirdparty/gtsam_points/include/gtsam_points/factors/integrated_ndt_factor.hpp:67`~`73`

## 4.2 LightNDT 목적함수

LightNDT 구현은 scoreless 이차형이다.

```text
cost = r^T Sigma^{-1} r
```

근거:

- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_light_ndt_factor_impl.hpp:228`

## 4.3 gradient/Hessian 차이

NDT:

```text
weight = -d1 * d2 * exp(-d2/2 * m)
H ~= weight * J^T Sigma^{-1} J    (H1-only)
b   = weight * J^T Sigma^{-1} r
```

근거:

- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:280`
- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:282`~`289`

LightNDT:

```text
H = J^T Sigma^{-1} J
b = J^T Sigma^{-1} r
```

근거:

- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_light_ndt_factor_impl.hpp:241`~`248`

핵심 차이:

- NDT는 residual 크기에 따라 `weight`가 계속 변한다.
- LightNDT는 추가 지수 가중이 없어서 LM/GN 관점에서 더 단순하다.

---

## 5. 왜 성능/수렴 특성이 달라지는가

## 5.1 NDT가 느려지기 쉬운 이유

1. 지수형 항(`exp`)이 들어간 비선형 cost
2. residual 의존 weight로 유효 gradient가 상황에 따라 약해짐
3. H1-only 근사로 곡률 정보 일부 생략

결과적으로 LM에서 반복 수가 커지기 쉽다.

관련 코드:

- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:248`
- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:280`
- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:282`

## 5.2 LightNDT가 빠르기 쉬운 이유

1. 목적함수가 이차형이라 선형화 일관성이 높음
2. 추가 지수 가중/파라미터(d1,d2) 계산이 없음
3. correspondence는 NDT와 유사하지만, 최적화 대상 함수가 단순

결과적으로 outer iteration이 작게 나올 가능성이 크다.

관련 코드:

- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_light_ndt_factor_impl.hpp:228`
- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_light_ndt_factor_impl.hpp:244`

## 5.3 LM 상호작용 관점

이 프로젝트에서는 factor가 LM에 선형화 결과를 공급한다.

- `error()`도 correspondence를 갱신
- `linearize()`도 correspondence를 갱신

근거:

- `thirdparty/gtsam_points/src/gtsam_points/factors/integrated_matching_cost_factor.cpp:32`
- `thirdparty/gtsam_points/src/gtsam_points/factors/integrated_matching_cost_factor.cpp:40`

따라서 NDT/LightNDT 비교는 "이름"보다, 각 factor가 LM에 넘기는 `H,b`의 성질 차이로 보는 것이 정확하다.

---

## 6. 실무 해석 가이드

## 6.1 안전한 문장

- "현재 코드 조합에서 NDT는 느려지기 쉬운 구조"
- "LightNDT는 동일한 voxel correspondence를 쓰되 목적함수를 LM 친화적으로 단순화"

## 6.2 피해야 할 과장

- "NDT는 언제나 느리다" (일반화 과다)
- "LightNDT는 항상 더 정확하다" (데이터/초기값/튜닝 의존)

## 6.3 운영 권장

- 기본: LightNDT
- fallback/검증: GICP 또는 NDT
- 보고서 해석은 반드시 현재 설정 조건(headless/threads/tolerance/graph 구성) 한정으로 표기

---

## 7. 한 줄 결론

NDT와 LightNDT의 본질적 차이는 correspondence가 아니라 **cost와 선형화 방식**이며,
그 차이가 현재 LM 기반 파이프라인에서 반복 수/시간 차이로 직접 나타난다.
