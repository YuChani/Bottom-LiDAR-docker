# NDT vs LightNDT: 왜 이렇게 결정해야 하는가

## 이 문서의 질문

1. 현재 `IntegratedNDT`를 계속 기본으로 써야 하는가?
2. `LightNDT`로 전환하면 어떤 이득/리스크가 있는가?
3. "포기"가 아니라 "운영 전략"으로 어떻게 설계해야 하는가?

---

## 1) 결론 (Decision First)

현재 프로젝트와 측정값 기준으로는 다음이 가장 합리적이다.

> **기본 backend는 LightNDT로 전환하고, NDT는 fallback/검증용으로 유지한다.**

이 결론의 이유:

- 실측 성능 차이가 크고 일관적이다.
- NDT 느림은 단순 파라미터 실수가 아니라 구조적 성격이다.
- NDT를 완전히 제거하지 않고 보조 채널로 유지하면 리스크를 통제할 수 있다.

---

## 2) 데이터 기반 근거

10회 반복 결과(동일 조건, `--headless --threads 8`):

- NDT: `2557.1 ms`, outer iter `74`, mean T `0.101971`, mean R `0.697636`
- LightNDT: `335.4 ms`, outer iter `6`, mean T `0.159027`, mean R `0.640614`
- GICP: `658.0 ms`, outer iter `9`, mean T `0.085399`, mean R `0.499411`

NDT는 속도 측면에서 LightNDT 대비 약 `7.6x`, GICP 대비 약 `3.9x` 느리다.

원본: `/tmp/bench_compare_10runs_summary.csv`

---

## 3) 왜 NDT 기본 운용이 불리한가

### 3.1 수학/최적화 조합의 구조적 불리함

- NDT는 지수형 score 기반 비용
  - `integrated_ndt_factor_impl.hpp:260`~`264`
- 가변 weight(`-d1*d2*exp(...)`) 적용
  - `integrated_ndt_factor_impl.hpp:280`
- Hessian은 H1-only 근사
  - `integrated_ndt_factor_impl.hpp:282`~`286`

이 조합은 robust 성격은 주지만, 반복 수 증가에 취약하다.

### 3.2 LM 수용판정과의 결합

LM은 `modelFidelity = costChange / linearizedCostChange`를 사용한다.

- `levenberg_marquardt_ext.cpp:258`
- `levenberg_marquardt_ext.cpp:260`

비선형성이 강하고 correspondence가 자주 바뀌면,
로컬 모델 충실도가 낮아져 보수적 step이 반복된다.

### 3.3 correspondence 갱신 구조

`error()`와 `linearize()` 둘 다 correspondence를 갱신한다.

- `integrated_matching_cost_factor.cpp:36`
- `integrated_matching_cost_factor.cpp:43`

정확도 측면 장점이 있지만, 반복 중 모델 일관성/비용 측면에서 불리할 수 있다.

---

## 4) 왜 LightNDT를 기본으로 두는가

LightNDT는 correspondence는 NDT 계열을 유지하면서,
목적함수를 이차형으로 단순화했다.

- `cost = r^T * inv_cov * r`
  - `integrated_light_ndt_factor_impl.hpp:228`
- `H = J^T * inv_cov * J`, `b = J^T * inv_cov * r`
  - `integrated_light_ndt_factor_impl.hpp:244`~`248`

즉 LM/GN 친화성이 높아 반복 수가 낮아지고, 실제로 속도가 크게 개선된다.

---

## 5) "NDT 정석" 관점에서의 오해 정리

## 오해 1: 현재 NDT가 정석이 아니어서 느리다

부분적으로 맞고, 부분적으로 틀리다.

- `d1/d2`, score 구조는 Magnusson 계열과 정합적이다.
- 다만 최적화 엔진/선형화 정책(H1-only, LM 결합)은 PCL의 고전 루프와 다르다.

즉, "틀린 구현"이라기보다 "다른 목적(그래프 최적화 안정성) 우선 구현"이다.

## 오해 2: LightNDT는 PCL NDT와 같다

아니다.

- LightNDT: scoreless quadratic
- PCL NDT: score + 미분 + line search 계열(예: `computeDerivatives`, `computeStepLengthMT`)

---

## 6) 권장 운영 모델 (현실적 하이브리드)

## Stage 1: 즉시 적용

1. 기본 backend를 LightNDT로 설정
2. NDT 선택 스위치를 유지(비활성 기본값)
3. 로그에 backend별 `iter/time/error` 표준 출력 유지

## Stage 2: 안정성 확보

1. LightNDT 실패 조건 정의(예: 비정상 delta, error 폭증)
2. 실패 시 1회 GICP 또는 NDT fallback
3. fallback 빈도/성공률 모니터링

## Stage 3: 장기 개선

1. NDT 전용 coarse-to-fine 실험
2. 종료 조건 개선(`delta_x` 기반, 상대 개선율 기반)
3. 필요 시 NDT 구조개편(H2 일부 반영/신뢰영역 재설계)

---

## 7) 수용 기준(acceptance criteria)

운영 전환을 판단할 때 다음 기준을 권장한다.

1. p95 registration time 목표 달성 (예: 500ms 이하)
2. 수렴 실패율 목표 (예: 2% 이하)
3. 기준 데이터셋에서 R/T error 악화 허용치 이내
4. fallback 발생률 상한(예: 5% 이하)

이 기준을 넘기면 LightNDT 기본화는 실무적으로 타당하다.

---

## 8) 참고 레퍼런스

- Magnusson Thesis (NDT 기반):
  - http://www.diva-portal.org/smash/get/diva2:276162/FULLTEXT02.pdf
- PCL NDT 헤더/구현:
  - https://github.com/PointCloudLibrary/pcl/blob/master/registration/include/pcl/registration/ndt.h
  - https://github.com/PointCloudLibrary/pcl/blob/master/registration/include/pcl/registration/impl/ndt.hpp
- GICP 원논문:
  - https://www.roboticsproceedings.org/rss05/p21.pdf
- small_gicp (현대 구현 참고):
  - https://github.com/koide3/small_gicp
  - https://doi.org/10.21105/joss.06948
