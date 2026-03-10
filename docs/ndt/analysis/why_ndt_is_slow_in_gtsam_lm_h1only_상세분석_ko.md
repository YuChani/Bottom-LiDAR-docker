# 왜 현재 NDT가 느린가?

## 범위와 결론

이 문서는 현재 코드베이스의 NDT 조합:

- `IntegratedNDTFactor` (score 기반 NDT 비용)
- `LevenbergMarquardtOptimizerExt` (LM)
- `H1-only` Gauss-Newton Hessian 근사

에서 **왜 반복 수가 높고 시간이 길어지는지**를 "왜(Why)" 중심으로 설명한다.

핵심 결론은 다음 한 줄이다.

> 현재 조합은 "무조건 느리다"가 아니라, **느려지기 쉬운 구조**이며, 실제 실험에서 그 경향이 강하게 재현되었다.

---

## 1) 실험 증거: 느림은 일시 노이즈가 아니라 반복되는 패턴

최근 10회 반복 실험(`--headless --threads 8`) 집계:

- 원본 데이터: `/tmp/bench_compare_10runs_summary.csv`
- NDT: 평균 `2557.1 ms`, outer iter `74`, inner iter `74`
- GICP: 평균 `658.0 ms`, outer iter `9`, inner iter `9`
- LightNDT: 평균 `335.4 ms`, outer iter `6`, inner iter `16`

즉, 현재 환경에서는 NDT가 LightNDT 대비 약 `7.6x`, GICP 대비 약 `3.9x` 느리다.

이 차이는 단순한 1회성 변동이 아니라 반복 실험에서 안정적으로 관측된다.

---

## 2) 현재 NDT 구현의 수학적 구조

### 2.1 비용 함수

코드:

- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:260`
- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:264`

수식 형태:

```text
score = -d1 * exp(-d2/2 * m)
cost  = -d1 - score = -d1 * (1 - exp(-d2/2 * m))
m     = r^T * Sigma^{-1} * r
```

여기서 `r = mean_B - T*mean_A`.

### 2.2 가중치와 선형화

코드:

- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:280`
- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:282`

```text
weight = -d1 * d2 * exp(-d2/2 * m)
H ≈ weight * J^T * Sigma^{-1} * J   // H1 only
b  = weight * J^T * Sigma^{-1} * r
```

주석에도 명시되어 있듯 H2/H3는 생략되어 있고, PSD 보장을 위해 H1 항만 사용한다.

---

## 3) 왜 느려지는가 (Why Breakdown)

## Why A: 지수형 비용의 포화 구간

`exp(-d2/2 * m)` 구조는 `m`이 커질 때 빠르게 0에 가까워진다.

그 결과:

- 비용 변화량이 작아지고
- gradient 신호가 약해지고
- LM이 "충분히 줄었는지" 판단하기 어려워짐

즉, 초반 오정렬/어려운 구간에서 큰 전진이 잘 안 나와 반복 수가 늘어난다.

관련 코드:

- `integrated_ndt_factor_impl.hpp:248`
- `integrated_ndt_factor_impl.hpp:254`
- `integrated_ndt_factor_impl.hpp:264`

## Why B: 가중치가 잔차에 의존하며 쉽게 약해짐

`weight = -d1*d2*exp(...)` 이므로, `m`이 커지면 weight가 바로 줄어든다.

의미:

- 정렬이 덜 된 포인트의 영향력이 작아짐
- "틀린 상태를 빠르게 고치기 위한 힘"이 약해짐

이는 outlier 억제에는 유리할 수 있지만, 수렴 속도 관점에서는 불리한 경우가 많다.

관련 코드:

- `integrated_ndt_factor_impl.hpp:280`

## Why C: H1-only 근사로 인한 곡률 정보 손실

현재 구현은 H1만 사용한다.

- 장점: PSD 보장, 수치 안정성 확보
- 단점: 실제 비선형 곡률(H2/H3) 정보 소실

즉, 스텝 방향/크기가 보수적으로 되는 경향이 생기고, 많은 반복을 통해 천천히 내려가게 된다.

관련 코드:

- `integrated_ndt_factor_impl.hpp:282`
- `integrated_ndt_factor_impl.hpp:283`

## Why D: LM 내부에서 모델-현실 불일치가 쉽게 발생

LM은 `modelFidelity = costChange / linearizedCostChange`로 스텝 품질을 본다.

관련 코드:

- `thirdparty/gtsam_points/src/gtsam_points/optimizers/levenberg_marquardt_ext.cpp:258`
- `thirdparty/gtsam_points/src/gtsam_points/optimizers/levenberg_marquardt_ext.cpp:260`

비선형성이 강하고(H1-only 근사 포함) correspondence도 갱신되기 때문에,
"선형화가 예측한 감소량"과 "실제 감소량"이 벌어지기 쉽다.

그러면 LM이 보수적으로 움직여 반복 수가 증가한다.

## Why E: correspondence가 error/linearize 모두에서 갱신됨

관련 코드:

- `thirdparty/gtsam_points/src/gtsam_points/factors/integrated_matching_cost_factor.cpp:36`
- `thirdparty/gtsam_points/src/gtsam_points/factors/integrated_matching_cost_factor.cpp:43`

장점은 stale correspondence 방지지만,
한편으로는 "같은 스텝 평가" 내에서도 대응점 상태가 바뀌어 로컬 근사 일관성이 흔들릴 수 있다.

특히 NDT처럼 비용 표면이 예민한 경우 반복 증가를 유도할 수 있다.

---

## 4) 파라미터 관점에서 자주 생기는 오해

## 오해 1: outlier/resolution을 키우면 d1,d2가 커져 무조건 빨라진다

아니다. 핵심은 단순히 d1,d2 "크기"가 아니라 다음이다.

- `weight = -d1*d2*exp(...)`의 실제 유효 스케일
- `exp(...)`로 인한 포화/감쇠
- correspondence 갱신/LM 수용 조건과의 결합

즉, 일부 구간에서는 빨라질 수 있어도 전체 수렴이 자동으로 개선되지는 않는다.

## 오해 2: H2/H3를 넣으면 무조건 빨라진다

H2/H3는 정보를 늘리지만, indefinite 성분 때문에 안정성이 깨질 수 있다.

그래서 "속도 vs 안정성"의 교환 관계를 반드시 설계해야 한다.

---

## 5) 현재 코드에서 직접 확인되는 설정 포인트

- NDT 기본값(현재): `resolution=3.0`, `outlier_ratio=0.5`
  - `integrated_ndt_factor_impl.hpp:27`
  - `integrated_ndt_factor_impl.hpp:28`
- LM 공통 종료 조건:
  - `maxIterations=100`, `relativeErrorTol=1e-5`, `absoluteErrorTol=1e-5`
  - `src/main.cpp:675`
  - `src/main.cpp:676`
  - `src/main.cpp:677`

즉, 지금 벤치마크의 느림은 "옵티마이저를 잘못 썼다"보다
"비용-선형화-수용판정 조합"의 결과로 보는 것이 맞다.

---

## 6) 실무 결론

현재 조합에서 NDT는 다음 특성을 보인다.

- 장점: score 기반 robust 성격, 보수적 안정성
- 단점: 반복 수 증가에 취약, 벽시계 시간 증가

따라서 운영 전략은 보통 다음이 합리적이다.

1. 기본 backend는 LightNDT(속도 확보)
2. NDT는 검증/백업 backend로 유지
3. 필요 시 특정 실패 케이스에서만 NDT fallback

이 전략은 "NDT 포기"가 아니라 "NDT 역할 분리"다.

---

## 참고

- NDT 수식/코드 매핑: `docs/ndt/NDT_CODE_MATH_MAPPING.md`
- Magnusson 비교: `docs/ndt/magnusson_implementation_comparison_ko.md`
- LightNDT 비교: `docs/ndt/comparison/light_ndt_vs_ndt_구현_수학_비교_정리.md`
