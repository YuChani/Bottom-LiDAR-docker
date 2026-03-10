# LightNDT 수학 구조와 속도 특성: Why 중심 상세 분석

## 문서 목적

이 문서는 다음 질문에 답한다.

1. LightNDT의 수학 구조는 정확히 무엇인가?
2. 왜 현재 환경에서 LightNDT가 NDT보다 훨씬 빠른가?
3. 빠른 대신 어떤 리스크를 감수하는가?

분석 기준은 현재 구현 코드이다.

- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_light_ndt_factor_impl.hpp`
- 비교 대상: `integrated_ndt_factor_impl.hpp`, `integrated_gicp_factor_impl.hpp`

---

## 1) LightNDT의 핵심 아이디어

LightNDT는 "NDT correspondence(복셀 분포)"는 유지하되,
NDT score의 지수형 항(`exp`, `d1`, `d2`)을 제거한 **scoreless Mahalanobis LSQ**다.

즉,

- 대응점 모델: point-to-voxel-distribution
- 목적함수: 순수 이차형
- 선형화: 표준 `J^T W J`, `J^T W r`

---

## 2) 수학 구조 (코드 1:1 대응)

## 2.1 Residual

코드:

- `integrated_light_ndt_factor_impl.hpp:225`
- `integrated_light_ndt_factor_impl.hpp:226`

```text
r_i = mean_B - T * mean_A
```

## 2.2 Cost

코드:

- `integrated_light_ndt_factor_impl.hpp:228`

```text
E_i = r_i^T * Sigma_B^{-1} * r_i
E   = Σ_i E_i
```

여기서 `Sigma_B^{-1}`는 타겟 복셀 공분산의 역행렬이다.

## 2.3 Jacobian / Hessian / Gradient

코드:

- Jacobian 구성: `integrated_light_ndt_factor_impl.hpp:233`~`239`
- Hessian/Gradient 누적: `integrated_light_ndt_factor_impl.hpp:241`~`248`

```text
H += J^T * Sigma_B^{-1} * J
b += J^T * Sigma_B^{-1} * r
```

중요: NDT와 달리 `weight = f(exp(...))` 스칼라가 없다.

---

## 3) 대응점(association) 구조

LightNDT는 per-point로 다음을 수행한다.

1. 소스 포인트를 현재 pose로 변환
2. 해당 voxel 좌표 계산
3. `DIRECT1/7/27` 이웃 후보 탐색
4. Mahalanobis 거리가 최소인 voxel 선택

코드:

- search mode: `integrated_light_ndt_factor_impl.hpp:109`~`129`
- 후보 탐색/최소 선택: `integrated_light_ndt_factor_impl.hpp:152`~`167`
- correspondence 저장: `integrated_light_ndt_factor_impl.hpp:170`~`173`

즉, correspondence 레벨에서는 NDT와 거의 동일한 파이프라인을 공유한다.

---

## 4) 왜 빠른가? (Why)

## Why A: 지수 score 계산/미분 체인이 없음

NDT는 `exp(-d2/2*m)`와 `weight=-d1*d2*exp(...)`를 사용한다.

- `integrated_ndt_factor_impl.hpp:254`
- `integrated_ndt_factor_impl.hpp:280`

LightNDT는 이 체인을 제거하고 바로 이차형을 최소화한다.

효과:

- 비용 표면이 단순
- 선형화와 실제 비용의 괴리가 상대적으로 작음
- LM의 step 수용이 더 안정적

## Why B: GN/LM이 다루기 쉬운 목적함수

LightNDT는 본질적으로 weighted least squares라,
LM이 가정하는 로컬 이차 모델과 잘 맞는다.

반면 NDT는 지수형 score 때문에 같은 LM에서도 반복이 늘기 쉽다.

## Why C: 가중치 급감으로 인한 "초반 추진력 손실"이 없음

NDT는 큰 잔차에서 `exp(...)`가 작아져 update 신호가 약해지기 쉽다.

LightNDT는 그런 감쇠 스칼라를 사용하지 않아,
초기 정렬 오차가 있을 때도 비교적 꾸준한 gradient를 유지한다.

---

## 5) LightNDT와 GICP는 같은가?

짧게 말하면 **완전히 같지는 않다.**

## 공통점

- 둘 다 scoreless Mahalanobis LSQ
- 둘 다 `H = J^T W J`, `b = J^T W r` 구조

코드:

- LightNDT: `integrated_light_ndt_factor_impl.hpp:244`~`248`
- GICP: `integrated_gicp_factor_impl.hpp:282`~`286`

## 차이점(핵심)

1. `W` 정의
   - LightNDT: `W = Sigma_voxel^{-1}`
   - GICP: `W = (Sigma_B + R*Sigma_A*R^T)^{-1}`
     - `integrated_gicp_factor_impl.hpp:260`~`263`

2. correspondence 모델
   - LightNDT: voxel 후보(DIRECT1/7/27)
   - GICP: kd-tree NN point correspondence
     - `integrated_gicp_factor_impl.hpp:168`

즉 "로컬 선형대수 형태는 유사"하지만, 모델링 대상과 정보행렬이 다르다.

---

## 6) 빠른 대신 어떤 리스크가 있나?

## 리스크 A: NDT score 기반 robust 성격 감소

LightNDT는 exp score 기반 down-weighting을 쓰지 않는다.
따라서 outlier/강한 오정렬 상황에서 robust 성격은 약해질 수 있다.

## 리스크 B: voxel 통계 품질 의존

LightNDT는 타겟 voxel 공분산에 강하게 의존한다.

- 공분산이 나쁘거나
- voxel 해상도가 데이터와 안 맞으면

빠르게 수렴해도 잘못된 로컬 해에 들어갈 수 있다.

## 리스크 C: "빠른데 틀린" 상태를 놓치기 쉬움

속도만 보면 LightNDT가 우세하지만,
실제 운영에서는 실패 감지 지표(예: 오차 급증, score 급락, 비정상 delta)와 함께 써야 한다.

---

## 7) 현재 실험값으로 본 해석

10회 반복 결과(동일 조건):

- LightNDT: `335.4 ms`, outer iter `6`
- NDT: `2557.1 ms`, outer iter `74`
- GICP: `658.0 ms`, outer iter `9`

해석:

- LightNDT가 빠른 이유는 correspondence가 가벼워서가 아니라,
  **목적함수/선형화 구조가 LM 친화적이기 때문**이 더 크다.
- NDT가 느린 이유는 계산량 절대치보다 반복 수 증가가 지배적이다.

---

## 8) 실무 적용 권장

1. 기본 backend는 LightNDT로 운용
2. NDT는 고난도 구간 fallback 또는 회귀 검증용으로 유지
3. 운영 지표에 "속도 + 실패율 + 자세오차"를 같이 본다

즉, LightNDT는 "NDT의 완전 대체"보다
"현실적인 기본 엔진"으로 두고 NDT를 전략적으로 병행하는 것이 안정적이다.

---

## 참고

- NDT 수식 원형: `docs/ndt/magnusson_implementation_comparison_ko.md`
- NDT 코드 매핑: `docs/ndt/NDT_CODE_MATH_MAPPING.md`
- 비교 문서: `docs/ndt/comparison/ndt_lightndt_gicp_수학_컨셉_비교.md`
