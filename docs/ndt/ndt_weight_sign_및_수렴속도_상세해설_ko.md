# NDT `raw_weight` 부호와 수렴속도: 현재 코드 기준 상세 해설

## 문서 목적

이 문서는 아래 질문을 현재 워킹트리 코드 기준으로 명확히 설명한다.

1. 왜 `raw_weight` 앞에 `-`가 필요한가?
2. 왜 weight 처리 방식을 바꾸면 속도가 달라지는가?
3. `compute_ndt_params()`에서 나오는 `c1, c2, d3, d1, d2` 값은 현재 코드에서 실제로 얼마인가?

> 참고: 코드에는 `c3`가 없고 **`d3`**가 있다.

---

## 1) 현재 코드의 핵심 수식 경로

대상 파일: `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp`

- Mahalanobis 거리

$$
m = r^T \Sigma^{-1} r
$$

- 지수항

$$
e\_term = \exp\left(-\frac{d2}{2}m\right)
$$

- score / cost

$$
score = -d1 \cdot e\_term
$$

$$
cost = -d1 - score = -d1(1-e\_term)
$$

- 현재 워킹트리의 가중(수정된 상태)

$$
raw\_weight = d1 \cdot d2 \cdot e\_term
$$

- 현재 워킹트리의 선형화 누적

$$
H \;+=\; raw\_weight \cdot J^T\Sigma^{-1}J,
\qquad
b \;+=\; raw\_weight \cdot J^T\Sigma^{-1}r
$$

---

## 2) `compute_ndt_params()`에서 나오는 값의 의미

대상 파일: `thirdparty/gtsam_points/include/gtsam_points/factors/integrated_ndt_factor.hpp`

함수 내부:

$$
c1 = 10(1-p), \qquad c2 = \frac{p}{r^3}, \qquad d3 = -\ln(c2)
$$

$$
d1 = -\ln(c1+c2)-d3
$$

$$
d2 = -2\ln\left(\frac{-\ln(c1e^{-1/2}+c2)-d3}{d1}\right)
$$

여기서,

- `p` = `outlier_ratio`
- `r` = `resolution`

의미:

- `c1`: inlier(정상 정합) 쪽 스케일 상수
- `c2`: outlier 균등 배경항(복셀 체적 `r^3` 반영)
- `d3`: 로그 공간 오프셋(중간 계산용)
- `d1`: score 진폭을 결정 (`d1<0`)
- `d2`: Mahalanobis 거리 감쇠 기울기 (`d2>0`)

---

## 3) 현재 네 코드 설정으로 계산한 실제 수치

NDT 생성부(`src/main.cpp`)에서 현재:

- `outlier_ratio = 0.01`
- `resolution`은 별도 설정이 없어 기본값 `1.0`

수치 계산 결과:

- `c1 = 9.9`
- `c2 = 0.01`
- `d3 = 4.605170185988091`
- `d1 = -6.898714534329987`
- `d2 = 0.15027142719495`

즉,

- `d1 < 0`, `d2 > 0`, `e_term > 0`

은 항상 성립한다.

---

## 4) 왜 `raw_weight` 앞 `-`가 필요하다고 말하는가?

정석 형태(이전 코드/문서 기준)는

$$
raw\_weight_{correct} = -d1\cdot d2\cdot e\_term
$$

이고, 이 경우 양수다.

하지만 현재 수정은

$$
raw\_weight_{current} = d1\cdot d2\cdot e\_term
$$

이므로 음수다.

### 예시 (현재 수치로 계산)

- `m=0`: `e_term=1.0`
  - `raw_weight_correct = +1.036679679`
  - `raw_weight_current = -1.036679679`
- `m=10`: `e_term=0.471725922`
  - `raw_weight_correct = +0.489028677`
  - `raw_weight_current = -0.489028677`

즉 현재 방식은 Hessian/gradient 누적에 **음수 스칼라**를 곱게 된다.

---

## 5) 왜 속도가 "빨라 보이는데" 결과가 나빠질 수 있나?

현재 실행(`benchmark_output_current.txt`)에서 NDT는:

- Mean T: `0.077161 m`
- Mean R: `4.577224 deg`
- Time: `261 ms`

회전 오차가 비정상적으로 커졌는데 시간은 매우 짧다.

이 패턴은 보통 다음과 맞다.

1. 선형화 가중 부호가 뒤집혀(음수 weight),
2. LM이 정상적인 곡면에서 내려가는 대신 비정상적인 방향/조기 종료 패턴을 보이고,
3. "빠른데 품질이 깨진" 결과가 나온다.

즉 "진짜로 더 잘 수렴해서 빨라진 것"이 아니라,
**수학적으로 일관성이 깨진 상태에서 비정상적으로 끝난 것**에 가깝다.

---

## 6) "수렴속도"와 "weight 처리"의 진짜 관계

혼동하기 쉬운 포인트를 분리하면:

- `-` 부호 유무 문제: **부호 일관성/안정성 문제**
- `raw_weight` vs `weight^0.25` 문제: **감쇠 강도(robust성-속도 트레이드오프) 문제**

즉,

1. `-`는 먼저 수학적으로 맞춰야 하는 필수 조건
2. 그 다음 `raw_weight`를 그대로 쓸지, `^0.25`로 완화할지 결정하는 것이 속도/정확도 튜닝

---

## 7) 결론

- 현재 워킹트리 수정(`raw_weight = d1*d2*e_term`)은 부호 관점에서 문제가 생길 수 있는 형태다.
- 이로 인해 시간이 짧아져 보여도, 회전 오차가 크게 악화되는 비정상 패턴이 나타났다.
- 따라서 먼저 `raw_weight` 부호 일관성을 복구한 뒤,
  - 정통식(`raw_weight` 그대로) vs
  - 완화식(`raw_weight^(1/4)`)
  을 A/B로 비교하는 것이 올바른 절차다.
