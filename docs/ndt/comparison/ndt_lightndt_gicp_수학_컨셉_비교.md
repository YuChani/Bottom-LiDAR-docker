# NDT, LightNDT, GICP 수학/컨셉 상세 비교

## 문서 목적

이 문서는 현재 코드베이스 구현 기준으로 NDT, LightNDT, GICP의 차이를

- 컨셉(무엇을 맞추는가)
- 목적함수(무엇을 최소화/최대화하는가)
- Hessian/Gradient 구성(어떤 가중이 들어가는가)
- 실무적 트레이드오프(속도/강건성/정확도)

관점에서 상세히 정리한다.

---

## 비교 대상 구현 파일

- NDT: `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp`
- LightNDT: `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_light_ndt_factor_impl.hpp`
- GICP: `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_gicp_factor_impl.hpp`

---

## 1) 한눈에 보는 핵심 차이

| 항목 | NDT | LightNDT | GICP |
|---|---|---|---|
| 기본 컨셉 | Point-to-Distribution (voxel Gaussian) | NDT 대응점/공분산을 쓰는 scoreless LSQ | Point-to-Point with fused covariance |
| 대응점 모델 | source 점 -> target voxel mean | source 점 -> target voxel mean | source 점 -> target point |
| 정보행렬 | target voxel `Σ_B^{-1}` | target voxel `Σ_B^{-1}` | `(Σ_B + RΣ_A R^T)^{-1}` |
| 비용함수 | score 기반 비선형 비용 | Mahalanobis 제곱합 | Mahalanobis 제곱합 |
| 추가 스칼라 가중 | 있음 (`weight`, 현재 구현은 `raw_weight^(1/4)`) | 없음 | 없음 |
| 강건성 성격 | 상대적으로 높음 (score 기반 감쇠) | 중간 | 데이터/법선 품질 의존 |
| 계산량 경향 | 중~높음 | 낮음 | 중~높음 (covariance fusion + correspondence) |

---

## 2) 공통 기반: 잔차/자코비안 틀

세 방법 모두 local linearization에서 공통적으로 아래 형태를 사용한다.

- 잔차: `r = mean_B - T * mean_A`
- 선형화 기본 형태:

$$
H \sim J^T W J, \quad b \sim J^T W r
$$

차이는 **가중행렬 W를 어떻게 정의하느냐**, 그리고 **추가 스칼라 가중을 넣느냐**에서 발생한다.

---

## 3) NDT: score 기반 비선형 가중 모델

## 3.1 코드 경로 요약

NDT 구현에서는 먼저 Mahalanobis 거리

$$
m = r^T \Sigma_B^{-1} r
$$

를 계산하고, 다음 score/cost 경로를 사용한다.

- `score_function = -d1 * exp(-d2/2 * m)`
- `cost = -d1 - score_function`

코드 근거:

- `integrated_ndt_factor_impl.hpp`의 `score_function`, `cost`, `raw_weight`, `weight` 블록

## 3.2 Hessian/Gradient 구성

현재 구현은 score 미분에서 나온 스칼라 가중을 적용한다.

$$
H_{\text{NDT}} \approx \tilde{\alpha} \, J^T \Sigma_B^{-1} J,
\quad
b_{\text{NDT}} \approx \tilde{\alpha} \, J^T \Sigma_B^{-1} r
$$

여기서 현재 코드의

$$
\tilde{\alpha} = \left(-d_1 d_2 e^{-\frac{d_2}{2} m}\right)^{1/4}
$$

를 사용한다 (`weight = sqrt(sqrt(raw_weight))`).

## 3.3 의미

- 거리 `m`가 커질수록 점 기여가 줄어드는 경향이 있어 outlier 영향 완화에 유리하다.
- 대신 objective와 선형화 스케일 사이의 근사/휴리스틱 선택에 따라 수렴 특성이 달라질 수 있다.

---

## 4) LightNDT: scoreless Mahalanobis LSQ

## 4.1 코드 경로 요약

LightNDT는 NDT와 동일한 대응점(voxel mean + inv_cov)을 사용하되,
score/exp/d1/d2를 제거하고 바로 Mahalanobis 제곱합을 비용으로 둔다.

$$
E_{\text{LightNDT}} = r^T \Sigma_B^{-1} r
$$

코드 근거:

- `integrated_light_ndt_factor_impl.hpp`의 `cost = residual.transpose() * inv_cov_B * residual`

## 4.2 Hessian/Gradient 구성

$$
H_{\text{LightNDT}} = J^T \Sigma_B^{-1} J,
\quad
b_{\text{LightNDT}} = J^T \Sigma_B^{-1} r
$$

추가 스칼라 가중은 없다.

## 4.3 의미

- 수식 구조가 단순하고 계산이 빠르다.
- NDT score 기반 감쇠가 사라져, 강건성/정확도 균형이 데이터에 더 민감해질 수 있다.

---

## 5) GICP: 양쪽 공분산 융합 모델

## 5.1 코드 경로 요약

GICP는 source/target 점 각각의 공분산을 사용해 fused covariance를 만든다.

$$
\Sigma_{\text{fused}} = \Sigma_B + R \Sigma_A R^T,
\quad
W = \Sigma_{\text{fused}}^{-1}
$$

그리고 비용을

$$
E_{\text{GICP}} = r^T W r
$$

로 둔다.

코드 근거:

- `integrated_gicp_factor_impl.hpp`의 `RCR = cov_B + delta * cov_A * delta^T`
- `mahalanobis = RCR^{-1}` 후 `error = residual^T * mahalanobis * residual`

## 5.2 Hessian/Gradient 구성

$$
H_{\text{GICP}} = J^T W J,
\quad
b_{\text{GICP}} = J^T W r
$$

즉 LightNDT와 선형대수 형태는 비슷하지만, **W의 정의가 다르다**.

---

## 6) "LightNDT와 GICP가 수학적으로 같은가?"에 대한 정확한 답

결론: **완전히 같지 않다.**

같은 점:

- 둘 다 scoreless 형태의 Mahalanobis LSQ를 푼다.
- 선형화 형태가 `J^T W J`, `J^T W r`로 동일하다.

다른 점(핵심):

1. **가중행렬 W 정의**
   - LightNDT: `W = Σ_B^{-1}` (target voxel 분포만)
   - GICP: `W = (Σ_B + RΣ_A R^T)^{-1}` (source+target 융합)

2. **대응점 모델**
   - LightNDT: 점-복셀 평균(Point-to-Distribution의 단순화)
   - GICP: 점-점 대응 + 양쪽 local geometry 반영

3. **기하학적 해석**
   - LightNDT: voxel 통계 지형 기반
   - GICP: 쌍대 공분산(plane/line/point 성질) 융합 기반

---

## 7) 실무 관점 트레이드오프

- NDT
  - 장점: score 기반 감쇠로 outlier/초기 오정렬에 상대적으로 유리
  - 단점: 계산/튜닝 복잡도 증가

- LightNDT
  - 장점: 구현 단순, 빠른 실행
  - 단점: score 기반 강건성 축소

- GICP
  - 장점: local geometry 반영 정확도 우수(환경 조건 맞으면 강함)
  - 단점: 대응/공분산 품질에 민감, 계산량 증가 가능

---

## 8) 외부 참고 문헌 (권위 자료)

1. Magnusson, M. (2009). *The Three-Dimensional Normal-Distributions Transform*.
   - https://www.diva-portal.org/smash/get/diva2:276162/FULLTEXT02.pdf
2. Segal, A., Haehnel, D., Thrun, S. (2009). *Generalized-ICP* (RSS).
   - https://www.robots.ox.ac.uk/~avsegal/resources/papers/Generalized_ICP.pdf
3. Stoyanov, T., Magnusson, M., et al. (2012). *Fast and Accurate Scan Registration through Minimization of the Distance between Compact 3D NDT Representations* (IJRR).

---

## 9) 요약

- NDT는 **score 기반 비선형 가중**을 가진 Point-to-Distribution 계열이다.
- LightNDT는 NDT를 **scoreless Mahalanobis LSQ**로 단순화한 형태다.
- GICP는 LightNDT와 선형대수 형태는 비슷해도, **가중행렬 W를 양쪽 공분산 융합으로 정의**하므로 수학적으로 동일하지 않다.
