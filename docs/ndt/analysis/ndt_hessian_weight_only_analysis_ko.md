# NDT Hessian에서 score 계열 가중을 제거하고 `J^T Σ^{-1} J`만 사용할 때의 해석

## 질문

Magnusson NDT의 Hessian 맥락에서 앞의 score/지수 가중(예: `-d1*d2*exp(...)`)을 제거하고,
`J^T Σ^{-1} J` 항만 사용하면 무엇이 되는가?

## 짧은 결론

- `Σ` 자체가 아니라 **`Σ^{-1}`(역공분산, 정보행렬)**이 방향별 가중 역할을 한다.
- score/지수 가중을 빼고 `J^T Σ^{-1} J`만 쓰면, NDT의 핵심 비선형 가중 구조가 빠져
  사실상 **Mahalanobis 가중 LSQ(GICP에 가까운 형태)**로 수렴한다.
- 즉 "동일한 NDT Hessian"이 아니라, NDT objective의 성질을 일부 잃은 **근사/대체 모델**이 된다.

---

## 코드 기준 현재 구조

현재 구현은 Hessian/gradient를 아래 형태로 누적한다.

- weight 계산: `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp`
  - `raw_weight = -gauss_d1 * gauss_d2 * e_term`
  - `weight = sqrt(sqrt(raw_weight))` (현재 휴리스틱)
- Hessian/gradient 누적: 동일 파일
  - `H += weight * J^T * Σ^{-1} * J`
  - `b += weight * J^T * Σ^{-1} * r`

문서 매핑:

- `docs/ndt/NDT_CODE_MATH_MAPPING.md`
- `docs/ndt/magnusson_implementation_comparison_ko.md`
- `docs/ndt/weighted_gauss_newton_feasibility_ko.md`

---

## 수학적으로 무엇이 바뀌는가

NDT에서 포인트당 cost는 (부호/상수 이동 포함) 본질적으로

\[
E_i \propto f\big(m_i\big),\quad
m_i = r_i^T \Sigma_i^{-1} r_i,
\]

그리고 gradient/Hessian에는 chain rule로 인해 score/지수 기반 스칼라가 붙는다.

\[
\alpha_i \sim (-d_1 d_2)e^{-\frac{d_2}{2}m_i}
\]

즉 구현 관점의 GN형 근사는

\[
H_i \approx \alpha_i J_i^T \Sigma_i^{-1} J_i
\]

이다.

### 만약 `\alpha_i`를 제거하면

\[
H_i \approx J_i^T \Sigma_i^{-1} J_i
\]

가 되어, 포인트별 기여가 `m_i`(정합 거리)에 따라 줄어드는 NDT 특성이 사라진다.

---

## 실무적 의미

1. **강건성(robustness) 저하 가능성**
   - 원래 NDT는 멀리 떨어진 포인트의 기여를 지수적으로 낮춘다.
   - 이를 제거하면 outlier/오정렬 포인트 영향이 상대적으로 커질 수 있다.

2. **모델 성질 변화**
   - `Σ^{-1}`의 방향 가중(평면/선형 구조 반영)은 유지된다.
   - 하지만 "확률적 NDT score의 비선형 래핑"이 빠져 objective가 달라진다.

3. **GICP 유사화**
   - `J^T Σ^{-1} J`만 보면 구조적으로 GICP류 Mahalanobis LSQ와 가까워진다.
   - 따라서 "NDT를 정확히 푼다"기보다 "NDT의 일부 기하만 빌린 근사"에 가깝다.

---

## 오해 방지 포인트

- "`Σ`가 weight"라는 말은 엄밀히는 부정확하다.
  - 정확한 표현은 **`Σ^{-1}`가 anisotropic weight(정보행렬)** 역할을 한다는 것.
- NDT에서 스칼라 weight(`\alpha_i`)와 행렬 weight(`Σ^{-1}`)는 역할이 다르다.
  - `Σ^{-1}`: 방향별(기하학적) 가중
  - `\alpha_i`: 정합 상태(`m_i`)에 따른 크기 가중

---

## 외부 구현과의 대조 포인트

- `ndt_omp` 구현에서도 score에서 유도된 스케일을 gradient/Hessian에 반영한다.
  - 참조: `/tmp/ndt_omp_upstream/include/pclomp/ndt_omp_impl.hpp`
  - `e_x_cov_x = gauss_d2 * exp(...)` 후 `e_x_cov_x *= gauss_d1_`를 gradient/Hessian 항에 사용.
- 즉 외부 대표 구현도 `J^T \Sigma^{-1} J`만 단독으로 쓰지 않고,
  **score 계열 스칼라(혹은 동등 스케일)**를 함께 사용해 NDT 특성을 유지한다.

---

## 정리

- `J^T Σ^{-1} J`만 쓰는 것은 계산적으로 단순하고 PSD 성질 관리가 쉬울 수 있다.
- 하지만 그것만으로는 Magnusson NDT의 score 기반 미분 구조와 동일하지 않다.
- 따라서 이 선택은 "틀렸다/맞다"의 이분법보다, **NDT objective를 단순화한 근사 선택**으로 보는 것이 정확하다.
