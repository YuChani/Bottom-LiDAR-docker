# NDT에서 `score_function` 없이 `weight/J^T Σ^{-1} J` 중심으로 푸는 방식 정리

## 문서 목적

이 문서는 아래 해석을 명확히 설명한다.

- "NDT 원래 score를 직접 쓰지 않고, `J^T Σ^{-1} J`(및 대응 gradient 항) 중심으로 최적화하면 무엇이 되는가?"
- "여기서 `Σ`를 weight라고 봐도 되는가?"

핵심 결론은 다음과 같다.

- 정확한 weight는 `Σ`가 아니라 **`Σ^{-1}`(정보행렬)**이다.
- `score` 기반 스칼라 가중을 생략하고 `J^T Σ^{-1} J`만 사용하면,
  NDT의 비선형/확률적 성질이 약해지고 **Mahalanobis 가중 LSQ(GICP 유사)**로 바뀐다.

---

## 1) 원래 NDT의 구조 (개념)

NDT는 보통 포인트별 Mahalanobis 거리

\[
m_i = r_i^T \Sigma_i^{-1} r_i
\]

를 기반으로 score를 만들고, 그 score의 미분으로 gradient/Hessian 스케일이 결정된다.

실무 구현 관점에서 보면(기호 단순화):

\[
H_i \approx \alpha_i J_i^T \Sigma_i^{-1} J_i,
\qquad
b_i \approx \alpha_i J_i^T \Sigma_i^{-1} r_i
\]

여기서

- `J`: 잔차의 자코비안
- `Σ^{-1}`: 방향별(기하학적) 가중
- `α_i`: score/지수항에서 유도되는 스칼라 가중

즉 NDT는 **행렬 가중(`Σ^{-1}`) + 스칼라 가중(`α_i`)**의 이중 구조다.

---

## 2) 질문한 방식: `score_function` 없이 `J^T Σ^{-1} J`만 쓰면?

`α_i`(score 계열 스칼라)를 제거하면 각 포인트는 사실상

\[
H_i \approx J_i^T \Sigma_i^{-1} J_i,
\qquad
b_i \approx J_i^T \Sigma_i^{-1} r_i
\]

만 남는다.

이때 해석은 다음과 같다.

1. NDT score의 "거리-의존 비선형 감쇠"가 사라짐
2. 멀리 벗어난 점의 영향이 충분히 줄어들지 않을 수 있음
3. 결과적으로 objective는 **가중 least squares** 쪽으로 이동
4. 구조적으로는 **GICP류 Mahalanobis 최소화**에 가까워짐

즉 "NDT를 그대로 푼다"가 아니라,
**NDT의 기하(공분산/정보행렬)는 유지하되 score 기반 robust 특성은 약화된 근사 모델**이 된다.

---

## 3) `Σ`가 weight라는 표현의 정확한 의미

자주 생기는 오해를 정리하면:

- 부정확: "`Σ`가 weight다"
- 정확: "**`Σ^{-1}`가 anisotropic weight(정보행렬)**다"

왜냐하면 실제 에너지와 Hessian은 `Σ`가 아니라 `Σ^{-1}`로 정의되기 때문이다.

\[
E_i = r_i^T \Sigma_i^{-1} r_i
\]

그래서 "weight-only" 방식이라고 할 때도,
대부분은 `Σ^{-1}` 기반 방향 가중을 말한다.

---

## 4) 장단점 요약

### 장점

- 구현이 단순해짐
- `J^T Σ^{-1} J`는 PSD 성질 관리가 쉬워 LM/GN과 궁합이 좋음
- 계산 안정성이 좋아지는 경우가 있음

### 단점

- 원래 NDT의 score 기반 robust 성질이 약화됨
- outlier/오정렬 점에 더 민감해질 수 있음
- "NDT 원식과 동일"이라고 말하기 어려움

---

## 5) 실제 코드 문맥과 연결

현재 코드에서 NDT 핵심 항은 아래 파일에 있다.

- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp`

여기서 확인할 포인트:

- `score_function` 계산
- `cost` 계산(최소화 형태)
- `raw_weight` 및 `weight` 계산
- `H += weight * J^T * Σ^{-1} * J`
- `b += weight * J^T * Σ^{-1} * r`

즉 코드 자체도 "score/cost"와 "가중 Hessian"이 연결된 구조이며,
score 계열 스칼라를 제거하면 모델 성질이 바뀐다는 해석이 자연스럽다.

---

## 6) 실무적으로 어떻게 부르면 정확한가

아래처럼 부르면 혼동이 가장 적다.

- "NDT 원식"
- "NDT의 weighted GN 근사"
- "scoreless Mahalanobis weighted LSQ (GICP-like)"

특히 `score_function`을 제거한 버전은
"NDT의 단순화 근사" 또는 "GICP 유사화"라고 명시하는 것이 정확하다.

---

## 7) 한 줄 결론

`score_function` 없이 `J^T Σ^{-1} J`만 쓰는 방식은 가능하지만,
그것은 원래 NDT를 그대로 푸는 것이 아니라
**NDT의 일부를 차용한 Mahalanobis 가중 최소자승 근사**로 보는 것이 수학적으로 정확하다.
