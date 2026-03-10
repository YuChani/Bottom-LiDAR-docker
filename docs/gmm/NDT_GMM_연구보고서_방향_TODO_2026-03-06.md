# NDT에서 Gaussian Mixture를 사용할 때의 연구 보고서

## 0. 문서 목적

이 문서는 다음 질문에 답하기 위해 작성했다.

1. GMM(Gaussian Mixture Model)이 무엇인가?
2. NDT에 GMM을 붙이면 수학/구조가 어떻게 바뀌는가?
3. 현재 코드베이스에서 어떤 방식으로 구현하는 것이 안전한가?
4. 실제 진행 방향, 단계(step), TODO는 무엇인가?

---

## 1. GMM이란 무엇인가?

## 1.1 정의

GMM은 데이터 분포를 "가우시안 1개"가 아니라 "가우시안 K개의 혼합"으로 표현하는 확률 모델이다.

```text
p(x) = Σ_k π_k N(x | μ_k, Σ_k)
```

- `π_k`: 혼합 가중치(합이 1)
- `μ_k`: k번째 평균
- `Σ_k`: k번째 공분산

핵심은 "복잡한 분포를 여러 단순 분포의 합으로 근사"하는 것이다.

## 1.2 NDT와의 연결

기본 NDT는 voxel당 단일 Gaussian을 사용한다. 이 경우:

- 한 voxel 안에 다중 표면/다중 방향 구조가 있을 때 표현력이 부족하고
- voxel 경계에서 correspondence가 급변해 최적화가 불안정해질 수 있다.

GMM을 쓰면 voxel(또는 voxel 이웃)을 여러 Gaussian으로 표현할 수 있어:

- 분포 표현력이 좋아지고
- 확률적 soft assignment(책임도, responsibility)로 경계 불연속 완화가 가능하다.

---

## 2. 현재 코드의 NDT 구조 (출발점)

현재 NDT factor의 핵심 경로:

- correspondence 업데이트: `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:98`
- per-point cost/Hessian 누적: `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:210`
- NDT 파라미터 계산(`d1,d2`): `thirdparty/gtsam_points/include/gtsam_points/factors/integrated_ndt_factor.hpp:67`
- factor 그래프 공통 흐름(`error`, `linearize` 둘 다 correspondence 업데이트):
  - `thirdparty/gtsam_points/src/gtsam_points/factors/integrated_matching_cost_factor.cpp:32`
  - `thirdparty/gtsam_points/src/gtsam_points/factors/integrated_matching_cost_factor.cpp:40`
- main 팩터 등록 분기:
  - `src/main.cpp:622`(NDT), `src/main.cpp:632`(LightNDT)

중요한 구조적 사실:

- 지금은 "best voxel 1개"를 고르는 hard assignment이다.
- LM 내부에서 `error()`와 `linearize()` 모두 association이 바뀔 수 있다.

---

## 3. NDT + GMM 수학 구조 (권장 형태)

## 3.1 직관

기존 NDT: 각 포인트가 "하나의 분포"에만 대응

GMM-NDT: 각 포인트가 "여러 분포"에 확률적으로 대응

## 3.2 책임도(responsibility)

선형화 기준 pose `x0`에서 각 포인트 `i`와 혼합성분 `k`에 대해:

```text
γ_ik(x0) = 
  π_k N(q_i(x0) | μ_k, Σ_k)
  ---------------------------------
  Σ_j π_j N(q_i(x0) | μ_j, Σ_j)
```

여기서 `q_i(x)=T(x)p_i`.

## 3.3 LM 친화형 목적함수 (권장)

현재 아키텍처에서는 exact log-sum-exp를 바로 미분하기보다,
책임도 `γ_ik`를 고정한 surrogate가 안정적이다.

```text
Q(x|x0) = 1/2 Σ_i Σ_k γ_ik(x0) || Σ_k^{-1/2} (q_i(x)-μ_k) ||^2
```

즉, LM 입장에서는 "가중 least squares"로 풀 수 있다.

---

## 4. 왜 이 방식이 현재 프로젝트에 맞는가?

Oracle 검토 핵심(요약):

1. 현 구조는 association이 `error/linearize`에서 갱신되므로 LM 모델 충실도에 민감하다.
2. GMM에서 `γ`까지 LM trial마다 같이 바뀌면 objective 일관성이 더 깨진다.
3. 따라서 **LM 내부에서는 correspondence/γ를 고정하고**, LM 바깥 outer loop에서만 업데이트하는 방식이 가장 안전하다.

즉, 권장 전략은:

- 내부(LM step): frozen `γ`, frozen association
- 외부(outer iteration): association + `γ` 재계산

이 구조는 현재 코드의 H1-only 계열과도 호환성이 높다.

---

## 5. 구현 방향 (프로젝트 맞춤)

## Phase A: Research Prototype (빠른 검증)

목표: "되는지"를 빠르게 확인

1. `NdtCorrespondence`를 단일 항목에서 다중 항목으로 확장
   - 예: `vector<MixtureComponentMatch>`
   - 각 항목: `γ, μ, inv_cov`
2. correspondence 단계에서 best 1개 대신 top-K 후보를 저장
3. `evaluate()`에서 K개 성분을 weighted sum으로 누적

리스크: 메모리/연산량 증가

## Phase B: Stable Variant (실운영 후보)

목표: LM 안정성 확보

1. linearization point 기준으로 `γ` 계산 후 캐시
2. LM 내부 `error/linearize`는 캐시된 `γ`만 사용
3. outer loop에서만 `γ`/association 갱신

리스크 통제:

- 공분산 고유값 클램프
- 책임도 하한(`γ_min`) 기반 outlier gating
- mixture component 수 제한(`K=2` 기본)

## Phase C: 고급 실험

1. exact mixture likelihood(`-log Σ πN`) 비교 실험
2. coarse-to-fine + GMM 결합 실험
3. runtime/accuracy tradeoff 경계선 도출

---

## 6. Step-by-step 실행 절차

1. baseline 고정
   - `./build_local/lidar_registration_benchmark --headless --factor NDT --threads 8`
2. 새 factor 스켈레톤 추가
   - `IntegratedGMMNDTFactor_` (NDT impl 복제 후 최소 변경)
3. main factor 등록
   - `src/main.cpp` factor list + `create_factor()` 분기 추가
4. top-K mixture correspondence 구현
5. frozen-γ evaluate 구현
6. outer loop 재평가 훅 추가
7. 5회 반복 벤치마크
8. 기존 NDT/LightNDT와 표 비교

---

## 7. TODO (실행 체크리스트)

## P0 (바로 시작)

- [ ] `docs/gmm/` 실험 템플릿 생성(로그 경로/CSV 스키마 고정)
- [ ] `IntegratedGMMNDTFactor_` 클래스 파일 생성(초기엔 K=1 동작 동일성 확보)
- [ ] K=1일 때 기존 NDT와 수치 parity 검증

## P1 (핵심 구현)

- [ ] correspondence에서 top-K 후보 추출 (`K=2`)
- [ ] responsibility `γ` 계산 및 정규화
- [ ] `evaluate()`에서 `Σ_k γ_ik J_k^T W_k J_k`, `Σ_k γ_ik J_k^T W_k r_k` 누적
- [ ] `γ`/association 캐시 도입 + LM 내부 동결

## P2 (안정화)

- [ ] `γ_min` gating, `d2_max` gating 추가
- [ ] covariance floor, eigen clamp 로그 추가
- [ ] diverge case 리플레이 테스트셋 구축

## P3 (성능/품질)

- [ ] K=1/2/3 스윕
- [ ] DIRECT1/7/27와 조합 스윕
- [ ] runtime overhead(<=1.5x) 조건 만족 여부 점검

---

## 8. 수용 기준 (Acceptance Criteria)

1. K=1 모드에서 기존 NDT 대비 pose/error/time 차이가 허용 오차 이내
2. K=2 모드에서 수렴 실패율 감소 또는 최종 오차 개선
3. LM inner reject 급증 없음(모델 충실도 붕괴 방지)
4. 시간 증가가 허용 범위 내(목표: NDT 대비 <=1.5x, 실패율 개선 시)

---

## 9. 연구 참고 자료

아래는 이번 방향 설정에 직접 유효한 핵심 참고 문헌이다.

1. Biber, P., Strasser, W. (2003), NDT original
   - https://ieeexplore.ieee.org/document/1249285
2. Magnusson, M. (2009), 3D-NDT thesis
   - https://www.diva-portal.org/smash/get/diva2:276162/FULLTEXT02.pdf
3. Myronenko, A., Song, X. (2010), CPD (GMM 기반 확률 정합)
   - https://ieeexplore.ieee.org/document/5432191
4. Segal, A., Haehnel, D., Thrun, S. (2009), GICP
   - https://www.roboticsproceedings.org/rss05/p21.pdf
5. PCL NDT 구현 문서
   - https://pointclouds.org/documentation/classpcl_1_1_normal_distributions_transform.html

참고: GMM-NDT 계열은 논문/구현마다 목적함수(정확 likelihood vs surrogate)와 미분 취급이 다르므로,
보고서/실험에서는 반드시 "현재 구현한 목적함수"를 명시해야 한다.

---

## 10. 결론

현재 코드베이스에서 GMM을 NDT에 도입하는 가장 현실적인 방법은,

1. 혼합 대응(top-K + responsibility)으로 표현력을 올리고,
2. LM 내부에서는 대응/책임도를 고정해 objective 일관성을 유지하며,
3. outer loop에서만 mixture를 갱신하는 2단 구조를 채택하는 것이다.

이 방향은 기존 NDT의 수렴 불안정 원인을 악화시키지 않으면서,
다중 표면/경계 구간에서 정합 품질을 개선할 가능성이 가장 높다.
