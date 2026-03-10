# LightNDT vs NDT 구현/수학 상세 비교

## 비교 대상 파일

- NDT
  - `thirdparty/gtsam_points/include/gtsam_points/factors/integrated_ndt_factor.hpp`
  - `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp`
- LightNDT
  - `thirdparty/gtsam_points/include/gtsam_points/factors/integrated_light_ndt_factor.hpp`
  - `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_light_ndt_factor_impl.hpp`
- 벤치마크 연결
  - `src/main.cpp`

---

## 1) 구현 구조 차이 (클래스/파라미터)

### NDT에만 있는 상태/설정

- `resolution`, `outlier_ratio` 멤버를 보유하고, 매 correspondence 업데이트 시 `compute_ndt_params(...)`로 `gauss_d1`, `gauss_d2`를 갱신
- 관련 setter 존재
  - `set_resolution(...)`
  - `set_outlier_ratio(...)`

### LightNDT에서 제거된 항목

- `resolution`, `outlier_ratio`, `gauss_d1`, `gauss_d2` 없음
- 즉 "score 기반 지수 가중"을 만들기 위한 상태를 아예 들고 있지 않음

### 공통 항목

- `NDTSearchMode (DIRECT1/7/27)`
- correspondence 업데이트 tolerance
- `regularization_epsilon`, `inv_cov_cache`
- voxel neighbor 탐색 + 최소 Mahalanobis 기준 correspondence 선택

요약하면, **LightNDT는 correspondence 탐색 구조는 NDT와 거의 동일하지만, score 계열 파라미터 경로를 제거한 경량 버전**이다.

---

## 2) evaluate()에서 실제로 달라진 코드 경로

## 2.1 NDT 경로

`integrated_ndt_factor_impl.hpp` 기준:

1. `mahalanobis_dist = r^T Σ^{-1} r` 계산
2. `exponent = -d2/2 * mahalanobis_dist`, `e_term = exp(exponent)` 계산
3. `score_function = -d1 * e_term`
4. `cost = -d1 - score_function` (최소화용 변환)
5. 미분/선형화용 스칼라
   - `raw_weight = -d1 * d2 * e_term`
   - `weight = sqrt(sqrt(raw_weight))` (현재 구현)
6. `H, b` 누적 시 `weight`를 곱함
   - `H += weight * J^T Σ^{-1} J`
   - `b += weight * J^T Σ^{-1} r`

## 2.2 LightNDT 경로

`integrated_light_ndt_factor_impl.hpp` 기준:

1. `cost = r^T Σ^{-1} r`
2. score/exponential/`d1,d2` 계산 없음
3. `H, b` 누적에서 스칼라 weight 없음
   - `H += J^T Σ^{-1} J`
   - `b += J^T Σ^{-1} r`

즉, **LightNDT는 `J^T Σ^{-1} J` 중심의 scoreless Mahalanobis LSQ**로 동작한다.

---

## 3) 수학식 비교

포인트 i 기준(기호 단순화):

- 잔차: `r_i`
- 자코비안: `J_i`
- 정보행렬: `W_i = Σ_i^{-1}`

### NDT (현재 구현)

- 비용(스칼라):

$$
E_i^{\mathrm{NDT}} = -d_1\left(1 - e^{-\frac{d_2}{2} m_i}\right), \quad m_i = r_i^T W_i r_i
$$

- 선형화 누적(구현):

$$
H_i \approx \tilde{\alpha}_i J_i^T W_i J_i,
\quad
b_i \approx \tilde{\alpha}_i J_i^T W_i r_i
$$

$$
\tilde{\alpha}_i = \left(-d_1 d_2 e^{-\frac{d_2}{2} m_i}\right)^{1/4}
$$

### LightNDT

- 비용(스칼라):

$$
E_i^{\mathrm{Light}} = r_i^T W_i r_i
$$

- 선형화 누적:

$$
H_i = J_i^T W_i J_i,
\quad
b_i = J_i^T W_i r_i
$$

---

## 4) 핵심 수학적 차이 해석

1. **목적함수 형태**
   - NDT: 지수 score 기반(비선형 robust 성격)
   - LightNDT: 순수 Mahalanobis 제곱합

2. **가중 구조**
   - NDT: `W_i(=Σ^{-1})` + 스칼라 `weight`의 이중 가중
   - LightNDT: `W_i(=Σ^{-1})`만 사용

3. **outlier/원거리 점 영향**
   - NDT: 거리 증가 시 지수 감쇠(영향 축소)
   - LightNDT: 지수 감쇠가 없어 상대적으로 영향 유지

4. **objective-선형화 일관성 관점**
   - LightNDT: 비용 `r^TWr`와 `J^T W J` 선형화가 직접적으로 대응
   - NDT(현재): 비용은 score 기반인데 선형화 스케일에 `weight^0.25`가 들어가므로, 엄밀 미분 일치 관점에서는 추가 휴리스틱이 포함됨

---

## 5) 벤치마크 연결에서 달라진 부분

`src/main.cpp`에서 다음이 추가됨:

- factor list에 `LightNDT` 항목 추가
- `create_factor(...)`에 `IntegratedLightNDTFactor_<PointCloud>` 생성 분기 추가
- headless 루프에서 `LightNDT`가 자동 실행/출력되도록 반영

---

## 6) 최근 실행 기준 관찰값 (동일 데이터, headless)

최근 실행 로그 기준 주요 비교:

- NDT: Mean T `0.098348`, Mean R `0.688667`, `3893 ms`
- LightNDT: Mean T `0.158550`, Mean R `0.646398`, `1030 ms`

해석:

- 이번 데이터에서는 **LightNDT가 속도는 크게 개선**
- 대신 **평행이동 오차는 증가**
- 회전 오차는 평균/최대 항목에서 일부 개선/열화가 섞여 나타남 (데이터/초기치 의존)

---

## 7) 정리

- LightNDT는 NDT의 correspondence/공분산 기반 기하는 유지하면서,
  score/지수 가중 경로를 제거해 **단순하고 빠른 Mahalanobis LSQ 형태**로 만든 구현이다.
- NDT는 score 기반 강건성/비선형성 이점이 있지만 계산량/튜닝 복잡도가 더 크다.
- 어떤 것이 더 "좋다"라기보다, **정확도-속도 트레이드오프를 어떤 쪽으로 둘 것인지**에 따라 선택해야 한다.
