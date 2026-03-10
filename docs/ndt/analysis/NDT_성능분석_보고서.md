# NDT 수렴 속도 분석 보고서

## 1. 개요

본 보고서는 LiDAR 포인트 클라우드 정합(registration) 벤치마크에서 **NDT(Normal Distributions Transform) 팩터의 수렴 속도가 다른 팩터 대비 현저히 느린 원인**을 분석한 결과를 정리한 것이다.

### 분석 대상
- **프로젝트**: Bottom-LiDAR-docker (gtsam_points 기반 LiDAR 정합 벤치마크)
- **옵티마이저**: Levenberg-Marquardt (LM)
- **비교 팩터**: ICP, Point-to-Plane, GICP, VGICP, NDT, LightNDT, LOAM (총 7종)

---

## 2. 벤치마크 결과

| 팩터 | 소요 시간 (ms) | 반복 횟수 | 평균 Translation 오차 (m) |
|------|---------------|----------|--------------------------|
| LOAM | 70 ~ 78 | 9 | 0.086 |
| LightNDT | 307 ~ 317 | 6 ~ 7 | 0.089 |
| Point-to-Plane | 413 ~ 424 | 16 | 0.116 |
| GICP | 546 ~ 553 | 10 | 0.107 |
| Point-to-Point (ICP) | 652 ~ 681 | 27 | 0.119 |
| VGICP | 711 ~ 741 | 12 | 0.102 |
| **NDT** | **2095 ~ 2274** | **64 ~ 70** | **0.102 ~ 0.106** |

### 핵심 관찰
- NDT는 **64~70회 반복**으로 다른 팩터(7~27회) 대비 **3~10배 많은 반복**이 필요
- 소요 시간 약 **2,200ms**로 가장 빠른 LOAM(~75ms) 대비 **약 30배**, 동일 voxel 기반인 LightNDT(~310ms) 대비 **약 7배** 느림
- 정합 정확도(0.102m)는 VGICP(0.102m)와 동등하여 **정확도 문제가 아닌 수렴 속도 문제**임이 확인됨

---

## 3. NDT vs LightNDT 비용 함수 비교

NDT와 LightNDT는 동일한 voxel 기반 대응점 탐색(DIRECT7)을 사용하지만, **비용 함수 구조가 근본적으로 다르다.**

### 3.1 NDT 비용 함수 (지수 함수 기반)

```
cost = −d₁ · (1 − exp(−d₂/2 · mᵀΣ⁻¹m))
```

- `d₁ = log(outlier_ratio / num_points)` (음수, 약 −6.9)
- `d₂ = −2 · log(−d₁) / d₁` (양수, 약 5.5 @ outlier_ratio=0.01)
- `m = mean_B − T·mean_A` (잔차 벡터)
- `Σ⁻¹` = 타깃 voxel의 공분산 역행렬

**소스 코드** (`integrated_ndt_factor_impl.hpp`, line 260-264):
```cpp
const double score_function = -gauss_d1 * e_term;  // = |d1| * exp(...)
const double cost = -gauss_d1 - score_function;     // = -d1 * (1 - exp(...))
```

### 3.2 LightNDT 비용 함수 (순수 이차 함수)

```
cost = mᵀΣ⁻¹m
```

**소스 코드** (`integrated_light_ndt_factor_impl.hpp`, line 228):
```cpp
const double cost = residual.transpose() * inv_cov_B * residual;
```

### 3.3 비교 요약

| 특성 | NDT | LightNDT |
|------|-----|----------|
| 비용 함수 형태 | 지수 (비선형 포화) | 순수 이차 |
| Hessian 근사 | H1만 사용 (H2, H3 생략) | 정확한 GN Hessian |
| 가중치 | `weight = −d₁·d₂·exp(...)` | 없음 (항등) |
| GN 근사 정확도 | 낮음 (곡률 과대추정) | 완벽 (이차 함수이므로) |

---

## 4. 근본 원인 분석

NDT의 느린 수렴은 **단일 파라미터 문제가 아닌 구조적 알고리즘 한계**에 기인한다. 4가지 상호작용하는 메커니즘이 악순환(vicious cycle)을 형성한다.

### 4.1 Hessian 근사의 구조적 부정확성

NDT의 비용 함수를 포즈 파라미터 `x`에 대해 미분하면 전체 Hessian은 세 항으로 구성된다:

```
H_full = H₁ + H₂ + H₃
```

- **H₁** (Gauss-Newton 항): `weight · Jᵀ Σ⁻¹ J` — 항상 양정치(PSD)
- **H₂** (2차 미분 항): 비용 함수의 2차 도함수에서 발생 — **음수 성분 포함**
- **H₃** (자코비안 2차 항): Lie group 위의 2차 효과 — **음수 성분 포함**

**문제**: H₂와 H₃는 음수 고유값을 가질 수 있어, `H_full`이 양정치(PSD)가 아닐 수 있다. PSD가 아니면 LM/GN 옵티마이저가 올바른 하강 방향을 찾을 수 없으므로, NDT 구현체는 **H₁만 사용**한다.

**소스 코드** (`integrated_ndt_factor_impl.hpp`, line 282-289):
```cpp
//  Gauss-Newton 근사 Hessian (Magnusson Eq. 6.13의 H1 항) 
// H ≈ weight * J^T * Σ^{-1} * J   (H2, H3 항 생략 → PSD 보장)
*H_target += weight * J_target.transpose() * inv_cov_B * J_target;
```

**결과**: H₁만으로 구성된 Hessian은 실제 곡률을 **과대추정**한다. 과대추정된 곡률은 스텝 크기를 **과소**하게 만들어, 매 반복마다 이동량이 부족해진다.

> **비유**: 언덕 경사가 실제보다 가파르다고 판단하여, 한 발짝씩 조금만 내딛는 것과 같다.

### 4.2 지수 함수의 포화 (Saturation) 효과

NDT의 비용 함수는 잔차가 작을 때 포화된다:

```
cost = −d₁ · (1 − exp(−d₂/2 · m))
```

- 잔차 `m`이 작아지면 `exp(−d₂/2 · m) → 1`이 되어 비용이 0에 접근
- 이때 **기울기(gradient)도 0에 접근**하여, 옵티마이저가 최적해 근처에서 진전을 만들기 어려움

반면 LightNDT의 이차 비용 `mᵀΣ⁻¹m`은 잔차에 비례하여 선형적으로 감소하므로 이 문제가 없다.

### 4.3 Per-Point 가중치의 소실

NDT의 per-point 가중치:

```cpp
weight = −gauss_d1 · gauss_d2 · e_term
```

- `e_term = exp(−d₂/2 · m)`: 잔차가 클수록 0에 가까워짐
- 정합이 잘 안 되는 점(큰 잔차)의 가중치가 **0에 수렴** → gradient 기여 소실
- 정합이 잘 되는 점(작은 잔차)의 가중치만 남지만, 이 점들은 이미 잔차가 작아 gradient 자체가 미미

**LightNDT와의 차이**: LightNDT는 가중치 없이 `Jᵀ Σ⁻¹ J`를 그대로 사용하므로, 모든 점이 잔차 크기에 비례하여 gradient에 기여한다.

### 4.4 LM 모델 충실도 악순환

Levenberg-Marquardt 옵티마이저는 매 반복에서 **모델 충실도(model fidelity)**를 검사한다:

```
ρ = (실제 비용 감소량) / (선형 모델 예측 감소량)
```

NDT에서 발생하는 악순환:

1. H₁-only Hessian이 곡률을 과대추정 → 선형 모델이 실제보다 큰 비용 감소를 예측
2. 실제 비용 감소가 예측보다 작음 → `ρ < 1` (모델 충실도 낮음)
3. LM이 `λ`(damping factor)를 증가시킴 → 스텝 크기 더 축소
4. 스텝이 더 작아짐 → 비용 감소가 더 미미해짐 → `ρ`가 더 낮아짐
5. **λ 증가 → 스텝 축소 → ρ 감소 → λ 더 증가** 악순환 반복

이 악순환이 NDT가 **64~70회 반복**에도 수렴하지 못하고, `maxIterations=100` 제한에 접근하는 근본 원인이다.

---

## 5. outlier_ratio 튜닝 실험 결과

### 가설
`outlier_ratio` 값이 비용 함수의 `d₁`, `d₂` 파라미터를 결정하므로, 이 값을 조정하면 수렴 속도가 개선될 수 있다.

### 실험 내용
- **원래 설정**: `main.cpp`에서 `set_outlier_ratio(0.01)` → `d₁ ≈ −6.9`, `d₂ ≈ 5.5`
- **변경 설정**: `set_outlier_ratio` 호출 제거 → 라이브러리 기본값 `0.1` 적용 → `d₁ ≈ −4.51`, `d₂ ≈ 0.231`

### 결과

| 설정 | outlier_ratio | d₂ | 반복 횟수 | 소요 시간 (ms) |
|------|--------------|-----|----------|---------------|
| 원본 | 0.01 | ~5.5 | 64 ~ 70 | 2095 ~ 2274 |
| 변경 | 0.10 | ~0.231 | 64 ~ 70 | ~2240 |

**결론**: `d₂` 값이 5.5에서 0.231로 약 24배 감소했음에도 **반복 횟수에 변화 없음**. 이는 근본 원인이 `outlier_ratio` 파라미터가 아닌 **비용 함수 구조 자체**에 있음을 실증적으로 확인해준다.

### 원인 해석
`outlier_ratio`를 바꾸면 `d₂`가 변하여 지수 함수의 감쇠 속도가 달라지지만, 4.1절~4.4절의 구조적 문제(H₁-only Hessian, 포화, 가중치 소실, LM 악순환)는 `d₂` 값에 무관하게 존재한다. `d₂`가 작아지면 비용 함수가 더 평탄해지지만, 동시에 gradient도 작아져 스텝 크기가 줄어드는 trade-off가 발생하여 결과적으로 상쇄된다.

---

## 6. main.cpp 팩터 설정 구조

### NDT 설정 (main.cpp line 624~631)
```cpp
auto factor = gtsam::make_shared<gtsam_points::IntegratedNDTFactor_<...>>(...);
factor->set_num_threads(num_threads);
factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT7);
factor->set_outlier_ratio(0.01);                    // NDT만의 추가 파라미터
factor->set_regularization_epsilon(1e-3);
factor->set_correspondence_update_tolerance(...);
```

### LightNDT 설정 (main.cpp line 635~641)
```cpp
auto factor = gtsam::make_shared<gtsam_points::IntegratedLightNDTFactor_<...>>(...);
factor->set_num_threads(num_threads);
factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT7);
// set_outlier_ratio 호출 없음 (LightNDT에는 해당 파라미터 없음)
factor->set_regularization_epsilon(1e-3);
factor->set_correspondence_update_tolerance(...);
```

**관찰**: NDT만 `set_outlier_ratio(0.01)`를 추가로 호출한다. 다른 팩터들(GICP, VGICP 등)은 해당 함수를 호출하지 않으며, 이는 `outlier_ratio`가 NDT 고유의 비용 함수 파라미터이기 때문이다.

### LM 옵티마이저 설정 (main.cpp line 686~705)
```cpp
gtsam_points::LevenbergMarquardtExtParams lm_params;
lm_params.maxIterations = 100;
lm_params.relativeErrorTol = 1e-5;
lm_params.absoluteErrorTol = 1e-5;
lm_params.lambdaLowerBound = 1e-6;
```

이 LM 파라미터는 모든 팩터에 동일하게 적용되므로, NDT의 느린 수렴은 옵티마이저 설정이 아닌 팩터 자체의 비용 함수 구조에 기인한다.

---

## 7. 잠재적 해결 방안

> ⚠️ 아래 방안들은 **라이브러리 코어(gtsam_points)를 수정**해야 하며, 각각 trade-off가 존재한다.

### 7.1 H₂ 항 부분 복원 + LM 댐핑 강화

**방법**: Hessian에 H₂ 항을 추가하되, `H_full = H₁ + H₂ + λI`에서 `λ`를 충분히 크게 설정하여 PSD를 보장한다.

- **장점**: 더 정확한 곡률 추정 → 적절한 스텝 크기
- **단점**: `λ` 초기값 설정이 까다로움, PSD 위반 시 발산 위험

### 7.2 sqrt(cost) 변환

**방법**: `f(x) = sqrt(cost(x))`로 변환하면, `J_new = J / (2·sqrt(cost))`이 되어 GN Hessian `J_newᵀJ_new`가 원래 비용 함수의 곡률에 더 가까워진다.

- **장점**: GN 근사 정확도 향상
- **단점**: `cost → 0` 근처에서 수치 불안정 (0으로 나누기)

### 7.3 LightNDT로 대체

**방법**: NDT 대신 LightNDT를 사용한다.

- **장점**: 구현 변경 없음, 7배 빠름, 정확도 동등 이상 (0.089m vs 0.102m)
- **단점**: NDT의 로버스트 특성(아웃라이어 자동 하향 가중) 상실

### 7.4 LM 파라미터 NDT 전용 튜닝

**방법**: NDT 사용 시에만 `lambdaInitial`, `lambdaFactor` 등을 별도로 설정한다.

- **장점**: 기존 코드 구조 유지
- **단점**: 근본 원인 해결 아님, 데이터셋에 따라 재튜닝 필요

---

## 8. 결론

| 항목 | 내용 |
|------|------|
| **근본 원인** | NDT의 지수 비용 함수 + H₁-only Gauss-Newton Hessian의 구조적 부적합 |
| **파라미터 튜닝** | `outlier_ratio` 변경으로는 해결 불가 (실증 확인) |
| **LM 옵티마이저** | 모든 팩터 동일 설정이므로 옵티마이저 문제 아님 |
| **실용적 권장** | 클린 데이터 환경에서는 **LightNDT 사용** (7배 빠름, 정확도 동등) |
| **구조적 해결** | H₂ 항 복원 또는 sqrt(cost) 변환 필요 (라이브러리 수정 필수) |

NDT의 느린 수렴은 단순한 파라미터 문제가 아닌, **비용 함수의 수학적 구조와 Gauss-Newton 근사 사이의 근본적 불일치**에서 비롯된다. 이는 Magnusson(2009)의 원래 NDT 논문에서도 인지된 문제로, H₂/H₃ 항의 비양정치성(indefiniteness) 때문에 의도적으로 생략한 것이다. 결과적으로 NDT는 정확한 수렴을 위해 많은 반복이 불가피하며, 동일한 voxel 대응점 구조를 순수 이차 비용으로 사용하는 LightNDT가 실용적인 대안이 된다.

---

## 부록: 파일 참조

| 파일 | 설명 |
|------|------|
| `src/main.cpp` | 벤치마크 메인 (팩터 생성, LM 설정) |
| `thirdparty/gtsam_points/.../integrated_ndt_factor_impl.hpp` | NDT evaluate() 구현 (H₁-only GN) |
| `thirdparty/gtsam_points/.../integrated_light_ndt_factor_impl.hpp` | LightNDT evaluate() 구현 (순수 이차) |
| `thirdparty/gtsam_points/.../integrated_matching_cost_factor.cpp` | 기반 클래스 linearize() |
| `thirdparty/gtsam_points/.../levenberg_marquardt_ext.cpp` | LM 옵티마이저 (모델 충실도 검사) |
