# Magnusson Eq. 6.12/6.13 기반 Weighted Gauss-Newton의 gtsam_points 적용 가능성 분석

## 개요

이 문서는 Martin Magnusson의 박사학위 논문 (2009)에서 제시된 NDT Hessian의 full Newton 정의(Eq. 6.12, 6.13)를 분석하고, **weighted Gauss-Newton** 방법이 `gtsam_points` 라이브러리에서 사용 가능한지를 평가한다.

**핵심 질문:** 현재 gtsam_points의 Gauss-Newton 근사 Hessian을 Magnusson의 full Hessian 또는 weighted Gauss-Newton으로 대체/확장할 수 있는가?

---

## 1. Magnusson의 수식 정리

### 1.1 Eq. 6.12: NDT Score의 Gradient

$$g_i = \frac{\partial s}{\partial p_i} = \sum_{k=1}^n d_1 d_2 \, \vec{q}_k^T \Sigma_k^{-1} \vec{J}_{k,i} \exp\left( -\frac{d_2}{2} \vec{q}_k^T \Sigma_k^{-1} \vec{q}_k \right)$$

- $\vec{q}_k = \vec{x}_k' - \vec{\mu}_k$: 변환된 소스 포인트와 NDT 셀 평균 간의 차이 벡터
- $\vec{J}_{k,i} = \frac{\partial \vec{x}_k'}{\partial p_i}$: 변환의 자코비안
- $d_1, d_2$: NDT 파라미터 (Eq. 6.9-6.10에서 유도)

### 1.2 Eq. 6.13: NDT Score의 Full Hessian

$$H_{ij} = \sum_{k=1}^n d_1 d_2 \exp(\cdots) \left[ \underbrace{-d_2 (\vec{q}_k^T \Sigma_k^{-1} \vec{J}_{k,i})(\vec{q}_k^T \Sigma_k^{-1} \vec{J}_{k,j})}_{H_2} + \underbrace{\vec{q}_k^T \Sigma_k^{-1} \vec{H}_{k,ij}}_{H_3} + \underbrace{\vec{J}_{k,j}^T \Sigma_k^{-1} \vec{J}_{k,i}}_{H_1} \right]$$

세 개 항의 의미:

| 항 | 수식 | 의미 | 특성 |
|----|------|------|------|
| $H_1$ | $J^T \Sigma^{-1} J$ | Gauss-Newton 근사 항 | 항상 PSD (양의 준정부호) |
| $H_2$ | $-d_2 (q^T\Sigma^{-1}J)(q^T\Sigma^{-1}J)^T$ | 스코어의 2차 미분에서 나오는 외적 항 | 항상 NSD (음의 준정부호) |
| $H_3$ | $q^T \Sigma^{-1} H_E$ | 변환 함수의 2차 미분 항 | 부호 불확정 |

### 1.3 Weighted Gauss-Newton의 정의

"Weighted Gauss-Newton"은 엄밀한 학술 용어가 아니라, NDT 최적화에서 다음 두 가지 의미로 사용된다:

**의미 1: 지수 가중 Gauss-Newton**
- $H_2$, $H_3$를 생략하고 $H_1$만 사용하되, 각 포인트에 **지수 가중치** $w_k = \exp(-d_2/2 \cdot m_k)$를 적용
- 현재 gtsam_points가 사용하는 방식과 동일

**의미 2: Stoyanov/Magnusson (2012)의 확장**
- $H_3$는 생략하되, $H_2$를 포함하여 "weighted" 형태의 Gauss-Newton 시스템 구성
- Full Newton보다 안정적이면서, 순수 GN보다 빠른 수렴

---

## 2. 현재 gtsam_points 구현 분석

### 2.1 현재 Hessian 계산 방식

```cpp
// evaluate() 메서드 내부 (integrated_ndt_factor_impl.hpp)

// Magnusson Eq. 6.12에서 도출되는 포인트별 weight (양수 스칼라)
// score function의 1차 미분: ∂s/∂p = weight * q^T * Σ^{-1} * J
const double weight = -gauss_d1 * gauss_d2 * e_term;

// 가중 자코비안: (6x4) = weight * J^T * Σ^{-1}
Eigen::Matrix<double, 6, 4> J_target_weighted = weight * J_target.transpose() * inv_cov_B;

// GN Hessian 누적: H ≈ weight * J^T * Σ^{-1} * J   (H1 항만 사용)
*H_target += J_target_weighted * J_target;

// Gradient 누적: b = weight * J^T * Σ^{-1} * q
*b_target += J_target_weighted * residual;
```

이것은 **의미 1의 weighted Gauss-Newton**과 정확히 일치한다. 즉, 현재 구현은 이미 "weighted Gauss-Newton"을 사용하고 있다. 코드에서 이 스칼라를 `weight`라고 명명하여 score function의 미분에서 도출되는 의미를 명확히 하였다.

### 2.2 아키텍처적 특징

`IntegratedMatchingCostFactor`의 `evaluate()` 메서드는 **H와 b를 직접 반환**한다:

```cpp
virtual double evaluate(
    const Eigen::Isometry3d& delta,
    Eigen::Matrix<double, 6, 6>* H_target,      // 6x6 Hessian
    Eigen::Matrix<double, 6, 6>* H_source,
    Eigen::Matrix<double, 6, 6>* H_target_source,
    Eigen::Matrix<double, 6, 1>* b_target,      // 6x1 gradient
    Eigen::Matrix<double, 6, 1>* b_source) const;
```

이것은 `gtsam::HessianFactor(H, b, error)`를 직접 구성하는 인터페이스이다. **일반적인 GTSAM factor가 residual + Jacobian을 반환하여 $J^TJ$로 Hessian을 구성하는 것과 다르다.** 이 아키텍처 덕분에, `evaluate()`에서 **임의의 Hessian 구조**를 반환할 수 있다.

---

## 3. Full Hessian (Eq. 6.13) 적용 가능성

### 3.1 구현 가능 여부: ✅ 가능

아키텍처적으로 `evaluate()`가 H와 b를 직접 반환하므로, $H_2$ 항과 $H_3$ 항을 추가하는 것은 코드 수준에서 직접적으로 가능하다.

$H_2$ 항 추가 코드 (의사 코드):

```cpp
// H2 항: -d2 * weight * (q^T * Σ^{-1} * J_i) * (q^T * Σ^{-1} * J_j)^T
Eigen::Matrix<double, 1, 6> qCJ_target = (residual.transpose() * inv_cov_B * J_target);
// H2 = weight * (-d2) * qCJ^T * qCJ  (rank-1 outer product, NSD)
*H_target += weight * (-gauss_d2) * qCJ_target.transpose() * qCJ_target;
```

$H_3$ 항 추가를 위해서는 SE(3) 변환의 2차 미분 $\frac{\partial^2 x'}{\partial p_i \partial p_j}$이 필요하다. Magnusson은 Euler 각도 기반의 명시적 공식 (Eq. 6.21)을 제공하지만, SE(3) Lie algebra에서는 다른 형태가 된다.

### 3.2 고려해야 할 문제점

#### 문제 1: Hessian의 양정치성(PSD) 상실

$H_2$ 항은 **음의 준정부호(NSD)**이다:

$$H_2 = -d_2 \cdot \text{weight} \cdot (q^T\Sigma^{-1}J)^T(q^T\Sigma^{-1}J)$$

이 항은 $-d_2 \cdot \text{weight} \cdot v v^T$ 형태의 **음의** rank-1 행렬이다. $H_1 + H_2$의 합이 indefinite가 될 수 있으며, 이 경우 Newton 스텝이 극대점이나 안장점 방향을 가리킬 수 있다.

**완화 방법:**
- GTSAM의 LM damping: $H + \lambda I$에서 $\lambda$를 충분히 크게 설정하면 PSD를 복원할 수 있다
- $H_2$의 크기가 $H_1$보다 작은 경우 (정렬이 잘 된 경우), 문제가 발생하지 않을 수 있다
- 조건부 적용: $H_1 + H_2$가 PSD인 경우에만 $H_2$를 포함

#### 문제 2: SE(3)에서의 변환 Hessian ($H_3$)

Magnusson의 Eq. 6.21은 Euler 각도 기반이므로 SE(3) Lie algebra에서는 직접 사용할 수 없다. SE(3)에서의 변환 Hessian은:

$$\frac{\partial^2 (Rx + t)}{\partial \xi_i \partial \xi_j}$$

Lie algebra의 adjoint representation을 통해 유도 가능하지만, 계산이 복잡하고 수치적 이점이 크지 않다. 실제로 **PCL, ndt_omp 등 대부분의 NDT 구현에서도 $H_3$는 생략**한다.

#### 문제 3: 계산 비용 증가

| 항목 | 현재 (GN) | Full Hessian |
|------|-----------|-------------|
| Hessian 계산 | $O(36n)$ | $O(36n + 36n + 36n) = O(108n)$ |
| 추가 행렬 연산 | 없음 | rank-1 외적, 변환 Hessian |
| 수렴 반복 횟수 | 다소 많음 | 적음 (2차 수렴) |
| 총 비용 | 보통 | 반복당 비용 ↑, 반복 횟수 ↓ |

실시간 LiDAR SLAM에서는 반복당 비용이 적은 GN이 일반적으로 더 유리하다.

---

## 4. Weighted Gauss-Newton 변형의 적용 가능성

### 4.1 의미 1: 지수 가중 GN (현재 구현과 동일)

**이미 적용되어 있다.** 현재 `weight = -d1 * d2 * exp(-d2/2 * m)`이 각 포인트의 가중치 역할을 한다.

- 정렬이 잘 된 포인트 ($m \approx 0$): $\text{weight} \approx -d_1 d_2$ (최대 가중치)
- 오정렬 포인트 ($m \gg 0$): $\text{weight} \approx 0$ (자동 하향 가중)

이것은 **robust M-estimator**와 유사한 효과를 가진다. 지수 함수가 Welsch/Leclerc 커널과 동일한 형태이기 때문이다.

### 4.2 의미 2: $H_2$ 포함 weighted GN

$H_3$는 생략하되 $H_2$만 추가하는 "중간" 방식이다.

**구현 가능 여부: ✅ 가능**

```cpp
// 현재 코드에 H2 항만 추가
const double weight = -gauss_d1 * gauss_d2 * e_term;

Eigen::Matrix<double, 6, 4> J_target_weighted = weight * J_target.transpose() * inv_cov_B;

// H1 (기존)
*H_target += J_target_weighted * J_target;

// H2 추가 (새로운 코드)
Eigen::Matrix<double, 1, 4> q_inv_cov = residual.transpose() * inv_cov_B;
Eigen::Matrix<double, 1, 6> q_inv_cov_J = q_inv_cov * J_target;
*H_target += weight * (-gauss_d2) * q_inv_cov_J.transpose() * q_inv_cov_J;

// b (기존)
*b_target += J_target_weighted * residual;
```

**장점:**
- 최적점 근처에서 수렴 속도 향상 (2차 수렴에 근접)
- $H_3$보다 구현이 간단

**단점:**
- PSD 보장 상실 → LM damping 의존
- 초기 추정치가 나쁜 경우 발산 가능성

### 4.3 GTSAM LM과의 상호작용

GTSAM의 Levenberg-Marquardt 옵티마이저는 다음 시스템을 풀어야 한다:

$$(H + \lambda I) \delta = -b$$

$H_2$ 추가 시:
- $H = H_1 + H_2$에서 $H_1$은 PSD, $H_2$는 NSD
- $\lambda$가 충분히 크면 $H + \lambda I$는 여전히 PSD → 수렴 보장
- LM의 초기 $\lambda$가 작으면 ($\lambda_0 = 10^{-5}$ 등), 첫 몇 반복에서 indefinite Hessian이 발생할 수 있다
- GTSAM의 LM 구현은 이 경우 $\lambda$를 자동으로 증가시켜 PSD를 복원한다

**따라서 GTSAM의 LM 옵티마이저와 함께 사용할 경우, $H_2$ 포함은 안전하다.** 다만, $\lambda$ 증가로 인한 초기 수렴 속도 저하가 발생할 수 있다.

---

## 5. 실용적 권장 사항

### 5.1 결정 매트릭스

| 시나리오 | 권장 방법 | 이유 |
|---------|-----------|------|
| 실시간 LiDAR SLAM (순차 정합) | 현재 GN (변경 불필요) | 초기 추정치가 좋고, 수렴 속도 충분 |
| 루프 클로저 (큰 초기 오차) | $H_2$ 포함 고려 | 수렴 속도 향상 가능, LM이 안정성 보장 |
| 전역 정합 (초기 추정치 없음) | 현재 GN 유지 | $H_2$ 포함 시 발산 위험이 크고, GN이 더 안정적 |
| 고정밀 지도 생성 | $H_2$ 포함 고려 | 최적점 근처에서 더 빠른 수렴 |

### 5.2 구현 전략 (적용할 경우)

1. **설정 가능한 옵션으로 추가:**
   ```cpp
   enum class NDTHessianMode {
     GAUSS_NEWTON,       // 현재 방식 (H1만)
     WEIGHTED_NEWTON,    // H1 + H2
     FULL_NEWTON         // H1 + H2 + H3 (SE(3) 변환 Hessian 포함)
   };
   ```

2. **조건부 $H_2$ 적용:**
   - $\|q\|$가 임계값 이하인 경우에만 $H_2$ 포함 (최적점 근처에서만 활성화)
   - 이를 통해 수렴 초기의 불안정성 방지

3. **PSD 검증:**
   - $H_1 + H_2$의 최소 고유값이 음수이면 $H_2$ 제외
   - 또는 LM의 $\lambda$ 최소값을 보수적으로 설정

---

## 6. 결론

### Magnusson의 Eq. 6.12, 6.13을 통한 weighted Gauss-Newton이 gtsam_points에서 사용 가능한가?

**예, 아키텍처적으로 완전히 가능하며, 실용적으로도 유의미한 옵션이다.**

구체적 결론:

1. **현재 구현은 이미 "weighted Gauss-Newton"이다** (의미 1). 지수 가중치 `weight`가 각 포인트의 기여도를 자동으로 조절한다. 코드에서 이 스칼라를 `weight`라고 명명하여 score function의 미분에서 도출되는 의미를 명확히 하였다.

2. **$H_2$ 항 추가(의미 2)는 기술적으로 가능하다.** `evaluate()`가 H와 b를 직접 반환하는 아키텍처 덕분에, 임의의 Hessian 구조를 구성할 수 있다. $H_2 = -d_2 \cdot \text{weight} \cdot (q^T\Sigma^{-1}J)^T(q^T\Sigma^{-1}J)$를 기존 코드에 10줄 이내로 추가 가능하다.

3. **$H_3$ 항(변환의 2차 미분)은 SE(3) Lie algebra에서의 유도가 필요하여 구현 복잡도가 높다.** 대부분의 실용적 구현에서 $H_3$는 생략하며, 이점이 제한적이다.

4. **PSD 보장 상실은 GTSAM의 LM damping으로 완화 가능하다.** LM 옵티마이저가 자동으로 $\lambda$를 조절하여 $(H + \lambda I)$의 PSD를 보장한다.

5. **실용적 권장:** 현재의 GN 근사는 순차적 LiDAR SLAM에 적합하다. $H_2$ 포함은 루프 클로저나 고정밀 정합 시나리오에서 실험적으로 검증할 가치가 있다. 설정 옵션(`NDTHessianMode`)으로 추가하여 사용자가 선택할 수 있게 하는 것이 가장 실용적인 접근이다.

---

## 부록 A: $H_2$ 항 추가 구현 예시

기존 `evaluate()` 메서드에 $H_2$ 항을 추가하는 구체적인 코드 변경:

```cpp
// === 기존 코드 (H1만) ===
const double weight = -gauss_d1 * gauss_d2 * e_term;  // Eq. 6.12 포인트별 가중치

Eigen::Matrix<double, 6, 4> J_target_weighted = weight * J_target.transpose() * inv_cov_B;
Eigen::Matrix<double, 6, 4> J_source_weighted = weight * J_source.transpose() * inv_cov_B;

// H1 항 (Gauss-Newton 근사)
*H_target += J_target_weighted * J_target;
*H_source += J_source_weighted * J_source;
*H_target_source += J_target_weighted * J_source;

// === 추가 코드 (H2 항) ===
// q^T * Σ^{-1}: (1x4)
Eigen::RowVector4d qC = residual.transpose() * inv_cov_B;

// q^T * Σ^{-1} * J: (1x6) for target and source
Eigen::Matrix<double, 1, 6> qCJ_target = qC * J_target;
Eigen::Matrix<double, 1, 6> qCJ_source = qC * J_source;

// H2 = weight * (-d2) * (qCJ)^T * (qCJ)  [rank-1 outer product, NSD]
double h2_scale = weight * (-gauss_d2);
*H_target += h2_scale * qCJ_target.transpose() * qCJ_target;
*H_source += h2_scale * qCJ_source.transpose() * qCJ_source;
*H_target_source += h2_scale * qCJ_target.transpose() * qCJ_source;

// === b (gradient)는 변경 없음 ===
*b_target += J_target_weighted * residual;
*b_source += J_source_weighted * residual;
```

---

## 부록 B: 수학적 증명 — $H_2$가 NSD인 이유

$H_2 = -d_2 \cdot \text{weight} \cdot v v^T$ 에서:
- $\text{scale} > 0$ (양수)
- $d_2 > 0$ (양수)
- $v v^T$는 rank-1 PSD 행렬

따라서 $H_2 = -(\text{양수}) \cdot (\text{PSD}) = \text{NSD}$

임의의 벡터 $x$에 대해:

$$x^T H_2 x = -d_2 \cdot \text{weight} \cdot (v^T x)^2 \leq 0$$

이므로 $H_2$는 항상 음의 준정부호이다. ∎

---

## 참고 문헌

1. Magnusson, M. (2009). *The Three-Dimensional Normal-Distributions Transform — an Efficient Representation for Registration, Surface Analysis, and Loop Detection.* PhD thesis, Örebro University. Chapter 6, Equations 6.12-6.13.
2. Stoyanov, T., Magnusson, M., Andreasson, H., & Lilienthal, A.J. (2012). *Fast and Accurate Scan Registration through Minimization of the Distance between Compact 3D NDT Representations.* IJRR, 31(12), 1377-1393.
3. Koide, K. (2021). gtsam_points: A collection of GTSAM factors for point cloud SLAM. GitHub.
4. GTSAM Documentation: `HessianFactor`, `LevenbergMarquardtOptimizer`.
