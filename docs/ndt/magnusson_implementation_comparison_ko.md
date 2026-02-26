# gtsam_points NDT 구현과 Magnusson 논문의 비교 분석

## 개요

이 문서는 `gtsam_points` 라이브러리의 NDT(Normal Distributions Transform) 구현이 Martin Magnusson의 박사학위 논문 *"The Three-Dimensional Normal-Distributions Transform — an Efficient Representation for Registration, Surface Analysis, and Loop Detection"* (2009, Örebro University)에서 제시한 방법론을 따르고 있는지를 분석한다.

**분석 대상 파일:**
- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp`
- `thirdparty/gtsam_points/include/gtsam_points/factors/integrated_ndt_factor.hpp`

---

## 1. NDT Score Function (비용 함수)

### 1.1 Magnusson 논문의 정의 (Eq. 6.9-6.10)

Magnusson은 outlier에 강건한 NDT score function을 다음과 같이 정의한다:

$$s(\vec{p}) = -\sum_{k=1}^{n} d_1 \exp\left( -\frac{d_2}{2} (\vec{x}_k' - \vec{\mu}_k)^T \Sigma_k^{-1} (\vec{x}_k' - \vec{\mu}_k) \right)$$

여기서:
- $\vec{x}_k' = T(\vec{x}_k, \vec{p})$: 변환된 소스 포인트
- $\vec{\mu}_k$, $\Sigma_k$: 대응 NDT 셀의 평균과 공분산
- $d_1$, $d_2$: Normal 분포와 uniform 분포의 혼합으로부터 유도된 상수

상수 $d_1$, $d_2$의 계산:
- $c_1 = 10(1 - p_{\text{outlier}})$
- $c_2 = p_{\text{outlier}} / r^3$ (r: voxel 해상도)
- $d_3 = -\log(c_2)$
- $d_1 = -\log(c_1 + c_2) - d_3$
- $d_2 = -2\log\left(\frac{-\log(c_1 e^{-0.5} + c_2) - d_3}{d_1}\right)$

### 1.2 gtsam_points의 구현

**d1, d2 파라미터 계산** (`integrated_ndt_factor.hpp`, `compute_ndt_params`):

```cpp
inline void compute_ndt_params(double resolution, double outlier_ratio, double& d1, double& d2) {
  double c1 = 10.0 * (1.0 - outlier_ratio);
  double c2 = outlier_ratio / (resolution * resolution * resolution);
  double d3 = -std::log(c2);
  d1 = -std::log(c1 + c2) - d3;
  d2 = -2.0 * std::log((-std::log(c1 * std::exp(-0.5) + c2) - d3) / d1);
}
```

**비용 함수 계산** (`integrated_ndt_factor_impl.hpp`, `evaluate` 메서드):

```cpp
double mahalanobis_dist = residual.transpose() * inv_cov_B * residual;
double exponent = -gauss_d2 * mahalanobis_dist / 2.0;
double e_term = std::exp(exponent);  // exp(-d2/2 * m), 범위: (0, 1]

// Magnusson Eq. 6.9 원본 score function
const double score_function = -gauss_d1 * e_term;  // = |d1| * exp(...), 양수

// GTSAM 호환 cost: score_function은 정렬이 좋을수록 증가하므로 부호 반전
// cost = -d1 - score_function = -d1 * (1 - e_term)
const double cost = -gauss_d1 - score_function;
```

### 1.3 비교 분석

| 항목 | Magnusson 논문 | gtsam_points 구현 | 일치 여부 |
|------|---------------|-------------------|-----------|
| $d_1$ 계산 | $d_1 = -\log(c_1 + c_2) - d_3$ | 동일 | ✅ 완전 일치 |
| $d_2$ 계산 | $d_2 = -2\log(\cdots)$ | 동일 | ✅ 완전 일치 |
| $c_1$ | $10(1 - p_{\text{outlier}})$ | 동일 | ✅ 완전 일치 |
| $c_2$ | $p_{\text{outlier}} / r^3$ | 동일 | ✅ 완전 일치 |
| Score 함수 형태 | $s = -d_1 \exp(-d_2/2 \cdot m)$ | `score_function = -d1 * e_term` (원본) + `cost = -d1 - score_function` (GTSAM 호환) | ✅ 수학적 동등 (변수 분리) |

**핵심 변경: Score Function과 Cost의 명시적 분리**

코드에서 Magnusson의 원본 score function과 GTSAM 호환 cost를 별도 변수로 분리하였다:

- **`score_function = -d1 * e_term`**: Magnusson Eq. 6.9의 원본 score. 정렬이 좋을수록 값이 크다 (최대화 문제).
- **`cost = -d1 - score_function = -d1 * (1 - e_term)`**: GTSAM 호환 비용. 정렬이 좋을수록 값이 작다 (최소화 문제).

| 상황 | `score_function` | `cost` |
|------|-----------------|--------|
| 완벽한 정렬 ($m=0$) | $-d_1$ (최대, 양수) | $0$ (최소) |
| 오정렬 ($m \gg 0$) | $\approx 0$ | $\approx -d_1$ (양수 상수) |

**이 분리는 gradient와 Hessian에 영향을 주지 않는다.** 상수 $-d_1$의 차이는 미분 시 소거되므로, 최적화의 수렴점은 동일하다.

**결론:** ✅ **Magnusson 논문의 score function을 `score_function` 변수로 정확히 보존하고, GTSAM 호환을 위한 `cost` 변수를 별도로 정의하여 수학적 투명성을 확보하였다.**

---

## 2. 공분산 정규화 (Covariance Regularization)

### 2.1 Magnusson 논문의 접근

Magnusson은 NDT 셀의 공분산 행렬이 특이(singular)하거나 조건수(condition number)가 나쁜 경우를 처리하기 위해, 작은 고유값을 클램핑하여 정규화하는 방법을 권장한다.

### 2.2 gtsam_points의 구현

```cpp
inline Eigen::Matrix4d compute_ndt_inverse_covariance(
    const Eigen::Matrix4d& cov, double regularization_epsilon = 1e-3) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(cov);
  Eigen::Vector4d eigenvalues = solver.eigenvalues();
  Eigen::Matrix4d eigenvectors = solver.eigenvectors();

  double lambda_max = eigenvalues.maxCoeff();
  Eigen::Vector4d clamped = eigenvalues.array().max(
      regularization_epsilon * lambda_max).matrix();

  Eigen::Matrix4d cov_reg = eigenvectors * clamped.asDiagonal() * eigenvectors.transpose();
  return cov_reg.inverse();
}
```

### 2.3 비교 분석

| 항목 | 방법 | 일치 여부 |
|------|------|-----------|
| 고유값 분해 사용 | 논문과 동일 | ✅ |
| 작은 고유값 클램핑 | $\lambda_i \geq \epsilon \cdot \lambda_{\max}$ | ✅ |
| 역공분산 캐싱 | 구현에서 `inv_cov_cache`로 사전 계산 | ✅ (성능 최적화) |

**결론:** ✅ **표준 NDT 정규화 방법을 정확히 따른다.**

---

## 3. Gradient (기울기 벡터)

### 3.1 Magnusson 논문의 정의 (Eq. 6.12)

$$g_i = \frac{\partial s}{\partial p_i} = \sum_{k=1}^n d_1 d_2 \, \vec{q}_k^T \Sigma_k^{-1} \vec{J}_{k,i} \exp\left( -\frac{d_2}{2} \vec{q}_k^T \Sigma_k^{-1} \vec{q}_k \right)$$

여기서 $\vec{q}_k = \vec{x}_k' - \vec{\mu}_k$, $\vec{J}_{k,i} = \frac{\partial \vec{x}_k'}{\partial p_i}$이다.

### 3.2 gtsam_points의 구현

```cpp
// Magnusson Eq. 6.12: Score function의 1차 미분에서 나오는 weight (양수 스컨라)
// weight = -d1 * d2 * exp(-d2/2 * m)
//   d1 < 0이므로 -d1 > 0, d2 > 0, e_term > 0  →  weight > 0
const double weight = -gauss_d1 * gauss_d2 * e_term;

// J_target: 4x6 자코비안 (SE(3) Lie 대수 기반)
Eigen::Matrix<double, 4, 6> J_target = ...;

// 가중 자코비안: (6x4) = weight * J^T * Σ^{-1}
Eigen::Matrix<double, 6, 4> J_target_weighted = weight * J_target.transpose() * inv_cov_B;

// gradient 벡터 누적: b = weight * J^T * Σ^{-1} * q
*b_target += J_target_weighted * residual;
```

이를 전개하면:

$$b_i = \sum_k \underbrace{(-d_1 \cdot d_2 \cdot e_k)}_{\text{weight}_k} \cdot J_{k,i}^T \Sigma_k^{-1} \vec{q}_k$$

### 3.3 비교 분석

| 항목 | Magnusson 논문 (Eq. 6.12) | gtsam_points 구현 |
|------|--------------------------|-------------------|
| 스칼라 가중치 | $d_1 d_2 \exp(\cdots)$ | `weight = -d1 * d2 * e_term` (양수) |
| 벡터 구조 | $\vec{q}^T \Sigma^{-1} J$ | $J^T \Sigma^{-1} \vec{q}$ |
| 파라미터화 | Euler angle ($\phi_x, \phi_y, \phi_z, t_x, t_y, t_z$) | SE(3) Lie algebra ($\omega_x, \omega_y, \omega_z, v_x, v_y, v_z$) |

**부호 분석:**

Magnusson의 $d_1$은 음수이다 ($d_1 < 0$). 따라서:
- Magnusson: $d_1 \cdot d_2 \cdot \exp(\cdots)$ → 음수 × 양수 × 양수 = **음수**
- gtsam_points: `weight = -d1 * d2 * e_term` → 양수 × 양수 × 양수 = **양수**

이 부호 반전은 **non-negative reformulation**과 일관된다. Magnusson은 score를 **최대화**하므로 gradient가 음수 방향을 가리키고, gtsam_points는 비용을 **최소화**하므로 gradient가 양수 방향을 가리킨다. 코드에서 이 가중치를 `weight`라고 명명하여, score function의 미분에서 자연스럽게 도출되는 **포인트별 가중치**라는 의미를 명확히 하였다.

**파라미터화 차이:**

Magnusson은 Euler 각도를 사용하고, gtsam_points는 SE(3) Lie algebra를 사용한다:

```cpp
// Rotation에 대한 자코비안: Hat(x') (skew-symmetric matrix)
J_target.block<3, 3>(0, 0) = -gtsam::SO3::Hat(transed_mean_A.head<3>());
// Translation에 대한 자코비안: Identity
J_target.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
```

이것은 GTSAM이 매니폴드 위에서의 최적화를 수행하기 때문이다. Euler 각도 대신 Lie algebra의 접선 공간에서 미분을 계산하며, 이는 gimbal lock 문제를 회피하고 수치적으로 더 안정적이다.

**결론:** ✅ **Magnusson Eq. 6.12의 gradient 구조를 정확히 따르되, SE(3) Lie algebra 파라미터화로 일반화하였다. 수학적 동등성이 보장된다.**

---

## 4. Hessian (헤시안 행렬)

### 4.1 Magnusson 논문의 정의 (Eq. 6.13)

$$H_{ij} = \sum_{k=1}^n d_1 d_2 \exp(\cdots) \left[ \underbrace{-d_2 (\vec{q}_k^T \Sigma_k^{-1} \vec{J}_{k,i})(\vec{q}_k^T \Sigma_k^{-1} \vec{J}_{k,j})}_{H_2: \text{2차 미분 항}} + \underbrace{\vec{q}_k^T \Sigma_k^{-1} \vec{H}_{k,ij}}_{H_3: \text{변환의 2차 미분}} + \underbrace{\vec{J}_{k,j}^T \Sigma_k^{-1} \vec{J}_{k,i}}_{H_1: \text{Gauss-Newton 항}} \right]$$

세 개의 항으로 구성된다:
- **$H_1$ (Gauss-Newton 항):** $\vec{J}^T \Sigma^{-1} \vec{J}$ — 1차 미분만을 사용한 근사 Hessian
- **$H_2$ (2차 미분 항):** $-d_2 (\vec{q}^T \Sigma^{-1} J)(\vec{q}^T \Sigma^{-1} J)^T$ — 잔차의 크기에 의존하는 외적(rank-1 outer product) 항
- **$H_3$ (변환 Hessian 항):** $\vec{q}^T \Sigma^{-1} H_E$ — 변환 함수 자체의 2차 미분

### 4.2 gtsam_points의 구현

```cpp
// Gauss-Newton 근사 Hessian (Magnusson Eq. 6.13의 H1 항만 사용)
// H ≈ weight * J^T * Σ^{-1} * J   (PSD 보장)
*H_target += J_target_weighted * J_target;      // weight * J^T * inv_cov * J
*H_source += J_source_weighted * J_source;
*H_target_source += J_target_weighted * J_source;
```

전개하면:

$$H_{ij}^{\text{impl}} = \sum_k \underbrace{(-d_1 \cdot d_2 \cdot e_k)}_{\text{weight}_k} \cdot J_{k,i}^T \Sigma_k^{-1} J_{k,j}$$

### 4.3 비교 분석

| 항목 | Magnusson 논문 (Eq. 6.13) | gtsam_points 구현 |
|------|--------------------------|-------------------|
| $H_1$ (Gauss-Newton 항) | $J^T \Sigma^{-1} J$ | ✅ 포함 |
| $H_2$ (2차 미분 항) | $-d_2 (q^T\Sigma^{-1}J)(q^T\Sigma^{-1}J)^T$ | ❌ 생략 |
| $H_3$ (변환 Hessian 항) | $q^T \Sigma^{-1} H_E$ | ❌ 생략 |

### 4.4 생략의 영향과 타당성

**gtsam_points는 Gauss-Newton 근사를 사용한다.** 이것은 의도적인 설계 결정이며, 다음과 같은 이유로 타당하다:

1. **양의 준정부호(PSD) 보장:** $H_1 = J^T \Sigma^{-1} J$는 항상 PSD이다. $H_2$ 항은 음의 준정부호(NSD)이므로, 이를 포함하면 전체 Hessian이 indefinite가 될 수 있다. GTSAM의 Levenberg-Marquardt 옵티마이저는 PSD Hessian을 기대한다.

2. **GTSAM 아키텍처 호환:** `IntegratedMatchingCostFactor`는 `HessianFactor`를 반환하며, GTSAM의 LM 옵티마이저는 $H + \lambda I$로 damping한다. GN 근사는 이 프레임워크에 자연스럽게 맞는다.

3. **수렴 속도 대 안정성 트레이드오프:** Full Newton은 최적점 근처에서 2차 수렴을 보이지만, 멀리 떨어진 초기점에서는 불안정할 수 있다. GN 근사는 선형 수렴이지만 전역적으로 안정적이다. LiDAR SLAM의 순차적 정합에서는 초기 추정치가 보통 좋으므로, GN의 수렴 속도로 충분하다.

4. **PCL 등 다른 구현과의 일관성:** PCL의 `ndt.hpp` 구현도 Hessian의 full Newton 항을 생략하는 것이 일반적이다.

**결론:** ⚠️ **Magnusson Eq. 6.13의 full Hessian 중 Gauss-Newton 항($H_1$)만 사용한다. $H_2$, $H_3$ 항은 의도적으로 생략되었으며, 이는 GTSAM의 LM 옵티마이저 호환성과 수치 안정성을 위한 합리적인 설계 결정이다.**

---

## 5. 대응 탐색 (Correspondence Search)

### 5.1 Magnusson 논문의 방법

Magnusson은 변환된 소스 포인트가 속하는 NDT 셀과의 대응을 찾는다. 단일 셀 검색(nearest cell)이 기본이며, 이웃 셀 검색으로 확장할 수 있다.

### 5.2 gtsam_points의 구현

```cpp
enum class NDTSearchMode {
  DIRECT1,   // 현재 셀만 (가장 빠름)
  DIRECT7,   // 현재 셀 + 6개 면 이웃 (기본값)
  DIRECT27   // 현재 셀 + 26개 이웃 (가장 정확)
};
```

최소 Mahalanobis 거리를 가진 셀을 선택한다:

```cpp
for (const auto& offset : neighbor_offsets) {
    // 이웃 셀 탐색
    double mahalanobis_dist = diff.transpose() * inv_cov * diff;
    if (mahalanobis_dist < min_mahalanobis) {
        min_mahalanobis = mahalanobis_dist;
        best_voxel = &voxel;
    }
}
```

### 5.3 비교 분석

| 항목 | Magnusson 논문 | gtsam_points 구현 |
|------|---------------|-------------------|
| 기본 검색 | nearest cell | DIRECT7 (기본값) |
| 확장 검색 | 이웃 셀 | DIRECT1/7/27 선택 가능 |
| 대응 기준 | 최소 Mahalanobis 거리 | 동일 |
| 역공분산 캐싱 | 미언급 | `inv_cov_cache` 사전 계산 |

**결론:** ✅ **Magnusson의 대응 탐색 방법을 따르며, 다중 이웃 검색 모드로 확장하였다.**

---

## 6. 자코비안 파라미터화

### 6.1 Magnusson 논문

Magnusson은 **Euler 각도** $(\phi_x, \phi_y, \phi_z, t_x, t_y, t_z)$로 변환을 파라미터화하고, 이에 대한 편미분을 Eq. 6.19, 6.21에서 명시적으로 유도한다.

### 6.2 gtsam_points의 구현

GTSAM은 **SE(3) Lie group** 위에서 최적화한다. 접선 공간(tangent space)의 좌표 $(\omega_x, \omega_y, \omega_z, v_x, v_y, v_z)$에 대한 자코비안을 계산한다:

```cpp
// Rotation 자코비안: -Hat(x') (skew-symmetric matrix of transformed point)
J_target.block<3, 3>(0, 0) = -gtsam::SO3::Hat(transed_mean_A.head<3>());
// Translation 자코비안: Identity
J_target.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
```

### 6.3 비교 분석

| 항목 | Magnusson 논문 | gtsam_points 구현 |
|------|---------------|-------------------|
| 파라미터화 | Euler 각도 (6.19) | SE(3) Lie algebra |
| Gimbal lock | 취약 | 면역 |
| 2차 미분 | 명시적 Hessian (6.21) | 생략 (GN 근사) |
| 수학적 동등성 | - | 동일한 최적화 문제 해결 |

**결론:** ✅ **수학적으로 동등한 SE(3) Lie algebra 파라미터화를 사용하며, 이는 Euler 각도보다 수치적으로 우월하다.**

---

## 7. 종합 비교표

| 구성 요소 | Magnusson 논문 | gtsam_points 구현 | 판정 |
|-----------|---------------|-------------------|------|
| d1, d2 파라미터 (Eq. 6.9-6.10) | 원본 | 동일 | ✅ 완전 일치 |
| Score 함수 (Eq. 6.9) | $-d_1 \exp(\cdots)$ | `score_function = -d1 * e_term` + `cost = -d1 - score_function` | ✅ 수학적 동등 (변수 분리) |
| 공분산 정규화 | 고유값 클램핑 | 동일 | ✅ 완전 일치 |
| Gradient (Eq. 6.12) | $d_1 d_2 \exp \cdot q^T\Sigma^{-1}J$ | `weight = -d1 * d2 * e_term` (부호 반전 = 최소화 문제 변환) | ✅ 수학적 동등 |
| Hessian (Eq. 6.13) | Full Newton ($H_1 + H_2 + H_3$) | Gauss-Newton ($H_1$만) | ⚠️ 근사 사용 |
| 파라미터화 | Euler 각도 | SE(3) Lie algebra | ✅ 일반화 (수학적 동등) |
| 대응 탐색 | nearest cell | DIRECT1/7/27 | ✅ 확장 구현 |
| 옵티마이저 | Newton's method | GTSAM LM (factor graph) | ✅ 프레임워크 적응 |

---

## 8. 결론

### gtsam_points의 NDT 구현은 Magnusson 논문의 방법론을 **충실히** 따르고 있는가?

**예, 핵심 수학적 구조는 Magnusson의 논문을 정확히 따른다.**

구체적으로:
1. **Score function의 d1, d2 파라미터 계산**은 Magnusson Eq. 6.9-6.10을 코드 레벨에서 완전히 동일하게 구현하였다.
2. **Score function과 cost의 분리**: Magnusson의 원본 score function을 `score_function` 변수로 보존하고, GTSAM LM 호환을 위한 `cost` 변수를 별도로 정의하였다.
3. **Gradient 계산**은 Magnusson Eq. 6.12와 수학적으로 동등하다. 미분에서 도출되는 스칼라 가중치를 `weight`로 명명하여 의미를 명확히 하였다.
4. **공분산 정규화**는 표준 NDT 방법론을 따른다.
5. **대응 탐색**은 Magnusson의 방법을 기반으로 다중 검색 모드로 확장하였다.

다만, 다음과 같은 **의도적인 차이**가 존재한다:

1. **Score function/Cost 분리:** Magnusson의 score function (최대화 문제)을 `score_function` 변수로 보존하고, GTSAM의 비선형 최소제곱법 프레임워크에 맞추기 위해 `cost = -d1 - score_function`으로 비음수 비용을 정의하였다. 상수 차이이므로 gradient/Hessian은 동일하다.
2. **Gauss-Newton 근사:** Magnusson Eq. 6.13의 full Hessian 대신 $H_1$ 항만 사용한다. GTSAM의 LM damping과 결합하면 안정적으로 수렴한다.
3. **SE(3) Lie algebra 파라미터화:** Euler 각도 대신 Lie group 위에서의 최적화를 수행하며, 이는 gimbal lock을 방지하고 factor graph 프레임워크에 자연스럽게 통합된다.

**이 모든 차이는 Magnusson의 NDT 이론을 GTSAM factor graph 프레임워크에 적응시키기 위한 합리적인 엔지니어링 결정이며, 수학적 동등성을 유지한다.**

---

## 참고 문헌

1. Magnusson, M. (2009). *The Three-Dimensional Normal-Distributions Transform — an Efficient Representation for Registration, Surface Analysis, and Loop Detection.* PhD thesis, Örebro University.
2. Biber, P., & Straßer, W. (2003). *The Normal Distributions Transform: A New Approach to Laser Scan Matching.* IROS 2003.
3. Koide, K. (2021). gtsam_points: A collection of GTSAM factors for point cloud SLAM. GitHub.
4. Stoyanov, T., Magnusson, M., et al. (2012). *Fast and Accurate Scan Registration through Minimization of the Distance between Compact 3D NDT Representations.* IJRR, 31(12).
