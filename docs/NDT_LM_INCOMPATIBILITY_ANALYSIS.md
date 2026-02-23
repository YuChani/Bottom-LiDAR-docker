# NDT와 Levenberg-Marquardt 최적화의 비호환성 분석

## 목차
1. [개요](#1-개요)
2. [NDT가 느린 이유 (29,232ms)](#2-ndt가-느린-이유-29232ms)
3. [gtsam_points 프레임워크와의 구조적 불일치](#3-gtsam_points-프레임워크와의-구조적-불일치)
4. [Newton's Method vs Levenberg-Marquardt: 핵심 비호환성](#4-newtons-method-vs-levenberg-marquardt-핵심-비호환성)
5. [PCL 구현과의 비교](#5-pcl-구현과의-비교)
6. [수학적 상세 분석](#6-수학적-상세-분석)
7. [결론 및 개선 방향](#7-결론-및-개선-방향)

---

## 1. 개요

벤치마크 실험 결과, NDT (Normal Distributions Transform)는 **29,232ms**로 6개 알고리즘 중 가장 느린 수행 시간을 기록했다. 이는 두 번째로 느린 GICP(11,480ms)의 약 2.5배, 가장 빠른 LOAM(65ms)의 약 450배에 달하는 수치이다.

| 알고리즘 | 수행 시간 (ms) | Mean Translation (m) | Mean Rotation (°) |
|----------|---------------|---------------------|-------------------|
| Point-to-Point ICP | 9,253 | 0.095 | 0.488 |
| Point-to-Plane ICP | 8,130 | 0.062 | 0.449 |
| GICP | 11,480 | 0.084 | 0.551 |
| VGICP | 9,589 | 0.216 | 1.038 |
| LOAM | 65 | 0.289 | 1.048 |
| **NDT** | **29,232** | **0.078** | **0.510** |

NDT의 정합 정확도(Mean T: 0.078m, Mean R: 0.510°)는 GICP와 유사한 수준으로 우수하지만, 수행 시간이 비정상적으로 길다. 본 문서에서는 이 현상의 근본 원인을 **세 가지 관점**에서 분석한다:

1. **계산량 관점**: NDT의 correspondence 탐색과 비용 함수 계산이 본질적으로 비싼 이유
2. **프레임워크 관점**: gtsam_points의 `IntegratedMatchingCostFactor`가 최소제곱(LSQ) 기반으로 설계되어 NDT와 구조적으로 불일치하는 이유
3. **최적화 이론 관점**: NDT가 Newton's method를 전제로 설계되었는데 Levenberg-Marquardt(LM)의 Gauss-Newton 근사와 결합 시 발생하는 수학적 비호환성

---

## 2. NDT가 느린 이유 (29,232ms)

### 2.1 DIRECT7 Correspondence 탐색의 높은 비용

NDT의 기본 탐색 모드는 `DIRECT7`로, 소스 포인트 하나당 **7개의 복셀**(현재 복셀 + 6개 면 이웃)을 탐색한다.

**코드 (`integrated_ndt_factor_impl.hpp`, 라인 118-177)**:
```cpp
case NDTSearchMode::DIRECT7:
    neighbor_offsets.push_back(Eigen::Vector3i(0, 0, 0));   // 현재 복셀
    neighbor_offsets.push_back(Eigen::Vector3i(1, 0, 0));   // +x
    neighbor_offsets.push_back(Eigen::Vector3i(-1, 0, 0));  // -x
    neighbor_offsets.push_back(Eigen::Vector3i(0, 1, 0));   // +y
    neighbor_offsets.push_back(Eigen::Vector3i(0, -1, 0));  // -y
    neighbor_offsets.push_back(Eigen::Vector3i(0, 0, 1));   // +z
    neighbor_offsets.push_back(Eigen::Vector3i(0, 0, -1));  // -z

// 각 이웃에 대해 Mahalanobis 거리를 계산하여 최적 복셀 선택
for (const auto& offset : neighbor_offsets) {
    Eigen::Vector3i neighbor_coord = coord + offset;
    const auto voxel_id = target_voxels->lookup_voxel_index(neighbor_coord);
    if (voxel_id < 0) continue;

    const auto& voxel = target_voxels->lookup_voxel(voxel_id);
    const Eigen::Matrix4d& inv_cov = inv_cov_cache[voxel_id];

    Eigen::Vector4d diff = pt - voxel.mean;
    double mahalanobis_dist = diff.transpose() * inv_cov * diff;  // 4x4 행렬곱

    if (mahalanobis_dist < min_mahalanobis) {
        min_mahalanobis = mahalanobis_dist;
        best_voxel = &voxel;
        best_inv_cov = inv_cov;
    }
}
```

**포인트당 연산 비용 비교**:

| 알고리즘 | Correspondence 방식 | 포인트당 탐색 비용 |
|----------|---------------------|-------------------|
| GICP | KdTree 1-NN | 1회 트리 탐색 (O(log n)) |
| VGICP | 복셀 해시 1회 | 1회 해시 룩업 |
| **NDT** | **복셀 해시 7회 + Mahalanobis 7회** | **7회 해시 룩업 + 7회 4×4 행렬곱** |

소스 포인트 클라우드의 크기가 약 50,000점이라면, NDT는 매 correspondence 업데이트마다:
- **350,000회**의 해시 룩업 (50,000 × 7)
- **350,000회**의 4×4 Mahalanobis 거리 계산
을 수행해야 한다. VGICP의 50,000회와 비교하면 **7배**의 탐색 비용이 발생한다.

### 2.2 역공분산 행렬 계산의 초기 비용

NDT는 각 복셀에 대해 **4×4 고유값 분해(eigendecomposition)**를 수행하여 역공분산 행렬을 계산한다.

**코드 (`integrated_ndt_factor.hpp`, 라인 44-54)**:
```cpp
inline Eigen::Matrix4d compute_ndt_inverse_covariance(
    const Eigen::Matrix4d& cov, double regularization_epsilon = 1e-3) {
    
    // 1. 4×4 고유값 분해 (O(n³) = O(64))
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(cov);
    Eigen::Vector4d eigenvalues = solver.eigenvalues();
    Eigen::Matrix4d eigenvectors = solver.eigenvectors();

    // 2. 고유값 정규화: 작은 고유값을 epsilon * lambda_max로 클램핑
    double lambda_max = eigenvalues.maxCoeff();
    Eigen::Vector4d clamped = eigenvalues.array()
        .max(regularization_epsilon * lambda_max).matrix();

    // 3. 정규화된 공분산 재구성 후 역행렬
    Eigen::Matrix4d cov_reg = eigenvectors * clamped.asDiagonal() * eigenvectors.transpose();
    return cov_reg.inverse();  // 4×4 역행렬
}
```

이 연산은 복셀당 수행되며, 캐싱(`inv_cov_cached`)이 적용되어 **한 번만 계산**된다. 그러나 복셀맵의 크기가 수만 개에 달할 경우 초기 비용이 상당하다.

**GICP와의 차이**: GICP는 포인트별 공분산을 전처리 단계에서 계산하며, NDT처럼 별도의 고유값 분해 + 정규화 과정을 거치지 않는다.

### 2.3 `compute_ndt_params()`의 불필요한 반복 호출

**코드 (`integrated_ndt_factor_impl.hpp`, 라인 115-116)**:
```cpp
correspondences.resize(frame::size(*source));
compute_ndt_params(resolution, outlier_ratio, gauss_d1, gauss_d2);  // 매번 재계산!
```

`compute_ndt_params()`는 `resolution`과 `outlier_ratio`로부터 가우시안 파라미터 `d1`, `d2`를 계산하는데, 이 값들은 **최적화 과정에서 절대 변하지 않는 상수**이다. 그럼에도 `update_correspondences()`가 호출될 때마다 다시 계산된다.

**`compute_ndt_params()`의 내부 연산 (`integrated_ndt_factor.hpp`, 라인 67-73)**:
```cpp
inline void compute_ndt_params(double resolution, double outlier_ratio, 
                                double& d1, double& d2) {
    double c1 = 10.0 * (1.0 - outlier_ratio);    // c1 = 10.0 * 0.9 = 9.0
    double c2 = outlier_ratio / (resolution³);     // c2 = 0.1 / 0.125 = 0.8
    double d3 = -log(c2);                          // d3 ≈ 0.223
    d1 = -log(c1 + c2) - d3;                       // d1 ≈ -2.505 (음수)
    d2 = -2.0 * log((-log(c1*exp(-0.5) + c2) - d3) / d1);  // d2 > 0
}
```

`log()`, `exp()` 등 초월함수가 포함되어 있어, 비록 단일 호출의 비용은 작지만 불필요한 반복이다. 이 값은 생성자 또는 파라미터 설정 시 한 번만 계산하면 된다.

### 2.4 evaluate() 내부의 지수함수 연산

NDT의 비용 함수는 **지수함수(exponential)**를 포함하여, GICP/ICP의 단순 이차형식(quadratic form)보다 포인트당 계산 비용이 높다.

**NDT evaluate() — 포인트당 핵심 연산 (`integrated_ndt_factor_impl.hpp`, 라인 240-282)**:
```cpp
// 1. Mahalanobis 거리 (4×4 행렬곱)
double mahalanobis_dist = residual.transpose() * inv_cov_B * residual;

// 2. 지수함수 계산 (exp()는 CPU에서 ~20-100 clock cycles)
double exponent = -gauss_d2 * mahalanobis_dist / 2.0;
double e_term = std::exp(exponent);          // <-- 비싼 연산

// 3. 비용 계산
const double error = -gauss_d1 * (1.0 - e_term);

// 4. derivative_scale 계산 (추가 곱셈)
double derivative_scale = -gauss_d1 * gauss_d2 * e_term;  // <-- e_term 재사용

// 5. 가중 야코비안 (derivative_scale이 곱해짐)
Eigen::Matrix<double, 6, 4> J_target_weighted = derivative_scale * J_target.transpose() * inv_cov_B;

// 6. 헤시안 누적
*H_target += J_target_weighted * J_target;
```

**GICP evaluate() — 포인트당 핵심 연산 (`integrated_gicp_factor_impl.hpp`, 라인 266-286)**:
```cpp
// 1. 이차형식 (4×4 행렬곱)
const double error = residual.transpose() * mahalanobis * residual;

// 2. 가중 야코비안 (곱셈만, exp() 없음)
Eigen::Matrix<double, 6, 4> J_target_mahalanobis = J_target.transpose() * mahalanobis;

// 3. 헤시안 누적
*H_target += J_target_mahalanobis * J_target;
```

**차이점 요약**:
- NDT: Mahalanobis + `exp()` + `derivative_scale` 곱셈 + 가중 야코비안
- GICP: Mahalanobis + 가중 야코비안 (exp() 없음)

50,000포인트 × LM 반복횟수만큼 이 차이가 누적된다.

### 2.5 LM 반복 횟수 증가 효과 (핵심)

NDT가 느린 **가장 큰 원인**은 단일 연산의 비용이 아니라, **LM 최적화기가 더 많은 반복을 필요로 한다**는 점이다.

LM의 각 반복에서는:
1. `linearize()` 호출 → `update_correspondences()` + `evaluate()`
2. 선형 시스템 풀기 (Hx = -b)
3. 갱신 적용 및 비용 감소 확인

NDT의 비이차적(non-quadratic) 비용 곡면에서 LM의 Gauss-Newton 근사가 부정확하기 때문에:
- 매 스텝의 갱신량(step size)이 보수적이 됨
- 비용 감소가 기대보다 적어 LM이 damping factor λ를 증가시킴
- λ가 커지면 gradient descent 방향으로 퇴화 → 수렴 속도 저하
- **더 많은 linearize() 호출** = 더 많은 correspondence 탐색(7배 비용) + evaluate(exp() 비용)

이것이 NDT의 29,232ms가 단순히 "연산이 비싸서"가 아니라 **"LM과의 비호환성으로 인한 반복 횟수 폭증"** 때문임을 의미한다. 이에 대한 수학적 분석은 4장에서 상세히 다룬다.

---

## 3. gtsam_points 프레임워크와의 구조적 불일치

### 3.1 IntegratedMatchingCostFactor의 설계 전제

gtsam_points의 모든 스캔 매칭 팩터는 `IntegratedMatchingCostFactor` 기반 클래스를 상속한다.

**코드 (`integrated_matching_cost_factor.hpp`, 라인 16-18)**:
```cpp
/**
 * @brief Abstraction of LSQ-based scan matching constraints between point clouds
 */
class IntegratedMatchingCostFactor : public gtsam::NonlinearFactor {
```

이 클래스의 주석이 명시적으로 밝히고 있듯이, 이 프레임워크는 **"LSQ(Least Squares)-기반 스캔 매칭"**을 위해 설계되었다. 이는 비용 함수가 다음과 같은 형태를 가정한다:

$$E(\mathbf{p}) = \sum_{i} \| \mathbf{r}_i(\mathbf{p}) \|^2_{\mathbf{W}_i} = \sum_{i} \mathbf{r}_i^T \mathbf{W}_i \mathbf{r}_i$$

여기서 $\mathbf{r}_i$는 잔차 벡터, $\mathbf{W}_i$는 가중 행렬이다.

### 3.2 linearize()가 반환하는 HessianFactor

`linearize()` 메서드는 `gtsam::HessianFactor`를 반환한다:

**코드 (`integrated_matching_cost_factor.cpp`, 라인 37-55)**:
```cpp
gtsam::GaussianFactor::shared_ptr IntegratedMatchingCostFactor::linearize(
    const gtsam::Values& values) const {
    
    auto delta = calc_delta(values);
    update_correspondences(delta);
    
    Eigen::Matrix<double, 6, 6> H_target, H_source, H_target_source;
    Eigen::Matrix<double, 6, 1> b_target, b_source;
    // ... 초기화 ...
    
    double error = evaluate(delta, &H_target, &H_source, &H_target_source, 
                           &b_target, &b_source);
    
    // HessianFactor로 패키징하여 반환
    return gtsam::HessianFactor::shared_ptr(
        new gtsam::HessianFactor(..., H_target, H_target_source, b_target, 
                                      H_source, b_source, error));
}
```

GTSAM의 `HessianFactor`는 **이차 근사**를 표현한다:

$$E(\mathbf{p} + \delta\mathbf{p}) \approx E(\mathbf{p}) + \mathbf{b}^T \delta\mathbf{p} + \frac{1}{2} \delta\mathbf{p}^T \mathbf{H} \delta\mathbf{p}$$

이 근사가 정확하려면, $\mathbf{H}$와 $\mathbf{b}$가 비용 함수의 **실제** 2차 미분과 1차 미분이어야 한다.

### 3.3 GICP: 프레임워크와 완벽한 적합

GICP의 비용 함수는 Mahalanobis 거리의 합이다:

$$E_{\text{GICP}} = \sum_{i} \mathbf{r}_i^T \mathbf{W}_i \mathbf{r}_i$$

이것은 **정확히 최소제곱 형태**이다. Gauss-Newton 헤시안은:

$$\mathbf{H}_{\text{GN}} = \sum_{i} \mathbf{J}_i^T \mathbf{W}_i \mathbf{J}_i$$

여기서 $\mathbf{J}_i = \frac{\partial \mathbf{r}_i}{\partial \mathbf{p}}$는 4×6 야코비안이다.

**코드에서 확인 (`integrated_gicp_factor_impl.hpp`, 라인 279-286)**:
```cpp
Eigen::Matrix<double, 6, 4> J_target_mahalanobis = J_target.transpose() * mahalanobis;
*H_target += J_target_mahalanobis * J_target;    // H += J^T W J
*b_target += J_target_mahalanobis * residual;    // b += J^T W r
```

GICP는:
- 잔차 $\mathbf{r}_i$가 4차원 벡터 → 각 포인트가 헤시안에 **rank-3~4 기여**
- Gauss-Newton 근사가 잔차가 작을 때 정확
- `HessianFactor`의 이차 근사가 비용 곡면을 잘 반영

### 3.4 NDT: 프레임워크와의 근본적 불일치

NDT의 비용 함수는 **지수함수 기반**이다 (Magnusson 2009, Eq. 6.9):

$$E_{\text{NDT}} = \sum_{i} \left[ -d_1 \cdot \exp\!\left( -\frac{d_2}{2} \cdot \mathbf{r}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i \right) \right]$$

gtsam_points의 구현에서는 비음수 변환을 적용한다:

$$E_{\text{NDT}}^{\text{code}} = \sum_{i} \left[ -d_1 \cdot \left(1 - \exp\!\left( -\frac{d_2}{2} \cdot \mathbf{r}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i \right) \right) \right]$$

여기서 $d_1 < 0$이므로 $-d_1 > 0$이고, $E_{\text{NDT}}^{\text{code}} \geq 0$.

**이것은 최소제곱 형태가 아니다.** 비용 함수 안에 `exp()`가 존재하여:
- 비용 곡면이 이차형식(quadratic)이 아닌 **가우시안 형태**
- 잔차와 비용의 관계가 비선형적
- Gauss-Newton의 $\mathbf{H} \approx \mathbf{J}^T \mathbf{J}$ 근사가 성립하지 않음

**코드에서의 "유사 Gauss-Newton" 구조 (`integrated_ndt_factor_impl.hpp`, 라인 271-281)**:
```cpp
// derivative_scale = -d1 * d2 * exp(-d2/2 * mahal_dist)
double derivative_scale = -gauss_d1 * gauss_d2 * e_term;

// "가중 야코비안" = derivative_scale * J^T * Σ^{-1}
Eigen::Matrix<double, 6, 4> J_target_weighted = 
    derivative_scale * J_target.transpose() * inv_cov_B;

// 헤시안 누적: H += derivative_scale * J^T * Σ^{-1} * J
*H_target += J_target_weighted * J_target;

// 그래디언트 누적: b += derivative_scale * J^T * Σ^{-1} * r
*b_target += J_target_weighted * residual;
```

이 코드가 만들어내는 헤시안은:

$$\mathbf{H}_{\text{NDT}}^{\text{code}} = \sum_{i} \underbrace{(-d_1 \cdot d_2 \cdot e^{s_i})}_{\text{derivative\_scale}} \cdot \mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{J}_i$$

여기서 $s_i = -\frac{d_2}{2} \mathbf{r}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i$.

이것은 `derivative_scale`이라는 **포인트별 가중치**가 추가된 형태로, 표준 Gauss-Newton($\mathbf{J}^T\mathbf{J}$)도 아니고 완전한 Newton 헤시안도 아닌 **중간 형태**이다.

### 3.5 derivative_scale의 물리적 의미와 문제점

`derivative_scale = -d1 * d2 * exp(s_i)`의 행동을 분석하면:

| 정합 상태 | Mahalanobis 거리 | exp(s_i) | derivative_scale |
|----------|-----------------|----------|-----------------|
| **완벽 정합** (r ≈ 0) | ≈ 0 | ≈ 1.0 | 최대 |
| 보통 정합 | 중간 | 중간 | 중간 |
| **불량 정합** (r 큼) | 큼 | ≈ 0 | ≈ 0 |

**문제**: 이미 잘 정합된 포인트가 헤시안에 **더 크게** 기여하고, 정합이 안 된 포인트는 기여가 거의 없다. 이는:
- 최적화 초기(잔차가 클 때): 대부분의 포인트가 거의 기여하지 않아 헤시안이 **ill-conditioned**
- 최적화 후기(잔차가 작을 때): 잘 되어 있는 포인트만 기여하여 **특정 방향으로만 잘 조건화**

반면 GICP에서는 가중 행렬 $\mathbf{W}_i$가 포인트의 기하학적 불확실성에만 의존하고 정합 품질과 무관하므로, 모든 포인트가 일관적으로 헤시안에 기여한다.

---

## 4. Newton's Method vs Levenberg-Marquardt: 핵심 비호환성

### 4.1 원본 NDT의 최적화 방법 (Magnusson 2009)

Magnusson의 박사 학위 논문(2009)에서 NDT는 **Newton's method**로 최적화하도록 설계되었다:

$$\mathbf{p}_{k+1} = \mathbf{p}_k - \mathbf{H}^{-1} \mathbf{g}$$

여기서:
- $\mathbf{g} = \frac{\partial E}{\partial \mathbf{p}}$는 **1차 미분(gradient)**
- $\mathbf{H} = \frac{\partial^2 E}{\partial \mathbf{p}^2}$는 **완전한 2차 미분(full Hessian)**

NDT 비용 함수의 완전한 헤시안은 (포인트 $i$ 에 대해):

$$\mathbf{H}_i^{\text{full}} = \frac{\partial^2 E_i}{\partial \mathbf{p}^2}$$

$E_i = -d_1 \cdot \exp(s_i)$이고 $s_i = -\frac{d_2}{2} \mathbf{r}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i$일 때, 연쇄 법칙을 적용하면:

$$\mathbf{H}_i^{\text{full}} = -d_1 \cdot e^{s_i} \left[ \frac{\partial s_i}{\partial \mathbf{p}} \frac{\partial s_i}{\partial \mathbf{p}}^T + \frac{\partial^2 s_i}{\partial \mathbf{p}^2} \right]$$

여기서:
- $\frac{\partial s_i}{\partial \mathbf{p}} = -d_2 \cdot \mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i$ (6×1 벡터)
- $\frac{\partial^2 s_i}{\partial \mathbf{p}^2} = -d_2 \left( \mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{J}_i + \sum_k (\boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i)_k \frac{\partial^2 r_{ik}}{\partial \mathbf{p}^2} \right)$

따라서 완전한 헤시안은:

$$\boxed{ \mathbf{H}_i^{\text{full}} = -d_1 e^{s_i} \left[ d_2^2 (\mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i)(\mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i)^T - d_2 \mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{J}_i + \text{(2차 잔차 미분 항)} \right] }$$

### 4.2 gtsam_points에서의 근사 헤시안

gtsam_points의 NDT 구현이 계산하는 헤시안은:

$$\mathbf{H}_i^{\text{code}} = \underbrace{(-d_1 \cdot d_2 \cdot e^{s_i})}_{\text{derivative\_scale}} \cdot \mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{J}_i$$

이것은 4.1절의 완전한 헤시안에서 **두 가지 항을 누락**시킨 것이다:

#### 누락된 항 1: 외적 항 (Outer Product Term)

$$\Delta \mathbf{H}_1 = -d_1 \cdot d_2^2 \cdot e^{s_i} \cdot (\mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i)(\mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i)^T$$

이 항의 크기는 $d_2 \cdot \| \boldsymbol{\Sigma}_i^{-1/2} \mathbf{r}_i \|^2$에 비례한다. 즉:
- **잔차가 클 때** (최적화 초기): 이 항이 $\mathbf{J}^T \boldsymbol{\Sigma}^{-1} \mathbf{J}$ 항과 비슷하거나 더 큼
- **잔차가 작을 때** (수렴 근처): 이 항은 무시 가능

따라서 **최적화 초기에 헤시안의 심각한 오차**가 발생한다.

#### 누락된 항 2: 2차 잔차 미분 항 (Second-order Residual Term)

$$\Delta \mathbf{H}_2 = -d_1 \cdot d_2 \cdot e^{s_i} \cdot \sum_k (\boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i)_k \frac{\partial^2 r_{ik}}{\partial \mathbf{p}^2}$$

이 항은 잔차의 2차 미분 $\frac{\partial^2 r_{ik}}{\partial \mathbf{p}^2}$을 포함한다. SE(3) 위의 변환에서 회전 성분의 2차 미분은 0이 아니므로, 이 항도 일반적으로 무시할 수 없다.

### 4.3 Gauss-Newton 근사의 유효 조건과 NDT에서의 위반

표준 최소제곱 문제 $E = \sum_i \| \mathbf{r}_i \|^2$에서 Gauss-Newton 근사 $\mathbf{H} \approx \mathbf{J}^T \mathbf{J}$가 유효한 조건은:

> **잔차가 충분히 작아서 2차 항 $\sum_i r_i \cdot \nabla^2 r_i$가 무시 가능할 때**

NDT에서는 이 조건이 **이중으로** 위반된다:

1. **비용 함수 구조 위반**: NDT의 비용은 $E_i = f(\| \mathbf{r}_i \|^2_{\boldsymbol{\Sigma}^{-1}})$로, 잔차의 제곱합이 아닌 **잔차의 제곱의 함수**이다. 이 비선형 래핑(exponential wrapping)은 추가적인 미분 항을 생성한다.

2. **잔차 크기 조건 위반**: NDT의 가우시안 모델에서 대응점 사이의 거리가 복셀 크기(resolution) 수준일 수 있어, "잔차가 작다"는 가정이 최적화 과정 상당 부분에서 성립하지 않는다.

### 4.4 헤시안 조건수(Condition Number) 문제

#### NDT: Rank-1 기여

NDT에서 각 포인트의 비용은 **스칼라**이다:

$$E_i = -d_1 (1 - e^{s_i})$$

$s_i$는 6개 파라미터의 스칼라 함수이므로, 그래디언트 $\mathbf{g}_i = \frac{\partial E_i}{\partial \mathbf{p}}$는 6×1 벡터이다. 완전한 Newton 헤시안의 외적 항은:

$$\mathbf{g}_i \mathbf{g}_i^T \quad (\text{rank-1 행렬})$$

따라서 단일 포인트의 헤시안 기여는 최대 **rank-1**이다. 6×6 헤시안을 완전히 채우려면 최소 6개의 독립적인 방향의 포인트가 필요하다.

gtsam_points의 근사 헤시안에서는:

$$\mathbf{H}_i^{\text{code}} = \alpha_i \cdot \mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{J}_i$$

$\mathbf{J}_i$는 4×6이고 $\boldsymbol{\Sigma}_i^{-1}$은 4×4이므로, $\mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{J}_i$는 최대 rank-3이다 (4차원 중 동차좌표 성분이 0이므로 유효 차원은 3). 그러나 `derivative_scale`이 0에 가까운 포인트가 많아 **유효 기여 포인트 수가 감소**한다.

#### GICP: Rank-3~4 기여

GICP에서 각 포인트의 잔차는 **4차원 벡터**이다:

$$\mathbf{r}_i = \bar{\mathbf{b}}_i - T \bar{\mathbf{a}}_i$$

헤시안 기여:
$$\mathbf{J}_i^T \mathbf{W}_i \mathbf{J}_i \quad (\text{rank-3~4 행렬})$$

모든 포인트가 (derivative_scale 없이) 동등하게 기여하므로, 소수의 포인트만으로도 6×6 헤시안이 잘 조건화(well-conditioned)된다.

#### 조건수 비교 (정성적)

| 특성 | GICP | NDT |
|-----|------|-----|
| 포인트당 헤시안 rank | 3~4 | 1~3 (실질적으로 감소) |
| 가중치 변동 | 일정 (기하학적) | 0~최대 (정합 품질 의존) |
| 유효 기여 포인트 비율 | ~100% | 초기: <<100%, 후기: ~100% |
| 헤시안 조건수 | 양호 | 불량 (특히 초기) |

### 4.5 LM Damping과의 상호작용

Levenberg-Marquardt는 헤시안에 damping을 추가한다:

$$(\mathbf{H} + \lambda \mathbf{I}) \delta\mathbf{p} = -\mathbf{g}$$

**GICP에서의 LM**:
- $\mathbf{H}$가 잘 조건화됨 → 작은 $\lambda$로 충분
- 갱신 방향이 Newton 방향에 가까움
- 빠른 이차 수렴 (superlinear convergence)

**NDT에서의 LM**:
- $\mathbf{H}$가 불완전하고 ill-conditioned → LM이 큰 $\lambda$를 설정
- $\lambda \gg \|\mathbf{H}\|$이면: $\delta\mathbf{p} \approx -\frac{1}{\lambda}\mathbf{g}$ (순수 gradient descent)
- Gradient descent는 지수함수 비용 곡면에서 **선형 수렴** (매우 느림)
- 스텝이 작아 비용 감소가 미미 → λ가 더 증가 → **악순환**

이것이 NDT가 LM 아래에서 수렴에 많은 반복을 필요로 하는 수학적 메커니즘이다.

### 4.6 수렴 차수 비교

| 최적화 방법 | NDT와의 수렴 차수 |
|------------|------------------|
| **Newton's method** (Full Hessian) | **이차 수렴 (quadratic)**: 매 스텝마다 유효 자릿수 2배 |
| Gauss-Newton (gtsam_points 구현) | 초선형~선형 수렴: 누락된 헤시안 항 때문에 이차 수렴 불가 |
| LM + 불량 헤시안 | **선형 수렴 (linear)**: λ가 클 때 gradient descent로 퇴화 |

동일한 수렴 정밀도(예: 1e-5)에 도달하는 데 필요한 반복 횟수:
- Newton: ~5-10회
- LM (GICP): ~10-20회 (잘 조건화된 Gauss-Newton)
- LM (NDT): ~30-50+회 (불완전 헤시안 + 큰 λ)

---

## 5. PCL 구현과의 비교

PCL(Point Cloud Library)의 `NormalDistributionsTransform` 클래스는 NDT를 **Newton's method**로 직접 최적화한다.

### 5.1 PCL NDT의 최적화 흐름

```
1. score_gradient 계산: ∂E/∂p (해석적 1차 미분)
2. score_hessian 계산: ∂²E/∂p² (해석적 완전 2차 미분)
3. Newton step: Δp = -H⁻¹ g
4. Line search로 step size 결정
5. 수렴 판정
```

### 5.2 핵심 차이점

| 특성 | PCL NDT | gtsam_points NDT |
|------|---------|-------------------|
| 최적화 방법 | Newton's method | Levenberg-Marquardt |
| 헤시안 | 완전한 해석적 2차 미분 | 1차 항만 (Gauss-Newton 근사) |
| 외적 항 포함 | ✅ 포함 | ❌ 누락 |
| 2차 잔차 미분 | ✅ 포함 | ❌ 누락 |
| 수렴 차수 | 이차 (quadratic) | 선형~초선형 |
| Line search | ✅ (More-Thuente) | ❌ (LM trust region) |
| 프레임워크 제약 | 없음 (독립 구현) | `IntegratedMatchingCostFactor` LSQ 구조 |

### 5.3 gtsam_points가 Gauss-Newton 근사를 사용하는 이유

gtsam_points는 **범용 GTSAM 팩터 그래프 프레임워크** 위에 구축되어 있다. GTSAM의 최적화기(LM, ISAM2 등)는 모든 팩터가 `linearize()`를 통해 `GaussianFactor`를 반환하도록 요구한다. 이 인터페이스는 다음을 암묵적으로 가정한다:

1. 비용이 최소제곱 형태이거나, 최소제곱에 가까움
2. Gauss-Newton 근사가 유효함
3. `HessianFactor`의 H, b가 Gauss-Newton 구조를 따름

NDT를 이 프레임워크에 통합하기 위해서는 완전한 Newton 헤시안 대신 Gauss-Newton 유사 형태로 헤시안을 구성할 수밖에 없었다. 이것은 **프레임워크의 구조적 제약**이지 구현상의 실수가 아니다.

---

## 6. 수학적 상세 분석

### 6.1 NDT 비용 함수 유도

NDT는 타겟 포인트 클라우드를 복셀 단위의 정규분포 집합으로 표현한다. 각 복셀 $k$는 평균 $\boldsymbol{\mu}_k$와 공분산 $\boldsymbol{\Sigma}_k$를 가진 가우시안이다.

소스 포인트 $\mathbf{a}_i$를 변환 $T$로 변환한 $\mathbf{q}_i = T \mathbf{a}_i$가 대응 복셀 $k(i)$에 속할 확률:

$$p(\mathbf{q}_i) \propto \exp\!\left( -\frac{1}{2} (\mathbf{q}_i - \boldsymbol{\mu}_{k(i)})^T \boldsymbol{\Sigma}_{k(i)}^{-1} (\mathbf{q}_i - \boldsymbol{\mu}_{k(i)}) \right)$$

Magnusson (2009)의 score function (Eq. 6.9):

$$\boxed{ s(\mathbf{p}) = -\sum_{i} d_1 \exp\!\left( -\frac{d_2}{2} (\mathbf{q}_i - \boldsymbol{\mu}_{k(i)})^T \boldsymbol{\Sigma}_{k(i)}^{-1} (\mathbf{q}_i - \boldsymbol{\mu}_{k(i)}) \right) }$$

파라미터 $d_1$, $d_2$는 균일 분포(outlier)와 가우시안 분포(inlier)의 혼합 모델에서 유도된다 (Eq. 6.10):

$$d_1 = -\log(c_1 + c_2) - d_3, \quad d_2 = -2\log\!\left(\frac{-\log(c_1 e^{-1/2} + c_2) - d_3}{d_1}\right)$$

여기서 $c_1 = 10(1 - p_{\text{outlier}})$, $c_2 = \frac{p_{\text{outlier}}}{V}$ ($V$는 복셀 부피), $d_3 = -\log(c_2)$.

### 6.2 gtsam_points의 비음수 변환

gtsam_points에서는 GTSAM의 최적화기가 **비용 최소화**를 수행하므로, 원본 score를 비음수 비용으로 변환한다:

$$E_i^{\text{code}} = -d_1 \cdot (1 - e^{s_i})$$

여기서 $s_i = -\frac{d_2}{2} \mathbf{r}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i \leq 0$.

- $d_1 < 0$이므로 $-d_1 > 0$
- $0 \leq 1 - e^{s_i} \leq 1$ (∵ $s_i \leq 0$이면 $e^{s_i} \leq 1$)
- 따라서 $E_i^{\text{code}} \geq 0$

**완벽 정합** ($\mathbf{r}_i = 0$): $s_i = 0$, $e^{s_i} = 1$, $E_i = 0$ (최솟값)
**불량 정합** ($\|\mathbf{r}_i\| \to \infty$): $s_i \to -\infty$, $e^{s_i} \to 0$, $E_i \to -d_1 = |d_1|$ (최댓값, 유한)

이 변환은 그래디언트와 헤시안을 변경하지 않는다 (상수항 $-d_1$의 미분은 0).

### 6.3 1차 미분 (Gradient) 유도

$E_i = -d_1(1 - e^{s_i})$의 파라미터 $\mathbf{p}$에 대한 미분:

$$\frac{\partial E_i}{\partial \mathbf{p}} = -d_1 \cdot (-e^{s_i}) \cdot \frac{\partial s_i}{\partial \mathbf{p}} = d_1 \cdot e^{s_i} \cdot \frac{\partial s_i}{\partial \mathbf{p}}$$

$s_i = -\frac{d_2}{2} \mathbf{r}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i$의 미분:

$$\frac{\partial s_i}{\partial \mathbf{p}} = -d_2 \cdot \mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i$$

여기서 $\mathbf{J}_i = \frac{\partial \mathbf{r}_i}{\partial \mathbf{p}}$ (4×6 야코비안).

따라서:

$$\frac{\partial E_i}{\partial \mathbf{p}} = -d_1 \cdot d_2 \cdot e^{s_i} \cdot \mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i$$

코드에서 이것은:
```cpp
// b += derivative_scale * J^T * inv_cov * residual
*b_target += J_target_weighted * residual;
// 여기서 J_target_weighted = (-d1 * d2 * e_term) * J^T * inv_cov
```
→ **그래디언트는 정확하게 구현되어 있다.**

### 6.4 2차 미분 (Hessian) 유도 — 완전한 형태

그래디언트를 한 번 더 미분한다:

$$\frac{\partial^2 E_i}{\partial \mathbf{p}^2} = \frac{\partial}{\partial \mathbf{p}} \left[ d_1 e^{s_i} \cdot \frac{\partial s_i}{\partial \mathbf{p}} \right]$$

곱의 미분법:

$$= d_1 e^{s_i} \left[ \frac{\partial s_i}{\partial \mathbf{p}} \cdot \frac{\partial s_i}{\partial \mathbf{p}}^T + \frac{\partial^2 s_i}{\partial \mathbf{p}^2} \right]$$

#### 항 A: 외적 항

$$\text{A} = d_1 e^{s_i} \cdot \frac{\partial s_i}{\partial \mathbf{p}} \cdot \frac{\partial s_i}{\partial \mathbf{p}}^T = d_1 d_2^2 e^{s_i} \cdot (\mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i)(\mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i)^T$$

이것은 6×1 벡터의 외적으로, **rank-1 행렬**이다.

#### 항 B: $s_i$의 2차 미분

$$\frac{\partial^2 s_i}{\partial \mathbf{p}^2} = -d_2 \left( \mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{J}_i + \sum_k (\boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i)_k \frac{\partial^2 r_{ik}}{\partial \mathbf{p}^2} \right)$$

따라서:

$$\text{B} = -d_1 d_2 e^{s_i} \cdot \mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{J}_i - d_1 d_2 e^{s_i} \cdot \sum_k (\boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i)_k \frac{\partial^2 r_{ik}}{\partial \mathbf{p}^2}$$

#### 완전한 헤시안

$$\mathbf{H}_i^{\text{full}} = \underbrace{-d_1 d_2 e^{s_i} \cdot \mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{J}_i}_{\text{gtsam\_points가 계산하는 항}} + \underbrace{d_1 d_2^2 e^{s_i} \cdot (\mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i)(\cdots)^T}_{\text{누락: 외적 항 (rank-1)}} - \underbrace{d_1 d_2 e^{s_i} \sum_k (\boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i)_k \frac{\partial^2 r_{ik}}{\partial \mathbf{p}^2}}_{\text{누락: 2차 잔차 미분 항}}$$

### 6.5 누락된 항의 크기 추정

외적 항의 크기를 첫째 항 대비 추정한다.

$\mathbf{v}_i = \mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i$ (6×1 벡터)로 정의하면:

- 첫째 항의 스펙트럴 노름: $\| -d_1 d_2 e^{s_i} \mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{J}_i \| \sim |d_1| d_2 e^{s_i} \cdot \sigma_{\max}(\mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{J}_i)$
- 외적 항의 스펙트럴 노름: $\| d_1 d_2^2 e^{s_i} \mathbf{v}_i \mathbf{v}_i^T \| = |d_1| d_2^2 e^{s_i} \|\mathbf{v}_i\|^2$

비율:

$$\frac{\text{외적 항}}{\text{첫째 항}} \sim \frac{d_2 \|\mathbf{v}_i\|^2}{\sigma_{\max}(\mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{J}_i)} = \frac{d_2 \cdot \mathbf{r}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{J}_i \mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i}{\sigma_{\max}(\mathbf{J}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{J}_i)}$$

이 비율은 대략 $d_2 \cdot \text{(Mahalanobis 거리)}$ 에 비례한다. Mahalanobis 거리가 1 이상(즉, 포인트가 복셀 분포의 1σ 이상 떨어져 있을 때)이면 외적 항이 첫째 항과 **같은 크기 이상**이 된다.

**결론**: 최적화 초기(잔차가 큰 상태)에서 헤시안 오차가 50% 이상이 될 수 있으며, 이는 LM의 스텝 방향을 심각하게 왜곡시킨다.

### 6.6 GICP에서 Gauss-Newton이 유효한 이유

GICP의 비용 함수:

$$E_{\text{GICP}} = \sum_i \mathbf{r}_i^T \mathbf{W}_i \mathbf{r}_i$$

완전한 헤시안:

$$\mathbf{H}_i^{\text{GICP}} = 2\mathbf{J}_i^T \mathbf{W}_i \mathbf{J}_i + 2\sum_k (\mathbf{W}_i \mathbf{r}_i)_k \frac{\partial^2 r_{ik}}{\partial \mathbf{p}^2}$$

Gauss-Newton 근사: $\mathbf{H}_i \approx 2\mathbf{J}_i^T \mathbf{W}_i \mathbf{J}_i$

누락된 항 $2\sum_k (\mathbf{W}_i \mathbf{r}_i)_k \frac{\partial^2 r_{ik}}{\partial \mathbf{p}^2}$의 크기는 **잔차에 선형으로 비례**한다. 수렴 근처에서 잔차가 작아지면 이 항은 자연스럽게 사라진다.

**NDT와의 핵심 차이**: NDT에서 누락된 외적 항은 **잔차의 제곱**에 비례하며, exponential wrapping 때문에 추가적인 $d_2$ 팩터가 붙는다. 이는 GICP의 2차 항보다 훨씬 크고, 수렴 과정 전반에 걸쳐 영향을 미친다.

---

## 7. 결론 및 개선 방향

### 7.1 문제 요약

NDT의 29,232ms 수행 시간은 세 가지 원인이 **복합적으로** 작용한 결과이다:

```
[근본 원인: 수학적 비호환성]
  NDT의 지수함수 비용 ≠ LSQ 프레임워크의 이차 비용 가정
  → 불완전한 헤시안 (2개 항 누락)
      → ill-conditioned 헤시안
          → LM damping 증가 (λ ↑)
              → gradient descent로 퇴화
                  → 많은 반복 필요

[증폭 원인: 높은 반복당 비용]
  매 반복마다:
  ├─ correspondence: 50,000점 × 7복셀 × (해시 + Mahalanobis) = 350,000 연산
  ├─ evaluate: 50,000점 × (exp() + derivative_scale + 행렬곱) 
  └─ compute_ndt_params: 불필요한 재계산

[결과]
  (많은 반복) × (높은 반복당 비용) = 29,232ms
```

### 7.2 가능한 개선 방향

#### 방향 1: 완전한 Newton 헤시안 구현

GTSAM의 `HessianFactor` 인터페이스를 그대로 활용하되, `evaluate()` 함수 내에서 누락된 외적 항과 2차 잔차 미분 항을 추가 계산하여 헤시안에 반영한다.

**장점**: 이차 수렴 복원, 반복 횟수 대폭 감소
**단점**: 포인트당 연산량 증가 (외적 계산 + 2차 미분), 구현 복잡도 상승

#### 방향 2: Line Search 기반 Newton 최적화기

PCL처럼 GTSAM의 LM 대신 직접 Newton + Line Search (More-Thuente 등) 최적화기를 구현한다.

**장점**: NDT 본래의 수렴 특성 활용, PCL 수준의 성능 기대
**단점**: GTSAM 팩터 그래프 프레임워크와의 통합이 어려움 (ISAM2 등과 호환 불가)

#### 방향 3: 탐색 비용 감소

- `DIRECT7` → `DIRECT1`로 변경 (정확도 소폭 하락 가능)
- `compute_ndt_params()`를 생성자에서 1회만 호출
- Correspondence update tolerance 설정으로 불필요한 재탐색 방지

**장점**: 구현 간단, 기존 프레임워크 유지
**단점**: 근본 원인(헤시안 부정확)을 해결하지 못함

#### 방향 4: 비용 함수 이차 근사

NDT 비용 함수를 Mahalanobis 거리 기준으로 이차 함수로 근사:

$$E_i \approx \alpha_i \cdot \mathbf{r}_i^T \boldsymbol{\Sigma}_i^{-1} \mathbf{r}_i$$

이렇게 하면 GICP와 동일한 구조가 되어 Gauss-Newton이 정확해지지만, NDT의 고유한 robust성(outlier에 대한 유한 비용)을 상실한다.

### 7.3 결론

gtsam_points에서의 NDT 구현은 기술적으로 정확하지만, **프레임워크의 LSQ 가정과 NDT의 비이차 비용 함수 사이의 근본적 불일치**로 인해 최적의 성능을 발휘하지 못한다. 이것은 구현 버그가 아닌 **설계 트레이드오프**이다: gtsam_points는 다양한 스캔 매칭 알고리즘을 통일된 인터페이스로 제공하는 범용성을 택했고, 그 대가로 NDT처럼 최소제곱 구조에 맞지 않는 알고리즘에서 성능 저하가 발생한다.

Newton's method가 NDT 비용 함수의 지수적 곡률(exponential curvature)을 완전한 2차 미분으로 정확하게 포착하는 반면, LM의 Gauss-Newton 근사는 이 곡률 정보의 상당 부분을 버린다. 이것이 29,232ms의 수행 시간으로 나타나는 근본 원인이다.

---

## 참고 문헌

- Magnusson, M. (2009). *"The Three-Dimensional Normal-Distributions Transform — an Efficient Representation for Registration, Surface Analysis, and Loop Detection."* PhD Thesis, Örebro University. (Eq. 6.9-6.10)
- Segal, A., Hähnel, D., & Thrun, S. (2009). *"Generalized-ICP."* RSS 2009.
- Koide, K., Yokozuka, M., Oishi, S., & Banno, A. (2021). *"Voxelized GICP for Fast and Accurate 3D Point Cloud Registration."* ICRA 2021.
- Biber, P. & Straßer, W. (2003). *"The Normal Distributions Transform: A New Approach to Laser Scan Matching."* IROS 2003.
- Nocedal, J. & Wright, S. (2006). *"Numerical Optimization."* Springer. (Chapter 10: Least-Squares Problems)
- GTSAM Documentation: https://gtsam.org/
- gtsam_points: https://github.com/koide3/gtsam_points
