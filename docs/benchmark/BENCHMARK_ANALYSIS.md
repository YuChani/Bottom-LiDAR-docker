# LiDAR 정합 알고리즘 벤치마크 심층 분석

## 목차
1. [실험 환경 및 벤치마크 구조](#1-실험-환경-및-벤치마크-구조)
2. [Point-to-Point ICP](#2-point-to-point-icp)
3. [Point-to-Plane ICP](#3-point-to-plane-icp)
4. [GICP (Generalized ICP)](#4-gicp-generalized-icp)
5. [VGICP (Voxelized GICP)](#5-vgicp-voxelized-gicp)
6. [LOAM (Lidar Odometry And Mapping)](#6-loam-lidar-odometry-and-mapping)
7. [NDT (Normal Distributions Transform)](#7-ndt-normal-distributions-transform)
8. [알고리즘 간 비교 분석](#8-알고리즘-간-비교-분석)
9. [결론](#9-결론)

---

## 1. 실험 환경 및 벤치마크 구조

### 1.1 데이터

- **입력**: 64-ring LiDAR로 취득한 PCD 파일 7개 (프레임당 약 50,000 포인트)
- **Voxel downsampling**: 0.5m 해상도로 다운샘플링 후 사용
- **전처리**: 각 알고리즘에 따라 KdTree 생성, 공분산 추정, 법선 추정, 복셀맵 생성 등 수행

### 1.2 팩터 그래프 (Factor Graph) 구조

본 벤치마크는 단일 프레임 쌍이 아닌 **GTSAM 기반 팩터 그래프 최적화**를 사용합니다.

```
노드: x0, x1, x2, ..., x6 (7개 프레임의 SE(3) 자세)
팩터:
  - Prior factor: x0에 GT 자세를 강하게 고정 (σ = 1e-6)
  - Binary factors: full_connection=true → C(7,2) = 21개 이진 팩터
    (x0↔x1, x0↔x2, ..., x5↔x6 — 모든 프레임 쌍)
```

**초기값**: Ground Truth에 `noise_scale = 0.1`의 가우시안 노이즈를 SE(3) Lie 대수 공간에서 추가:
```cpp
initial_values.insert(i, gt_poses[i] * Pose3::Expmap(Vector6::Random() * noise_scale));
```
이는 6차원 벡터 `[ω₁, ω₂, ω₃, v₁, v₂, v₃]`에 각각 최대 ±0.1의 랜덤 값을 부여하며, 이를 Exponential Map으로 SE(3)에 매핑합니다.

### 1.3 최적화기: Levenberg-Marquardt (LM)

```
maxIterations = 100
relativeErrorTol = 1e-5
absoluteErrorTol = 1e-5
```

LM 알고리즘은 각 반복(iteration)마다:
1. 모든 팩터에서 잔차(residual)와 야코비안(Jacobian)을 계산
2. 정규방정식 `(J^T J + λI) Δx = -J^T r`을 풀어 업데이트 `Δx`를 구함
3. 비용 감소 여부에 따라 damping factor `λ`를 조절

각 팩터의 `linearize()` 호출 시, correspondence 재탐색 여부는 `correspondence_update_tolerance`에 의해 결정됩니다.

### 1.4 최종 벤치마크 결과

| 알고리즘 | 평균 병진 오차 (m) | 평균 회전 오차 (°) | 최대 병진 오차 (m) | 최대 회전 오차 (°) | 총 소요시간 (ms) |
|---------|:-----------------:|:-----------------:|:-----------------:|:-----------------:|:---------------:|
| Point-to-Point ICP | 0.095 | 0.488 | 0.219 | 0.908 | 9,253 |
| Point-to-Plane ICP | 0.062 | 0.449 | 0.126 | 0.930 | 8,130 |
| GICP | 0.084 | 0.551 | 0.165 | 1.103 | 11,480 |
| VGICP | 0.216 | 1.038 | 1.081 | 3.465 | 9,589 |
| LOAM | 0.289 | 1.048 | 0.873 | 2.328 | 65 |
| NDT | 0.078 | 0.510 | 0.143 | 1.129 | 29,232 |

이하 각 알고리즘에 대해 **수학적 정의**, **코드 구현 상세**, 그리고 **결과가 그렇게 나온 이유**를 분석합니다.

---

## 2. Point-to-Point ICP

### 2.1 수학적 정의

**비용 함수**:

$$E(\mathbf{T}) = \sum_{i=1}^{N} \| \mathbf{p}_B^{(i)} - \mathbf{T} \cdot \mathbf{p}_A^{(i)} \|^2$$

여기서:
- $\mathbf{T} \in SE(3)$: 소스(A)에서 타겟(B)으로의 변환
- $\mathbf{p}_A^{(i)}$: 소스 포인트 클라우드의 i번째 점
- $\mathbf{p}_B^{(i)}$: KdTree 1-NN 탐색으로 찾은 타겟 포인트 클라우드의 대응점
- $N$: 유효 대응쌍 수 (약 50,000개/팩터)

잔차 벡터:

$$\mathbf{r}_i = \mathbf{p}_B^{(i)} - \mathbf{T} \cdot \mathbf{p}_A^{(i)} \in \mathbb{R}^3$$

### 2.2 야코비안 (Jacobian)

GTSAM의 SE(3) 표현에서 야코비안은 6차원 Lie 대수 `[ω, v]` (회전 3 + 병진 3)에 대해 계산됩니다.

**타겟 자세(`target_pose`) 에 대한 야코비안**:
```cpp
J_target.block<3, 3>(0, 0) = -skew(Tp_A);  // 회전 성분: -[T·p_A]×
J_target.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();  // 병진 성분: I
```

**소스 자세(`source_pose`) 에 대한 야코비안**:
```cpp
J_source.block<3, 3>(0, 0) = R * skew(p_A);   // 회전 성분: R·[p_A]×
J_source.block<3, 3>(0, 3) = -R;               // 병진 성분: -R
```

여기서 `skew(v)`는 3차원 벡터 `v`의 반대칭 행렬(skew-symmetric matrix):

$$[\mathbf{v}]_\times = \begin{bmatrix} 0 & -v_3 & v_2 \\ v_3 & 0 & -v_1 \\ -v_2 & v_1 & 0 \end{bmatrix}$$

이 야코비안은 SE(3) 위 좌측 미분(left perturbation)에 기반합니다:

$$\frac{\partial (\mathbf{T} \cdot \mathbf{p})}{\partial \boldsymbol{\xi}} = \begin{bmatrix} -[\mathbf{T}\mathbf{p}]_\times & \mathbf{I} \end{bmatrix}$$

### 2.3 대응점 탐색 (Correspondence Search)

```cpp
// integrated_icp_factor_impl.hpp
size_t index = target_tree->knn_search(Tp_A.data(), 1, &k_index, &k_sq_dist);
if (k_sq_dist > max_correspondence_dist_sq) {
    correspondences[i] = -1;  // 거리 임계값 초과 → 아웃라이어
    continue;
}
correspondences[i] = k_index;
```

- KdTree 기반 1-NN 탐색
- `max_correspondence_dist_sq = 1.0` (1m²) — 기본 임계값
- `correspondence_update_tolerance = (0.0, 0.0)` — 매 LM 반복마다 재탐색

### 2.4 Hessian 구성 (Gauss-Newton 근사)

각 대응쌍의 기여:

$$\mathbf{H}_i = \mathbf{J}_i^T \mathbf{J}_i, \quad \mathbf{b}_i = -\mathbf{J}_i^T \mathbf{r}_i$$

최종 팩터의 Hessian은 이들의 합:

$$\mathbf{H} = \sum_{i=1}^{N} \mathbf{J}_i^T \mathbf{J}_i \in \mathbb{R}^{12 \times 12}$$

12×12 행렬인 이유: 이진 팩터로서 두 SE(3) 자세 (각 6 DoF)에 대한 결합 Hessian.

코드에서는 효율적인 상삼각(upper-triangular) 누적을 사용:
```cpp
sum_H_target_target += J_target.transpose() * J_target;
sum_H_target_source += J_target.transpose() * J_source;
sum_H_source_source += J_source.transpose() * J_source;
sum_b_target -= J_target.transpose() * residual;
sum_b_source -= J_source.transpose() * residual;
sum_e += 0.5 * residual.squaredNorm();
```

### 2.5 결과 분석

**병진 오차 0.095m, 회전 오차 0.488° — 중간 수준의 정확도**

**이유**:

1. **비용 함수의 기하학적 한계**: Point-to-Point 비용은 대응점 간 유클리드 거리만 최소화합니다. 평면(plane)이나 엣지(edge) 같은 기하학적 구조를 활용하지 않으므로, 평면 위에서의 슬라이딩(sliding) 방향에 대한 구속력이 약합니다.

2. **등방성(Isotropic) 오차 모델**: 모든 방향에 동일한 가중치를 부여합니다. LiDAR 포인트의 오차는 실제로 비등방성(anisotropic)이며, 빔 방향(range)으로의 오차가 접선(tangential) 방향보다 큽니다. 이를 무시하면 최적해가 왜곡됩니다.

3. **대응점 수 (~50,000)**: 풍부한 대응점으로 인해 통계적 평균 효과가 작용하여, 개별 오류 대응점의 영향이 희석됩니다. 이것이 알고리즘의 단순함에도 불구하고 합리적 수준의 결과를 내는 주된 이유입니다.

4. **처리 시간 9,253ms**: 21개 팩터 × 50,000 대응점/팩터 × KdTree 1-NN 탐색 비용. Point-to-Plane보다 느린 이유는 **수렴이 느려 반복 횟수가 더 많기 때문**입니다.

---

## 3. Point-to-Plane ICP

### 3.1 수학적 정의

**비용 함수**:

$$E(\mathbf{T}) = \sum_{i=1}^{N} \left( \mathbf{n}_B^{(i)} \cdot \left( \mathbf{p}_B^{(i)} - \mathbf{T} \cdot \mathbf{p}_A^{(i)} \right) \right)^2$$

여기서 $\mathbf{n}_B^{(i)}$는 타겟 포인트의 법선 벡터입니다.

이 비용은 점-평면 거리의 제곱합을 최소화하며, 기하학적으로 소스 점이 타겟의 **접평면(tangent plane)**에 놓이도록 최적화합니다.

잔차:

$$r_i = \mathbf{n}_B^{(i)} \cdot \left( \mathbf{p}_B^{(i)} - \mathbf{T} \cdot \mathbf{p}_A^{(i)} \right) \in \mathbb{R}^1$$

이것이 스칼라가 아닌 벡터로 구현되는 이유는 코드에서 element-wise 곱셈을 사용하기 때문입니다:

### 3.2 코드 구현 — 법선을 이용한 가중치 적용

```cpp
// integrated_icp_factor_impl.hpp (use_point_to_plane == true)
Eigen::Vector3d residual = pt_B - Tp_A;
Eigen::Vector3d normal = target_normals[correspondences[i]];

// Element-wise 곱셈으로 점-평면 잔차 생성
residual = normal.array() * residual.array();

// 야코비안도 동일하게 법선으로 스케일링
Eigen::Matrix<double, 3, 6> J_target, J_source;
// ... (Point-to-Point과 동일하게 계산 후)
J_target = normal.asDiagonal() * J_target;
J_source = normal.asDiagonal() * J_source;
```

**핵심 구현 디테일**: 잔차를 `n^T · (p_B - Tp_A)` 스칼라로 만드는 대신, `n ⊙ (p_B - Tp_A)` (element-wise 곱)로 3차원 벡터를 유지합니다. 이는 수학적으로 동등하지만, 스칼라 대신 3차원 벡터를 사용하면 Gauss-Newton Hessian에서 법선 방향뿐 아니라 **법선의 각 성분이 기여하는 정보**를 더 세밀하게 반영합니다.

수학적 등가성 증명:

$$\| \mathbf{n} \odot \mathbf{r} \|^2 = \sum_j (n_j r_j)^2 = \mathbf{r}^T \text{diag}(\mathbf{n})^2 \mathbf{r}$$

이는 법선의 각 성분을 가중치로 사용하는 **가중 최소자승법**과 동일합니다.

### 3.3 법선 벡터 추정

```cpp
// normal_estimation.hpp → covariance_estimation.hpp
// k_neighbors = 10, regularization = EIG
// 공분산 행렬의 고유값을 (1e-3, 1.0, 1.0)으로 정규화
```

법선은 포인트의 k=10 이웃으로부터 공분산 행렬을 계산하고, 최소 고유값에 해당하는 고유벡터를 법선으로 사용합니다. EIG 정규화는 최소 고유값을 `1e-3 × λ_max`로 설정하여 **평면적 구조(planar structure)**를 강화합니다.

### 3.4 결과 분석

**병진 오차 0.062m, 회전 오차 0.449° — 전체 알고리즘 중 최고 수준의 병진 정확도**

**Point-to-Point 대비 개선 이유**:

1. **접선 방향 자유도 허용**: Point-to-Plane 비용은 법선 방향의 오차만 벌칙합니다. 접선 방향으로의 슬라이딩은 비용에 기여하지 않으므로, 평면 구조에서의 대응점 오류가 최적화를 방해하지 않습니다.

2. **이차 수렴(Quadratic convergence)에 가까운 특성**: Point-to-Plane ICP는 Gauss-Newton 기반에서 Point-to-Point보다 더 넓은 수렴 영역(basin of convergence)을 가지며, 더 적은 반복으로 수렴합니다. 이론적으로, 평면 환경에서 Point-to-Plane은 한 반복(one step) 만에 정확한 해를 찾을 수 있습니다.

3. **더 빠른 수렴 → 더 적은 반복 → 더 짧은 시간**: 8,130ms vs 9,253ms. 반복 횟수 감소가 시간 단축의 주된 원인입니다. 개별 반복의 비용은 법선 조회 오버헤드로 약간 더 크지만, 총 반복 수가 줄어 전체 시간이 단축됩니다.

4. **병진 오차 0.062m vs 0.095m (35% 개선)**: 법선 방향으로의 구속이 더 정확한 병진 추정으로 이어집니다. 특히 벽면, 바닥 등 큰 평면 구조가 있는 LiDAR 데이터에서 효과적입니다.

5. **회전 오차는 유사 (0.449° vs 0.488°)**: 법선 기반 비용은 병진에 더 직접적 영향을 미치며, 회전 추정은 점들의 공간적 분포에 더 의존합니다. 두 알고리즘 모두 동일한 대응점 분포를 사용하므로 회전 정확도 차이는 상대적으로 작습니다.

---

## 4. GICP (Generalized ICP)

### 4.1 수학적 정의

GICP는 각 포인트의 **국소 공분산 행렬(local covariance matrix)**을 활용하여 비등방성(anisotropic) 오차를 모델링합니다.

**비용 함수** (Segal et al., RSS 2005):

$$E(\mathbf{T}) = \sum_{i=1}^{N} \hat{\mathbf{d}}_i^T \left( \mathbf{C}_B^{(i)} + \mathbf{R} \, \mathbf{C}_A^{(i)} \, \mathbf{R}^T \right)^{-1} \hat{\mathbf{d}}_i$$

여기서:
- $\hat{\mathbf{d}}_i = \mathbf{p}_B^{(i)} - \mathbf{T} \cdot \mathbf{p}_A^{(i)} \in \mathbb{R}^3$: 대응점 간 잔차
- $\mathbf{C}_A^{(i)}, \mathbf{C}_B^{(i)} \in \mathbb{R}^{3 \times 3}$: 각 포인트의 국소 공분산 행렬
- $\mathbf{R} \in SO(3)$: 변환 $\mathbf{T}$의 회전 성분
- $\mathbf{T} \in SE(3)$: 소스에서 타겟으로의 강체 변환

원본 논문에서는 결합 공분산의 회전 변환에 **회전 행렬 $\mathbf{R}$만** 사용합니다. 이는 공분산이 원점이 아닌 각 포인트의 국소 좌표계에서 정의되므로, 회전만으로 방향을 정렬하면 충분하기 때문입니다.

**마할라노비스 행렬(Mahalanobis matrix)**:

$$\mathbf{W}_i = \left( \mathbf{C}_B^{(i)} + \mathbf{R} \, \mathbf{C}_A^{(i)} \, \mathbf{R}^T \right)^{-1}$$

> **참고 — 본 코드의 구현 차이**: `integrated_gicp_factor_impl.hpp`에서는 4×4 동차 좌표를 사용하여 `delta.matrix() * cov_A * delta.matrix().transpose()` (전체 $\mathbf{T}$)로 계산합니다. 다만, 공분산의 유효 성분은 3×3 상좌 블록(top-left)이고 4번째 행/열은 0이므로, 병진 성분은 결과에 영향을 미치지 않아 수학적으로 $\mathbf{R} \mathbf{C}_A \mathbf{R}^T$와 동치입니다.

이 가중치는 포인트의 기하학적 형상에 따라 자동으로 조절됩니다:
- **평면 위 점**: 법선 방향 고유값 작음(1e-3) → 법선 방향 가중치 큼 → Point-to-Plane과 유사한 효과
- **엣지 위 점**: 한 방향 고유값만 큼 → 엣지 방향으로는 구속 약함
- **고립된 점**: 모든 고유값 유사 → 등방성 가중치 → Point-to-Point과 유사

### 4.2 코드 구현 — 마할라노비스 행렬 캐싱

```cpp
// integrated_gicp_factor_impl.hpp
// GICP 모드 FULL: 4×4 마할라노비스 행렬을 각 대응점마다 사전 계산
Eigen::Matrix4d RCR = Eigen::Matrix4d::Identity();
RCR.block<3, 3>(0, 0) = R * source_covs[i] * R.transpose();

Eigen::Matrix4d target_cov = Eigen::Matrix4d::Identity();
target_cov.block<3, 3>(0, 0) = target_covs[correspondences[i]];

// 결합 공분산의 역행렬 = 마할라노비스 행렬
mahalanobis[i] = (target_cov + RCR).inverse();
```

**핵심**: 마할라노비스 행렬은 correspondence가 업데이트될 때만 재계산됩니다. Linearization point가 변하지 않으면 캐시된 값을 재사용합니다.

### 4.3 가중 야코비안

```cpp
// 잔차 벡터 (4차원, 동차 좌표)
Eigen::Vector4d residual;
residual.head<3>() = pt_B - Tp_A;
residual.w() = 0.0;

// 가중 야코비안: J^T * Mahalanobis
Eigen::Matrix<double, 4, 6> J_target, J_source;
// ... (SE(3) 야코비안 계산)

auto J_target_mahalanobis = J_target.transpose() * mahalanobis[i];
auto J_source_mahalanobis = J_source.transpose() * mahalanobis[i];

sum_H_target_target += J_target_mahalanobis * J_target;
sum_H_target_source += J_target_mahalanobis * J_source;
sum_H_source_source += J_source_mahalanobis * J_source;
sum_b_target -= J_target_mahalanobis * residual;
sum_b_source -= J_source_mahalanobis * residual;
sum_e += 0.5 * residual.transpose() * mahalanobis[i] * residual;
```

Hessian 블록:

$$\mathbf{H}_{AB} = \sum_i \mathbf{J}_A^{(i)T} \mathbf{W}_i \mathbf{J}_B^{(i)}$$

### 4.4 공분산 추정 상세

```cpp
// covariance_estimation.hpp
// k_neighbors = 10
// regularization_method = EIG
// eigenvalues = [1e-3, 1.0, 1.0]  (최소, 중간, 최대)
```

각 포인트 $\mathbf{p}_i$에 대해:
1. 10개 최근접 이웃 $\{\mathbf{q}_j\}$를 찾음
2. 공분산 행렬 계산: $\mathbf{C} = \frac{1}{k}\sum_j (\mathbf{q}_j - \bar{\mathbf{q}})(\mathbf{q}_j - \bar{\mathbf{q}})^T$
3. 고유값 분해: $\mathbf{C} = \mathbf{V} \text{diag}(\lambda_1, \lambda_2, \lambda_3) \mathbf{V}^T$
4. EIG 정규화: 최소 고유값을 $10^{-3} \times \lambda_{\max}$로 대체

이 정규화로 인해 공분산은 항상 **평면적 타원체(oblate spheroid)** 형태를 가지며, 법선 방향으로의 불확실성이 작습니다.

### 4.5 결과 분석

**병진 오차 0.084m, 회전 오차 0.551° — Point-to-Plane보다 약간 나쁨**

**예상과 다른 결과의 원인**:

1. **이론 vs 실제의 괴리**: GICP는 이론적으로 Point-to-Plane의 상위 모델입니다(각 포인트의 공분산을 독립적으로 반영). 그러나 실제 성능은 **공분산 추정의 품질**에 크게 의존합니다.

2. **EIG 정규화의 영향**: 고유값을 `[1e-3, 1.0, 1.0]`으로 강제 설정하면 **모든 포인트가 동일한 비등방성 비율**을 갖습니다. 이는 엣지 포인트(2개의 큰 고유값을 가져야 함)의 공분산을 왜곡합니다.

3. **4×4 동차 좌표 사용 오버헤드**: GICP는 `Eigen::Matrix4d`를 사용하여 4번째 차원(`w=0`)을 포함합니다. 이는 계산 효율성에 불필요한 오버헤드를 추가하며, 마할라노비스 행렬의 `4×4 → 6×6` 변환 과정에서 수치적 민감성이 증가할 수 있습니다.

4. **마할라노비스 행렬 캐싱의 한계**: `correspondence_update_tolerance = (0.0, 0.0)`이지만, 마할라노비스 행렬은 linearization point에서의 `R`에 의존합니다. LM의 damping으로 인해 linearization point가 미세하게 변하면서, 캐시된 마할라노비스가 현재 `R`과 불일치하는 순간이 발생합니다.

5. **처리 시간 11,480ms — 가장 느린 밀집 정합 알고리즘**: 매 대응점마다 `4×4 역행렬 계산 + 가중 Hessian 누적`이 필요합니다. 50,000개 대응점 × 21개 팩터 × 역행렬 계산 = 상당한 연산량입니다.

---

## 5. VGICP (Voxelized GICP)

### 5.1 수학적 정의

VGICP는 GICP와 동일한 마할라노비스 비용 함수를 사용하되, 타겟의 포인트별 공분산 대신 **복셀(voxel) 기반 가우시안 분포**를 사용합니다.

**비용 함수** (Koide et al., "Voxelized GICP for Fast and Accurate 3D Point Cloud Registration", ICRA 2021):

$$E(\mathbf{T}) = \sum_{i=1}^{N} \mathbf{d}_i^T \left( \mathbf{B}_k + \mathbf{T} \, \mathbf{A}_i \, \mathbf{T}^T \right)^{-1} \mathbf{d}_i$$

여기서:
- $\mathbf{d}_i = \boldsymbol{\mu}_{B_k} - \mathbf{T} \, \boldsymbol{\mu}_{A_i}$ (소스 점의 변환 결과와 대응 복셀 평균 간 차이)
- $\mathbf{T} \in SE(3)$: 4×4 동차 변환 행렬 (회전 $\mathbf{R}$ + 병진 $\mathbf{t}$)
- $\mathbf{B}_k$: 대응 복셀 $k$의 공분산 (복셀 내 포인트들로부터 계산)
- $\boldsymbol{\mu}_{B_k}$: 대응 복셀 $k$의 평균
- $\mathbf{A}_i$: 소스 포인트 $i$의 국소 공분산
- $\boldsymbol{\mu}_{A_i}$: 소스 포인트 $i$의 위치

> **논문 vs 코드 대응 관계**: 코드에서 `delta.matrix() * frame::cov(*source, i) * delta.matrix().transpose()`는 $\mathbf{T} \, \mathbf{A}_i \, \mathbf{T}^T$에 정확히 대응합니다. 여기서 `delta.matrix()`는 4×4 동차 변환 행렬 $\mathbf{T}$입니다. 4×4 공분산의 4번째 행/열은 0이므로 $\mathbf{T} \mathbf{A}_i \mathbf{T}^T$의 상위 3×3 블록은 $\mathbf{R} \mathbf{A}_i^{3 \times 3} \mathbf{R}^T$와 수학적으로 동치이지만, 논문의 표기법은 변환 전체($\mathbf{T}$)를 명시적으로 사용합니다.
>
> **GICP와의 차이점**: GICP(Segal et al. 2005)가 포인트별 $\mathbf{C}_B^i$를 사용하는 반면, VGICP는 복셀 수준의 $\mathbf{B}_k$를 사용합니다. 하나의 복셀이 다수의 소스 포인트와 대응될 수 있으므로, 대응점 탐색의 계산 복잡도가 $O(N \log N) \rightarrow O(N)$으로 줄어듭니다.

### 5.2 복셀 기반 대응점 탐색 — VGICP의 핵심 차이

```cpp
// integrated_vgicp_factor_impl.hpp
Eigen::Vector3d Tp_A = T * pt_A;
Eigen::Vector3i voxel_coord = target_voxelmap->voxel_coord(Tp_A);
int voxel_id = target_voxelmap->lookup_voxel_index(voxel_coord);

if (voxel_id < 0) {
    correspondences[i] = -1;  // 빈 복셀 → 아웃라이어
    continue;
}
correspondences[i] = voxel_id;
```

**KdTree 대신 해시맵 기반 단일 복셀 조회**:
1. 변환된 소스 점 $\mathbf{T} \cdot \mathbf{p}_s$의 3D 좌표를 복셀 해상도(0.5m)로 양자화
2. 해당 복셀 좌표로 해시맵에서 직접 조회 — **O(1) 연산**
3. 빈 복셀이면 대응점 없음(아웃라이어)

**이웃 복셀 탐색 없음**: NDT의 DIRECT7(현재 + 6개 면 이웃)과 달리, VGICP는 오직 **해당 복셀 하나만** 조회합니다.

### 5.3 복셀맵 구조

```cpp
// GaussianVoxelMapCPU (resolution = 0.5m)
// 각 복셀에 저장:
//   - mean (μ): 복셀 내 모든 포인트의 평균 위치
//   - cov (Σ): 복셀 내 모든 포인트의 공분산 행렬
//   - num_points: 포인트 수
```

복셀 해상도 0.5m은 다운샘플링 해상도와 동일합니다. 이는 복셀당 평균 1~3개 포인트만 포함됨을 의미하며, 복셀의 통계적 대표성이 제한적일 수 있습니다.

### 5.4 결과 분석

**병진 오차 0.216m, 회전 오차 1.038° — Frame 3에서의 극단적 오류가 평균을 끌어올림**

**Frame 3 국소 최솟값(Local Minimum) 상세 분석**:

- **최대 병진 오차 1.081m, 최대 회전 오차 3.465°** — 이 값은 Frame 3에서 발생

1. **단일 복셀 조회의 구조적 한계**: 초기 자세에 `noise_scale = 0.1` 노이즈가 인가됩니다. 이 노이즈가 포인트를 **인접 복셀로 이동**시키면, 올바른 대응 복셀을 찾지 못합니다. 복셀 해상도가 0.5m이므로, 병진 노이즈 ~0.1m만으로도 경계 근처 포인트의 약 20%가 잘못된 복셀에 매핑될 수 있습니다.

2. **잘못된 대응 → 잘못된 기울기 → 국소 최솟값**: 다수의 포인트가 잘못된 복셀과 매핑되면, 비용 함수의 기울기가 올바른 해 방향을 가리키지 않습니다. 최적화는 잘못된 기울기를 따라 이동하여 국소 최솟값에 빠집니다.

3. **Frame 3의 특이성**: 데이터의 공간적 분포에 따라 특정 프레임에서 복셀 경계 효과가 증폭됩니다. 균일하지 않은 포인트 밀도 + 노이즈 방향의 불운한 조합이 Frame 3에서 발생한 것입니다.

4. **대조: 다른 프레임은 정상 수렴**: Frame 3를 제외하면 VGICP의 성능은 ICP와 유사합니다. 이는 단일 복셀 조회가 **대부분의 경우 충분하지만**, 엣지 케이스에 취약함을 보여줍니다.

**GICP 대비 속도 비교 (9,589ms vs 11,480ms)**:
- 대응점 탐색: O(1) 해시맵 vs O(log n) KdTree → VGICP가 빠름
- 마할라노비스 계산: 복셀의 공분산(미리 계산됨) vs 포인트별 공분산 조회 → 유사
- 단, Frame 3의 수렴 실패로 인해 반복 횟수가 증가하여 시간이 일부 상쇄됨

---

## 6. LOAM (Lidar Odometry And Mapping)

### 6.1 수학적 정의

LOAM은 원시 포인트 대신 **기하학적 특징(feature)**을 사용합니다:
- **Edge 포인트**: 곡률(curvature)이 높은 점 (엣지 구조)
- **Planar 포인트**: 곡률이 낮은 점 (평면 구조)

**Edge 비용 함수** (점-직선 거리):

$$E_{\text{edge}} = \sum_{i} \frac{\| (\mathbf{p}_j - \mathbf{T}\mathbf{p}_i) \times (\mathbf{p}_l - \mathbf{T}\mathbf{p}_i) \|}{\| \mathbf{p}_j - \mathbf{p}_l \|}$$

여기서:
- $\mathbf{p}_i$: 소스의 edge 특징점
- $\mathbf{p}_j, \mathbf{p}_l$: 타겟의 2-NN edge 대응점 (직선을 정의)
- 분자: 두 벡터의 외적 크기 = 평행사변형 넓이 = 밑변 × 높이
- 분모: 밑변 길이 → 나누면 높이(=점-직선 거리)

**Planar 비용 함수** (점-평면 거리):

$$E_{\text{plane}} = \sum_{i} \frac{(\mathbf{p}_j - \mathbf{T}\mathbf{p}_i) \cdot \left[ (\mathbf{p}_j - \mathbf{p}_l) \times (\mathbf{p}_j - \mathbf{p}_m) \right]}{\| (\mathbf{p}_j - \mathbf{p}_l) \times (\mathbf{p}_j - \mathbf{p}_m) \|}$$

여기서:
- $\mathbf{p}_j, \mathbf{p}_l, \mathbf{p}_m$: 타겟의 3-NN planar 대응점 (평면을 정의)
- 외적 $(\mathbf{p}_j - \mathbf{p}_l) \times (\mathbf{p}_j - \mathbf{p}_m)$은 평면의 법선을 정의
- 정규화하면 단위 법선, 이를 점-평면 거리 계산에 사용

**결합 비용**:

$$E_{\text{LOAM}} = E_{\text{edge}} + E_{\text{plane}}$$

코드에서:
```cpp
error = edge_error + plane_error;  // 잔차 벡터 연결(concatenation)
// Hessian: H = H_edge + H_plane (블록 덧셈)
```

### 6.2 특징 추출 (Feature Extraction)

```cpp
// loam_feature.cpp — LIO-SAM 스타일 특징 추출
```

**과정**:
1. **링(Ring) 분류**: 64-ring LiDAR의 각 점을 수직 각도(vertical angle)로 링 번호 할당
2. **곡률(Curvature) 계산**: 각 점에 대해 좌우 5개 이웃과의 거리 차이로 곡률 추정

$$c_i = \frac{1}{2k} \left\| \sum_{j \in \mathcal{N}_i} (\mathbf{p}_j - \mathbf{p}_i) \right\|$$

3. **특징 선별**: 
   - 곡률 상위 → Edge 포인트 (링당 최대 20개)
   - 곡률 하위 → Planar 포인트 (링당 최대 40개)
4. **유효성 필터링**: 같은 방향(occluded), 평행한 표면(parallel beam) 등의 불안정 특징 제거

**특징 수**: 프레임당 약 768개 edge + 1,536개 planar = **총 약 2,304개 특징** (밀집 정합의 50,000개 대비 약 1/22)

### 6.3 대응점 검증 — 수정된 버그

```cpp
// integrated_loam_factor_impl.hpp
// validate_correspondences() — plane correspondence의 세 점 검증

// 수정 후 (올바른 코드):
const double theta_thresh = 5.0 / M_PI / 180.0;  // ≈ 0.0873 rad (5°)

// 세 점이 다른 스캔라인(ring)에서 왔는지 확인:
double vertical_angle = asin(dz / dist);
if (abs(v_angle_i - vertical_angle) < theta_thresh) {
    // 같은 스캔라인 → correspondence 무효화
}
```

이 검증은 plane correspondence의 세 점이 **서로 다른 스캔라인**에서 왔는지 확인합니다. 같은 스캔라인의 세 점은 유사한 수직 각도를 가지므로 안정적인 평면을 정의하지 못합니다.

**수정 전 버그**: `M_PI * 180.0 ≈ 565.49`로, 임계값이 모든 `asin()` 반환값보다 크므로 검증이 **항상 통과**했습니다.

### 6.4 Correspondence Update Tolerance

```cpp
// main.cpp — 수정 후
loam_factor->set_correspondence_update_tolerance(0.005, 0.02);
```

이 tolerance는:
- 회전 변화 < 0.005 rad (≈ 0.29°) **그리고** 병진 변화 < 0.02m 일 때 correspondence를 재사용
- 수렴 근처에서 correspondence가 안정화되어 **진동(oscillation)** 방지

**LOAM에서만 tolerance가 필요한 이유**:
- LOAM의 대응점은 2-NN, 3-NN 탐색으로 **직선/평면을 정의**합니다
- 작은 자세 변화가 NN 순서를 바꾸면, 전혀 다른 직선/평면이 정의됩니다
- 이는 비용 함수의 불연속적 점프를 야기 → 진동
- ICP/GICP/NDT의 1-NN은 순서가 바뀌어도 비용 변화가 연속적이므로 tolerance 불필요

### 6.5 야코비안 구조

**Edge 야코비안**: 외적의 미분을 체인 룰로 계산

```cpp
// J = d(||a × b|| / ||c||) / dξ
// a = p_j - Tp_i, b = p_l - Tp_i, c = p_j - p_l
Eigen::Vector3d cross_ab = a.cross(b);
double cross_norm = cross_ab.norm();
// 외적의 야코비안: d(a×b)/da = -[b]×, d(a×b)/db = [a]×
```

**Planar 야코비안**: 내적과 외적의 조합

```cpp
// J = d(a · n / ||n||) / dξ
// n = (p_j - p_l) × (p_j - p_m), a = p_j - Tp_i
// n은 상수(타겟에만 의존) → Jacobian은 a에 대한 것만 계산
```

### 6.6 결과 분석

**병진 오차 0.289m, 회전 오차 1.048° — 정확도 최하위, 속도 최상위 (65ms)**

**정확도가 낮은 이유**:

1. **특징점 수의 압도적 차이**: ~2,304개 특징 vs ~50,000개 밀집 포인트. 정보량이 약 1/22입니다. 팩터 그래프의 정규방정식에서 Hessian의 조건수(condition number)가 나빠지며, 해의 불확실성이 증가합니다.

2. **특징 기반 대응의 불안정성**: Edge 2-NN, Plane 3-NN 탐색은 특징점의 공간적 분포에 민감합니다. 특징점이 듬성듬성 분포하면, NN 탐색이 기하학적으로 무관한 점을 반환할 수 있습니다.

3. **결합 비용의 비균형**: Edge와 Plane 잔차를 단순 합산하므로, 스케일이 다른 두 비용이 서로 간섭할 수 있습니다. Edge 잔차(점-직선 거리)와 Plane 잔차(점-평면 거리)의 단위는 동일(미터)하지만, 기울기의 방향이 다르므로 최적화 경로가 비효율적일 수 있습니다.

4. **적은 대응쌍으로 인한 과적합(Overfitting) 위험**: 21개 팩터 × ~2,304개 특징/팩터 ≈ 48,000개 관측. 12×6 = 72 DoF에 비해 충분하지만, 아웃라이어 비율이 높으면 효과적 관측 수가 급감합니다.

**속도가 빠른 이유 (65ms — 다음으로 빠른 P2PL ICP의 125배)**:

1. **특징점 수 자체가 적음**: linearize() 호출 시 50,000개 대신 2,304개만 처리
2. **빠른 수렴**: tolerance 설정 후 11회 반복으로 수렴 (vs 다른 알고리즘의 수십 회)
3. **단순한 Hessian 구조**: edge + plane 잔차의 단순 합산, 역행렬 계산 없음

---

## 7. NDT (Normal Distributions Transform)

### 7.1 수학적 정의

NDT는 타겟 포인트 클라우드를 **3D 정규 분포(가우시안)의 격자**로 표현하고, 소스 포인트가 이 분포에 속할 확률을 최대화합니다.

**스코어 함수** (Magnusson, "The Three-Dimensional Normal-Distributions Transform", PhD thesis, 2009, Eq. 6.9):

$$s(\mathbf{x}) = -d_1 \exp\left( -\frac{d_2}{2} \, \mathbf{x}^T \boldsymbol{\Sigma}_v^{-1} \mathbf{x} \right)$$

**최적화 비용 함수** (전체 포인트에 대한 스코어 합의 부호 반전):

$$E(\mathbf{T}) = -\sum_{i=1}^{N} s(\mathbf{x}_i) = \sum_{i=1}^{N} d_1 \exp\left( -\frac{d_2}{2} \, \mathbf{x}_i^T \boldsymbol{\Sigma}_v^{-1} \mathbf{x}_i \right)$$

여기서:
- $\mathbf{x}_i = \boldsymbol{\mu}_v - \mathbf{T} \cdot \mathbf{p}_s^{(i)}$ (변환된 소스 점과 복셀 평균 간 차이)
- $\boldsymbol{\Sigma}_v^{-1}$: 복셀의 역공분산 행렬
- $d_1 < 0$: 비용의 스케일링 팩터 — 가우시안과 균일 분포의 혼합 모델에서 유도되며, 아웃라이어 비율에 의존
- $d_2 > 0$: 지수 함수의 감쇠율 — 복셀 해상도와 관련

> **코드의 비음수(non-negative) 재정의**: 실제 구현에서는 수치 안정성을 위해 원본 스코어 함수를 다음과 같이 변환합니다:
>
> $$E_{\text{code}}(\mathbf{T}) = \sum_{i=1}^{N} -d_1 \left(1 - \exp\left( -\frac{d_2}{2} \, \mathbf{x}_i^T \boldsymbol{\Sigma}_v^{-1} \mathbf{x}_i \right)\right)$$
>
> $d_1 < 0$이므로 $-d_1 > 0$입니다. 이 변환은:
> - **완벽한 정렬** ($\mathbf{x} = 0$): $\exp(\cdot) = 1$ → 비용 $= 0$ (최소)
> - **나쁜 정렬**: $\exp(\cdot) \approx 0$ → 비용 $\approx |d_1|$ (포화)
>
> 원본 수식과 상수 차이($+d_1$ 오프셋)만 있으므로, **기울기(gradient)와 헤시안(Hessian)은 원본과 동일**합니다. 코드 주석에도 이 점이 명시되어 있습니다.

**NDT 파라미터 계산**:
```cpp
double compute_ndt_params(double resolution, double outlier_ratio) {
    double p_outlier = 1.0 / (resolution * resolution * resolution);  // 균일 분포 확률
    // d1, d2는 가우시안과 균일 분포의 혼합 모델에서 유도
    // outlier_ratio = 0.1 → 10%의 포인트가 아웃라이어로 가정
}
```

- $d_1$: 비용의 스케일링 팩터 — 아웃라이어 비율과 관련
- $d_2$: 지수 함수의 감쇠율 — 복셀 해상도와 관련
- 이 파라미터들은 **아웃라이어 강건성**을 제공합니다: 복셀 평균에서 멀리 떨어진 점의 비용 기여가 포화(saturate)됩니다

### 7.2 비용 함수의 기하학적 해석

NDT 스코어 함수 $s(\mathbf{x}) = -d_1 \exp(-\frac{d_2}{2} \mathbf{x}^T \boldsymbol{\Sigma}^{-1} \mathbf{x})$는 가우시안 혼합 모델의 음의 로그 우도(negative log-likelihood)에서 유도됩니다:

1. **가우시안 복셀에 가까운 점**: $\mathbf{x}^T \boldsymbol{\Sigma}_v^{-1} \mathbf{x} \approx 0$ → $s(\mathbf{x}) \approx -d_1$ (최대 스코어, 가장 낮은 비용)
2. **가우시안에서 먼 점**: $\exp(\cdot) \approx 0$ → $s(\mathbf{x}) \approx 0$ (스코어 포화)
3. **코드의 비음수 재정의에서**: 비용 $= 0$ (완벽 정렬) ~ $|d_1|$ (나쁜 정렬) 범위로 매핑

이 포화 특성은 M-estimator와 유사한 **강건 추정(robust estimation)** 효과를 제공합니다. 복셀 밖의 아웃라이어가 비용에 미치는 영향이 $|d_1|$으로 상한이 정해지므로, 극단적 아웃라이어가 최적화를 지배하지 못합니다.

### 7.3 DIRECT7 이웃 탐색

```cpp
// integrated_ndt_factor_impl.hpp
// NdtCorrespondence 탐색: 현재 복셀 + 6개 면 이웃 (총 7개)
for (int axis = 0; axis < 3; axis++) {
    for (int dir : {-1, +1}) {
        Eigen::Vector3i neighbor = voxel_coord;
        neighbor[axis] += dir;
        // 이웃 복셀 조회, 마할라노비스 거리 비교
    }
}
// 7개 중 마할라노비스 거리가 최소인 복셀을 대응점으로 선택
```

VGICP의 단일 복셀 조회와 달리, NDT는 **7개 후보** 중 최적을 선택합니다. 이는 복셀 경계 효과를 크게 완화합니다.

### 7.4 역공분산 정규화

```cpp
// compute_ndt_inverse_covariance()
// 1. 복셀 공분산의 고유값 분해
// 2. 최소 고유값을 ε * λ_max로 클램핑 (ε = 1e-3)
// 3. 정규화된 고유값으로 역공분산 재구성

eigenvalues = max(eigenvalues, epsilon * max_eigenvalue);
inv_cov = V * diag(1.0 / eigenvalues) * V^T;
```

이 정규화는:
- **특이(singular) 공분산 방지**: 평면 위의 포인트는 한 고유값이 0에 가까움 → 역행렬 불가
- 정규화 후 역공분산의 최대 고유값과 최소 고유값의 비율이 최대 `1/ε = 1000`으로 제한

### 7.5 기울기와 Hessian

**1차 미분**:

$$\frac{\partial E_i}{\partial \boldsymbol{\xi}} = d_1 d_2 \exp(s_i) \cdot \mathbf{x}_i^T \boldsymbol{\Sigma}_v^{-1} \frac{\partial \mathbf{x}_i}{\partial \boldsymbol{\xi}}$$

여기서 $s_i = -\frac{d_2}{2} \mathbf{x}_i^T \boldsymbol{\Sigma}_v^{-1} \mathbf{x}_i$

**2차 미분 (PSD Hessian)**:

```cpp
// derivative_scale = -d1 * d2 * exp(s)  (항상 양수)
// PSD (Positive Semi-Definite) Hessian 근사 사용:
H += derivative_scale * J^T * inv_cov * J;
// 2차 항 (-2 * d2 * (J^T * inv_cov * x)(J^T * inv_cov * x)^T)은 생략
// 이는 Gauss-Newton 스타일 근사로, Hessian의 양정치성을 보장
```

PSD Hessian을 사용하는 이유:
- Levenberg-Marquardt가 양정치 Hessian을 요구
- 2차 항을 포함하면 비양정치(indefinite)가 될 수 있어 최적화 불안정

### 7.6 결과 분석

**병진 오차 0.078m, 회전 오차 0.510° — Point-to-Plane에 근접하는 높은 정확도, 극히 느린 속도**

**높은 정확도의 이유**:

1. **DIRECT7 이웃 탐색**: 7개 복셀 중 최적을 선택함으로써 복셀 경계 문제를 효과적으로 완화합니다. VGICP의 Frame 3 같은 국소 최솟값이 발생하지 않습니다.

2. **강건 비용 함수**: 지수 함수에 의한 아웃라이어 포화가 잘못된 대응점의 영향을 자동으로 억제합니다. 이는 ICP 계열의 단순 제곱 비용보다 훨씬 강건합니다.

3. **역공분산의 비등방성 가중치**: GICP와 유사하게, 복셀의 기하학적 형상(평면, 엣지)에 따라 자동으로 가중치가 조절됩니다.

**극히 느린 속도 (29,232ms)의 이유**:

1. **V자형 비용 곡선**: NDT의 이산적 복셀 할당은 최적화 과정에서 비용 함수의 불연속적 변화를 야기합니다. 포인트가 다른 복셀로 이동하면 비용이 갑자기 증가(V자의 상승 구간)하고, 새 복셀에 적응하면서 다시 감소합니다.

2. **100회 최대 반복 소진**: V자형 비용 변화로 인해 LM의 수렴 조건(`relativeErrorTol = 1e-5`)이 만족되지 않고, 반복 횟수 상한에 도달합니다.

3. **반복당 높은 비용**: 매 반복마다:
   - 50,000개 소스 점 × 7개 복셀 조회 = 350,000회 해시맵 조회 + 마할라노비스 거리 계산
   - 각 대응점에서 `exp()` 계산 (연산 비용 높음)
   - 50,000개 × 21개 팩터 = 1,050,000개 대응점의 가중 Hessian 누적

4. **Tolerance 변경이 정확도를 악화시키는 이유**: 
   - NDT의 역공분산은 **최적화 시작 시 한 번만 계산**되어 전체 과정에서 고정됩니다
   - Tolerance를 설정하면 correspondence도 일부 고정되는데, 복셀 경계를 넘는 포인트의 대응이 갱신되지 않음
   - V자형 패턴의 "하강 → 재상승 → 재하강"은 실제로 더 나은 해를 탐색하는 과정이며, 이를 tolerance로 차단하면 **첫 번째 국소 최솟값에 갇힘**
   - 실험 결과: tolerance `(0.01, 0.05)` 적용 시 평균 회전 오차가 0.51° → 3.49°로 6.8배 악화

---

## 8. 알고리즘 간 비교 분석

### 8.1 정확도 순위와 원인

| 순위 | 알고리즘 | 평균 병진 오차 | 핵심 요인 |
|:---:|---------|:----------:|----------|
| 1 | Point-to-Plane ICP | 0.062m | 법선 활용 + 밀집 대응점 + 안정적 수렴 |
| 2 | NDT | 0.078m | 강건 비용함수 + DIRECT7 + 비등방 가중치 |
| 3 | GICP | 0.084m | 마할라노비스 가중치. 공분산 정규화 한계 |
| 4 | Point-to-Point ICP | 0.095m | 등방성 비용. 통계적 평균화로 보완 |
| 5 | VGICP | 0.216m | Frame 3 국소 최솟값이 평균 악화 |
| 6 | LOAM | 0.289m | 적은 특징점. 대응 불안정 |

### 8.2 속도 순위와 원인

| 순위 | 알고리즘 | 소요시간 | 핵심 요인 |
|:---:|---------|:------:|----------|
| 1 | LOAM | 65ms | 2,304개 특징 + 11회 반복 |
| 2 | Point-to-Plane ICP | 8,130ms | 빠른 수렴 (적은 반복) |
| 3 | Point-to-Point ICP | 9,253ms | 느린 수렴 (더 많은 반복) |
| 4 | VGICP | 9,589ms | O(1) 해시맵 탐색 but Frame 3 수렴 실패 |
| 5 | GICP | 11,480ms | 마할라노비스 역행렬 × 50k 포인트 |
| 6 | NDT | 29,232ms | 100회 반복 소진 + 7-복셀 탐색 + exp() |

### 8.3 대응점 탐색 전략 비교

| 알고리즘 | 탐색 방법 | 복잡도 | 강점 | 약점 |
|---------|----------|:-----:|------|------|
| P2P / P2PL ICP | KdTree 1-NN | O(log n) | 정확한 최근접점 | 트리 구축 비용 |
| GICP | KdTree 1-NN | O(log n) | 마할라노비스 가중 | 역행렬 연산 추가 |
| VGICP | 해시맵 단일 복셀 | O(1) | 매우 빠름 | 복셀 경계 취약 |
| LOAM | KdTree 2/3-NN | O(k log n) | 기하 구조 활용 | k-NN 순서 민감 |
| NDT | 해시맵 7-복셀 | O(7) ≈ O(1) | 경계 효과 완화 | 7× 연산량 |

### 8.4 비용 함수 특성 비교

| 알고리즘 | 비용 함수 형태 | 가중치 | 강건성 | 정보 활용 |
|---------|-------------|--------|:-----:|----------|
| P2P ICP | $\|p_B - Tp_A\|^2$ | 없음 (등방성) | 낮음 | 점 위치만 |
| P2PL ICP | $(n \cdot (p_B - Tp_A))^2$ | 법선 방향 | 중간 | 점 위치 + 법선 |
| GICP | $r^T M^{-1} r$ | 마할라노비스 | 중간 | 점 위치 + 국소 공분산 |
| VGICP | $r^T M^{-1} r$ | 마할라노비스 | 중간 | 점 위치 + 복셀 공분산 |
| LOAM | 점-직선 + 점-평면 | 없음 | 낮음 | 엣지 + 평면 기하 구조 |
| NDT | $-d_1(1-\exp(-\frac{d_2}{2}x^T\Sigma^{-1}x))$ | 역공분산 + 포화 | **높음** | 복셀 분포 + 아웃라이어 모델 |

### 8.5 팩터 그래프의 영향

21개 이진 팩터(full connection)의 효과:

1. **정보 전파**: x0에 강한 prior가 있으므로, x0↔x_i 팩터를 통해 정보가 모든 노드에 전파됩니다. 이는 단일 쌍 정합보다 훨씬 강건한 결과를 제공합니다.

2. **오차 분산**: 특정 프레임 쌍의 정합 실패가 다른 팩터에 의해 보정됩니다. VGICP의 Frame 3 오류도 full connection 없이는 더 심각했을 것입니다.

3. **최적화 비용 증가**: 21개 팩터 × 점당 비용이 총 연산량을 결정합니다. LOAM의 65ms가 가능한 이유는 팩터당 연산량이 작기 때문입니다.

### 8.6 Point-to-Plane이 GICP보다 나은 이유

이론적으로 GICP가 상위 모델임에도 불구하고 Point-to-Plane이 더 나은 결과를 보인 이유:

1. **모델 복잡도 vs 데이터**: GICP의 포인트별 4×4 마할라노비스 행렬은 추가적인 자유 파라미터(공분산)를 도입합니다. 이 공분산의 추정 오류가 최적화에 **잡음(noise)**으로 작용합니다.

2. **EIG 정규화의 획일성**: 모든 포인트의 공분산이 `[1e-3, 1.0, 1.0]` 비율로 강제되므로, GICP의 핵심 장점인 "포인트별 적응적 가중치"가 사실상 무력화됩니다. 이 경우 GICP는 **일률적인 비등방 ICP**로 퇴화하며, Point-to-Plane의 법선 가중치보다 유연하지 못합니다.

3. **수치적 안정성**: Point-to-Plane의 `diag(n)` 가중치는 조건수가 양호합니다 (법선은 단위 벡터). GICP의 마할라노비스 행렬은 공분산의 역행렬이므로 조건수가 최대 1000(= 1/ε)까지 높아질 수 있습니다.

---

## 9. 결론

### 9.1 핵심 발견

1. **Point-to-Plane ICP가 본 벤치마크에서 최적의 정확도/속도 균형**을 보여줍니다. 법선 정보를 활용하면서도 모델 복잡도가 적절하여, 공분산 추정 오류의 영향을 받지 않습니다.

2. **NDT는 가장 강건한 비용 함수**를 가지지만, 이산적 복셀 구조로 인한 수렴 문제(V자형 패턴)가 속도를 심각하게 저하시킵니다. 적응적 해상도나 다해상도(multi-resolution) NDT가 이를 완화할 수 있습니다.

3. **GICP의 이론적 우위가 실현되지 못한 핵심 원인은 공분산 정규화**입니다. 더 정교한 공분산 추정(예: 적응적 k-이웃, 표면 유형별 정규화)이 필요합니다.

4. **VGICP의 단일 복셀 조회는 속도-정확도 트레이드오프**입니다. 대부분의 경우 충분하지만, 초기 자세 오류가 클 때 치명적입니다. 2~3개 이웃 복셀만 추가해도 크게 개선될 수 있습니다.

5. **LOAM은 설계 목적이 다릅니다**. 실시간 오도메트리(10Hz+)를 위해 밀집 정합을 포기한 알고리즘이므로, 오프라인 벤치마크에서의 정확도 비교는 공정하지 않습니다. 65ms의 처리 시간은 이 설계 의도를 잘 반영합니다.

### 9.2 각 알고리즘의 적합한 사용 시나리오

| 알고리즘 | 추천 사용 시나리오 |
|---------|----------------|
| Point-to-Point ICP | 법선 추정이 불가능한 희소 데이터, 빠른 프로토타이핑 |
| Point-to-Plane ICP | **범용 오프라인 정합 — 가장 추천** |
| GICP | 비등방성 오차 모델링이 중요한 경우 (정교한 공분산 추정 전제) |
| VGICP | 대규모 포인트 클라우드의 실시간/준실시간 정합 (GPU 가속 시) |
| LOAM | **실시간 LiDAR 오도메트리** (10Hz 이상 요구) |
| NDT | 강건성이 최우선인 환경 (동적 장애물, 아웃라이어 다수) |
