# main.cpp - Code Documentation

## Summary

이 파일은 LiDAR 포인트 클라우드 정합(registration)을 위한 데모 애플리케이션입니다.

### 주요 기능
1. PCD 파일들을 로드하고 Ground Truth 포즈와 매칭
2. 다양한 스캔 매칭 알고리즘 (ICP, GICP, VGICP, LOAM) 적용
3. Factor Graph 기반 포즈 최적화 (LM, iSAM2)
4. 3D 시각화 및 결과 비교

---

## Core Algorithms

| Algorithm  | Error Metric                        | 특징                      |
|------------|-------------------------------------|---------------------------|
| ICP        | E = Σ \|\|q_i - T·p_i\|\|²          | Point-to-Point 거리       |
| ICP_PLANE  | E = Σ [n_i · (q_i - T·p_i)]²        | Point-to-Plane 거리       |
| GICP       | E = Σ r^T · C^(-1) · r              | Mahalanobis 거리 (공분산) |
| VGICP      | 위와 동일 (voxel 기반)               | Voxel 가우시안 분포 매칭   |
| LOAM       | E_edge + E_plane                    | Edge/Planar 특징점 분리   |

---

## Data Flow

```
PCD Files → Point Cloud → Covariance Estimation → Voxelmap → Factor Graph → Optimization
```

---

## Coordinate Frames

- **W (World)**: 글로벌 좌표계
- **B (Base)**: 로봇 베이스 좌표계  
- **L (LiDAR)**: 라이다 센서 좌표계
- **L0 (Origin LiDAR)**: 첫 번째 프레임의 LiDAR 좌표계 (상대 포즈 기준)

---

## Headers

### Standard C++ Libraries
- `<chrono>`: 시간 측정 (최적화 소요 시간)
- `<thread>`: 멀티스레딩 (비동기 최적화)
- `<fstream>`: 파일 I/O (gt-tum.txt 읽기)
- `<iostream>`: 콘솔 출력
- `<filesystem>`: 디렉토리 탐색 (PCD 파일 목록)
- `<algorithm>`: std::sort, std::transform
- `<map>`: timestamp → pose 매핑
- `<cmath>`: M_PI, std::abs
- `<boost/format.hpp>`: 포맷팅된 문자열 출력

### GTSAM (Georgia Tech Smoothing and Mapping)
Factor Graph 라이브러리로 로봇 상태 추정에 사용됩니다.

핵심 개념:
- **Factor Graph**: 변수(노드)와 제약조건(팩터)로 구성된 그래프
- **Pose3**: SE(3) 상의 6-DoF 포즈 (3D 위치 + 3D 회전)
- **Values**: 변수들의 현재 추정값을 저장하는 컨테이너

### gtsam_points
GTSAM에 포인트 클라우드 정합 기능을 추가하는 확장 라이브러리입니다.
- https://github.com/koide3/gtsam_points

---

## MatchingCostFactorDemo Class

### Role
LiDAR 스캔 매칭 데모의 핵심 클래스. 데이터 로드, UI 제어, 최적화 실행을 담당합니다.

### Member Variables
| Variable | Type | Description |
|----------|------|-------------|
| poses, poses_gt | gtsam::Values | 현재 추정 포즈 / Ground Truth 포즈 |
| frames | vector<PointCloud::Ptr> | 각 프레임의 포인트 클라우드 |
| voxelmaps | vector<GaussianVoxelMap::Ptr> | VGICP용 복셀맵 |
| loam_features | vector<LOAMFeatures> | LOAM용 에지/평면 특징점 |

### Workflow
1. **생성자**: 데이터 로드 + 전처리 + UI 설정
2. **add noise 버튼**: 포즈에 노이즈 추가 (테스트용)
3. **optimize 버튼**: Factor Graph 최적화 실행
4. **결과**: 최적화된 포즈 vs GT 비교

---

## Constructor - Initialization Sequence

1. 로깅 시스템 초기화
2. 3D 뷰어 초기화
3. Extrinsic 변환 행렬 설정 (T_base_lidar)
4. Ground Truth 포즈 로드 (gt-tum.txt)
5. PCD 파일들 로드 및 전처리:
   - 포인트 로드 → 공분산 추정 → 법선 추정 → 복셀맵 생성 → LOAM 특징 추출
6. UI 콜백 등록

### Extrinsic Transformation: T_base_lidar
Base 프레임에서 LiDAR 프레임으로의 변환

좌표계 관계:
```
p_base = T_base_lidar * p_lidar
```

- t = [0, 0, 0.124]^T (LiDAR가 Base보다 12.4cm 위에 있음)
- R = I (회전 없음)

### TUM Format (Ground Truth)
```
timestamp tx ty tz qx qy qz qw
```
각 행은 특정 시간에서의 로봇 포즈를 나타냅니다.

### Timestamp → Pose Matching
`find_closest_pose`: PCD 타임스탬프와 가장 가까운 GT 포즈를 찾습니다.

알고리즘:
1. lower_bound로 target_ts 이상인 첫 번째 요소 찾기
2. 이전 요소와 비교하여 더 가까운 쪽 선택

시간 복잡도: O(log n) - std::map의 lower_bound 사용

### Relative Pose Calculation
첫 번째 프레임의 LiDAR 좌표계를 기준(origin)으로 설정합니다.

좌표 변환 체인:
```
W_T_L = W_T_B * T_base_lidar

여기서:
- W_T_B: World에서 Base로의 변환 (GT에서 제공)
- T_base_lidar: Base에서 LiDAR로의 변환 (extrinsic)
- W_T_L: World에서 LiDAR로의 변환
```

상대 포즈 계산:
```
L0_T_Li = W_T_L_origin^(-1) * W_T_L_i
```
이는 첫 번째 LiDAR 프레임 기준의 i번째 프레임 포즈입니다.

### float → double 변환
```
[x, y, z] → [x, y, z, 1]^T (4D homogeneous coordinate)
```
이유: SE(3) 변환을 4x4 행렬 곱셈으로 표현하기 위함

---

## Covariance Estimation (GICP/VGICP용)

각 포인트에 대해 3x3 공분산 행렬을 추정합니다.

공분산 Σ_i는 점 p_i 주변의 지역적 형상을 나타냅니다:
- **Planar surface**: 가장 작은 고유값이 0에 가까움
- **Edge/line**: 두 개의 작은 고유값
- **Corner**: 세 고유값이 비슷함

GICP에서 사용:
```
C_fused = Σ_target + R * Σ_source * R^T
```

---

## Normal Estimation (Point-to-Plane ICP용)

법선 벡터 n_i는 점 p_i에서의 표면 방향을 나타냅니다.

추정 방법: PCA (Principal Component Analysis)
- k 최근접 이웃 점들로 공분산 행렬 계산
- 가장 작은 고유값에 해당하는 고유벡터 = 법선

Point-to-Plane 오차:
```
e_i = n_i · (q_i - T·p_i)
```

---

## Gaussian VoxelMap (VGICP용)

3D 공간을 복셀로 분할하고, 각 복셀에 가우시안 분포 저장

각 복셀 v_k는 다음을 저장:
- μ_k: 복셀 내 점들의 평균 위치
- Σ_k: 복셀 내 점들의 공분산 행렬

파라미터:
- resolution = 0.5m: 복셀 크기 (클수록 계산 빠름, 작을수록 정밀)

VGICP에서의 사용:
```
E = Σ (μ_voxel - T·p_i)^T · C_voxel^(-1) · (μ_voxel - T·p_i)
```

---

## Noise Addition (Testing)

Lie Algebra 표현:
```
noise = Pose3::Expmap(ξ)

ξ ∈ se(3): 6D twist vector [ω; v]
- ω: rotation (angle-axis)
- v: translation
```

노이즈 적용:
```
poses_noisy[i] = poses_gt[i] * noise
```

---

## Correspondence Update Tolerance

포즈 변화가 이 임계값보다 작으면 대응점 재탐색을 건너뜁니다.

- **rot**: 회전 변화 임계값 (rad)
- **trans**: 이동 변화 임계값 (m)

0으로 설정 시 매 iteration마다 대응점 업데이트 (정확하지만 느림)

---

## create_factor() - Algorithm Details

### 1. ICP (Point-to-Point)

```
Cost: E = Σᵢ ||qᵢ - T·pᵢ||²
```

여기서:
- pᵢ: source 포인트
- qᵢ: target 포인트 (최근접)
- T: SE(3) 변환 행렬

Jacobian (w.r.t. pose):
```
∂e/∂ξ = [-[T·p]_× | I₃]  (6D twist에 대한 미분)
```

- **장점**: 단순, 빠름
- **단점**: Local minimum에 빠지기 쉬움, 초기값에 민감

### 2. ICP_PLANE (Point-to-Plane)

```
Cost: E = Σᵢ [nᵢ · (qᵢ - T·pᵢ)]²
```

여기서:
- nᵢ: target 포인트의 법선 벡터
- nᵢ · v: 법선 방향 거리 (수직 거리)

- **장점**: Point-to-Point보다 빠른 수렴
- **단점**: 법선 추정 필요

### 3. GICP (Generalized ICP) - Plane-to-Plane

```
Cost: E = Σᵢ rᵢᵀ · Cᵢ⁻¹ · rᵢ  (Mahalanobis distance)
```

여기서:
- rᵢ = qᵢ - T·pᵢ: residual vector
- Cᵢ = Σ_qᵢ + R·Σ_pᵢ·Rᵀ: fused covariance
- Σ_qᵢ, Σ_pᵢ: 각 점의 3x3 공분산 행렬

수학적 의미:
- 각 점을 가우시안 분포 N(μ, Σ)로 모델링
- 두 분포 간의 KL divergence 최소화
- 평면-평면 매칭으로 해석 가능

- **장점**: 기하학적 불확실성 고려, 강건함
- **단점**: 공분산 추정 필요, 계산량 증가

### 4. VGICP (Voxelized GICP)

GICP와 동일, 단 대응점을 k-NN 대신 복셀 룩업으로 찾음

```
E = Σᵢ (μ_voxel - T·pᵢ)ᵀ · C_voxel⁻¹ · (μ_voxel - T·pᵢ)
```

복셀 구조:
- 공간을 격자로 분할 (resolution = 0.5m)
- 각 복셀에 포함된 점들의 평균 μ와 공분산 Σ 저장
- O(1) 복셀 룩업으로 대응점 탐색 (k-NN의 O(log n) 대비 빠름)

- **장점**: 매우 빠름, 대규모 포인트 클라우드에 적합
- **단점**: 복셀 해상도 선택 중요

### 5. LOAM (Lidar Odometry and Mapping)

```
Total Cost: E = E_edge + E_plane
```

**Edge Error (Point-to-Line):**
```
E_edge = Σᵢ ||(T·pᵢ - x_j) × (T·pᵢ - x_l)||² / ||x_j - x_l||²
```
- x_j, x_l: 에지를 정의하는 두 target 점
- ×: 외적 (직선까지의 거리 계산에 사용)

**Plane Error (Point-to-Plane):**
```
E_plane = Σᵢ [nᵢ · (T·pᵢ - x_j)]²
```
- nᵢ = (x_j - x_l) × (x_j - x_m) / ||(x_j - x_l) × (x_j - x_m)|| (세 점으로 정의되는 평면의 법선)

**특징 추출:**
- Edge points: 높은 곡률 (법선이 급변하는 영역)
- Planar points: 낮은 곡률 (일정한 법선 방향)

- **장점**: 기하학적 특징 활용, 구조화된 환경에서 효과적
- **단점**: 특징 추출 품질에 의존, 비구조화 환경에서 취약

---

## run_optimization() - Optimization Details

### Factor Graph Structure

```
x₀ -------- x₁ -------- x₂ -------- x₃ ... xₙ
 |           |           |           |
 |           \-----------|-----------/
 |                       |
Prior              Registration Factors
```

- **Prior Factor**: x₀의 위치를 고정 (앵커)
- **Registration Factors**: 프레임 간 정합 제약
  - full_connection=true: 모든 프레임 쌍에 factor
  - full_connection=false: 연속 프레임에만 factor

### Prior Factor
```
||x₀ - x₀_prior||²_Σ
```
Σ = (1e6)⁻¹ · I₆ (매우 작은 분산 = 높은 확신도)
이는 x₀가 x₀_prior에서 거의 움직이지 않도록 강제합니다.

### Factor Connection Pattern
- **full_connection = true**: 모든 프레임 쌍에 factor 추가 (loop closure 효과)
  - x₀ ↔ x₁, x₀ ↔ x₂, ..., x₀ ↔ xₙ, x₁ ↔ x₂, ...
- **full_connection = false**: 연속 프레임에만 factor 추가 (odometry chain)
  - x₀ → x₁ → x₂ → ... → xₙ

---

## Optimization Algorithms

### 1. Levenberg-Marquardt (LM)

Update rule:
```
(H + λI) · Δx = -g
x_{k+1} = x_k ⊞ Δx
```

여기서:
- H = J^T · J: Gauss-Newton 근사 Hessian
- g = J^T · r: gradient
- λ: damping factor (trust region 조절)
- ⊞: manifold update (SE(3)에서의 덧셈)

특징:
- Batch 최적화: 모든 factor를 동시에 고려
- 수렴 시까지 반복 (maxIterations)
- 결과가 안정적, 전역 최적에 가까움

Parameters:
- maxIterations = 100: 최대 반복 횟수
- relativeErrorTol = 1e-5: 상대 오차 변화 임계값
- absoluteErrorTol = 1e-5: 절대 오차 임계값

### 2. iSAM2 (Incremental Smoothing and Mapping)

핵심 아이디어:
- Bayes Tree를 사용한 incremental factor graph 유지
- 새 factor 추가 시 영향받는 변수만 재계산
- 전체 batch 최적화보다 효율적

Update 과정:
1. 새 factor/변수 추가
2. 영향받는 clique 식별
3. 해당 clique만 재선형화 및 해결

Parameters:
- relinearizeSkip = 1: 재선형화 건너뛰기 빈도
- relinearizeThreshold = 0: 재선형화 임계값 (0=항상 재선형화)

특징:
- Online/incremental 시나리오에 적합
- 새 데이터 추가 시 빠른 업데이트
- SLAM에서 주로 사용

---

## Result Analysis

Error 계산:
```
error = GT^(-1) * Optimized
```

- **Translation Error**: ||error.translation()|| (유클리드 거리)
- **Rotation Error**: angle(error.rotation()) (축-각도 표현의 각도 성분)

---

## LOAM Feature Extraction

### Curvature-based Classification (Original LOAM)

```
c = ||Σⱼ∈S (pⱼ - pᵢ)||² / (|S| · ||pᵢ||²)
```

여기서:
- S: pᵢ의 이웃 점들
- |S|: 이웃 점 개수

분류 기준:
- c > threshold_high → Edge point (코너, 에지)
- c < threshold_low → Planar point (평면)

### Current Implementation - Normal-based Classification

법선 벡터의 정렬도를 사용한 간이 방법:

```
max_axis = max(|n_x|, |n_y|, |n_z|)
```

분류 기준:
- max_axis > 0.9 → Planar (법선이 축에 잘 정렬됨)
- max_axis < 0.7 → Edge (법선이 축에 잘 정렬되지 않음)

아이디어: 평면 표면의 법선은 주 축(x, y, z)과 잘 정렬되는 경향이 있음 (벽, 바닥, 천장 등). 반면 에지/코너에서는 법선이 축에 잘 정렬되지 않음.

예시:
- 바닥 법선: n = [0, 0, 1] → max_axis = 1.0 (planar)
- 벽 법선: n = [1, 0, 0] → max_axis = 1.0 (planar)
- 45도 에지: n = [0.7, 0, 0.7] → max_axis = 0.7 (edge 경계)
- 코너: n = [0.57, 0.57, 0.57] → max_axis = 0.57 (edge)

### Parameters
- downsample_factor = 5: 5개 점 중 1개만 검사
- max_edge = 2000: 최대 에지 포인트 수
- max_planar = 4000: 최대 평면 포인트 수

### Fallback Sampling
법선 기반 분류가 충분한 특징점을 찾지 못한 경우 균일 샘플링으로 대체합니다.

발생 상황:
- 비구조화된 환경 (자연 지형 등)
- 법선 추정 품질이 낮은 경우

---

## main() Function

애플리케이션의 진입점. 데모 객체를 생성하고 3D 뷰어를 실행합니다.

### Execution Flow
1. MatchingCostFactorDemo 생성 (데이터 로드 + 전처리)
2. guik::LightViewer::spin() - 이벤트 루프 시작 (UI 상호작용)
3. 프로그램 종료
