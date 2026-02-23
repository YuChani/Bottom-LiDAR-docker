# NDT Factor 사용 가이드

**최종 업데이트**: 2026-02-13  
**대상**: Bottom-LiDAR-Docker 프로젝트 사용자

---

## 목차
1. [개요](#1-개요)
2. [빠른 시작](#2-빠른-시작)
3. [NDT 파라미터 설명](#3-ndt-파라미터-설명)
4. [성능 튜닝 가이드](#4-성능-튜닝-가이드)
5. [다른 Factor들과의 비교](#5-다른-factor들과의-비교)
6. [문제 해결](#6-문제-해결)
7. [고급 사용법](#7-고급-사용법)

---

## 1. 개요

### 1.1 NDT(Normal Distributions Transform)란?

NDT는 3D Point Cloud Registration을 위한 확률 기반 정합 방법입니다. 전통적인 Point-to-Point/Plane 방식과 달리, **각 복셀(Voxel)을 가우시안 분포로 모델링**하여 정합을 수행합니다.

**핵심 아이디어**:
```
전통적 방법: "점 A를 점 B에 가깝게 이동"
NDT 방법:    "점 A를 가우시안 분포 B의 확률이 높은 위치로 이동"
```

### 1.2 언제 NDT를 사용해야 하나?

| 상황 | 추천 방법 | 이유 |
|------|----------|------|
| 구조화된 환경 (건물, 도로) | GICP, VGICP | 평면 특징이 많아 Point-to-Plane이 효과적 |
| 불규칙한 환경 (숲, 실내) | **NDT** | 확률 분포가 복잡한 형상을 잘 표현 |
| 노이즈가 많은 데이터 | **NDT** | 가우시안 분포가 outlier에 robust |
| 실시간 처리 필요 | VGICP, LOAM | NDT는 중간 속도 |
| 높은 정확도 필요 | GICP, **NDT** | 확률 기반 최적화가 정밀함 |

---

## 2. 빠른 시작

### 2.1 애플리케이션 빌드 (이미 완료된 경우 생략)

```bash
# Docker 컨테이너 접속
docker exec -it bottom-lidar bash

# 작업 디렉토리로 이동
cd /root/workdir

# 빌드 (이미 완료됨)
cd build
cmake ..
make -j$(nproc)

# 실행 파일 확인
ls -lh lidar_registration_benchmark  # 35MB 실행 파일
```

### 2.2 NDT로 Registration 실행

#### **STEP 1**: 데이터 준비

PCD 파일들과 Ground Truth 포즈 파일이 필요합니다.

```bash
# 예시: KITTI 데이터셋
./data/
  └── kitti/
      └── 00/
          ├── 000000.pcd
          ├── 000001.pcd
          ├── ...
          └── poses.txt  # Ground Truth (TUM 형식)
```

**Ground Truth 형식 (TUM)**:
```
# timestamp tx ty tz qx qy qz qw
1000000000.0 0.0 0.0 0.0 0.0 0.0 0.0 1.0
1000000001.0 0.1 0.0 0.0 0.0 0.0 0.0 1.0
...
```

#### **STEP 2**: 애플리케이션 실행

```bash
cd /root/workdir
./build/lidar_registration_benchmark ./data/kitti/00 ./data/kitti/00/poses.txt
```

**실행 시 인수**:
- `인수 1`: PCD 파일들이 있는 디렉토리
- `인수 2`: Ground Truth 포즈 파일 (옵션)

#### **STEP 3**: UI에서 NDT 선택

애플리케이션 실행 후 나타나는 GUI 창에서:

```
┌─────────────────────────────────────┐
│ Matching Cost Factor Demo           │
├─────────────────────────────────────┤
│ Factor Type: [VGICP ▼]              │ ← 클릭
│   • Point2Point                      │
│   • Point2Plane                      │
│   • GICP                             │
│   • VGICP                            │
│   • LOAM                             │
│   • NDT                              │ ← 선택!
│   • VGICP_GPU                        │
│                                      │
│ Num Threads: [4]                     │ ← 스레드 조정 가능
│ Downsample Resolution: [0.25]        │
│ Voxel Resolution: [1.0]              │
│                                      │
│ Optimizer: [LM ▼]                    │
│   • Levenberg-Marquardt              │
│   • ISAM2                            │
│                                      │
│ [▶ Run Registration]                 │ ← 실행 버튼
└─────────────────────────────────────┘
```

#### **STEP 4**: 결과 확인

**3D 뷰어**:
- 빨간색 Point Cloud: Source (변환 전)
- 녹색 Point Cloud: Target
- 파란색 Point Cloud: 정합 후 Source (변환 후)
- 노란색 선: Ground Truth 궤적
- 흰색 선: 추정 궤적

**콘솔 출력**:
```
[Iteration 0] Error: 1234.56, Inliers: 8520/10000 (85.2%)
[Iteration 1] Error: 456.78, Inliers: 9120/10000 (91.2%)
[Iteration 2] Error: 123.45, Inliers: 9450/10000 (94.5%)
Optimization converged in 3 iterations
Total time: 0.42 seconds
Translation error: 0.05 m
Rotation error: 0.12 deg
```

---

## 3. NDT 파라미터 설명

현재 NDT 파라미터는 `src/main.cpp` lines 434-436에 설정되어 있습니다.  
파라미터를 변경하려면 해당 코드를 수정 후 재빌드하세요.

### 3.1 Search Mode (검색 모드)

```cpp
factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT7);
```

| 모드 | 검색 복셀 수 | 속도 | 정확도 | 추천 상황 |
|------|-------------|------|--------|----------|
| **DIRECT1** | 1 (중심만) | 빠름 ⚡⚡⚡ | 낮음 | 초기 정합이 매우 정확할 때 |
| **DIRECT7** | 7 (중심+6면) | 중간 ⚡⚡ | 중간 | **대부분의 경우 (기본값)** |
| **DIRECT27** | 27 (중심+26개) | 느림 ⚡ | 높음 | 고정밀 정합 필요시 |

**DIRECT7 설명**:
```
중심 복셀 + 6개 면 이웃 = 총 7개 복셀 검색

     [Z+]
      |
[Y-]--[중심]--[Y+]
      |
     [Z-]

  [X-] [X+]
```

**변경 예시**:
```cpp
// 속도 우선 (초기 정합이 정확한 경우)
factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT1);

// 정확도 우선 (처리 시간 여유가 있는 경우)
factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT27);
```

### 3.2 Outlier Ratio (이상치 비율)

```cpp
factor->set_outlier_ratio(0.1);  // 0.0 ~ 1.0
```

**의미**: 전체 포인트 중 몇 %를 outlier로 간주할 것인가?

| 값 | 설명 | 추천 상황 |
|----|------|----------|
| **0.0** | Outlier 없음 가정 | 고품질 센서, 실내 환경 |
| **0.05-0.15** | 일반적인 outlier 수준 | **대부분의 LiDAR (기본값 0.1)** |
| **0.2-0.5** | 높은 outlier 수준 | 노이즈 많은 환경, 동적 객체 많음 |

**동작 원리**:
- Outlier ratio가 높을수록 → 균등 분포(uniform distribution) 가중치 증가
- 가우시안 분포에서 벗어난 점들에 대해 페널티 감소
- 결과적으로 outlier에 더 robust해짐

**변경 예시**:
```cpp
// 고품질 데이터 (실내 ToF 센서)
factor->set_outlier_ratio(0.0);

// 노이즈 많은 환경 (비오는 날 실외 LiDAR)
factor->set_outlier_ratio(0.3);
```

### 3.3 Regularization Epsilon (정규화 계수)

```cpp
factor->set_regularization_epsilon(1e-3);  // 1e-5 ~ 1e-1
```

**의미**: 공분산 행렬의 작은 고유값을 얼마나 정규화할 것인가?

**수식**:
```
λ_regularized = max(λ_original, ε * λ_max)
```

| 값 | 효과 | 추천 상황 |
|----|------|----------|
| **1e-4** | 원래 분포 거의 유지 | 고품질 데이터, 충분한 포인트 |
| **1e-3** | 균형잡힌 정규화 | **대부분의 경우 (기본값)** |
| **1e-2** | 강한 정규화 | 포인트 밀도 낮음, 수치 불안정 |

**변경이 필요한 경우**:
- **수치 오류 발생 시** (NaN, Inf 출력):
  ```cpp
  factor->set_regularization_epsilon(1e-2);  // 정규화 강화
  ```
- **과도한 평활화** (디테일 손실):
  ```cpp
  factor->set_regularization_epsilon(1e-4);  // 정규화 약화
  ```

### 3.4 Num Threads (스레드 수)

```cpp
factor->set_num_threads(num_threads);  // UI에서 설정
```

**의미**: Correspondence 검색 및 비용 계산에 사용할 CPU 스레드 수

**추천값**:
```cpp
// CPU 코어 수 확인
int max_threads = std::thread::hardware_concurrency();

// 일반적으로 코어 수 - 1 또는 코어 수 사용
factor->set_num_threads(max_threads);
```

**주의사항**:
- 스레드 수가 너무 많으면 오버헤드 증가
- 다른 프로세스가 실행 중이면 일부 코어 남겨두기

---

## 4. 성능 튜닝 가이드

### 4.1 속도 최적화

**목표**: Registration 시간 단축 (정확도 약간 희생)

```cpp
// src/main.cpp, create_factor() 함수 내 수정

// 1. 검색 모드를 DIRECT1으로 변경
factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT1);

// 2. 스레드 수 최대화
factor->set_num_threads(std::thread::hardware_concurrency());

// 3. Voxel 해상도 증가 (UI에서 조정 가능)
//    Voxel Resolution: 0.5 → 1.0 (복셀이 커질수록 빠름)
```

**추가 최적화**:
```cpp
// Downsample Resolution 증가 (더 적은 포인트 처리)
// UI에서 "Downsample Resolution" 값 증가: 0.25 → 0.5
```

### 4.2 정확도 최적화

**목표**: 최고 정밀도 (처리 시간 증가)

```cpp
// 1. 검색 모드를 DIRECT27으로 변경
factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT27);

// 2. Outlier ratio 낮추기 (고품질 데이터 가정)
factor->set_outlier_ratio(0.05);

// 3. 정규화 약화 (원래 분포 유지)
factor->set_regularization_epsilon(1e-4);

// 4. Voxel 해상도 감소 (더 세밀한 분포)
//    Voxel Resolution: 1.0 → 0.5
```

### 4.3 Robust성 최적화

**목표**: 노이즈 및 outlier에 강건한 정합

```cpp
// 1. Outlier ratio 증가
factor->set_outlier_ratio(0.2);

// 2. 정규화 강화 (수치 안정성)
factor->set_regularization_epsilon(1e-2);

// 3. 검색 모드는 DIRECT7 유지 (균형)
factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT7);
```

### 4.4 데이터셋별 추천 설정

#### **KITTI (자율주행, 실외)**
```cpp
factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT7);
factor->set_outlier_ratio(0.1);
factor->set_regularization_epsilon(1e-3);
// Voxel Resolution: 1.0m
// Downsample Resolution: 0.25m
```

#### **실내 환경 (ToF 센서)**
```cpp
factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT7);
factor->set_outlier_ratio(0.05);  // 노이즈 적음
factor->set_regularization_epsilon(1e-3);
// Voxel Resolution: 0.5m (작은 공간)
// Downsample Resolution: 0.1m
```

#### **숲/불규칙 환경**
```cpp
factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT27);  // 정확도 우선
factor->set_outlier_ratio(0.15);  // outlier 많음
factor->set_regularization_epsilon(1e-3);
// Voxel Resolution: 0.75m
// Downsample Resolution: 0.2m
```

---

## 5. 다른 Factor들과의 비교

### 5.1 정량적 비교 (예시 - KITTI 기준)

| Factor | Registration 시간 | Translation 오차 | Rotation 오차 | 메모리 사용량 |
|--------|------------------|-----------------|--------------|-------------|
| Point2Point | 0.05초 | 0.15m | 0.5° | 낮음 |
| Point2Plane | 0.08초 | 0.10m | 0.3° | 낮음 |
| GICP | 0.50초 | 0.05m | 0.1° | 중간 |
| VGICP | 0.15초 | 0.06m | 0.12° | 중간 |
| LOAM | 0.12초 | 0.07m | 0.15° | 낮음 |
| **NDT** | **0.25초** | **0.06m** | **0.13°** | **높음** |
| VGICP_GPU | 0.05초 | 0.06m | 0.12° | 중간 (GPU) |

**참고**: 실제 성능은 데이터셋, 하드웨어, 파라미터에 따라 달라집니다.

### 5.2 정성적 비교

#### **NDT vs GICP/VGICP**

| 특성 | NDT | GICP/VGICP |
|------|-----|------------|
| **알고리즘** | 가우시안 분포 매칭 | 포인트-포인트 공분산 매칭 |
| **강점** | Outlier robust, 불규칙 환경 | 정밀함, 구조화된 환경 |
| **약점** | 메모리 사용량 높음 | Correspondence 검색 비용 높음 (GICP) |
| **속도** | 중간 | GICP 느림, VGICP 빠름 |
| **정확도** | 높음 | 높음 |

**언제 NDT를 선택?**
- ✅ 불규칙한 환경 (숲, 복잡한 실내)
- ✅ Outlier가 많은 데이터
- ✅ 평면이 적은 환경
- ❌ 메모리가 제한적인 경우
- ❌ 실시간 처리 필수 (→ VGICP 또는 LOAM 추천)

#### **NDT vs LOAM**

| 특성 | NDT | LOAM |
|------|-----|------|
| **입력 데이터** | 일반 Point Cloud | Edge/Planar Features |
| **전처리** | 필요 없음 | Feature 추출 필요 |
| **속도** | 중간 | 빠름 |
| **정확도** | 높음 (dense) | 높음 (feature-based) |
| **환경 의존성** | 낮음 | Edge/Plane 특징 필요 |

**언제 NDT를 선택?**
- ✅ Feature 추출이 어려운 환경 (균질한 표면)
- ✅ Dense Point Cloud 사용 가능
- ❌ 실시간 SLAM 필요 (→ LOAM 추천)
- ❌ LiDAR ring 정보 없음 (LOAM은 ring 필요)

---

## 6. 문제 해결

### 6.1 빌드 에러

#### **문제**: `integrated_ndt_factor.hpp` 파일을 찾을 수 없음

**원인**: `libgtsam_points.so`가 설치되지 않음

**해결**:
```bash
cd /root/workdir/thirdparty/gtsam_points/build
sudo make install

# 설치 확인
ls /usr/local/include/gtsam_points/factors/integrated_ndt_factor.hpp
```

#### **문제**: LSP 오류 (VS Code 등)

**원인**: LSP가 시스템 헤더를 인식하지 못함

**해결**: 무시하세요. 실제 빌드는 성공합니다.
```bash
# Docker에서 빌드 테스트
docker exec bottom-lidar bash -c "cd /root/workdir/build && make"
```

### 6.2 런타임 에러

#### **문제**: Segmentation Fault 발생

**가능한 원인**:
1. **Voxel Map이 비어있음**:
   ```cpp
   // 디버깅: 복셀 개수 확인
   std::cout << "Voxels: " << target_voxelmap->size() << std::endl;
   ```
   **해결**: Voxel Resolution을 줄이거나 Downsample Resolution 증가

2. **메모리 부족**:
   **해결**: 스레드 수 감소, Downsample Resolution 증가

#### **문제**: NaN 또는 Inf 출력

**원인**: 공분산 행렬이 singular (고유값 0에 가까움)

**해결**:
```cpp
// 정규화 강화
factor->set_regularization_epsilon(1e-2);  // 기존 1e-3 → 1e-2
```

#### **문제**: 정합이 실패하거나 발산

**원인**: 초기 정합이 부정확

**해결**:
1. **초기 포즈 개선**:
   ```cpp
   // Odometry 또는 IMU로 초기 추정값 제공
   gtsam::Pose3 init_pose = compute_initial_guess(source, target);
   ```

2. **검색 모드 확장**:
   ```cpp
   factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT27);
   ```

3. **Multi-resolution 전략**:
   ```cpp
   // 1. 큰 복셀로 초기 정합
   factor->set_resolution(2.0);
   optimize();
   
   // 2. 작은 복셀로 정밀 정합
   factor->set_resolution(1.0);
   optimize();
   ```

### 6.3 성능 문제

#### **문제**: 처리 속도가 너무 느림

**체크리스트**:
1. **스레드 수 확인**:
   ```cpp
   factor->set_num_threads(8);  // CPU 코어 수에 맞게
   ```

2. **검색 모드 확인**:
   ```cpp
   // DIRECT27 → DIRECT7 변경
   factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT7);
   ```

3. **Downsample Resolution 증가**:
   ```
   UI에서: 0.1 → 0.25 또는 0.5
   ```

4. **프로파일링**:
   ```cpp
   auto start = std::chrono::high_resolution_clock::now();
   factor->update_correspondences();
   auto end = std::chrono::high_resolution_clock::now();
   std::cout << "Correspondence: " 
             << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() 
             << " ms" << std::endl;
   ```

---

## 7. 고급 사용법

### 7.1 파라미터 자동 튜닝

**목표**: Point Cloud 특성에 따라 파라미터 자동 설정

```cpp
// src/main.cpp에 추가 (create_factor 함수 내)

// 1. 포인트 밀도 계산
double avg_nn_dist = compute_average_nn_distance(source);

// 2. Outlier ratio 자동 설정
double auto_outlier_ratio = std::min(0.3, avg_nn_dist / voxel_resolution);
factor->set_outlier_ratio(auto_outlier_ratio);

// 3. 포인트 수에 따른 검색 모드 선택
if(source->size() > 50000) {
  factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT1);  // 빠르게
} else {
  factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT7);  // 정확하게
}
```

**Helper 함수**:
```cpp
double compute_average_nn_distance(const gtsam_points::PointCloud::ConstPtr& cloud) {
  // KD-Tree로 평균 nearest neighbor distance 계산
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);
  
  double sum = 0.0;
  for(size_t i = 0; i < cloud->size(); i += 100) {  // 샘플링
    std::vector<int> indices(2);
    std::vector<float> distances(2);
    kdtree.nearestKSearch(cloud->points[i], 2, indices, distances);
    sum += std::sqrt(distances[1]);  // 자기 자신 제외
  }
  return sum / (cloud->size() / 100);
}
```

### 7.2 Multi-Resolution NDT

**목표**: 거친 정합 → 정밀 정합 순차 수행

```cpp
// src/main.cpp, 새로운 함수 추가

gtsam::Pose3 multi_resolution_ndt(
  const gtsam_points::PointCloud::ConstPtr& source,
  const gtsam_points::PointCloud::ConstPtr& target,
  const gtsam::Pose3& initial_guess)
{
  std::vector<double> resolutions = {2.0, 1.0, 0.5};  // 거친 → 세밀한
  gtsam::Pose3 current_pose = initial_guess;
  
  for(double res : resolutions) {
    // Voxel Map 재생성
    auto voxelmap = create_voxelmap(target, res);
    
    // NDT Factor 생성
    auto factor = gtsam::make_shared<gtsam_points::IntegratedNDTFactor>(
      X(0), X(1), voxelmap, source);
    factor->set_num_threads(8);
    factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT7);
    
    // Resolution에 따른 파라미터 조정
    if(res > 1.0) {
      factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT1);  // 빠르게
      factor->set_outlier_ratio(0.2);  // robust하게
    } else {
      factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT7);
      factor->set_outlier_ratio(0.1);
    }
    
    // 최적화
    current_pose = optimize(factor, current_pose);
  }
  
  return current_pose;
}
```

### 7.3 NDT + ICP Hybrid

**목표**: NDT로 초기 정합 + GICP로 정밀 정합

```cpp
// 1단계: NDT로 빠르게 초기 정합
auto ndt_factor = gtsam::make_shared<gtsam_points::IntegratedNDTFactor>(...);
ndt_factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT7);
gtsam::Pose3 ndt_result = optimize(ndt_factor, initial_guess);

// 2단계: GICP로 정밀 정합
auto gicp_factor = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(...);
gtsam::Pose3 final_result = optimize(gicp_factor, ndt_result);
```

**장점**:
- NDT: 넓은 수렴 범위 (초기 정합 부정확해도 OK)
- GICP: 높은 정밀도 (수렴 범위 좁음, 초기값 중요)
- Hybrid: 양쪽 장점 활용

### 7.4 커스텀 비용 함수

**목표**: NDT + 추가 제약 조건 (예: Planar Constraint)

```cpp
// gtsam_points/factors/integrated_ndt_factor_impl.hpp 수정 (고급 사용자)

// evaluate() 함수 내에 추가
Eigen::VectorXd evaluate_with_constraint(...) {
  // 기존 NDT 비용
  Eigen::VectorXd ndt_cost = evaluate_ndt(...);
  
  // 추가: Planar constraint (Z축 이동 제한)
  double z_translation = delta.translation().z();
  double planar_penalty = 100.0 * z_translation * z_translation;
  
  // 결합
  Eigen::VectorXd total_cost(ndt_cost.size() + 1);
  total_cost.head(ndt_cost.size()) = ndt_cost;
  total_cost(ndt_cost.size()) = planar_penalty;
  
  return total_cost;
}
```

---

## 8. 참고 자료

### 8.1 프로젝트 문서

- **NDT Factor 상세 설명**: `docs/ndt_factor_detailed_ko.md` (34KB, 1,069줄)
  - NDT 알고리즘 이론
  - 클래스 구조 및 구현
  - 야코비안 유도
  - 사용 예제

- **UML 다이어그램**: `docs/uml_diagrams.md` (20KB, 883줄)
  - 클래스 계층 구조
  - 데이터 흐름
  - Main ↔ NDT 통합 구조

- **통합 변경사항**: `docs/integration_changelog.md`
  - Main 코드 수정 내역
  - 빌드 검증 결과

### 8.2 논문 및 외부 자료

**NDT 알고리즘**:
- Biber & Straßer (2003), "The Normal Distributions Transform: A New Approach to Laser Scan Matching", IROS
- Magnusson (2009), "The Three-Dimensional Normal-Distributions Transform", PhD Thesis, Örebro University
- Magnusson et al. (2007), "Scan registration for autonomous mining vehicles using 3D-NDT", Journal of Field Robotics

**참조 구현**:
- koide3/ndt_omp: https://github.com/koide3/ndt_omp
- tier4/ndt_omp: https://github.com/tier4/ndt_omp (Autoware 기반)
- PCL NDT: https://pointclouds.org/documentation/classpcl_1_1_normal_distributions_transform.html

**GTSAM**:
- GTSAM 공식 문서: https://gtsam.org/
- Factor Graph Tutorial: https://gtsam.org/tutorials/intro.html

### 8.3 관련 파일

**구현 코드**:
```
thirdparty/gtsam_points/
├── include/gtsam_points/
│   ├── types/gaussian_voxelmap_cpu.hpp       # NDT 분포 계산
│   └── factors/
│       ├── integrated_ndt_factor.hpp         # NDT Factor 인터페이스
│       └── impl/integrated_ndt_factor_impl.hpp  # NDT Factor 구현
└── src/gtsam_points/factors/
    └── integrated_ndt_factor.cpp             # 템플릿 인스턴스화
```

**Main 애플리케이션**:
```
src/
├── main.cpp                  # NDT 통합 (lines 43, 276, 430-438)
├── loam_feature.cpp          # LOAM feature 추출
└── loam_feature.hpp
```

---

## 9. FAQ

### Q1: NDT가 GICP/VGICP보다 느린데, 언제 사용해야 하나요?

**A**: NDT는 **불규칙한 환경** 및 **outlier가 많은 데이터**에서 더 robust합니다. 속도가 중요하면 VGICP, 정확도와 robust성이 중요하면 NDT를 선택하세요.

### Q2: 초기 정합이 부정확할 때 NDT가 실패합니다. 해결책은?

**A**: 
1. 검색 모드를 DIRECT27로 변경 (더 넓은 검색)
2. Multi-resolution 전략 사용 (큰 복셀로 초기 정합)
3. ICP로 초기 정합 후 NDT 사용

### Q3: 메모리 사용량이 너무 높습니다. 어떻게 줄이나요?

**A**:
1. Voxel Resolution 증가 (1.0 → 2.0) - 복셀 수 감소
2. Downsample Resolution 증가 (0.25 → 0.5) - 포인트 수 감소
3. 스레드 수 감소 (병렬 처리 메모리 오버헤드)

### Q4: UI에서 NDT 파라미터를 조정하고 싶어요.

**A**: 현재는 코드 수정 후 재빌드가 필요합니다. 향후 개선 사항으로 UI 슬라이더 추가 예정입니다.  
지금은 `src/main.cpp` lines 434-436을 수정하세요.

### Q5: NDT와 LOAM 중 어떤 것이 더 나은가요?

**A**:
- **LOAM**: 실시간 SLAM, LiDAR ring 정보 있음, edge/plane 특징 명확
- **NDT**: Dense registration, ring 정보 불필요, 균질한 표면

일반적으로 SLAM은 LOAM, registration은 NDT를 추천합니다.

### Q6: 실행 시 "Factor type not found" 오류가 납니다.

**A**: `libgtsam_points.so`가 재빌드되었는지 확인하세요.
```bash
# gtsam_points 재빌드
cd thirdparty/gtsam_points/build
make -j$(nproc)
sudo make install

# Main 애플리케이션 재빌드
cd /root/workdir/build
make -j$(nproc)
```

---

## 10. 다음 단계

### 10.1 기본 사용 완료 후

1. **성능 비교**:
   ```bash
   # 모든 Factor 타입으로 동일 데이터 실행
   # 시간, 정확도 비교
   ```

2. **파라미터 튜닝**:
   - 자신의 데이터셋에 맞는 최적 파라미터 찾기
   - Search mode, outlier ratio, epsilon 조정

3. **벤치마크**:
   - KITTI, ETH, TUM 등 공개 데이터셋으로 평가
   - 다른 방법들과 정량적 비교

### 10.2 고급 기능 활용

1. **Multi-resolution 구현** (섹션 7.2)
2. **Hybrid 방법 구현** (섹션 7.3)
3. **UI 파라미터 노출** (섹션 7.1)
4. **자동 파라미터 튜닝** (섹션 7.1)

### 10.3 기여 및 개선

**개선 아이디어**:
- Adaptive voxel resolution
- GPU 가속 NDT (CUDA)
- Real-time NDT SLAM
- ROS2 integration

**버그 리포트 및 제안**:
- GitHub Issues: [프로젝트 저장소]
- 문서 개선 요청 환영

---

**문서 버전**: 1.0  
**최종 수정**: 2026-02-13  
**작성자**: Sisyphus AI  
**라이선스**: MIT (프로젝트와 동일)

---

**END OF DOCUMENT**
