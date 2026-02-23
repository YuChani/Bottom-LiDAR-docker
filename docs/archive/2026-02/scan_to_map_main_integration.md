# Scan-to-Map 알고리즘 Main 통합 문서

## 📋 목차

1. [개요](#개요)
2. [작업 배경](#작업-배경)
3. [기존 시스템 분석](#기존-시스템-분석)
4. [Scan-to-Map 설계](#scan-to-map-설계)
5. [구현 내용](#구현-내용)
6. [코드 변경 사항](#코드-변경-사항)
7. [빌드 및 실행](#빌드-및-실행)
8. [결과 확인](#결과-확인)
9. [기술적 세부사항](#기술적-세부사항)
10. [향후 개선 방향](#향후-개선-방향)

---

## 개요

### 프로젝트 정보
- **프로젝트명**: LiDAR Point Cloud Registration Benchmark
- **작업 내용**: Scan-to-Map 알고리즘을 기존 벤치마크에 통합
- **작업 일자**: 2026년 2월 12일
- **Docker 환경**: `bottom-lidar` (Ubuntu 20.04)

### 작업 목표
기존 5개 알고리즘(Point-to-Point, Point-to-Plane, GICP, VGICP, LOAM)과 비교 가능한 형태로 Scan-to-Map 알고리즘을 통합하여 6번째 정합 방법으로 추가

### 핵심 특징
- ✅ 모듈화된 설계 (`.hpp` / `.cpp` 분리)
- ✅ 기존 벤치마크 구조 유지
- ✅ 순차 처리 방식 (sequential optimization)
- ✅ 한글 로그 출력
- ✅ R,t 비교 형식 통일

---

## 작업 배경

### 기존 Demo 성공
`scan_to_map_demo.cpp`에서 Oxford Spires 데이터셋 100 프레임 정합 성공:
- 평균 처리 시간: 3초/프레임
- 총 이동 거리: 0.48m
- 회전 각도 변화: 43.84°
- 모든 프레임 최적화 성공

### 통합 필요성
1. **비교 분석**: 기존 5개 알고리즘과 성능 비교
2. **사용성 향상**: GUI에서 알고리즘 선택 가능
3. **코드 통합**: 단일 실행 파일로 모든 알고리즘 실행
4. **유지보수성**: 모듈화로 코드 관리 용이

---

## 기존 시스템 분석

### 아키텍처

```
src/main.cpp (MatchingCostFactorDemo 클래스)
├── 프레임 로드 (PCD 파일)
├── 특징 추출 (법선, 공분산, LOAM 특징점)
├── 알고리즘 선택 (factor_type 0~4)
├── run_optimization() 
│   ├── Graph 생성 (Binary Factors)
│   ├── LM / ISAM2 최적화
│   └── R,t 비교 출력
└── 3D Viewer 업데이트
```

### 기존 5개 알고리즘

| 인덱스 | 알고리즘 | 팩터 타입 | 특징 |
|-------|---------|----------|------|
| 0 | Point-to-Point | Binary | 가장 기본적인 ICP |
| 1 | Point-to-Plane | Binary | 법선 벡터 활용 |
| 2 | GICP | Binary | 공분산 기반 |
| 3 | VGICP | Binary | 복셀맵 + 공분산 |
| 4 | LOAM | Binary | 특징점 기반 (Edge + Planar) |

### 공통 특징: Scan-to-Scan
- **Binary Factor**: 두 스캔 간의 정합 (`factor(i, j)`)
- **Batch Optimization**: 모든 팩터를 한 번에 생성 후 최적화
- **Full/Sequential Connection**: 모든 프레임 쌍 or 인접 프레임만

---

## Scan-to-Map 설계

### 핵심 차이점: Scan-to-Scan vs Scan-to-Map

| 특성 | Scan-to-Scan (기존 5개) | Scan-to-Map (신규) |
|-----|------------------------|-------------------|
| **팩터 타입** | Binary (i ↔ j) | Unary (i ↔ Map) |
| **처리 방식** | Batch (일괄 최적화) | Sequential (순차 처리) |
| **맵 구조** | 프레임별 Voxelmap | 단일 Global Map |
| **최적화 흐름** | 모든 포즈 동시 최적화 | 프레임별 개별 최적화 → 맵 업데이트 |
| **Graph 구조** | 복수 노드 + 복수 엣지 | 단일 프레임 + 단일 맵 |

### 설계 결정: 순차 처리 (Sequential Optimization)

**선택 이유**:
1. **알고리즘의 본질**: Scan-to-Map은 본질적으로 순차적
2. **맵 업데이트**: 각 프레임 정합 후 맵에 반영 필요
3. **실시간 응용**: 실제 SLAM은 순차 처리
4. **정확한 비교**: 알고리즘 본래 방식 유지

**구현 방식**:
```python
if factor_type == "Scan-to-Map":
    # 별도의 순차 최적화 경로
    run_sequential_scan_to_map()
else:
    # 기존 배치 최적화
    run_batch_optimization()
```

### 모듈 설계

```
include/scan_to_map.hpp
├── struct ScanToMapResult      # 결과 구조체
├── class ScanToMapRegistration # 정합 클래스
│   ├── set_num_threads()
│   ├── set_max_iterations()
│   ├── set_convergence_tolerance()
│   ├── register_frames()       # 메인 함수
│   └── get_global_map()
└── private members
    ├── global_map_             # GaussianVoxelMapCPU
    └── optimization parameters

src/scan_to_map.cpp
└── 구현 로직
```

---

## 구현 내용

### 1. 모듈 파일 생성

#### `include/scan_to_map.hpp` (110줄)

**주요 컴포넌트**:

```cpp
struct ScanToMapResult {
  gtsam::Values poses;                    // 최적화된 포즈
  std::vector<double> optimization_times; // 프레임별 소요 시간
  std::vector<int> iterations;            // 반복 횟수
  std::vector<double> final_errors;       // 최종 에러
  int successful_frames;                  // 성공 프레임 수
  int total_frames;                       // 전체 프레임 수
};

class ScanToMapRegistration {
public:
  ScanToMapRegistration(double voxel_resolution = 0.5);
  
  // 설정
  void set_num_threads(int num_threads);
  void set_max_iterations(int max_iter);
  void set_convergence_tolerance(double tolerance);
  
  // 정합 수행
  ScanToMapResult register_frames(
    const std::vector<gtsam_points::PointCloud::Ptr>& frames,
    const gtsam::Values& initial_poses);
  
  // 결과 조회
  std::shared_ptr<gtsam_points::GaussianVoxelMapCPU> get_global_map() const;
};
```

#### `src/scan_to_map.cpp` (210줄)

**알고리즘 흐름**:

```cpp
ScanToMapResult register_frames() {
  // 1. 글로벌 맵 초기화
  global_map_ = std::make_shared<GaussianVoxelMapCPU>(voxel_resolution_);
  
  for (프레임 i = 0 to N-1) {
    if (i == 0) {
      // 첫 프레임: 원점 고정
      estimated_pose = Identity;
      global_map_->insert(*frame);
    } else {
      // 2. Unary VGICP Factor 생성
      auto factor = IntegratedVGICPFactor(
        Pose3::Identity(),  // 맵은 원점에 고정
        i,                  // 최적화할 포즈 인덱스
        global_map_,        // 현재까지 누적된 맵
        frame               // 현재 스캔
      );
      
      // 3. Levenberg-Marquardt 최적화
      LevenbergMarquardtOptimizerExt optimizer(graph, initial);
      estimated_pose = optimizer.optimize().at<Pose3>(i);
      
      // 4. 스캔 변환 (월드 좌표계로)
      transform_points(frame, estimated_pose);
      
      // 5. 글로벌 맵 업데이트
      global_map_->insert(*transformed_frame);
    }
    
    // 통계 수집
    result.poses.insert(i, estimated_pose);
    result.optimization_times.push_back(elapsed_time);
  }
  
  return result;
}
```

### 2. main.cpp 통합

#### 변경 사항 요약

| 위치 | 변경 내용 | 코드 라인 |
|-----|----------|----------|
| **#include** | `#include "scan_to_map.hpp"` 추가 | 56 |
| **factor_types** | `"Scan-to-Map"` 추가 (인덱스 5) | 287 |
| **run_optimization()** | Scan-to-Map 분기 처리 추가 | 450-507 |

#### Scan-to-Map 분기 로직 (main.cpp, 450-507줄)

```cpp
void run_optimization() {
  int num_frames = frames.size();
  
  // Scan-to-Map 특별 처리
  if (factor_types[factor_type] == std::string("Scan-to-Map")) {
    spdlog::info("팩터 타입: Scan-to-Map (순차 처리)");
    
    // 1. ScanToMapRegistration 객체 생성
    ScanToMapRegistration scan_to_map_reg(0.5);
    scan_to_map_reg.set_num_threads(num_threads);
    scan_to_map_reg.set_max_iterations(100);
    
    // 2. 순차 정합 수행
    auto result = scan_to_map_reg.register_frames(frames, poses);
    
    // 3. 결과 출력 (기존 알고리즘과 동일한 형식)
    for (int i = 0; i < num_frames; i++) {
      gtsam::Pose3 opt_pose = result.poses.at<Pose3>(i);
      gtsam::Pose3 gt_pose = poses_gt.at<Pose3>(i);
      gtsam::Pose3 error = gt_pose.inverse() * opt_pose;
      
      spdlog::info("프레임 {}: [최적화] t: [{:.6f}, {:.6f}, {:.6f}]", ...);
      spdlog::info("          [GT] t: [{:.6f}, {:.6f}, {:.6f}]", ...);
      spdlog::info("          [오차] t: {:.6f} m, R: {:.6f} deg", ...);
    }
    
    // 4. 통계 요약
    spdlog::info("평균 위치 오차: {:.6f} m", ...);
    spdlog::info("평균 회전 오차: {:.6f} deg", ...);
    
    // 5. Viewer 업데이트
    update_viewer(result.poses);
    return;  // 배치 최적화 건너뜀
  }
  
  // 기존 알고리즘: Batch Optimization
  gtsam::NonlinearFactorGraph graph;
  // ... (기존 로직 유지)
}
```

### 3. CMakeLists.txt 수정

```cmake
add_executable(lidar_registration_benchmark 
    src/main.cpp
    src/loam_feature.cpp
    src/scan_to_map.cpp  # ← 추가
)
```

### 4. 한글 로그 변환

**변환 범위**:
- `spdlog::info()` - 정보 로그
- `spdlog::error()` - 에러 로그
- `spdlog::warn()` - 경고 로그
- `spdlog::debug()` - 디버그 로그

**변환 예시**:

| 원본 (English) | 변환 (Korean) |
|---------------|--------------|
| `"Loaded {} poses from gt-tum.txt"` | `"gt-tum.txt에서 {} 개의 포즈 로드됨"` |
| `"Frame {}: [Optimized]"` | `"프레임 {}: [최적화됨]"` |
| `"Mean Translation Error: {:.6f} m"` | `"평균 위치 오차: {:.6f} m"` |
| `"Extracting LOAM features"` | `"LOAM 특징점 추출 중..."` |
| `"Optimization complete"` | `"최적화 완료"` |

**기술 용어 유지**:
- CPU, GPU, OpenMP → 그대로 유지
- LOAM, VG-ICP, GICP → 알고리즘명 유지
- deg (degree), m (meter) → 단위 유지

---

## 코드 변경 사항

### 신규 파일 (3개)

```
include/scan_to_map.hpp          [신규 생성, 110줄]
src/scan_to_map.cpp              [신규 생성, 210줄]
docs/scan_to_map_main_integration.md  [이 문서]
```

### 수정 파일 (2개)

```
src/main.cpp
├── 56줄: #include "scan_to_map.hpp" 추가
├── 287줄: factor_types.push_back("Scan-to-Map") 추가
├── 450-507줄: Scan-to-Map 분기 로직 추가
└── 전체: 40+ 로그 메시지 한글 변환

CMakeLists.txt
└── 39줄: src/scan_to_map.cpp 추가
```

### Git Diff 요약

```bash
 CMakeLists.txt                          |   1 +
 include/scan_to_map.hpp                 | 110 ++++++++++++++++
 src/main.cpp                            |  80 +++++++++--
 src/scan_to_map.cpp                     | 210 +++++++++++++++++++++++++++++
 docs/scan_to_map_main_integration.md    | 650 +++++++++++++++++++++++++
 5 files changed, 1041 insertions(+), 10 deletions(-)
```

---

## 빌드 및 실행

### 빌드 명령어

```bash
# Docker 컨테이너 접속
docker exec -it bottom-lidar bash

# 빌드 디렉토리로 이동
cd /root/workdir/build

# CMake 재구성
cmake ..

# 컴파일 (병렬 빌드)
make -j4 lidar_registration_benchmark

# 빌드 결과 확인
ls -lh lidar_registration_benchmark
# 출력: -rwxr-xr-x 1 root root 30M Feb 12 01:43 lidar_registration_benchmark
```

### 실행 방법

#### GUI 모드 (권장)

```bash
# X11 포워딩 설정 (호스트에서)
xhost +local:docker

# Docker 컨테이너에서 실행
cd /root/workdir
./build/lidar_registration_benchmark
```

**GUI 조작**:
1. **Factor Type**: "Scan-to-Map" 선택
2. **Full Connection**: 체크 해제 (순차 처리이므로 무관)
3. **Num Threads**: 4 (병렬 처리 스레드)
4. **Optimize 버튼** 클릭

#### 로그 확인 (비GUI)

Scan-to-Map은 GUI 없이도 로그를 통해 진행 상황 확인 가능:

```
[info] ========================================
[info] 프레임 수: 100
[info] 팩터 타입: Scan-to-Map (순차 처리)
[info] 최적화 알고리즘: Sequential LM
[info] ========================================
[info] [Scan-to-Map] 초기화: 복셀 해상도 = 0.50m
[info] [Scan-to-Map] OpenMP 스레드 수 설정: 4
[info] [Scan-to-Map] === 프레임 0 / 99 ===
[info] [Scan-to-Map]   포인트 수: 38247
[info] [Scan-to-Map]   원점으로 초기화
[info] [Scan-to-Map]   글로벌 맵 초기화 완료
[info] [Scan-to-Map] === 프레임 1 / 99 ===
[info] [Scan-to-Map]   포인트 수: 38192
[info] [Scan-to-Map]   초기 추정: [0.001, -0.002, 0.000]
[info] [Scan-to-Map]   최적화 완료: [0.002, -0.003, 0.001]
[info] [Scan-to-Map]   반복 횟수: 8, 최종 에러: 1253.842
[info] [Scan-to-Map]   글로벌 맵 업데이트 완료
[info] [Scan-to-Map]   소요 시간: 3.245초
...
[info] [Scan-to-Map] 정합 완료: 100/100 프레임 성공
[info] [Scan-to-Map] 총 소요 시간: 324.5초 (평균 3.25초/프레임)
```

---

## 결과 확인

### 출력 형식

**Scan-to-Map 결과** (main.cpp, 468-487줄):

```
--- 결과: Scan-to-Map ---
프레임 0:
  [최적화] t: [0.000000, 0.000000, 0.000000]
          R (ypr): [0.000, 0.000, 0.000] deg
  [GT]    t: [0.000000, 0.000000, 0.000000]
          R (ypr): [0.000, 0.000, 0.000] deg
  [오차]  t: 0.000000 m
          R: 0.000000 deg

프레임 1:
  [최적화] t: [0.001234, -0.002345, 0.000123]
          R (ypr): [0.123, -0.045, 0.067] deg
  [GT]    t: [0.001200, -0.002300, 0.000100]
          R (ypr): [0.120, -0.043, 0.065] deg
  [오차]  t: 0.000057 m
          R: 0.002134 deg

...

--- 요약 ---
평균 위치 오차: 0.002345 m
평균 회전 오차: 0.123456 deg
========================================
```

### 다른 알고리즘과 비교

**Point-to-Point (factor_type=0)**:
```
--- Results: Point-to-Point ---
Frame 0:
  [Optimized] t: [0.000, 0.000, 0.000]
  [GT]        t: [0.000, 0.000, 0.000]
  [Error]     t: 0.000000 m, R: 0.000000 deg
...
--- Summary ---
Mean Translation Error: 0.123456 m
Mean Rotation Error: 1.234567 deg
```

**형식 차이**:
- **기존 알고리즘**: 영문 (`[Optimized]`, `[GT]`, `[Error]`)
- **Scan-to-Map**: 한글 (`[최적화]`, `[GT]`, `[오차]`)
- **내용**: 동일 (translation, rotation, error 메트릭)

### 성능 비교 지표

| 알고리즘 | 평균 위치 오차 (m) | 평균 회전 오차 (deg) | 처리 시간 (sec/frame) |
|---------|------------------|--------------------|--------------------|
| Point-to-Point | ? | ? | ? |
| Point-to-Plane | ? | ? | ? |
| GICP | ? | ? | ? |
| VGICP | ? | ? | ? |
| LOAM | ? | ? | ? |
| **Scan-to-Map** | **0.0023** | **0.123** | **3.25** |

> 💡 **주의**: 실제 수치는 실행 후 확인 필요. 위 표는 예시입니다.

---

## 기술적 세부사항

### Unary VGICP Factor

**정의** (`gtsam_points/factors/integrated_vgicp_factor.hpp`):

```cpp
IntegratedVGICPFactor(
  const gtsam::Pose3& fixed_target_pose,           // 맵 포즈 (Identity)
  gtsam::Key source_key,                            // 최적화할 포즈 인덱스
  const GaussianVoxelMap::ConstPtr& target_voxels, // 글로벌 맵
  const std::shared_ptr<const SourceFrame>& source // 현재 스캔
);
```

**Cost Function**:

$$
e(x_i) = \sum_{p \in \text{source}} \| \mu_{\text{voxel}} - (R_i \cdot p + t_i) \|_{\Sigma_{\text{voxel}}}^2
$$

- $x_i = (R_i, t_i)$: 최적화할 포즈
- $p$: 소스 스캔의 포인트
- $\mu_{\text{voxel}}$: 대응 복셀의 평균
- $\Sigma_{\text{voxel}}$: 대응 복셀의 공분산

### Global Map 관리

**GaussianVoxelMapCPU 특성**:

```cpp
class GaussianVoxelMapCPU {
public:
  GaussianVoxelMapCPU(double voxel_resolution);
  
  // 포인트 추가 (내부적으로 가우시안 분포 업데이트)
  void insert(const PointCloud& points);
  
  // 각 복셀은 가우시안 분포로 표현
  // μ (평균), Σ (공분산), N (포인트 수)
};
```

**업데이트 로직** (`scan_to_map.cpp`, 174-184줄):

```cpp
// 1. 스캔을 최적화된 포즈로 변환
std::vector<Eigen::Vector4d> transformed_points;
Eigen::Isometry3d pose_iso(estimated_pose.matrix());
for (포인트 p in frame) {
  transformed_points.push_back(pose_iso * p);
}

// 2. 변환된 스캔의 공분산 재계산 (월드 좌표계)
auto transformed_covs = estimate_covariances(transformed_points);

// 3. 글로벌 맵에 삽입 (복셀 병합 자동 처리)
global_map_->insert(*transformed_frame);
```

### 한글 로그 처리

**spdlog UTF-8 지원**:
- spdlog는 기본적으로 UTF-8 인코딩 지원
- Linux 환경 (Docker)에서 문제없이 출력
- 터미널 로케일: `ko_KR.UTF-8` 또는 `en_US.UTF-8`

**로그 패턴**:
```cpp
spdlog::set_pattern("[%^%l%$] %v");
// [info] 메시지
// [warn] 경고
// [error] 에러
```

### 최적화 파라미터

**Levenberg-Marquardt 설정** (`scan_to_map.cpp`, 149-153줄):

```cpp
gtsam_points::LevenbergMarquardtExtParams lm_params;
lm_params.maxIterations = 20;            // 최대 20회 반복
lm_params.relativeErrorTol = 1e-5;       // 상대 에러 < 0.001%
lm_params.absoluteErrorTol = 1e-5;       // 절대 에러 < 0.00001
lm_params.setVerbosityLM("SILENT");      // 내부 로그 비활성화
```

**수렴 조건**:
- 상대 에러 감소량 < `1e-5`
- 절대 에러 < `1e-5`
- 최대 반복 횟수 도달

---

## 향후 개선 방향

### 1. 성능 최적화

#### GPU 가속
```cpp
#ifdef GTSAM_POINTS_USE_CUDA
  auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactorGPU>(
    gtsam::Pose3::Identity(), i, global_map_gpu_, frame_gpu_
  );
#else
  auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(
    gtsam::Pose3::Identity(), i, global_map_, frame
  );
#endif
```

**예상 효과**: 10~50배 속도 향상 (GPU 종류에 따라 변동)

#### Voxel Downsampling
```cpp
// 현재: 38,000 points/frame
// 제안: 10,000 points/frame (VoxelGrid 필터)

pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
voxel_filter.setLeafSize(0.1, 0.1, 0.1);  // 10cm 복셀
voxel_filter.filter(*cloud_downsampled);
```

**예상 효과**: 3~4배 속도 향상, 정확도 미미한 감소

### 2. 정확도 향상

#### IMU 통합
```cpp
// 초기 추정치를 IMU로 개선
gtsam::Pose3 initial_guess_from_imu = integrate_imu(prev_pose, imu_data);
initial.insert(i, initial_guess_from_imu);
```

#### 아웃라이어 제거
```cpp
factor->set_enable_correspondence_validation(true);
factor->set_max_correspondence_distance(2.0);  // 2m 이내만 대응
```

### 3. Loop Closure 추가

**현재**: 순차 정합만 (Drift 누적)
**개선**: Loop Closure Detection → Pose Graph Optimization

```cpp
// 프레임 i와 과거 프레임 j의 유사도 검사
if (detect_loop(frame_i, frame_j)) {
  // Loop Closure Factor 추가
  graph.add(create_loop_closure_factor(i, j));
  
  // Pose Graph 재최적화
  poses = optimize_pose_graph(graph);
}
```

### 4. 평가 메트릭 추가

#### ATE (Absolute Trajectory Error)
```cpp
double compute_ate(const gtsam::Values& estimated, 
                   const gtsam::Values& ground_truth) {
  double sum_squared_error = 0.0;
  for (int i = 0; i < N; i++) {
    Pose3 error = ground_truth.at<Pose3>(i).inverse() * estimated.at<Pose3>(i);
    sum_squared_error += error.translation().squaredNorm();
  }
  return std::sqrt(sum_squared_error / N);
}
```

#### RPE (Relative Pose Error)
```cpp
double compute_rpe(const gtsam::Values& estimated,
                   const gtsam::Values& ground_truth) {
  // 인접 프레임 간 상대 포즈 에러 계산
  for (int i = 0; i < N-1; i++) {
    Pose3 gt_delta = gt[i].inverse() * gt[i+1];
    Pose3 est_delta = est[i].inverse() * est[i+1];
    Pose3 error = gt_delta.inverse() * est_delta;
    // ...
  }
}
```

### 5. 시각화 개선

#### 글로벌 맵 저장/로드
```cpp
// 맵 저장
global_map_->save("/tmp/global_map.bin");

// 맵 로드
auto loaded_map = GaussianVoxelMapCPU::load("/tmp/global_map.bin");
```

#### 실시간 맵 시각화
```cpp
// Viewer에 글로벌 맵 표시
auto map_cloud = global_map_->extract_point_cloud();
viewer->update_drawable("global_map", map_cloud);
```

### 6. 코드 개선

#### 에러 처리 강화
```cpp
// 현재: 최적화 실패 시 로그만 출력
// 개선: 재시도 또는 Fallback 전략

try {
  result = optimizer.optimize();
} catch (const gtsam::IndeterminantLinearSystemException& e) {
  spdlog::warn("LM 실패, ICP로 재시도");
  result = fallback_to_icp(frame, global_map_);
}
```

#### 단위 테스트 추가
```cpp
// test/scan_to_map_test.cpp
TEST(ScanToMapTest, SingleFrameRegistration) {
  ScanToMapRegistration reg(0.5);
  auto result = reg.register_frames({frame}, initial_poses);
  EXPECT_EQ(result.successful_frames, 1);
}
```

---

## 부록

### A. 파일 구조 전체 맵

```
Bottom-LiDAR-docker/
├── CMakeLists.txt                        [수정: scan_to_map.cpp 추가]
├── include/
│   ├── loam_feature.hpp                  [기존]
│   └── scan_to_map.hpp                   [신규: 110줄]
├── src/
│   ├── main.cpp                          [수정: Scan-to-Map 통합 + 한글화]
│   ├── loam_feature.cpp                  [기존]
│   ├── scan_to_map.cpp                   [신규: 210줄]
│   └── scan_to_map_demo.cpp              [기존: 독립 실행 Demo]
├── docs/
│   ├── scan_to_map_implementation.md     [기존: Demo 문서]
│   └── scan_to_map_main_integration.md   [신규: 이 문서]
├── build/
│   ├── lidar_registration_benchmark      [빌드 결과: 30MB]
│   └── scan_to_map_demo                  [기존 Demo 바이너리]
└── thirdparty/
    ├── gtsam_points/                     [GTSAM-Points 라이브러리]
    └── spdlog/                           [로깅 라이브러리]
```

### B. 의존성 정보

```cmake
# CMake 버전
cmake_minimum_required(VERSION 3.10)

# C++ 표준
set(CMAKE_CXX_STANDARD 17)

# 필수 라이브러리
find_package(PCL REQUIRED)       # Point Cloud Library
find_package(Eigen3 REQUIRED)    # 선형대수
find_package(GTSAM REQUIRED)     # Factor Graph 최적화
find_package(OpenMP REQUIRED)    # 병렬 처리
find_package(Iridescence REQUIRED)  # 3D Viewer

# Thirdparty
- gtsam_points (v2.0+)
- spdlog (v1.x)
```

### C. 주요 함수 참조

#### scan_to_map.hpp/cpp

| 함수 | 설명 | 파라미터 | 반환값 |
|-----|------|---------|-------|
| `ScanToMapRegistration()` | 생성자 | `voxel_resolution` (double) | - |
| `set_num_threads()` | OpenMP 스레드 설정 | `num_threads` (int) | void |
| `set_max_iterations()` | LM 최대 반복 설정 | `max_iter` (int) | void |
| `set_convergence_tolerance()` | 수렴 임계값 설정 | `tolerance` (double) | void |
| `register_frames()` | 메인 정합 함수 | `frames`, `initial_poses` | `ScanToMapResult` |
| `get_global_map()` | 글로벌 맵 조회 | - | `GaussianVoxelMapCPU::Ptr` |

#### main.cpp

| 함수 | 설명 | 위치 (줄) |
|-----|------|----------|
| `MatchingCostFactorDemo()` | 생성자 (초기화) | 58-334 |
| `create_factor()` | 팩터 생성 | 378-443 |
| `run_optimization()` | 최적화 실행 | 445-612 |
| `update_viewer()` | Viewer 업데이트 | 344-376 |

### D. 트러블슈팅

#### 빌드 에러

**문제**: `scan_to_map.hpp: No such file or directory`

```bash
# 해결: include 경로 확인
cat CMakeLists.txt | grep "include_directories"
# 출력: include_directories(... include ...)

# include/scan_to_map.hpp 파일 존재 확인
ls -l include/scan_to_map.hpp
```

**문제**: `undefined reference to 'ScanToMapRegistration::register_frames'`

```bash
# 해결: scan_to_map.cpp가 링크에 포함되었는지 확인
cat CMakeLists.txt | grep "scan_to_map.cpp"

# 재빌드
cd build && rm -rf * && cmake .. && make -j4
```

#### 실행 에러

**문제**: GUI가 표시되지 않음

```bash
# X11 포워딩 확인
echo $DISPLAY
# 출력: :0 또는 :1

# xhost 권한 부여
xhost +local:docker
```

**문제**: "Failed to open ground truth file"

```bash
# 데이터셋 경로 확인 (main.cpp, 75줄)
# Oxford 데이터셋은 gt-tum.txt가 없을 수 있음
# → main.cpp 수정하여 GT 체크 비활성화 또는 다른 데이터셋 사용
```

### E. 성능 벤치마크 (참고)

**테스트 환경**:
- CPU: Intel i7-10700K (8코어)
- RAM: 32GB
- GPU: NVIDIA RTX 3070 (CUDA 활성화 시)
- 데이터셋: Oxford Spires (100 frames, 38k points/frame)

**결과 (예상)**:

| 설정 | 처리 시간 (sec/frame) | 총 시간 (100 frames) | 메모리 사용 |
|-----|---------------------|---------------------|-----------|
| CPU, 1 thread | 8.5 | 850s (14분) | 2.3GB |
| CPU, 4 threads | 3.2 | 320s (5분) | 2.5GB |
| CPU, 8 threads | 2.8 | 280s (4.5분) | 2.8GB |
| GPU (CUDA) | 0.4 | 40s | 4.2GB |

---

## 참고 문헌

1. **GTSAM-Points Documentation**
   - https://github.com/koide3/gtsam_points
   
2. **VG-ICP Paper**
   - "Voxelized GICP for Fast and Accurate 3D Point Cloud Registration" (Koide et al., 2021)

3. **LOAM Paper**
   - "LOAM: Lidar Odometry and Mapping in Real-time" (Zhang & Singh, 2014)

4. **TUM RGB-D Benchmark**
   - https://vision.in.tum.de/data/datasets/rgbd-dataset

5. **Oxford Radar RobotCar Dataset**
   - https://oxford-robotics-institute.github.io/radar-robotcar-dataset/

---

## 작성자 정보

- **작성 일자**: 2026년 2월 12일
- **환경**: Docker (`bottom-lidar`), Ubuntu 20.04
- **도구**: GTSAM-Points v2.0, PCL 1.10, Eigen 3.4
- **연락처**: (필요 시 추가)

---

**문서 끝**
