# GTSAM-Points 하이퍼파라미터 튜토리얼

## 개요
이 문서는 `gtsam_points` 라이브러리를 사용한 점군 정합(Point Cloud Registration) 데모 코드에서 사용되는 모든 하이퍼파라미터를 정리하고 설명합니다.

**데모 프로그램**: `/root/workdir/src/main.cpp`  
**목적**: ICP, GICP, VGICP 알고리즘 비교 및 Ground Truth와의 정합 정확도 평가

---

## 1. 데이터 설정

### 1.1 입력 데이터
```cpp
const std::string data_path = "/root/workdir/data/pcd";
```

| 항목 | 값 | 설명 |
|------|-----|------|
| **PCD 파일 개수** | 5개 | 연속된 LiDAR 프레임 |
| **점군 포인트 수** | ~54,400개/프레임 | 각 PCD 파일당 |
| **파일 포맷** | Binary PCD | PCL 없이 커스텀 파서 사용 |
| **포인트 크기** | 48 bytes | x,y,z + intensity + timestamp + ring 등 |
| **좌표계** | LiDAR local frame | 센서 기준 좌표 |

### 1.2 Ground Truth 데이터
```cpp
// gt-tum.txt: timestamp tx ty tz qx qy qz qw
std::map<double, gtsam::Pose3> gt_poses;
```

| 항목 | 값 | 설명 |
|------|-----|------|
| **GT 포즈 개수** | 12,270개 | TUM 포맷 |
| **좌표계** | World ← Base | 로봇 베이스 기준 |
| **매칭 방식** | Timestamp 기반 | PCD 파일명 → 가장 가까운 GT 포즈 |

---

## 2. 센서 외부 캘리브레이션 (Extrinsic Calibration)

### 2.1 T_base_lidar 변환
```cpp
// sensor.yaml에서 가져온 값
Eigen::Vector3d t_base_lidar(0.0, 0.0, 0.124);
Eigen::Quaterniond q_base_lidar(0.0, 0.0, 0.0, 1.0);  // w,x,y,z
gtsam::Pose3 T_base_lidar(gtsam::Rot3(q_base_lidar), t_base_lidar);
```

| 파라미터 | 값 | 설명 |
|----------|-----|------|
| **Translation (x, y, z)** | `[0.0, 0.0, 0.124]` m | 베이스 → LiDAR 이동 |
| **Rotation (quaternion)** | `[w:0, x:0, y:0, z:1]` | 180° yaw 회전 |
| **Rotation (ypr)** | `[180°, 0°, 0°]` | Yaw-Pitch-Roll |

### 2.2 좌표 변환 체인
```cpp
W_T_L = W_T_B * T_base_lidar;           // World ← LiDAR
L0_T_Li = W_T_L_origin.inverse() * W_T_L;  // 첫 프레임 기준 상대 포즈
```

| 변환 | 의미 | 용도 |
|------|------|------|
| **W_T_B** | World ← Base | gt-tum.txt의 포즈 |
| **T_base_lidar** | Base ← LiDAR | 센서 외부 캘리브레이션 |
| **W_T_L** | World ← LiDAR | LiDAR 절대 포즈 |
| **L0_T_Li** | LiDAR0 ← LiDARi | 첫 프레임 기준 상대 포즈 (최적화에 사용) |

---

## 3. 점군 전처리 (Point Cloud Preprocessing)

### 3.1 공분산 추정 (Covariance Estimation)
```cpp
auto covs = gtsam_points::estimate_covariances(points);
```

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| **k_neighbors** | 10 | KNN 탐색 이웃 개수 |
| **알고리즘** | PCA 기반 | 각 점의 로컬 표면 특성 추정 |

**목적**: GICP에서 사용하는 각 점의 불확실성(covariance) 계산

### 3.2 법선 추정 (Normal Estimation)
```cpp
frame->add_normals(gtsam_points::estimate_normals(frame->points, frame->size()));
```

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| **k_neighbors** | 10 | KNN 탐색 이웃 개수 |
| **방법** | PCA 최소 고유벡터 | 로컬 평면 법선 방향 |

**목적**: Point-to-Plane ICP에서 사용하는 표면 법선 벡터 계산

### 3.3 복셀맵 생성 (Voxel Map)
```cpp
auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(0.5);
voxelmap->insert(*frame);
```

| 파라미터 | 값 | 설명 |
|----------|-----|------|
| **Voxel Resolution** | `0.5` m | 복셀 한 변의 길이 |
| **데이터 구조** | Gaussian Voxel | 각 복셀에 평균, 공분산 저장 |
| **사용 알고리즘** | VGICP | 빠른 대응점 탐색 |

---

## 4. ICP 알고리즘 파라미터

### 4.1 지원 알고리즘 종류
```cpp
factor_types = {"ICP", "ICP_PLANE", "GICP", "VGICP", "VGICP_GPU"};
```

| 알고리즘 | 설명 | 특징 |
|----------|------|------|
| **ICP** | Point-to-Point | 가장 기본적인 방법 |
| **ICP_PLANE** | Point-to-Plane | 법선 벡터 활용, 평면에 강건 |
| **GICP** | Generalized ICP | 점의 공분산 활용, 정확도 높음 |
| **VGICP** | Voxelized GICP | 복셀맵 활용, 속도 빠름 |
| **VGICP_GPU** | GPU 가속 VGICP | CUDA 필요 (현재 비활성화) |

### 4.2 ICP/ICP_PLANE/GICP 파라미터
```cpp
auto factor = gtsam::make_shared<gtsam_points::IntegratedICPFactor>(...);
factor->set_correspondence_update_tolerance(correspondence_update_tolerance_rot, 
                                             correspondence_update_tolerance_trans);
factor->set_num_threads(num_threads);
```

| 파라미터 | 기본값 | 범위 | 설명 |
|----------|--------|------|------|
| **correspondence_update_tolerance_rot** | `0.0` rad | 0.0 - 0.1 | 회전 변화량 임계값 (0 = 매 iteration 업데이트) |
| **correspondence_update_tolerance_trans** | `0.0` m | 0.0 - 1.0 | 이동 변화량 임계값 (0 = 매 iteration 업데이트) |
| **num_threads** | `1` | 1 - 128 | 병렬 처리 스레드 수 |
| **max_correspondence_distance** | `1.0` m | (기본값) | 대응점 최대 거리 |

**correspondence_update_tolerance 설명**:
- 값이 `0.0`이면: 매 iteration마다 대응점 재계산 (정확하지만 느림)
- 값이 클수록: 포즈 변화가 클 때만 재계산 (빠르지만 덜 정확)

### 4.3 VGICP 전용 파라미터
```cpp
auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(
    target_key, source_key, target_voxelmap, source);
factor->set_num_threads(num_threads);
```

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| **Voxel Resolution** | `0.5` m | 복셀맵 해상도 (작을수록 정밀) |
| **num_threads** | `1` | 병렬 처리 스레드 수 |

**참고**: VGICP는 복셀맵을 사용하므로 correspondence_update_tolerance 미지원

---

## 5. 최적화 알고리즘 파라미터

### 5.1 지원 옵티마이저
```cpp
optimizer_types = {"LM", "ISAM2"};
```

### 5.2 Levenberg-Marquardt (LM) 파라미터
```cpp
gtsam_points::LevenbergMarquardtExtParams lm_params;
lm_params.maxIterations = 100;
lm_params.relativeErrorTol = 1e-5;
lm_params.absoluteErrorTol = 1e-5;
```

| 파라미터 | 값 | 설명 |
|----------|-----|------|
| **maxIterations** | `100` | 최대 반복 횟수 |
| **relativeErrorTol** | `1e-5` | 상대 오차 수렴 조건 |
| **absoluteErrorTol** | `1e-5` | 절대 오차 수렴 조건 |

**수렴 조건**:
- Relative Error < `1e-5` 또는
- Absolute Error < `1e-5` 또는
- Iterations ≥ `100`

### 5.3 iSAM2 파라미터
```cpp
gtsam::ISAM2Params isam2_params;
isam2_params.relinearizeSkip = 1;
isam2_params.setRelinearizeThreshold(0.0);
```

| 파라미터 | 값 | 설명 |
|----------|-----|------|
| **relinearizeSkip** | `1` | 재선형화 주기 (1 = 매 update) |
| **relinearizeThreshold** | `0.0` | 재선형화 임계값 (0 = 항상 재선형화) |
| **추가 iterations** | `5` | update 후 추가 최적화 횟수 |

---

## 6. Factor Graph 구성

### 6.1 Prior Factor
```cpp
graph.add(gtsam::PriorFactor<gtsam::Pose3>(
    0, poses.at<gtsam::Pose3>(0), 
    gtsam::noiseModel::Isotropic::Precision(6, 1e6)));
```

| 파라미터 | 값 | 설명 |
|----------|-----|------|
| **Key** | `0` | 첫 번째 프레임 고정 |
| **Precision** | `1e6` | 노이즈 정밀도 (매우 강한 제약) |
| **Dimension** | `6` | SE(3) 자유도 (3 translation + 3 rotation) |

### 6.2 Registration Factor 연결 방식
```cpp
bool full_connection = true;
int j_end = full_connection ? 5 : std::min(i + 2, 5);
```

| 모드 | 설명 | Factor 개수 (5 프레임) |
|------|------|------------------------|
| **full_connection = true** | 모든 프레임 쌍 연결 | 10개 (5C2) |
| **full_connection = false** | 인접 프레임만 연결 | 4개 (연속) |

**예시 (full_connection = true)**:
```
Frame 0 ↔ Frame 1, 2, 3, 4  (4개)
Frame 1 ↔ Frame 2, 3, 4     (3개)
Frame 2 ↔ Frame 3, 4        (2개)
Frame 3 ↔ Frame 4           (1개)
총 10개 factors
```

---

## 7. 시각화 및 UI 파라미터

### 7.1 포즈 노이즈 추가
```cpp
float pose_noise_scale = 0.1;
```

| 파라미터 | 기본값 | 범위 | 설명 |
|----------|--------|------|------|
| **pose_noise_scale** | `0.1` | 0.0 - 10.0 | 초기 포즈에 추가할 랜덤 노이즈 크기 (se(3) tangent space) |

**용도**: 최적화 알고리즘의 수렴 성능 테스트

### 7.2 3D Viewer 설정
```cpp
viewer->enable_vsync();
auto drawable = guik::Rainbow().add("model_matrix", pose);
```

| 설정 | 값 | 설명 |
|------|-----|------|
| **VSync** | Enabled | 화면 주사율 동기화 |
| **좌표계 크기** | 5.0 m | 각 프레임의 coordinate system 표시 크기 |
| **Factor 선 색상** | Green (0, 1, 0) | Factor graph 연결선 |

---

## 8. 로깅 설정 (spdlog)

### 8.1 Logger 초기화
```cpp
auto console = spdlog::stdout_color_mt("demo");
spdlog::set_level(spdlog::level::info);
spdlog::set_pattern("[%^%l%$] %v");
```

| 파라미터 | 값 | 설명 |
|----------|-----|------|
| **Log Level** | `info` | debug, info, warn, error 중 info 이상만 출력 |
| **Pattern** | `[%^%l%$] %v` | 색상 + 레벨 + 메시지 |
| **Output** | stdout (colored) | 터미널 컬러 출력 |

### 8.2 출력 정보
- PCD 파일 로딩 상태
- Timestamp 매칭 결과
- 각 프레임의 상대 포즈 (translation)
- 최적화 결과 (Optimized vs GT)
- 포즈 오차 (translation error, rotation error)
- 평균 통계 (Mean Translation Error, Mean Rotation Error)

---

## 9. 하이퍼파라미터 튜닝 가이드

### 9.1 정확도 향상
| 파라미터 | 조정 방향 | 효과 |
|----------|----------|------|
| **Voxel Resolution** | 감소 (0.5 → 0.2) | 더 세밀한 매칭, 계산 시간 증가 |
| **k_neighbors** | 증가 (10 → 20) | 더 안정적인 covariance/normal 추정 |
| **maxIterations** | 증가 (100 → 200) | 더 정확한 수렴 |
| **full_connection** | true | 모든 프레임 쌍 최적화, 전역 일관성 향상 |

### 9.2 속도 향상
| 파라미터 | 조정 방향 | 효과 |
|----------|----------|------|
| **Voxel Resolution** | 증가 (0.5 → 1.0) | 빠른 처리, 정확도 감소 |
| **num_threads** | 증가 (1 → 8) | 병렬 처리로 속도 향상 |
| **correspondence_update_tolerance** | 증가 (0.0 → 0.01) | 대응점 재계산 빈도 감소 |
| **full_connection** | false | 인접 프레임만 연결, Factor 개수 감소 |

### 9.3 알고리즘 선택 가이드
| 상황 | 추천 알고리즘 | 이유 |
|------|--------------|------|
| **실시간 처리** | VGICP | 복셀맵 활용으로 빠름 |
| **최고 정확도** | GICP | 점별 공분산 활용 |
| **평면 환경** | ICP_PLANE | 법선 벡터로 평면 최적화 |
| **단순 테스트** | ICP | 가장 기본적, 빠른 실행 |

---

## 10. 현재 설정 요약

### 10.1 기본 파라미터 (main.cpp)
```yaml
Data:
  frames: 5
  points_per_frame: ~54400
  
Calibration:
  T_base_lidar:
    translation: [0.0, 0.0, 0.124]
    rotation_ypr: [180°, 0°, 0°]

Preprocessing:
  k_neighbors_covariance: 10
  k_neighbors_normal: 10
  voxel_resolution: 0.5 m

ICP_Factors:
  correspondence_update_tolerance_rot: 0.0 rad
  correspondence_update_tolerance_trans: 0.0 m
  max_correspondence_distance: 1.0 m
  num_threads: 1

Optimization:
  optimizer: LM
  maxIterations: 100
  relativeErrorTol: 1e-5
  absoluteErrorTol: 1e-5

Graph:
  full_connection: true
  prior_precision: 1e6

Visualization:
  pose_noise_scale: 0.1
  coordinate_system_size: 5.0 m

Logging:
  level: info
  color_output: enabled
```

### 10.2 실행 방법
```bash
cd /root/workdir/build
./calculate_rt
```

**UI 컨트롤**:
1. `noise_scale` 슬라이더: 초기 포즈 노이즈 조정
2. `add noise` 버튼: 노이즈 추가
3. `full connection` 체크박스: Factor graph 연결 방식
4. `num threads` 슬라이더: 병렬 처리 스레드 수
5. `factor type` 드롭다운: ICP/ICP_PLANE/GICP/VGICP 선택
6. `optimizer type` 드롭다운: LM/ISAM2 선택
7. `corr update tolerance` 슬라이더: 대응점 업데이트 임계값
8. `optimize` 버튼: 최적화 실행

---

## 11. 참고 자료

### 11.1 코드 위치
- **메인 코드**: `/root/workdir/src/main.cpp`
- **PCD 파서**: `/root/workdir/include/gtsam_points/include/gtsam_points/util/read_points.hpp`
- **데이터**: `/root/workdir/data/pcd/`

### 11.2 라이브러리 문서
- **gtsam_points**: https://github.com/koide3/gtsam_points
- **GTSAM**: https://gtsam.org/
- **Iridescence**: https://github.com/koide3/iridescence

### 11.3 논문 참고
- **ICP**: Besl & McKay, 1992
- **Point-to-Plane ICP**: Chen & Medioni, 1992
- **GICP**: Segal et al., 2009
- **VGICP**: Koide et al., 2021

---

## 변경 이력
- **2026-01-26**: 초기 문서 작성 (main.cpp 기준)
