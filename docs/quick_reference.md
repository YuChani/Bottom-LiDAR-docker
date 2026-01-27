# 하이퍼파라미터 빠른 참조 (Quick Reference)

## 주요 설정값 요약

### 데이터 구조
- **프레임 수**: 5개
- **포인트/프레임**: ~54,400개
- **복셀 해상도**: 0.5m

### 센서 캘리브레이션 (T_base_lidar)
```cpp
translation: [0.0, 0.0, 0.124] m
rotation: [180°, 0°, 0°] (ypr)
```

### 전처리
- **k_neighbors (covariance)**: 10
- **k_neighbors (normals)**: 10

### ICP 알고리즘
| 파라미터 | 값 | 위치 |
|---------|-----|------|
| correspondence_update_tolerance_rot | 0.0 rad | line 241 |
| correspondence_update_tolerance_trans | 0.0 m | line 242 |
| num_threads | 1 | line 237 |

### 최적화 (LM)
| 파라미터 | 값 | 위치 |
|---------|-----|------|
| maxIterations | 100 | line 387 |
| relativeErrorTol | 1e-5 | line 388 |
| absoluteErrorTol | 1e-5 | line 389 |

### Factor Graph
- **Prior Precision**: 1e6 (line 371)
- **Full Connection**: true (line 237)

### UI 기본값
- **pose_noise_scale**: 0.1 (line 227)

---

## 빠른 튜닝 가이드

### 정확도 우선
```cpp
voxel_resolution = 0.2;        // 더 세밀
k_neighbors = 20;              // 더 안정적
maxIterations = 200;           // 더 정확
full_connection = true;        // 전역 최적화
```

### 속도 우선
```cpp
voxel_resolution = 1.0;        // 더 빠름
num_threads = 8;               // 병렬 처리
correspondence_update_tolerance_trans = 0.01;  // 업데이트 감소
full_connection = false;       // 인접만 연결
```

### 알고리즘 선택
- **실시간**: VGICP
- **최고 정확도**: GICP
- **평면 환경**: ICP_PLANE
- **테스트**: ICP

---

## 파일 위치
- **한글 튜토리얼**: `/root/workdir/docs/hyperparameters_tutorial.md`
- **English Tutorial**: `/root/workdir/docs/hyperparameters_tutorial_en.md`
- **소스 코드**: `/root/workdir/src/main.cpp`

---

## 실행 방법
```bash
cd /root/workdir/build
./calculate_rt

# 또는 빌드부터
cd /root/workdir
./run_build.sh
cd build
./calculate_rt
```

---

## 주요 코드 라인 참조

| 설정 | 라인 | 코드 |
|------|------|------|
| T_base_lidar | 62-64 | `Eigen::Vector3d t_base_lidar(0.0, 0.0, 0.124)` |
| Voxel 생성 | 197 | `GaussianVoxelMapCPU(0.5)` |
| Covariance 추정 | 183 | `estimate_covariances(points)` |
| Normal 추정 | 192 | `estimate_normals(frame->points, frame->size())` |
| LM 파라미터 | 387-389 | `maxIterations`, `relativeErrorTol`, `absoluteErrorTol` |
| Prior Factor | 371 | `gtsam::noiseModel::Isotropic::Precision(6, 1e6)` |

---

생성일: 2026-01-26
