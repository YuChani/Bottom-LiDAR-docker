# Documentation

이 폴더에는 GTSAM-Points 데모 프로젝트의 문서가 포함되어 있습니다.

## 문서 목록

### 📘 [hyperparameters_tutorial.md](./hyperparameters_tutorial.md) (한글)
**완전한 하이퍼파라미터 가이드**

main.cpp에서 사용되는 모든 하이퍼파라미터에 대한 상세한 설명입니다.

**내용**:
- 데이터 구성 (PCD 파일, Ground Truth)
- 센서 외부 캘리브레이션 (T_base_lidar)
- 점군 전처리 (Covariance, Normal, Voxelmap)
- ICP 알고리즘 파라미터 (ICP, ICP_PLANE, GICP, VGICP)
- 최적화 알고리즘 (LM, iSAM2)
- Factor Graph 구성
- 시각화 및 UI 설정
- 로깅 설정 (spdlog)
- 튜닝 가이드
- 현재 설정 요약

---

### 📗 [hyperparameters_tutorial_en.md](./hyperparameters_tutorial_en.md) (English)
**Complete Hyperparameter Guide (English Version)**

Comprehensive documentation of all hyperparameters used in main.cpp.

**Contents**:
- Data Configuration (PCD files, Ground Truth)
- Extrinsic Calibration (T_base_lidar)
- Point Cloud Preprocessing (Covariance, Normal, Voxelmap)
- ICP Algorithm Parameters (ICP, ICP_PLANE, GICP, VGICP)
- Optimization Algorithms (LM, iSAM2)
- Factor Graph Construction
- Visualization and UI Settings
- Logging Configuration (spdlog)
- Tuning Guide
- Current Configuration Summary

---

### 📙 [quick_reference.md](./quick_reference.md)
**빠른 참조 가이드 (Quick Reference)**

핵심 하이퍼파라미터를 한눈에 볼 수 있는 간단한 치트시트입니다.

**내용**:
- 주요 설정값 표
- 빠른 튜닝 가이드 (정확도 vs 속도)
- 알고리즘 선택 가이드
- 코드 라인 참조
- 실행 방법

---

## 사용 방법

### 처음 시작하는 경우
1. **quick_reference.md**부터 읽어보세요 (빠른 시작)
2. 상세한 내용은 **hyperparameters_tutorial.md** 참조

### 특정 파라미터 찾기
- 목차를 활용하여 원하는 섹션으로 이동
- 표 형태로 정리되어 검색이 쉽습니다

### 튜닝이 필요한 경우
- **섹션 9: 하이퍼파라미터 튜닝 가이드** 참조
- 정확도 향상 vs 속도 향상 트레이드오프 확인

---

## 관련 파일

```
/root/workdir/
├── src/
│   └── main.cpp                          # 메인 데모 코드
├── data/
│   └── pcd/
│       ├── *.pcd                         # 점군 데이터
│       └── gt-tum.txt                    # Ground Truth 포즈
├── include/
│   └── gtsam_points/
│       └── include/gtsam_points/util/
│           └── read_points.hpp           # PCD 파서
└── docs/
    ├── README.md                         # 이 파일
    ├── hyperparameters_tutorial.md       # 상세 가이드 (한글)
    ├── hyperparameters_tutorial_en.md    # 상세 가이드 (English)
    └── quick_reference.md                # 빠른 참조
```

---

## 실행 예제

```bash
# 빌드
cd /root/workdir
./run_build.sh

# 실행
cd build
./calculate_rt
```

**UI 사용법**:
- `noise_scale`: 슬라이더로 노이즈 크기 조정
- `add noise`: 버튼으로 초기 포즈에 노이즈 추가
- `full connection`: 체크박스로 전체/인접 연결 선택
- `num threads`: 병렬 스레드 수 조정
- `factor type`: ICP/ICP_PLANE/GICP/VGICP 선택
- `optimizer type`: LM/ISAM2 선택
- `optimize`: 최적화 실행

---

## 주요 하이퍼파라미터 요약

### 전처리
- **voxel_resolution**: 0.5m (VGICP 복셀 크기)
- **k_neighbors**: 10 (covariance/normal 추정 이웃 수)

### ICP
- **correspondence_update_tolerance**: 0.0 (매 iteration 업데이트)
- **num_threads**: 1 (단일 스레드)
- **max_correspondence_distance**: 1.0m

### 최적화 (LM)
- **maxIterations**: 100
- **relativeErrorTol**: 1e-5
- **absoluteErrorTol**: 1e-5

### Factor Graph
- **prior_precision**: 1e6 (첫 프레임 고정)
- **full_connection**: true (모든 프레임 쌍 연결)

---

## 기여 및 수정

문서 수정이 필요한 경우:
1. 해당 `.md` 파일 직접 수정
2. 변경 이력 섹션에 날짜와 내용 추가
3. 코드 라인 번호가 변경된 경우 업데이트

---

## 참고 자료

- **gtsam_points**: https://github.com/koide3/gtsam_points
- **GTSAM**: https://gtsam.org/
- **Iridescence**: https://github.com/koide3/iridescence
- **spdlog**: https://github.com/gabime/spdlog

---

생성일: 2026-01-26  
최종 수정: 2026-01-26
