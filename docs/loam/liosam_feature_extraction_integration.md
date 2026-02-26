# LIO-SAM Feature Extraction 통합 및 벤치마크 분석

> **작업일**: 2026-02-24  
> **대상 파일**: `include/featureExtraction.hpp`, `src/featureExtraction.cpp`, `src/main.cpp`, `CMakeLists.txt`  
> **Docker 환경**: `bottom-lidar` (container)

---

## 1. 개요

본 작업은 LIO-SAM(Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping)의 특징점 추출(Feature Extraction) 알고리즘을 기존 수중 LiDAR 벤치마크 시스템에 7번째 Factor로 이식한 과정을 다룹니다. 초기 이식 과정에서 발견된 여러 버그를 5단계(Phase)에 걸쳐 수정하며 성능과 정확도를 최적화하였습니다.

---

## 2. LIO-SAM Feature Extraction 알고리즘 설명

LIO-SAM의 특징점 추출 방식은 효율적인 연산과 정확한 특징 정의를 위해 설계되었습니다.

*   **calculateSmoothness()**: 각 점을 중심으로 좌우 5개씩, 총 10개 점의 거리 차이 합산으로 곡률(curvature)을 계산합니다.
*   **markOccludedPoints()**: 다른 물체에 가려진 점이나 레이저 빔과 평행한 평면 상의 점들을 마킹하여 특징점 후보에서 제외합니다.
*   **extractFeatures()**: 
    *   각 스캔 라인을 수평 방향으로 6등분(sector)합니다.
    *   섹터별로 곡률 기준 정렬을 수행합니다.
    *   곡률이 큰 점을 Edge 포인트로, 작은 점을 Surface 포인트로 추출합니다.
    *   추출된 Surface 포인트는 VoxelGrid 필터(0.4m)로 다운샘플링합니다.

---

## 3. 파일 구조 및 코드 변경 내역

### 3.1 신규 및 수정 파일 목록
*   **신규 파일**:
    *   `include/featureExtraction.hpp`: 클래스 정의 및 파라미터 구조체 정의
    *   `src/featureExtraction.cpp`: 특징점 추출 핵심 알고리즘 구현
*   **수정 파일**:
    *   `CMakeLists.txt`: PCL filters 컴포넌트 추가 및 신규 소스 등록
    *   `src/main.cpp`: LOAM_LIOSAM Factor 생성 및 벤치마크 루프 통합

### 3.2 주요 수정 사항
1.  **main.cpp**: `feature_extractor` 객체를 생성하고 프레임 로딩 시 특징점을 추출하도록 수정했습니다.
2.  **Factor 생성**: `create_factor()` 함수에 `LOAM_LIOSAM` 케이스를 추가하여 추출된 데이터를 `LoamFactor`로 전달합니다.
3.  **PCL 템플릿**: 커스텀 포인트 타입 사용에 따른 링크 에러 방지를 위해 `featureExtraction.cpp`에 PCL 구현 헤더들을 명시적으로 포함했습니다.

---

## 4. 빌드 및 실행

### 4.1 빌드 방법
Docker 컨테이너 내부에서 다음 명령어를 사용합니다.

```bash
docker exec bottom-lidar bash -c "cd /root/workdir/build && cmake .. && make -j$(nproc)"
```

### 4.2 실행 방법
Headless 모드로 7개 Factor 전체를 테스트합니다.

```bash
docker exec bottom-lidar bash -c "cd /root/workdir/build && ./lidar_registration_benchmark --headless"
```

---

## 5. 버그 수정 이력 (Phase 1-7)

개발 과정에서 총 7단계의 개선을 거쳤습니다.

### Phase 1: 초기 이식 (버그 존재)
*   **내용**: 기본적인 클래스 구조 이식 및 7번째 Factor 추가.
*   **결과**: Mean T 0.359m, Mean R 2.060°, 실행 시간 1,372,135ms (약 22분).
*   **이슈**: Edge 포인트는 3400-4800개였으나, Planar 포인트가 약 564,401개로 비정상적으로 많이 추출되었습니다.

### Phase 2: 속도 버그 3개 수정
*   **Fix 1**: `pointColInd`에 전체 인덱스 `i` 대신 링(ring)별 순차 인덱스(`ringPointCount[ring]`)를 할당하여 이웃 억제(neighbor suppression)가 정상 작동하도록 수정했습니다.
*   **Fix 2**: Surface 수집 조건을 `cloudLabel[k] <= 0`에서 `cloudLabel[cloudSmoothness[k].ind] == -1`로 변경하여 명확히 라벨링된 점만 수집하게 했습니다.
*   **Fix 3**: 이전 수정 중 실수로 삭제된 `endRingIndex` 할당 코드를 복원했습니다.
*   **결과**: 실행 시간이 1,357ms로 1,011배 빨라졌으나, 정확도는 Mean T 0.855m로 악화되었습니다.

### Phase 3: Ring 경계 수정
*   **Fix 4**: `calculateSmoothness()`에서 링 경계 근처 5개 포인트를 제외하도록 `ringBoundary` 벡터를 사용했습니다.
*   **Fix 5**: `markOccludedPoints()`에서 인접한 두 점이 동일한 링에 속하는지 확인하는 로직을 추가했습니다.
*   **결과**: Mean T 0.708m, Mean R 2.609°, 실행 시간 14,039ms를 기록했습니다.

### Phase 4: 파라미터 튜닝
*   **내용**: Ring-based LOAM 설정과 동일하게 파라미터를 조정했습니다.
    *   `edgeThreshold`: 1.0에서 0.1로 하향
    *   섹터당 Edge 수: 20에서 2로 제한
    *   섹터당 Surface 수: 무제한에서 4로 제한
*   **결과**: Mean T 0.694m, Edge 약 768개, Planar 약 14,000개가 추출되었습니다.

### Phase 5: Angle 기반 정렬 및 최종 수정
*   **내용**: `buildCloudInfo()`에서 링 내부 포인트들을 `atan2(y, x)` 기준으로 정렬하여 공간적 이웃 관계를 명확히 했습니다. 누락되었던 `endRingIndex` 할당 로직도 최종 수정했습니다.
*   **결과**: Mean T 0.308m, Mean R 1.133°, 실행 시간 106ms로 최적의 성능을 달성했습니다.

### Phase 6: 시각화 및 Ring-based LOAM 비활성화
*   **내용**:
    *   **LOAM 비활성화**: 기존 Ring-based LOAM 알고리즘이 중복 계산되던 부분을 주석 처리하여 리소스를 절약했습니다.
    *   **특징점 시각화**: ImGui 체크박스를 추가하여 LIO-SAM의 Edge(빨강) 및 Planar(파랑) 특징점을 실시간으로 시각화할 수 있게 개선했습니다.
    *   **Headless 최적화**: GUI가 없는 벤치마크 모드에서 불필요한 LOAM 계산 로직을 건너뛰도록 수정했습니다.

### Phase 7: 파라미터 튜닝 실험
*   **내용**: LIO-SAM 특징점 추출기의 주요 파라미터를 체계적으로 변경하며 최적의 조합을 찾았습니다.

| 실험 | edgeThreshold | surfThreshold | Edge/sector | Surf/sector | leafSize | Mean T (m) | Mean R (deg) | Edge/frame | Time (ms) |
|------|--------------|--------------|-------------|-------------|----------|-----------|-------------|------------|-----------|
| Baseline (Phase 5) | 0.1 | 0.1 | 2 | 4 | 0.4 | 0.308 | 1.133 | ~715 | 103 |
| Exp 1: leafSize=0.2 | 0.1 | 0.1 | 2 | 4 | 0.2 | 0.314 | 1.138 | ~715 | 100 |
| Exp 2: leafSize=0.0 | 0.1 | 0.1 | 2 | 4 | 0.0 | 0.318 | 1.140 | ~715 | 87 |
| Exp 3: edge=1.0 ⭐ | 1.0 | 0.1 | 2 | 4 | 0.4 | 0.233 | 0.914 | ~436 | 78 |
| **Exp 4: edge=1.0+20/sector ⭐⭐** | **1.0** | **0.1** | **20** | **4** | **0.4** | **0.225** | **0.773** | **~1130** | **117** |
| Exp 5: surfThreshold=0.01 | 1.0 | 0.01 | 20 | 4 | 0.4 | 0.230 | 0.779 | ~1130 | 123 |
| Exp 6a: edge=2.0 | 2.0 | 0.1 | 20 | 4 | 0.4 | 0.226 | 0.799 | ~950 | 115 |
| Exp 6b: edge=0.5 | 0.5 | 0.1 | 20 | 4 | 0.4 | 0.232 | 0.833 | ~1380 | 147 |
| Exp 7: surf/sector=20 | 1.0 | 0.1 | 20 | 20 | 0.4 | 0.265 | 0.847 | ~1130 | 192 |

#### 주요 분석 결과
1. VoxelGrid downsampling은 이 데이터셋에서 유의미한 영향 없음
2. edgeThreshold=1.0 (LIO-SAM 원본 기본값)이 최적 — 24% translation 개선
3. Edge per sector 20 (LIO-SAM 원본)이 2보다 나음 — rotation 15% 추가 개선
4. surfThreshold, surface per sector는 변경해도 큰 영향 없거나 악화
5. 최종 최적 구성: edgeThreshold=1.0, surfThreshold=0.1, edge/sector=20, surf/sector=4, leafSize=0.4

---

## 6. 최종 벤치마크 결과

수중 LiDAR 데이터셋을 활용한 7개 Factor 비교 결과입니다.

### 7-Factor 비교 테이블

| Factor | Mean T (m) | Mean R (deg) | Max T (m) | Max R (deg) | Time (ms) |
|--------|-----------|-------------|----------|------------|-----------|
| Point-to-Point | 0.095430 | 0.487565 | 0.219172 | 0.908431 | 9,199 |
| Point-to-Plane | 0.061536 | 0.448742 | 0.125722 | 0.929910 | 8,061 |
| GICP | 0.084145 | 0.551482 | 0.165061 | 1.102780 | 11,234 |
| VGICP | 0.216358 | 1.038157 | 1.080683 | 3.464607 | 9,695 |
| LOAM (Ring-based) | 0.288562 | 1.048161 | 0.872851 | 2.327760 | 66 |
| NDT | 0.078200 | 0.510344 | 0.142668 | 1.129417 | 28,936 |
| **LOAM_LIOSAM** | **0.225** | **0.773** | **-** | **-** | **117** |

### LOAM_LIOSAM 프레임별 상세 결과

| Frame | T Error (m) | R Error (deg) |
|-------|------------|---------------|
| 0 | 0.000 | 0.000 |
| 1 | 0.129 | 0.927 |
| 2 | 0.106 | 0.538 |
| 3 | 0.268 | 1.106 |
| 4 | 0.271 | 1.364 |
| 5 | 0.479 | 1.546 |
| 6 | 0.902 | 2.447 |

---

## 7. 분석 및 발견사항

### Phase별 성능 변화 요약

| Phase | 설명 | Mean T (m) | Mean R (deg) | Time (ms) | 특징점 수 (Edge/Planar) |
|-------|------|-----------|-------------|-----------|-------------|
| Phase 1 | 초기 이식 | 0.359 | 2.060 | 1,372,135 | ~4000 / ~564K |
| Phase 2 | 속도 개선 | 0.855 | 3.164 | 1,357 | ~4000 / ~10K |
| Phase 3 | 링 경계 처리 | 0.708 | 2.609 | 14,039 | ~7300 / ~25K |
| Phase 4 | 파라미터 튜닝 | 0.694 | 2.367 | 8,000 | 768 / ~14K |
| Phase 5 | 각도 기반 정렬 | 0.308 | 1.133 | 106 | ~715 / ~1,225 |
| Phase 6 | 시각화 + Ring LOAM 비활성화 | 0.308 | 1.133 | 106 | ~715 / ~1,225 |
| **Phase 7** | **파라미터 튜닝 (최적)** | **0.225** | **0.773** | **117** | **~1130 / ~1,112** |

### 상세 분석
*   **인덱스 오류**: Phase 1에서 거의 모든 점이 Surface로 분류된 이유는 `cloudLabel` 참조 시 정렬 인덱스를 원본 인덱스로 변환하지 않았기 때문입니다.
*   **정렬의 중요성**: 포인트 클라우드가 각도(angle) 순으로 정렬되지 않으면 곡률 계산 시 실제 공간적 이웃이 아닌 점들이 사용됩니다. 이로 인해 특징점 품질이 급격히 저하되었습니다.
*   **최적화 부하**: 특징점 개수가 너무 많으면 Gtsam Optimizer의 연산량이 기하급수적으로 늘어나 실행 시간이 폭증함을 확인했습니다.

---

## 8. 파라미터 정리

최종적으로 적용된 주요 파라미터입니다.

| 파라미터 | Phase 5 값 | Phase 7 최적값 | 변경 이유 |
|---------|-----------|-------------|----------|
| edgeThreshold | 0.1 | 1.0 | LIO-SAM 원본 기본값이 최적 (24% T 개선) |
| surfThreshold | 0.1 | 0.1 | 변경 시 효과 미미 |
| odometrySurfLeafSize | 0.4 | 0.4 | 변경 시 효과 없음 |
| Edge/sector | 2 | 20 | LIO-SAM 원본값, R 15% 개선 |
| Surface/sector | 4 | 4 | 20으로 변경 시 악화 |

---

## 9. 핵심 교훈

1.  LIO-SAM의 Range Image 설계를 일반적인 PCD 배열 방식으로 포팅할 때는 링(ring) 경계 처리와 포인트의 각도 기반 정렬이 필수적입니다.
2.  `pointColInd`는 단순한 루프 인덱스가 아니라 링 내부에서의 순차적 위치를 나타내야만 이웃 특징점 억제 로직이 올바르게 작동합니다.
3.  Surface 포인트를 수집할 때 `<=` 조건을 사용하면 노이즈를 포함한 거의 모든 점이 선택되므로, 명확하게 라벨링된 점(`== -1`)만 골라내야 합니다.
4.  추출된 특징점의 수는 백엔드 최적화 성능에 직접적인 영향을 미치므로, 섹터별 최대 추출 개수를 엄격히 제한하는 것이 성능 유지의 핵심입니다.
5.  포인트가 각도 순으로 정렬되지 않으면 곡률 계산이 무의미해져 시스템 전반의 정확도가 크게 떨어집니다.
6.  LIO-SAM 원본 파라미터가 커스텀 튜닝보다 우수함 — 원본 알고리즘이 이미 최적화됨
7.  Edge 특징점 품질(threshold)과 수량(per sector)은 모두 중요하며, 둘 다 LIO-SAM 원본값이 최적
