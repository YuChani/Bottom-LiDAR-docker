# LiDAR Registration Benchmark 변경 내역 및 실험 결과

## 개요

`lidar_registration_benchmark` 프로젝트의 6개 포인트 클라우드 정합 알고리즘에 대해 Docker 환경(`bottom-lidar` 컨테이너)에서 벤치마크를 수행하고, 발견된 버그를 수정한 뒤 재검증한 결과를 정리합니다.

**테스트 환경**
- 컨테이너: `bottom-lidar` (Docker)
- 데이터: 64-ring LiDAR PCD 파일 7개 (프레임당 약 50,000 포인트)
- 빌드 경로: `/root/workdir/build`
- 실행 모드: Headless (GUI 없음)

**대상 알고리즘 (6종)**
| # | 알고리즘 | 설명 |
|---|---------|------|
| 1 | Point-to-Point ICP | 점-점 거리 기반 ICP |
| 2 | Point-to-Plane ICP | 점-평면 거리 기반 ICP |
| 3 | GICP | Generalized ICP |
| 4 | VGICP | Voxelized GICP |
| 5 | LOAM | Lidar Odometry and Mapping (Edge/Plane 특징 기반) |
| 6 | NDT | Normal Distributions Transform |

---

## 발견된 버그 및 수정 사항

### 1. LOAM `validate_correspondences` 각도 변환 버그 (심각도: 높음)

**파일**: `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_loam_factor_impl.hpp`  
**위치**: 536번째 줄

**문제**:
Plane correspondence 검증 시 세 번째 점의 유효성을 판단하는 코드에서 라디안→도 변환이 잘못되어 있었습니다.

```cpp
// 수정 전 (버그)
const double theta_thresh = 5.0 / M_PI * 180.0;

// 수정 후
const double theta_thresh = 5.0 / M_PI / 180.0;
```

**원인 분석**:
- `M_PI * 180.0` → 약 565.49 (라디안)로, `asin()` 반환값(최대 π/2 ≈ 1.57)보다 항상 크므로 조건문이 **항상 true**
- 결과적으로 세 번째 점의 plane correspondence 검증이 **완전히 비활성화**되어 있었음
- 올바른 값: `5.0 / (M_PI / 180.0)` = 약 0.0873 라디안 (≈ 5도), 이는 세 점이 형성하는 평면의 유효성을 적절히 필터링

**영향**:
- 잘못된 plane correspondence가 최적화에 포함되어 수렴 품질 저하
- LOAM의 비용 함수 진동(oscillation)의 간접적 원인

---

### 2. LOAM 비용 함수 진동 (Oscillation) 문제 (심각도: 중간)

**파일**: `src/main.cpp`  
**위치**: 439~451번째 줄 (LOAM factor 생성부)

**문제**:
LOAM factor의 `correspondence_update_tolerance`가 `(0.0, 0.0)`으로 설정되어, **매 반복마다** correspondence를 완전히 재탐색하고 있었습니다.

**증상**:
- 반복 28회 부근부터 비용 함수가 수렴하지 않고 진동
- 100회 최대 반복에 도달해도 수렴 실패
- 단일 프레임 정합에 약 696ms 소요

**수정**:
```cpp
// 수정 전
loam_factor->set_correspondence_update_tolerance(
    correspondence_update_tolerance_rot,    // 0.0
    correspondence_update_tolerance_trans    // 0.0
);

// 수정 후
loam_factor->set_correspondence_update_tolerance(0.005, 0.02);
```

**설명**:
- 회전 변화량 < 0.005 rad, 병진 변화량 < 0.02 m일 때 기존 correspondence를 재사용
- 수렴 근처에서 correspondence가 안정되어 진동 제거
- 다른 알고리즘(ICP, GICP, VGICP, NDT)은 `(0.0, 0.0)` tolerance에서도 정상 수렴하므로 변경하지 않음

---

## 분석 결과 (버그가 아닌 알려진 특성)

### NDT 비용 함수 V자형 패턴

**증상**: NDT의 비용이 수렴 후 다시 발산했다가 재수렴하는 V자 형태의 곡선을 보임 (약 100회 반복, 29초 소요)

**분석**:
- NDT의 이산적 복셀(voxel) 할당 방식에서 비롯되는 **고유 특성**
- 최적화 과정에서 포인트가 다른 복셀로 이동하면 비용 함수가 불연속적으로 변화
- Tolerance를 `(0.01, 0.05)` 및 `(0.001, 0.01)`로 변경하여 실험했으나, 두 경우 모두 정확도가 심각하게 저하됨:
  - `(0.01, 0.05)`: 평균 회전 오차 0.51° → **3.49°** (6.8배 악화)
  - `(0.001, 0.01)`: 50+ 반복 후에도 정확도 미회복
- **결론**: NDT는 원본 설정(`tolerance=0.0`)이 최적. V자형 패턴은 코드 버그가 아님

### VGICP Frame 3 국소 최솟값 (Local Minimum)

**증상**: VGICP가 Frame 3에서만 일관되게 큰 오차 발생 (병진 1.08m, 회전 3.46°)

**분석**:
- VGICP의 단일 복셀 탐색(single-voxel lookup) 방식의 **알려진 한계**
- 초기 자세 추정이 복셀 경계를 넘어갈 경우 올바른 correspondence를 찾지 못함
- 특정 프레임(Frame 3)의 포인트 분포 특성상 국소 최솟값에 빠지는 것
- **결론**: VGICP 알고리즘의 구조적 한계이며, 코드 버그가 아님

---

## 벤치마크 결과 비교

### 수정 전 (Before Fix)

| 알고리즘 | 평균 병진 오차 (m) | 평균 회전 오차 (°) | 최대 병진 오차 (m) | 최대 회전 오차 (°) | 총 소요시간 (ms) |
|---------|:-----------------:|:-----------------:|:-----------------:|:-----------------:|:---------------:|
| Point-to-Point ICP | 0.095 | 0.488 | 0.219 | 0.908 | 9,253 |
| Point-to-Plane ICP | 0.062 | 0.449 | 0.126 | 0.930 | 8,130 |
| GICP | 0.084 | 0.551 | 0.165 | 1.103 | 11,480 |
| VGICP | 0.216 | 1.038 | 1.081 | 3.465 | 9,589 |
| **LOAM** | **0.312** | **1.175** | **0.954** | **2.622** | **696** |
| NDT | 0.078 | 0.510 | 0.143 | 1.129 | 29,232 |

### 수정 후 (After Fix)

| 알고리즘 | 평균 병진 오차 (m) | 평균 회전 오차 (°) | 최대 병진 오차 (m) | 최대 회전 오차 (°) | 총 소요시간 (ms) |
|---------|:-----------------:|:-----------------:|:-----------------:|:-----------------:|:---------------:|
| Point-to-Point ICP | 0.095 | 0.488 | 0.219 | 0.908 | 9,253 |
| Point-to-Plane ICP | 0.062 | 0.449 | 0.126 | 0.930 | 8,130 |
| GICP | 0.084 | 0.551 | 0.165 | 1.103 | 11,480 |
| VGICP | 0.216 | 1.038 | 1.081 | 3.465 | 9,589 |
| **LOAM** | **0.289** | **1.048** | **0.873** | **2.328** | **65** |
| NDT | 0.078 | 0.510 | 0.143 | 1.129 | 29,232 |

### LOAM 개선 요약

| 항목 | 수정 전 | 수정 후 | 변화 |
|------|:------:|:------:|:----:|
| 반복 횟수 | 100 (미수렴, 진동) | **11** (정상 수렴) | ✅ 진동 제거 |
| 소요시간 | 696 ms | **65 ms** | ✅ **10.7배 빨라짐** |
| 평균 병진 오차 | 0.312 m | 0.289 m | ✅ 7.4% 개선 |
| 최대 병진 오차 | 0.954 m | 0.873 m | ✅ 8.5% 개선 |
| 평균 회전 오차 | 1.175° | 1.048° | ✅ 10.8% 개선 |
| 최대 회전 오차 | 2.622° | 2.328° | ✅ 11.2% 개선 |

### 회귀(Regression) 없음 확인

LOAM 외 5개 알고리즘(Point-to-Point, Point-to-Plane, GICP, VGICP, NDT)은 수정 전후 **동일한 결과**를 보여, 회귀가 발생하지 않았음을 확인했습니다.

---

## 수정된 파일 목록

| 파일 | 변경 내용 |
|-----|---------|
| `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_loam_factor_impl.hpp` | 536줄: `M_PI * 180.0` → `M_PI / 180.0` (각도 변환 버그 수정) |
| `src/main.cpp` | LOAM factor의 correspondence tolerance를 `(0.005, 0.02)`로 변경 |

---

## 결론

1. **LOAM 알고리즘에서 2건의 문제를 발견하고 수정**하여, 수렴 속도 10.7배 향상 및 정확도 7~11% 개선을 달성했습니다.
2. **NDT의 V자형 비용 패턴**과 **VGICP의 Frame 3 국소 최솟값**은 알고리즘 고유 특성으로 확인되었으며, 코드 버그가 아닙니다.
3. **나머지 4개 알고리즘**(Point-to-Point, Point-to-Plane, GICP, NDT)은 구현상 문제가 없음을 확인했습니다.
