# main.cpp 변경내역: Headless 벤치마크 및 NDT 통합

> **작업일**: 2026-02-20  
> **대상 파일**: `src/main.cpp`  
> **Docker 환경**: `kwu-cvrl/bottom-lidar:latest` (container: `bottom-lidar`)

---

## 1. 변경사항 요약

| 구분 | 내용 | 상태 |
|------|------|------|
| Headless 모드 추가 | `--headless` 플래그로 GUI 없이 벤치마크 실행 | ✅ 적용 |
| NDT Factor 통합 | 6번째 알고리즘으로 NDT 추가 | ✅ 적용 |
| Factor 이름 변경 | `ICP` → `Point-to-Point`, `ICP_PLANE` → `Point-to-Plane` | ✅ 적용 |
| Summary 통계 확장 | Max Error, Optimization Time 추가 | ✅ 적용 |
| 비교표 자동 출력 | 헤드리스 모드 종료 시 6개 알고리즘 비교표 출력 | ✅ 적용 |
| NDT resolution 실험 | `set_resolution(0.5)` 적용 후 정확도 저하로 **revert** | ⚠️ Revert 완료 |

---

## 2. Headless 모드 구현

### 2.1 실행 방법

```bash
# GUI 모드 (기존과 동일)
./lidar_registration_benchmark

# Headless 모드 (GUI 없이 6개 Factor 순차 실행)
./lidar_registration_benchmark --headless
```

### 2.2 구현 내용

**`main()` 함수**: `--headless` 인자 파싱 추가
```cpp
bool headless_mode = false;
for (int i = 1; i < argc; i++) {
    if (std::strcmp(argv[i], "--headless") == 0) headless_mode = true;
}
MatchingCostFactorDemo demo(headless_mode);
```

**`MatchingCostFactorDemo` 생성자**: `headless` 멤버 변수에 따라 GUI 초기화 분기
- `headless=true`: Viewer/ImGui 초기화 생략, UI 콜백 미등록
- `headless=false`: 기존 GUI 모드 동작 유지

**`run_all_factors_headless()`**: 새로 추가된 메서드
1. 동일한 노이즈가 적용된 초기 포즈 생성 (모든 Factor에 동일 조건)
2. 6개 Factor를 순차 실행 (Point-to-Point, Point-to-Plane, GICP, VGICP, LOAM, NDT)
3. 각 Factor별 프레임별 에러 출력
4. 최종 비교표 출력

### 2.3 GUI 가드 처리

GUI 관련 코드에 `headless` 가드를 추가하여 headless 모드에서 segfault 방지:

- Viewer 초기화: `if (!headless) { viewer = guik::LightViewer::instance(); ... }`
- 포인트 클라우드 시각화: `if (!headless && viewer) { viewer->update_drawable(...); }`
- 뷰어 업데이트: `update_viewer()` 함수 진입부에 `if (headless) return;`
- LM 콜백: `if (!headless) { guik::LightViewer::instance()->append_text(...); }`
- ISAM2 콜백: 동일한 패턴 적용

---

## 3. NDT Factor 통합

### 3.1 헤더 추가

```cpp
#include <gtsam_points/factors/integrated_ndt_factor.hpp>
```

### 3.2 Factor 타입 등록

```cpp
factor_types.push_back("NDT"); // 5번째 (0-indexed)
```

### 3.3 NDT Factor 생성 블록

```cpp
else if (factor_types[factor_type] == std::string("NDT"))
{
    auto factor = gtsam::make_shared<gtsam_points::IntegratedNDTFactor_<gtsam_points::PointCloud>>(
        target_key, source_key, target_voxelmap, source);
    factor->set_num_threads(num_threads);
    factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT7);
    factor->set_outlier_ratio(0.1);
    factor->set_regularization_epsilon(1e-3);
    factor->set_correspondence_update_tolerance(
        correspondence_update_tolerance_rot,
        correspondence_update_tolerance_trans);
    return factor;
}
```

### 3.4 NDT 파라미터 설명

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| `search_mode` | `DIRECT7` | 중심 + 6방향(±x,±y,±z) 복셀 탐색 |
| `outlier_ratio` | 0.1 | 아웃라이어 비율 10% |
| `regularization_epsilon` | 1e-3 | 공분산 행렬 정규화 |
| `resolution` | 1.0 (기본값) | NDT 스코어링 함수의 d1/d2 결정 |
| `correspondence_update_tolerance` | (0.0, 0.0) | 매 iteration 대응점 갱신 |

---

## 4. Factor 이름 변경

| 변경 전 | 변경 후 | 이유 |
|---------|---------|------|
| `ICP` | `Point-to-Point` | 알고리즘 특성을 명확히 표현 |
| `ICP_PLANE` | `Point-to-Plane` | 알고리즘 특성을 명확히 표현 |

이 변경은 `factor_types` 배열의 문자열과 `create_factor()` 함수의 비교 문자열 양쪽에 적용.

---

## 5. Summary 통계 확장

### 5.1 기존 출력 항목
- Mean Translation Error
- Mean Rotation Error

### 5.2 추가된 항목
- **Max Translation Error**: 프레임별 최대 병진 오차
- **Max Rotation Error**: 프레임별 최대 회전 오차
- **Optimization Time**: 최적화 소요 시간 (ms)

### 5.3 클래스 멤버 변수 추가

```cpp
double last_mean_trans_error = 0.0;
double last_mean_rot_error = 0.0;
double last_max_trans_error = 0.0;
double last_max_rot_error = 0.0;
double last_total_ms = 0.0;
```

---

## 6. NDT Resolution 실험 및 Revert

### 6.1 실험 배경

NDT Factor의 `resolution` 기본값(1.0)과 voxelmap 해상도(0.5m)가 불일치하여 `set_resolution(0.5)`를 적용해 봄.

### 6.2 실험 결과

| 설정 | Mean T (m) | Mean R (°) | Max T (m) | Max R (°) | 시간 (ms) |
|------|-----------|------------|----------|----------|----------|
| resolution=1.0 (기본) | **0.078** | **0.510** | **0.143** | **1.129** | 29,091 |
| resolution=0.5 (실험) | 0.115 | 3.617 | 0.212 | 5.512 | 4,315 |

### 6.3 분석

- resolution=0.5로 변경 시 속도는 29초 → 4초로 대폭 개선
- 그러나 **Mean Rotation Error가 0.51° → 3.62°로 7배 악화**
- 원인: `compute_ndt_params()`에서 `c2 = outlier_ratio / resolution³`
  - resolution=0.5: c2 = 0.1/0.125 = 0.8 → 아웃라이어 확률이 지배적
  - resolution=1.0: c2 = 0.1/1.0 = 0.1 → 적절한 균형
- resolution은 NDT 스코어링 함수의 형상을 결정하는 파라미터이며, voxelmap 해상도와 일치시킬 필요 없음

### 6.4 결론

**`set_resolution(0.5)` 삭제 (revert)**. 기본값 1.0 유지가 정확도 면에서 최적.

---

## 7. 최종 벤치마크 결과

revert 후 재빌드/재실행하여 확인한 최종 결과:

| Factor | Mean T (m) | Mean R (°) | Max T (m) | Max R (°) | 시간 (ms) | 상태 |
|--------|-----------|------------|----------|----------|----------|------|
| Point-to-Plane | 0.062 | 0.449 | 0.126 | 0.930 | 8,069 | ✅ 최고 성능 |
| NDT | 0.078 | 0.510 | 0.143 | 1.129 | 29,091 | ✅ 정확도 2위 |
| GICP | 0.084 | 0.551 | 0.165 | 1.103 | 11,448 | ✅ 양호 |
| Point-to-Point | 0.095 | 0.488 | 0.219 | 0.908 | 8,982 | ✅ 양호 |
| VGICP | 0.216 | 1.038 | 1.081 | 3.465 | 9,786 | ⚠️ Frame 3 이상치 |
| LOAM | 0.312 | 1.157 | 0.954 | 2.622 | 706 | ❌ 수렴 실패 |

### 알고리즘별 특이사항

**VGICP**: Frame 3에서 Translation Error 1.08m 발생. 초기 추정값이 국소 최적점에 빠짐. 코드 버그 아닌 수렴 특성 문제.

**LOAM**: iter 28부터 cost가 2763.57 ↔ 2763.92 사이를 무한 진동. 수중 LiDAR 데이터에서 edge/planar 피처 부족(~2.6%)으로 인한 구조적 한계. lambda가 1e-105까지 감소해도 수렴하지 못함.

**NDT**: 정확도는 2위이나 29초로 가장 느림. 101회 max iteration 도달이 주 원인. 비볼록 비용 함수 특성상 LM 수렴 판정이 어려움.

---

## 8. 코드 변경 상세 (git diff 기준)

### 추가된 include
- `#include <cstring>` (strcmp 사용)
- `#include <gtsam_points/factors/integrated_ndt_factor.hpp>` (NDT Factor)

### MatchingCostFactorDemo 클래스 변경
- 생성자: `(bool headless_mode = false)` 매개변수 추가
- 멤버 변수: `bool headless`, `last_mean_trans_error`, `last_mean_rot_error`, `last_max_trans_error`, `last_max_rot_error`, `last_total_ms` 추가
- `run_all_factors_headless()` 메서드 추가
- `run_optimization()`: 타이밍 측정 코드 추가, Max Error 계산 추가
- GUI 관련 코드: `if (!headless)` 가드 추가

### main() 함수 변경
- `--headless` 인자 파싱 추가
- headless 모드 분기 처리 (`run_all_factors_headless()` vs `guik::LightViewer::instance()->spin()`)
