# LiDAR Registration Benchmark 결과 분석

> **실험 날짜**: 2026-02-20 (Baseline, NDT 해상도 실험) / 2026-02-20 (NDT 속도 최적화)  
> **데이터**: 7 프레임 (수중 LiDAR 포인트클라우드, ~50K 포인트/프레임)  
> **옵티마이저**: Levenberg-Marquardt (max 101 iterations)  
> **노이즈 스케일**: 0.1  
> **Voxelmap 해상도**: 0.5m  

---

## 1. 전체 결과 요약

| Factor | Mean T (m) | Mean R (°) | Max T (m) | Max R (°) | 시간 (ms) | 반복 횟수 | 상태 |
|--------|-----------|------------|----------|----------|----------|----------|------|
| **Point-to-Plane** | **0.062** | **0.449** | 0.126 | 0.930 | 8,040 | 25 | ✅ 최고 성능 |
| **NDT** | **0.078** | **0.510** | 0.143 | 1.129 | 32,956 | 101 | ✅ 양호 |
| GICP | 0.084 | 0.551 | 0.165 | 1.103 | 11,324 | 31 | ✅ 양호 |
| Point-to-Point | 0.095 | 0.488 | 0.219 | 0.908 | 9,036 | 31 | ✅ 양호 |
| **VGICP** | **0.216** | **1.038** | **1.081** | **3.465** | 9,702 | 67 | ⚠️ 문제 있음 |
| **LOAM** | **0.312** | **1.157** | **0.954** | **2.622** | 694 | 101 | ❌ 수렴 실패 |

### 순위 (Mean Translation Error 기준)
1. 🥇 **Point-to-Plane** — 0.062m, 0.449°
2. 🥈 **NDT** — 0.078m, 0.510°
3. 🥉 **GICP** — 0.084m, 0.551°
4. **Point-to-Point** — 0.095m, 0.488°
5. ⚠️ **VGICP** — 0.216m, 1.038° (Frame 3 이상치)
6. ❌ **LOAM** — 0.312m, 1.157° (수렴 실패)

---

## 2. 정상 작동 Factor 분석 (상위 4개)

### 2.1 Point-to-Plane (1위)
- **가장 정확하고 안정적인 결과**
- 모든 프레임에서 Translation Error < 0.13m, Rotation Error < 1.0°
- 25회 반복으로 빠르게 수렴
- 법선 벡터 정보를 활용하여 평면 정합에 강점

### 2.2 NDT (2위)
- Translation 정확도 우수 (Mean 0.078m, Max 0.143m)
- 101회 반복까지 진행되었으나 비용 함수가 꾸준히 감소
- **단점**: 32.9초로 가장 느림 (NDT 분포 계산 + 7방향 탐색 비용)
- `NDTSearchMode::DIRECT7`로 7방향 그래디언트 탐색 사용

### 2.3 GICP (3위)
- 공분산 행렬 기반 정합으로 안정적 수렴
- 31회 반복, 11.3초 소요
- Max Rotation Error 1.1°로 약간 높지만 허용 범위 내

### 2.4 Point-to-Point (4위)
- 가장 단순한 거리 기반 정합
- Mean Translation은 0.095m으로 양호하나, Max Translation 0.219m으로 상대적으로 높음
- 31회 반복, 9.0초 소요

---

## 3. 문제 Factor 분석

### 3.1 ⚠️ VGICP — Frame 3 이상치 문제

#### 프레임별 에러 상세

| Frame | T Error (m) | R Error (°) | 판정 |
|-------|------------|------------|------|
| 0 | 0.000 | 0.000 | ✅ 고정 프레임 |
| 1 | 0.048 | 0.508 | ✅ 정상 |
| 2 | 0.040 | 0.444 | ✅ 정상 |
| **3** | **1.081** | **3.465** | ❌ **이상치** |
| 4 | 0.115 | 1.105 | ⚠️ 약간 높음 |
| 5 | 0.095 | 0.576 | ✅ 정상 |
| 6 | 0.136 | 1.169 | ⚠️ 약간 높음 |

#### Frame 3 상세 비교
```
[GT]        t: [-8.006, -0.711, 0.390]   R: [-3.190°, -0.367°, 1.759°]
[VGICP]     t: [-9.070, -0.522, 0.420]   R: [ 0.262°, -0.657°, 1.790°]
[에러]      t: 1.081m                     R: 3.465°
```

#### 원인 분석
1. **국소 최적점 (Local Minimum)**: VGICP는 복셀화된 공분산을 사용하므로, 복셀 경계에서 대응점이 크게 변동할 수 있음
2. **Frame 3 특성**: GT에서 [-8.006m, -0.711m] 이동으로 가장 큰 변위를 가진 프레임 중 하나. 큰 변위에서 초기 대응점이 잘못 설정되면 잘못된 최적점으로 수렴
3. **67회 반복**: 다른 factor 대비 반복 횟수가 높고, 비용 함수가 초반에 증가 후 감소하는 불안정한 패턴 (iter 0: 284K → iter 4: 371K → iter 17: 305K)
4. **Frame 4 영향**: Frame 3의 이상치가 Frame 4의 정합에도 영향 (1.105° 에러)

#### 잠재적 해결 방안
- VGICP 복셀 해상도 조정 (현재 0.5m → 0.3m 또는 1.0m)
- 대응점 최대 거리 제한 추가
- 초기 추정값 개선 (GT에 가까운 초기값 사용)

### 3.2 ❌ LOAM — 비용 함수 진동 (수렴 실패)

#### 수렴 과정
```
iter  0: cost = 5364.89  (빠르게 감소)
iter  5: cost = 2761.05  
iter  7: cost = 2753.04  (최소값 도달)
iter  8: cost = 2755.77  (증가 시작)
...
iter 28: cost = 2763.59  ┐
iter 29: cost = 2763.92  │ 이후 2763.57 ↔ 2763.92
iter 30: cost = 2763.57  │ 무한 진동 (101회까지)
iter 31: cost = 2763.92  ┘
```

#### 프레임별 에러 상세

| Frame | T Error (m) | R Error (°) | 판정 |
|-------|------------|------------|------|
| 0 | 0.000 | 0.000 | ✅ 고정 프레임 |
| 1 | 0.126 | 0.710 | ⚠️ 높음 |
| 2 | 0.212 | 1.294 | ❌ 높음 |
| 3 | 0.243 | 1.031 | ❌ 높음 |
| 4 | 0.225 | 0.987 | ❌ 높음 |
| 5 | 0.422 | 1.458 | ❌ 매우 높음 |
| **6** | **0.954** | **2.622** | ❌ **매우 높음** |

#### 원인 분석
1. **피처 부족**: 프레임당 edge ~490개, planar ~870개로 전체 포인트 (~50K) 대비 매우 적음 (약 2.6%)
2. **수중 환경 특성**: 수중 LiDAR 데이터는 지상 대비 에지/평면 피처가 불분명하여 LOAM 피처 추출이 어려움
3. **비용 함수 진동**: LM 옵티마이저의 lambda가 계속 감소하지만 (1e-5 → 1e-105), 비용이 2763.57 ↔ 2763.92 사이를 반복. 이는 두 개의 거의 동일한 에너지 상태 사이에서 왕복하는 것을 의미
4. **누적 오차**: 프레임이 진행될수록 에러가 급격히 증가 (Frame 1: 0.126m → Frame 6: 0.954m)

#### 잠재적 해결 방안
- LOAM 피처 추출 파라미터 조정 (더 많은 피처 추출)
- 에지/평면 판별 임계값 완화
- 수중 환경에 특화된 피처 추출 방법 적용
- LM 옵티마이저에 수렴 판별 기준 강화 (진동 감지 시 조기 종료)

---

## 4. NDT 해상도 실험

NDT Factor의 `resolution` 파라미터를 변경하여 성능 비교를 수행했다.

### 실험 조건
- **기본값 (resolution=1.0)**: NDT 내부 기본 해상도 사용
- **변경값 (resolution=0.5)**: voxelmap 해상도(0.5)와 일치시킴

### 결과 비교

| 설정 | Mean T (m) | Mean R (°) | Max T (m) | Max R (°) | 시간 (ms) | 반복 |
|------|-----------|------------|----------|----------|----------|------|
| **resolution=1.0 (기본)** | **0.078** | **0.510** | **0.143** | **1.129** | 32,956 | 101 |
| resolution=0.5 | 0.115 | 3.617 | 0.301 | 5.512 | 4,965 | 101 |

### 분석
- **resolution=0.5는 성능이 크게 저하됨**
  - Mean Rotation Error가 0.510° → 3.617°로 **7배 악화**
  - Max Rotation Error가 1.129° → 5.512°로 **5배 악화**
- **원인**: NDT의 `compute_ndt_params` 함수에서 resolution은 확률 밀도 함수의 d1, d2 파라미터를 결정함
  ```
  c2 = outlier_ratio / (resolution³)
  ```
  - resolution=0.5일 때 c2 = 0.1/0.125 = 0.8으로, 아웃라이어 확률이 지배적이 됨
  - resolution=1.0일 때 c2 = 0.1/1.0 = 0.1로, 정상 분포와 아웃라이어 분포의 균형이 적절
- **결론**: **resolution=1.0 (기본값)을 유지해야 함**. voxelmap 해상도와 일치시킬 필요 없음

---

## 5. NDT 속도 최적화

### 5.1 문제점

NDT는 정확도 2위(0.078m)임에도 **32,956ms로 가장 느린 factor**였다. 원인 분석:

| 원인 | 영향도 | 설명 |
|------|--------|------|
| 101회 반복 (max iterations 도달) | ~70% | 비볼록 비용 함수로 인해 LM이 수렴 판정에 실패 |
| inv_cov 매 호출 재계산 | ~12% | 타겟 복셀의 역공분산을 매 iteration마다 eigendecomposition 수행 |
| NDT 분포 계산 + 7방향 탐색 | ~18% | `DIRECT7` 모드로 7개 방향의 복셀을 탐색 |

### 5.2 적용한 최적화

#### 최적화 A: inv_cov 캐시 (클래스 레벨)

**원리**: 타겟 포인트클라우드의 복셀 공분산은 정합 과정에서 변하지 않으므로, `compute_ndt_inverse_covariance()` 결과를 1회만 계산하고 캐시하여 재사용.

**구현 위치**: `integrated_ndt_factor_impl.hpp`
```cpp
// 클래스 멤버로 캐시 추가
std::vector<Eigen::Matrix3d> inv_cov_cache;
bool inv_cov_cached;

// update_correspondences()에서 1회만 계산
if (!inv_cov_cached) {
  inv_cov_cache.resize(num_correspondences);
  for (int i = 0; i < num_correspondences; i++) {
    inv_cov_cache[i] = compute_ndt_inverse_covariance(target_covs[i]);
  }
  inv_cov_cached = true;
}
```

**결과**: 32,956ms → **28,879ms** (**12.4% 속도 향상**), 정확도 변화 없음

#### 최적화 B: correspondence_update_tolerance (GICP 패턴)

**원리**: GICP와 동일하게, 이전 iteration 대비 포즈 변화가 tolerance 이하면 대응점 재탐색을 건너뜀.

**구현 위치**: `integrated_ndt_factor.hpp`, `integrated_ndt_factor_impl.hpp`
```cpp
// GICP와 동일한 API
void set_correspondence_update_tolerance(double angle, double trans);
```

**실험 결과** (tolerance=0.01rad / 0.1m, 모든 factor에 적용):

| Factor | Mean T (m) | Mean R (°) | 시간 (ms) | 반복 | 비고 |
|--------|-----------|------------|----------|------|------|
| NDT | 0.141 | 1.466 | 17,460 | 101 | ❌ 정확도 크게 저하 |
| GICP | 0.092 | 0.523 | 2,942 | 9 | ✅ 정확도 유지, 4배 가속 |
| Point-to-Plane | 0.076 | 0.412 | 3,255 | 12 | ✅ 정확도 유지, 2.5배 가속 |
| LOAM | 0.296 | 1.035 | 50 | 9 | ✅ 정확도 유사, 14배 가속 |

**NDT에서 tolerance가 해로운 이유**:
- NDT의 비용 함수는 **비볼록**이므로, 대응점이 업데이트되지 않으면 잘못된 국소 최적점으로 빠지기 쉬움
- 다른 factor(GICP, Point-to-Plane)는 비용 함수가 상대적으로 **볼록**하여 tolerance 적용이 안전
- **결론**: NDT에는 tolerance=0.0 (매 iteration 대응점 갱신)을 유지해야 함

### 5.3 최종 설정

```cpp
// main.cpp - NDT factor 생성 블록
factor->set_num_threads(num_threads);
factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT7);
factor->set_outlier_ratio(0.1);
factor->set_regularization_epsilon(1e-3);
factor->set_correspondence_update_tolerance(
    correspondence_update_tolerance_rot,    // 기본값: 0.0
    correspondence_update_tolerance_trans   // 기본값: 0.0
);
// tolerance=0.0이면 매 iteration 대응점 갱신 (NDT에 최적)
// inv_cov 캐시는 자동 적용 (내부 구현)
```

### 5.4 최적화 전후 비교

| 구분 | Mean T (m) | Mean R (°) | Max T (m) | Max R (°) | 시간 (ms) | 반복 |
|------|-----------|------------|----------|----------|----------|------|
| **Baseline (최적화 전)** | 0.078 | 0.510 | 0.143 | 1.129 | 32,956 | 101 |
| **inv_cov 캐시 적용** | 0.078 | 0.510 | 0.143 | 1.129 | **28,879** | 101 |
| tolerance=0.01/0.1 (**폐기**) | 0.141 | 1.466 | — | — | 17,460 | 101 |

> ✅ **채택**: inv_cov 캐시만 적용 (12.4% 속도 향상, 정확도 동일)
> ❌ **폐기**: tolerance 적용 (정확도 81% 악화 → NDT 비볼록 특성 때문)

---

## 6. NDT 구현 이력

### 이전 문제 (수정 완료)
이전 세션에서 NDT Factor를 gtsam_points 라이브러리에 통합 구현했다. 초기 구현에서 다음과 같은 문제가 있었다:

- **초기 상태**: Mean R=4.577°, 11회 반복, cost_d=0 (그래디언트 없음)
- **원인**: NDT 비용 함수의 Jacobian/Hessian 계산 오류
- **수정 내용**:
  - `IntegratedNDTFactor` 클래스를 gtsam_points 내부에 구현
  - `compute_ndt_params()`: NDT 확률 모델 파라미터 계산 (d1, d2)
  - `compute_ndt_inverse_covariance()`: 정규화된 역공분산 계산 (epsilon 정규화 포함)
  - 7방향 탐색 모드 (`DIRECT7`): ±x, ±y, ±z, center
  - outlier_ratio, regularization_epsilon 설정 지원

### 현재 NDT 설정 (main.cpp)
```cpp
factor->set_num_threads(num_threads);
factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT7);
factor->set_outlier_ratio(0.1);
factor->set_regularization_epsilon(1e-3);
factor->set_correspondence_update_tolerance(0.0, 0.0);  // NDT는 매 iteration 갱신
// resolution = 1.0 (기본값 유지)
// inv_cov 캐시: 자동 적용 (타겟 복셀 역공분산 1회 계산 후 재사용)
```

---

## 7. 종합 결론

### 최종 벤치마크 결과 (최적화 후)

| Factor | Mean T (m) | Mean R (°) | Max T (m) | Max R (°) | 시간 (ms) | 반복 | 상태 |
|--------|-----------|------------|----------|----------|----------|------|------|
| **Point-to-Plane** | **0.062** | **0.449** | 0.126 | 0.930 | 8,238 | 25 | ✅ 최고 성능 |
| **NDT** | **0.078** | **0.510** | 0.143 | 1.129 | **28,879** | 101 | ✅ 양호 (12% 가속) |
| GICP | 0.084 | 0.551 | 0.165 | 1.103 | 11,496 | 31 | ✅ 양호 |
| Point-to-Point | 0.095 | 0.488 | 0.219 | 0.908 | 9,291 | 31 | ✅ 양호 |
| **VGICP** | **0.216** | **1.038** | **1.081** | **3.465** | 9,679 | 67 | ⚠️ 문제 있음 |
| **LOAM** | **0.312** | **1.157** | **0.954** | **2.622** | 703 | 101 | ❌ 수렴 실패 |

### 정상 작동 Factor (4개)
| Factor | 추천 용도 |
|--------|----------|
| Point-to-Plane | 정확도 우선 시 **최고 추천** |
| NDT | 정확도 양호, inv_cov 캐시로 12% 가속 (추가 최적화 여지 있음) |
| GICP | 균형 잡힌 성능 |
| Point-to-Point | 단순하고 빠른 초기 정합에 적합 |

### 문제가 있는 Factor (2개)
| Factor | 문제 | 심각도 | 다음 단계 |
|--------|------|--------|----------|
| VGICP | Frame 3 국소 최적점 | ⚠️ 중간 | 복셀 해상도/대응점 거리 조정 필요 |
| LOAM | 비용 함수 진동, 수렴 실패 | ❌ 높음 | 피처 추출 파라미터 조정 또는 수중 환경 특화 필요 |

### 핵심 발견사항
1. **NDT는 정상 작동**: 구현한 NDT Factor가 2위 성능으로 안정적으로 동작
2. **inv_cov 캐시 최적화**: 타겟 복셀 역공분산을 1회만 계산하여 12.4% 속도 향상 (32,956→28,879ms)
3. **tolerance는 NDT에 부적합**: 비볼록 비용 함수 특성으로 인해 정확도 81% 악화 → 폐기
4. **VGICP Frame 3 이상치**: 1.08m 오차로 다른 프레임 대비 10배 이상 큰 에러
5. **LOAM 수렴 실패**: 수중 환경에서 edge/planar 피처가 부족하여 구조적 한계
6. **NDT resolution 실험**: resolution=0.5는 성능 악화, 기본값(1.0) 유지가 최적
7. **NDT 속도의 근본 원인**: 101회 max iteration 도달이 주 원인 (~70%). 비볼록 비용 함수로 LM 수렴 판정이 어려움 → 진동 감지 기반 조기 종료가 향후 과제
