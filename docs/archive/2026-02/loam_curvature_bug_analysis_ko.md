# LOAM 곡률 구현 버그 분석

## 요약 보고서

**문제점**: `loam_feature.cpp`의 곡률 계산 방식이 오리지널 LOAM 알고리즘 및 LIO-SAM 구현과 다르며, 이로 인해 잘못된 특징 추출(feature extraction)이 발생합니다.

**영향**: 
- 잘못된 정규화로 인해 거리에 따른 곡률 값의 편향 발생
- 추출된 특징점이 LOAM의 의도된 동작과 일치하지 않음
- 엣지/평면 포인트 분류 오류로 인한 정합(registration) 성능 저하

**상태**: 공식 수정이 필요한 치명적인 버그

---

## 1. 공식 비교

### 1.1 Original LOAM (laboshinl/loam_velodyne)

**소스**: [BasicScanRegistration.cpp](https://github.com/laboshinl/loam_velodyne/blob/master/src/lib/BasicScanRegistration.cpp#L291)

```cpp
// Calculate point curvatures
float pointWeight = -2 * _config.curvatureRegion;  // 예: region=5인 경우 -10

for (size_t i = startIdx; i <= endIdx; i++) {
    float diffX = pointWeight * _laserCloud[i].x;  // -10*Xi로 시작
    float diffY = pointWeight * _laserCloud[i].y;
    float diffZ = pointWeight * _laserCloud[i].z;
    
    for (int j = 1; j <= _config.curvatureRegion; j++) {
        diffX += _laserCloud[i + j].x + _laserCloud[i - j].x;  // 이웃 점 추가
        diffY += _laserCloud[i + j].y + _laserCloud[i - j].y;
        diffZ += _laserCloud[i + j].z + _laserCloud[i - j].z;
    }
    
    _regionCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
}
```

**수학적 공식**:
```
c = ||Σ(j=-5 to 5, j≠0) X_{i+j} - 10·X_i||²
c = diffX² + diffY² + diffZ²
```

**주요 특징**:
- **제곱 노름**(Squared norm, ||·||²) 사용
- 포인트 거리(`||X_i||`)에 의한 정규화 없음
- 중심점(-10 가중치)과 이웃 점들의 합 사이의 차이 벡터를 계산
- 곡률의 단위는 **거리²** (m²)

### 1.2 LIO-SAM

**소스**: [featureExtraction.cpp](https://github.com/TixiaoShan/LIO-SAM/blob/master/src/featureExtraction.cpp#L88)

```cpp
void calculateSmoothness() {
    int cloudSize = extractedCloud->points.size();
    for (int i = 5; i < cloudSize - 5; i++) {
        float diffRange = cloudInfo.pointRange[i-5] + cloudInfo.pointRange[i-4]
                        + cloudInfo.pointRange[i-3] + cloudInfo.pointRange[i-2]
                        + cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i] * 10
                        + cloudInfo.pointRange[i+1] + cloudInfo.pointRange[i+2]
                        + cloudInfo.pointRange[i+3] + cloudInfo.pointRange[i+4]
                        + cloudInfo.pointRange[i+5];            

        cloudCurvature[i] = diffRange * diffRange;  // squared
        // ... (중략: cloudSmoothness 할당)
    }
}
```

**수학적 공식**:
```
c = (Σ(j=-5 to 5, j≠0) range_{i+j} - 10·range_i)²
```

**주요 특징**:
- 3D 좌표 대신 **1D 거리(range)**(센서로부터의 거리) 사용
- 여전히 **제곱(squared)** 형태를 유지
- 포인트 거리에 의한 정규화 없음
- 계산 효율성이 높음 (1D vs 3D)
- 곡률의 단위는 **거리²** (m²)

### 1.3 현재 구현 (loam_feature.cpp) - **오류 발생**

**소스**: `src/loam_feature.cpp` (122-135라인)

```cpp
// 링 기반 곡률 계산
for (int i = half_neighbor; i < n - half_neighbor; i++) {
    size_t curr_idx = indices[i];
    Eigen::Vector3d X_i = points_with_ring[curr_idx].point.head<3>();
    
    // 이웃과의 차이 합산
    Eigen::Vector3d diff_sum = Eigen::Vector3d::Zero();
    for (int j = -half_neighbor; j <= half_neighbor; j++) {
        if (j == 0) continue;
        size_t neighbor_idx = indices[i + j];
        Eigen::Vector3d X_j = points_with_ring[neighbor_idx].point.head<3>();
        diff_sum += (X_i - X_j);  // ❌ 오류: (X_i - X_j)가 아니라 X_j여야 함
    }
    
    // ❌ 잘못된 공식: ||Xi|| 및 |S|로 나눔
    double xi_norm = X_i.norm();
    double c = (xi_norm > 0.1) ? diff_sum.norm() / (2 * half_neighbor * xi_norm) : 0.0;
    //         ^^^^^^^^^^^^^^^^^  ^^^^^^^^^^^^^^^^^^  ^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //         임계값 체크         잘못된 노름          잘못된 정규화
}
```

**수학적 공식**:
```
c = ||Σ(j=-5 to 5, j≠0) (X_i - X_{i+j})|| / (10 · ||X_i||)
c = ||10·X_i - Σ X_{i+j}|| / (10 · ||X_i||)
```

**주요 특징**:
- 제곱 노름(||·||²)이 아닌 **노름**(||·||) 사용
- **오류**: `(10 · ||Xi||)`로 정규화함
- 곡률을 무차원(dimensionless)으로 만들고 스케일 의존적으로 만듦
- 오리지널 LOAM과 수학적 의미가 완전히 다름

---

## 2. 버그에 대한 수학적 분석

### 2.1 LOAM이 계산하는 것

Original LOAM은 다음을 계산합니다:
```
c_LOAM = ||Σ X_j - 10·X_i||²
```

이를 전개하면 다음과 같습니다:
```
c_LOAM = (Σ X_j - 10·X_i)ᵀ(Σ X_j - 10·X_i)
        = ||Σ X_j||² - 20·X_iᵀ(Σ X_j) + 100·||X_i||²
```

**물리적 의미**: 
- 중심점이 주변 이웃 점들의 무게 중심(centroid)에서 얼마나 벗어나 있는지를 측정합니다.
- 높은 곡률 → 점이 매끄러운 표면에서 멀리 떨어져 있음 (엣지 특징점)
- 낮은 곡률 → 점이 매끄러운 표면 위에 있음 (평면 특징점)

### 2.2 현재 구현이 계산하는 것

현재 구현은 다음을 계산합니다:
```
c_current = ||10·X_i - Σ X_j|| / (10 · ||X_i||)
```

분자는 LOAM의 공식과 동일합니다 (제곱이 아닌 제곱근 형태라는 점만 다름):
```
c_current = √c_LOAM / (10 · ||X_i||)
```

**문제점**:

1. **잘못된 스케일**: `||Xi||`로 나누면 곡률이 센서와의 거리에 의존하게 됩니다.
    - 먼 거리의 점 (큰 ||Xi||) → 인위적으로 작은 곡률
    - 가까운 거리의 점 (작은 ||Xi||) → 인위적으로 큰 곡률
    - **결과**: 특징점 선택 시 거리 편향(distance bias) 발생

2. **잘못된 단위**: 
    - LOAM: `c`의 단위는 **m²** (거리 제곱)
    - 현재: `c`는 **무차원** (정규화됨)
    - LOAM 논문에서 제시된 임계값을 그대로 사용할 수 없음

3. **제곱 미사용**: `||·||²` 대신 `||·||`를 사용하면 스케일이 비선형적으로 변합니다.
    - LOAM은 큰 편차를 제곱하여 강조합니다.
    - 현재 구현은 그 범위를 압축합니다.

### 2.3 수치 예시

서로 다른 거리에 있는 두 점을 고려해 보겠습니다:

**점 A (센서 근처)**: Xi = [1, 0, 0]m, 이웃 합 = [9.5, 0, 0]m
```
LOAM:    c = ||(9.5, 0, 0) - (10, 0, 0)||² = 0.25 m²
현재 구현: c = ||(10, 0, 0) - (9.5, 0, 0)|| / (10 · 1) = 0.5 / 10 = 0.05
```

**점 B (센서에서 먼 곳)**: Xi = [10, 0, 0]m, 이웃 합 = [95, 0, 0]m  
(기하학적으로 동일하며 거리만 10배 멂)
```
LOAM:    c = ||(95, 0, 0) - (100, 0, 0)||² = 25 m² (100배 더 큼, 정상!)
현재 구현: c = ||(100, 0, 0) - (95, 0, 0)|| / (10 · 10) = 5 / 100 = 0.05 (동일한 값! 오류!)
```

**핵심 이슈**: 현재 구현은 기하학적으로 유사한 점이라도 거리가 다르면 동일한 곡률을 부여하는 반면, LOAM은 거리²에 따라 적절히 스케일링합니다.

---

## 3. 버그 영향 분석

### 3.1 특징 추출 실패

| 문제점 | 설명 | 영향 |
|-------|-------------|--------|
| **거리 편향** | 먼 거리의 점들이 인위적으로 낮은 곡률을 가짐 | 먼 물체의 엣지 특징점이 무시됨 |
| **임계값 호환성** | 정규화된 곡률은 다른 임계값을 필요로 함 | LOAM 논문의 권장값을 사용할 수 없음 |
| **스케일 민감도** | 제곱근과 정규화로 인해 곡률 값이 압축됨 | 엣지/평면 특징점 간의 구분이 모호해짐 |
| **순위 오류** | 거리에 따라 특징점의 상대적 순위가 변함 | 최적이 아닌 특징점이 선택됨 |

### 3.2 정합 성능 문제

1. **엣지 특징점 부족**: 먼 거리의 엣지(예: 50m 밖의 건물 모서리)가 검출되지 않을 수 있습니다.
2. **근거리 평면 과다 선택**: 가까운 평면이 엣지로 오분류될 수 있습니다.
3. **일관성 없는 동작**: 장면의 깊이에 따라 특징 추출 품질이 달라집니다.
4. **매칭 품질 저하**: 잘못된 특징점으로 인해 대응점(correspondence) 매칭 성능이 저하됩니다.

### 3.3 실제 발생 증상

사용자는 다음과 같은 현상을 겪을 수 있습니다:
- 긴 복도에서 정합 드리프트 발생 (먼 거리의 엣지 특징점 누락)
- 벽에 가까워지거나 멀어질 때 불안정한 추적 (거리 의존적 동작)
- 깊이 범위에 따른 특징점 수의 변동
- 레퍼런스 LOAM 구현체에 비해 낮은 성능

---

## 4. 올바른 구현

### 4.1 링 기반 방식 수정

**`loam_feature.cpp`의 122-135라인을 다음으로 교체**:

```cpp
// 곡률 계산 (LOAM 오리지널 공식)
for (int i = half_neighbor; i < n - half_neighbor; i++) {
    size_t curr_idx = indices[i];
    Eigen::Vector3d X_i = points_with_ring[curr_idx].point.head<3>();
    
    // 가중치 합 계산: -10*Xi + 이웃 점들의 합
    Eigen::Vector3d diff_vec = -2.0 * half_neighbor * X_i;  // -10*Xi로 시작
    for (int j = -half_neighbor; j <= half_neighbor; j++) {
        if (j == 0) continue;
        size_t neighbor_idx = indices[i + j];
        Eigen::Vector3d X_j = points_with_ring[neighbor_idx].point.head<3>();
        diff_vec += X_j;  // 각 이웃 점 추가
    }
    
    // LOAM 곡률: 제곱 노름 사용, 정규화 없음
    double c = diff_vec.squaredNorm();  // ||diff_vec||²
    
    index_curvatures.emplace_back(curr_idx, static_cast<float>(c));
}
```

### 4.2 KNN 기반 방식 수정

**`loam_feature.cpp`의 230-246라인을 다음으로 교체**:

```cpp
for(int i = 0; i < num_points; i++) {
    // KNN 검색 (k=11)
    std::array<size_t, 11> neighbors;
    std::array<double, 11> sq_dists;
    size_t num_found = kdtree.knn_search(cloud->points[i].data(), 11, 
                                          neighbors.data(), sq_dists.data());
    
    Eigen::Vector3d X_i = cloud->points[i].head<3>();
    
    // -10*Xi + 이웃 합 계산
    Eigen::Vector3d diff_vec = -10.0 * X_i;  // 중심점 가중치
    for(size_t j = 0; j < num_found; j++) {
        size_t idx = neighbors[j];
        if (idx == static_cast<size_t>(i)) continue;  // 자기 자신 제외
        Eigen::Vector3d X_j = cloud->points[idx].head<3>();
        diff_vec += X_j;
    }
    
    // LOAM 곡률: 제곱 노름
    double c = diff_vec.squaredNorm();
    curvatures[i] = static_cast<float>(c);
}
```

### 4.3 대안: LIO-SAM 거리 기반 접근 방식

성능 향상을 위해 LIO-SAM의 1D 거리 방식을 구현할 수 있습니다:

```cpp
// 링 기반 추출 시
for (int i = half_neighbor; i < n - half_neighbor; i++) {
    size_t curr_idx = indices[i];
    double range_i = points_with_ring[curr_idx].point.head<3>().norm();
    
    // 이웃 거리 합산
    double range_sum = 0.0;
    for (int j = -half_neighbor; j <= half_neighbor; j++) {
        if (j == 0) continue;
        size_t neighbor_idx = indices[i + j];
        range_sum += points_with_ring[neighbor_idx].point.head<3>().norm();
    }
    
    // LIO-SAM 공식: (거리 합 - 10*중심 거리)²
    double diff_range = range_sum - 10.0 * range_i;
    double c = diff_range * diff_range;
    
    index_curvatures.emplace_back(curr_idx, static_cast<float>(c));
}
```

**장점**:
- 계산 속도 빠름 (1D vs 3D)
- LIO-SAM과 동일한 동작
- LOAM의 수학적 특성을 여전히 유지함

---

## 5. 검증 전략

### 5.1 단위 테스트

공식을 비교하는 테스트 케이스를 생성합니다:

```cpp
TEST(LOAMCurvature, FormulaCorrectness) {
    // 테스트 포인트 생성: 평면
    std::vector<Eigen::Vector3d> flat_points = create_flat_surface();
    double curvature_flat = compute_curvature(flat_points[5], flat_points);
    EXPECT_NEAR(curvature_flat, 0.0, 1e-6);  // 0에 가까워야 함
    
    // 테스트 포인트 생성: 날카로운 엣지
    std::vector<Eigen::Vector3d> edge_points = create_edge();
    double curvature_edge = compute_curvature(edge_points[5], edge_points);
    EXPECT_GT(curvature_edge, 1.0);  // 값이 커야 함
    
    // 거리 불변성 테스트 (동일한 기하 구조를 다른 스케일에서 테스트)
    auto scaled_points = scale_points(edge_points, 10.0);
    double curvature_scaled = compute_curvature(scaled_points[5], scaled_points);
    EXPECT_NEAR(curvature_scaled / curvature_edge, 100.0, 0.01);  // 거리²에 비례해야 함
}
```

### 5.2 통합 테스트

1. **특징점 수 일관성**: 
    - 동일한 장면에 대해 다른 거리에서 테스트합니다.
    - 엣지/평면 포인트의 비율이 일정하게 유지되어야 합니다.

2. **정합 정확도**:
    - 레퍼런스 LOAM 구현체와 비교합니다.
    - KITTI 데이터셋에서 정렬 오차를 측정합니다.

3. **시각적 검증**:
    - 추출된 특징점을 곡률에 따라 색상화하여 시각화합니다.
    - 기하학적 불연속 지점에서 엣지가 발생하는지 확인합니다.

### 5.3 회귀 방지

CI/CD에 추가:
```cpp
// 공식이 LOAM 레퍼런스와 일치하는지 확인
TEST(LOAMCurvature, MatchesReference) {
    auto points = load_test_scan();
    auto curvatures_ours = extract_loam_features_ring_based(points);
    auto curvatures_reference = loam_reference_implementation(points);
    
    for (size_t i = 0; i < curvatures_ours.size(); i++) {
        EXPECT_NEAR(curvatures_ours[i], curvatures_reference[i], 1e-4);
    }
}
```

---

## 6. 마이그레이션 가이드

### 6.1 필요한 코드 변경

| 파일 | 함수 | 라인 | 변경 유형 |
|------|----------|-------|-------------|
| `loam_feature.cpp` | `extract_loam_features_ring_based()` | 122-135 | 공식 수정 (심각) |
| `loam_feature.cpp` | `extract_loam_features_knn()` | 230-246 | 공식 수정 (심각) |
| `loam_feature.cpp` | 주석 | 68, 132, 244 | 문서 업데이트 |

### 6.2 파라미터 조정

공식을 수정한 후에는 곡률 값이 크게 변하게 됩니다:

**기존 (버그) 임계값**:
```cpp
// 무차원, 일반적으로 0.0-0.1 범위
const float edge_threshold = 0.05;
const float planar_threshold = 0.01;
```

**신규 (정상) 임계값**:
```cpp
// 단위: m², 실외 장면에서 일반적으로 0.001-1.0 범위
const float edge_threshold = 0.1;      // 테스트를 통해 조정 필요
const float planar_threshold = 0.01;   // 테스트를 통해 조정 필요
```

하드코딩된 임계값 대신 백분위수 기반 선택(현재 코드의 164-186라인 방식)을 사용하는 것이 좋습니다.

### 6.3 하위 호환성

**중대한 변경 사항**: 이 변경은 재조정(retuning)이 필요한 중대한 변경 사항입니다.

**마이그레이션 경로**:
1. 공식 수정
2. 테스트 데이터셋에서 특징 추출 재실행
3. 새로운 곡률 분포 분석
4. 필요한 경우 엣지/평면 선택 백분위수 조정 (현재: 엣지 5%, 평면 10%)
5. 정합 성능 재평가

---

## 7. 참고 문헌

### 원본 논문
1. **LOAM**: Zhang, J., & Singh, S. (2014). "LOAM: Lidar Odometry and Mapping in Real-time." *Robotics: Science and Systems (RSS)*.
   - [PDF](https://www.ri.cmu.edu/pub_files/2014/7/Ji_LidarMapping_RSS2014_v8.pdf)
   - [DOI: 10.15607/RSS.2014.X.007](https://doi.org/10.15607/RSS.2014.X.007)

2. **LIO-SAM**: Shan, T., Englot, B., Meyers, D., Wang, W., Ratti, C., & Rus, D. (2020). "LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping." *IROS 2020*.

### 레퍼런스 구현체
1. **loam_velodyne**: https://github.com/laboshinl/loam_velodyne
   - `src/lib/BasicScanRegistration.cpp` 291라인 참조
    
2. **LIO-SAM**: https://github.com/TixiaoShan/LIO-SAM
   - `src/featureExtraction.cpp` 67라인 (`calculateSmoothness()`) 참조

3. **A-LOAM**: https://github.com/HKUST-Aerial-Robotics/A-LOAM
   - 대안적인 레퍼런스 구현체

---

## 8. 부록: 공식 유도

### A. 왜 LOAM에서는 `-10*Xi`를 사용하는가?

LOAM은 해당 점 `i`가 국소적인 매끄러움(smoothness)에서 얼마나 벗어나는지 계산하고자 합니다.

**매끄러운 표면 가정**: 점들이 매끄러운 표면 위에 있다면, 중심점은 이웃 점들의 평균과 가까워야 합니다:
```
X_i ≈ (1/10) · Σ X_j
```

정리하면:
```
10·X_i ≈ Σ X_j
```

**곡률 측정**: 매끄러움에서의 이탈 정도:
```
c = ||Σ X_j - 10·X_i||²
```

`c`가 작으면 → 표면이 매끄러움 (평면 특징점)  
`c`가 크면 → 표면이 굽어 있음 (엣지 특징점)

### B. 왜 제곱 노름을 사용하는가?

`||·||` 대신 `||·||²`를 사용하는 이유:
1. **계산 효율성**: 제곱근 연산이 필요 없음
2. **아웃라이어 강조**: 제곱 페널티를 통해 큰 편차를 더 강조함
3. **정렬 불변성**: 특징점 선택 시 `||a||² > ||b||²` 이면 `||a|| > ||b||`가 성립함

### C. 왜 ||Xi||로 정규화하지 않는가?

`||Xi||`로 정규화하는 것이 곡률을 **스케일 불변**으로 만들어 직관적으로 보일 수 있지만, LOAM에서는 **틀린** 방식입니다:

**문제점**: LOAM은 거리가 중요한 센서 프레임에서 동작합니다:
- 가까운 점: 샘플링 밀도가 높음 → 더 작은 절대 편차가 예상됨
- 먼 점: 샘플링 밀도가 낮음 → 더 큰 절대 편차가 예상됨

**올바른 동작**: 곡률은 다음을 고려하여 `거리²`에 비례해야 합니다:
- 측정 노이즈 (거리에 따라 증가)
- 포인트 간격 (거리에 따라 선형적으로 증가)
- 각해상도(angular resolution) 효과

**결과**: LOAM의 정규화되지 않은 곡률은 특징점의 신뢰도에 따라 자연스럽게 가중치를 부여하게 됩니다.

---

## 문서 메타데이터

- **생성일**: 2026-02-10
- **작성자**: Sisyphus AI (ULTRAWORK 모드)
- **최종 수정일**: 2026-02-10
- **버전**: 1.0
- **관련 이슈**: loam_feature.cpp 곡률 버그
- **상태**: 분석 완료, 수정 권고
