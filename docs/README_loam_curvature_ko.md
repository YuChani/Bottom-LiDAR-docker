# LOAM 곡률 버그 분석 - 전체 문서

## 📋 개요

이 디렉토리는 `loam_feature.cpp`의 곡률 계산 버그에 대한 포괄적인 분석과 오리지널 LOAM 및 LIO-SAM 구현과의 차이점을 담고 있습니다.

**분석 날짜**: 2026-02-10  
**상태**: ✅ 완료 - 버그 식별, 문서화, 해결 방법 제공  
**심각도**: 🔴 치명적 - 특징 추출(feature extraction) 품질에 영향을 미침

---

## 📁 문서 파일

### 1. [`loam_curvature_bug_analysis.md`](./loam_curvature_bug_analysis.md)
**유형**: 기술 분석 문서 (490라인)

**내용**:
- ✅ 전체 공식 비교 (Original LOAM vs LIO-SAM vs 현재 구현)
- ✅ 유도 과정을 포함한 수학적 버그 분석
- ✅ 버그를 보여주는 수치 예시
- ✅ 영향 분석 (거리 편향, 특징 추출 실패)
- ✅ 올바른 구현 코드
- ✅ 검증 전략
- ✅ 마이그레이션 가이드

**권장 대상**: 버그와 해결 방법에 대한 상세한 기술적 이해가 필요한 경우.

---

### 2. [`loam_curvature_diagrams.md`](./loam_curvature_diagrams.md)
**유형**: 시각 자료 문서 (221라인)

**내용**:
- 📊 7개의 UML 다이어그램 가이드
- 📊 빠른 참조를 위한 시각적 요약
- 📊 수치 예시 시각화
- 📊 빠른 이해를 위한 ASCII 아트 다이어그램

**권장 대상**: 시각적 설명을 선호하거나 발표 자료가 필요한 경우.

---

### 3. [`loam_curvature_uml.puml`](./loam_curvature_uml.puml)
**유형**: PlantUML 다이어그램 소스 (653라인)

**7개 다이어그램 포함**:
1. **Curvature Comparison** - 공식의 병렬 비교
2. **Algorithm Flow** - 단계별 실행 순서도
3. **Bug Impact Sequence** - 거리에 따른 동작 시연
4. **Mathematical Comparison** - 단위를 포함한 공식 세부 분석
5. **Class Diagram** - 구현을 보여주는 OOP 구조
6. **State Machine** - 특징 분류 상태
7. **Component Diagram** - 시스템 아키텍처 뷰

**온라인 보기**: http://www.plantuml.com/plantuml/ (내용 복사 후 붙여넣기)  
**로컬 보기**: VS Code에 PlantUML 확장 설치 또는 `plantuml` CLI 사용

---

## 🐛 버그 요약

### 문제점

**현재 구현** (오류):
```cpp
c = ||10·X_i - Σ X_j|| / (10·||X_i||)
```

**Original LOAM** (정상):
```cpp
c = ||Σ X_j - 10·X_i||²
```

### 주요 차이점

| 항목 | Original LOAM | 현재 구현 (버그) | 영향 |
|--------|---------------|-----------------|--------|
| **정규화(Normalization)** | 없음 | `||X_i||`로 나눔 | 거리 편향 발생 |
| **지수(Exponent)** | 제곱 (`²`) | 단일 지수 (`¹`) | 스케일 불일치 |
| **단위(Units)** | m² | 무차원(Dimensionless) | 임계값 호환 불가 |
| **거리 의존성** | 있음 (r²에 비례) | 없음 (정규화로 제거됨) | 먼 거리의 엣지 누락 |

### 영향

- ❌ **원거리 지점 편향**: 먼 거리의 엣지(예: 50m 밖의 건물 모서리) 곡률이 인위적으로 낮게 계산됨
- ❌ **근거리 지점 편향**: 가까운 평면이 엣지로 잘못 분류될 수 있음
- ❌ **정합(Registration) 품질 저하**: 잘못된 특징점으로 인해 부정확한 대응 관계 형성
- ❌ **일관성 없는 동작**: 장면의 깊이에 따라 특징점 품질이 달라짐

---

## ✅ 해결 방법

### 수정할 파일

- `src/loam_feature.cpp`
  - `extract_loam_features_ring_based()` (122-135라인)
  - `extract_loam_features_knn()` (230-246라인)

### 코드 변경

**기존 코드** (버그):
```cpp
Eigen::Vector3d diff_sum = Eigen::Vector3d::Zero();
for (int j = -5; j <= 5; j++) {
    if (j == 0) continue;
    diff_sum += (X_i - X_j);  // ❌
}
double c = diff_sum.norm() / (10 * X_i.norm());  // ❌
```

**수정 코드** (정상):
```cpp
Eigen::Vector3d diff_vec = -10.0 * X_i;  // 중심점 가중치 적용
for (int j = -5; j <= 5; j++) {
    if (j == 0) continue;
    diff_vec += X_j;  // ✓
}
double c = diff_vec.squaredNorm();  // ✓
```

### 수정 후 기대 결과

- ✅ 곡률 값이 original LOAM과 일치함
- ✅ 거리에 따른 특징점 검출 특성 복구
- ✅ 정합 정확도 향상
- ✅ 깊이 범위 전체에서 일관된 동작

---

## 🔍 근거 및 참고 문헌

### 소스 코드 참조

| 구현체 | 저장소 | 파일 | 라인 |
|----------------|------------|------|------|
| **Original LOAM** | [laboshinl/loam_velodyne](https://github.com/laboshinl/loam_velodyne) | `BasicScanRegistration.cpp` | 291 |
| **LIO-SAM** | [TixiaoShan/LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) | `featureExtraction.cpp` | 67 |
| **현재 구현 (버그)** | 이 저장소 | `src/loam_feature.cpp` | 122-135 |

### 학술 논문

1. **LOAM**: Zhang, J., & Singh, S. (2014). "LOAM: Lidar Odometry and Mapping in Real-time." *RSS 2014*.  
   📄 [PDF](https://www.ri.cmu.edu/pub_files/2014/7/Ji_LidarMapping_RSS2014_v8.pdf)

2. **LIO-SAM**: Shan, T., et al. (2020). "LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping." *IROS 2020*.

---

## 📊 빠른 시각적 참조

```
┌────────────────────────────────────────────────────────┐
│                    곡률 공식 비교                      │
├────────────────────────────────────────────────────────┤
│                                                        │
│ Original LOAM:                                         │
│   c = ||Σ X_j - 10·X_i||²                            │
│   • 제곱 노름(Squared norm)                            │
│   • 정규화 없음 (NO normalization)                      │
│   • 단위: m²                                           │
│   ✓ 정상 (CORRECT)                                     │
│                                                        │
│ LIO-SAM:                                               │
│   c = (Σ range_j - 10·range_i)²                      │
│   • 1D 거리 (최적화됨)                                 │
│   • 여전히 제곱 형태                                   │
│   • 단위: m²                                           │
│   ✓ 정상 (CORRECT)                                     │
│                                                        │
│ 현재 구현:                                             │
│   c = ||10·X_i - Σ X_j|| / (10·||X_i||)             │
│   • 제곱 아님                                          │
│   • ||X_i||로 정규화됨                                 │
│   • 단위: 무차원                                       │
│   ✗ 오류 (WRONG) - 거리 편향 발생!                     │
│                                                        │
└────────────────────────────────────────────────────────┘
```

---

## 🧪 검증

### 수정 전

베이스라인 테스트 실행:
```bash
# 테스트 스캔에서 특징점 추출
./build/loam_feature_test --scan test_data/sample.pcd

# 기대 결과: 곡률 값이 0.01-0.1 범위 내 (무차원)
# 증상: 먼 거리의 엣지가 평면과 유사하게 낮은 곡률을 가질 수 있음
```

### 수정 후

정확성 확인:
```bash
# 동일한 테스트 스캔
./build/loam_feature_test --scan test_data/sample.pcd

# 기대 결과: 곡률 값이 0.001-1.0 m² 범위 내 (거리에 따라 스케일링됨)
# 결과: 먼 거리의 엣지가 비례적으로 더 높은 곡률을 가져야 함
```

### 단위 테스트 (권장)

`test/test_loam_feature.cpp`에 회귀 테스트 추가:
```cpp
TEST(LOAMCurvature, DistanceInvariance) {
    // 기하학적으로 유사한 특징점이 서로 다른 거리에 있을 때,
    // 곡률이 거리²에 비례하는지 확인
}

TEST(LOAMCurvature, MatchesReferenceLOAM) {
    // 레퍼런스 구현체와 비교
}
```

---

## 📈 다음 단계

1. **검토**: 기술적 세부 사항은 `loam_curvature_bug_analysis.md`를 읽어보세요.
2. **시각화**: `loam_curvature_diagrams.md`의 다이어그램을 확인하거나 `loam_curvature_uml.puml`을 렌더링해 보세요.
3. **수정 적용**: 위에서 설명한 대로 `src/loam_feature.cpp`를 수정하세요.
4. **테스트**: 수정 전후의 검증 테스트를 실행하세요.
5. **재조정**: 엣지/평면 선택 백분위수(percentiles)를 조정해야 할 수도 있습니다.
6. **배포**: 이 분석 내용을 참고하여 변경 사항을 커밋하세요.

---

## 📞 질문이 있으신가요?

다음을 참조하세요:
- **기술적 세부 사항**: `loam_curvature_bug_analysis.md` 섹션 2 (수학적 분석)
- **시각적 설명**: `loam_curvature_diagrams.md` 섹션 "한 장으로 보는 버그"
- **수정 지침**: `loam_curvature_bug_analysis.md` 섹션 4 (올바른 구현)
- **마이그레이션**: `loam_curvature_bug_analysis.md` 섹션 6 (마이그레이션 가이드)

---

## 📝 문서 이력

| 날짜 | 이벤트 |
|------|-------|
| 2026-02-10 | Sisyphus AI에 의해 초기 분석 완료 |
| 2026-02-10 | 현재 구현에서 버그 식별 |
| 2026-02-10 | LOAM/LIO-SAM 레퍼런스 구현과 비교 |
| 2026-02-10 | 포괄적인 문서 및 UML 다이어그램 생성 |
| 2026-02-10 | 검증 전략과 함께 해결 방법 제공 |

---

**ULTRAWORK MODE에서 분석 완료** 🚀
