# Bottom-LiDAR-Docker 문서 아카이브 구조

**정리 날짜**: 2026-02-19  
**목적**: 날짜별 문서 정리 및 최신 문서 접근성 향상

---

## 현재 문서 구조

```
docs/
├── README.md                          # 프로젝트 문서 메인
├── ARCHIVE_STRUCTURE.md               # 이 파일
├── ndt/                               # NDT Factor 문서 (최신, 2026-02-19)
│   ├── README.md                      # NDT 파일 위치 및 구현 설명
│   ├── uml.md                         # UML 다이어그램 (PlantUML 6종)
│   ├── usage_guide.md                 # NDT 사용 가이드
│   └── omp_comparison.md              # NDT vs ndt_omp 비교
└── archive/
    ├── 2026-01/                       # 2026년 1월 문서 (3개)
    │   ├── hyperparameters_tutorial.md
    │   ├── quick_reference.md
    │   └── demo_matching_cost_factors_report.md
    └── 2026-02/                       # 2026년 2월 문서 (17개)
        ├── (LOAM Curvature 관련 5개)
        ├── (Scan-to-Map 관련 3개)
        └── (NDT 이전 버전 관련 9개)
```

---

## 메인 디렉토리 문서 (최신)

### NDT Factor 문서 (`docs/ndt/`, 2026-02-19)

gtsam_points 라이브러리 내부에 NDT Factor를 구현한 최종 결과물입니다.

| 문서 | 설명 | 대상 독자 |
|------|------|-----------|
| **[ndt/README.md](./ndt/README.md)** | NDT 소스 파일 위치, 구현 설명, 수학적 배경, 버그 수정 이력 | 개발자 (심화) |
| **[ndt/uml.md](./ndt/uml.md)** | 클래스 계층, 데이터 구조, 실행 시퀀스 등 PlantUML 6종 | 개발자 (아키텍처) |
| **[ndt/usage_guide.md](./ndt/usage_guide.md)** | NDT Factor 사용법, 파라미터 설명, main.cpp 적용 예시 | 사용자 (시작하기) |
| **[ndt/omp_comparison.md](./ndt/omp_comparison.md)** | gtsam_points NDT vs pcl_ndt_omp 비교 분석 | 개발자 (참고) |

**읽기 순서 추천**:
1. `ndt/usage_guide.md` — 빠른 시작 및 파라미터 설명
2. `ndt/README.md` — 파일 위치, 구현 상세, 수학적 배경
3. `ndt/uml.md` — 구조 시각화 (6개 다이어그램)
4. `ndt/omp_comparison.md` — 다른 NDT 구현과의 비교

---

## 아카이브 문서

### 2026년 1월 (archive/2026-01/) — 3개

| 문서 | 날짜 | 설명 |
|------|------|------|
| hyperparameters_tutorial.md | 01-27 | Hyperparameter 튜토리얼 |
| quick_reference.md | 01-27 | 빠른 참조 가이드 |
| demo_matching_cost_factors_report.md | 01-26 | Matching Cost Factor 데모 리포트 |

**주제**: 초기 프로젝트 문서, 하이퍼파라미터 가이드

---

### 2026년 2월 (archive/2026-02/) — 17개

#### LOAM Curvature 버그 수정 (02-10) — 5개

| 문서 | 크기 | 설명 |
|------|------|------|
| README_loam_curvature_ko.md | 9.4KB | LOAM 곡률 계산 README |
| loam_curvature_bug_analysis_ko.md | 18KB | 버그 분석 보고서 |
| loam_curvature_diagrams_ko.md | 11KB | 다이어그램 설명 |
| loam_curvature_uml_ko.puml | 16KB | PlantUML 소스 |
| loam_feature_bug_fixes.md | 9.7KB | 버그 수정 내역 |

**주제**: LOAM Feature 추출의 곡률 계산 버그 수정 및 문서화

---

#### Scan-to-Map 구현 (02-11 ~ 02-12) — 3개

| 문서 | 날짜 | 크기 | 설명 |
|------|------|------|------|
| scan_to_map_implementation.md | 02-11 | 17KB | Scan-to-Map 구현 |
| scan_to_map_main_integration.md | 02-12 | 25KB | Main 통합 문서 |
| scan_to_map_analysis_ko.md | 02-12 | 19KB | 분석 보고서 |

**주제**: Scan-to-Map 방식 구현 및 Main 애플리케이션 통합

---

#### NDT Factor 이전 버전 (02-13 ~ 02-19) — 9개

| 문서 | 날짜 | 설명 | 상태 |
|------|------|------|------|
| ndt_implementation_ko.md | 02-13 | NDT 구현 초기 문서 (초안) | → `docs/ndt/README.md`로 대체 |
| ndt_factor_detailed_ko.md | 02-13 | NDT Factor 상세 설명 | → `docs/ndt/README.md`로 대체 |
| ndt_comprehensive_analysis.md | 02-13 | NDT 종합 분석 | → `docs/ndt/README.md`로 대체 |
| ndt_exponential_fix.md | 02-13 | NDT 지수 함수 버그 수정 | → `docs/ndt/README.md`에 통합 |
| ndt_performance_report.md | 02-13 | NDT 성능 보고서 | 아카이브 |
| ndt_structure_verification.md | 02-13 | NDT 구조 검증 | 아카이브 |
| ndt_factor_location_and_math_verification.md | 02-13 | NDT 파일 위치 및 수학 검증 | → `docs/ndt/README.md`로 대체 |
| integration_changelog.md | 02-13 | NDT Main 통합 변경사항 | → `docs/ndt/usage_guide.md`로 대체 |
| uml_diagrams.md | 02-13 | UML 다이어그램 (이전 버전) | → `docs/ndt/uml.md`로 대체 |

**참고**: 위 문서들은 이전 NDT 구현(프로젝트 로컬 `include/ndt/`)에 대한 문서입니다.
2026-02-19에 NDT를 gtsam_points 내부로 재구현하면서 `docs/ndt/` 폴더의 새 문서로 대체되었습니다.

---

## 문서 버전 관리 원칙

### 메인 디렉토리 (`docs/`, `docs/ndt/`)
- **최신 문서만 유지**: 현재 활발히 사용되는 문서
- **주제별 하위 폴더**: NDT 등 주요 주제는 전용 폴더
- **접근성 우선**: README.md에서 모든 문서 링크 제공
- **업데이트 주기**: 실시간 (작업 완료 시마다)

### 아카이브 (`docs/archive/YYYY-MM/`)
- **완료된 작업 문서**: 참고용으로 보관
- **날짜별 분류**: 연도-월 단위 디렉토리
- **삭제 금지**: 모든 히스토리 보존

### 아카이브 시점
다음 조건 중 하나 이상 만족 시 아카이브:
- 새 버전 문서로 대체됨
- 작업 완료 후 1주일 경과
- 동일 주제의 새 작업 시작
- 문서가 5개 이상 누적

---

## 빠른 찾기

### 사용자가 원하는 것별 문서 안내

#### "NDT를 사용하고 싶어요"
→ **[`ndt/usage_guide.md`](./ndt/usage_guide.md)** (빠른 시작, 파라미터 설명)

#### "NDT 구현을 깊이 이해하고 싶어요"
→ **[`ndt/README.md`](./ndt/README.md)** (파일 위치, 수학적 배경, 구현 상세)

#### "NDT 시스템 구조를 보고 싶어요"
→ **[`ndt/uml.md`](./ndt/uml.md)** (6개 PlantUML 다이어그램)

#### "NDT와 다른 구현을 비교하고 싶어요"
→ **[`ndt/omp_comparison.md`](./ndt/omp_comparison.md)** (gtsam_points NDT vs pcl_ndt_omp)

#### "LOAM 버그 수정 내역을 보고 싶어요"
→ **`archive/2026-02/loam_curvature_bug_analysis_ko.md`**

#### "Scan-to-Map 구현을 보고 싶어요"
→ **`archive/2026-02/scan_to_map_implementation.md`**

---

## 변경 이력

| 날짜 | 변경 내용 | 작업자 |
|------|----------|--------|
| 2026-02-19 | `docs/ndt/` 폴더 생성, NDT 최신 문서 4개 배치 | Sisyphus AI |
| 2026-02-19 | NDT 이전 문서 8개 `archive/2026-02/`로 이동 | Sisyphus AI |
| 2026-02-19 | `ndt_usage_guide.md`, `ndt_omp_comparison.md` → `ndt/` 폴더로 이동 | Sisyphus AI |
| 2026-02-19 | README.md, ARCHIVE_STRUCTURE.md 전면 갱신 | Sisyphus AI |
| 2026-02-13 | 문서 날짜별 정리, 아카이브 디렉토리 생성 | Sisyphus AI |
| 2026-02-13 | NDT 최신 문서 4개 메인 디렉토리 유지 | Sisyphus AI |
| 2026-02-13 | 이전 문서 archive/2026-01, archive/2026-02로 이동 | Sisyphus AI |

---

**문서 정리 완료**: 2026-02-19  
**메인 문서 수**: 6개 (README + ARCHIVE_STRUCTURE + NDT 4개)  
**아카이브 문서 수**: 20개 (2026-01: 3개, 2026-02: 17개)
