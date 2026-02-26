# Documentation

Bottom-LiDAR-Docker 프로젝트 문서입니다.

---

## 문서 구조

```
docs/
├── README.md                          # 이 파일
├── benchmark/                         # 벤치마크 결과 및 분석
│   ├── BENCHMARK_ANALYSIS.md          # 6-Factor 심층 분석 (790줄)
│   ├── BENCHMARK_REPORT.md            # 벤치마크 + 버그 수정 결과
│   └── benchmark_results.md           # NDT 해상도 실험 결과
├── changelog/                         # 변경내역 (날짜순)
│   ├── 변경내역_headless_benchmark.md  # Headless 모드 + NDT 통합
│   ├── 변경내역_ndt_magnusson_refactoring.md  # NDT Magnusson 리팩토링
│   └── 변경내역_NDT_VGICP_개선_2026-02-26.md  # NDT/VGICP 수렴 개선
├── loam/                              # LOAM / LIO-SAM 관련
│   └── liosam_feature_extraction_integration.md  # LIO-SAM 통합 및 7-Factor 분석
├── ndt/                               # NDT Factor 문서
│   ├── README.md                      # NDT 파일 위치 및 구현 설명
│   ├── uml.md                         # UML 다이어그램 (PlantUML 6종)
│   ├── usage_guide.md                 # NDT 사용 가이드
│   ├── omp_comparison.md              # NDT vs ndt_omp 비교
│   ├── magnusson_implementation_comparison_ko.md  # Magnusson 논문 구현 비교
│   ├── weighted_gauss_newton_feasibility_ko.md    # Weighted Gauss-Newton 타당성
│   ├── NDT_CODE_MATH_MAPPING.md       # NDT 코드↔수식 1:1 매핑
│   └── NDT_LM_INCOMPATIBILITY_ANALYSIS.md  # NDT vs LM 비호환성 분석
└── archive/                           # 과거 문서 아카이브
    ├── ARCHIVE_STRUCTURE.md           # 아카이브 구조 설명
    ├── 2026-01/                       # 2026년 1월 (3개)
    └── 2026-02/                       # 2026년 2월 (17개)
```

---

## 벤치마크 (`docs/benchmark/`)

Factor별 정합 성능을 측정하고 분석한 결과물입니다.

| 문서 | 설명 | 대상 독자 |
|------|------|-----------|
| **[BENCHMARK_ANALYSIS.md](./benchmark/BENCHMARK_ANALYSIS.md)** | 6개 Factor 심층 비교 분석 (Translation/Rotation 오차, 수렴 속도) | 분석가 |
| **[BENCHMARK_REPORT.md](./benchmark/BENCHMARK_REPORT.md)** | 벤치마크 결과 요약 + 버그 수정 전후 비교 | 개발자 |
| **[benchmark_results.md](./benchmark/benchmark_results.md)** | NDT 해상도별 실험 결과 (ndt_resolution 파라미터 스터디) | 개발자 |

---

## 변경내역 (`docs/changelog/`)

코드 변경 사항을 날짜순으로 기록합니다.

| 문서 | 날짜 | 주요 내용 |
|------|------|-----------|
| **[변경내역_NDT_VGICP_개선_2026-02-26.md](./changelog/변경내역_NDT_VGICP_개선_2026-02-26.md)** | 2026-02-26 | NDT score_function 리팩토링, VGICP error() 오버라이드, Voxel 해상도 1.0m |
| **[변경내역_ndt_magnusson_refactoring.md](./changelog/변경내역_ndt_magnusson_refactoring.md)** | 2026-02-25 | NDT Magnusson 논문 기반 리팩토링 |
| **[변경내역_headless_benchmark.md](./changelog/변경내역_headless_benchmark.md)** | 2026-02-23 | Headless 벤치마크 모드 추가, NDT Factor 통합 |

---

## LOAM / LIO-SAM (`docs/loam/`)

| 문서 | 설명 | 대상 독자 |
|------|------|-----------|
| **[liosam_feature_extraction_integration.md](./loam/liosam_feature_extraction_integration.md)** | LIO-SAM FeatureExtraction 이식, 코드 변경점, 7-Factor 벤치마크 | 개발자, 분석가 |

---

## NDT Factor (`docs/ndt/`)

NDT(Normal Distributions Transform) Factor를 gtsam_points 라이브러리 내부에 구현한 내용을 정리한 문서입니다.

| 문서 | 설명 | 대상 독자 |
|------|------|-----------|
| **[README.md](./ndt/README.md)** | NDT 소스 파일 위치, 구현 설명, 수학적 배경, 버그 수정 이력 | 개발자 (심화) |
| **[uml.md](./ndt/uml.md)** | 클래스 계층, 데이터 구조, 실행 시퀀스 등 PlantUML 6종 | 개발자 (아키텍처) |
| **[usage_guide.md](./ndt/usage_guide.md)** | NDT Factor 사용법, 파라미터 설명, main.cpp 적용 예시 | 사용자 (시작하기) |
| **[omp_comparison.md](./ndt/omp_comparison.md)** | gtsam_points NDT vs pcl_ndt_omp 비교 분석 | 개발자 (참고) |
| **[magnusson_implementation_comparison_ko.md](./ndt/magnusson_implementation_comparison_ko.md)** | Magnusson 논문 vs 구현 코드 비교 분석 | 연구자 |
| **[weighted_gauss_newton_feasibility_ko.md](./ndt/weighted_gauss_newton_feasibility_ko.md)** | Weighted Gauss-Newton 방법 gtsam_points 적용 타당성 | 연구자 |
| **[NDT_CODE_MATH_MAPPING.md](./ndt/NDT_CODE_MATH_MAPPING.md)** | NDT 코드 변수 ↔ Magnusson 수식 1:1 매핑 (796줄) | 연구자 (심화) |
| **[NDT_LM_INCOMPATIBILITY_ANALYSIS.md](./ndt/NDT_LM_INCOMPATIBILITY_ANALYSIS.md)** | NDT Score Function과 LM 옵티마이저 비호환성 분석 | 연구자 (심화) |

**읽기 순서 추천**:
1. `ndt/usage_guide.md` — 빠른 시작 및 파라미터 설명
2. `ndt/README.md` — 파일 위치, 구현 상세, 수학적 배경
3. `ndt/uml.md` — 구조 시각화 (6개 다이어그램)
4. `ndt/magnusson_implementation_comparison_ko.md` — 논문 vs 코드 비교
5. `ndt/NDT_CODE_MATH_MAPPING.md` — 수식 1:1 매핑 (심화)

---

## 아카이브 (`docs/archive/`)

과거 작업 결과물을 날짜별로 보관합니다. 자세한 내용은 [ARCHIVE_STRUCTURE.md](./archive/ARCHIVE_STRUCTURE.md) 참조.

### 2026년 1월 (archive/2026-01/) — 3개

| 문서 | 설명 |
|------|------|
| hyperparameters_tutorial.md | 하이퍼파라미터 튜토리얼 |
| quick_reference.md | 빠른 참조 가이드 |
| demo_matching_cost_factors_report.md | Matching Cost Factor 데모 리포트 |

### 2026년 2월 (archive/2026-02/) — 17개

| 주제 | 파일 수 | 설명 |
|------|---------|------|
| LOAM Curvature 버그 수정 | 5개 | LOAM 곡률 계산 버그 분석 및 수정 |
| Scan-to-Map 구현 | 3개 | Scan-to-Map 방식 구현 및 통합 |
| NDT Factor 이전 버전 | 9개 | NDT 초기 구현, 분석, 변경 이력 (→ `docs/ndt/`로 대체됨) |

---

## 프로젝트 파일 구조 (핵심)

```
/root/workdir/
├── src/
│   └── main.cpp                                           # 메인 코드 (NDT/GICP/VGICP 사용)
├── thirdparty/gtsam_points/
│   ├── include/gtsam_points/factors/
│   │   ├── integrated_ndt_factor.hpp                      # NDT Factor 헤더
│   │   ├── integrated_vgicp_factor.hpp                    # VGICP Factor 헤더
│   │   └── impl/
│   │       ├── integrated_ndt_factor_impl.hpp             # NDT Factor 템플릿 구현
│   │       └── integrated_vgicp_factor_impl.hpp           # VGICP Factor 템플릿 구현
│   └── src/gtsam_points/factors/
│       └── integrated_matching_cost_factor.cpp            # 공통 Matching Cost Factor
├── data/pcd/                                              # 점군 데이터 + Ground Truth
├── build/                                                 # 빌드 디렉토리
└── docs/                                                  # 문서 (이 폴더)
```

---

## 실행 방법

```bash
# Docker 컨테이너 내부에서
cd /root/workdir/build
make -j$(nproc)

# GUI 모드 (기본)
./lidar_registration_benchmark

# Headless 모드 (GUI 없이 6개 Factor 벤치마크)
./lidar_registration_benchmark --headless
```

**UI 사용법** (GUI 모드):
- `factor type`: Point-to-Point / Point-to-Plane / GICP / VGICP / LOAM / NDT / **LOAM_LIOSAM** 선택
- `optimizer type`: LM / ISAM2 선택
- `optimize`: 최적화 실행
- `noise_scale` / `add noise`: 초기 포즈 노이즈 조정
- `full connection`: 전체/인접 프레임 연결 선택

---

## 참고 자료

- **gtsam_points**: https://github.com/koide3/gtsam_points
- **GTSAM**: https://gtsam.org/
- **Iridescence**: https://github.com/koide3/iridescence
- **spdlog**: https://github.com/gabime/spdlog

---

생성일: 2026-01-26  
최종 수정: 2026-02-26
