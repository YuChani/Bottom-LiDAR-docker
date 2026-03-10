# Documentation

Bottom-LiDAR-Docker 문서를 주제별로 재정리한 인덱스입니다.

---

## 문서 구조 (주제 기준)

```text
docs/
├── README.md
├── benchmark/                         # 벤치마크 결과/분석
├── ndt/                               # NDT 핵심 문서 + 세부 주제 폴더
│   ├── analysis/                      # 원인 분석/수학 해설
│   ├── comparison/                    # NDT vs LightNDT/GICP 비교
│   ├── experiments/                   # 실험 결과 보고서
│   └── checklists/                    # 실행 체크리스트
├── gmm/                               # GMM-NDT 연구/작업지시
├── loam/                              # LOAM/LIO-SAM 관련
├── changelog/                         # 변경 이력
└── archive/                           # 월별 아카이브
```

---

## 1) 벤치마크 (`docs/benchmark/`)

| 문서 | 설명 |
|---|---|
| [BENCHMARK_ANALYSIS.md](./benchmark/BENCHMARK_ANALYSIS.md) | Multi-Factor 심층 분석 |
| [BENCHMARK_REPORT.md](./benchmark/BENCHMARK_REPORT.md) | 벤치마크 요약 + 수정 결과 |
| [factor_5run_threads8_상세분석_2026-03-10.md](./benchmark/factor_5run_threads8_상세분석_2026-03-10.md) | **5회 반복(threads=8) factor별 원인/문제/개선 상세 분석** |
| [benchmark_results.md](./benchmark/benchmark_results.md) | NDT 해상도별 결과 |
| [main_정합알고리즘_수학검증_및_headless결과_2026-03-04.md](./benchmark/main_정합알고리즘_수학검증_및_headless결과_2026-03-04.md) | main 경로 중심 검증/결과 |

---

## 2) NDT (`docs/ndt/`)

### 2.1 핵심 구현/가이드

| 문서 | 설명 |
|---|---|
| [README.md](./ndt/README.md) | NDT 구현 개요/파일 역할 |
| [usage_guide.md](./ndt/usage_guide.md) | 사용 가이드 |
| [uml.md](./ndt/uml.md) | 구조 다이어그램 |
| [NDT_CODE_MATH_MAPPING.md](./ndt/NDT_CODE_MATH_MAPPING.md) | 코드-수식 1:1 매핑 |
| [NDT_LM_INCOMPATIBILITY_ANALYSIS.md](./ndt/NDT_LM_INCOMPATIBILITY_ANALYSIS.md) | NDT-LM 상호작용 분석 |
| [magnusson_implementation_comparison_ko.md](./ndt/magnusson_implementation_comparison_ko.md) | 논문 대비 구현 비교 |
| [weighted_gauss_newton_feasibility_ko.md](./ndt/weighted_gauss_newton_feasibility_ko.md) | WGN 적용 타당성 |
| [scoreless_weight_only_method_ko.md](./ndt/scoreless_weight_only_method_ko.md) | scoreless 방식 정리 |
| [omp_comparison.md](./ndt/omp_comparison.md) | ndt_omp 비교 |

### 2.2 분석 문서 (`docs/ndt/analysis/`)

| 문서 | 설명 |
|---|---|
| [NDT_성능분석_보고서.md](./ndt/analysis/NDT_성능분석_보고서.md) | NDT 성능 분석 |
| [why_ndt_is_slow_in_gtsam_lm_h1only_상세분석_ko.md](./ndt/analysis/why_ndt_is_slow_in_gtsam_lm_h1only_상세분석_ko.md) | NDT 저속 원인 상세 |
| [why_lightndt_is_fast_수학구조_상세분석_ko.md](./ndt/analysis/why_lightndt_is_fast_수학구조_상세분석_ko.md) | LightNDT 고속 원인 상세 |
| [ndt_수렴속도_재검증_원인분석_2026-03-04.md](./ndt/analysis/ndt_수렴속도_재검증_원인분석_2026-03-04.md) | 수렴속도 재검증 |
| [ndt_weight_sign_및_수렴속도_상세해설_ko.md](./ndt/analysis/ndt_weight_sign_및_수렴속도_상세해설_ko.md) | weight/sign 해설 |
| [ndt_hessian_weight_only_analysis_ko.md](./ndt/analysis/ndt_hessian_weight_only_analysis_ko.md) | Hessian weight-only 분석 |
| [J_target_J_source_수학_상세해설_2026-03-05.md](./ndt/analysis/J_target_J_source_수학_상세해설_2026-03-05.md) | Jacobian 블록 해설 |
| [NDT_수학_코드_7문항_상세분석_2026-03-09.md](./ndt/analysis/NDT_수학_코드_7문항_상세분석_2026-03-09.md) | d1/d2/d3, Jacobian 4×6, weight, LM 불일치 등 7문항 상세 분석 |
| [NDT_Hessian_H1_H2_H3_수학_분석_2026-03-09.md](./ndt/analysis/NDT_Hessian_H1_H2_H3_수학_분석_2026-03-09.md) | **H1/H2/H3 Hessian 수학적 분해** — PSD 조건, 수렴 효과, GTSAM 호환성, 달팽이 수렴 원인 |

### 2.3 비교 문서 (`docs/ndt/comparison/`)

| 문서 | 설명 |
|---|---|
| [light_ndt_vs_ndt_구현_수학_비교_정리.md](./ndt/comparison/light_ndt_vs_ndt_구현_수학_비교_정리.md) | LightNDT vs NDT 비교 |
| [NDT_vs_LightNDT_코드구조_수학차이_상세분석_2026-03-06.md](./ndt/comparison/NDT_vs_LightNDT_코드구조_수학차이_상세분석_2026-03-06.md) | 코드구조/수학 차이 |
| [ndt_lightndt_gicp_수학_컨셉_비교.md](./ndt/comparison/ndt_lightndt_gicp_수학_컨셉_비교.md) | NDT/LightNDT/GICP 컨셉 비교 |
| [why_ndt_vs_lightndt_운영의사결정_가이드_ko.md](./ndt/comparison/why_ndt_vs_lightndt_운영의사결정_가이드_ko.md) | 운영 의사결정 가이드 |

### 2.4 실험/체크리스트

| 문서 | 설명 |
|---|---|
| [full_project_factor_실험_수학코드_종합보고서_2026-03-06.md](./ndt/experiments/full_project_factor_실험_수학코드_종합보고서_2026-03-06.md) | 전체 factor 종합 실험/코드 분석 |
| [NDT_Conditional_H2_최종보고서_2026-03-09.md](./ndt/experiments/NDT_Conditional_H2_최종보고서_2026-03-09.md) | **NDT H2 Hessian 최적화 최종 보고서** (iter 67.6%↓) |
| [NDT_P0_3패치_적용결과_보고서_2026-03-06.md](./ndt/experiments/NDT_P0_3패치_적용결과_보고서_2026-03-06.md) | P0 패치 3개 적용 결과 |
| [ndt_속도개선_매트릭스_실험_2026-03-05.md](./ndt/experiments/ndt_속도개선_매트릭스_실험_2026-03-05.md) | NDT 매트릭스 실험 |
| [factor_개선_실행체크리스트_2026-03-06.md](./ndt/checklists/factor_개선_실행체크리스트_2026-03-06.md) | 실행 체크리스트 |

---

## 3) GMM (`docs/gmm/`)

| 문서 | 설명 |
|---|---|
| [NDT_GMM_연구보고서_방향_TODO_2026-03-06.md](./gmm/NDT_GMM_연구보고서_방향_TODO_2026-03-06.md) | GMM-NDT 연구 방향/TODO |
| [GMM_NDT_구현_작업지시서.md](./gmm/GMM_NDT_구현_작업지시서.md) | 구현 작업지시서 |

---

## 4) LOAM (`docs/loam/`)

| 문서 | 설명 |
|---|---|
| [liosam_feature_extraction_integration.md](./loam/liosam_feature_extraction_integration.md) | LIO-SAM feature 통합/분석 |

---

## 5) 변경이력 (`docs/changelog/`)

날짜별 코드/문서 변경 기록을 보관합니다.

---

## 6) 아카이브 (`docs/archive/`)

이전 버전 문서는 월별(`2026-01`, `2026-02`)로 보관합니다.

---

## 문서 추가 규칙

- NDT 분석 문서 -> `docs/ndt/analysis/`
- NDT 비교 문서 -> `docs/ndt/comparison/`
- NDT 실험 결과 -> `docs/ndt/experiments/`
- 실행 체크리스트 -> `docs/ndt/checklists/`
- GMM 관련 문서 -> `docs/gmm/`
- 변경 내역 -> `docs/changelog/`

---

최종 수정: 2026-03-09
