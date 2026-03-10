# LiDAR Factor 5회 반복 실험 상세 분석 (2026-03-10, threads=8)

## 1) 분석 범위와 근거

- 실험 입력: `bench_all_threads8_5runs_summary.json` (headless, threads=8, 5회 반복)
- 코드 경로 기준:
  - factor 생성/파라미터: `src/main.cpp:552`
  - LM 실행 파라미터/콜백: `src/main.cpp:687`
  - 전체 factor 일괄 벤치 루프: `src/main.cpp:849`
- 신규 GMM_NDT 설정 기준:
  - `num_components=2`, `gamma_min=1e-4`, `freeze_mixture_in_lm=false`
  - 코드: `src/main.cpp:644`
- 해석 방식:
  - 관측치(수치) -> LM 관점 root cause -> 실제 코드베이스에서 가능한 개선안 -> 재검증 지표
  - Oracle 검토(`bg_51c8e50e`) + 외부 레퍼런스(`bg_a3af6dbf`) 반영

## 2) 5회 반복 집계 요약

| Factor | Mean T (m) | Mean R (deg) | Max T (m) | Max R (deg) | ms(avg±std) | final_iter | lm_calls | neg_cost_count | max_lambda |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Point-to-Point | 0.100990 | 0.436051 | 0.204713 | 0.687970 | 1820.0 ± 29.2 | 26 | 26 | 0 | 1e-6 |
| Point-to-Plane | 0.061799 | 0.430725 | 0.127270 | 0.852305 | 1178.4 ± 30.9 | 15 | 15 | 0 | 1e-6 |
| GICP | 0.085399 | 0.499411 | 0.168046 | 1.047266 | 1047.8 ± 34.8 | 9 | 9 | 0 | 1e-6 |
| VGICP | 0.117390 | 0.635614 | 0.202933 | 1.177762 | 1322.6 ± 8.7 | 12 | 23 | 12 | 1e5 |
| NDT | 0.105657 | 0.730808 | 0.191520 | 1.320227 | 5664.8 ± 111.1 | 70 | 70 | 0 | 1e-6 |
| LightNDT | 0.159027 | 0.640614 | 0.337532 | 0.960458 | 650.2 ± 31.1 | 6 | 16 | 11 | 1e4 |
| GMM_NDT | 0.099698 | 0.771133 | 0.174744 | 1.370901 | 9635.0 ± 288.8 | 66 | 66 | 0 | 1e-6 |
| LOAM_LIOSAM | 0.225414 | 0.774622 | 0.616540 | 1.429813 | 57.8 ± 1.9 | 8 | 8 | 0 | 1e-6 |

핵심 관측:
- 정확도(Mean T): Point-to-Plane > GICP > GMM_NDT > Point-to-Point > NDT > VGICP > LightNDT > LOAM_LIOSAM
- 속도(ms): LOAM_LIOSAM >> LightNDT > GICP > Point-to-Plane > VGICP > Point-to-Point > NDT > GMM_NDT
- LM 비정상 신호:
  - VGICP: `neg_cost_count=12`, `max_lambda=1e5` (신뢰영역 반복 축소)
  - LightNDT: `neg_cost_count=11`, `max_lambda=1e4`

## 3) Factor별 상세 진단 (원인/문제/개선/기대효과)

### 3.1 Point-to-Point

- 관측:
  - 정확도는 중간권, 반복 수 26, 시간 1.82s
- Root cause:
  - 등방성 거리 최소화 특성상 구조적 방향(평면 접선/축 방향) 제약이 약함
- 문제점:
  - 대칭/장복도 환경에서 관측가능성이 약한 축으로 미끄러짐 가능
- 개선안:
  1. 노말/곡률 기반 correspondence 가중치 적용
  2. Point-to-Plane과 하이브리드 구성(초기 안정화 후 전환)
- 기대 개선:
  - 회전 안정성, 반복 수 감소
- 검증 지표:
  - Mean R 감소, final_iter 감소, 씬 유형별 편차 감소

### 3.2 Point-to-Plane

- 관측:
  - Mean T 최상(0.061799), 반복 15, 시간 1.18s
- Root cause:
  - 평면 노말 방향 구속이 강해 병진 수렴이 유리
- 문제점:
  - 노말 품질/방향 일관성 저하 시 회전 오차 튐 가능
- 개선안:
  1. 노말 신뢰도(이웃 수, curvature) 기반 가중치
  2. 노말 방향 일관성 보정(view direction 기반)
- 기대 개선:
  - 회전 분산 감소, outlier 프레임 안정화
- 검증 지표:
  - Max R 감소, 프레임별 회전 오차 tail 축소

### 3.3 GICP

- 관측:
  - 속도-정확도 균형 최고권(1.05s, Mean T 0.085)
- Root cause:
  - 공분산 기반 이방성 거리로 point-only 대비 조건수 개선
- 문제점:
  - 퇴화 공분산(평면/선형 구조) 시 수치 불안정 잠재
- 개선안:
  1. 정보행렬 SPD 보장(eigenvalue floor)
  2. correspondence 급변 억제(업데이트 tolerance 보수화)
- 기대 개선:
  - 난환경에서 lambda 급등 억제, 재현성 향상
- 검증 지표:
  - max_lambda, neg_cost_count 추적(현재 0 유지 여부)

### 3.4 VGICP

- 관측:
  - `neg_cost_count=12`, `lm_calls(23) > final_iter(12)`, `max_lambda=1e5`
  - 반복 거부/재시도 패턴이 명확
- Root cause (LM 관점):
  - 실제 비용 감소(`cost_d`)와 선형화 예측 감소가 자주 불일치
  - voxel 공분산 품질/가중 일관성/야코비안 일치성 중 하나 이상 취약 가능
- 문제점:
  - damping이 상한까지 올라가며 실질 탐색 반경 축소 -> 수렴 지연/정확도 저하
- 개선안(우선순위 높음):
  1. voxel 유효성 강화(min points per voxel)
  2. 공분산 SPD 강제(대칭화 + eigen floor)
  3. `pred_d` vs `cost_d` 계측 추가(oracle 권고)
  4. 필요 시 factor 단위 Jacobian finite-diff 검증 경로 추가
- 기대 개선:
  - step reject 감소, lm_calls 감소, Mean T/R 개선
- 검증 지표:
  - `neg_cost_count` 12 -> 0~2 수준, `max_lambda` 1e5 -> 1e2 이하 목표

### 3.5 NDT

- 관측:
  - 반복 70, 시간 5.66s로 고비용
- Root cause:
  - voxel 검색 + 가우시안 평가 비용이 반복마다 큼
  - 단일 해상도에서 basin 진입이 느려 반복 수가 큼
- 문제점:
  - 실시간성 부족, 데이터셋 스케일 변화에 민감
- 개선안(코드베이스 친화):
  1. coarse-to-fine 다중 해상도 스케줄(기존 CLI 추가 없이 내부 stage)
  2. voxel 통계 캐시 재사용
  3. LM 종료 기준에 step/gradient 기반 보완
- 기대 개선:
  - 반복 수/시간 동시 감소
- 검증 지표:
  - final_iter 70 -> 30~45 목표, ms 5.6s -> 2~3.5s 목표

### 3.6 LightNDT

- 관측:
  - 매우 빠름(0.65s) but 정확도 열위, `neg_cost_count=11`, `max_lambda=1e4`
- Root cause:
  - 경량화 근사로 계산량은 줄었지만 선형화-실비용 정합성이 깨지는 구간 존재
- 문제점:
  - 빠르지만 trust-region thrashing으로 강건성이 낮아질 수 있음
- 개선안:
  1. `cost_d<0` 발생 시 correspondence/voxel assignment 1회 freeze
  2. 해당 반복에서만 full NDT fallback(guarded fallback)
- 기대 개선:
  - 속도 일부 희생으로 안정성/정확도 상승
- 검증 지표:
  - neg_cost_count 11 -> 3 이하, Mean T 개선

### 3.7 GMM_NDT

- 관측:
  - NDT 대비 Mean T 개선(0.105657 -> 0.099698)
  - 회전 악화(0.730808 -> 0.771133), 시간 급증(5.66s -> 9.64s)
- Root cause:
  - 혼합모델 책임도(gamma) 누적이 병진 쪽 gradient를 강화하나, 회전 스케일 균형이 충분치 않을 가능성
  - component 합산 비용 자체가 커서 평가당 연산량이 큼
- 구현 근거:
  - gamma 계산(log-sum-exp), gamma_min 클램프, top-K component 누적
  - 코드: `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_gmm_ndt_factor_impl.hpp:116`
- 문제점:
  - 현재 기본값(`num_components=2`, `freeze_mixture_in_lm=false`)에서 LM 반복 내 mixture 변동 가능성
  - 회전 제약 대비 병진 우세로 보이는 지표
- 개선안(우선순위 높음):
  1. 단계별 mixture freeze 정책 도입(초기 n회 freeze=true, 후반 false)
  2. 회전/병진 스케일 밸런싱(정보행렬 단위 정규화)
  3. 적응형 component pruning(낮은 gamma 컷)
  4. coarse-to-fine + GMM 결합(외부 CLI 변경 없이 내부 stage)
- 기대 개선:
  - 회전 오차 회복, 시간 단축, NDT 대비 우위 명확화
- 검증 지표:
  - Mean R 0.77 -> 0.68 이하, ms 9.6s -> 6~7.5s 목표

### 3.8 LOAM_LIOSAM

- 관측:
  - 압도적 속도(57.8ms), 정확도 최하
- Root cause:
  - edge/plane sparse feature 기반이라 제약 밀도가 낮고 씬 의존성 큼
- 문제점:
  - 단독 최종 정합으로 쓰면 드리프트/편향 위험
- 개선안:
  1. 최종 목적함수 대신 초기화(초기 pose seed)로 활용
  2. 후단에 dense factor(Point-to-Plane/GICP/NDT) 결합
- 기대 개선:
  - 전체 파이프라인 속도 유지 + 정확도 보완
- 검증 지표:
  - 2-stage 파이프라인에서 total ms/Mean T trade-off 비교

## 4) 교차 이슈: LM 신뢰도 계측 항목 (즉시 적용 권장)

반복 거부가 관측된 VGICP/LightNDT를 중심으로, 아래를 로깅해야 원인 분리가 가능하다.

1. `cost_prev`, `cost_new`, `cost_d`
2. `pred_d` (선형화 예측 감소량)
3. `rho = cost_d / pred_d`
4. iteration별 correspondence 변경량

판정 규칙:
- `pred_d > 0`인데 `cost_d < 0`가 잦으면, 단순 난데이터보다 Jacobian/weight 일관성 문제 가능성이 큼
- `max_lambda`가 상한에 반복 도달하면 trust-region 축소 루프가 진행 중

## 5) 개선 우선순위 로드맵

### P0 (바로 적용)
- VGICP/LightNDT: SPD 보강 + `cost_d/pred_d/rho` 계측 + correspondence 급변 억제
- GMM_NDT: mixture freeze 스케줄(초기 freeze) + gamma pruning

### P1 (단기)
- NDT/GMM_NDT 내부 coarse-to-fine stage 도입 (CLI 확장 없이 내부 실행)
- 회전/병진 단위 밸런스 재가중

### P2 (중기)
- LOAM seed + dense factor refinement 하이브리드 벤치 라인 추가

## 6) 결론

- 현재 기준 최적의 균형은 `GICP`와 `Point-to-Plane`.
- `VGICP`/`LightNDT`는 빠르거나 잠재력이 있지만 LM 일관성 이슈(음수 `cost_d`, lambda 급등) 해결이 선행돼야 한다.
- `NDT`는 안정적이나 느리고, `GMM_NDT`는 병진 개선을 확인했지만 회전/시간이 아직 손해다.
- 다음 라운드의 핵심은 "정확도 알고리즘 변경"보다 "LM-요소 일관성(예측 vs 실제 감소량) 복원"이다.
