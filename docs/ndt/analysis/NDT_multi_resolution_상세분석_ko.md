# Multi-resolution NDT 상세 분석

## 0. 문서 목적

이 문서는 Bottom-LiDAR 프로젝트에서 NDT의 병목(높은 반복 수/긴 실행 시간)을 줄이기 위해
"coarse voxel -> fine voxel" 2단계 최적화(Multi-resolution NDT)를 어떻게 설계/적용할지 정리한다.

핵심 목표는 다음 3가지다.

1. 현재 코드/실험 근거를 기반으로 Multi-resolution NDT의 필요성을 설명
2. 단일 해상도 실험 실패 사례(예: `resolution=0.5`)를 재해석하여 다단계 적용 조건 제시
3. 실제 코드 수정 지점, 검증 계획, 리스크 완화 방안을 문서화

---

## 1. 현재 상태 요약 (코드/실험 근거)

## 1.1 현재 파이프라인

- 벤치마크는 `src/main.cpp`에서 팩터를 생성하고 LM으로 최적화한다.
- NDT 생성 지점: `src/main.cpp:622`
- 현재 NDT 설정:
  - `search_mode = DIRECT7` (`src/main.cpp:627`)
  - `regularization_epsilon = 1e-3` (`src/main.cpp:628`)
  - `correspondence_update_tolerance = (corr_rot, corr_trans)` (`src/main.cpp:629`)
- 복셀맵은 프레임 로딩 단계에서 단일 해상도 `1.0`으로 생성된다 (`src/main.cpp:284`).

즉, 현재 구조는 "단일 voxel 해상도 + 단일 단계 LM"이다.

## 1.2 성능 관찰

문서화된 반복 실험에서 NDT는 정확도는 양호하지만 시간/반복 수가 크다.

- 5회 평균 실험: NDT `2523.4 ms`, outer iter `74`
  - 근거: `docs/ndt/experiments/full_project_factor_실험_수학코드_종합보고서_2026-03-06.md:86`
- 동일 표에서 Point-to-Plane `516 ms`, LightNDT `324.2 ms`
  - 근거: 같은 문서 `:83`, `:87`

해석: NDT는 "정확도 문제"보다 "수렴 속도 문제"가 우선 과제다.

## 1.3 단일 해상도 변경 실험의 교훈

`resolution=1.0 -> 0.5` 단일 변경 실험에서 다음 결과가 관찰되었다.

- 시간: 32,956 ms -> 4,965 ms (속도 개선)
- 하지만 회전 오차: 0.510 deg -> 3.617 deg (정확도 급락)
- 근거: `docs/benchmark/benchmark_results.md:145`, `:146`

문서 분석에서는 그 이유를 `compute_ndt_params()`의 해상도 의존성으로 설명한다.

```text
c2 = outlier_ratio / (resolution^3)
```

해석:

- 해상도를 무조건 작게 하면, 점유/분포 파라미터가 바뀌어 비용 함수 형상 자체가 급격히 변할 수 있다.
- 따라서 "한 번에 fine으로 고정"은 위험하고, 단계 전환이 필요하다.

---

## 2. 왜 Multi-resolution NDT가 필요한가

Multi-resolution NDT는 단순 튜닝이 아니라 "최적화 문제의 난이도 조절" 전략이다.

## 2.1 직관

- Coarse stage: 비용 표면을 부드럽게 만들어 큰 초기 오차에서도 basin 진입을 유도
- Fine stage: 이미 basin 안에 들어온 상태에서 정밀 정합

## 2.2 현재 코드와의 연결점

현재 구조에서 NDT는 다음 특성을 가진다.

- 대응점 탐색은 복셀 기반 (`DIRECT7`)이며 해상도에 민감
- LM은 선형화 모델 충실도에 민감하고, correspondence 변화가 크면 step rejection이 증가
- NDT는 지수형 score/가중치 구조로 인해 반복 수가 늘어나는 경향이 있음
  - 참조: `docs/ndt/NDT_LM_INCOMPATIBILITY_ANALYSIS.md`

따라서 coarse -> fine로 문제를 분리하면,

1. 초기 큰 오차 구간의 불안정성을 coarse stage에 격리
2. fine stage는 작은 보정만 담당
3. 결과적으로 전체 wall-clock과 실패율을 함께 낮출 가능성이 높다.

---

## 3. 설계 원칙 (이 프로젝트 기준)

## 3.1 "해상도 전환 = 새 최적화 문제"로 취급

같은 optimizer 인스턴스에서 factor 내부 해상도만 바꾸는 방식은 피한다.

- 권장: stage 경계에서 graph/factor를 재구성하고 LM을 재시작
- 이유: correspondence/linearization/cache 불일치로 인한 불연속 거동 방지

## 3.2 고정해야 할 것과 바꿔야 할 것

고정:

- 동일 프레임/동일 graph topology
- 동일 소스/타깃 점군 전처리 규칙
- 동일 covariance regularization 규칙

변경(오직 stage 경계에서):

- voxel resolution
- NDT 관련 파라미터(필요 시 outlier_ratio/tolerance/search_mode)
- LM iteration budget

## 3.3 우선순위

1. main 파이프라인 2-pass 구현 (저위험)
2. 성능 검증/파라미터 스윕
3. 필요 시 factor 내부 다중 레벨화(고위험)

## 3.4 Stage Handoff 계약 (필수)

해상도 전환 시 아래 계약을 반드시 지킨다.

1. coarse 종료 포즈를 fine 초기값으로 전달
2. fine 단계는 "새 graph + 새 factor + 새 optimizer"로 시작
3. LM lambda/step 이력은 carry-over하지 않고 재초기화
4. coarse cost와 fine cost를 동일 스케일로 직접 비교하지 않음

이 계약이 없으면, "objective가 달라졌는데 같은 trust-region 이력으로 계속 진행"하는 문제가 발생한다.

---

## 4. 코드 수정 포인트 (파일 단위)

아래는 "지금 구조를 최대한 유지"하는 최소 변경 경로다.

## 4.1 `src/main.cpp`

### A) 데이터 구조 확장

- 단일 `voxelmaps` 외에 stage별 맵을 보관할 컨테이너 추가
- 예시:
  - `voxelmaps_coarse`
  - `voxelmaps_fine`

관련 기존 지점:

- 복셀맵 생성: `src/main.cpp:284`
- 벡터 멤버: `src/main.cpp:818`

### B) 내부 설정값 추가 (CLI 미확장)

- 사용자 요구사항에 따라 **CLI 옵션은 추가하지 않는다**.
- 대신 `MatchingCostFactorDemo` 내부 설정으로 관리한다.
  - `enable_multi_res_ndt`
  - `ndt_coarse_resolution`, `ndt_fine_resolution`
  - `ndt_coarse_max_iter`, `ndt_fine_max_iter`
- 기존 CLI(`--headless`, `--factor`, `--threads`, `--corr-rot`, `--corr-trans`)는 유지한다.

관련 기존 지점:

- 클래스 멤버 영역: `src/main.cpp:795`
- 기본 파라미터 초기화 영역: `src/main.cpp:361`

### C) 최적화 루프 분리

- 기존 `run_optimization()`은 단일 stage로 유지
- NDT 전용 `run_optimization_ndt_multires()` 추가
  1) coarse graph 구성 + LM
  2) coarse 결과 pose를 초기값으로 fine graph 재구성 + LM

관련 기존 지점:

- 최적화 함수: `src/main.cpp:647`
- factor graph 구성 루프: `src/main.cpp:657`, `src/main.cpp:662`
- headless 전체 실행 루프: `src/main.cpp:834`

### D) 결과 로그 확장

- coarse/fine stage별 시간/반복 수를 로그로 분리 출력
- 최종 비교표는 기존 형식 유지(회귀 방지)

관련 기존 지점:

- summary 출력: `src/main.cpp:786`
- 최종 비교표: `src/main.cpp:904`

## 4.2 NDT factor 파일 (`thirdparty/gtsam_points/...`)

1차 구현에서는 "수정하지 않는 것"을 권장한다.

- `integrated_ndt_factor.hpp`
- `impl/integrated_ndt_factor_impl.hpp`

이유:

- factor 내부에 다중 레벨 전환 로직을 넣으면 correspondence/cache/상태 동기화 난이도가 급증
- main에서 stage 경계 재구성으로 대부분의 이득을 먼저 검증 가능

---

## 5. 권장 초기 파라미터 (2-stage)

아래 값은 내부 실험/외부 구현 관례를 종합한 "시작점"이다.

| 항목 | Coarse Stage | Fine Stage |
|---|---:|---:|
| voxel resolution | 2.0 | 1.0 |
| max iterations | 20 | 30 |
| search mode | DIRECT7 | DIRECT7 (또는 DIRECT1 A/B) |
| corr tolerance | 현행값 유지 또는 완화 | 현행값 유지 |
| 역할 | basin 진입 | 정밀 보정 |

주의:

- 과거 단일 실험에서 `resolution=0.5`는 정확도 악화가 컸다.
- 따라서 fine을 바로 0.5로 고정하지 말고 1.0부터 시작해 검증 후 하향한다.

## 5.1 외부 구현 관찰 범위 (참고)

외부 사례에서 자주 보이는 대략적 범위는 다음과 같다.

- coarse resolution: 2.0~5.0
- fine resolution: 0.5~1.0
- coarse iter: 30~50
- fine iter: 10~20

단, Bottom-LiDAR에서는 기존 실험에서 `resolution=0.5` 단독 적용 부작용이 확인되었으므로,
"외부 기본값 그대로 복사"가 아니라 내부 데이터 특성 기준으로 재보정해야 한다.

---

## 6. 검증 계획 (실험 설계)

## 6.1 실험군

1. Baseline: 단일 NDT (현행)
2. Multi-res A: 2.0 -> 1.0
3. Multi-res B: 1.5 -> 1.0
4. Multi-res C(선택): 2.0 -> 1.0 -> 0.5

## 6.2 고정 조건

- 동일 데이터셋/동일 noise scale
- 동일 factor 연결 구조(full_connection)
- 동일 thread 수
- 동일 seed(가능한 범위)

## 6.3 지표

- Mean/Max T error
- Mean/Max R error
- 총 시간(ms)
- stage별 시간(ms)
- outer/inner iteration
- LM step rejection 패턴(가능하면 로그)

## 6.4 합격 기준 (초안)

- 정확도 저하 10% 이내
- NDT 총 시간 30% 이상 단축
- 실행 안정성(발산/비정상 종료/극단적 lambda 진동) 악화 없음

## 6.5 필수 로그 아티팩트

최소한 아래 항목은 run마다 저장한다.

- level별 cost 추이 (coarse/fine)
- level별 LM lambda 추이
- accepted/rejected step 수
- correspondence 유효 비율(가능하면)
- 종료 사유(수렴/반복초과/실패)

이 로그가 없으면 "왜 빨라졌는지/왜 실패했는지"를 재현성 있게 설명하기 어렵다.

---

## 7. 리스크와 대응

## 리스크 1: 단계 전환 직후 cost 불연속

- 원인: 해상도 전환과 correspondence 급변
- 대응: stage 경계에서 optimizer/factor 재생성, 초기 lambda 보수적 설정

## 리스크 2: 과도한 fine 해상도로 인한 분산/불안정

- 원인: sparse 환경에서 voxel 통계가 취약
- 대응: fine=1.0부터 시작, 포인트 밀도 확인 후 0.5 검토

## 리스크 3: 속도 개선은 있으나 정확도 급락

- 원인: coarse stage bias가 fine에서 회복되지 않음
- 대응: coarse iteration 제한, fine iteration 확보, 필요 시 outlier_ratio 분리 튜닝

---

## 8. 단계적 적용 로드맵

1. **Phase 1 (구현 최소화)**
   - `main.cpp` 2-pass만 추가
   - NDT factor 코어는 미수정

2. **Phase 2 (정량 검증)**
   - 5회 반복 평균/표준편차 기록
   - baseline 대비 개선율 문서화

3. **Phase 3 (고급화, 필요 시)**
   - 3-stage 스케줄
   - search mode stage별 최적화
   - factor 내부 레벨 전환 연구

---

## 9. 결론

Multi-resolution NDT는 현재 프로젝트의 NDT 병목(고반복/장시간)을 해결할 유효한 후보이다.
단, 핵심은 "해상도를 바꾼다" 자체가 아니라,

- 단계 경계에서 문제를 분리하고,
- stage별 역할(초기 basin 진입 vs 정밀 보정)을 명확히 나누며,
- 동일 실험 조건에서 정량 검증으로 채택 여부를 결정하는 것이다.

이 프로젝트에서는 `src/main.cpp` 중심 2-pass 구현이 가장 낮은 리스크/높은 실행 가능성을 가진다.

---

## 10. 문서 완성도 체크리스트

- [ ] 단일 해상도 한계에 대한 내부 근거(수치/파일 경로) 포함
- [ ] 단계 전환 시 objective 불연속 위험과 대응 명시
- [ ] LM 재시작/스케일링/수용 기준을 level별로 분리 기술
- [ ] stage별 파라미터를 "왜 그 값인지" 근거와 함께 제시
- [ ] 실패 모드(과평활/세분화 불안정/step rejection cascade)와 대응 포함
- [ ] 검증 계획에 baseline/ablation/로그 아티팩트가 모두 포함

---

## 참고 자료

### 내부 문서/코드

- `src/main.cpp`
- `docs/benchmark/benchmark_results.md`
- `docs/ndt/experiments/full_project_factor_실험_수학코드_종합보고서_2026-03-06.md`
- `docs/ndt/NDT_LM_INCOMPATIBILITY_ANALYSIS.md`
- `docs/ndt/analysis/NDT_성능분석_보고서.md`

### 외부 레퍼런스

- PCL NDT Tutorial: https://pcl.readthedocs.io/projects/tutorials/en/master/normal_distributions_transform.html
- Autoware NDT Scan Matcher: https://autowarefoundation.github.io/autoware.universe/main/localization/ndt_scan_matcher/
- Apollo NDT (예: `SetResolution` 패턴): https://github.com/ApolloAuto/apollo/tree/master/modules/localization/ndt
- ndt_omp: https://github.com/koide3/ndt_omp
- Magnusson (2009): The Three-Dimensional Normal-Distributions Transform
