# main 전체 정합 알고리즘 수학 검증 및 headless 결과 해석 (2026-03-04)

## 1) 목적

- `src/main.cpp`에서 실제로 선택되는 정합 factor들(Point-to-Point, Point-to-Plane, GICP, VGICP, NDT, LightNDT, LOAM_LIOSAM)이 수학적으로 일관된 형태로 구현되어 있는지 코드 기준으로 점검한다.
- 동일 코드 기준의 `--headless` 벤치마크 결과를 표로 정리하고, 왜 이런 순위/수치가 나오는지 원인을 설명한다.

## 2) 점검 범위 및 기준

- 점검 파일
  - `src/main.cpp:509`
  - `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_icp_factor_impl.hpp:207`
  - `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_gicp_factor_impl.hpp:266`
  - `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_vgicp_factor_impl.hpp:307`
  - `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:260`
  - `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_light_ndt_factor_impl.hpp:228`
  - `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_loam_factor_impl.hpp:472`
- 판정 기준
  - residual 정의가 목적함수와 일치하는가
  - Jacobian 차원/구조가 SE(3) 선형화와 일치하는가
  - Hessian/gradient 누적(`J^T W J`, `J^T W r`)이 목적함수 미분과 일관되는가

## 3) main 파이프라인에서 결과에 큰 영향을 주는 설정

- `full_connection = true`로 프레임 쌍을 촘촘히 연결 (`src/main.cpp:619`)
- 모든 factor에 `num_threads` 적용 (`src/main.cpp:524`, `533`, `542`, `551`, `585`, `596`, `578`)
- NDT는 `DIRECT7` 탐색 + `outlier_ratio=0.01` (`src/main.cpp:586`, `587`)
- LightNDT도 `DIRECT7` 사용 (`src/main.cpp:597`)

해석 포인트:
- full connection은 정확도 측면에서 유리하지만 factor 수가 늘어 runtime이 증가한다.
- NDT의 `DIRECT7`은 대응 강건성은 올리지만 점당 탐색량이 증가한다.
- NDT의 `outlier_ratio`는 지수항 가중 감쇠 강도(실질 step 크기)에 영향을 준다.

## 4) 알고리즘별 수학 감사 결과 (코드 기준)

| 알고리즘 | 판정 | 핵심 근거 |
|---|---|---|
| Point-to-Point ICP | 정상 | `residual = mean_B - delta*mean_A`, `H += J^T J`, `b += J^T r` (`integrated_icp_factor_impl.hpp:208`, `234-238`) |
| Point-to-Plane ICP | 정상 | normal 투영 residual/Jacobian 적용 후 동일 GN 누적 (`integrated_icp_factor_impl.hpp:210-213`, `228-232`) |
| GICP | 정상 | Mahalanobis 목적 `r^T M r`, `H += J^T M J`, `b += J^T M r` (`integrated_gicp_factor_impl.hpp:266`, `279-286`) |
| VGICP | 정상 | GICP와 동일한 D2D 형태, 대응/공분산만 voxel 기반 (`integrated_vgicp_factor_impl.hpp:307`, `321-328`) |
| NDT | 정상(근사 명시) | 지수형 cost + GN 근사 Hessian(H1) 사용 (`integrated_ndt_factor_impl.hpp:260-265`, `282-289`) |
| LightNDT | 정상 | score 항 없이 순수 Mahalanobis 이차형 (`integrated_light_ndt_factor_impl.hpp:228`, `241-248`) |
| LOAM_LIOSAM | 정상 | edge + plane factor의 오차/선형화 합산 (`integrated_loam_factor_impl.hpp:472-483`) |

요약:
- 이번 감사 범위에서 “수학적 오류(부호/차원/누적식 불일치)”는 발견되지 않았다.
- NDT는 이론적으로 가능한 full Newton 항(H2/H3)을 생략한 Gauss-Newton 근사이며, 이는 안정성/PSD 목적의 일반적 구현 선택으로 분류된다.

## 5) 최신 headless 결과 요약

출처: `benchmark_output_main_audit.txt`

| 알고리즘 | Mean T (m) | Mean R (deg) | Time (ms) |
|---|---:|---:|---:|
| Point-to-Point | 0.095565 | 0.486174 | 5099.607 |
| Point-to-Plane | **0.062059** | **0.443208** | 1825.022 |
| GICP | 0.083675 | 0.539470 | 2358.527 |
| VGICP | 0.112651 | 0.623472 | 2439.050 |
| NDT | 0.097598 | 0.714911 | **7645.368** |
| LightNDT | 0.158550 | 0.646398 | 1055.967 |
| LOAM_LIOSAM | 0.225414 | 0.774622 | **120.030** |

## 6) 왜 이런 결과가 나오는가 (핵심 해석)

1. Point-to-Plane가 가장 좋은 정확도를 보인 이유
   - 현재 데이터가 평면 구조 제약을 잘 제공하고, point-to-plane residual이 회전/병진에 민감하게 작동한다.
   - full connection에서 누적 제약이 충분히 들어가며 LM이 안정적으로 수렴한다.

2. GICP/VGICP가 Point-to-Plane보다 약간 뒤처진 이유
   - D2D(Mahalanobis) 연산은 강건하지만 공분산/대응 품질에 민감하다.
   - 본 데이터/설정에서는 point-to-plane 단순 구조가 오히려 더 직접적인 제약으로 작용했다.

3. NDT가 가장 느린 축에 있는 이유
   - `DIRECT7` 탐색으로 대응 계산량이 증가한다.
   - 지수 가중(`exp`) 기반으로 step이 보수적이 되기 쉬워 iteration이 늘어난다.
   - full connection 조건에서 factor 수 증가 효과가 더 크게 반영된다.

4. LightNDT가 빠르지만 정확도 손해가 있는 이유
   - NDT의 score(지수 robust) 항을 제거하고 이차형으로 단순화해 per-iteration 연산이 가벼워진다.
   - 대신 outlier/비선형 대응에서 NDT score가 제공하던 완충이 줄어 정확도 손해가 발생할 수 있다.

5. LOAM_LIOSAM이 매우 빠르지만 오차가 큰 이유
   - 특징점(edge/plane)만 사용해 점 수가 크게 줄어 매우 빠르다.
   - 샘플 수 감소와 특징 품질 의존성 때문에 전역 정합 정확도는 상대적으로 불리할 수 있다.

## 7) 외부 레퍼런스와의 정합성

- ICP/Point-to-Plane: Besl & McKay(1992), Chen & Medioni(1992) 계열 특성과 동일한 경향.
- GICP: Segal et al.(2009)의 공분산 융합(D2D) 구조와 구현 패턴 일치.
- VGICP: fast_gicp 공개 구현/문헌의 voxelized D2D 가속 방향성과 일치.
- NDT: Biber & Strasser(2003), Magnusson(2009)의 지수 기반 점수함수/강건성-속도 트레이드오프와 일치.

## 8) 결론

- `main`에서 사용하는 7개 정합 알고리즘은 코드 수준 수학 감사 기준에서 모두 정상 범주로 판정된다.
- 성능 차이는 구현 오류보다는 목적함수 형태(이차형 vs 지수형), 대응 방식(점/복셀/특징), 그래프 연결도(full connection)에서 기인한다.
- 현재 설정에서는 Point-to-Plane이 정확도 우위, LOAM_LIOSAM이 속도 우위, NDT는 강건성 대가로 runtime 손해가 큰 프로파일이다.

## 9) TODO (요청사항: 한글 정리)

- [ ] NDT 속도 개선 실험 매트릭스 추가 실행
  - 축: `search_mode(DIRECT1/DIRECT7)`, `outlier_ratio`, `correspondence tolerance`, `threads`
  - 목표: 정확도 하락 10% 이내에서 runtime 최소 조합 찾기
- [ ] 동일 조건 반복 실행(시드 고정)으로 분산/재현성 표 추가
- [ ] 알고리즘별 iteration 수/평균 선형화 시간/solve 시간 분해 표 추가
- [ ] 문서 인덱스(`docs/README.md`, `docs/benchmark/BENCHMARK_REPORT.md`)에 본 보고서 링크 연결
- [ ] 필요 시 `full_connection=false` 비교표를 별도 섹션으로 추가해 실제 운영 시나리오 의사결정 지원
