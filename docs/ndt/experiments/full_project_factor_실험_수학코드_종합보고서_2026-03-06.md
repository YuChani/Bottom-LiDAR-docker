# 전체 Factor 실험 + 수학/코드 종합 분석 보고서

## 0. 목적

본 문서는 현재 프로젝트의 모든 활성 factor에 대해 다음을 동시에 수행한 결과를 정리한다.

1. 동일 조건 반복 실험(정량)
2. factor별 수학식 검증(이론-코드 대응)
3. 코드 경로 중심 병목 분석(왜 느리고/빠른지)

핵심 초점은 "무엇이 빠르다"보다 **"왜 그렇게 되는가"**이다.

중요: 본 실험은 "풀 팩토리얼(다요인 상호작용 분석)"이 아니라,
`factor_type` 단일 요인 중심의 **동일 파이프라인 비교 벤치마크**다.

---

## 1. 실험 설정

## 1.1 실행 커맨드

동일 조건으로 전체 factor를 5회 반복 실행했다.

```bash
./build_local/lidar_registration_benchmark --headless --threads 8
```

반복 로그:

- `/tmp/fullproj_bench_run01.log`
- `/tmp/fullproj_bench_run02.log`
- `/tmp/fullproj_bench_run03.log`
- `/tmp/fullproj_bench_run04.log`
- `/tmp/fullproj_bench_run05.log`

집계 파일:

- `/tmp/fullproj_factor_analysis_5runs.csv`

## 1.2 실험 경로 검증

headless 실행은 동일 코드 경로에서 factor를 순회하며 실행한다.

- factor 생성 분기: `src/main.cpp:550`
- headless 전체 루프: `src/main.cpp:834`
- LM 실행/종료 설정: `src/main.cpp:672`, `src/main.cpp:675`, `src/main.cpp:676`, `src/main.cpp:677`

추가로 본 벤치마크는 pose graph를 구성해(연결 방식 포함) 최적화한다.

- 그래프 factor 추가 루프: `src/main.cpp:657`~`src/main.cpp:665`
- `full_connection` 분기: `src/main.cpp:659`

활성 factor 목록:

- Point-to-Point, Point-to-Plane, GICP, VGICP, NDT, LightNDT, LOAM_LIOSAM
  - `src/main.cpp:369`~`src/main.cpp:376`

---

## 1.3 공정성 체크리스트 (고정 vs 상이)

| 항목 | 상태 | 근거 |
|---|---|---|
| 옵티마이저(LM) | 고정 | `src/main.cpp:672` |
| LM tolerance/maxIter | 고정 | `src/main.cpp:675`~`src/main.cpp:678` |
| threads | 고정(`--threads 8`) | 실행 커맨드 |
| correspondence tolerance | 대부분 공통 변수 사용 | `src/main.cpp:564`, `src/main.cpp:573`, `src/main.cpp:582`, `src/main.cpp:591`, `src/main.cpp:629`, `src/main.cpp:639` |
| LOAM correspondence tolerance | 별도 고정값 사용 | `src/main.cpp:618` |
| LOAM max correspondence distance | 별도 2.0 설정 | `src/main.cpp:617` |
| NDT search mode/epsilon | 명시 설정 | `src/main.cpp:627`, `src/main.cpp:628` |
| NDT resolution/outlier_ratio | 현재 기본값 사용(미튜닝) | `integrated_ndt_factor_impl.hpp:27`, `integrated_ndt_factor_impl.hpp:28` |

해석상 의미: 본 비교는 "동일 실행 파이프라인" 관점에서 공정하지만,
각 factor의 완전한 최적 튜닝 조건 비교는 아니다.

---

## 2. 정량 결과 (5회 평균)

| Factor | Time (ms) mean±std | Outer Iter | Inner Iter | Mean T Error (m) | Mean R Error (deg) |
|---|---:|---:|---:|---:|---:|
| Point-to-Point | 810.0 ± 7.7 | 26 | 26 | 0.100990 | 0.436051 |
| Point-to-Plane | 516.0 ± 14.3 | 15 | 15 | 0.061799 | 0.430725 |
| GICP | 630.6 ± 13.6 | 9 | 9 | 0.085399 | 0.499411 |
| VGICP | 857.6 ± 8.0 | 12 | 23 | 0.117390 | 0.635614 |
| NDT | 2523.4 ± 20.7 | 74 | 74 | 0.101971 | 0.697636 |
| LightNDT | 324.2 ± 4.4 | 6 | 16 | 0.159027 | 0.640614 |
| LOAM_LIOSAM | 41.4 ± 0.9 | 8 | 8 | 0.225414 | 0.774622 |

핵심 관찰:

- NDT는 반복 수가 압도적으로 높아(74) 벽시계 시간이 가장 길다.
- LightNDT는 가장 적은 outer iteration(6)으로 빠르다.
- VGICP는 outer 12인데 inner 23으로, LM trial 증가가 관측된다.

주의: 본 5회 반복에서는 정확도 지표(`mean_t`, `mean_r`)가 run 간 동일하게 관측되었다.
따라서 정확도에 대한 통계적 분산/신뢰구간 주장은 본 데이터만으로 수행하지 않는다.

---

## 3. Factor별 수학식-코드 검증

아래는 "수식 정의 -> 코드 구현" 매핑이다.

## 3.1 Point-to-Point ICP

### 수학식

```text
E = Σ || p_t - T p_s ||^2
H = Σ J^T J
b = Σ J^T r
```

### 코드 근거

- correspondence: 1-NN KD-tree
  - `integrated_icp_factor_impl.hpp:151`
- residual/error
  - `integrated_icp_factor_impl.hpp:208`, `integrated_icp_factor_impl.hpp:215`
- Hessian/gradient
  - `integrated_icp_factor_impl.hpp:234`~`238`

## 3.2 Point-to-Plane ICP

### 수학식

```text
r_impl = n \odot (p_t - T p_s)
E = Σ ||r_impl||^2
```

### 코드 근거

- Point-to-Plane은 `IntegratedPointToPlaneICPFactor` alias로 ICP factor의 point-to-plane 모드를 사용
  - `integrated_icp_factor.hpp:140`~`161`
- normal 투영
  - `integrated_icp_factor_impl.hpp:210`~`213`
- Jacobian normal 가중
  - `integrated_icp_factor_impl.hpp:228`~`232`

중요: 본 구현은 통상 문헌의 `(n^T r)^2`를 직접 계산한다기보다,
코드상 `n \odot r` 성분별 가중 후 제곱합 형태다. 따라서 본 보고서의 Point-to-Plane 해석은
"본 구현체" 기준이며 일반 P2PL 구현 전체로 확장하지 않는다.

## 3.3 GICP

### 수학식

```text
W = (Sigma_B + R Sigma_A R^T)^(-1)
E = Σ r^T W r
H = Σ J^T W J
b = Σ J^T W r
```

### 코드 근거

- fused covariance 생성
  - `integrated_gicp_factor_impl.hpp:178`~`180`
- error
  - `integrated_gicp_factor_impl.hpp:266`
- Hessian/gradient
  - `integrated_gicp_factor_impl.hpp:279`~`286`

## 3.4 VGICP

### 수학식

```text
W = (Sigma_voxel_B + R Sigma_A R^T)^(-1)
E = Σ r^T W r
```

### 코드 근거

- voxel correspondence
  - `integrated_vgicp_factor_impl.hpp:135`~`141`
- fused covariance with voxel cov
  - `integrated_vgicp_factor_impl.hpp:150`, `integrated_vgicp_factor_impl.hpp:301`~`303`
- error/Hessian
  - `integrated_vgicp_factor_impl.hpp:307`, `integrated_vgicp_factor_impl.hpp:321`~`328`

### 구조적 특징(중요)

VGICP는 `error()`에서 correspondence를 고정하고 mahalanobis만 갱신하는 경로를 별도로 둔다.

- `integrated_vgicp_factor_impl.hpp:189`~`202`

이는 voxel 경계 불연속으로 LM step이 연속 거절되는 문제를 줄이기 위한 설계다.

## 3.5 NDT

### 수학식

```text
m = r^T Sigma^-1 r
score = -d1 * exp(-d2/2 * m)
cost  = -d1 - score
weight = -d1 * d2 * exp(-d2/2 * m)
H ~= weight * J^T Sigma^-1 J   (H1 only)
```

### 코드 근거

- d1/d2 계산 함수
  - `integrated_ndt_factor.hpp:67`~`73`
- cost/score/exp
  - `integrated_ndt_factor_impl.hpp:248`, `integrated_ndt_factor_impl.hpp:261`, `integrated_ndt_factor_impl.hpp:264`
- H1-only 근사
  - `integrated_ndt_factor_impl.hpp:282`~`286`

## 3.6 LightNDT

### 수학식

```text
E = Σ r^T Sigma_voxel^-1 r
H = Σ J^T Sigma_voxel^-1 J
b = Σ J^T Sigma_voxel^-1 r
```

### 코드 근거

- cost
  - `integrated_light_ndt_factor_impl.hpp:228`
- Hessian/gradient
  - `integrated_light_ndt_factor_impl.hpp:241`~`248`
- correspondence는 NDT와 유사한 voxel neighbor 탐색
  - `integrated_light_ndt_factor_impl.hpp:109`~`129`, `integrated_light_ndt_factor_impl.hpp:152`~`167`

## 3.7 LOAM_LIOSAM

LOAM factor는 edge + plane 두 서브 factor 합으로 구성된다.

- 결합부: `integrated_loam_factor_impl.hpp:472`~`488`

### Point-to-Edge 항

```text
residual ~ ((x_i - x_j) x (x_i - x_l)) / ||x_j - x_l||
E_edge = ||residual||^2
```

- `integrated_loam_factor_impl.hpp:341`~`347`

### Point-to-Plane 항

```text
normal = (x_j - x_l) x (x_j - x_m)
r_plane = normal \odot (x_j - T x_i)
E_plane = ||r_plane||^2
```

- `integrated_loam_factor_impl.hpp:158`, `integrated_loam_factor_impl.hpp:172`~`175`

---

## 4. 코드 중심 병목 분석 (왜 이 속도가 나오는가)

## 4.1 공통 병목: correspondence update + 선형화 반복

공통 기반 class에서 `linearize()`와 `error()` 모두 correspondence 갱신이 관여한다.

- `integrated_matching_cost_factor.cpp:36`
- `integrated_matching_cost_factor.cpp:43`

LM은 매 iteration마다 그래프 선형화 및 step 수용판정을 수행한다.

- fidelity 계산: `levenberg_marquardt_ext.cpp:258`~`260`
- linearize 호출: `levenberg_marquardt_ext.cpp:363`

즉, factor별 correspondence/update 비용이 반복 수와 곱해져 총시간을 결정한다.

## 4.2 Factor별 why

### Point-to-Point / Point-to-Plane

- KD-tree 1NN(`knn_search`) + 단순 LSQ
- point-to-plane은 normal 투영 추가로 반복 수 감소(26 -> 15)가 관측됨

### GICP

- KD-tree 대응 + fused covariance inverse 비용
- 반복 수는 낮지만(`9`), per-point 행렬연산이 ICP보다 무겁다

### VGICP

- correspondence는 voxel lookup으로 KD-tree 부담을 줄이지만,
- fused covariance 갱신과 mahalanobis 캐시 경로가 존재
- LM inner iteration이 높게 관측(`23`)되어 시간 증가

### NDT

- 지수형 cost + residual 의존 weight + H1-only 근사
- outer/inner가 모두 높게 고정(`74`)
- "한 번 계산이 매우 비싸서"라기보다 "반복이 너무 많아서"가 지배적 병목

### LightNDT

- NDT와 유사 correspondence를 쓰지만 목적함수를 이차형으로 단순화
- LM 친화도가 높아 outer iteration이 매우 낮음(`6`)
- 결과적으로 전체 시간 최소(LOAM feature 기반 제외)

### LOAM_LIOSAM

- 사전 추출된 edge/plane feature에 대해만 계산
- 전체 포인트 대비 샘플 수가 작아 매우 빠른 시간(`41 ms`)을 보임
- 대신 오차 지표는 본 실험에서 가장 큼(특징점 선택/분포 민감)

---

## 5. 중요한 해석 주의점

1. 본 실험은 "같은 파이프라인 조건" 비교다. 각 factor의 최적 튜닝 상태 비교는 아니다.
2. LOAM_LIOSAM은 feature 기반 서브샘플링 구조라 전체 포인트 직접 정합 계열과 계산 성격이 다르다.
3. LightNDT의 빠름은 항상 절대우세가 아니라, 현재 데이터/초기조건/설정에서의 관측값이다.
4. NDT의 느림은 구현 오류라기보다 현재 조합(지수 cost + H1-only + LM 상호작용)에서의 구조적 경향으로 해석하는 것이 타당하다.

---

## 6. 결론

현재 프로젝트 기준으로는 다음이 데이터와 코드 양쪽에서 일치한다.

- NDT: robust 성향을 유지하는 대신 반복 수 증가로 시간 손해가 큼
- LightNDT: 수학 구조가 LM에 잘 맞아 반복 수가 작고 매우 빠름
- GICP/VGICP: 중간 지대(정확도/시간 절충), 구현 구조에 따라 inner iteration 차이가 발생
- LOAM_LIOSAM: feature 기반 구조로 매우 빠르지만, 이번 실험에서는 오차가 상대적으로 큼

즉, 운영 관점에서는:

1. 속도 우선 기본값: LightNDT
2. 강건성/검증 채널: NDT 또는 GICP fallback
3. 장기 개선: NDT 전용 다단계/coarse-to-fine 및 종료조건 개선

조건부 문장(과잉 일반화 방지):

- 본 순위와 해석은 **현재 설정**(headless, threads=8, 본 데이터셋, 본 그래프 구성, 본 파라미터)에서만 성립한다.
- 다른 초기화/데이터/파라미터 교차설계에서는 순위가 바뀔 수 있다.

---

## 7. 외부 수학 검증 참고

- ICP: Besl & McKay 1992
  - https://ieeexplore.ieee.org/document/121791
- Point-to-Plane: Chen & Medioni 1991
  - https://ieeexplore.ieee.org/document/131487
- Point-to-Plane 선형화 참고: Low 2004
  - https://www.comp.nus.edu.sg/~lowkl/publications/lowk_point-to-plane_icp_techrep.pdf
- GICP: Segal et al. 2009
  - https://www.roboticsproceedings.org/rss05/p21.pdf
- VGICP: Koide et al. 2021
  - https://ieeexplore.ieee.org/document/9560835
- NDT 원형: Biber & Straßer 2003
  - https://ieeexplore.ieee.org/document/1249285
- PCL Registration 문서
  - https://pointclouds.org/documentation/group__registration.html
- LOAM: Zhang & Singh 2014
  - https://www.roboticsproceedings.org/rss10/p07.pdf

---

## 8. 코드리뷰 확장: `main`-`base factor`-`optimizer` 실행 경로 해부

본 절은 사용자가 요청한 "함수/변수 의미 중심" 리뷰를 위해 추가한 섹션이다.

원칙:

- `Observed`: 코드에서 직접 확인되는 사실만 기술
- `Inferred`: 의도 추정은 최소화하고, 필요한 경우 명시

## 8.1 엔드-투-엔드 호출 흐름 (Headless 기준)

```text
main()
  -> MatchingCostFactorDemo(headless=true)
    -> 데이터 로드/프레임 생성/voxelmap 생성/GT 초기화
  -> run_all_factors_headless()
    -> factor_type 순회
      -> poses 초기화(동일 noisy 시작점)
      -> run_optimization()
        -> graph 구성(Prior + pairwise factor)
        -> LevenbergMarquardtOptimizerExt.optimize()
          -> iterate() -> graph.linearize() -> tryLambda() 반복
          -> factor.error()/linearize()에서 correspondence+cost 평가
        -> frame별 error 및 요약 통계 출력
```

핵심 진입점:

- `main` 인자 파싱: `src/main.cpp:916`
- headless 실행 분기: `src/main.cpp:949`
- factor 전체 벤치 루프: `src/main.cpp:834`
- 실제 최적화 수행: `src/main.cpp:647`

## 8.2 `MatchingCostFactorDemo` 멤버 변수 의미

아래는 실험 해석에서 중요한 상태 변수들이다.

- `poses` (`src/main.cpp:814`): 현재 최적화 대상 state (`gtsam::Values`)
- `poses_gt` (`src/main.cpp:815`): GT state. 오류 계산의 기준
- `frames` (`src/main.cpp:817`): 각 프레임 점군 + normal + cov
- `voxelmaps` (`src/main.cpp:818`): 각 프레임의 Gaussian voxel map
- `factor_type` (`src/main.cpp:801`): 현재 실행할 factor index
- `optimizer_type` (`src/main.cpp:807`): `LM`/`ISAM2` 선택값
- `full_connection` (`src/main.cpp:802`): 그래프 간선 밀도 제어
- `correspondence_update_tolerance_rot/trans` (`src/main.cpp:809`, `src/main.cpp:810`): correspondence 재탐색 최소 이동 임계
- `last_mean_trans_error`, `last_mean_rot_error`, `last_total_ms` (`src/main.cpp:827`~`src/main.cpp:831`): 마지막 실행의 대표 지표 캐시

## 8.3 데이터 준비 단계에서 각 함수가 하는 일

생성자(`MatchingCostFactorDemo::MatchingCostFactorDemo`)는 단순 초기화가 아니라 실험 상태를 완성한다.

1) 데이터 경로 확인

- 기본 `/root/workdir/data/pcd` 검사 후, 실패 시 CWD 기준 `data/pcd`로 fallback
- 근거: `src/main.cpp:100`~`src/main.cpp:105`

2) GT 로드 및 timestamp 매칭

- `gt-tum.txt`를 `map<double, Pose3>`로 적재
- 최근접 timestamp 보간 없이 nearest-pose 선택
- 근거: `src/main.cpp:121`~`src/main.cpp:136`, `src/main.cpp:172`~`src/main.cpp:181`

3) 프레임별 점군/특징/공분산 생성

- PCD 읽기 -> homogeneous(4D) 변환 -> covariance/normal 추정
- LIO-SAM feature 추출 결과(`edge_points`, `planar_points`)를 별도 보관
- 근거: `src/main.cpp:215`, `src/main.cpp:243`~`src/main.cpp:253`, `src/main.cpp:267`, `src/main.cpp:277`~`src/main.cpp:279`

4) VGICP/NDT용 voxelmap 구축

- `GaussianVoxelMapCPU(1.0)` 생성 후 frame 삽입
- 근거: `src/main.cpp:284`~`src/main.cpp:286`

## 8.4 `create_factor()` 분기: "같은 그래프 틀 + 다른 잔차"

팩터 생성 함수는 동일한 그래프 구조에 factor별 비용 함수를 꽂아 넣는 위치다.

- 함수 위치: `src/main.cpp:550`
- 공통 패턴: factor 생성 -> correspondence tolerance 적용 -> thread 적용
- 분기별 핵심 setter:
  - ICP/P2PL/GICP/VGICP: tolerance + threads
  - LOAM_LIOSAM: validation on + edge/plane max distance + 고정 tolerance
  - NDT: `set_search_mode(DIRECT7)`, `set_regularization_epsilon(1e-3)`, tolerance
  - LightNDT: NDT와 동일하되 score 지수항 없음

NDT 파라미터 경로(중요):

- `main.cpp`에서는 `resolution/outlier_ratio`를 강제하지 않음
- 따라서 impl 기본값(`resolution=3.0`, `outlier_ratio=0.01`)이 실제 사용
- 근거: `src/main.cpp:622`~`src/main.cpp:630`, `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:27`, `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:28`

## 8.5 `run_optimization()`에서 실제로 계산되는 것

1) 그래프 구성

- prior(키 0 고정) + pairwise registration factor 집합
- 근거: `src/main.cpp:654`, `src/main.cpp:657`~`src/main.cpp:665`

2) LM 파라미터

- `maxIterations=100`, `relativeErrorTol=1e-5`, `absoluteErrorTol=1e-5`
- `lambdaLowerBound=1e-6`로 undamped GN 완전 퇴화 억제
- 근거: `src/main.cpp:675`~`src/main.cpp:679`

3) 결과 산출

- frame별 pose error 계산 후 평균/최대 요약 저장
- headless 표 생성 시 이 캐시를 재사용
- 근거: `src/main.cpp:748`~`src/main.cpp:792`, `src/main.cpp:891`~`src/main.cpp:912`

## 9. 베이스 클래스 `IntegratedMatchingCostFactor` 코드리뷰

파일:

- 선언: `thirdparty/gtsam_points/include/gtsam_points/factors/integrated_matching_cost_factor.hpp`
- 구현: `thirdparty/gtsam_points/src/gtsam_points/factors/integrated_matching_cost_factor.cpp`

역할 요약:

- 하위 factor가 반드시 구현해야 할 2개 훅
  - `update_correspondences(delta)`
  - `evaluate(delta, H_*, b_*)`
- 공통 `error()`/`linearize()`에서 위 훅을 호출해 GTSAM HessianFactor를 만든다.

핵심 함수 의미:

1) `calc_delta(values)` (`.../integrated_matching_cost_factor.cpp:60`)

- binary: `delta = T_target^{-1} * T_source`
- unary: `delta = T_fixed_target^{-1} * T_source`
- 즉 하위 factor는 언제나 "타겟 좌표계에서 본 소스 변환"을 입력으로 받는다.

2) `error(values)` (`.../integrated_matching_cost_factor.cpp:32`)

- `calc_delta` -> `update_correspondences` -> `evaluate(delta)` 순서
- LM의 trial step에서도 최신 pose 기준 correspondence를 맞추려는 경로

3) `linearize(values)` (`.../integrated_matching_cost_factor.cpp:40`)

- 동일하게 correspondence 업데이트 후, Hessian/gradient 블록 계산
- binary면 (target, source) 2-key HessianFactor, unary면 source 1-key HessianFactor 생성

의미적으로 중요한 점:

- "factor의 수학"은 `evaluate()`에 들어가고,
- "최적화와 연결되는 계약"은 base class가 고정한다.

## 10. Factor별 API/수식/상태 변경 포인트 상세

## 10.1 ICP / Point-to-Plane (`integrated_icp_factor.hpp`, `impl/...icp...`)

- `update_correspondences`: 소스 점을 `delta`로 변환 후 target KD-tree 1NN 검색
  - `knn_search(..., 1, ...)` 사용
  - 근거: `.../integrated_icp_factor_impl.hpp:151`
- `evaluate`:
  - point-to-point: `r = p_t - T p_s`
  - point-to-plane: `r = n ⊙ (p_t - T p_s)`
  - Hessian/gradient: `J^T J`, `J^T r`
  - 근거: `.../integrated_icp_factor_impl.hpp:208`~`239`

세터 의미:

- `set_max_correspondence_distance`: trimming threshold
- `set_point_to_plane_distance`: 동일 factor 코드에서 모드 전환

## 10.2 GICP (`integrated_gicp_factor.hpp`, `impl/...gicp...`)

- `update_correspondences`:
  - 1NN 매칭 + `R Cov_A R^T + Cov_B` 역행렬(마할라노비스) 캐시 생성
  - 캐시 모드: `FULL`/`COMPACT`/`NONE`
  - 근거: `.../integrated_gicp_factor_impl.hpp:172`~`191`
- `evaluate`:
  - `E = r^T W r`, `W=(R Cov_A R^T + Cov_B)^{-1}`
  - 근거: `.../integrated_gicp_factor_impl.hpp:266`, `...:279`~`286`

세터 의미:

- `set_fused_cov_cache_mode`: 메모리-속도 절충 지점

## 10.3 VGICP (`integrated_vgicp_factor.hpp`, `impl/...vgicp...`)

- `update_correspondences`:
  - KD-tree 대신 voxel index lookup으로 target 분포 선택
  - 근거: `.../integrated_vgicp_factor_impl.hpp:135`~`142`
- `error()` override가 핵심:
  - LM trial마다 voxel 재탐색 대신 `update_mahalanobis()`만 수행
  - voxel 경계 불연속으로 인한 step reject 악화를 줄이기 위한 설계
  - 근거: `.../integrated_vgicp_factor.hpp:99`~`104`, `.../integrated_vgicp_factor_impl.hpp:189`~`202`, `...:205`~`253`
- `evaluate`:
  - 본질은 GICP와 같은 `r^T W r` 구조

## 10.4 NDT (`integrated_ndt_factor.hpp`, `impl/...ndt...`)

- `update_correspondences`:
  - search mode(`DIRECT1/7/27`) 이웃 voxel 후보 중 최소 Mahalanobis 선택
  - inv-cov 캐시(`inv_cov_cache`)는 voxel 단위로 저장
  - `compute_ndt_params(resolution, outlier_ratio, d1, d2)`를 매 업데이트 시 계산
  - 근거: `.../integrated_ndt_factor_impl.hpp:121`~`151`, `...:164`~`186`, `...:119`
- `evaluate`:
  - score 지수항: `score = -d1 * exp(-d2/2 * m)`
  - cost: `-d1 - score`
  - Hessian은 H1-only 근사(`weight * J^T Σ^{-1} J`)
  - 근거: `.../integrated_ndt_factor_impl.hpp:248`, `...:261`, `...:264`, `...:282`~`286`

변수 의미(실전에서 자주 헷갈리는 부분):

- `gauss_d1`, `gauss_d2`: score 곡선의 진폭/감쇠를 결정
- `regularization_epsilon`: covariance 고유값 clamp 강도
- `search_mode`: 대응의 지역성 vs 계산량 trade-off

## 10.5 LightNDT (`integrated_light_ndt_factor.hpp`, `impl/...light_ndt...`)

- correspondence 탐색은 NDT와 동일 구조
- 차이는 evaluate의 목적함수:
  - NDT처럼 지수 score를 쓰지 않고, 직접 `r^T Σ^{-1} r`
  - Hessian/gradient도 선형 least-squares 형태
  - 근거: `.../integrated_light_ndt_factor_impl.hpp:228`, `...:241`~`248`

의미:

- correspondence 비용은 비슷하지만, 최적화 표면이 LM에 더 "정상적인 이차형"으로 보인다.

## 10.6 LOAM (`integrated_loam_factor.hpp`, `impl/...loam...`)

- `IntegratedLOAMFactor`는 wrapper다.
  - 내부에 `edge_factor` + `plane_factor`를 보유
  - update/evaluate에서 둘을 호출해 합산
  - 근거: `.../integrated_loam_factor_impl.hpp:455`~`489`
- edge residual: 선분 거리(외적 기반)
  - 근거: `.../integrated_loam_factor_impl.hpp:345`~`347`
- plane residual: 3점으로 만든 normal 기반 평면 거리
  - 근거: `.../integrated_loam_factor_impl.hpp:158`, `...:172`~`175`
- `validate_correspondences()`:
  - 동일 scan line 대응 제거 옵션
  - 근거: `.../integrated_loam_factor_impl.hpp:498`~`540`

## 11. LM 확장 옵티마이저(`LevenbergMarquardtOptimizerExt`) 리뷰

파일: `thirdparty/gtsam_points/src/gtsam_points/optimizers/levenberg_marquardt_ext.cpp`

실행 루프:

1) `optimize()`

- `iterate()` 반복, 종료 조건 검사
- 종료 조건:
  - `iterations > maxIterations`
  - `delta_error < absoluteErrorTol`
  - `delta_error / |currentError| < relativeErrorTol`
  - `newError < errorTol`
- 근거: `.../levenberg_marquardt_ext.cpp:413`~`423`

2) `iterate()`

- 현재 values로 graph 선형화
- lambda를 바꾸며 `tryLambda()` 반복
- 근거: `.../levenberg_marquardt_ext.cpp:359`~`395`

3) `tryLambda()`

- damped system 구성 -> 선형 해법 -> retract -> nonlinear error 재계산
- 모델 충실도(`modelFidelity = costChange / linearizedCostChange`) 기반 수용/거절
- 성공 시 `decreaseLambda`, 실패 시 `increaseLambda`
- 근거: `.../levenberg_marquardt_ext.cpp:255`~`261`, `...:328`~`347`

코드리뷰 관점 핵심:

- factor의 correspondence 재탐색 정책이 LM의 inner iteration 품질에 직접 영향
- VGICP가 `error()`를 override한 이유가 바로 이 지점(불연속 완화)

## 12. 실험 해석에 직접 연결되는 코드 포인트 (요약)

1) NDT가 느린 이유를 코드로 보면

- 점당 연산 자체보다, 높은 outer/inner 반복이 누적 시간 지배
- 지수형 score + H1-only 근사 + correspondence/weight 재평가가 결합되어 반복 수 증가

2) LightNDT가 빠른 이유를 코드로 보면

- update_correspondences는 거의 동일
- evaluate가 이차형으로 단순해 LM 수용성이 높고 반복 수가 작음

3) 공정 비교를 위해 이미 맞춘 것

- headless 동일 루프, 동일 초기 noisy pose, 동일 LM 틀
- 차이는 "factor 수학/자료연결 방식" 자체

## 13. 과장 방지(Overclaim Guardrails)

- 본 문서는 코드 관찰 기반이며, 모든 결론은 현재 설정/데이터셋/그래프 연결 방식에 한정된다.
- "항상/절대" 표현을 피하고, factor별 우열은 조건부로 해석한다.
- 구현 수식과 논문 일반식이 다를 수 있는 부분은 구현 파일 기준으로만 기술했다.
