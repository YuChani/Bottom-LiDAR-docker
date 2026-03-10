# GMM-NDT 구현 작업지시서

## 1. 목적

이 문서는 `NDT + Gaussian Mixture`를 현재 코드베이스에 도입하기 위한 실행 지시서다.
연구 개요 문서(`docs/gmm/NDT_GMM_연구보고서_방향_TODO_2026-03-06.md`)를 실제 코드 작업 단위로 쪼갠다.

---

## 2. 구현 원칙

- LM 내부(`linearize`/`error`)에서는 correspondence와 responsibility(`gamma`)를 고정한다.
- correspondence/`gamma` 재계산은 outer step에서만 수행한다.
- 초기 버전은 `K=1` parity를 먼저 맞춘 뒤 `K=2`로 확장한다.
- 기존 NDT/LightNDT 코드는 그대로 유지하고, 새 factor를 추가한다.

---

## 3. 파일 단위 작업 계획

## A. 신규 파일 추가

1) 헤더

- [ ] `thirdparty/gtsam_points/include/gtsam_points/factors/integrated_gmm_ndt_factor.hpp` 추가
  - `IntegratedGMMNDTFactor_` 클래스 선언
  - `GmmComponentMatch` 구조체 선언
  - public 설정 함수:
    - `set_num_threads(int)`
    - `set_search_mode(NDTSearchMode)`
    - `set_regularization_epsilon(double)`
    - `set_correspondence_update_tolerance(double, double)`
    - `set_num_components(int)`
    - `set_gamma_min(double)`
    - `set_freeze_mixture_in_lm(bool)`

2) 구현

- [ ] `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_gmm_ndt_factor_impl.hpp` 추가
  - `update_correspondences()`에서 top-K 후보 수집
  - `compute_responsibilities()` 구현
  - `evaluate()`에서 `Σ_k gamma_ik * J_k^T W_k J_k`, `Σ_k gamma_ik * J_k^T W_k r_k` 누적

3) 템플릿 인스턴스

- [ ] `thirdparty/gtsam_points/src/gtsam_points/factors/integrated_gmm_ndt_factor.cpp` 추가
  - `PointCloud`, `DummyFrame` 템플릿 인스턴스 선언

---

## B. 기존 파일 수정

4) 빌드 연결

- [ ] `thirdparty/gtsam_points/CMakeLists.txt` 수정
  - 새 cpp 파일 `integrated_gmm_ndt_factor.cpp` 빌드 대상에 추가

5) 메인 벤치마크 등록

- [ ] `src/main.cpp` 수정
  - `factor_types`에 `GMM_NDT` 항목 추가
  - `create_factor()`에 `GMM_NDT` 분기 추가
  - 기존 NDT/LightNDT와 동일한 기본 설정(threads/search_mode/epsilon/tolerance) 적용

6) 공개 include 정리(필요 시)

- [ ] `thirdparty/gtsam_points/include/gtsam_points/factors/` 계층 include 정리
  - 다른 파일에서 `IntegratedGMMNDTFactor`를 바로 include할 수 있도록 정리

---

## 4. 구현 세부 지시

## Step 1: K=1 Parity 모드 먼저 구현

- [ ] `num_components` 기본값을 1로 시작
- [ ] `gamma=1` 고정 모드에서 기존 NDT와 동일한 correspondence/evaluate 경로 재현
- [ ] 아래 항목이 기존 NDT와 유사한지 검증
  - outer/inner iteration
  - mean T/R error
  - optimization time

완료 기준:

- [ ] `--factor NDT`와 `--factor GMM_NDT(K=1)` 결과 차이가 허용 오차 이내

## Step 2: Top-K 후보 및 gamma 계산 추가

- [ ] `NdtCorrespondence` 단일 구조 대신 point당 `vector<GmmComponentMatch>` 캐시
- [ ] `DIRECT1/7/27` 이웃 voxel에서 후보 추출 후 상위 K개 유지
- [ ] softmax 형태로 `gamma` 정규화
- [ ] `gamma_min` 미만 성분 제거

완료 기준:

- [ ] 포인트당 유효 mixture 성분 수가 기대 범위(K 이하)로 유지
- [ ] NaN/Inf 없음

## Step 3: LM 내부 고정(freeze) 정책 적용

- [ ] `cache_linearization_pose` 도입
- [ ] `linearize(values)` 호출 시 pose가 바뀔 때만 mixture 재계산
- [ ] `error(values)`는 같은 LM step 내에서 캐시된 mixture를 사용

완료 기준:

- [ ] LM의 inner reject 폭증 현상 없음
- [ ] 모델 충실도 로그가 기존 NDT 대비 급악화되지 않음

## Step 4: Outer loop 재평가 훅 연결

- [ ] 한 번의 LM solve 이후 correspondence/`gamma`를 재갱신하는 외부 루프 구현(최소 실험 플래그)
- [ ] outer loop 횟수/수렴 조건 추가

완료 기준:

- [ ] K=2에서 NDT 대비 실패율 감소 또는 오차 개선 사례 확보

---

## 5. 수치 안정성 가드레일

- [ ] covariance eigen clamp 유지 (`regularization_epsilon`)
- [ ] `gamma_min` gating 적용
- [ ] mahalanobis distance 상한 게이트(`d2_max`) 옵션 추가
- [ ] log-sum-exp 경로 실험 시 overflow/underflow 안전 구현

---

## 6. 실험/검증 명령

## 6.1 빌드

- [ ] `make -j$(nproc)` in `build_local`

## 6.2 단일 factor smoke

- [ ] `./build_local/lidar_registration_benchmark --headless --factor GMM_NDT --threads 8`

## 6.3 비교 실험

- [ ] `NDT`, `LightNDT`, `GMM_NDT(K=1)`, `GMM_NDT(K=2)` 각각 5회 실행
- [ ] 로그 저장: `/tmp/gmm_ndt_*.log`
- [ ] 집계 CSV 저장: `/tmp/gmm_ndt_compare_5runs.csv`

검증 지표:

- [ ] mean T, mean R
- [ ] max T, max R
- [ ] time(ms)
- [ ] outer/inner iter
- [ ] 실패율(수렴 실패/비정상 종료)

---

## 7. Acceptance Criteria

- [ ] AC-1: `GMM_NDT(K=1)`가 기존 `NDT`와 수치적으로 동등
- [ ] AC-2: `GMM_NDT(K=2)`가 최소 하나 이상 지표에서 개선
  - 실패율 감소 또는
  - mean error 개선 또는
  - 같은 오차에서 시간 단축
- [ ] AC-3: 시간 오버헤드가 허용 범위 이내(목표: 기존 NDT 대비 1.5x 이하)
- [ ] AC-4: 회귀 없음(기존 NDT/LightNDT 동작 불변)

---

## 8. 작업 순서(권장)

1. 신규 factor 스켈레톤 추가
2. K=1 parity 확보
3. main 등록 + smoke
4. K=2 mixture 추가
5. freeze 정책 적용
6. outer loop 확장
7. 5회 반복 비교표 작성
8. 결과를 `docs/gmm/`에 보고서로 반영

---

## 9. 보고 산출물

작업 완료 시 아래 파일을 반드시 남긴다.

- [ ] 구현 보고: `docs/gmm/GMM_NDT_구현결과_보고서.md`
- [ ] 실험 집계: `/tmp/gmm_ndt_compare_5runs.csv`
- [ ] 실험 로그: `/tmp/gmm_ndt_*.log`
