# full_connection=true vs false 벤치마크 비교 보고서 (2026-03-12)

## 1. 목적

본 문서는 `src/main.cpp`의 `full_connection` 토글을 직접 `false -> true -> false`로 변경하면서,

- `chain` 그래프(`full_connection = false`)
- `full all-pairs` 그래프(`full_connection = true`)

두 설정에서 `lidar_registration_benchmark --headless --threads 8`를 실제로 실행한 결과를 비교한다.

핵심 질문은 다음과 같다.

1. `full_connection`을 chain으로 바꾸면 전체 runtime이 얼마나 줄어드는가?
2. iteration 수는 같이 줄어드는가, 아니면 별개로 움직이는가?
3. 정확도(Mean/Max T/R error)는 얼마나 달라지는가?
4. 이 차이는 현재 프로젝트의 실험 목적(odometry vs batch global registration)과 어떻게 연결되는가?

---

## 2. 실험 설정

## 2.1 실행 환경

- 저장소 경로: `/home/chani/personal/Bottom-LiDAR-docker`
- 실행 환경: Docker 컨테이너 `bottom-lidar`
- 실행 파일: `/root/workdir/build/lidar_registration_benchmark`
- 실행 스레드: `--threads 8`
- 실행 모드: `--headless`

## 2.2 코드 토글 대상

비교한 설정은 `src/main.cpp:386`의 다음 한 줄이다.

```cpp
full_connection = false;   // chain
full_connection = true;    // full all-pairs
```

실제 그래프 생성은 `build_registration_graph()`에서 다음 분기로 결정된다.

```cpp
int j_end = full_connection ? num_frames : std::min(i + 2, num_frames);
for (int j = i + 1; j < j_end; j++) {
  auto factor = create_factor(i, j, frames[i], active_voxelmaps[i], nullptr, frames[j]);
  graph.add(factor);
}
```

의미는 다음과 같다.

- `full_connection = true`: 모든 `i < j` 쌍에 factor 생성
- `full_connection = false`: 각 pose가 바로 다음 pose와만 연결되는 chain 생성

## 2.3 실행 절차

다음 순서로 실험했다.

1. 현재 사용자 코드 상태 확인: `src/main.cpp:386`은 `false`
2. `false` 상태에서 Docker 내부 재빌드 및 실행
3. 로그 저장: `full_connection_false_run.log`
4. `src/main.cpp:386`을 `true`로 변경
5. Docker 내부 재빌드 및 실행
6. 로그 저장: `full_connection_true_run.log`
7. 실험 종료 후 `src/main.cpp:386`을 다시 `false`로 복원
8. `false` 상태로 한 번 더 재빌드하여 코드와 실행 파일 상태를 사용자 변경에 맞춤

즉, 보고서 작성 시점의 소스 상태는 다시 `full_connection = false`이다.

---

## 3. 그래프 구조 차이

현재 벤치마크는 7개 프레임(`x0` ~ `x6`)을 사용한다.

### 3.1 full_connection = true

- Prior factor: `x0` 고정
- Binary registration factors: 모든 프레임 쌍 연결
- factor 수: `C(7,2) = 21`

예시:

```text
x0-x1, x0-x2, x0-x3, x0-x4, x0-x5, x0-x6,
x1-x2, x1-x3, ..., x5-x6
```

### 3.2 full_connection = false

- Prior factor: `x0` 고정
- Binary registration factors: 인접 프레임만 연결
- factor 수: `7 - 1 = 6`

예시:

```text
x0-x1, x1-x2, x2-x3, x3-x4, x4-x5, x5-x6
```

즉, 이번 비교는 단순한 파라미터 스위치가 아니라 **그래프 topology 자체를 바꾸는 비교**다.

---

## 4. 원시 결과 요약

출처:

- chain 로그: `full_connection_false_run.log:692`
- full 로그: `full_connection_true_run.log:697`

| Factor | Chain(false) Mean T | Chain Mean R | Chain Max T | Chain Max R | Chain ms | Chain Iter |
|---|---:|---:|---:|---:|---:|---:|
| Point-to-Point | 0.228122 | 3.081607 | 0.786029 | 6.717394 | 436 | 19 |
| Point-to-Plane | 0.043472 | 0.428229 | 0.093456 | 0.695091 | 407 | 16 |
| GICP | 1.635813 | 5.098419 | 3.072735 | 9.087081 | 304 | 5 |
| VGICP | 1.211390 | 4.623732 | 2.657716 | 8.480980 | 574 | 20 |
| NDT | 0.394681 | 4.002565 | 1.288696 | 7.768180 | 1783 | 65 |
| LightNDT | 0.107436 | 0.858916 | 0.174495 | 1.402911 | 307 | 12 |
| LOAM_LIOSAM | 0.109340 | 0.506580 | 0.198987 | 0.886227 | 22 | 10 |

| Factor | Full(true) Mean T | Full Mean R | Full Max T | Full Max R | Full ms | Full Iter |
|---|---:|---:|---:|---:|---:|---:|
| Point-to-Point | 0.100990 | 0.436051 | 0.204713 | 0.687970 | 1949 | 27 |
| Point-to-Plane | 0.061799 | 0.430725 | 0.127270 | 0.852305 | 1268 | 16 |
| GICP | 0.085399 | 0.499411 | 0.168046 | 1.047266 | 1088 | 10 |
| VGICP | 0.117390 | 0.635614 | 0.202933 | 1.177762 | 1365 | 12 |
| NDT | 0.105657 | 0.730808 | 0.191520 | 1.320227 | 5620 | 71 |
| LightNDT | 0.159027 | 0.640614 | 0.337532 | 0.960458 | 728 | 6 |
| LOAM_LIOSAM | 0.225414 | 0.774622 | 0.616540 | 1.429813 | 62 | 9 |

---

## 5. 직접 비교표

아래 표는 `full - chain` 기준 변화량을 해석하기 쉽게 정리한 것이다.

| Factor | 시간 변화 | Iter 변화 | 정확도 변화 | 요약 |
|---|---|---|---|---|
| Point-to-Point | `436 -> 1949` (+347%) | `19 -> 27` 증가 | Mean/Max T,R 모두 full 우세 | full이 훨씬 정확하지만 매우 느림 |
| Point-to-Plane | `407 -> 1268` (+212%) | `16 -> 16` 동일 | chain이 약간 더 정확 | chain이 이득 |
| GICP | `304 -> 1088` (+258%) | `5 -> 10` 증가 | full 압승 | full이 정확도는 매우 좋지만 느림 |
| VGICP | `574 -> 1365` (+138%) | `20 -> 12` 감소 | full 압승 | full은 iter 감소, 시간은 증가 |
| NDT | `1783 -> 5620` (+215%) | `65 -> 71` 증가 | full 압승 | full이 훨씬 정확, 훨씬 느림 |
| LightNDT | `307 -> 728` (+137%) | `12 -> 6` 감소 | Mean R은 full 우세, Mean T는 chain 우세 | iter 감소가 시간 감소로 이어지지 않음 |
| LOAM_LIOSAM | `22 -> 62` (+182%) | `10 -> 9` 감소 | chain 우세 | chain이 더 빠르고 더 정확 |

---

## 6. 핵심 관찰

## 6.1 시간은 거의 전부 chain이 유리했다

이번 단일 실행에서는 모든 factor에서 `chain(false)`가 `full(true)`보다 빨랐다.

이유는 구조적으로 명확하다.

- full graph: factor 21개
- chain graph: factor 6개

즉 full은 chain보다 factor 수가 3.5배 많다.

LM 한 번의 outer iteration에서 해야 하는 일은 다음과 같다.

1. 모든 factor의 `linearize()` 호출
2. correspondence 갱신/재평가
3. Hessian/gradient 누적
4. sparse linear solve

그래프가 dense해지면 이 모든 비용이 같이 커진다.

그래서 이번 결과에서 runtime이 증가한 것은 매우 자연스럽다.

예를 들어:

- NDT: `1783 ms -> 5620 ms`
- GICP: `304 ms -> 1088 ms`
- Point-to-Point: `436 ms -> 1949 ms`

즉 `full_connection`은 정확도를 위한 전역 제약을 주는 대신, **한 iteration의 가격을 크게 올리는 설정**으로 작동했다.

## 6.2 iteration 수는 예측보다 복잡하게 움직였다

가장 중요한 관찰 중 하나는 다음이다.

> graph를 chain으로 바꾸면 iteration도 자동으로 줄어들 것이라고 기대하면 안 된다.

실제 결과는 세 부류로 갈렸다.

### A. full에서 iteration이 늘어난 경우

- Point-to-Point: `19 -> 27`
- GICP: `5 -> 10`
- NDT: `65 -> 71`

이 경우는 full graph의 장거리 제약들이 optimization을 더 어렵게 만들었거나, 더 많은 제약을 동시에 만족시키기 위해 더 많은 outer iteration이 필요했던 것으로 해석할 수 있다.

### B. full에서 iteration이 줄어든 경우

- VGICP: `20 -> 12`
- LightNDT: `12 -> 6`
- LOAM_LIOSAM: `10 -> 9`

이 경우는 full graph의 정보 전파가 전역 pose를 더 빠르게 정렬시켜 iteration 수는 줄인 것으로 볼 수 있다.

### C. iteration 변화가 거의 없는 경우

- Point-to-Plane: `16 -> 16`

즉 iteration은 graph density 하나만으로 결정되지 않는다.

정확히는,

- cost 구조
- factor 품질
- 장거리 pair의 유효성
- LM의 step acceptance

가 함께 작용한다.

## 6.3 정확도는 대체로 full graph가 강했다

이번 실험에서 가장 강하게 드러난 것은 다음이다.

> 전역 제약의 도움을 크게 받는 factor들은 full graph에서 정확도가 매우 좋아졌다.

대표 예시는 다음과 같다.

### NDT

- Mean T: `0.394681 -> 0.105657`
- Mean R: `4.002565 -> 0.730808`
- Max T: `1.288696 -> 0.191520`
- Max R: `7.768180 -> 1.320227`

NDT는 full graph에서 정확도 이득이 매우 컸다.

### GICP

- Mean T: `1.635813 -> 0.085399`
- Mean R: `5.098419 -> 0.499411`

GICP도 full graph에서 사실상 다른 알고리즘처럼 보일 정도로 정확도가 개선됐다.

### VGICP / Point-to-Point

이 둘도 full graph에서 Mean/Max error가 크게 개선됐다.

즉 full graph는 단순히 “느린 대신 조금 더 좋다”가 아니라,
일부 factor에서는 **정확도를 본질적으로 다른 수준으로 끌어올리는 역할**을 했다.

## 6.4 하지만 full graph가 항상 정확도를 올리지는 않았다

이번 단일 실행에서 예외도 있었다.

### Point-to-Plane

- Mean T는 chain이 더 좋음: `0.043472 < 0.061799`
- Mean R도 사실상 거의 동일

### LOAM_LIOSAM

- chain이 시간도 더 빠르고 정확도도 더 좋음

이는 중요한 시사점을 준다.

> 장거리 제약이 항상 이득은 아니다.

가능한 해석은 다음과 같다.

1. 일부 알고리즘은 인접 frame 정합만으로도 충분히 안정적이다.
2. long-range pair의 overlap/대응 품질이 낮으면, full graph의 추가 제약이 오히려 해가 될 수 있다.
3. 특히 특징 기반 또는 강한 국소 구조 기반 factor는 sparse chain에서도 충분히 잘 동작할 수 있다.

---

## 7. NDT 관점에서의 해석

이번 실험에서 사용자 질문과 가장 직접 연결되는 것은 NDT다.

### 7.1 chain으로 바꾸면 시간은 크게 준다

- full: `5620 ms`, `71 iter`
- chain: `1783 ms`, `65 iter`

즉 NDT에서 chain 전환은 **runtime 절감**에 매우 강하게 작동했다.

### 7.2 하지만 정확도 손실도 매우 컸다

- Mean T: `0.105657 -> 0.394681` (악화)
- Mean R: `0.730808 -> 4.002565` (악화)
- Max T: `0.191520 -> 1.288696` (악화)
- Max R: `1.320227 -> 7.768180` (악화)

즉 NDT는 이번 데이터/초기조건/설정에서는 full graph의 전역 보정에 크게 의존하고 있었다.

### 7.3 이것이 의미하는 바

NDT의 느림을 분석할 때 흔히

- NDT factor 자체가 느리다
- iteration이 많다

에만 집중하게 된다.

하지만 이번 결과는 또 다른 사실을 보여준다.

> 현재 NDT는 factor 하나하나의 느림뿐 아니라, 전역 정확도를 유지하려면 dense graph의 도움도 많이 받고 있다.

즉 chain 실험만 보고 “NDT가 빨라졌다”고 결론내리면 안 된다.

더 정확히는,

- `full`: 느리지만 global consistency를 얻음
- `chain`: 빠르지만 odometry drift 성격이 강해짐

으로 읽어야 한다.

---

## 8. 이번 비교에서 특히 중요한 오해 방지 포인트

## 8.1 raw LM error는 직접 비교 대상이 아니다

`full_connection=true`는 factor 개수가 더 많다.

따라서 optimizer가 출력하는 raw objective는 항 개수 자체가 달라져 직접 비교하면 안 된다.

이번 보고서에서는 Oracle 권고에 따라 다음 지표를 중심으로 비교했다.

- Mean/Max Translation Error
- Mean/Max Rotation Error
- Total time (ms)
- Iteration count

## 8.2 이번 결과는 단일 실행이다

`src/main.cpp`의 headless 경로는 GT에 random noise를 넣어 초기값을 만든다.

즉 단일 실행만으로 확정 결론을 내리면 위험하다.

이번 보고서는 **방향성 확인용 단일 run 비교**로 보는 것이 맞고,
논문/최종 보고서 수준 결론을 내려면 최소 5회 반복 평균/표준편차 비교가 필요하다.

## 8.3 full vs chain은 하이퍼파라미터가 아니라 실험 목적 자체를 바꾼다

이 설정은 단지 속도-정확도 조절 knob가 아니다.

- `full`: batch/global registration 성격
- `chain`: odometry/local registration 성격

즉 같은 알고리즘을 조금 튜닝한 비교가 아니라,
**문제 정의를 바꾸는 비교**에 가깝다.

---

## 9. 결론

이번 직접 실행 결과를 한 문장으로 요약하면 다음과 같다.

> `full_connection=false`(chain)는 거의 모든 factor에서 runtime을 크게 줄였지만, `full_connection=true`(full graph)는 여러 factor, 특히 NDT/GICP/VGICP/Point-to-Point에서 정확도를 크게 개선했다.

조금 더 세분화하면 다음과 같다.

1. **속도 우선**이면 chain이 유리하다.
   - factor 수가 21개에서 6개로 줄어들며 runtime 감소 효과가 매우 크다.

2. **전역 정확도 우선**이면 full graph가 유리하다.
   - 특히 NDT, GICP, VGICP는 full graph에서 정확도 개선 폭이 크다.

3. **iteration은 보조 지표**로 봐야 한다.
   - graph를 sparse하게 만든다고 iteration이 항상 줄어들지는 않았다.
   - 이번 실험에서는 “iteration 수”보다 “한 iteration의 비용” 차이가 runtime을 더 강하게 좌우했다.

4. **NDT 해석에서 특히 중요**하다.
   - chain으로 바꾸면 분명 빨라지지만, 지금 데이터/설정에서는 오차가 크게 나빠진다.
   - 따라서 NDT의 실용성을 평가할 때는 `full/chain`을 분리해 목적별로 봐야 한다.

---

## 10. 실무적 권장 해석

현재 결과만 기준으로 하면 다음처럼 해석하는 것이 가장 안전하다.

- `odometry 스타일 성능`을 보고 싶으면: `full_connection = false`
- `global batch registration 성능`을 보고 싶으면: `full_connection = true`
- `NDT의 구조적 느림`을 논할 때는: full graph가 주는 정확도 이득과 chain에서의 속도 이득을 분리해서 해석해야 함

즉,

> 앞으로 NDT/LightNDT/GICP 비교를 계속할 때는 `full`과 `chain`을 같은 표에 섞지 말고, 실험 목적별로 별도 표를 두는 것이 맞다.

---

## 11. 재현 로그

- chain 실행 로그: `full_connection_false_run.log`
- full 실행 로그: `full_connection_true_run.log`
- 실험 후 복원 상태: `src/main.cpp:386` → `full_connection = false`

---

## 12. 추가 검토: chain에서 회전 오차가 큰 이유가 구현 버그인가?

이번 비교 결과를 보고 가장 자연스럽게 드는 의심은 다음이다.

> chain 모드에서 rotation error가 이렇게 커진 것이, 혹시 factor 구현이나 chain/full 분기 코드의 버그 때문은 아닌가?

이 질문에 답하기 위해 다음 세 가지를 별도로 다시 점검했다.

1. chain/full에서 실제로 바뀌는 코드가 무엇인가
2. 각 factor의 residual/Jacobian/Hessian 구현에 명백한 오류가 있는가
3. 최종 회전 오차 계산 방식이 chain에 불리하게 잘못 정의되어 있는가

결론부터 말하면,

> 현재 코드 기준으로는 `구현 버그`보다 `graph topology 변화에 따른 회전 drift 증가`로 해석하는 근거가 훨씬 강하다.

## 12.1 chain/full에서 바뀌는 것은 factor 구현이 아니라 graph topology다

가장 먼저 확인한 것은 chain과 full에서 어떤 코드가 달라지는지였다.

그래프 생성부는 다음과 같다.

```cpp
int j_end = full_connection ? num_frames : std::min(i + 2, num_frames);
for (int j = i + 1; j < j_end; j++) {
  auto factor = create_factor(i, j, frames[i], active_voxelmaps[i], nullptr, frames[j]);
  graph.add(factor);
}
```

즉,

- `full_connection = true`면 모든 `(i, j)` 쌍에 factor 추가
- `full_connection = false`면 `(i, i+1)` chain만 추가

이다.

중요한 점은 다음이다.

> chain/full에 따라 **factor 클래스 자체가 달라지지 않는다.**

`create_factor()`를 다시 확인한 결과,

- Point-to-Point
- Point-to-Plane
- GICP
- VGICP
- NDT
- LightNDT
- LOAM_LIOSAM

모두 chain과 full에서 **동일한 factor 구현**, **동일한 correspondence tolerance**, **동일한 search mode/epsilon**, **동일한 thread 수**를 사용한다.

즉 이번 비교에서 달라지는 것은 사실상

- factor의 내용
- factor 파라미터
- error 계산식

이 아니라,

- **몇 개의 factor를 어떤 pose 쌍에 거느냐**

뿐이다.

이 사실만으로도, chain에서 오차가 커졌다고 해서 먼저 factor 구현 버그를 의심할 근거는 약해진다.

## 12.2 오차 계산 방식은 전역 pose drift를 그대로 측정하는 정상적인 정의다

최종 오차는 `summarize_results()`에서 다음처럼 계산한다.

```cpp
gtsam::Pose3 error = gt_pose.inverse() * opt_pose;
double te = error.translation().norm();
double re = error.rotation().axisAngle().second * 180.0 / M_PI;
```

이 정의는 다음 의미를 가진다.

- translation error: GT 대비 전역 병진 차이의 크기
- rotation error: GT 대비 전역 회전 차이의 axis-angle 크기

즉 chain에서 local pair 오차가 누적되면,

- 뒤 프레임으로 갈수록 pose drift가 쌓이고
- 그 drift가 전역 rotation error로 그대로 드러난다

이는 오히려 **odometry 스타일 평가에 자연스러운 메트릭**이다.

따라서 “chain에서 rotation error가 크게 나온다”는 관측만으로 메트릭 버그를 의심할 이유는 크지 않다.

## 12.3 factor 구현 재검토 결과: 공통적인 명백한 오류는 보이지 않는다

### Point-to-Point / Point-to-Plane

구현은 다음 형태다.

```text
residual = mean_B - T mean_A
H += J^T J
b += J^T r
```

Point-to-Plane은 여기에 normal projection만 추가한다.

이는 표준적인 SE(3) Gauss-Newton 형태로 보이며, chain/full에 따라 다르게 동작하도록 분기한 흔적도 없다.

### GICP / VGICP

구현은 다음 형태다.

```text
error = r^T M r
H += J^T M J
b += J^T M r
```

여기서 `M`은 fused covariance 기반의 Mahalanobis 행렬이다.

식 자체는 정상 범주이고, chain에서만 달라지는 구현은 발견되지 않았다.

### NDT / LightNDT

NDT는 현재 저장소 분석과 일치하게,

- score 기반 exponential cost
- residual-dependent weight
- H1-only Gauss-Newton 근사

형태로 구현돼 있다.

LightNDT는 동일한 voxel correspondence를 쓰되,

- pure Mahalanobis LSQ
- score/weight 없음

구조다.

이 둘 역시 chain/full 분기는 없고, graph에 들어가는 factor 수만 달라진다.

### LOAM_LIOSAM

LOAM_LIOSAM은 edge factor와 plane factor를 합산하는 구조다.

```text
error = edge_factor + plane_factor
```

이 구현도 chain/full에 따라 달라지는 분기는 보이지 않았다.

## 12.4 가장 강한 반증: chain 열화가 모든 factor에서 동시에 일어나지 않는다

만약 chain/full 차이에서 rotation error가 커진 원인이

- 공통 오차 계산 버그
- chain 생성 코드의 치명적 index 버그
- 모든 factor에 공통인 Jacobian/Hessian 버그

같은 것이었다면, 일반적으로 모든 factor에서 비슷한 방향의 붕괴가 나타나는 쪽이 더 자연스럽다.

그런데 실제 결과는 그렇지 않았다.

### chain에서 rotation error가 크게 나빠진 factor

- Point-to-Point: Mean R `3.081607` vs full `0.436051`
- GICP: Mean R `5.098419` vs full `0.499411`
- VGICP: Mean R `4.623732` vs full `0.635614`
- NDT: Mean R `4.002565` vs full `0.730808`

### chain에서 거의 유지되거나 오히려 더 나은 factor

- Point-to-Plane: Mean R `0.428229` vs full `0.430725`
- LOAM_LIOSAM: Mean R `0.506580` vs full `0.774622`

즉 chain 열화는 **공통 붕괴**가 아니라,

- 어떤 factor는 전역 제약 제거에 매우 민감하고
- 어떤 factor는 local chain에서도 안정적인

형태로 나타난다.

이 패턴은 구현 버그보다 **factor 성격 + graph topology 민감도 차이**로 보는 편이 훨씬 자연스럽다.

## 12.5 왜 특히 rotation error가 더 커 보이는가

rotation error는 단순 yaw 차이만 보는 것이 아니라,

- 전체 3D rotation 차이의 axis-angle 크기

를 본다.

또한 chain graph에서는 다음 구조가 생긴다.

```text
x0 - x1 - x2 - x3 - x4 - x5 - x6
```

이 구조에서는

- `x2`의 작은 회전 오차가
- `x3`, `x4`, `x5`, `x6`로 전파되고
- 마지막 프레임들에서는 누적 rotation drift로 보이기 쉽다

반면 full graph에서는

- `x0-x4`, `x0-x5`, `x1-x6` 같은 장거리 제약이 추가되어
- 중간에 누적된 회전 오차를 다시 잡아줄 수 있다.

즉 rotation error가 chain에서 커지는 것은,

- 메트릭이 회전에 민감해서가 아니라
- **회전 drift가 실제로 누적되고 있기 때문**

이라고 보는 것이 맞다.

## 12.6 가장 합리적인 최종 판단

지금까지의 코드를 기준으로 판단하면 다음과 같다.

1. chain/full은 factor 구현 변경이 아니라 graph connectivity 변경이다.
2. 오차 계산 방식은 전역 pose 기준으로 정상적이다.
3. 각 factor 구현은 수식상 일관된 형태이고, chain 전용 분기 버그는 보이지 않는다.
4. chain 열화는 모든 factor 공통 현상이 아니며, 특정 factor군에만 강하다.
5. 따라서 이번 회전 오차 증가는 구현 버그보다 **전역 제약 상실에 따른 회전 drift 증가**로 보는 것이 가장 타당하다.

조금 더 직설적으로 말하면,

> 지금 보이는 현상은 “chain 코드가 잘못 구현되어서 rotation error가 커진 것”이라기보다, “같은 factor를 full graph에서 chain graph로 옮겼더니 전역 보정이 사라져 회전 drift가 드러난 것”에 가깝다.

## 12.7 단, 완전히 배제하지는 못하는 설계상 민감 지점

버그라고 단정할 수는 없지만, chain에서 더 민감하게 작용할 수 있는 설계 특성은 하나 있다.

- graph는 항상 `i < j` 방향으로 factor를 만들고
- `target = i`, `source = j`로 두며
- 각 factor는 target frame 쪽 표현(점/복셀/특징)에 기대어 정합한다

이것이 full에서는 많은 장거리 제약으로 상쇄될 수 있지만,
chain에서는 지역적 bias가 누적될 가능성은 있다.

그러나 이것도 현재 증거만으로는 “구현 오류”보다는 **설계 선택의 drift 민감도**로 보는 편이 더 정확하다.
