# EM 알고리즘을 이용한 MoG (Mixture of Gaussians) 피팅 튜토리얼

> **문서 범위**: 포인트 클라우드가 주어졌을 때, EM 알고리즘으로 가우시안 혼합 모델(MoG/GMM)을 피팅하는
> **전체 과정**을 이론·수치 예제·의사 코드·구현 매핑과 함께 서술한다.
>
> **언어**: 본문 한국어, 수식 LaTeX (`$...$` / `$$...$$`).
>
> **코드 참조 규칙**: `파일명:라인` 형식으로 실제 구현 위치를 표기한다.
>
> **관련 문서**:
> - 수학적 증명·유도 → [`gmm-mathematical-foundations.md`](./gmm-mathematical-foundations.md)
> - 아키텍처·설계 결정 → [`gmm-design.md`](./gmm-design.md)
> - 복셀 구조 학습 → [`gmm-voxel-structure-study.md`](./gmm-voxel-structure-study.md)

---

## 목차

1. [왜 EM이 필요한가?](#1-왜-em이-필요한가)
2. [MoG 모델 정의](#2-mog-모델-정의)
3. [EM 알고리즘 개요](#3-em-알고리즘-개요)
4. [E-step: 책임도 계산](#4-e-step-책임도-계산)
5. [M-step: 파라미터 갱신](#5-m-step-파라미터-갱신)
6. [수렴 판정](#6-수렴-판정)
7. [수치 예제: 6개 포인트, 2개 가우시안](#7-수치-예제-6개-포인트-2개-가우시안)
8. [저수지 샘플링 — 포인트 수집 단계](#8-저수지-샘플링--포인트-수집-단계)
9. [Cold-Start vs Warm-Start](#9-cold-start-vs-warm-start)
10. [후처리: 정칙화·Pruning·재정규화](#10-후처리-정칙화pruning재정규화)
11. [엣지 케이스 처리](#11-엣지-케이스-처리)
12. [End-to-End 데이터 흐름](#12-end-to-end-데이터-흐름)
13. [의사 코드 종합](#13-의사-코드-종합)
14. [수식-코드 매핑 요약](#14-수식-코드-매핑-요약)
15. [참고 문헌](#15-참고-문헌)

---

## 1. 왜 EM이 필요한가?

### 1.1 문제 상황

복셀 하나에 $N$개의 3D 포인트 $\{\mathbf{p}_1, \ldots, \mathbf{p}_N\}$이 들어있다고 하자. 이 포인트들이 **하나의 가우시안**으로 잘 설명된다면 단순히 표본 평균과 표본 공분산을 구하면 된다. 그러나 LiDAR 스캔에서는 벽의 모서리, 바닥과 벽의 경계면 등이 하나의 복셀에 공존하는 경우가 빈번하다.

단일 가우시안으로 이런 **다봉(multimodal)** 분포를 모델링하면:

- 평균이 두 면 사이의 빈 공간에 위치
- 공분산이 비정상적으로 커짐
- 정합(registration) 시 잘못된 대응(correspondence)을 유발

### 1.2 MoG의 해법

$K$개의 가우시안을 **혼합(mixture)**하면 각 컴포넌트가 서로 다른 면(surface)을 담당한다. 문제는 어떤 포인트가 어떤 컴포넌트에 속하는지 모른다는 것이다 — 이것이 **숨은 변수(latent variable)** 문제다.

> **닭과 달걀 딜레마**:
> - 각 포인트의 소속을 알면 → 컴포넌트 파라미터를 쉽게 구할 수 있다
> - 컴포넌트 파라미터를 알면 → 각 포인트의 소속을 쉽게 구할 수 있다
> - 둘 다 모를 때 → **EM 알고리즘**이 이 교착을 깨준다

### 1.3 EM의 역할

EM(Expectation-Maximization)은 위 교착을 **반복적으로** 풀어나간다:

1. **E-step**: 현재 파라미터로 각 포인트가 각 컴포넌트에 속할 "확률"(책임도)을 계산
2. **M-step**: 그 책임도를 가중치로 삼아 파라미터를 갱신

이 두 단계를 로그-우도가 수렴할 때까지 반복하면, 국소 최적(local optimum)에 도달한다.

---

## 2. MoG 모델 정의

### 2.1 확률 모델

$K$개 컴포넌트의 가우시안 혼합 모델(MoG, Mixture of Gaussians)은 다음과 같이 정의된다:

$$p(\mathbf{x}) = \sum_{k=1}^{K} \pi_k \, \mathcal{N}(\mathbf{x} \mid \boldsymbol{\mu}_k, \boldsymbol{\Sigma}_k)$$

여기서:

| 기호 | 의미 | 제약 조건 |
|------|------|-----------|
| $K$ | 컴포넌트 수 | 본 구현: `max_components = 3` |
| $\pi_k$ | 혼합 가중치 | $\pi_k \geq 0$, $\sum_{k=1}^{K} \pi_k = 1$ |
| $\boldsymbol{\mu}_k$ | 컴포넌트 $k$의 평균 벡터 | $\mathbb{R}^3$ |
| $\boldsymbol{\Sigma}_k$ | 컴포넌트 $k$의 공분산 행렬 | $\mathbb{R}^{3 \times 3}$, 양의 정부호(PD) |

> **코드 참조**: `include/gmm/gmm_voxelmap_cpu.hpp:19-28` — `GMMComponent` 구조체가 위 파라미터를 저장한다.
>
> ```cpp
> struct GMMComponent {
>     Eigen::Vector3d mean;
>     Eigen::Matrix3d covariance;
>     double weight;  // = π_k
> };
> ```

### 2.2 다변량 가우시안 밀도

각 컴포넌트의 밀도 함수:

$$\mathcal{N}(\mathbf{x} \mid \boldsymbol{\mu}_k, \boldsymbol{\Sigma}_k) = \frac{1}{(2\pi)^{d/2} |\boldsymbol{\Sigma}_k|^{1/2}} \exp\!\left( -\frac{1}{2} (\mathbf{x} - \boldsymbol{\mu}_k)^T \boldsymbol{\Sigma}_k^{-1} (\mathbf{x} - \boldsymbol{\mu}_k) \right)$$

본 구현에서 $d = 3$ (3D 포인트 클라우드).

---

## 3. EM 알고리즘 개요

### 3.1 목표

관측 데이터 $\mathcal{P} = \{\mathbf{p}_1, \ldots, \mathbf{p}_N\}$이 주어졌을 때, **로그-우도(log-likelihood)**를 최대화하는 파라미터 $\Theta = \{(\pi_k, \boldsymbol{\mu}_k, \boldsymbol{\Sigma}_k)\}_{k=1}^{K}$를 찾는다:

$$\mathcal{L}(\Theta) = \sum_{i=1}^{N} \ln\!\left( \sum_{k=1}^{K} \pi_k \, \mathcal{N}(\mathbf{p}_i \mid \boldsymbol{\mu}_k, \boldsymbol{\Sigma}_k) \right)$$

합(sum)이 로그 안에 있어서 직접 미분으로 닫힌 형태(closed-form) 해를 구할 수 없다. EM은 이를 **하한(lower bound)**을 반복적으로 올려가며 해결한다.

### 3.2 알고리즘 흐름

```
초기화: Θ^(0) 설정 (k-means 등)
repeat:
    [E-step]  현재 Θ^(t)로 책임도 r_ik 계산
    [M-step]  r_ik를 이용하여 Θ^(t+1) 갱신
    수렴 여부 확인: |L^(t+1) - L^(t)| < τ ?
until 수렴 또는 최대 반복 도달
```

> **코드 참조**: Armadillo의 `arma::gmm_full` 클래스가 위 루프를 내부적으로 수행한다.
> - Cold-start: `src/gmm/mixture_em_backend.cpp:157-199`
> - Warm-start: `src/gmm/mixture_em_backend.cpp:204-280`

---

## 4. E-step: 책임도 계산

### 4.1 책임도(Responsibility) 정의

E-step에서는 각 포인트 $\mathbf{p}_i$가 컴포넌트 $k$에 "속할 확률"을 계산한다. 이를 **책임도**(responsibility) $\gamma_{ik}$라 한다:

$$\gamma_{ik} = \frac{\pi_k \, \mathcal{N}(\mathbf{p}_i \mid \boldsymbol{\mu}_k, \boldsymbol{\Sigma}_k)}{\displaystyle\sum_{j=1}^{K} \pi_j \, \mathcal{N}(\mathbf{p}_i \mid \boldsymbol{\mu}_j, \boldsymbol{\Sigma}_j)}$$

### 4.2 직관적 해석

- **분자**: "컴포넌트 $k$가 포인트 $\mathbf{p}_i$를 생성했을 확률" (사전 가중치 × 우도)
- **분모**: 모든 컴포넌트에 대한 정규화 상수
- 결과적으로 $\gamma_{ik} \in [0, 1]$이고 $\sum_{k=1}^{K} \gamma_{ik} = 1$

### 4.3 예시

포인트 $\mathbf{p}_i$가 컴포넌트 1의 평균 근처에 있고 컴포넌트 2의 평균에서 멀다면:
- $\gamma_{i1} \approx 0.95$  (컴포넌트 1에 대한 책임도 높음)
- $\gamma_{i2} \approx 0.05$  (컴포넌트 2에 대한 책임도 낮음)

이처럼 **"소속을 확률적으로 부드럽게 할당"**하는 것이 EM의 핵심이다. 이것은 K-means의 hard assignment ($\gamma_{ik} \in \{0, 1\}$)를 soft하게 일반화한 것이다.

---

## 5. M-step: 파라미터 갱신

### 5.1 유효 포인트 수

컴포넌트 $k$에 할당된 "유효 포인트 수":

$$N_k = \sum_{i=1}^{N} \gamma_{ik}$$

이것은 정수가 아니라 실수다. 예를 들어 3개 포인트의 책임도가 $(0.9, 0.7, 0.1)$이면 $N_k = 1.7$이다.

### 5.2 평균 갱신

$$\boldsymbol{\mu}_k^{\text{new}} = \frac{1}{N_k} \sum_{i=1}^{N} \gamma_{ik} \, \mathbf{p}_i$$

**해석**: 책임도를 가중치로 사용한 **가중 평균(weighted mean)**. 컴포넌트 $k$에 많이 속하는 포인트가 평균에 더 크게 기여한다.

### 5.3 공분산 갱신

$$\boldsymbol{\Sigma}_k^{\text{new}} = \frac{1}{N_k} \sum_{i=1}^{N} \gamma_{ik} \, (\mathbf{p}_i - \boldsymbol{\mu}_k^{\text{new}})(\mathbf{p}_i - \boldsymbol{\mu}_k^{\text{new}})^T$$

**해석**: 책임도를 가중치로 사용한 **가중 공분산(weighted covariance)**. 새로운 평균 $\boldsymbol{\mu}_k^{\text{new}}$을 기준으로 계산한다.

> **주의**: 공분산 갱신 시 반드시 갱신된 평균 $\boldsymbol{\mu}_k^{\text{new}}$을 사용해야 한다. 이전 평균으로 계산하면 편향이 발생한다.

### 5.4 혼합 가중치 갱신

$$\pi_k^{\text{new}} = \frac{N_k}{N}$$

**해석**: 전체 데이터 중 컴포넌트 $k$가 담당하는 비율. $\sum_k \pi_k^{\text{new}} = 1$을 자동으로 만족한다.

### 5.5 M-step이 가중 평균/가중 분산인 이유

M-step의 수식을 자세히 보면:

- 만약 모든 $\gamma_{ik}$가 0 또는 1이라면 (hard assignment) → 그냥 클러스터별 표본 평균/표본 공분산
- $\gamma_{ik}$가 연속값이라면 (soft assignment) → **가중 평균/가중 공분산**

이것이 바로 "가평균, 가분산"의 의미다. EM의 M-step은 본질적으로 **책임도로 가중된 통계량**을 계산하는 것이다.

---

## 6. 수렴 판정

### 6.1 로그-우도 기반 판정

매 반복 $t$마다 로그-우도를 계산하고, 변화량이 임계치 $\tau$ 미만이면 수렴으로 판정:

$$\left| \mathcal{L}^{(t+1)} - \mathcal{L}^{(t)} \right| < \tau$$

### 6.2 EM의 수렴 보장

EM 알고리즘은 다음 성질을 갖는다:

- **단조 증가**: 매 반복마다 $\mathcal{L}^{(t+1)} \geq \mathcal{L}^{(t)}$ (증명: Jensen 부등식)
- **수렴 보장**: 로그-우도는 위로 유계(bounded above)이므로 단조 수열이 수렴
- **국소 최적**: 전역 최적(global optimum)은 보장하지 않음 → 초기화가 중요

### 6.3 본 구현의 설정

| 파라미터 | 코드명 | 기본값 | 역할 |
|----------|--------|--------|------|
| $T_{\max}$ | `max_em_iterations` | 20 | EM 최대 반복 횟수 |
| $\tau$ | `convergence_tol` | $10^{-4}$ | 로그-우도 수렴 임계치 |

> **코드 참조**: `include/gmm/gmm_voxelmap_cpu.hpp:47-48`
>
> ```cpp
> int max_em_iterations = 20;
> double convergence_tol = 1e-4;
> ```

Cold-start에서는 `arma::gmm_full::learn()` 호출 시 `max_iter`와 `var_floor`를 전달한다:

> **코드 참조**: `src/gmm/mixture_em_backend.cpp:181-186`
>
> ```cpp
> bool status = model.learn(data, K, arma::eucl_dist,
>     arma::random_spread, 20, 0,  // 20 k-means iterations
>     params.convergence_tol, false);
> ```

---

## 7. 수치 예제: 6개 포인트, 2개 가우시안

### 7.1 데이터

6개의 2D 포인트 (설명 편의상 $d=2$):

$$\mathcal{P} = \{(1,1),\; (1.5, 1.2),\; (1.2, 0.8),\; (5,5),\; (5.5, 5.3),\; (4.8, 5.1)\}$$

눈으로 봐도 두 개의 군집이 보인다: 왼쪽 아래 $(1, 1)$ 부근과 오른쪽 위 $(5, 5)$ 부근.

### 7.2 초기화

K-means 또는 랜덤으로 초기 파라미터를 설정한다:

| | $\pi_k$ | $\boldsymbol{\mu}_k$ | $\boldsymbol{\Sigma}_k$ |
|---|---|---|---|
| $k=1$ | 0.5 | $(2, 2)$ | $\begin{pmatrix} 1 & 0 \\ 0 & 1 \end{pmatrix}$ |
| $k=2$ | 0.5 | $(4, 4)$ | $\begin{pmatrix} 1 & 0 \\ 0 & 1 \end{pmatrix}$ |

### 7.3 반복 1 — E-step

각 포인트에 대해 두 컴포넌트의 가우시안 밀도를 계산한 후 책임도를 구한다.

**포인트 $(1, 1)$의 경우:**

- 컴포넌트 1까지 거리: $\| (1,1) - (2,2) \|^2 = 2$ → $\mathcal{N} \propto \exp(-1) \approx 0.368$
- 컴포넌트 2까지 거리: $\| (1,1) - (4,4) \|^2 = 18$ → $\mathcal{N} \propto \exp(-9) \approx 0.000123$
- $\gamma_{1,1} = \frac{0.5 \times 0.368}{0.5 \times 0.368 + 0.5 \times 0.000123} \approx 0.9997$
- $\gamma_{1,2} \approx 0.0003$

**포인트 $(5, 5)$의 경우:**

- 컴포넌트 1까지 거리: $\| (5,5) - (2,2) \|^2 = 18$ → 매우 작은 밀도
- 컴포넌트 2까지 거리: $\| (5,5) - (4,4) \|^2 = 2$ → 높은 밀도
- $\gamma_{4,1} \approx 0.0003$, $\gamma_{4,2} \approx 0.9997$

**전체 책임도 행렬** (대략적 값):

| 포인트 | $\gamma_{i,1}$ | $\gamma_{i,2}$ |
|--------|-----------------|-----------------|
| $(1, 1)$ | ≈ 1.00 | ≈ 0.00 |
| $(1.5, 1.2)$ | ≈ 0.999 | ≈ 0.001 |
| $(1.2, 0.8)$ | ≈ 1.00 | ≈ 0.00 |
| $(5, 5)$ | ≈ 0.00 | ≈ 1.00 |
| $(5.5, 5.3)$ | ≈ 0.001 | ≈ 0.999 |
| $(4.8, 5.1)$ | ≈ 0.00 | ≈ 1.00 |

### 7.4 반복 1 — M-step

**유효 포인트 수:**
- $N_1 = 1.00 + 0.999 + 1.00 + \ldots \approx 3.0$
- $N_2 \approx 3.0$

**평균 갱신:**
- $\boldsymbol{\mu}_1^{\text{new}} \approx \frac{1}{3}[(1,1) + (1.5,1.2) + (1.2,0.8)] = (1.233, 1.0)$
- $\boldsymbol{\mu}_2^{\text{new}} \approx \frac{1}{3}[(5,5) + (5.5,5.3) + (4.8,5.1)] = (5.1, 5.133)$

**혼합 가중치:**
- $\pi_1^{\text{new}} = 3/6 = 0.5$, $\pi_2^{\text{new}} = 3/6 = 0.5$

### 7.5 수렴

이 예제에서는 두 군집이 잘 분리되어 있어 **1~2회 반복만에 거의 수렴**한다. 실제 LiDAR 데이터에서는 군집 경계가 모호하여 더 많은 반복이 필요할 수 있다.

---

## 8. 저수지 샘플링 — 포인트 수집 단계

### 8.1 왜 저수지 샘플링인가?

EM을 실행하려면 포인트들을 메모리에 보관해야 한다. 그러나:

- 실시간 LiDAR에서는 포인트가 **스트림**으로 도착
- 복셀당 수천 개의 포인트를 모두 저장하면 **메모리 폭발**
- 전체 포인트의 **균등 부분집합**을 효율적으로 유지해야 함

**저수지 샘플링(Reservoir Sampling, Algorithm R)**이 이 문제를 해결한다.

### 8.2 Algorithm R

용량 $C$의 저수지(reservoir)에 스트림에서 포인트를 하나씩 받으며 균등 무작위 부분집합을 유지한다:

```
Algorithm R (Reservoir Sampling)
─────────────────────────────────
입력: 포인트 스트림 p_1, p_2, ..., 저수지 용량 C

1. 처음 C개: 그대로 저장
   reservoir[i] = p_i  (i = 1, ..., C)

2. i번째 포인트 (i > C):
   j = random(1, i)     // 1~i 중 균등 난수
   if j ≤ C:
       reservoir[j] = p_i  // 확률 C/i로 기존 원소 교체
```

### 8.3 핵심 성질

- 어떤 시점이든 저수지의 각 원소는 **전체 스트림에서 균등하게 추출**
- 공간 복잡도: $O(C)$ 고정 (본 구현: $C = 256$)
- 시간 복잡도: 포인트당 $O(1)$

> **증명**: 수학적 증명은 [`gmm-mathematical-foundations.md` §2](./gmm-mathematical-foundations.md#2-저수지-샘플링-reservoir-sampling) 참조.

### 8.4 구현

> **코드 참조**: `src/gmm/gmm_voxelmap_cpu.cpp:36-59`
>
> ```cpp
> bool GMMVoxel::add(const Eigen::Vector4d& point) {
>     total_points_++;
>     if (total_points_ <= setting_.reservoir_capacity) {
>         reservoir_.push_back(point);
>     } else {
>         // Algorithm R: 확률 C/N으로 교체
>         std::uniform_int_distribution<int> dist(0, total_points_ - 1);
>         int j = dist(rng_);
>         if (j < setting_.reservoir_capacity) {
>             reservoir_[j] = point;
>         }
>     }
>     needs_update_ = true;
>     return true;
> }
> ```

---

## 9. Cold-Start vs Warm-Start

### 9.1 의사 결정 흐름도

```
                     finalize() 호출
                          │
                          ▼
               ┌─ 기존 GMM 결과 있음? ─┐
               │                        │
              Yes                      No
               │                        │
               ▼                        ▼
         ┌─────────┐            ┌──────────┐
         │Warm-Start│            │Cold-Start │
         │(기존 유지)│            │(처음부터) │
         └─────────┘            └──────────┘
               │                        │
               ▼                        ▼
     arma::keep_existing       arma::random_spread
     기존 파라미터로 EM 시작     k-means 20회 → EM
               │                        │
               └────────┬───────────────┘
                        ▼
               extract_result()
               (정칙화 + pruning + 재정규화)
```

### 9.2 Cold-Start (최초 피팅)

복셀에 GMM 결과가 아직 없을 때:

1. 저수지의 포인트를 Armadillo 행렬로 변환
2. `arma::random_spread` — 랜덤 초기 배치 후 **k-means 20회** 반복으로 초기 중심 결정
3. EM 반복으로 파라미터 정밀 조정

> **코드 참조**: `src/gmm/mixture_em_backend.cpp:157-199`
>
> ```cpp
> GMMFitResult fit_gmm(const std::vector<Eigen::Vector4d>& points,
>                      int K, const GMMFitParams& params) {
>     // ... 데이터 변환 ...
>     arma::gmm_full model;
>     bool status = model.learn(data, K, arma::eucl_dist,
>         arma::random_spread, 20, 0,       // k-means 20회
>         params.convergence_tol, false);
>     return extract_result(model, K, params);
> }
> ```

### 9.3 Warm-Start (증분 갱신)

복셀에 이미 GMM 결과가 있고, 새 포인트가 추가된 경우:

1. 기존 GMM 파라미터로 Armadillo 모델 초기화 (`model.set_params()`)
2. `arma::keep_existing` — **기존 파라미터에서 바로 EM** 실행 (k-means 건너뜀)
3. 새 데이터에 맞게 파라미터 미세 조정

> **코드 참조**: `src/gmm/mixture_em_backend.cpp:204-280`
>
> ```cpp
> GMMFitResult fit_gmm(const std::vector<Eigen::Vector4d>& points,
>                      int K, const GMMFitParams& params,
>                      const GMMFitResult& prev_result) {
>     // 기존 파라미터를 Armadillo 형식으로 설정
>     model.set_params(means, covs, hefts);
>     // 정칙화는 extract_result에서 하므로 여기서는 제거
>     bool status = model.learn(data, K, arma::eucl_dist,
>         arma::keep_existing, 0, 0,  // k-means 생략
>         params.convergence_tol, false);
>     return extract_result(model, K, params);
> }
> ```

### 9.4 Warm-Start의 이점

- **속도**: k-means 20회를 건너뛰므로 수렴이 빠름
- **안정성**: 이전 결과에서 시작하므로 급격한 변화 방지
- **점진적 적응**: 새 포인트가 기존 분포와 유사하면 소수 반복으로 수렴

### 9.5 구현에서의 분기

> **코드 참조**: `src/gmm/gmm_voxelmap_cpu.cpp:63-100`
>
> ```cpp
> void GMMVoxel::finalize() {
>     if (reservoir_.size() < 2) {
>         // single_component_fallback 호출
>         return;
>     }
>     GMMFitResult result;
>     if (!components_.empty()) {
>         // Warm-start: 기존 결과 전달
>         GMMFitResult prev;
>         // ... 기존 컴포넌트를 prev로 변환 ...
>         result = fit_gmm(reservoir_, K, params, prev);
>     } else {
>         // Cold-start: 기존 결과 없음
>         result = fit_gmm(reservoir_, K, params);
>     }
> }
> ```

---

## 10. 후처리: 정칙화·Pruning·재정규화

EM이 끝난 뒤 Armadillo가 반환한 결과를 그대로 사용하지 않는다. 수치적 안정성과 품질을 위해 **세 단계 후처리**를 거친다.

### 10.1 공분산 정칙화 (Regularization)

EM 결과의 공분산이 특이(singular)하거나 거의 특이할 수 있다 (예: 포인트가 일직선/평면에 몰린 경우). 고유값 분해 후 최소 고유값을 클램핑한다:

$$\lambda_i' = \max(\lambda_i, \;\varepsilon_\lambda \cdot \lambda_{\max})$$

여기서:
- $\lambda_i$: 공분산의 $i$번째 고유값
- $\lambda_{\max}$: 최대 고유값
- $\varepsilon_\lambda$: 정칙화 비율 (기본값 $10^{-3}$)

그리고 복원:

$$\boldsymbol{\Sigma}_k' = V \, \text{diag}(\lambda_1', \lambda_2', \lambda_3') \, V^T$$

> **코드 참조**: `src/gmm/mixture_em_backend.cpp:106-131`
>
> ```cpp
> Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
> Eigen::Vector3d eigenvalues = solver.eigenvalues();
> double max_eigenvalue = eigenvalues.maxCoeff();
> double min_eigenvalue = params.covariance_regularization * max_eigenvalue;
> for (int d = 0; d < 3; d++) {
>     eigenvalues(d) = std::max(eigenvalues(d), min_eigenvalue);
> }
> cov = solver.eigenvectors() * eigenvalues.asDiagonal()
>     * solver.eigenvectors().transpose();
> ```

### 10.2 Pruning (가중치 기반 제거)

혼합 가중치 $\pi_k$가 너무 작은 컴포넌트는 의미 없는 "유령 컴포넌트"다. 임계치 미만은 제거한다:

$$\pi_k < \pi_{\min} \implies \text{컴포넌트 } k \text{ 제거}$$

기본값: $\pi_{\min} = 0.01$ (1% 미만)

> **코드 참조**: `src/gmm/mixture_em_backend.cpp:138-142`
>
> ```cpp
> if (weight >= params.min_weight_threshold) {
>     result.components.push_back({mean, cov, weight});
> }
> ```

### 10.3 재정규화 (Re-normalization)

Pruning으로 컴포넌트가 제거되면 남은 가중치의 합이 1이 안 된다. 재정규화:

$$\pi_k' = \frac{\pi_k}{\sum_{j \in \text{survived}} \pi_j}$$

> **코드 참조**: `src/gmm/mixture_em_backend.cpp:145-149`
>
> ```cpp
> double total_weight = 0;
> for (auto& comp : result.components)
>     total_weight += comp.weight;
> for (auto& comp : result.components)
>     comp.weight /= total_weight;
> ```

---

## 11. 엣지 케이스 처리

### 11.1 포인트 수 부족 ($N < 2$)

포인트가 0~1개면 EM을 실행할 수 없다. **단일 컴포넌트 폴백(fallback)**으로 처리:

- $N = 0$: 빈 결과 반환
- $N = 1$: 평균 = 유일한 포인트, 공분산 = $\varepsilon I$, 가중치 = 1.0

> **코드 참조**: `src/gmm/mixture_em_backend.cpp:17-57`

### 11.2 $N < K$ (포인트 수 < 컴포넌트 수)

포인트가 2개인데 $K = 3$이면 3개 가우시안을 피팅할 수 없다. 실제 컴포넌트 수를 줄인다:

$$K_{\text{actual}} = \min(K, N)$$

> **코드 참조**: `src/gmm/mixture_em_backend.cpp:167-168`
>
> ```cpp
> int actual_K = std::min(K, static_cast<int>(points.size()));
> ```

### 11.3 모든 컴포넌트가 Pruning됨

극히 드물지만, 모든 컴포넌트의 가중치가 $\pi_{\min}$ 미만일 수 있다. 이 경우 결과의 `success` 플래그가 `false`이고, 상위 코드에서 처리한다.

### 11.4 퇴화 공분산 (Degenerate Covariance)

포인트가 동일 직선이나 평면에 놓이면 공분산의 일부 고유값이 0이다. §10.1의 정칙화가 이를 처리:
- 평면 분포: 1개 고유값이 매우 작음 → 클램핑으로 최소 두께 보장
- 선형 분포: 2개 고유값이 매우 작음 → 클램핑

---

## 12. End-to-End 데이터 흐름

전체 파이프라인을 ASCII 다이어그램으로 나타내면:

```
LiDAR 포인트 스트림
        │
        ▼
┌───────────────────┐
│  GMMVoxelMapCPU   │  insert() → 해시 기반 복셀 분배
│  ::insert()       │
└───────┬───────────┘
        │  복셀별 처리
        ▼
┌───────────────────┐
│  GMMVoxel::add()  │  저수지 샘플링 (Algorithm R)
│                   │  용량 C=256, 스트림에서 균등 추출
│  reservoir_[]     │
└───────┬───────────┘
        │  finalize() 호출 시
        ▼
┌───────────────────────────────────────────────┐
│  GMMVoxel::finalize()                         │
│                                               │
│  N < 2? ──Yes──→ single_component_fallback()  │
│    │                                          │
│   No                                          │
│    │                                          │
│  기존 GMM 있음? ──Yes──→ Warm-Start           │
│    │                      fit_gmm(..., prev)  │
│   No                                          │
│    │                                          │
│  Cold-Start                                   │
│  fit_gmm(...)                                 │
└───────┬───────────────────────────────────────┘
        │
        ▼
┌───────────────────────────────────────────────┐
│  mixture_em_backend.cpp                       │
│                                               │
│  1. Eigen → Armadillo 변환 (4D→3D)           │
│  2. arma::gmm_full::learn()                   │
│     ├─ Cold: random_spread + k-means(20) + EM │
│     └─ Warm: keep_existing + EM               │
│  3. extract_result()                          │
│     ├─ 고유값 정칙화 (eigenvalue clamping)     │
│     ├─ 저가중치 pruning (π < 0.01 제거)       │
│     └─ 가중치 재정규화 (Σπ = 1)              │
└───────┬───────────────────────────────────────┘
        │
        ▼
┌───────────────────┐
│  GMMFitResult     │  컴포넌트별: {μ_k, Σ_k, π_k}
│  → components_    │  Factor에서 정합에 사용
└───────────────────┘
```

### 타이밍

- `add()`: 포인트 삽입 시마다 호출 — $O(1)$
- `finalize()`: 지연 호출(deferred) — 검색(`knn_search`) 또는 명시적 `finalize_all()` 시 트리거
- `finalize_all()`: **OMP 병렬** — 모든 더티 복셀에 대해 동시 EM 실행

> **코드 참조**: `src/gmm/gmm_voxelmap_cpu.cpp:194-208`
>
> ```cpp
> void GMMVoxelMapCPU::finalize_all() {
>     #pragma omp parallel for schedule(dynamic) num_threads(num_threads_)
>     for (int i = 0; i < dirty.size(); i++) {
>         dirty[i]->finalize();
>     }
> }
> ```

---

## 13. 의사 코드 종합

### 13.1 EM 알고리즘 전체

```
Algorithm: EM for Mixture of Gaussians
──────────────────────────────────────
입력: 포인트 P = {p_1, ..., p_N}, 컴포넌트 수 K
      수렴 임계치 τ, 최대 반복 T_max
출력: {(π_k, μ_k, Σ_k)}_{k=1}^K

[초기화]
  Cold-start:  k-means로 K개 클러스터 → 각 클러스터의 mean/cov/weight
  Warm-start:  이전 반복의 (π_k, μ_k, Σ_k)를 그대로 사용

[반복]
for t = 1, 2, ..., T_max:

    ── E-step ──
    for i = 1 to N:
        for k = 1 to K:
            γ_ik = π_k · N(p_i | μ_k, Σ_k) / Σ_j [π_j · N(p_i | μ_j, Σ_j)]

    ── M-step ──
    for k = 1 to K:
        N_k = Σ_i γ_ik
        μ_k = (1/N_k) · Σ_i γ_ik · p_i
        Σ_k = (1/N_k) · Σ_i γ_ik · (p_i - μ_k)(p_i - μ_k)^T
        π_k = N_k / N

    ── 수렴 확인 ──
    L = Σ_i ln(Σ_k π_k · N(p_i | μ_k, Σ_k))
    if |L - L_prev| < τ:
        break
    L_prev = L

[후처리]
  for k = 1 to K:
      정칙화: Σ_k의 고유값 클램핑
      pruning: π_k < π_min이면 컴포넌트 제거
  재정규화: 남은 π_k들의 합을 1로 정규화

return {(π_k, μ_k, Σ_k)}
```

### 13.2 복셀 레벨 전체 파이프라인

```
Algorithm: GMMVoxel Pipeline
────────────────────────────
입력: 포인트 스트림 (LiDAR 스캔), 저수지 용량 C, 컴포넌트 수 K

[포인트 수집 — 실시간]
for each incoming point p:
    voxel = hash(p)에 해당하는 GMMVoxel
    voxel.add(p):
        if total_points ≤ C:
            reservoir에 추가
        else:
            확률 C/total_points로 reservoir의 랜덤 위치에 교체

[EM 피팅 — 지연 실행]
when finalize() triggered:
    if |reservoir| < 2:
        단일 컴포넌트 폴백
        return
    K_actual = min(K, |reservoir|)
    if 기존 GMM 없음:
        cold-start: random_spread + k-means(20) + EM
    else:
        warm-start: keep_existing + EM (기존 파라미터에서 시작)
    후처리: 정칙화 → pruning → 재정규화

[정합에 사용]
Factor에서 knn_search() 호출 시:
    아직 finalize 안 된 복셀 → lazy finalize 트리거
    가장 가까운 복셀의 GMM 컴포넌트를 WTA로 선택
```

---

## 14. 수식-코드 매핑 요약

본 문서에서 다룬 핵심 수식과 구현 위치를 정리한다. 상세 매핑(22건)은 [`gmm-mathematical-foundations.md` §11](./gmm-mathematical-foundations.md#11-수식-코드-매핑-종합-테이블) 참조.

| 수식 | 설명 | 코드 위치 |
|------|------|-----------|
| $p(\mathbf{x}) = \sum_k \pi_k \mathcal{N}(\mathbf{x} \mid \boldsymbol{\mu}_k, \boldsymbol{\Sigma}_k)$ | MoG 모델 | `gmm_voxelmap_cpu.hpp:19-28` (GMMComponent) |
| $\gamma_{ik} = \frac{\pi_k \mathcal{N}_k}{\sum_j \pi_j \mathcal{N}_j}$ | E-step 책임도 | Armadillo 내부 (`arma::gmm_full::learn`) |
| $\boldsymbol{\mu}_k = \frac{1}{N_k}\sum_i \gamma_{ik}\mathbf{p}_i$ | M-step 평균 | Armadillo 내부 (`arma::gmm_full::learn`) |
| $\boldsymbol{\Sigma}_k = \frac{1}{N_k}\sum_i \gamma_{ik}(\mathbf{p}_i - \boldsymbol{\mu}_k)(\mathbf{p}_i - \boldsymbol{\mu}_k)^T$ | M-step 공분산 | Armadillo 내부 (`arma::gmm_full::learn`) |
| $\pi_k = N_k / N$ | M-step 가중치 | Armadillo 내부 (`arma::gmm_full::learn`) |
| 저수지 샘플링 (확률 $C/N$ 교체) | Algorithm R | `gmm_voxelmap_cpu.cpp:36-59` |
| $\lambda_i' = \max(\lambda_i, \varepsilon\lambda_{\max})$ | 고유값 정칙화 | `mixture_em_backend.cpp:106-131` |
| $\pi_k < \pi_{\min} \implies$ 제거 | Pruning | `mixture_em_backend.cpp:138-142` |
| $\pi_k' = \pi_k / \sum_j \pi_j$ | 재정규화 | `mixture_em_backend.cpp:145-149` |
| Cold-start: `random_spread` + k-means(20) + EM | 초기화 전략 | `mixture_em_backend.cpp:181-186` |
| Warm-start: `keep_existing` + EM | 증분 갱신 | `mixture_em_backend.cpp:244-247` |

---

## 15. 참고 문헌

1. **Dempster, A. P., Laird, N. M., & Rubin, D. B.** (1977). *Maximum Likelihood from Incomplete Data via the EM Algorithm.* Journal of the Royal Statistical Society, Series B, 39(1), 1–38.

2. **Bishop, C. M.** (2006). *Pattern Recognition and Machine Learning.* Chapter 9: Mixture Models and EM. Springer.

3. **Vitter, J. S.** (1985). *Random Sampling with a Reservoir.* ACM Transactions on Mathematical Software, 11(1), 37–57.

4. **Sanderson, C., & Curtin, R.** (2016). *Armadillo: a template-based C++ library for linear algebra.* Journal of Open Source Software, 1(2), 26.

5. **Magnusson, M.** (2009). *The Three-Dimensional Normal-Distributions Transform.* PhD Thesis, Örebro University.

> 수학적 유도의 상세 증명은 [`gmm-mathematical-foundations.md`](./gmm-mathematical-foundations.md) 참조.
> 아키텍처 설계 결정 근거는 [`gmm-design.md`](./gmm-design.md) 참조.

---

최종 수정: 2026-03-31
