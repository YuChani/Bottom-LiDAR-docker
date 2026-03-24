# GMM 기반 LiDAR 정합: 수학적 기초

> **문서 범위**: 이 프로젝트의 GMM 구현에 사용된 **모든 수학적 방법**을 유도·증명·해석한다.
> 아키텍처·코드 구조는 [`gmm-design.md`](./gmm-design.md)를 참고한다.
>
> **언어**: 본문 한국어, 수식 LaTeX (`$...$` / `$$...$$`).
>
> **코드 참조 규칙**: `파일명:라인` 형식으로 실제 구현 위치를 표기한다.

---

## 목차

1. [표기법 및 기호 정의](#1-표기법-및-기호-정의)
2. [저수지 샘플링 (Reservoir Sampling)](#2-저수지-샘플링-reservoir-sampling)
3. [가우시안 혼합 모델과 EM 알고리즘](#3-가우시안-혼합-모델과-em-알고리즘)
4. [공분산 정칙화 (Regularization)](#4-공분산-정칙화-regularization)
5. [초기화 전략](#5-초기화-전략)
6. [후처리: Pruning과 재정규화](#6-후처리-pruning과-재정규화)
7. [마할라노비스 거리와 Winner-Take-All 대응](#7-마할라노비스-거리와-winner-take-all-대응)
8. [π_k 가중 비용 함수](#8-π_k-가중-비용-함수)
9. [SE(3) Jacobian 유도](#9-se3-jacobian-유도)
10. [Gauss-Newton Hessian과 최적화](#10-gauss-newton-hessian과-최적화)
11. [수식-코드 매핑 종합 테이블](#11-수식-코드-매핑-종합-테이블)
12. [참고 문헌](#12-참고-문헌)

---

## 1. 표기법 및 기호 정의

### 1.1 스칼라·벡터·행렬

| 기호 | 의미 | 차원 |
|------|------|------|
| $N$ | 복셀에 관측된 총 포인트 수 | 스칼라 |
| $C$ | 저수지 용량 (`reservoir_capacity`) | 스칼라 |
| $K$ | 가우시안 컴포넌트 수 (`max_components`) | 스칼라 |
| $d$ | 공간 차원 (본 구현에서 $d = 3$) | 스칼라 |
| $\mathbf{p}_i$ | $i$-번째 3D 포인트 | $\mathbb{R}^3$ |
| $\boldsymbol{\mu}_k$ | 컴포넌트 $k$의 평균 | $\mathbb{R}^3$ |
| $\boldsymbol{\Sigma}_k$ | 컴포넌트 $k$의 공분산 | $\mathbb{R}^{3\times3}$ |
| $\pi_k$ | 컴포넌트 $k$의 혼합 가중치, $\sum_k \pi_k = 1$ | 스칼라 |
| $\gamma_{ik}$ | 포인트 $i$에 대한 컴포넌트 $k$의 책임도 (responsibility) | $[0,1]$ |
| $T \in SE(3)$ | 강체 변환 (회전 $R \in SO(3)$ + 병진 $\mathbf{t} \in \mathbb{R}^3$) | $4\times4$ |
| $\boldsymbol{\xi} = (\boldsymbol{\phi}, \boldsymbol{\rho})$ | SE(3) 리 대수 원소: 회전 $\boldsymbol{\phi} \in \mathbb{R}^3$, 병진 $\boldsymbol{\rho} \in \mathbb{R}^3$ | $\mathbb{R}^6$ |
| $[\mathbf{v}]_\times$ | 벡터 $\mathbf{v} \in \mathbb{R}^3$의 반대칭 행렬 (hat map) | $\mathbb{R}^{3\times3}$ |

### 1.2 하이퍼파라미터

| 기호 | 코드명 | 기본값 | 역할 |
|------|--------|--------|------|
| $K$ | `max_components` | 3 | 복셀당 최대 컴포넌트 수 |
| $T_{\max}$ | `max_em_iterations` | 20 | EM 최대 반복 |
| $\tau$ | `convergence_tol` | $10^{-4}$ | 로그-우도 수렴 임계치 |
| $\varepsilon$ | `covariance_regularization` | $10^{-3}$ | 공분산 대각 정칙화 |
| $\pi_{\min}$ | `min_weight_threshold` | 0.01 | 저가중치 pruning 임계치 |
| $C$ | `reservoir_capacity` | 256 | 저수지 최대 크기 |
| $\varepsilon_\lambda$ | `regularization_epsilon` | $10^{-3}$ | 고유값 클램핑 비율 |

> **코드 참조**: `include/gmm/mixture_em_backend.hpp:18-24`, `include/gmm/gmm_voxelmap_cpu.hpp:41-54`

### 1.3 4D 동차 좌표 규약

본 구현은 Eigen `Vector4d`를 사용한다. 포인트는 $(\mathbf{p}, 1)$, GMM 평균은 $(\boldsymbol{\mu}_k, 0)$으로 표현한다. $w = 0$ 규약은 공분산 행렬의 4행/4열이 항상 0임을 의미하며, 모든 수학적 연산은 상위-좌측 $3 \times 3$ 블록에서만 유효하다.

> **코드 참조**: `include/gmm/gmm_voxelmap_cpu.hpp:21-24` — `mean.w = 0`, `cov` 4×4 중 3×3만 사용

---

## 2. 저수지 샘플링 (Reservoir Sampling)

### 2.1 문제 정의

스트림으로 도착하는 $N$개 포인트에서 저장 공간 $C$ ($C \ll N$)만큼의 **균일 무작위 표본**을 유지해야 한다. 포인트 분포에 대한 사전 지식이 없으므로 $N$은 미리 알 수 없다.

### 2.2 Algorithm R (Vitter, 1985)

> **코드 참조**: `src/gmm/gmm_voxelmap_cpu.cpp:32-54`

**채우기 단계** ($N \le C$): 도착하는 모든 포인트를 무조건 저수지에 삽입한다.

$$
R[N] \leftarrow \mathbf{p}_N \quad (N \le C)
$$

**교체 단계** ($N > C$): $j \sim \text{Uniform}\{0, 1, \ldots, N-1\}$를 추출한다.

$$
\text{if } j < C: \quad R[j] \leftarrow \mathbf{p}_N \quad \text{(확률 } C/N \text{으로 교체)}
$$

$$
\text{if } j \ge C: \quad \text{버림} \quad \text{(확률 } 1 - C/N \text{)}
$$

> **코드**: 라인 46에서 `std::uniform_int_distribution<size_t> dist(0, total_points_seen_ - 1)`, 라인 48에서 `if (j < capacity)` 조건 검사.

### 2.3 균일성 증명 (수학적 귀납법)

**정리**: Algorithm R 실행 후, $N$번째 포인트까지 관측한 상태에서 저수지 내 각 슬롯이 특정 포인트 $\mathbf{p}_i$ ($1 \le i \le N$)를 담고 있을 확률은 모두 $C/N$이다.

**증명** (강한 귀납법):

**기저**: $N = C$일 때, 모든 $C$개 포인트가 저수지에 있으므로 각 포인트의 포함 확률은 $C/C = 1$. ✓

**귀납 가정**: $N = n$ ($n \ge C$)일 때, 각 포인트 $\mathbf{p}_i$ ($1 \le i \le n$)가 저수지에 있을 확률이 $C/n$이라고 가정한다.

**귀납 단계**: $(n+1)$-번째 포인트 $\mathbf{p}_{n+1}$이 도착한다.

1. **새 포인트의 포함 확률**:

$$
P(\mathbf{p}_{n+1} \in R) = P(j < C) = \frac{C}{n+1}
$$

2. **기존 포인트 $\mathbf{p}_i$ ($i \le n$)의 생존 확률**: $\mathbf{p}_i$가 $n$단계에서 저수지에 있었고 ($C/n$), $n+1$단계에서 교체되지 않아야 한다.

교체가 발생할 확률은 $C/(n+1)$이고, 교체 시 특정 슬롯이 선택될 확률은 $1/C$이다. 따라서 특정 포인트가 교체될 확률:

$$
P(\mathbf{p}_i \text{ 교체}) = \frac{C}{n+1} \cdot \frac{1}{C} = \frac{1}{n+1}
$$

생존 확률:

$$
P(\mathbf{p}_i \in R \mid n+1) = \frac{C}{n} \cdot \left(1 - \frac{1}{n+1}\right) = \frac{C}{n} \cdot \frac{n}{n+1} = \frac{C}{n+1} \quad \square
$$

### 2.4 EM 입력으로서의 저수지 표본

저수지는 전체 스트림의 **비편향 균일 표본**이다. EM 알고리즘이 이 표본으로 추정한 GMM 파라미터 $\hat{\theta}$는 전체 모집단에 대한 MLE의 일치 추정량(consistent estimator)이 된다. 표본 크기 $C$가 충분히 크면 ($C \gg K \cdot d^2$) Fisher 정보 행렬에 의한 Cramér-Rao 하한에 근접한다.

### 2.5 w-component 제로화

> **코드 참조**: `src/gmm/gmm_voxelmap_cpu.cpp:39` — `pt(3) = 0.0`

소스 포인트는 $w = 1$ (위치)이지만 저수지에 삽입할 때 $w = 0$으로 강제한다. 이는:
- 표본 평균 $\bar{\mathbf{p}}$의 $w$ 성분이 0이 되어 GMM 평균의 $w = 0$ 규약과 일치
- 공분산의 4행/4열이 자연스럽게 0이 되어 3×3 블록 외의 오염 방지

이 처리가 없으면 Phase 3.6에서 발견된 w-component 누출 버그가 발생한다: $w = 1$로 계산된 평균의 동차 좌표가 역공분산 연산에서 Mahalanobis 거리를 $\sim 35{,}000$ 수준으로 폭발시킨다.

---

## 3. 가우시안 혼합 모델과 EM 알고리즘

### 3.1 가우시안 혼합 모델 정의

$K$개 가우시안 컴포넌트로 구성된 혼합 모델의 확률 밀도 함수:

$$
p(\mathbf{x} \mid \Theta) = \sum_{k=1}^{K} \pi_k \cdot \mathcal{N}(\mathbf{x} \mid \boldsymbol{\mu}_k, \boldsymbol{\Sigma}_k)
$$

여기서 $\Theta = \{\pi_k, \boldsymbol{\mu}_k, \boldsymbol{\Sigma}_k\}_{k=1}^{K}$이고, 다변량 가우시안:

$$
\mathcal{N}(\mathbf{x} \mid \boldsymbol{\mu}_k, \boldsymbol{\Sigma}_k) = \frac{1}{(2\pi)^{d/2} |\boldsymbol{\Sigma}_k|^{1/2}} \exp\left(-\frac{1}{2} (\mathbf{x} - \boldsymbol{\mu}_k)^\top \boldsymbol{\Sigma}_k^{-1} (\mathbf{x} - \boldsymbol{\mu}_k)\right)
$$

본 구현에서 $d = 3$ (3D 공간), $K = \min(\texttt{max\_components}, N)$.

> **코드 참조**: 혼합 모델은 Armadillo `arma::gmm_full`이 내부적으로 구현하며, 파라미터는 `mixture_em_backend.cpp`에서 추출한다.

### 3.2 잠재 변수 모델

각 포인트 $\mathbf{x}_i$에 대해 잠재 변수 $z_i \in \{1, \ldots, K\}$를 도입한다. $z_i = k$는 "포인트 $i$가 컴포넌트 $k$에서 생성되었음"을 뜻한다.

**결합 확률**: $p(\mathbf{x}_i, z_i = k \mid \Theta) = \pi_k \cdot \mathcal{N}(\mathbf{x}_i \mid \boldsymbol{\mu}_k, \boldsymbol{\Sigma}_k)$

**주변화**: $p(\mathbf{x}_i \mid \Theta) = \sum_k p(\mathbf{x}_i, z_i = k \mid \Theta)$ — 이것이 §3.1의 혼합 밀도이다.

### 3.3 로그-우도와 직접 최적화의 어려움

관측 데이터 $\mathcal{X} = \{\mathbf{x}_1, \ldots, \mathbf{x}_N\}$의 로그-우도:

$$
\mathcal{L}(\Theta) = \sum_{i=1}^{N} \log \left( \sum_{k=1}^{K} \pi_k \cdot \mathcal{N}(\mathbf{x}_i \mid \boldsymbol{\mu}_k, \boldsymbol{\Sigma}_k) \right)
$$

$\log$ 안의 합(sum-inside-log) 때문에 $\partial \mathcal{L} / \partial \boldsymbol{\mu}_k = 0$을 닫힌 형태로 풀 수 없다. 이것이 EM 알고리즘이 필요한 근본 이유다.

### 3.4 EM 알고리즘: 일반 프레임워크

EM은 **완전 데이터 로그-우도의 기댓값**을 반복적으로 최대화한다.

**완전 데이터 로그-우도** ($z_i$를 관측했다고 가정):

$$
\mathcal{L}_c(\Theta) = \sum_{i=1}^{N} \sum_{k=1}^{K} \mathbb{1}[z_i = k] \left( \log \pi_k + \log \mathcal{N}(\mathbf{x}_i \mid \boldsymbol{\mu}_k, \boldsymbol{\Sigma}_k) \right)
$$

**Q-함수** (E-step의 출력): 잠재 변수의 사후 분포 하에서의 기댓값:

$$
\mathcal{Q}(\Theta, \Theta^{(t)}) = \sum_{i=1}^{N} \sum_{k=1}^{K} \gamma_{ik}^{(t)} \left[ \log \pi_k + \log \mathcal{N}(\mathbf{x}_i \mid \boldsymbol{\mu}_k, \boldsymbol{\Sigma}_k) \right]
$$

여기서 $\gamma_{ik}^{(t)} = \mathbb{E}[\mathbb{1}[z_i = k] \mid \mathbf{x}_i, \Theta^{(t)}] = P(z_i = k \mid \mathbf{x}_i, \Theta^{(t)})$.

### 3.5 E-step: 책임도 계산

**베이즈 정리** 적용:

$$
\gamma_{ik}^{(t)} = \frac{\pi_k^{(t)} \cdot \mathcal{N}(\mathbf{x}_i \mid \boldsymbol{\mu}_k^{(t)}, \boldsymbol{\Sigma}_k^{(t)})}{\sum_{j=1}^{K} \pi_j^{(t)} \cdot \mathcal{N}(\mathbf{x}_i \mid \boldsymbol{\mu}_j^{(t)}, \boldsymbol{\Sigma}_j^{(t)})}
$$

**유도**: $P(z_i = k \mid \mathbf{x}_i, \Theta) = P(\mathbf{x}_i, z_i = k) / P(\mathbf{x}_i) = \pi_k \mathcal{N}_k / \sum_j \pi_j \mathcal{N}_j$

**수치 안정성**: 분자/분모 모두 매우 작은 값이 될 수 있다. 실무에서는 log-space에서 계산 후 log-sum-exp 트릭을 적용한다:

$$
\log s_k = \log \pi_k - \frac{d}{2} \log(2\pi) - \frac{1}{2} \log |\boldsymbol{\Sigma}_k| - \frac{1}{2} (\mathbf{x}_i - \boldsymbol{\mu}_k)^\top \boldsymbol{\Sigma}_k^{-1} (\mathbf{x}_i - \boldsymbol{\mu}_k)
$$

$$
\gamma_{ik} = \frac{\exp(\log s_k)}{\sum_j \exp(\log s_j)} = \text{softmax}_k(\log s_1, \ldots, \log s_K)
$$

> **구현 참고**: Armadillo `gmm_full::learn()`이 E-step을 내부적으로 수행한다. 책임도 $\gamma_{ik}$는 코드베이스에 직접 노출되지 않는다.

### 3.6 M-step: 파라미터 갱신 유도

Q-함수를 각 파라미터에 대해 최대화한다.

#### 3.6.1 유효 포인트 수 $N_k$

$$
N_k = \sum_{i=1}^{N} \gamma_{ik}
$$

**해석**: 컴포넌트 $k$에 "소프트하게 할당된" 포인트의 유효 개수.

#### 3.6.2 가중치 $\pi_k$ — 라그랑주 승수법

$\sum_k \pi_k = 1$ 제약 하에서 $\mathcal{Q}$를 $\pi_k$에 대해 최대화한다.

라그랑지안:

$$
\mathcal{L}_\pi = \sum_{i,k} \gamma_{ik} \log \pi_k + \lambda \left(1 - \sum_k \pi_k\right)
$$

$$
\frac{\partial \mathcal{L}_\pi}{\partial \pi_k} = \frac{N_k}{\pi_k} - \lambda = 0 \implies \pi_k = \frac{N_k}{\lambda}
$$

$\sum_k \pi_k = 1$에서 $\lambda = \sum_k N_k = N$. 따라서:

$$
\boxed{\pi_k^{(t+1)} = \frac{N_k}{N}}
$$

> **코드 참조**: `mixture_em_backend.cpp:103` — `comp.weight = model.hefts(k)` (Armadillo가 계산한 $\pi_k$)

#### 3.6.3 평균 $\boldsymbol{\mu}_k$ — 미분

$\mathcal{Q}$에서 $\boldsymbol{\mu}_k$에 의존하는 항만 추출:

$$
\mathcal{Q}_{\mu_k} = -\frac{1}{2} \sum_{i=1}^{N} \gamma_{ik} (\mathbf{x}_i - \boldsymbol{\mu}_k)^\top \boldsymbol{\Sigma}_k^{-1} (\mathbf{x}_i - \boldsymbol{\mu}_k)
$$

$$
\frac{\partial \mathcal{Q}_{\mu_k}}{\partial \boldsymbol{\mu}_k} = \sum_{i=1}^{N} \gamma_{ik} \boldsymbol{\Sigma}_k^{-1} (\mathbf{x}_i - \boldsymbol{\mu}_k) = 0
$$

$\boldsymbol{\Sigma}_k^{-1}$은 양의 정부호이므로 약분 가능:

$$
\sum_{i} \gamma_{ik} (\mathbf{x}_i - \boldsymbol{\mu}_k) = 0 \implies \boxed{\boldsymbol{\mu}_k^{(t+1)} = \frac{1}{N_k} \sum_{i=1}^{N} \gamma_{ik} \mathbf{x}_i}
$$

**기하학적 해석**: 가중 무게중심. 각 포인트가 $\gamma_{ik}$ 만큼의 "질량"을 기여하여 중심을 결정한다.

> **코드 참조**: `mixture_em_backend.cpp:87-88` — `comp.mean = (m(0), m(1), m(2), 0.0)`

#### 3.6.4 공분산 $\boldsymbol{\Sigma}_k$ — 미분

$\mathcal{Q}$에서 $\boldsymbol{\Sigma}_k$에 의존하는 항:

$$
\mathcal{Q}_{\Sigma_k} = -\frac{N_k}{2} \log |\boldsymbol{\Sigma}_k| - \frac{1}{2} \sum_{i} \gamma_{ik} (\mathbf{x}_i - \boldsymbol{\mu}_k)^\top \boldsymbol{\Sigma}_k^{-1} (\mathbf{x}_i - \boldsymbol{\mu}_k)
$$

$\boldsymbol{\Lambda}_k = \boldsymbol{\Sigma}_k^{-1}$로 치환하고 행렬 미분 공식 $\frac{\partial}{\partial \boldsymbol{\Lambda}} \log |\boldsymbol{\Lambda}| = \boldsymbol{\Lambda}^{-\top}$, $\frac{\partial}{\partial \boldsymbol{\Lambda}} \text{tr}(\boldsymbol{\Lambda} A) = A^\top$을 적용:

$$
\frac{\partial \mathcal{Q}}{\partial \boldsymbol{\Lambda}_k} = \frac{N_k}{2} \boldsymbol{\Sigma}_k - \frac{1}{2} \sum_{i} \gamma_{ik} (\mathbf{x}_i - \boldsymbol{\mu}_k)(\mathbf{x}_i - \boldsymbol{\mu}_k)^\top = 0
$$

$$
\boxed{\boldsymbol{\Sigma}_k^{(t+1)} = \frac{1}{N_k} \sum_{i=1}^{N} \gamma_{ik} (\mathbf{x}_i - \boldsymbol{\mu}_k^{(t+1)})(\mathbf{x}_i - \boldsymbol{\mu}_k^{(t+1)})^\top}
$$

**해석**: 가중 산포 행렬 (weighted scatter matrix). 분모가 $N_k$ (not $N_k - 1$)이므로 **MLE 추정량**(편향)이다. Bessel 보정 ($N_k - 1$)은 적용하지 않는다.

> **코드 참조**: `mixture_em_backend.cpp:92-97` — 3×3 블록을 `model.fcovs.slice(k)`에서 복사
> 
> 폴백 경로(`single_component_fallback`)도 동일하게 $N$으로 나눈다: `mixture_em_backend.cpp:37` — `cov /= N`

### 3.7 수렴 보장: Jensen 부등식

**정리**: EM의 각 반복은 로그-우도를 단조 증가시킨다: $\mathcal{L}(\Theta^{(t+1)}) \ge \mathcal{L}(\Theta^{(t)})$.

**증명 스케치**:

임의의 분포 $q(z_i)$에 대해 Jensen 부등식으로:

$$
\mathcal{L}(\Theta) = \sum_i \log \sum_k q_k \frac{\pi_k \mathcal{N}_k}{q_k} \ge \sum_i \sum_k q_k \log \frac{\pi_k \mathcal{N}_k}{q_k} \equiv \text{ELBO}(q, \Theta)
$$

E-step에서 $q_k = \gamma_{ik}^{(t)}$로 설정하면 등호가 성립한다 (KL divergence = 0):

$$
\mathcal{L}(\Theta^{(t)}) = \text{ELBO}(\gamma^{(t)}, \Theta^{(t)})
$$

M-step에서 $\Theta^{(t+1)} = \arg\max_\Theta \text{ELBO}(\gamma^{(t)}, \Theta)$이므로:

$$
\mathcal{L}(\Theta^{(t+1)}) \ge \text{ELBO}(\gamma^{(t)}, \Theta^{(t+1)}) \ge \text{ELBO}(\gamma^{(t)}, \Theta^{(t)}) = \mathcal{L}(\Theta^{(t)}) \quad \square
$$

**한계**: EM은 **지역 최대치**(local maximum)로 수렴한다. 전역 최적해 보장이 없으므로 초기화 전략이 중요하다 (§5 참조).

### 3.8 수렴 판정

> **코드 참조**: `mixture_em_backend.cpp:186-187` — `params.convergence_tol`을 Armadillo에 전달

Armadillo의 내부 수렴 기준:

$$
|\mathcal{L}^{(t+1)} - \mathcal{L}^{(t)}| < \tau \quad \text{또는} \quad t = T_{\max}
$$

기본값: $\tau = 10^{-4}$, $T_{\max} = 20$.

### 3.9 식별 불가능성 (Identifiability)

가우시안 혼합 모델은 컴포넌트 레이블 순열(label permutation)에 대해 식별 불가능하다: $(\pi_1, \boldsymbol{\mu}_1, \boldsymbol{\Sigma}_1)$과 $(\pi_2, \boldsymbol{\mu}_2, \boldsymbol{\Sigma}_2)$를 교환해도 동일한 밀도를 생성한다.

이 점은 정합(registration) 응용에서는 문제가 되지 않는다: 비용 함수(§8)는 $\arg\min_k$로 최적 컴포넌트를 선택하며, 레이블 순서에 무관하다.

---

## 4. 공분산 정칙화 (Regularization)

본 구현은 **두 단계의 독립적 정칙화**를 적용한다. 각각의 수학적 근거와 역할이 다르다.

### 4.1 1단계: 가산적 정칙화 ($\varepsilon \cdot I$)

> **코드 참조**: `mixture_em_backend.cpp:100` — 사후 정칙화, `mixture_em_backend.cpp:41` — 폴백 정칙화

EM 완료 후 모든 컴포넌트에 무조건 적용:

$$
\boldsymbol{\Sigma}_k \leftarrow \boldsymbol{\Sigma}_k + \varepsilon \cdot I_3 \quad (\varepsilon = 10^{-3})
$$

**수학적 효과**: $\boldsymbol{\Sigma}_k$의 모든 고유값이 최소 $\varepsilon$만큼 증가한다. 원래 고유값이 $\lambda_1 \ge \lambda_2 \ge \lambda_3 \ge 0$이면:

$$
\lambda_i' = \lambda_i + \varepsilon > 0 \quad \forall i
$$

따라서 $\boldsymbol{\Sigma}_k + \varepsilon I$는 **양의 정부호**(positive definite)가 보장된다.

**Tikhonov 정칙화와의 연결**: 이 연산은 MAP 추정에서 공분산에 대한 등방 역-Wishart 사전분포 $\boldsymbol{\Sigma}_k \sim \mathcal{W}^{-1}(\nu_0, \varepsilon \cdot I)$를 부과하는 것과 유사하다. 사전분포의 효과가 $\varepsilon I$ 형태로 사후 공분산에 더해진다.

### 4.2 2단계: 고유값 클램핑 (역공분산 계산 시)

> **코드 참조**: `include/ndt/integrated_ndt_factor.hpp:45-55` — `compute_ndt_inverse_covariance()`

NDT Factor의 역공분산 계산 시 추가 정칙화를 적용한다:

$$
\boldsymbol{\Sigma}_k = V \cdot \text{diag}(\lambda_1, \lambda_2, \lambda_3, \lambda_4) \cdot V^\top \quad \text{(고유분해)}
$$

$$
\tilde{\lambda}_i = \max(\lambda_i, \varepsilon_\lambda \cdot \lambda_{\max}) \quad \text{(클램핑)}
$$

$$
\boldsymbol{\Sigma}_k^{-1} = V \cdot \text{diag}(1/\tilde{\lambda}_1, \ldots, 1/\tilde{\lambda}_4) \cdot V^\top
$$

**조건수 분석**: 클램핑 전 조건수 $\kappa = \lambda_{\max} / \lambda_{\min}$이 임의로 커질 수 있다. 클램핑 후:

$$
\kappa(\tilde{\boldsymbol{\Sigma}}) = \frac{\lambda_{\max}}{\max(\lambda_{\min}, \varepsilon_\lambda \cdot \lambda_{\max})} \le \frac{1}{\varepsilon_\lambda}
$$

$\varepsilon_\lambda = 10^{-3}$이면 $\kappa \le 1000$으로 제한된다.

**기하학적 해석**: 공분산의 고유벡터는 확률 밀도의 등고면(타원체)의 주축 방향이다. 고유값은 각 축 방향의 분산이다. 클램핑은 타원체의 **종횡비(aspect ratio)**를 $1/\varepsilon_\lambda$ 이하로 제한하여, "바늘"처럼 퇴화된 분포를 방지한다.

### 4.3 두 단계의 차이점

| 구분 | 1단계 ($\varepsilon I$ 가산) | 2단계 (고유값 클램핑) |
|------|:---:|:---:|
| 적용 시점 | EM 직후 | 역공분산 계산 시 |
| 적용 위치 | `mixture_em_backend.cpp:100` | `integrated_ndt_factor.hpp:51` |
| 수학적 효과 | 고유값 $+\varepsilon$ (등방 이동) | 고유값 $\ge \varepsilon_\lambda \cdot \lambda_{\max}$ (비례 바운드) |
| 목적 | PD 보장 | 역행렬 안정성, 조건수 제한 |
| 타원체 효과 | 최소 반지름 $\sqrt{\varepsilon}$ | 최대 종횡비 $1/\sqrt{\varepsilon_\lambda}$ |

두 정칙화는 **독립적이고 누적적**이다: 1단계가 먼저 적용된 공분산에 2단계가 추가로 적용된다.

### 4.4 4D 공간에서의 영 고유값

4×4 공분산 행렬은 $w$-차원에서 분산 0이므로 4번째 고유값 $\lambda_4 = 0$이다. `compute_ndt_inverse_covariance`는 $1/\tilde{\lambda}_4$를 계산하지만, $\tilde{\lambda}_4 = \max(0, \varepsilon_\lambda \cdot \lambda_{\max}) = \varepsilon_\lambda \cdot \lambda_{\max} > 0$이므로 0으로 나누는 문제는 발생하지 않는다. 다만 역공분산의 $w$-행/열은 매우 큰 값($\sim 1/(\varepsilon_\lambda \cdot \lambda_{\max})$)이 되지만, 비용 함수에서 잔차의 $w$ 성분이 항상 0이므로 (`residual(3) = 0`) 이 값은 비용에 기여하지 않는다.

---

## 5. 초기화 전략

### 5.1 Cold-Start: Random Spread

> **코드 참조**: `mixture_em_backend.cpp:180-188`

초기 컴포넌트가 없을 때 Armadillo의 `random_spread` 방식으로 $K$개 초기 평균을 선택한다.

**알고리즘** (k-means++ 변형):
1. 데이터에서 무작위로 첫 번째 평균 $\boldsymbol{\mu}_1$ 선택
2. $k = 2, \ldots, K$에 대해: 기존 평균들로부터의 최소 유클리드 거리가 최대인 점을 선택

$$
\boldsymbol{\mu}_k = \arg\max_{\mathbf{x} \in \mathcal{X}} \min_{j < k} \|\mathbf{x} - \boldsymbol{\mu}_j\|
$$

이후 20회 k-means 반복으로 초기 클러스터링을 개선한 뒤 EM을 시작한다.

**이론적 근거**: 데이터 공간을 고르게 커버하는 초기 평균은 EM이 좋은 지역 최적해에 수렴할 확률을 높인다. k-means++의 $O(\log K)$ 근사 보장(Arthur & Vassilvitskii, 2007)과 유사한 분산 특성을 가진다.

### 5.2 Warm-Start: 이전 파라미터 재활용

> **코드 참조**: `mixture_em_backend.cpp:225-256`

이전 `finalize()` 결과를 시드로 사용할 때, **역정칙화**가 필요하다.

**문제**: 저장된 $\boldsymbol{\Sigma}_k$에는 이미 1단계 정칙화 $\varepsilon I$가 포함되어 있다. `extract_result()`가 EM 후 다시 $\varepsilon I$를 추가하므로, 역정칙화 없이 Armadillo에 전달하면 **이중 적용**이 된다:

$$
\boldsymbol{\Sigma}_k^{\text{이중}} = \boldsymbol{\Sigma}_k^{\text{순수}} + 2\varepsilon I \ne \boldsymbol{\Sigma}_k^{\text{순수}} + \varepsilon I
$$

**해결**: Armadillo에 전달하기 전에 대각 원소에서 $\varepsilon$을 뺀다:

$$
\tilde{\Sigma}_{k,rr} = \max(\Sigma_{k,rr} - \varepsilon, 10^{-6}) \quad (r = 0,1,2)
$$

$$
\tilde{\Sigma}_{k,rc} = \Sigma_{k,rc} \quad (r \ne c)
$$

> **코드**: `mixture_em_backend.cpp:236-238` — `val -= params.covariance_regularization`, `val = std::max(val, 1e-6)`

**대수적 불변량**: $\varepsilon$-제거 → EM → `extract_result`의 $\varepsilon$-추가 순서로, 저장된 공분산은 항상 $\boldsymbol{\Sigma}_k^{\text{EM}} + \varepsilon I$ 형태를 유지한다.

### 5.3 가중치 정규화

> **코드 참조**: `mixture_em_backend.cpp:248-252`

Armadillo `set_params()`는 $\sum_k \pi_k = 1$을 요구한다. 이전 가중치 합이 정확히 1이 아닐 수 있으므로:

$$
\hat{\pi}_k = \begin{cases}
\pi_k / \sum_j \pi_j & \text{if } \sum_j \pi_j > 0 \\
1/K & \text{if } \sum_j \pi_j = 0 \text{ (모두 0인 경우 균등 분배)}
\end{cases}
$$

### 5.4 폴백 체인

```
warm-start 요청
    ├── N < 2 또는 initial_components 비어있음 → cold-start (§5.1)
    ├── K > max_components (설정 변경) → cold-start (§5.1)
    ├── warm-start EM 실패 → cold-start (§5.1)
    └── cold-start EM 실패 → single_component_fallback (§5.5)
```

### 5.5 단일 컴포넌트 폴백

> **코드 참조**: `mixture_em_backend.cpp:17-56`

$N < 2$ 또는 EM이 실패할 때:

$$
\boldsymbol{\mu} = \frac{1}{N} \sum_{i=1}^{N} \mathbf{p}_i \quad \text{(표본 평균)}
$$

$$
\boldsymbol{\Sigma} = \frac{1}{N} \sum_{i=1}^{N} (\mathbf{p}_i - \boldsymbol{\mu})(\mathbf{p}_i - \boldsymbol{\mu})^\top + \varepsilon I_3 \quad \text{(정칙화된 MLE 공분산)}
$$

$$
\pi = 1.0
$$

---

## 6. 후처리: Pruning과 재정규화

### 6.1 저가중치 Pruning

> **코드 참조**: `mixture_em_backend.cpp:120-123`

EM이 수렴한 후 의미 없는 컴포넌트를 제거한다:

$$
\text{제거 조건: } \pi_k < \pi_{\min} \quad (\pi_{\min} = 0.01)
$$

**수학적 의미**: $\pi_k < 0.01$이면 전체 밀도에 대한 기여도가 1% 미만이다. 이런 컴포넌트는:
- 비용 함수 평가 시 $\pi_k$로 가중되므로 실질적 기여 미미
- 소수 포인트에 과적합(overfit)된 "스퓨리어스" 컴포넌트일 가능성이 높음
- 역공분산 계산 비용만 증가

### 6.2 가중 평균 폴백

> **코드 참조**: `mixture_em_backend.cpp:108-116`, `126-134`

모든 컴포넌트가 pruning될 경우를 대비해 **사전에** 가중 평균을 계산한다:

$$
\bar{\boldsymbol{\mu}} = \frac{\sum_{k=1}^{K} \pi_k \boldsymbol{\mu}_k}{\sum_{k=1}^{K} \pi_k}
$$

이 평균은 혼합 분포의 **1차 모멘트**(기댓값)와 일치한다:

$$
\mathbb{E}[\mathbf{x}] = \sum_k \pi_k \boldsymbol{\mu}_k = \bar{\boldsymbol{\mu}} \cdot \sum_k \pi_k
$$

모두 제거되면: $\boldsymbol{\mu} = \bar{\boldsymbol{\mu}}$, $\boldsymbol{\Sigma} = \varepsilon I_3$, $\pi = 1.0$.

### 6.3 가중치 재정규화

> **코드 참조**: `mixture_em_backend.cpp:141-144`

Pruning 후 남은 컴포넌트의 가중치 합이 1이 아니므로:

$$
\pi_k' = \frac{\pi_k}{\sum_{j \in \text{survivors}} \pi_j}
$$

**불변량 복원**: $\sum_k \pi_k' = 1$.

**비용 함수에 대한 영향**: §8의 비용 함수에서 $\pi_k$는 각 포인트 비용에 대한 가중치이다. 재정규화는 비용의 전체 스케일을 유지하여 GTSAM의 LM 옵티마이저가 적절한 step size를 선택하도록 한다.

---

## 7. 마할라노비스 거리와 Winner-Take-All 대응

### 7.1 마할라노비스 거리

> **코드 참조**: `integrated_mixture_light_ndt_factor_impl.hpp:220-224`

변환된 소스 포인트 $\mathbf{q}_i = T \cdot \mathbf{p}_i$와 컴포넌트 $k$의 평균 $\boldsymbol{\mu}_k$ 사이의 마할라노비스 제곱 거리:

$$
d_M^2(\mathbf{q}_i, k) = (\mathbf{q}_i - \boldsymbol{\mu}_k)^\top \boldsymbol{\Sigma}_k^{-1} (\mathbf{q}_i - \boldsymbol{\mu}_k)
$$

**기하학적 해석**: 마할라노비스 거리는 **백색화(whitened) 공간**에서의 유클리드 거리이다. $\boldsymbol{\Sigma}_k = V \Lambda V^\top$일 때:

$$
d_M^2 = \sum_{j=1}^{3} \frac{(\mathbf{v}_j^\top (\mathbf{q}_i - \boldsymbol{\mu}_k))^2}{\lambda_j}
$$

즉, 각 주축(고유벡터 $\mathbf{v}_j$) 방향의 편차를 해당 분산($\lambda_j$)으로 나눈 값의 합이다. 분산이 작은 방향의 편차가 더 크게 페널티를 받는다.

**등마할라노비스 곡면**: $d_M^2 = c^2$인 점들의 집합은 $\boldsymbol{\mu}_k$를 중심으로 한 **타원체**이며, 반축 길이는 $c\sqrt{\lambda_j}$이다.

### 7.2 Winner-Take-All (WTA) 대응 선택

> **코드 참조**: `integrated_mixture_light_ndt_factor_impl.hpp:226-233`

각 소스 포인트 $i$에 대해 이웃 복셀의 모든 컴포넌트 중 마할라노비스 거리가 최소인 하나를 선택한다:

$$
k^*(i) = \arg\min_{k, v \in \mathcal{N}(\mathbf{q}_i)} d_M^2(\mathbf{q}_i, k)
$$

여기서 $\mathcal{N}(\mathbf{q}_i)$는 $\mathbf{q}_i$의 이웃 복셀 집합 (DIRECT7 모드: 현재 + 6면 이웃).

### 7.3 소프트 할당과의 비교

WTA는 E-step의 책임도 $\gamma_{ik}$를 **경판정**(hard decision)으로 대체한다:

$$
\text{소프트}: \quad \gamma_{ik} = \frac{\pi_k \mathcal{N}_k}{\sum_j \pi_j \mathcal{N}_j} \in (0, 1)
$$

$$
\text{WTA}: \quad \gamma_{ik} = \begin{cases} 1 & k = k^*(i) \\ 0 & \text{otherwise} \end{cases}
$$

### 7.4 영온도 극한으로서의 WTA

소프트 할당에 온도 파라미터 $\beta > 0$를 도입하면:

$$
\gamma_{ik}^{(\beta)} = \frac{\exp(-\beta \cdot d_M^2(\mathbf{q}_i, k))}{\sum_j \exp(-\beta \cdot d_M^2(\mathbf{q}_i, j))}
$$

$\beta \to \infty$ (영온도 극한)에서:

$$
\lim_{\beta \to \infty} \gamma_{ik}^{(\beta)} = \begin{cases} 1 & k = \arg\min_j d_M^2(\mathbf{q}_i, j) \\ 0 & \text{otherwise} \end{cases}
$$

따라서 WTA는 소프트 할당의 **영온도 극한**이다. 이 선택의 트레이드오프:

| | 소프트 할당 (log-sum-exp) | WTA (argmin) |
|---|---|---|
| 미분 가능성 | $C^\infty$ (everywhere smooth) | 불연속 (경계에서) |
| 계산 비용 | $O(K)$ 지수함수 평가 | $O(K)$ 비교만 |
| 구현 복잡도 | Jacobian에 softmax 미분 포함 | 표준 NDT Jacobian 재사용 가능 |
| 로버스트성 | 모든 컴포넌트 기여 → 노이즈에 둔감 | 최적 1개만 → 간결하지만 경계에서 불연속 |

본 구현은 WTA를 선택했다: GTSAM의 Levenberg-Marquardt 옵티마이저가 매 반복마다 대응을 재계산(re-linearize)하므로 경계 불연속이 실질적으로 평활화된다.

---

## 8. π_k 가중 비용 함수

### 8.1 비용 함수 정의

> **코드 참조**: `integrated_mixture_light_ndt_factor_impl.hpp:310-316`

잔차 벡터:

$$
\mathbf{r}_i = \boldsymbol{\mu}_{k^*(i)} - T \cdot \mathbf{p}_i
$$

> 코드: `residual = mean_B - transed_mean_A`, `residual(3) = 0`

포인트 비용:

$$
C_i = \pi_{k^*(i)} \cdot \mathbf{r}_i^\top \boldsymbol{\Sigma}_{k^*(i)}^{-1} \mathbf{r}_i
$$

전체 비용:

$$
\boxed{C(T) = \sum_{i=1}^{M} \pi_{k^*(i)} \cdot (\boldsymbol{\mu}_{k^*(i)} - T \mathbf{p}_i)^\top \boldsymbol{\Sigma}_{k^*(i)}^{-1} (\boldsymbol{\mu}_{k^*(i)} - T \mathbf{p}_i)}
$$

여기서 $M$은 유효 대응이 있는 포인트 수.

### 8.2 음의 로그-우도 근사로서의 해석

전체 혼합 밀도의 음의 로그-우도:

$$
-\log p(\mathbf{q}_i \mid \Theta) = -\log \sum_k \pi_k \mathcal{N}(\mathbf{q}_i \mid \boldsymbol{\mu}_k, \boldsymbol{\Sigma}_k)
$$

WTA 근사(§7.4)를 적용하면:

$$
-\log p(\mathbf{q}_i \mid \Theta) \approx -\log \left[ \pi_{k^*} \mathcal{N}(\mathbf{q}_i \mid \boldsymbol{\mu}_{k^*}, \boldsymbol{\Sigma}_{k^*}) \right]
$$

정규화 상수를 무시하고 핵심 항만 추출:

$$
= -\log \pi_{k^*} + \frac{1}{2} \mathbf{r}_i^\top \boldsymbol{\Sigma}_{k^*}^{-1} \mathbf{r}_i + \frac{1}{2} \log |\boldsymbol{\Sigma}_{k^*}| + \frac{d}{2} \log(2\pi)
$$

$\log |\boldsymbol{\Sigma}_{k^*}|$와 $\frac{d}{2}\log(2\pi)$는 포즈 $T$에 무관한 상수이므로 최적화에서 무시할 수 있다. 나머지:

$$
\propto -\log \pi_{k^*} + \frac{1}{2} \mathbf{r}_i^\top \boldsymbol{\Sigma}_{k^*}^{-1} \mathbf{r}_i
$$

본 구현의 비용 $C_i = \pi_{k^*} \cdot \mathbf{r}_i^\top \boldsymbol{\Sigma}_{k^*}^{-1} \mathbf{r}_i$는 이 근사에서 $-\log \pi_{k^*}$ 대신 $\pi_{k^*}$를 곱셈 가중치로 사용한 변형이다. 이는 정확한 음의 로그-우도는 아니지만, 다음 직관적 성질을 가진다:

- $\pi_{k^*} \approx 1$ (강한 컴포넌트): 비용 기여가 전액 반영
- $\pi_{k^*} \approx 0$ (약한 컴포넌트): 비용 기여가 거의 무시됨
- Hessian의 양의 반정부호성이 자동으로 보장됨 (§10 참조)

### 8.3 π_k와 로버스트 비용 함수

$\pi_k$ 가중은 M-추정량(M-estimator)의 가중 함수와 유사한 역할을 한다. Huber 손실에서의 가중:

$$
\text{Huber}: w(r) = \begin{cases} 1 & |r| \le \delta \\ \delta/|r| & |r| > \delta \end{cases}
$$

$\pi_k$ 가중:

$$
w_k = \pi_k \in [0, 1]
$$

차이점은 $\pi_k$가 잔차의 크기가 아니라 **컴포넌트의 사전 중요도**에 기반한다는 것이다. 하지만 효과는 유사하다: EM이 소수 포인트만 포함하는 스퓨리어스 컴포넌트에 낮은 $\pi_k$를 부여하면, 해당 대응의 비용 기여가 자동으로 감소한다.

---

## 9. SE(3) Jacobian 유도

### 9.1 SE(3) 포즈 매개변수화

강체 변환 $T \in SE(3)$:

$$
T = \begin{pmatrix} R & \mathbf{t} \\ \mathbf{0}^\top & 1 \end{pmatrix}, \quad R \in SO(3), \; \mathbf{t} \in \mathbb{R}^3
$$

리 대수 $\mathfrak{se}(3)$의 원소 $\boldsymbol{\xi} = (\boldsymbol{\phi}, \boldsymbol{\rho}) \in \mathbb{R}^6$에 대한 **좌측 섭동** 모델:

$$
T' = \exp(\hat{\boldsymbol{\xi}}) \cdot T
$$

여기서 $\hat{\boldsymbol{\xi}} = \begin{pmatrix} [\boldsymbol{\phi}]_\times & \boldsymbol{\rho} \\ \mathbf{0}^\top & 0 \end{pmatrix} \in \mathfrak{se}(3)$.

### 9.2 잔차의 포즈 의존성

잔차:

$$
\mathbf{r}(T_{\text{target}}, T_{\text{source}}) = \boldsymbol{\mu}_{k^*} - \underbrace{T_{\text{target}}^{-1} \cdot T_{\text{source}}}_{= \delta} \cdot \mathbf{p}_i
$$

GTSAM 컨벤션에서 `delta = T_target^{-1} * T_source`이며, 잔차는 타겟 프레임에서 정의된다.

### 9.3 Target Jacobian 유도

> **코드 참조**: `integrated_mixture_light_ndt_factor_impl.hpp:331-333`

$T_{\text{target}}$에 좌측 섭동 $\exp(\hat{\boldsymbol{\xi}}_t)$를 적용하면:

$$
\delta' = (\exp(\hat{\boldsymbol{\xi}}_t) \cdot T_{\text{target}})^{-1} \cdot T_{\text{source}} = T_{\text{target}}^{-1} \exp(-\hat{\boldsymbol{\xi}}_t) \cdot T_{\text{source}}
$$

변환된 소스 포인트:

$$
\mathbf{q}_i' = \delta' \cdot \mathbf{p}_i = T_{\text{target}}^{-1} \exp(-\hat{\boldsymbol{\xi}}_t) T_{\text{source}} \mathbf{p}_i
$$

$\boldsymbol{\xi}_t = 0$ 근방에서 1차 근사 $\exp(-\hat{\boldsymbol{\xi}}_t) \approx I - \hat{\boldsymbol{\xi}}_t$:

$$
\mathbf{q}_i' \approx \delta \cdot \mathbf{p}_i - \hat{\boldsymbol{\xi}}_t \cdot \delta \cdot \mathbf{p}_i = \mathbf{q}_i - \hat{\boldsymbol{\xi}}_t \cdot \mathbf{q}_i
$$

잔차의 변화:

$$
\mathbf{r}' = \boldsymbol{\mu}_{k^*} - \mathbf{q}_i' \approx \mathbf{r} + \hat{\boldsymbol{\xi}}_t \cdot \mathbf{q}_i
$$

$\hat{\boldsymbol{\xi}}_t \cdot \mathbf{q}_i$의 3D 부분 (동차 좌표의 상위 3성분):

$$
\hat{\boldsymbol{\xi}}_t \cdot \mathbf{q}_i = \begin{pmatrix} [\boldsymbol{\phi}]_\times & \boldsymbol{\rho} \\ \mathbf{0}^\top & 0 \end{pmatrix} \begin{pmatrix} \mathbf{q}_i^{(3)} \\ 1 \end{pmatrix} = \begin{pmatrix} [\boldsymbol{\phi}]_\times \mathbf{q}_i^{(3)} + \boldsymbol{\rho} \\ 0 \end{pmatrix}
$$

벡터 곱의 반교환 성질 $[\boldsymbol{\phi}]_\times \mathbf{q} = -[\mathbf{q}]_\times \boldsymbol{\phi}$를 사용:

$$
\frac{\partial \mathbf{r}}{\partial \boldsymbol{\xi}_t}\bigg|_{\boldsymbol{\xi}_t=0} = \begin{pmatrix} -[\mathbf{q}_i^{(3)}]_\times & I_3 \end{pmatrix} \in \mathbb{R}^{3 \times 6}
$$

4D 동차 좌표로 확장하면 (4행은 0):

$$
\boxed{J_{\text{target}} = \begin{pmatrix} -[T \mathbf{p}_i]_\times & I_3 \\ \mathbf{0}^\top & \mathbf{0}^\top \end{pmatrix} \in \mathbb{R}^{4 \times 6}}
$$

> **코드**: `J_target.block<3,3>(0,0) = -gtsam::SO3::Hat(transed_mean_A.head<3>())`, `J_target.block<3,3>(0,3) = I₃`

### 9.4 Source Jacobian 유도

> **코드 참조**: `integrated_mixture_light_ndt_factor_impl.hpp:338-340`

$T_{\text{source}}$에 좌측 섭동 $\exp(\hat{\boldsymbol{\xi}}_s)$를 적용하면:

$$
\delta' = T_{\text{target}}^{-1} \cdot \exp(\hat{\boldsymbol{\xi}}_s) \cdot T_{\text{source}}
$$

$$
\mathbf{q}_i' = T_{\text{target}}^{-1} \exp(\hat{\boldsymbol{\xi}}_s) T_{\text{source}} \mathbf{p}_i \approx \delta \cdot \mathbf{p}_i + T_{\text{target}}^{-1} \hat{\boldsymbol{\xi}}_s T_{\text{source}} \mathbf{p}_i
$$

$\hat{\boldsymbol{\xi}}_s$가 $T_{\text{source}}$ 앞에 적용되므로, $T_{\text{target}}^{-1}$로 변환하면:

$$
\delta' \cdot \mathbf{p}_i \approx \delta \mathbf{p}_i + R_\delta \hat{\boldsymbol{\xi}}_s \mathbf{p}_i^{(s)}
$$

여기서 $R_\delta = \delta_{\text{linear}} = (T_{\text{target}}^{-1} T_{\text{source}})_R$이고 $\mathbf{p}_i^{(s)}$는 소스 프레임에서의 포인트이다.

$\hat{\boldsymbol{\xi}}_s \cdot (\mathbf{p}_i, 1)^\top$의 3D 부분: $[\boldsymbol{\phi}_s]_\times \mathbf{p}_i + \boldsymbol{\rho}_s$

잔차 변화:

$$
\mathbf{r}' - \mathbf{r} = -R_\delta ([\boldsymbol{\phi}_s]_\times \mathbf{p}_i + \boldsymbol{\rho}_s) = R_\delta [\mathbf{p}_i]_\times \boldsymbol{\phi}_s - R_\delta \boldsymbol{\rho}_s
$$

따라서:

$$
\boxed{J_{\text{source}} = \begin{pmatrix} R \cdot [\mathbf{p}_i]_\times & -R \\ \mathbf{0}^\top & \mathbf{0}^\top \end{pmatrix} \in \mathbb{R}^{4 \times 6}}
$$

> **코드**: `J_source.block<3,3>(0,0) = delta.linear() * gtsam::SO3::Hat(mean_A.head<3>())`, `J_source.block<3,3>(0,3) = -delta.linear()`

### 9.5 부호 규약 직관

| Jacobian | 회전 블록 | 병진 블록 | 직관 |
|----------|----------|----------|------|
| $J_{\text{target}}$ | $-[T\mathbf{p}]_\times$ | $+I_3$ | 타겟 회전 → 변환점이 회전 → 잔차 반대 방향. 타겟 병진 → 잔차 직접 증가 |
| $J_{\text{source}}$ | $+R[\mathbf{p}]_\times$ | $-R$ | 소스 회전 → 소스 점 이동 → 잔차에 $R$ 전파. 소스 병진 → 잔차 감소 (target - source) |

---

## 10. Gauss-Newton Hessian과 최적화

### 10.1 전체 Hessian vs. Gauss-Newton 근사

비용 함수 $C(T) = \sum_i C_i(T)$의 정확한 Hessian:

$$
\frac{\partial^2 C_i}{\partial \boldsymbol{\xi}^2} = J_i^\top \boldsymbol{\Sigma}_{k^*}^{-1} J_i + \sum_m r_{i,m} \frac{\partial^2 r_{i,m}}{\partial \boldsymbol{\xi}^2}
$$

Gauss-Newton(GN) 근사는 **2차 미분 항을 무시**한다:

$$
\frac{\partial^2 C_i}{\partial \boldsymbol{\xi}^2} \approx J_i^\top \boldsymbol{\Sigma}_{k^*}^{-1} J_i
$$

**타당성**: 잔차가 작을 때 ($\mathbf{r}_i \approx 0$, 즉 정합이 잘 될 때) 2차 항의 기여가 미미하다. LiDAR 정합에서 반복적 재선형화(re-linearize)를 통해 잔차가 점차 감소하므로 GN 근사는 적절하다.

### 10.2 π_k 가중 Hessian 누적

> **코드 참조**: `integrated_mixture_light_ndt_factor_impl.hpp:343-354`

가중된 Jacobian:

$$
\tilde{J}_{\text{target}} = \pi_{k^*} J_{\text{target}}^\top \boldsymbol{\Sigma}_{k^*}^{-1} \quad \in \mathbb{R}^{6 \times 4}
$$

$$
\tilde{J}_{\text{source}} = \pi_{k^*} J_{\text{source}}^\top \boldsymbol{\Sigma}_{k^*}^{-1} \quad \in \mathbb{R}^{6 \times 4}
$$

Hessian 블록:

$$
H_{\text{target}} \mathrel{+}= \tilde{J}_{\text{target}} J_{\text{target}} = \pi_{k^*} J_{\text{target}}^\top \boldsymbol{\Sigma}_{k^*}^{-1} J_{\text{target}} \quad \in \mathbb{R}^{6 \times 6}
$$

$$
H_{\text{source}} \mathrel{+}= \tilde{J}_{\text{source}} J_{\text{source}} \quad \in \mathbb{R}^{6 \times 6}
$$

$$
H_{\text{target,source}} \mathrel{+}= \tilde{J}_{\text{target}} J_{\text{source}} \quad \in \mathbb{R}^{6 \times 6}
$$

Gradient 벡터:

$$
\mathbf{b}_{\text{target}} \mathrel{+}= \tilde{J}_{\text{target}} \mathbf{r}_i \quad \in \mathbb{R}^{6}
$$

$$
\mathbf{b}_{\text{source}} \mathrel{+}= \tilde{J}_{\text{source}} \mathbf{r}_i \quad \in \mathbb{R}^{6}
$$

### 10.3 양의 반정부호성 (PSD) 증명

**정리**: $H_{\text{target}} = \sum_i \pi_{k^*(i)} J_i^\top \boldsymbol{\Sigma}_{k^*(i)}^{-1} J_i \succeq 0$.

**증명**:

임의의 $\mathbf{v} \in \mathbb{R}^6$에 대해:

$$
\mathbf{v}^\top H_{\text{target}} \mathbf{v} = \sum_i \pi_{k^*} \mathbf{v}^\top J_i^\top \boldsymbol{\Sigma}_{k^*}^{-1} J_i \mathbf{v}
= \sum_i \pi_{k^*} (J_i \mathbf{v})^\top \boldsymbol{\Sigma}_{k^*}^{-1} (J_i \mathbf{v})
$$

- $\boldsymbol{\Sigma}_{k^*}^{-1} \succeq 0$ (§4의 정칙화에 의해 PD)
- $\pi_{k^*} \ge 0$
- 이차형식 $\mathbf{u}^\top A \mathbf{u} \ge 0$ for $A \succeq 0$

따라서 각 항 $\ge 0$이고, 합도 $\ge 0$. $\quad \square$

**LM에서의 의미**: $H + \lambda I$는 $\lambda > 0$이면 항상 양의 정부호이므로, GN 근사된 Hessian이 PSD인 것은 LM 알고리즘의 안정성을 보장한다.

### 10.4 전체 선형 시스템

GTSAM의 `HessianFactor`에 전달되는 $12 \times 12$ 블록 시스템:

$$
\begin{pmatrix} H_{\text{target}} & H_{\text{target,source}} \\ H_{\text{target,source}}^\top & H_{\text{source}} \end{pmatrix} \begin{pmatrix} \Delta\boldsymbol{\xi}_{\text{target}} \\ \Delta\boldsymbol{\xi}_{\text{source}} \end{pmatrix} = \begin{pmatrix} \mathbf{b}_{\text{target}} \\ \mathbf{b}_{\text{source}} \end{pmatrix}
$$

교차 블록 $H_{\text{target,source}}$는 두 포즈 업데이트 간의 상관관계를 인코딩한다.

### 10.5 병렬 리덕션

> **코드 참조**: `scan_matching_reduction.hpp` (OMP 및 TBB 버전)

각 스레드가 로컬 $(H^{(t)}, \mathbf{b}^{(t)}, C^{(t)})$를 누적한 후 합산:

$$
H = \sum_{t} H^{(t)}, \quad \mathbf{b} = \sum_{t} \mathbf{b}^{(t)}, \quad C = \sum_{t} C^{(t)}
$$

행렬 덧셈의 교환/결합 법칙에 의해 리덕션 순서와 무관하게 결과가 동일하다.

---

## 11. 수식-코드 매핑 종합 테이블

| # | 수식 | 코드 위치 | 비고 |
|---|------|-----------|------|
| 1 | $j \sim \text{Unif}\{0,\ldots,N{-}1\}$; $j < C$이면 교체 | `gmm_voxelmap_cpu.cpp:46-49` | Algorithm R |
| 2 | $\boldsymbol{\mu} = \frac{1}{N}\sum \mathbf{p}_i$ | `mixture_em_backend.cpp:23-27` | 폴백 평균 |
| 3 | $\boldsymbol{\Sigma} = \frac{1}{N}\sum(\mathbf{p}_i - \boldsymbol{\mu})(\mathbf{p}_i - \boldsymbol{\mu})^\top$ | `mixture_em_backend.cpp:31-37` | 편향 MLE |
| 4 | $\boldsymbol{\Sigma} \mathrel{+}= \varepsilon I_3$ | `mixture_em_backend.cpp:41, 100` | 가산 정칙화 |
| 5 | $\gamma_{ik} = \pi_k \mathcal{N}_k / \sum_j \pi_j \mathcal{N}_j$ | Armadillo 내부 | E-step |
| 6 | $\pi_k = N_k / N$ | Armadillo → `mixture_em_backend.cpp:103` | M-step 가중치 |
| 7 | $\boldsymbol{\mu}_k = \frac{1}{N_k}\sum \gamma_{ik} \mathbf{x}_i$ | Armadillo → `mixture_em_backend.cpp:87-88` | M-step 평균 |
| 8 | $\boldsymbol{\Sigma}_k = \frac{1}{N_k}\sum \gamma_{ik}(\mathbf{x}_i - \boldsymbol{\mu}_k)(\mathbf{x}_i - \boldsymbol{\mu}_k)^\top$ | Armadillo → `mixture_em_backend.cpp:92-97` | M-step 공분산 |
| 9 | $\|\mathcal{L}^{(t+1)} - \mathcal{L}^{(t)}\| < \tau$ | `mixture_em_backend.cpp:187` | 수렴 판정 |
| 10 | $\tilde{\Sigma}_{rr} = \max(\Sigma_{rr} - \varepsilon, 10^{-6})$ | `mixture_em_backend.cpp:236-238` | 역정칙화 |
| 11 | $\hat{\pi}_k = \pi_k / \sum_j \pi_j$ (또는 $1/K$) | `mixture_em_backend.cpp:248-252` | Warm-start 정규화 |
| 12 | 제거: $\pi_k < \pi_{\min}$ | `mixture_em_backend.cpp:120-123` | Pruning |
| 13 | $\pi_k' = \pi_k / \sum_{j \in S} \pi_j$ | `mixture_em_backend.cpp:141-144` | 재정규화 |
| 14 | $\tilde{\lambda}_i = \max(\lambda_i, \varepsilon_\lambda \lambda_{\max})$ | `integrated_ndt_factor.hpp:51` | 고유값 클램핑 |
| 15 | $\boldsymbol{\Sigma}^{-1} = V \text{diag}(1/\tilde{\lambda}) V^\top$ | `integrated_ndt_factor.hpp:53-54` | 역공분산 |
| 16 | $d_M^2 = (\mathbf{q} - \boldsymbol{\mu})^\top \boldsymbol{\Sigma}^{-1} (\mathbf{q} - \boldsymbol{\mu})$ | `..._factor_impl.hpp:224` | 마할라노비스 |
| 17 | $k^* = \arg\min_k d_M^2$ | `..._factor_impl.hpp:227` | WTA 대응 |
| 18 | $C_i = \pi_{k^*} \mathbf{r}^\top \boldsymbol{\Sigma}_{k^*}^{-1} \mathbf{r}$ | `..._factor_impl.hpp:316` | 포인트 비용 |
| 19 | $J_{\text{target}} = [-[T\mathbf{p}]_\times \; I_3]$ | `..._factor_impl.hpp:332-333` | Target Jacobian |
| 20 | $J_{\text{source}} = [R[\mathbf{p}]_\times \; -R]$ | `..._factor_impl.hpp:339-340` | Source Jacobian |
| 21 | $H \mathrel{+}= \pi_k J^\top \boldsymbol{\Sigma}^{-1} J$ | `..._factor_impl.hpp:348-350` | GN Hessian |
| 22 | $\mathbf{b} \mathrel{+}= \pi_k J^\top \boldsymbol{\Sigma}^{-1} \mathbf{r}$ | `..._factor_impl.hpp:353-354` | Gradient |

---

## 12. 참고 문헌

1. **Vitter, J. S.** (1985). "Random Sampling with a Reservoir." *ACM Trans. Math. Softw.*, 11(1), 37-57.
   — §2 저수지 샘플링의 원전. Algorithm R의 정확성 증명과 복잡도 분석.

2. **Dempster, A. P., Laird, N. M., Rubin, D. B.** (1977). "Maximum Likelihood from Incomplete Data via the EM Algorithm." *J. Royal Statist. Soc. B*, 39(1), 1-38.
   — §3 EM 알고리즘의 원전. Q-함수와 수렴 보장의 일반 프레임워크.

3. **Bishop, C. M.** (2006). *Pattern Recognition and Machine Learning*, Ch. 9.
   — §3 GMM의 EM 유도, 책임도, M-step 유도의 표준 참고서.

4. **Biber, P. & Straßer, W.** (2003). "The Normal Distributions Transform: A New Approach to Laser Scan Matching." *IROS 2003*.
   — NDT의 원전. 복셀 단위 가우시안 표현의 최초 제안.

5. **Magnusson, M.** (2009). "The Three-Dimensional Normal-Distributions Transform." *PhD thesis, Örebro University*.
   — NDT의 3D 확장, d1/d2 파라미터, Hessian 유도. §10의 GN 근사가 이 구조를 계승.

6. **Arthur, D. & Vassilvitskii, S.** (2007). "k-means++: The Advantages of Careful Seeding." *SODA 2007*.
   — §5.1 Random Spread 초기화의 이론적 근거. $O(\log K)$ 근사 보장.

7. **Jian, B. & Vemuri, B. C.** (2011). "Robust Point Set Registration Using Gaussian Mixture Models." *IEEE TPAMI*, 33(8), 1633-1645.
   — GMM 기반 정합의 대안적 접근: 두 GMM 간의 L2 거리 최소화. 본 구현의 WTA 방식과 대비.

8. **Myronenko, A. & Song, X.** (2010). "Point Set Registration: Coherent Point Drift." *IEEE TPAMI*, 32(12), 2262-2275.
   — CPD: GMM을 이용한 확률적 정합. 소프트 할당의 완전한 형태.

9. **Barfoot, T. D.** (2017). *State Estimation for Robotics*, Ch. 7.
   — SE(3) 리 군/대수, 좌측 섭동 모델, Jacobian 유도의 표준 참고서. §9의 유도 기반.

10. **Sandström, C., Magnusson, M., & Andreasson, H.** (2023). "γ-Mixture NDT." *IROS 2023*.
    — GMM과 NDT의 결합에 대한 최신 연구. 소프트 할당 + 혼합 가중 비용의 대안적 설계.

---

> **문서 버전**: Phase 3.6 (2026-03-24)
> 
> **변경 이력**:
> - v1.0 (2026-03-24): 전체 수학적 기초 문서 초안 작성
