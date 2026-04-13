# Point Cloud에 대한 Mixture of Gaussians (MoG) Fitting via EM — 수학적 이론

> **문서 범위**: Point cloud $\mathcal{X} = \{x_1, \ldots, x_N\} \subset \mathbb{R}^D$가 주어졌을 때,
> EM 알고리즘으로 Gaussian Mixture Model을 fitting하는 **수학적 이론 전체**를 서술한다.
> 구현 코드와 무관하게, 확률 모델 정의 → MLE의 난관 → EM 도출 → M-step closed-form 유도 →
> 수렴 보장 → 초기화 전략 → point cloud 특유 고려사항 → model selection까지 다룬다.
>
> **언어**: 본문 한국어, 수식 LaTeX (`$...$` / `$$...$$`).
>
> **관련 문서**:
> - 구현 중심 튜토리얼 → [`em-mog-fitting-tutorial.md`](./em-mog-fitting-tutorial.md)
> - 수학적 증명·유도 (코드 매핑 포함) → [`gmm-mathematical-foundations.md`](./gmm-mathematical-foundations.md)
> - 아키텍처·설계 결정 → [`gmm-design.md`](./gmm-design.md)

---

## 목차

1. [문제 정의](#1-문제-정의)
2. [확률 모델 (Probabilistic Model)](#2-확률-모델-probabilistic-model)
   - 2.1 [Mixture of Gaussians 정의](#21-mixture-of-gaussians-정의)
   - 2.2 [개별 Gaussian의 밀도 함수](#22-개별-gaussian의-밀도-함수)
   - 2.3 [잠재 변수 (Latent Variable) 해석](#23-잠재-변수-latent-variable-해석)
3. [EM 알고리즘을 사용하는 이유](#3-em-알고리즘을-사용하는-이유)
   - 3.1 [직접 MLE의 어려움](#31-직접-mle의-어려움)
   - 3.2 [EM의 핵심 아이디어](#32-em의-핵심-아이디어)
4. [EM 알고리즘 상세 도출](#4-em-알고리즘-상세-도출)
   - 4.1 [Complete-Data Log-Likelihood와 Q-Function](#41-complete-data-log-likelihood와-q-function)
   - 4.2 [E-Step (Expectation Step)](#42-e-step-expectation-step)
   - 4.3 [M-Step (Maximization Step)](#43-m-step-maximization-step)
   - 4.4 [수렴 판정](#44-수렴-판정)
5. [전체 알고리즘 요약](#5-전체-알고리즘-요약)
6. [초기화 전략 (Initialization)](#6-초기화-전략-initialization)
   - 6.1 [K-Means++ 초기화](#61-k-means-초기화)
   - 6.2 [Full K-Means 사전 초기화](#62-full-k-means-사전-초기화)
   - 6.3 [Random Initialization + Multiple Restarts](#63-random-initialization--multiple-restarts)
   - 6.4 [초기화 주의사항](#64-초기화-주의사항)
7. [Point Cloud 특유의 고려사항](#7-point-cloud-특유의-고려사항)
   - 7.1 [Degenerate Covariance 문제](#71-degenerate-covariance-문제)
   - 7.2 [공분산 구조 제한](#72-공분산-구조-제한)
   - 7.3 [대규모 Point Cloud 처리](#73-대규모-point-cloud-처리)
   - 7.4 [수치적 안정성 (Numerical Stability)](#74-수치적-안정성-numerical-stability)
8. [Component 수 K 선택](#8-component-수-k-선택)
   - 8.1 [파라미터 수 산정](#81-파라미터-수-산정)
   - 8.2 [BIC (Bayesian Information Criterion)](#82-bic-bayesian-information-criterion)
   - 8.3 [AIC (Akaike Information Criterion)](#83-aic-akaike-information-criterion)
   - 8.4 [기타 방법](#84-기타-방법)
9. [EM의 이론적 보장과 한계](#9-em의-이론적-보장과-한계)
10. [핵심 수식 요약 테이블](#10-핵심-수식-요약-테이블)
11. [참고 문헌](#11-참고-문헌)

---

## 1. 문제 정의

**주어진 것**: $N$개의 점으로 구성된 point cloud $\mathcal{X} = \{x_1, x_2, \dots, x_N\}$, 각 $x_n \in \mathbb{R}^D$ (3D point cloud의 경우 $D=3$)

**목표**: 이 point cloud의 분포를 $K$개의 Gaussian component의 혼합(mixture)으로 모델링하는 것.

---

## 2. 확률 모델 (Probabilistic Model)

### 2.1 Mixture of Gaussians 정의

데이터 포인트 $x$의 확률 밀도를 다음과 같이 정의한다:

$$p(x \mid \Theta) = \sum_{k=1}^{K} \pi_k \; \mathcal{N}(x \mid \mu_k, \Sigma_k)$$

여기서:

| 기호 | 의미 | 제약 조건 |
|------|------|-----------|
| $K$ | Gaussian component의 개수 | 사전 지정 또는 model selection으로 결정 |
| $\pi_k$ | $k$번째 component의 **mixing coefficient** (가중치) | $\pi_k \geq 0$, $\sum_{k=1}^K \pi_k = 1$ |
| $\mu_k \in \mathbb{R}^D$ | $k$번째 component의 **평균 벡터** | |
| $\Sigma_k \in \mathbb{R}^{D \times D}$ | $k$번째 component의 **공분산 행렬** | symmetric positive definite |
| $\Theta = \{\pi_k, \mu_k, \Sigma_k\}_{k=1}^K$ | 전체 파라미터 집합 | |

### 2.2 개별 Gaussian의 밀도 함수

$$\mathcal{N}(x \mid \mu_k, \Sigma_k) = \frac{1}{(2\pi)^{D/2} |\Sigma_k|^{1/2}} \exp\left(-\frac{1}{2}(x - \mu_k)^T \Sigma_k^{-1} (x - \mu_k)\right)$$

지수 부분의 $(x - \mu_k)^T \Sigma_k^{-1} (x - \mu_k)$는 **Mahalanobis 거리**의 제곱이다.

### 2.3 잠재 변수 (Latent Variable) 해석

각 데이터 포인트 $x_n$에 대해, "이 점이 어느 component에서 생성되었는가"를 나타내는 잠재 변수 $z_n \in \{1, 2, \dots, K\}$를 도입한다.

- **사전 확률 (Prior)**: $p(z_n = k) = \pi_k$
- **조건부 우도 (Likelihood)**: $p(x_n \mid z_n = k) = \mathcal{N}(x_n \mid \mu_k, \Sigma_k)$

완전 데이터(complete data)의 결합 분포:

$$p(x_n, z_n = k \mid \Theta) = \pi_k \cdot \mathcal{N}(x_n \mid \mu_k, \Sigma_k)$$

이것을 $z_n$에 대해 marginalize하면 원래의 mixture 모델을 복원한다:

$$p(x_n \mid \Theta) = \sum_{k=1}^K p(x_n, z_n = k \mid \Theta) = \sum_{k=1}^K \pi_k \cdot \mathcal{N}(x_n \mid \mu_k, \Sigma_k)$$

---

## 3. EM 알고리즘을 사용하는 이유

### 3.1 직접 MLE의 어려움

Log-likelihood를 직접 최적화하려면:

$$\ln p(\mathcal{X} \mid \Theta) = \sum_{n=1}^{N} \ln \left[ \sum_{k=1}^{K} \pi_k \; \mathcal{N}(x_n \mid \mu_k, \Sigma_k) \right]$$

**문제**: $\ln$ 안에 $\sum$이 있으므로, 이 식을 $\mu_k$나 $\Sigma_k$에 대해 미분해도 **closed-form solution이 존재하지 않는다.** 각 component의 파라미터가 다른 component들과 coupling되어 있기 때문이다.

### 3.2 EM의 핵심 아이디어

EM은 이 난관을 **두 단계로 분리**하여 해결한다:

1. **E-step**: 현재 파라미터 $\Theta^{(t)}$를 고정하고, 잠재 변수 $z_n$의 **사후 분포(posterior)**를 계산한다. → 각 점이 각 component에 속할 "책임(responsibility)"을 계산.
2. **M-step**: 이 responsibility를 고정하고, **expected complete-data log-likelihood**를 최대화하는 새로운 파라미터 $\Theta^{(t+1)}$를 구한다. → closed-form update가 존재!

> **닭과 달걀 딜레마**:
> - 각 포인트의 소속을 알면 → 컴포넌트 파라미터를 쉽게 구할 수 있다
> - 컴포넌트 파라미터를 알면 → 각 포인트의 소속을 쉽게 구할 수 있다
> - 둘 다 모를 때 → **EM 알고리즘**이 이 교착을 반복적으로 깨준다

---

## 4. EM 알고리즘 상세 도출

### 4.1 Complete-Data Log-Likelihood와 Q-Function

잠재 변수 $\mathcal{Z} = \{z_1, \ldots, z_N\}$를 포함한 **complete-data log-likelihood**:

$$\ell_c(\Theta; \mathcal{X}, \mathcal{Z}) = \sum_{n=1}^{N} \sum_{k=1}^{K} \mathbf{1}[z_n = k] \cdot \ln\left[\pi_k \cdot \mathcal{N}(x_n \mid \mu_k, \Sigma_k)\right]$$

Iteration $t$에서 현재 파라미터 $\Theta^{(t)}$가 주어졌을 때, EM은 **Q-function** (surrogate objective)을 정의한다:

$$Q(\Theta \mid \Theta^{(t)}) := \mathbb{E}_{\mathcal{Z} \mid \mathcal{X};\, \Theta^{(t)}}\left[\ell_c(\Theta; \mathcal{X}, \mathcal{Z})\right]$$

기대값의 선형성에 의해:

$$Q(\Theta \mid \Theta^{(t)}) = \sum_{n=1}^{N} \sum_{k=1}^{K} \underbrace{p(z_n = k \mid x_n, \Theta^{(t)})}_{\gamma_{nk}} \cdot \ln\left[\pi_k \cdot \mathcal{N}(x_n \mid \mu_k, \Sigma_k)\right]$$

이를 전개하면:

$$Q = \sum_{n=1}^{N} \sum_{k=1}^{K} \gamma_{nk} \left[ \ln \pi_k - \frac{D}{2}\ln(2\pi) - \frac{1}{2}\ln|\Sigma_k| - \frac{1}{2}(x_n - \mu_k)^T\Sigma_k^{-1}(x_n - \mu_k) \right]$$

### 4.2 E-Step (Expectation Step)

현재 파라미터 $\Theta^{(t)} = \{\pi_k^{(t)}, \mu_k^{(t)}, \Sigma_k^{(t)}\}$가 주어졌을 때, 각 데이터 포인트 $x_n$이 component $k$에서 생성되었을 **사후 확률(responsibility)**을 계산한다:

$$\boxed{\gamma_{nk} \equiv p(z_n = k \mid x_n, \Theta^{(t)}) = \frac{\pi_k^{(t)} \; \mathcal{N}(x_n \mid \mu_k^{(t)}, \Sigma_k^{(t)})}{\sum_{j=1}^{K} \pi_j^{(t)} \; \mathcal{N}(x_n \mid \mu_j^{(t)}, \Sigma_j^{(t)})}}$$

**도출**: Bayes' theorem을 직접 적용한 결과.

- **분자**: joint $p(x_n, z_n=k) = \pi_k \cdot \mathcal{N}(x_n \mid \mu_k, \Sigma_k)$
- **분모**: evidence $p(x_n) = \sum_j \pi_j \cdot \mathcal{N}(x_n \mid \mu_j, \Sigma_j)$

**$\gamma_{nk}$의 성질**:

- $0 \leq \gamma_{nk} \leq 1$
- $\sum_{k=1}^K \gamma_{nk} = 1$ (각 데이터 포인트에 대해)

**유효 데이터 수 (Effective number of points)**:

$$N_k = \sum_{n=1}^{N} \gamma_{nk}$$

$N_k$는 component $k$에 "소프트하게 할당된" 데이터 포인트 수. $\sum_k N_k = N$.

### 4.3 M-Step (Maximization Step)

E-step에서 구한 $\gamma_{nk}$를 고정하고, Q-function $Q(\Theta \mid \Theta^{(t)})$를 최대화한다.

#### 4.3.1 $\mu_k$ 업데이트 도출

$Q$를 $\mu_k$에 대해 미분하고 0으로 놓는다:

$$\frac{\partial Q}{\partial \mu_k} = \sum_{n=1}^{N} \gamma_{nk} \; \Sigma_k^{-1}(x_n - \mu_k) = 0$$

$\Sigma_k^{-1}$은 invertible이므로:

$$\sum_{n=1}^{N} \gamma_{nk} \; x_n = \mu_k \sum_{n=1}^{N} \gamma_{nk}$$

$$\boxed{\mu_k^{(t+1)} = \frac{1}{N_k} \sum_{n=1}^{N} \gamma_{nk} \; x_n}$$

**해석**: responsibility로 가중된 **가중 평균**. 직관적으로, component $k$에 가까운 점들이 더 큰 가중치를 받아 $\mu_k$를 끌어당긴다.

#### 4.3.2 $\Sigma_k$ 업데이트 도출

$Q$를 $\Sigma_k^{-1}$ (precision matrix)에 대해 미분한다. 행렬 미적분 항등식을 사용:

- $\frac{\partial \ln|\Sigma|}{\partial \Sigma^{-1}} = -\Sigma$ (symmetric $\Sigma$에 대해)
- $\frac{\partial (a^T \Sigma^{-1} a)}{\partial \Sigma^{-1}} = aa^T$ (symmetric $\Sigma$에 대해)

적용하면:

$$\frac{\partial Q}{\partial \Sigma_k^{-1}} = \sum_{n=1}^{N} \gamma_{nk} \left[ \frac{1}{2}\Sigma_k - \frac{1}{2}(x_n - \mu_k)(x_n - \mu_k)^T \right] = 0$$

정리하면:

$$\boxed{\Sigma_k^{(t+1)} = \frac{1}{N_k} \sum_{n=1}^{N} \gamma_{nk} \; (x_n - \mu_k^{(t+1)})(x_n - \mu_k^{(t+1)})^T}$$

**해석**: responsibility로 가중된 **가중 공분산**. 새로 업데이트된 $\mu_k^{(t+1)}$를 사용한다는 점에 주의.

> **주의**: 공분산 갱신 시 반드시 갱신된 평균 $\mu_k^{(t+1)}$을 사용해야 한다. 이전 평균으로 계산하면 편향이 발생한다.

#### 4.3.3 $\pi_k$ 업데이트 도출

$\pi_k$는 $\sum_k \pi_k = 1$ 제약 조건 하에서 최적화해야 하므로, **Lagrange multiplier** $\lambda$를 도입한다:

$$\mathcal{L} = \sum_{n=1}^N \sum_{k=1}^K \gamma_{nk} \ln \pi_k + \lambda \left(\sum_{k=1}^K \pi_k - 1\right)$$

$$\frac{\partial \mathcal{L}}{\partial \pi_k} = \frac{N_k}{\pi_k} + \lambda = 0 \implies \pi_k = -\frac{N_k}{\lambda}$$

$\sum_k \pi_k = 1$에서 $-\lambda = N$이므로:

$$\boxed{\pi_k^{(t+1)} = \frac{N_k}{N}}$$

**해석**: component $k$의 mixing coefficient는 단순히 그 component에 소프트 할당된 데이터 비율.

### 4.4 수렴 판정

**Log-likelihood 모니터링**:

$$\ell(\Theta) = \sum_{n=1}^{N} \ln \left[ \sum_{k=1}^{K} \pi_k \; \mathcal{N}(x_n \mid \mu_k, \Sigma_k) \right]$$

**EM의 핵심 보장**: 매 iteration마다 $\ell(\Theta^{(t+1)}) \geq \ell(\Theta^{(t)})$. Log-likelihood는 **단조 증가(monotonically non-decreasing)**한다.

이는 다음 부등식에서 유래:

$$\ell(\Theta; \mathcal{X}) \geq Q(\Theta \mid \Theta^{(t)}) + \underbrace{H(\mathcal{Z} \mid \mathcal{X}; \Theta^{(t)})}_{\text{const w.r.t.} \; \Theta}$$

여기서 $H$는 잠재 변수의 조건부 엔트로피.

**수렴 기준** (하나 이상 사용):

| 기준 | 수식 | 일반적 값 |
|------|------|-----------|
| Log-likelihood 절대 변화량 | $\|\ell(\Theta^{(t+1)}) - \ell(\Theta^{(t)})\| < \epsilon$ | $\epsilon = 10^{-6}$ |
| 상대 변화량 | $\frac{\|\ell^{(t+1)} - \ell^{(t)}\|}{\|\ell^{(t)}\| + \epsilon_{\text{abs}}} < \epsilon_{\text{rel}}$ | $\epsilon_{\text{rel}} = 10^{-4}$ |
| 파라미터 변화량 | $\max_k \|\mu_k^{(t+1)} - \mu_k^{(t)}\| < \delta$ | |
| 최대 반복 횟수 | $t > T_{\max}$ | $T_{\max} = 100 \sim 300$ |

---

## 5. 전체 알고리즘 요약

```
입력: Point cloud X = {x₁, ..., xₙ}, component 수 K
출력: 파라미터 Θ = {πₖ, μₖ, Σₖ}ₖ₌₁ᴷ

1. 초기화: π⁰, μ⁰, Σ⁰ 설정 (섹션 6 참조)

2. REPEAT:
   ┌─────────────────────────────────────────────────┐
   │ E-Step:                                         │
   │   for n = 1..N, k = 1..K:                      │
   │     γₙₖ = πₖ·𝒩(xₙ|μₖ,Σₖ) / Σⱼ πⱼ·𝒩(xₙ|μⱼ,Σⱼ) │
   │   for k = 1..K:                                │
   │     Nₖ = Σₙ γₙₖ                                │
   └─────────────────────────────────────────────────┘
   ┌─────────────────────────────────────────────────┐
   │ M-Step:                                         │
   │   for k = 1..K:                                │
   │     μₖ ← (1/Nₖ) Σₙ γₙₖ·xₙ                    │
   │     Σₖ ← (1/Nₖ) Σₙ γₙₖ·(xₙ-μₖ)(xₙ-μₖ)ᵀ     │
   │     πₖ ← Nₖ/N                                  │
   └─────────────────────────────────────────────────┘
   
   log-likelihood ℓ 계산

3. UNTIL 수렴 조건 만족
```

---

## 6. 초기화 전략 (Initialization)

EM은 **local optimum**에 수렴하므로, 초기화가 결과 품질에 결정적이다.

### 6.1 K-Means++ 초기화

Arthur & Vassilvitskii (2007)의 K-Means++ seeding — $O(\log K)$ 근사 보장을 제공하며 가장 권장되는 방법:

**알고리즘:**

1. $\mu_1$을 $\{x_1, \ldots, x_N\}$에서 균등 무작위 선택
2. $k = 2, \ldots, K$에 대해:
   - $D(x_n) = \min_{j < k} \|x_n - \mu_j\|^2$ 계산 (가장 가까운 기존 중심까지의 거리 제곱)
   - 확률 $P(\mu_k = x_n) = \frac{D(x_n)}{\sum_{m=1}^N D(x_m)}$로 다음 중심 선택
3. 결과 $\{\mu_1, \ldots, \mu_K\}$를 초기 평균으로 사용
4. $\Sigma_k = \sigma^2 I$, $\pi_k = 1/K$로 초기화

거리 제곱에 비례한 확률 샘플링으로 초기 중심들이 point cloud 전체에 잘 분산된다.

### 6.2 Full K-Means 사전 초기화

K-Means를 수렴까지 실행한 후:

$$\mu_k^{(0)} = \text{K-means centroid}_k, \quad \Sigma_k^{(0)} = \frac{1}{|C_k|}\sum_{n \in C_k}(x_n - \mu_k^{(0)})(x_n - \mu_k^{(0)})^T, \quad \pi_k^{(0)} = \frac{|C_k|}{N}$$

가장 robust하지만 대규모 point cloud에서는 계산 비용이 크다.

### 6.3 Random Initialization + Multiple Restarts

1. $\mu_k$를 데이터에서 랜덤하게 $K$개 점을 선택
2. $\Sigma_k = \text{diag}(\sigma^2)$ (데이터 전체 분산의 일부)
3. $\pi_k = 1/K$
4. 여러 번(예: 10회) 독립적으로 실행하여 **최고 log-likelihood 결과 선택**

### 6.4 초기화 주의사항

- $\Sigma_k$를 너무 작게 초기화하면 → 하나의 점에 collapse할 수 있음
- $\Sigma_k$를 너무 크게 초기화하면 → 모든 component가 같아져 수렴이 느림
- 모든 $\mu_k$가 같으면 → **symmetry breaking이 안 됨**, 영원히 같은 상태 유지

---

## 7. Point Cloud 특유의 고려사항

### 7.1 Degenerate Covariance (공분산 특이성) 문제

Point cloud에서 흔히 발생하는 상황:

- Component에 할당된 점이 너무 적을 때 ($N_k < D$)
- 점들이 평면(plane) 또는 직선(line) 위에 있을 때
- $\Sigma_k$가 **singular** (역행렬 계산 불가) 또는 **near-singular** (수치 불안정)

이 경우 $|\Sigma_k| \to 0$이면 $\mathcal{N}(x_n \mid \mu_k, \Sigma_k) \to \infty$가 되어 **likelihood blow-up** (pathological solution) 발생.

#### 해결 1: Tikhonov 정칙화 (표준 방법)

$$\Sigma_k^{\text{reg}} = \Sigma_k + \epsilon I$$

$\epsilon$은 작은 양수 (예: $10^{-6}$). Bayesian 관점에서 $\Sigma_k$에 Inverse-Wishart prior를 두는 것과 유사한 효과. 모든 고유값이 $\epsilon$ 이상으로 하한된다.

데이터 기반 $\epsilon$ 선택:

$$\epsilon = \epsilon_0 \cdot \frac{\text{tr}(\hat{\Sigma}_{\text{global}})}{D}$$

여기서 $\hat{\Sigma}_{\text{global}}$은 전체 point cloud의 표본 공분산.

#### 해결 2: 최소 고유값 클리핑

고유값 분해 $\Sigma_k = V \Lambda V^T$ 후:

$$\Sigma_k^{\text{reg}} = V \cdot \max(\Lambda, \epsilon I) \cdot V^T$$

#### 해결 3: Component 재시작

$N_k < N_{\min}$ (예: $N_{\min} = D + 1 = 4$, 3D의 경우)이면, 최대 가중치 component를 분할하거나 데이터에서 새 중심을 샘플링하여 component $k$를 재초기화.

### 7.2 공분산 구조 제한

계산량을 줄이거나 안정성을 높이기 위해 $\Sigma_k$의 구조를 제한할 수 있다:

| 유형 | 형태 | 파라미터 수 (3D, per component) | 설명 |
|------|------|-------------------------------|------|
| **Full** | 일반 $\Sigma_k$ | 6 | 임의 방향 타원체. 가장 유연하지만 파라미터 多 |
| **Diagonal** | $\text{diag}(\sigma_{k1}^2, \sigma_{k2}^2, \sigma_{k3}^2)$ | 3 | 축 정렬 타원체 |
| **Spherical** | $\sigma_k^2 I$ | 1 | 등방성(isotropic) 구형 |
| **Tied** | 모든 $k$가 같은 $\Sigma$ 공유 | 6 (전체) | 동일 형태, 위치만 다름 |

실제 LiDAR point cloud는 표면을 따라 분포하는 경향이 있어 **Full covariance**가 적절한 경우가 많다 — 평면을 따라 납작한 타원체(thin flat ellipsoid)를 모델링할 수 있기 때문.

### 7.3 대규모 Point Cloud 처리

$N$이 수십만~수백만일 때의 전략:

#### 7.3.1 계산 복잡도 분석

Per EM iteration:
- **E-step**: $O(NKD^2)$ — $\mathcal{N}(x_n \mid \mu_k, \Sigma_k)$ 계산에 $\Sigma_k^{-1}$ 곱셈 필요 ($\Sigma_k^{-1}$는 iteration당 1회 $O(D^3)$으로 사전 계산)
- **M-step**: $O(NKD^2)$ — outer product $(x_n - \mu_k)(x_n - \mu_k)^T$

총: $O(T \cdot NK \cdot D^2)$ ($T$ = iteration 수)

$N=10^6$, $K=100$, $D=3$: **iteration당 약 $9 \times 10^9$ 연산** — 상당한 비용.

#### 7.3.2 Sufficient Statistics 활용

E-step에서 전체 $\gamma$ 행렬($N \times K$)을 저장하지 않고, M-step에 필요한 **충분통계량**만 축적:

$$S_k^{(0)} = \sum_n \gamma_{nk} \quad (= N_k)$$

$$S_k^{(1)} = \sum_n \gamma_{nk} \, x_n$$

$$S_k^{(2)} = \sum_n \gamma_{nk} \, x_n x_n^T$$

그러면:

$$\mu_k = \frac{S_k^{(1)}}{S_k^{(0)}}, \qquad \Sigma_k = \frac{S_k^{(2)}}{S_k^{(0)}} - \mu_k \mu_k^T$$

메모리: $O(NK) \to O(K(D^2 + D + 1))$로 축소.

#### 7.3.3 Mini-batch EM

매 iteration에서 데이터의 subset만 사용:

- E-step을 mini-batch $\mathcal{B} \subset \{1,\ldots,N\}$에 대해서만 수행
- Running average로 충분통계량 축적:

$$\hat{N}_k \leftarrow (1-\eta)\hat{N}_k + \eta\sum_{n \in \mathcal{B}}\gamma_{nk}$$

#### 7.3.4 Subsampling + Refinement

먼저 subsampled cloud (voxel-grid filter 등)로 GMM을 학습하고, 전체 cloud에 대해 fine-tuning.

### 7.4 수치적 안정성 (Numerical Stability)

Gaussian 밀도 계산 시 $\exp(-\frac{1}{2}(x-\mu)^T\Sigma^{-1}(x-\mu))$가 **극도로 작은 값**이 될 수 있어 underflow 위험.

#### Log-space 연산

E-step을 log-space에서 수행:

$$\ln \gamma_{nk} = \ln \pi_k + \ln \mathcal{N}(x_n|\mu_k, \Sigma_k) - \text{logsumexp}_j \left[\ln \pi_j + \ln \mathcal{N}(x_n|\mu_j, \Sigma_j)\right]$$

**logsumexp trick**:

$$\text{logsumexp}(a_1, \ldots, a_K) = a_{\max} + \ln \sum_{k=1}^K \exp(a_k - a_{\max})$$

$a_{\max} = \max_k a_k$로, overflow/underflow를 방지한다.

#### Cholesky 분해 활용

$\Sigma_k$의 역행렬 계산 시 직접 역행렬 대신 **Cholesky decomposition** 사용:

$$\Sigma_k = L_k L_k^T$$

- **Log-determinant**: $\ln |\Sigma_k| = 2 \sum_i \ln (L_k)_{ii}$
- **Mahalanobis 거리**: $(x-\mu)^T \Sigma^{-1}(x-\mu) = \|L_k^{-1}(x-\mu)\|^2$ (forward substitution으로 $O(D^2)$에 계산)
- $\Sigma_k^{-1}$를 명시적으로 구하지 않으므로 수치적으로 안정

---

## 8. Component 수 K 선택

### 8.1 파라미터 수 산정

GMM의 자유 파라미터 수 $|\Theta|$:

| 공분산 유형 | 수식 | 3D ($D=3$) |
|------------|------|-----------|
| Full | $K \cdot \left[\frac{D(D+1)}{2} + D + 1\right] - 1$ | $10K - 1$ |
| Diagonal | $K \cdot (2D + 1) - 1$ | $7K - 1$ |
| Spherical | $K \cdot (D + 2) - 1$ | $5K - 1$ |
| Tied (Full) | $K \cdot (D+1) - 1 + \frac{D(D+1)}{2}$ | $4K + 5$ |

각 component 기여: $D(D+1)/2$ (covariance, symmetric) + $D$ (mean) + $1$ (weight). 전체에서 $-1$은 $\sum \pi_k = 1$ 제약.

### 8.2 BIC (Bayesian Information Criterion)

**Schwarz (1978)**:

$$\text{BIC}(K) = -2\,\ell(\hat{\Theta}_K; \mathcal{X}) + |\hat{\Theta}_K| \cdot \ln N$$

$\ln N$ penalty는 AIC보다 강하다. **GMM model selection에서 가장 널리 사용**.

BIC는 log-marginal-likelihood의 대표본 근사:

$$\ln p(\mathcal{X} \mid K) \approx \ell(\hat{\Theta}_K; \mathcal{X}) - \frac{|\hat{\Theta}_K|}{2}\ln N + O(1)$$

**절차**:
1. 각 후보 $K \in \{K_{\min}, \ldots, K_{\max}\}$에 대해 EM을 수렴까지 실행, $\text{BIC}(K)$ 계산
2. $K^* = \arg\min_K \text{BIC}(K)$ 선택
3. BIC 곡선은 전형적으로 "elbow" — 급격한 감소 후 평탄화 — 를 보임

### 8.3 AIC (Akaike Information Criterion)

**Akaike (1974)**:

$$\text{AIC}(K) = -2\,\ell(\hat{\Theta}_K; \mathcal{X}) + 2\,|\hat{\Theta}_K|$$

정보이론적 유도: true distribution과의 KL divergence 최소화, MLE bias에 대한 1차 보정. BIC보다 더 많은 component를 허용하는 경향.

### 8.4 기타 방법

| 방법 | 설명 |
|------|------|
| **Cross-validation** | Train/validation 분할, held-out log-likelihood로 $K$ 평가. 모델 가정에 자유로움 |
| **ICL (Integrated Completed Likelihood)** | $\text{ICL}(K) = \text{BIC}(K) + 2\sum_{n,k} \gamma_{nk} \ln \gamma_{nk}$. Classification uncertainty에 추가 페널티. 잘 분리된 클러스터 선호 |
| **Elbow method** | Log-likelihood vs $K$ 그래프에서 변곡점 |
| **Silhouette score** | 클러스터링 품질 측정 (hard assignment 필요) |
| **DPGMM** | Dirichlet Process GMM — $K$를 자동 결정. $K_{\text{eff}} \sim O(\alpha \ln N)$으로 데이터에서 추론. 사전에 $K$ 지정 불필요 |

---

## 9. EM의 이론적 보장과 한계

### 보장

- **단조 수렴**: 매 step에서 $\ell(\Theta^{(t+1)}) \geq \ell(\Theta^{(t)})$ (Jensen 부등식에 의한 증명)
- **수렴**: bounded log-likelihood + monotone → 반드시 수렴 (값의 수렴)
- **Stationary point**: 수렴점은 log-likelihood의 stationary point (필요 조건 만족)

### 한계

- **Local optimum**: Global maximum 보장 없음 → 초기화와 multiple restarts가 중요
- **수렴 속도**: Near-convergence에서 linear rate (Newton 방법의 quadratic rate보다 느림)
- **Singularity**: Component가 단일 점에 collapse 가능 → 정칙화 필요
- **Component 수**: $K$를 사전에 알아야 함 (또는 model selection / DPGMM 필요)

---

## 10. 핵심 수식 요약 테이블

| 수식 | 설명 |
|------|------|
| $p(x) = \sum_{k=1}^K \pi_k \, \mathcal{N}(x \mid \mu_k, \Sigma_k)$ | MoG 모델 정의 |
| $\mathcal{N}(x \mid \mu, \Sigma) = \frac{1}{(2\pi)^{D/2}\|\Sigma\|^{1/2}}\exp\left[-\frac{1}{2}(x-\mu)^T\Sigma^{-1}(x-\mu)\right]$ | Gaussian 밀도 함수 |
| $\gamma_{nk} = \frac{\pi_k\,\mathcal{N}(x_n \mid \mu_k, \Sigma_k)}{\sum_j \pi_j\,\mathcal{N}(x_n \mid \mu_j, \Sigma_j)}$ | E-Step: Responsibility |
| $N_k = \sum_n \gamma_{nk}$ | 유효 데이터 수 |
| $\mu_k \leftarrow \frac{1}{N_k}\sum_n \gamma_{nk}\,x_n$ | M-Step: 평균 업데이트 |
| $\Sigma_k \leftarrow \frac{1}{N_k}\sum_n \gamma_{nk}\,(x_n - \mu_k)(x_n - \mu_k)^T$ | M-Step: 공분산 업데이트 |
| $\pi_k \leftarrow N_k / N$ | M-Step: 가중치 업데이트 |
| $\Sigma_k^{\text{reg}} = \Sigma_k + \epsilon I$ | Tikhonov 정칙화 |
| $\ell(\Theta) = \sum_n \ln \sum_k \pi_k\,\mathcal{N}(x_n \mid \mu_k, \Sigma_k)$ | Log-likelihood |
| $\text{BIC}(K) = -2\ell(\hat{\Theta}_K) + \|\Theta_K\| \ln N$ | Bayesian Information Criterion |
| $\text{AIC}(K) = -2\ell(\hat{\Theta}_K) + 2\|\Theta_K\|$ | Akaike Information Criterion |
| Full covariance 3D: $\|\Theta\| = 10K - 1$ | 자유 파라미터 수 |

---

## 11. 참고 문헌

1. **Dempster, A. P., Laird, N. M., & Rubin, D. B.** (1977). *Maximum Likelihood from Incomplete Data via the EM Algorithm.* Journal of the Royal Statistical Society, Series B, 39(1), 1–38.

2. **Bishop, C. M.** (2006). *Pattern Recognition and Machine Learning.* Chapter 9: Mixture Models and EM. Springer.

3. **Arthur, D., & Vassilvitskii, S.** (2007). *k-means++: The Advantages of Careful Seeding.* ACM-SIAM Symposium on Discrete Algorithms (SODA).

4. **Schwarz, G.** (1978). *Estimating the Dimension of a Model.* Annals of Statistics, 6(2), 461–464.

5. **Petersen, K. B., & Pedersen, M. S.** *The Matrix Cookbook.* — EM M-step 공분산 도출에 사용된 행렬 미적분 항등식.

6. **Fraiss, S.** (2022). *Compact Gaussian Mixture Models for 3D Object Representation.* Master Thesis, TU Wien. — 3D point cloud에 대한 GMM 구축 및 시각화.

7. **Eckart, B., Kim, K., & Kautz, J.** (2016). *Accelerated Generative Models for 3D Point Cloud Data.* NVIDIA Research. — GPU 가속 hierarchical GMM fitting for LiDAR point clouds.

8. **Neal, R. M., & Hinton, G. E.** (1998). *A View of the EM Algorithm that Justifies Incremental, Sparse, and Other Variants.* — Mini-batch EM의 이론적 근거.

---

최종 수정: 2026-04-02
