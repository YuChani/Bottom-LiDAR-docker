# NDT Conditional H2 Hessian 최적화 — 최종 보고서

**작성일**: 2026-03-09  
**상태**: Step 3-B (Conditional H2, weight > 0.01) 적용 완료  
**수정 파일**: `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp`

---

## 1. 배경 및 목표

### 1.1 문제 정의

NDT(Normal Distributions Transform) factor가 다른 정합 factor 대비 **iteration 수가 압도적으로 많고**(74회 vs GICP 9회, LightNDT 6회), 결과적으로 **수렴 시간이 3~5배 느린** 문제를 해결하고자 함.

### 1.2 제약 조건

| 항목 | 제약 |
|---|---|
| 수정 범위 | NDT 구현 코드만 (`integrated_ndt_factor.hpp`, `integrated_ndt_factor_impl.hpp`) |
| main.cpp | 수정 불가 |
| LM optimizer | 수정 불가 |
| NDT 수식 | Canonical NDT (exponential score function) 정체성 보존 필수 |
| 실험 환경 | Docker 컨테이너 내 headless 벤치마크 |

### 1.3 목표

- Iteration 수를 74회에서 대폭 감소 (다른 factor 수준에 근접)
- 정합 정확도(Mean Translation Error, Mean Rotation Error) 유지
- 총 실행 시간 개선 또는 최소한 유지

---

## 2. 근본 원인 분석

### 2.1 NDT vs LightNDT — 유일한 차이

NDT와 LightNDT는 **correspondence 탐색, Jacobian 계산, voxel lookup**이 모두 동일하다. **딱 하나**, cost/weight 계산만 다르다:

```
NDT:
  cost   = -d1 * (1 - exp(-d2/2 * m²))
  weight = -d1 * d2 * exp(-d2/2 * m²)

LightNDT:
  cost   = m²  (Mahalanobis distance)
  weight = 1   (uniform)
```

### 2.2 왜 느린가: "달팽이 수렴"

NDT weight는 Mahalanobis 거리가 클수록 **지수적으로 감소**한다:

```
weight = -d1 * d2 * exp(-d2/2 * m²)
       ≈ 4.51 × 0.23 × exp(-0.115 × m²)
       ≈ 1.04 × exp(-0.115 × m²)
```

| Mahalanobis² (m²) | weight | Hessian 기여 |
|---:|---:|---|
| 0.0 | 1.04 | 100% |
| 1.0 | 0.93 | 89% |
| 5.0 | 0.59 | 57% |
| 10.0 | 0.33 | 32% |
| 20.0 | 0.10 | 10% |
| 50.0 | 0.003 | ~0% |

초기 정합에서 대부분의 포인트는 m² >> 10 → weight ≈ 0 → **Hessian이 극도로 작아짐** → LM step 극소 → "달팽이처럼" 한 발짝씩 수렴 (74 iterations).

LightNDT는 weight = 1 (uniform)이므로 모든 포인트가 동등하게 기여 → 큰 Hessian → 큰 step → 6 iterations에 수렴.

### 2.3 해법: H2 Hessian 항 추가

원래 NDT 구현은 Magnusson (2009) Eq. 6.13의 **H1 항(Gauss-Newton 근사)만** 사용:

```
H1 = w * J^T * Σ⁻¹ * J         (PSD 보장, but 작은 step)
```

Magnusson의 full Hessian에는 **H2 항**(second-order term)도 포함:

```
H2 = -w * d2 * (J^T Σ⁻¹ r)(J^T Σ⁻¹ r)^T    (rank-1 update, may be indefinite)
```

H2를 추가하면 **Newton step에 가까운 더 큰 step**을 얻어 수렴이 빨라진다.

### 2.4 H2와 LM optimizer의 궁합 문제

| 프레임워크 | Solve 방식 | Indefinite Hessian 대응 |
|---|---|---|
| PCL NDT / ndt_omp | Newton + More-Thuente line search + SVD | Indefinite 허용, line search로 보정 |
| **우리 코드 (GTSAM LM)** | Trust region + Cholesky factorization | **PSD 전제** → indefinite 시 λ 폭발 |

→ H2를 **무조건 전체 포인트에 적용**하면 Hessian이 indefinite 될 수 있고, LM의 λ가 폭발하여 gradient descent로 퇴화할 위험이 있음.

→ **해결**: weight가 충분히 큰 포인트(정합이 좋은 포인트)에만 조건부로 H2를 적용하면, H1의 PSD 항이 지배적이면서도 수렴 속도 개선 효과를 얻을 수 있음.

---

## 3. 실험 이력 (시간순)

### 3.0 기준선 (원복 상태, 5회 평균)

| Factor | Outer Iter | Time (ms) | Mean Trans (m) | Mean Rot (deg) |
|---|---:|---:|---:|---:|
| Point-to-Point | 26 | 810 | 0.1010 | 0.4361 |
| Point-to-Plane | 15 | 516 | 0.0618 | 0.4307 |
| GICP | 9 | 631 | 0.0854 | 0.4994 |
| VGICP | 12 | 858 | 0.1174 | 0.6356 |
| **NDT** | **74** | **2,523** | **0.1020** | **0.6976** |
| LightNDT | 6 | 324 | 0.1590 | 0.6406 |
| LOAM_LIOSAM | 8 | 41 | 0.2254 | 0.7746 |

### 3.1 Step 1+2: 공분산 정규화 + Weight-gating (단독)

**방법**: 공분산 고유값 정규화 강화 + weight < threshold인 포인트를 Hessian 기여에서 제외

**결과**: ❌ **실패 — 기준선보다 악화**

| 지표 | 기준선 | Step 1+2 | 변화 |
|---|---:|---:|---|
| Outer Iter | 74 | 85 | 14.9% 증가 ❌ |
| Time (ms) | 2,523 | ~6,920 | 174% 증가 ❌ |

**원인**: weight-gating이 Hessian에 기여하는 포인트 수를 줄임 → Hessian이 더 작아짐 → step이 더 작아져 수렴 악화.

**조치**: 원복 완료.

### 3.2 Step 3-A: Full H2 (α=1.0, 모든 포인트)

**방법**: Magnusson Eq. 6.13의 H2 항을 **모든 포인트**에 대해 무조건 추가.

**결과**: △ **부분 성공** (3회 평균)

| 지표 | 기준선 | Full H2 | 변화 |
|---|---:|---:|---|
| Outer Iter | 74 | 32 | 57% 감소 ✅ |
| Time (ms) | 2,523 | ~3,651 | 45% 증가 ❌ |
| Mean Trans (m) | 0.1020 | ~0.102 | 동일 |
| Mean Rot (deg) | 0.6976 | ~0.70 | 동일 |

**분석**: Iteration은 크게 줄었으나, 매 포인트마다 H2 계산(rank-1 outer product 3회) 추가로 per-iter 비용이 3.4배 증가 → 총 시간은 오히려 증가.

### 3.3 Step 3-B: Conditional H2 (weight > 0.01) ← **현재 적용**

**방법**: weight > 0.01인 포인트에만 H2 항 추가. 나머지 포인트는 H1만 사용.

**근거**: weight가 작은 포인트(정합이 나쁜 포인트)는 H2를 계산해도 기여가 미미하고, indefinite 위험만 높임. weight가 큰 포인트(정합이 좋은 포인트)에서 H2가 유의미한 수렴 가속 효과를 발휘.

**결과**: ✅ **현재 최선** (5회 개별 결과)

| 실험 | Outer Iter | Time (ms) |
|---:|---:|---:|
| 1 | 25 | 2,608 |
| 2 | 25 | 2,704 |
| 3 | 25 | 2,842 |
| 4 | 25 | 2,961 |
| 5 | 25 | 3,060 |
| **평균** | **25** | **2,835** |
| **표준편차** | **0** | **170** |

**기준선 대비 비교**:

| 지표 | 기준선 | Step 3-B | 변화 |
|---|---:|---:|---|
| Outer Iter | 74 | 25 | **67.6% 감소** ✅ |
| Time (ms) | 2,523 | 2,835 | 12.4% 증가 △ |
| Mean Trans (m) | 0.1020 | 0.1019 | 동일 ✅ |
| Mean Rot (deg) | 0.6976 | 0.7080 | 동일 ✅ |

### 3.4 Step 3-C: d2-gated H2 (Mahalanobis² < 9.0)

**방법**: weight 대신 Mahalanobis 거리로 gating. `mahalanobis_dist < 9.0`인 포인트에만 H2 적용.

**결과**: ❌ **실패**

| 지표 | 기준선 | d2-gated | 변화 |
|---|---:|---:|---|
| Outer Iter | 74 | 46 | 38% 감소 |
| Time (ms) | 2,523 | ~5,191 | 106% 증가 ❌ |

**원인**: threshold 9.0이 너무 느슨하여 거의 모든 포인트에 H2 적용됨 → Full H2와 비슷한 per-iter 비용.

---

## 4. 최종 적용 코드 변경

### 4.1 변경 파일

`thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp`

### 4.2 변경 내용 (Line 280-303)

```cpp
    const double weight = -gauss_d1 * gauss_d2 * e_term;

    //  Gauss-Newton 근사 Hessian (Magnusson Eq. 6.13의 H1 항) 
    // H1 = weight * J^T * Σ^{-1} * J   (PSD 보장)
    const Eigen::Matrix<double, 6, 1> Jt_Sinv_r_target = J_target.transpose() * inv_cov_B * residual;
    const Eigen::Matrix<double, 6, 1> Jt_Sinv_r_source = J_source.transpose() * inv_cov_B * residual;

    *H_target += weight * J_target.transpose() * inv_cov_B * J_target;
    *H_source += weight * J_source.transpose() * inv_cov_B * J_source;
    *H_target_source += weight * J_target.transpose() * inv_cov_B * J_source;

    // Step 3: H2 항 조건부 추가 (Magnusson Eq. 6.13)
    // weight가 충분한 포인트에만 H2 계산 → per-iter 비용 절감
    // H2 = -weight * d2 * (J^T Σ^{-1} r)(J^T Σ^{-1} r)^T
    if (weight > 0.01) {
      const double h2_coeff = -weight * gauss_d2;
      *H_target += h2_coeff * Jt_Sinv_r_target * Jt_Sinv_r_target.transpose();
      *H_source += h2_coeff * Jt_Sinv_r_source * Jt_Sinv_r_source.transpose();
      *H_target_source += h2_coeff * Jt_Sinv_r_target * Jt_Sinv_r_source.transpose();
    }

    // b = weight * J^T * Σ^{-1} * r    (gradient)
    *b_target += weight * Jt_Sinv_r_target;
    *b_source += weight * Jt_Sinv_r_source;
```

### 4.3 변경 전 코드 (기준선)

```cpp
    const double weight = -gauss_d1 * gauss_d2 * e_term;

    *H_target += weight * J_target.transpose() * inv_cov_B * J_target;
    *H_source += weight * J_source.transpose() * inv_cov_B * J_source;
    *H_target_source += weight * J_target.transpose() * inv_cov_B * J_source;

    *b_target += weight * (J_target.transpose() * inv_cov_B * residual);
    *b_source += weight * (J_source.transpose() * inv_cov_B * residual);
```

### 4.4 핵심 변경점 요약

| # | 변경 | 설명 |
|---|---|---|
| 1 | `Jt_Sinv_r` 벡터 사전 계산 | `J^T Σ⁻¹ r`을 별도 변수로 추출 → H2와 gradient 양쪽에서 재사용 |
| 2 | Conditional H2 블록 추가 | `weight > 0.01` 조건 하에 H2 = `-w·d2·(J^T Σ⁻¹ r)(J^T Σ⁻¹ r)^T` 추가 |
| 3 | gradient에 `Jt_Sinv_r` 재사용 | 기존 `J^T * inv_cov * residual` 재계산 → 사전 계산된 벡터 사용 |

---

## 5. 전체 Factor 성능 비교표

### 5.1 기준선 (Step 3-B 적용 전)

| Factor | Outer Iter | Time (ms) | Mean Trans (m) | Mean Rot (deg) |
|---|---:|---:|---:|---:|
| Point-to-Point | 26 | 810 | 0.1010 | 0.4361 |
| Point-to-Plane | 15 | 516 | 0.0618 | 0.4307 |
| GICP | 9 | 631 | 0.0854 | 0.4994 |
| VGICP | 12 | 858 | 0.1174 | 0.6356 |
| **NDT** | **74** | **2,523** | **0.1020** | **0.6976** |
| LightNDT | 6 | 324 | 0.1590 | 0.6406 |
| LOAM_LIOSAM | 8 | 41 | 0.2254 | 0.7746 |

### 5.2 Step 3-B 적용 후

| Factor | Outer Iter | Time (ms) | Mean Trans (m) | Mean Rot (deg) |
|---|---:|---:|---:|---:|
| Point-to-Point | 26 | 810 | 0.1010 | 0.4361 |
| Point-to-Plane | 15 | 516 | 0.0618 | 0.4307 |
| GICP | 9 | 631 | 0.0854 | 0.4994 |
| VGICP | 12 | 858 | 0.1174 | 0.6356 |
| **NDT** | **25** | **2,835** | **0.1019** | **0.7080** |
| LightNDT | 6 | 324 | 0.1590 | 0.6406 |
| LOAM_LIOSAM | 8 | 41 | 0.2254 | 0.7746 |

### 5.3 NDT 변화 요약

| 지표 | 기준선 | Step 3-B | 변화율 |
|---|---:|---:|---|
| Outer Iterations | 74 | 25 | **-66.2%** |
| Total Time | 2,523 ms | 2,835 ms | +12.4% |
| Per-iter Time | 34.1 ms | 113.4 ms | +232% (H2 계산 비용) |
| Mean Trans Error | 0.1020 m | 0.1019 m | -0.1% (동일) |
| Mean Rot Error | 0.6976° | 0.7080° | +1.5% (동일) |

---

## 6. 실패한 접근법 요약

| 접근법 | 핵심 아이디어 | 결과 | 실패 원인 |
|---|---|---|---|
| P0 패치 3개 | gradient clamp, 공분산 정규화, exponent clamp 강화 | 변화 없음 | 근본 원인(H1-only)을 건드리지 않음 |
| Step 1+2 | 공분산 정규화 + weight-gating | 85 iter, 6,920ms | Hessian 기여 포인트 제거 → step 축소 |
| Step 3-A (Full H2) | 모든 포인트에 H2 추가 | 32 iter, 3,651ms | Per-iter 비용 3.4x → 총 시간 45% 증가 |
| Step 3-C (d2-gated) | Mahalanobis < 9.0 조건 H2 | 46 iter, 5,191ms | Threshold 너무 느슨, 거의 모든 포인트 해당 |
| Cauchy/Student-t IRLS | NDT loss를 robust loss로 교체 | 미실시 (기각) | NDT 수식 정체성 파괴 (사용자 기각) |

---

## 7. 추가 최적화 여지 (미실시)

Oracle 분석에서 제안된 per-iter 비용 개선 전략. 총 시간을 기준선(2,523ms) 이하로 줄이기 위한 옵션:

| 전략 | 설명 | 난이도 | NDT 코드만 수정 |
|---|---|---|---|
| **Iteration scheduling** | mutable 카운터로 초반 N회만 H2, 이후 H1만 | 낮음 | ✅ 가능 |
| **H2 threshold 조정** | weight > 0.01 → 더 높은 값으로 조정 | 낮음 | ✅ 가능 |
| **Symmetric update** | 6x6 outer product의 upper triangle만 계산 | 중간 | ✅ 가능 |
| **Thread-local accumulation** | OMP shared writes 동기화 비용 제거 | 높음 | ❌ `scan_matching_reduce_omp` 수정 필요 |

---

## 8. 결론

### 8.1 성과

- **Iteration 67.6% 감소** (74 → 25): Magnusson의 full Hessian H2 항을 조건부로 추가하여, NDT의 "달팽이 수렴" 문제를 해결
- **정합 정확도 유지**: Mean Translation Error 0.1019m, Mean Rotation Error 0.708° (기준선과 동일)
- **NDT 수식 정체성 완전 보존**: exponential score function, d1/d2 파라미터, Mahalanobis 거리 기반 weighting 모두 원래 그대로

### 8.2 한계

- **총 실행 시간 12.4% 증가** (2,523ms → 2,835ms): H2 항의 rank-1 outer product 연산 비용으로 per-iter 시간이 증가
- 추가 최적화(iteration scheduling, threshold 조정 등)로 해결 가능하나 현재 미적용

### 8.3 판단

H2 조건부 추가는 NDT canonical 수식의 **원래 Hessian을 복원**하는 것이지, 수식을 변경하는 것이 아님. 
기존 구현이 H1만 사용한 것은 "Gauss-Newton 근사"이며, H2를 추가한 것이 Magnusson (2009) 원논문의 full Newton Hessian에 가까운 구현. 
따라서 이 변경은 **NDT를 더 정확하게 구현**한 것이다.

---

## 참조

- Magnusson, M. (2009). *The Three-Dimensional Normal-Distributions Transform*, PhD Thesis, Örebro University. Eq. 6.9, 6.13
- Biber, P., & Strasser, W. (2003). *The Normal Distributions Transform: A New Approach to Laser Scan Matching*, IROS
- Koide, K. (2024). *gtsam_points: A collection of GTSAM factors and optimizers for point cloud SLAM*

---

**최종 수정일**: 2026-03-09
