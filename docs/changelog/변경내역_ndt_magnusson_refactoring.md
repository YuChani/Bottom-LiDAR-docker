# NDT Factor 코드 변경내역 (Magnusson 논문 기반 리팩토링)

> **대상 파일**: `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp`  
> **참조 논문**: Magnusson, "The Three-Dimensional Normal-Distributions Transform" (2009, Örebro University)  
> **작업 일자**: 2026년 2월

---

## 1. 변경 요약

| # | 변경 항목 | 변경 전 | 변경 후 | 근거 |
|---|----------|---------|---------|------|
| 1 | `e_scaled` 가드 제거 | `gauss_d2 * e_term > 1.0`이면 `return 0.0` | `std::isnan(e_term)`일 때만 `return 0.0` | 버그: well-aligned 포인트 필터링 |
| 2 | `error` → `score_function` + `cost` 분리 | `error = -d1 * (1 - e_term)` | `score_function = -d1 * e_term` / `cost = -d1 - score_function` | Eq. 6.9 명시적 표현 |
| 3 | `derivative_scale` → `weight` 리네이밍 | `derivative_scale` | `weight` | Eq. 6.12의 의미 명확화 |
| 4 | 수학 주석 추가 | 주석 없음 | Eq. 6.9, 6.12, 6.13 체인 주석 | 코드-수식 매핑 |

---

## 2. 상세 변경 내용

### 2.1 버그 수정: `e_scaled` 가드 제거

**문제**: `gauss_d2 ≈ 5~8`이므로 `gauss_d2 * e_term > 1.0`은 `e_term > 0.13~0.2`일 때 발생한다. 즉 **Mahalanobis 거리가 작은(= 정렬이 잘 된) 포인트들이 필터링**되는 심각한 버그였다.

```cpp
// ❌ 변경 전: well-aligned 포인트가 제거됨
double e_scaled = gauss_d2 * e_term;
if (e_scaled > 1.0 || std::isnan(e_scaled)) {
  return 0.0;
}

// ✅ 변경 후: exponent < -700 가드로 underflow 방지, NaN만 체크
if (exponent < -700.0) {
  return 0.0;  // underflow 방지
}
double e_term = std::exp(exponent);
if (std::isnan(e_term)) {
  return 0.0;
}
```

**Magnusson 논문 근거**: Eq. 6.9에서 `e_term`은 `exp(-d2/2 * m)` 형태이며, 어떤 값이든 score에 기여해야 한다. `d2`를 곱해서 필터링하는 것은 논문에 없는 로직이다.

---

### 2.2 `score_function`과 `cost` 분리

**문제**: 기존 `error = -d1 * (1 - e_term)`이라는 변수명은 Magnusson Eq. 6.9의 score function과의 관계가 불명확했다.

```cpp
// ❌ 변경 전
const double error = -gauss_d1 * (1.0 - e_term);

// ✅ 변경 후
const double score_function = -gauss_d1 * e_term;       // Magnusson Eq. 6.9
const double cost = -gauss_d1 - score_function;          // = -d1 * (1 - e_term), GTSAM 호환
```

**설계 의도**:
- `score_function`: Magnusson Eq. 6.9 원본. 정렬이 좋을수록 **증가** (최대화 문제)
- `cost`: GTSAM LM은 cost가 **감소**해야 step을 accept하므로, 상수 `-d1`에서 score를 빼서 부호를 반전
- 상수 차이이므로 gradient와 Hessian은 동일 → 최적화 동작에 영향 없음

---

### 2.3 `derivative_scale` → `weight` 리네이밍

**문제**: `derivative_scale`이라는 변수명만으로는 이 값이 Magnusson Eq. 6.12에서 score function의 1차 미분으로부터 자연스럽게 나오는 weight라는 사실이 드러나지 않았다.

```cpp
// ❌ 변경 전
const double derivative_scale = -gauss_d1 * gauss_d2 * e_term;

// ✅ 변경 후
const double weight = -gauss_d1 * gauss_d2 * e_term;  // 양수 스칼라
```

**Eq. 6.12 유도 과정**:
```
∂s/∂p = (-d1) * exp(-d2/2 * q^T Σ^{-1} q) * (-d2) * q^T * Σ^{-1} * J
       = (-d1 * d2 * e_term) * q^T * Σ^{-1} * J
         ^^^^^^^^^^^^^^^^^^
         weight (양수: d1 < 0, d2 > 0, e_term > 0)
```

이 weight는 Magnusson의 Eq. 6.13 H1 항에서 Gauss-Newton Hessian 근사로 직결된다:
```
H ≈ weight * J^T * Σ^{-1} * J
```

---

### 2.4 수학 주석 체계

코드의 각 핵심 구간에 Magnusson 논문 수식 번호를 명시하여 코드-수식 매핑을 명확히 했다:

```
// ========== Magnusson 2009, Eq. 6.9: NDT Score Function ==========
//   score = -d1 * exp(-d2/2 * q^T Σ^{-1} q)

// ========== Magnusson 2009, Eq. 6.12: Gradient ==========
//   ∂s/∂p = weight * q^T * Σ^{-1} * J

// ========== Gauss-Newton 근사 Hessian (Eq. 6.13의 H1 항) ==========
//   H ≈ weight * J^T * Σ^{-1} * J
//   GTSAM LM 시스템: (H + λI)δ = -b
```

---

## 3. 수학적 체인 요약

```
Score Function (Eq. 6.9)
  s = -d1 * exp(-d2/2 * m)      ← score_function 변수
      │
      ▼ 1차 미분 (Eq. 6.12)
  ∂s/∂p = weight * J^T * Σ^{-1} * q    ← b (gradient vector)
      │     where weight = -d1 * d2 * e_term
      ▼ GN 근사 (Eq. 6.13 H1)
  H ≈ weight * J^T * Σ^{-1} * J        ← H (Hessian matrix)
      │
      ▼ GTSAM LM
  (H + λI)δ = -b                        ← 최적화 step 계산
```

---

## 4. 영향 범위

- **변경 파일**: `integrated_ndt_factor_impl.hpp` (1개 파일)
- **빌드 영향**: `gtsam_points` 라이브러리와 `lidar_registration_benchmark` 바이너리 재빌드 필요
- **동작 변경**: `e_scaled` 가드 제거로 인해 이전에 필터링되던 well-aligned 포인트들이 이제 정상적으로 기여함 → NDT 정합 정확도 향상 기대
- **API 변경**: 없음 (내부 구현만 변경)

---

## 5. 관련 문서

| 문서 | 위치 | 내용 |
|------|------|------|
| NDT 구현 비교 분석 | `docs/ndt/magnusson_implementation_comparison_ko.md` | gtsam_points NDT vs Magnusson 논문 상세 비교 |
| Weighted GN 적용 가능성 | `docs/ndt/weighted_gauss_newton_feasibility_ko.md` | Eq. 6.12/6.13 기반 weighted GN의 GTSAM 적용 분석 |
