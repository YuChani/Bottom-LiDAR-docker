# NDT Hessian H1/H2/H3 항 수학적 분석

**작성일**: 2026-03-09  
**상태**: 분석 완료, 코드 현재 H1-only 원복 상태  
**관련 문서**: [NDT_Conditional_H2_최종보고서_2026-03-09.md](../experiments/NDT_Conditional_H2_최종보고서_2026-03-09.md)

---

## 1. 개요

NDT 비용 함수의 Hessian은 수학적으로 세 항(H1, H2, H3)으로 분해된다.
현재 구현은 H1만 사용하며, 이것이 달팽이 수렴(slow convergence)의 핵심 원인 중 하나이다.

---

## 2. 비용 함수 정의

```
E(ξ) = -d1 * (1 - exp(-d2/2 * m))

여기서:
  ξ   = SE(3) 6DOF 변환 파라미터 (Lie algebra)
  r   = mean_B - T(ξ) * mean_A  (잔차 벡터, 4D 동차좌표)
  m   = r^T Σ^{-1} r             (마할라노비스 거리)
  Σ^{-1} = 정규화된 역공분산
  d1 < 0, d2 > 0                 (Magnusson 2009 Eq.6.9-6.10)
```

Gradient:
```
∂E/∂ξ = weight * J^T Σ^{-1} r

  weight = -d1 * d2 * exp(-d2/2 * m) > 0  (항상 양수)
  J = ∂r/∂ξ  (4×6 야코비안)
```

---

## 3. Hessian 완전 분해

비용 함수를 ξ에 대해 두 번 미분하면 세 항이 나온다:

```
H_full = ∂²E/∂ξ² = H1 + H2 + H3
```

### 3.1 H1 — Gauss-Newton 근사 항

```
H1 = weight * J^T Σ^{-1} J
```

**수학적 의미**:
- 잔차의 1차 선형화에 기반한 Gauss-Newton 근사
- 정보 행렬(Fisher information)과 동형: J^T W J 구조
- `weight > 0` 이므로 **항상 Positive Semi-Definite (PSD)**

**GTSAM 호환성**: ✅ Cholesky factorization에 안전  
**단점**: 비용 함수의 비선형 곡률 정보 누락 → 보수적 step → 느린 수렴

---

### 3.2 H2 — 지수 평탄화 보정 항

```
H2 = -weight * d2 * v * v^T

여기서 v = J^T Σ^{-1} r  (6×1 벡터, gradient의 핵심 항)
```

**수학적 의미**:
- `1 - exp(...)` 형태의 비용 함수에서 지수 항의 2차 곡률로부터 유도
- rank-1 outer product (`v * v^T`) 형태
- `weight > 0`, `d2 > 0` → H2는 **항상 Negative Semi-Definite (NSD)**

**물리적 효과**: Newton step을 더 크게 만들어 수렴 가속
- H1만 사용 시: step = H1^{-1} b → 보수적
- H1+H2 사용 시: H2가 H1의 eigen direction을 "확장" → 더 큰 step

**부정치(Indefinite) 조건**:
```
H1 + H2가 indefinite가 되는 조건:
  d2 * v^T H1^{-1} v > 1
  
  즉, d2 * r^T Σ^{-1} J H1^{-1} J^T Σ^{-1} r > 1
```
- 잔차 r이 크고 J가 잘 조건화(well-conditioned)될 때 indefinite 발생
- 정렬이 좋을 때 (weight가 클 때, m이 작을 때): r이 작으므로 v도 작아 조건 불만족 → PSD 유지
- **이것이 Conditional H2 (`weight > 0.01`)가 동작하는 이유**: 정렬이 좋은 포인트에만 H2 적용

**GTSAM 호환성**: ⚠️ 조건부. indefinite 시 Cholesky 음수 pivot → λ 폭발

---

### 3.3 H3 — 회전 곡률 항

```
H3_{jk} = weight * (∂J_j/∂ξ_k)^T Σ^{-1} r  (SE(3) 회전 2차 도함수)
```

**수학적 의미**:
- SO(3) 회전군의 비선형 곡률에서 발생
- Lie algebra Hat 연산자의 2차 미분: `∂(ω×p)/∂ω = -p×`의 추가 미분
- 잔차 r에 **선형 비례** → 정렬이 좋아질수록 H3 ≈ 0

**왜 생략하는가**:
1. 수렴 근방에서 무시 가능 (r → 0일 때 H3 → 0)
2. 부호가 ± 혼재 → PSD 보장 불가
3. 계산 비용 높음 (6×6 행렬 연산 추가)
4. **Magnusson 2009 논문도 생략 권장**

**GTSAM 호환성**: ❌ PSD 불보장, 사실상 사용 불가

---

## 4. 각 항의 비교 요약

| 항 | 수식 | 부호 보장 | 수렴 효과 | GTSAM 호환 | 사용 여부 |
|----|------|-----------|-----------|------------|----------|
| H1 | `w * J^T Σ^{-1} J` | 항상 PSD | 보수적 (느림) | ✅ 안전 | **현재 사용** |
| H2 | `-w * d2 * v v^T` | 항상 NSD | 가속 (큰 step) | ⚠️ 조건부 | 실험적 |
| H3 | `w * (∂J/∂ξ) Σ^{-1} r` | 불보장 | 미미 | ❌ 위험 | 생략 |

---

## 5. GTSAM LM과의 상호작용

### 5.1 Indefinite Hessian 발생 시 LM 동작

```
LM 업데이트: (H + λI) δξ = b

H가 indefinite → Cholesky 음수 pivot 감지
→ λ를 급격히 증가 (1e-6 → 10000까지 폭발)
→ λI가 dominant → 사실상 gradient descent로 퇴화
→ tiny step → 수렴 극히 느림
```

### 5.2 Full H2 실험 결과 (이전 세션)

- iter: 32회 (H1-only 74회 대비 57% 감소)  
- 시간: 45% **증가** (2523ms → 3659ms)
- **원인**: 일부 iteration에서 indefinite 발생 → λ 폭발 → iter당 시간 급증

### 5.3 Conditional H2 실험 결과 (`weight > 0.01`)

- iter: 25회 (67.6% 감소)
- 시간: 12.4% 증가 (2523ms → 2835ms)
- **분석**: weight > 0.01 조건이 r이 작은(정렬 좋은) 포인트만 선별 → indefinite 위험 회피

---

## 6. NDT 달팽이 수렴 패턴 확인 (2026-03-09 실측)

H1-only 원복 상태에서의 수렴 궤적:

```
iter  0: error = 1,602,940
iter 10: error = 1,444,200  → 10iter에 ~10% 감소
iter 30: error = 1,204,950  → 30iter에 ~25% 감소
iter 70: error = 1,003,160  → 70iter에도 겨우 ~37% 감소
```

- 매 iter cost_d ≈ 10,000 감소
- 초기 error 1.6M → 100+ iter 필요한 구조
- exponential weight decay (`exp(-d2/2 * m)`) + H1-only = flat landscape 패턴

**원인**: H1은 Gauss-Newton이므로 exponential cost의 비선형 곡률을 무시.
비용 함수가 가우시안 종 모양(bell shape)이라 0 근방이 매우 평평함.
H2가 있으면 이 곡률을 보정해 더 큰 step을 밟을 수 있음.

---

## 7. 전체 벤치마크 결과 (2026-03-09 실측)

H1-only 원복 상태:

| Factor | Outer Iter | Time (ms) | Mean Trans (m) | Mean Rot (deg) |
|--------|:----------:|:---------:|:--------------:|:--------------:|
| Point-to-Plane | 15 | **915** | 0.0618 | 0.4307 |
| LOAM_LIOSAM | 8 | 169 | 0.225 | 0.775 |
| GICP | 9 | 1090 | 0.0854 | 0.499 |
| Point-to-Point | 26 | 1458 | 0.101 | 0.436 |
| LightNDT | 6† | 792 | 0.159 | 0.641 |
| VGICP | 12† | 1753 | 0.117 | 0.636 |
| **NDT** | **70** | **8006** | **0.106** | **0.731** |

† LightNDT/VGICP: cost_d 음수 → λ 폭발(→10000) 후 강제 종료  
※ NDT 8006ms는 iter 15-17 시스템 스파이크(lin_msec 295ms×3) 이상치. 정상 성능 ~2500ms / 74iter

---

## 8. 다음 단계 옵션

| 옵션 | 설명 | 예상 효과 |
|------|------|-----------|
| **A: Conditional H2 재적용** | `weight > 0.01` 조건 H2 적용 | iter 67.6%↓, 시간 12.4%↑ |
| **B: Iteration scheduling** | 초반 N회만 H2, 이후 H1 전환 | iter 감소 + 후반 λ 폭발 방지 |
| **C: H2 threshold 조정** | `weight > 0.01` → 더 높은 값 | indefinite 위험 더 낮춤 |
| **D: Adaptive threshold** | m (마할라노비스) 기준으로 H2 적용 | 더 정확한 조건 제어 |
| **E: 다른 접근** | LM → GN 교체, 초기값 개선 등 | 검토 필요 |

---

## 9. 결론

- **H1**: 항상 PSD, GTSAM 안전, 단 비선형 곡률 무시 → 느린 수렴
- **H2**: 수렴 가속 효과 있으나 indefinite 위험 존재 → Conditional 적용이 현실적
- **H3**: 구현 복잡, 효과 미미, 부호 불안정 → 생략이 정답
- **핵심 병목**: H1-only + exponential cost의 flat landscape = NDT 달팽이 수렴의 수학적 원인

---

**참고**: Magnusson, M. (2009). *The Three-Dimensional Normal Distributions Transform.* Örebro University. Eq. 6.9-6.10, Section 6.3.
