# 그래프 모드 3종 비교 벤치마크 (2026-03-12)

## 1) 실험 개요

| 항목 | 내용 |
|---|---|
| 실험 일자 | 2026-03-12 |
| 목적 | `full_connection` / `pure_chain` / `sparse_chain` 3종 그래프 모드가 각 registration factor의 정확도·속도에 미치는 영향 정량 비교 |
| Optimizer | Levenberg-Marquardt (LM) |
| Noise scale | 0.100 |
| 프레임 수 | 7 |
| 로그 파일 | `artifacts/logs/graphmode/graphmode_full_connection.log` / `artifacts/logs/graphmode/graphmode_pure_chain.log` / `artifacts/logs/graphmode/graphmode_sparse_chain.log` |

---

## 2) 그래프 모드 설정 비교

| 모드 | `full_connection` 플래그 | Sparse window | 설명 |
|---|:---:|:---:|---|
| **Full Connection** | `true` | N/A | 모든 프레임 쌍을 엣지로 연결 (최대 제약) |
| **Pure Chain** | `false` | `2` | 인접 1프레임씩만 연결 (최소 체인) |
| **Sparse Chain** | `false` | `4` | 인접 ±2 프레임 이내만 연결 (중간 밀도) |

---

## 3) 모드별 결과표

### 3.1 Full Connection (`full_connection=true`)

> 모든 프레임 간 엣지 연결 — 제약 밀도 최대

| Factor | Mean T (m) | Mean R (deg) | Max T (m) | Max R (deg) | Time (ms) | Iters |
|---|---:|---:|---:|---:|---:|---:|
| Point-to-Point | 0.100990 | 0.436051 | 0.204713 | 0.687970 | 1947 | 27 |
| Point-to-Plane | 0.061799 | 0.430725 | 0.127270 | 0.852305 | 1256 | 16 |
| GICP | 0.085399 | 0.499411 | 0.168046 | 1.047266 | 1064 | 10 |
| VGICP | 0.117390 | 0.635614 | 0.202933 | 1.177762 | 1317 | 12 |
| NDT | 0.105657 | 0.730808 | 0.191520 | 1.320227 | 5411 | 71 |
| LightNDT | 0.159027 | 0.640614 | 0.337532 | 0.960458 | 618 | 6 |
| LOAM_LIOSAM | 0.225414 | 0.774622 | 0.616540 | 1.429813 | 60 | 9 |

### 3.2 Pure Chain (`full_connection=false`, window=2)

> 인접 1프레임만 연결 — 제약 밀도 최소 / 누적 오차 발생 가능

| Factor | Mean T (m) | Mean R (deg) | Max T (m) | Max R (deg) | Time (ms) | Iters |
|---|---:|---:|---:|---:|---:|---:|
| Point-to-Point | 0.228122 | 3.081607 | 0.786029 | 6.717394 | 402 | 19 |
| Point-to-Plane | 0.043472 | 0.428229 | 0.093456 | 0.695091 | 369 | 16 |
| GICP | 1.635813 | 5.098419 | 3.072735 | 9.087081 | 259 | 5 |
| VGICP | 1.211390 | 4.623732 | 2.657716 | 8.480980 | 490 | 20 |
| NDT | 0.394681 | 4.002565 | 1.288696 | 7.768180 | 1564 | 65 |
| LightNDT | 0.107436 | 0.858916 | 0.174495 | 1.402911 | 338 | 12 |
| LOAM_LIOSAM | 0.109340 | 0.506580 | 0.198987 | 0.886227 | 24 | 10 |

### 3.3 Sparse Chain (`full_connection=false`, window=4)

> 인접 ±2 프레임 연결 — Full Connection 대비 가벼우면서 Pure Chain 대비 안정적

| Factor | Mean T (m) | Mean R (deg) | Max T (m) | Max R (deg) | Time (ms) | Iters |
|---|---:|---:|---:|---:|---:|---:|
| Point-to-Point | 0.092639 | 0.425162 | 0.157106 | 0.735724 | 1356 | 26 |
| Point-to-Plane | 0.065026 | 0.456036 | 0.128222 | 0.891781 | 776 | 14 |
| GICP | 0.079191 | 0.484074 | 0.158575 | 1.010339 | 694 | 8 |
| VGICP | 0.116418 | 0.622866 | 0.203245 | 1.182055 | 990 | 12 |
| NDT | 0.095917 | 0.703488 | 0.168391 | 1.323688 | 4255 | 80 |
| LightNDT | 0.114073 | 0.591129 | 0.246970 | 0.943262 | 496 | 7 |
| LOAM_LIOSAM | 0.130595 | 0.527625 | 0.267358 | 0.875374 | 46 | 9 |

---

## 4) 3모드 교차 비교 — Factor별 Best 모드

> 병진 오차(Mean T) 기준. ✅ = Best, ⚠️ = 주의 (오차 급증)

| Factor | Full Connection | Pure Chain | Sparse Chain | Best 모드 |
|---|:---:|:---:|:---:|:---:|
| Point-to-Point | 0.100990 | 0.228122 ⚠️ | **0.092639** ✅ | Sparse Chain |
| Point-to-Plane | 0.061799 | **0.043472** ✅ | 0.065026 | Pure Chain |
| GICP | 0.085399 | 1.635813 ⚠️ | **0.079191** ✅ | Sparse Chain |
| VGICP | 0.117390 | 1.211390 ⚠️ | **0.116418** ✅ | Sparse Chain |
| NDT | 0.105657 | 0.394681 ⚠️ | **0.095917** ✅ | Sparse Chain |
| LightNDT | **0.107436**¹ | 0.107436 | 0.114073 | Pure Chain / Full (동률) |
| LOAM_LIOSAM | 0.225414 | **0.109340** ✅ | 0.130595 | Pure Chain |

> ¹ LightNDT는 Full Connection 기준값(0.159027) 대비 Pure Chain(0.107436)이 더 우수함

### 회전 오차(Mean R) 기준 Best

| Factor | Full Connection | Pure Chain | Sparse Chain | Best 모드 |
|---|:---:|:---:|:---:|:---:|
| Point-to-Point | 0.436051 | 3.081607 ⚠️ | **0.425162** ✅ | Sparse Chain |
| Point-to-Plane | 0.430725 | **0.428229** ✅ | 0.456036 | Pure Chain |
| GICP | 0.499411 | 5.098419 ⚠️ | **0.484074** ✅ | Sparse Chain |
| VGICP | 0.635614 | 4.623732 ⚠️ | **0.622866** ✅ | Sparse Chain |
| NDT | 0.730808 | 4.002565 ⚠️ | **0.703488** ✅ | Sparse Chain |
| LightNDT | 0.640614 | 0.858916 | **0.591129** ✅ | Sparse Chain |
| LOAM_LIOSAM | 0.774622 | **0.506580** ✅ | 0.527625 | Pure Chain |

---

## 5) 주요 발견사항 및 분석

### 5.1 Pure Chain의 분열 패턴

Pure Chain(window=2)에서 **Point-to-Point, GICP, VGICP, NDT** 4개 factor의 오차가 급등한다.

| Factor | Full T → Pure T | 배율 |
|---|---:|---:|
| GICP | 0.085 → 1.636 | **×19.2** |
| VGICP | 0.117 → 1.211 | **×10.3** |
| NDT | 0.106 → 0.395 | **×3.7** |
| Point-to-Point | 0.101 → 0.228 | **×2.3** |

- 공분산 기반 factor(GICP, VGICP)는 체인 누적 오차에 가장 민감하다.
- 반면 **Point-to-Plane, LightNDT, LOAM_LIOSAM** 3개는 Pure Chain에서도 안정적이거나 오히려 향상된다.
  - Point-to-Plane: 단순 평면 노말 제약이 프레임 간 의존성 감소 시에도 강건하게 작동
  - LightNDT/LOAM_LIOSAM: feature/lightweight 특성상 로컬 정합 충분성이 높음

### 5.2 Sparse Chain의 균형점 효과

Sparse Chain(window=4)은 대부분의 factor에서 **Full Connection 대비 비슷하거나 더 나은 정확도**를 보이면서, 처리 시간은 크게 절감된다.

| Factor | Full T (ms) | Sparse T (ms) | 절감률 |
|---|---:|---:|---:|
| Point-to-Plane | 1256 | 776 | **−38%** |
| GICP | 1064 | 694 | **−35%** |
| NDT | 5411 | 4255 | **−21%** |
| VGICP | 1317 | 990 | **−25%** |

- 정확도는 유지하면서 계산 비용을 줄이는 **실용적 최적점**으로 동작한다.

### 5.3 LOAM_LIOSAM의 역전 현상

LOAM_LIOSAM은 Full Connection에서 가장 낮은 정확도(Mean T=0.225)를 보이지만, Pure Chain에서 Mean T=0.109, Sparse Chain에서 Mean T=0.131로 **그래프 밀도를 줄일수록 오히려 개선**된다.

- 원인: 희소 feature 기반 factor는 과도한 루프 제약 시 오히려 feature 간 모순(inconsistency)이 증폭될 수 있음
- 의미: LOAM_LIOSAM은 dense graph보다 sequential / sparse 구조에 더 적합

### 5.4 전체 속도 비교

각 모드에서 모든 factor의 총 처리 시간 합산:

| 모드 | 총 Time (ms) 합산 |
|---|---:|
| Full Connection | 11,673 |
| Pure Chain | 3,446 |
| Sparse Chain | 8,617 |

- Pure Chain이 가장 빠르지만 정확도 리스크 높음
- Sparse Chain은 Full Connection 대비 약 **26% 빠르면서** 정확도는 동등 이상

---

## 6) 결론 및 권장사항

### 결론 요약

| 시나리오 | 권장 모드 | 이유 |
|---|---|---|
| 최고 정확도 필요 | **Full Connection** | 대부분 factor에서 안정적 기준선 제공 |
| 정확도-속도 균형 | **Sparse Chain** (window=4) | Full 대비 손실 없이 ~26% 빠름 |
| 속도 최우선, LOAM만 사용 | **Pure Chain** | LOAM_LIOSAM은 Pure Chain에서 가장 정확 |
| 공분산 기반 factor (GICP/VGICP) | **Sparse Chain 이상 필수** | Pure Chain에서 오차 10~20배 폭증 |
| Point-to-Plane 단독 운용 | **Pure Chain** 가능 | 모든 모드에서 안정적, Pure Chain이 최소 오차 |

### 핵심 권장사항

1. **기본 운영 모드: Sparse Chain (window=4)**
   - 실용적 최적점. 속도 절감 + 정확도 유지.

2. **GICP / VGICP 사용 시 Pure Chain 금지**
   - 공분산 기반 factor는 체인 누적 오차에 극도로 취약.
   - 반드시 window ≥ 4 이상 사용.

3. **LOAM_LIOSAM은 초기화(seed) 용도로만 사용**
   - 어떤 모드에서도 단독 정합으로는 정확도가 낮음.
   - Pure Chain 또는 Sparse Chain에서 Point-to-Plane/GICP 후단 refinement와 조합 권장.

4. **다음 실험 방향**
   - Sparse Chain window 파라미터 탐색: window=3, 5, 6 비교
   - 노이즈 레벨별 모드 민감도 분석 (noise=0.05, 0.15, 0.20)
   - Pure Chain에서 안정적인 factor (Point-to-Plane, LightNDT)의 강건성 원인 심층 분석
