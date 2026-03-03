# main.cpp 변경내역: 기본 Thread 수 자동 설정

> **작업일**: 2026-03-03  
> **대상 파일**: `src/main.cpp`  
> **실행 환경**: Docker container `bottom-lidar` (`/root/workdir/build`)  
> **목적**: 기본값이 1로 고정된 thread 수를 CPU 코어 수 기반으로 자동 설정하여 headless 벤치마크 실행 시간을 단축

---

## 1. 변경 요약

| 구분 | 변경 전 | 변경 후 | 비고 |
|---|---|---|---|
| 기본 thread 값 | `num_threads = 1;` | `num_threads = std::max(1u, std::thread::hardware_concurrency());` | 런타임 기본 병렬도 자동 적용 |

### 코드 변경 위치

- `src/main.cpp:360`

```cpp
// before
num_threads = 1;

// after
num_threads = std::max(1u, std::thread::hardware_concurrency());
```

---

## 2. 변경 의도

- 기존 기본값(1 thread)은 NDT/GICP/VGICP처럼 per-point 계산량이 큰 factor에서 병렬 이점을 거의 활용하지 못함.
- 이번 변경은 알고리즘/그래프 구조를 바꾸지 않고, 기본 실행 성능만 개선하는 런타임 파라미터 튜닝.
- UI의 `num threads` 수동 조절 기능은 그대로 유지됨 (`ImGui::DragInt("num threads", ...)`).

---

## 3. 검증 절차

아래 순서로 컨테이너 내부에서 검증:

```bash
cd /root/workdir/build
cmake --build . -j$(nproc)
./lidar_registration_benchmark --headless
```

- 빌드: 성공
- 실행: 성공
- 결과: 최종 비교표 출력 확인

---

## 4. 성능 비교 (관측값)

아래는 변경 전(기본 1 thread) 대비 변경 후(자동 thread)의 headless 실행 결과 비교.

| Factor | 변경 전 ms | 변경 후 ms | 개선 배율(전/후) |
|---|---:|---:|---:|
| Point-to-Point | 19308 | 8437 | 2.29x |
| Point-to-Plane | 9628 | 1668 | 5.77x |
| GICP | 11801 | 2571 | 4.59x |
| VGICP | 8498 | 2958 | 2.87x |
| NDT | 20536 | 5309 | 3.87x |
| LOAM_LIOSAM | 133 | 225 | 0.59x (증가) |

### 관찰 포인트

- 전체적으로 대부분 factor에서 최적화 시간이 크게 감소.
- NDT는 `20536ms -> 5309ms`로 큰 폭 개선.
- LOAM_LIOSAM은 해당 실행에서 오히려 증가했으나, 절대 시간은 여전히 매우 짧은 수준.

---

## 5. 영향도 및 리스크

### 영향도

- **구조 변경 없음**: 그래프 생성 로직, factor 수식, 최적화 흐름은 동일.
- **런타임 파라미터 변경만 존재**: 기본 thread 수 초기값만 변경.

### 리스크/주의사항

- 병렬 합산 순서 차이로 인해 부동소수점 오차 수준의 미세 수치 차이가 발생할 수 있음.
- 환경에 따라(코어 수, OpenMP/TBB 설정) 체감 개선 폭은 달라질 수 있음.
- `std::thread::hardware_concurrency()`가 0을 반환할 경우를 대비해 `std::max(1u, ...)`로 하한 보장.

---

## 6. 결론

- 이번 변경은 코드 구조를 건드리지 않고 기본 성능을 끌어올리는 안전한 개선.
- 특히 NDT와 GICP 계열에서 유의미한 시간 단축이 확인됨.
- 이후 추가 최적화는 `full_connection`, `NDT search mode`, `correspondence tolerance` 튜닝과 병행 시 효과가 더 커질 수 있음.
