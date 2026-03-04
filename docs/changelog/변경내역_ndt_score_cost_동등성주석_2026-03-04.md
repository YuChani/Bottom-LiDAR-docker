# NDT score/cost 동등성 주석 정리 변경내역

> 작업일: 2026-03-04  
> 대상: `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp`

## 변경 배경

- `score_function`과 `cost`가 "둘 다 같은 값인지"에 대한 해석 혼동을 줄이기 위해 주석을 명확화했다.
- 핵심은 두 식이 완전히 동일한 값은 아니지만, **상수항 차이만 있는 동등 표현**이라는 점이다.

## 변경 내용

- `integrated_ndt_factor_impl.hpp`의 `score_function`/`cost` 계산 블록 주석을 정리했다.
- 명시한 수식:

```text
cost = -d1 - score_function = -d1 * (1 - e_term)
```

- 명시한 의미:
  - `cost`와 `-score_function`은 상수항 `(-d1)` 차이만 존재
  - 따라서 최적점과 gradient/Hessian은 동일

## 검증

아래를 Docker 컨테이너에서 재실행하여 동작 이상이 없는지 확인했다.

```bash
cd /root/workdir/build
cmake --build . -j$(nproc)
ctest --output-on-failure
./lidar_registration_benchmark --headless
```

- 빌드: 성공
- CTest: 등록 테스트 없음 (`No tests were found!!!`)
- Headless benchmark: 정상 완료, 최종 비교표 출력 확인

## 영향도

- 코드 로직 변화 없음(주석/표현 명확화)
- 수치 결과나 최적화 경로의 의도된 변경 없음
