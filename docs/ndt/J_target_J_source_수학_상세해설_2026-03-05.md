# J_target vs J_source 수학 상세 해설 (2026-03-05)

## 1. 목적

이 문서는 `gtsam_points` 계열 factor에서 반복적으로 등장하는 아래 두 Jacobian의 차이를 수학적으로 설명한다.

- `J_target = ∂r / ∂xi_target`
- `J_source = ∂r / ∂xi_source`

핵심 질문은 다음이다.

1) 왜 두 Jacobian의 부호가 다르게 나오나?
2) 왜 source 쪽에는 `R`이 곱해지나?
3) 코드의 블록 식이 어떤 선형화 가정에서 나왔나?

## 2. 코드 기준 정의

### 2.1 상대변환 정의

`IntegratedMatchingCostFactor`는 아래를 사용한다.

- `delta = T_target^{-1} * T_source` (`thirdparty/gtsam_points/src/gtsam_points/factors/integrated_matching_cost_factor.cpp:64`)

즉 `delta`는 source 점을 target 좌표계로 보내는 변환이다 (`T_target_source`).

### 2.2 residual 정의

NDT/ICP/GICP/VGICP/LightNDT는 공통적으로

- `q = delta * a` (source 점 `a`를 target 좌표계로 변환)
- `r = b - q` (target 점 `b`와 변환된 source 점의 차)

를 사용한다. 예:

- `r = mean_B - transed_mean_A` (`thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:242`)

### 2.3 Jacobian 코드 형태

공통 패턴:

- `J_target.block<3,3>(0,0) = -Hat(q)`
- `J_target.block<3,3>(0,3) = I`
- `J_source.block<3,3>(0,0) = R * Hat(a)`
- `J_source.block<3,3>(0,3) = -R`

예:

- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:272`
- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:277`

동일한 블록 구조가 ICP/GICP/VGICP/LightNDT/LOAM에도 반복된다.

- ICP: `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_icp_factor_impl.hpp:220`
- GICP: `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_gicp_factor_impl.hpp:271`
- VGICP: `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_vgicp_factor_impl.hpp:313`
- LightNDT: `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_light_ndt_factor_impl.hpp:233`
- LOAM: `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_loam_factor_impl.hpp:180`

## 3. 선형화 가정과 기호

아래 기호를 사용한다.

- `a`: source frame의 점 (동차 4D에서 공간부 3D)
- `b`: target frame의 대응점
- `delta = [R, t]`, `q = R a + t`
- residual: `r = b - q`
- 소규모 증분: `xi = [phi, rho]` (회전 3 + 병진 3)
- `Hat(x) y = x × y`

## 4. J_target 유도

`delta = T_t^{-1} T_s`에서 target 포즈에 작은 증분 `xi_t`가 들어가면 1차 근사에서

- `delta' ≈ Exp(-xi_t^) delta`

이고,

- `q' = delta' a ≈ q - (phi_t × q + rho_t)`

따라서

- `r' = b - q' = r + (phi_t × q + rho_t)`

이므로

- `∂r/∂phi_t = -Hat(q)`
- `∂r/∂rho_t = I`

즉,

`J_target = [ -Hat(q), I ]`

가 된다. 코드와 동일하다.

## 5. J_source 유도

source 포즈에 작은 증분 `xi_s`가 들어가면

- `delta' ≈ delta Exp(xi_s^)`

이고

- `a' ≈ a + (phi_s × a + rho_s)`
- `q' = delta a' ≈ q + R(phi_s × a + rho_s)`

따라서

- `r' = b - q' = r - R(phi_s × a + rho_s)`

여기서 `phi × a = -Hat(a) phi`를 쓰면

- `-R(phi_s × a) = R Hat(a) phi_s`
- `-R rho_s = -R rho_s`

즉,

- `∂r/∂phi_s = R Hat(a)`
- `∂r/∂rho_s = -R`

정리하면

`J_source = [ R Hat(a), -R ]`

가 된다. 역시 코드와 동일하다.

## 6. 직관: 왜 두 Jacobian이 다르게 생기나?

1) **target 쪽 증분**은 `delta = T_t^{-1}T_s`에서 역변환을 통해 들어와서 부호가 반전된다.

2) **source 쪽 증분**은 source 로컬 좌표에서 발생한 변화가 target 좌표로 옮겨져야 하므로 `R`이 곱해진다.

3) residual이 `r = b - q`이므로, `q`에 들어간 증분은 최종 residual에서 한 번 더 부호 반전을 겪는다.

이 세 가지가 합쳐져 `J_target`과 `J_source`의 블록 부호/회전 계수가 달라진다.

## 7. Hessian/gradient 누적에서의 역할 차이

각 factor는 일반적으로 아래를 누적한다.

- `H_tt += J_target^T W J_target`
- `H_ss += J_source^T W J_source`
- `H_ts += J_target^T W J_source`
- `b_t  += J_target^T W r`
- `b_s  += J_source^T W r`

NDT 예시는 `W = weight * inv_cov` 구조다.

- `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:284`

즉 두 Jacobian은 같은 residual을 보되, 서로 다른 변수(target/source)의 접공간 방향을 나타낸다.

## 8. 4x6과 3x6 관계

현재 구현은 점/노멀/공분산을 homogeneous 4D로 보관한다.

- points `(x,y,z,1)`, normals `(nx,ny,nz,0)` (`thirdparty/gtsam_points/include/gtsam_points/types/point_cloud.hpp:106`)

그래서 Jacobian도 `4x6`을 기본으로 통일한다. 다만 기하적으로 유효한 축은 주로 상위 3차원이며, 일관되게 축소하면 `3x6` 표현과 동치가 될 수 있다.

## 9. 참고 문헌

- Barfoot, T. D., *State Estimation for Robotics*, 2017.
- GTSAM `Pose3::transformFrom` 계열 left-perturbation Jacobian 관례.
- 코드 레퍼런스:
  - `thirdparty/gtsam_points/src/gtsam_points/factors/integrated_matching_cost_factor.cpp:64`
  - `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_ndt_factor_impl.hpp:272`
  - `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_icp_factor_impl.hpp:220`
  - `thirdparty/gtsam_points/include/gtsam_points/factors/impl/integrated_gicp_factor_impl.hpp:271`
