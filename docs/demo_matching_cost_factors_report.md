# demo_matching_cost_factors.cpp 상세 분석 보고서

## 목차
1. [개요](#1-개요)
2. [핵심 개념](#2-핵심-개념)
3. [코드 구조](#3-코드-구조)
4. [클래스 상세 분석](#4-클래스-상세-분석)
5. [주요 변수 설명](#5-주요-변수-설명)
6. [Factor 타입별 상세 설명](#6-factor-타입별-상세-설명)
7. [최적화 알고리즘](#7-최적화-알고리즘)
8. [실행 흐름](#8-실행-흐름)
9. [튜토리얼: 단계별 동작 원리](#9-튜토리얼-단계별-동작-원리)
10. [참고 자료](#10-참고-자료)

---

## 1. 개요

### 1.1 파일 정보
- **파일 경로**: `/root/workdir/include/gtsam_points/src/demo/demo_matching_cost_factors.cpp`
- **목적**: 다양한 Point Cloud Matching Cost Factor들을 비교 시연하는 데모 프로그램
- **라이브러리**: gtsam_points (GTSAM 기반 포인트 클라우드 SLAM 라이브러리)

### 1.2 프로그램 기능
이 데모는 5개의 연속적인 LiDAR 스캔을 사용하여:
1. 다양한 ICP 변형 알고리즘 (ICP, Point-to-Plane ICP, GICP, VGICP) 비교
2. Levenberg-Marquardt와 iSAM2 옵티마이저 성능 비교
3. 실시간 시각화를 통한 최적화 과정 관찰
4. Ground Truth 대비 정확도 평가

### 1.3 의존성
```cpp
// GTSAM 핵심
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// gtsam_points Factor들
#include <gtsam_points/factors/integrated_icp_factor.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>

// 시각화 (iridescence)
#include <guik/viewer/light_viewer.hpp>
```

---

## 2. 핵심 개념

### 2.1 Factor Graph란?
Factor Graph는 확률적 추론을 위한 그래프 모델입니다:

```
[Node 0] ----Factor01---- [Node 1] ----Factor12---- [Node 2]
    |                         |                         |
    +--------Factor02---------+--------Factor13---------+
```

- **Node (변수)**: 최적화할 대상 (여기서는 각 스캔의 6DoF Pose)
- **Factor (제약조건)**: 변수들 간의 관계를 정의 (ICP 매칭 비용)

### 2.2 GTSAM 핵심 클래스

| 클래스 | 역할 |
|--------|------|
| `gtsam::Key` | 변수를 식별하는 정수 키 (0, 1, 2, ...) |
| `gtsam::Values` | Key-Value 쌍으로 변수들의 현재 추정값 저장 |
| `gtsam::Pose3` | SE(3) 6DoF 포즈 (위치 + 방향) |
| `gtsam::NonlinearFactorGraph` | Factor들의 집합 |

### 2.3 Matching Cost Factor 계층 구조

```
gtsam::NonlinearFactor (GTSAM 기본 클래스)
         │
         ▼
IntegratedMatchingCostFactor (gtsam_points 추상 클래스)
         │
         ├── IntegratedICPFactor (Point-to-Point ICP)
         │      └── IntegratedPointToPlaneICPFactor
         │
         ├── IntegratedGICPFactor (Generalized ICP)
         │
         └── IntegratedVGICPFactor (Voxelized GICP)
                └── IntegratedVGICPFactorGPU (GPU 가속)
```

---

## 3. 코드 구조

### 3.1 파일 구조
```
demo_matching_cost_factors.cpp
│
├── [라인 1-40] #include 및 전처리
│
├── [라인 41-460] class MatchingCostFactorDemo
│   │
│   ├── [라인 43-254] 생성자: 초기화 및 UI 설정
│   │   ├── [라인 47-74] Ground Truth 로드
│   │   ├── [라인 82-200] PCD 파일 로드 및 전처리
│   │   └── [라인 225-253] UI 콜백 등록
│   │
│   ├── [라인 263-292] update_viewer(): 시각화 업데이트
│   │
│   ├── [라인 294-341] create_factor(): Factor 생성 팩토리
│   │
│   ├── [라인 343-436] run_optimization(): 최적화 실행
│   │
│   └── [라인 439-459] 멤버 변수들
│
└── [라인 462-466] main(): 엔트리 포인트
```

### 3.2 주요 함수 흐름

```
main()
  │
  ▼
MatchingCostFactorDemo 생성자
  │
  ├── PCD 파일 로드 → PointCloud 생성
  ├── Covariance/Normal 추정
  ├── VoxelMap 생성
  └── UI 콜백 등록
       │
       ▼
  [사용자 "optimize" 버튼 클릭]
       │
       ▼
  run_optimization()
       │
       ├── NonlinearFactorGraph 구성
       ├── create_factor() 호출 (각 프레임 쌍마다)
       ├── LM 또는 iSAM2 최적화 실행
       └── 결과 출력 및 시각화
```

---

## 4. 클래스 상세 분석

### 4.1 MatchingCostFactorDemo 클래스

#### 생성자 분석

```cpp
MatchingCostFactorDemo() {
    // 1. 시각화 뷰어 초기화
    auto viewer = guik::LightViewer::instance();
    viewer->enable_vsync();
```

**센서-베이스 변환 정의** (라인 49-56):
```cpp
// sensor.yaml에서 정의된 LiDAR → Base 변환
// T_base_lidar_t_xyz_q_xyzw: [0.0, 0.0, 0.124, 0.0, 0.0, 1.0, 0.0]
Eigen::Vector3d t_base_lidar(0.0, 0.0, 0.124);
Eigen::Quaterniond q_base_lidar(0.0, 0.0, 0.0, 1.0);  // w, x, y, z
gtsam::Pose3 T_base_lidar(gtsam::Rot3(q_base_lidar), t_base_lidar);
```

| 파라미터 | 값 | 의미 |
|----------|-----|------|
| translation | (0, 0, 0.124) | LiDAR가 베이스보다 12.4cm 위에 위치 |
| quaternion | (w=0, x=0, y=0, z=1) | 180도 yaw 회전 |

**Ground Truth 로드** (라인 58-74):
```cpp
std::map<double, gtsam::Pose3> gt_poses;
std::ifstream ifs(data_path + "/gt-tum.txt");

// TUM 형식: timestamp tx ty tz qx qy qz qw
while (ifs >> timestamp >> trans.x() >> trans.y() >> trans.z() 
           >> quat.x() >> quat.y() >> quat.z() >> quat.w()) {
    gt_poses[timestamp] = gtsam::Pose3(gtsam::Rot3(quat), trans);
}
```

**포인트 클라우드 전처리** (라인 129-199):
```cpp
for (int i = 0; i < 5; i++) {
    // 1. PCD 파일에서 포인트 로드
    auto points_f = gtsam_points::read_points(points_path);
    
    // 2. float → double 변환 (동차 좌표계)
    std::vector<Eigen::Vector4d> points(points_f.size());
    std::transform(points_f.begin(), points_f.end(), points.begin(), 
        [](const Eigen::Vector3f& p) {
            return (Eigen::Vector4d() << p.cast<double>(), 1.0).finished();
        });
    
    // 3. 공분산 추정 (GICP에 필요)
    auto covs = gtsam_points::estimate_covariances(points);
    
    // 4. PointCloud 프레임 생성
    auto frame = std::make_shared<gtsam_points::PointCloudCPU>();
    frame->add_points(points);
    frame->add_covs(covs);
    frame->add_normals(gtsam_points::estimate_normals(frame->points, frame->size()));
    
    // 5. VoxelMap 생성 (VGICP에 필요)
    auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(0.5);
    voxelmap->insert(*frame);
}
```

---

## 5. 주요 변수 설명

### 5.1 포즈 관련 변수

| 변수 | 타입 | 역할 |
|------|------|------|
| `poses` | `gtsam::Values` | 최적화 대상 포즈 (노이즈 추가됨) |
| `poses_gt` | `gtsam::Values` | Ground Truth 포즈 (비교용) |
| `T_base_lidar` | `gtsam::Pose3` | 센서-베이스 외부 파라미터 |

### 5.2 포인트 클라우드 데이터

| 변수 | 타입 | 역할 |
|------|------|------|
| `frames` | `vector<PointCloud::Ptr>` | 5개 프레임의 포인트 클라우드 |
| `voxelmaps` | `vector<GaussianVoxelMap::Ptr>` | CPU용 복셀맵 |
| `voxelmaps_gpu` | `vector<GaussianVoxelMap::Ptr>` | GPU용 복셀맵 |

### 5.3 최적화 파라미터

| 변수 | 기본값 | 역할 |
|------|--------|------|
| `pose_noise_scale` | 0.1 | 초기 포즈에 추가할 노이즈 스케일 |
| `factor_type` | 0 (ICP) | 사용할 매칭 알고리즘 선택 |
| `optimizer_type` | 0 (LM) | LM 또는 iSAM2 선택 |
| `full_connection` | true | 모든 프레임 쌍 연결 여부 |
| `num_threads` | 1 | 병렬 처리 스레드 수 |
| `correspondence_update_tolerance_rot` | 0.0 | 대응점 업데이트 회전 허용치 |
| `correspondence_update_tolerance_trans` | 0.0 | 대응점 업데이트 이동 허용치 |

### 5.4 Factor 타입 목록

```cpp
factor_types = {"ICP", "ICP_PLANE", "GICP", "VGICP", "VGICP_GPU"};
```

---

## 6. Factor 타입별 상세 설명

### 6.1 ICP (Point-to-Point)

```cpp
auto factor = gtsam::make_shared<gtsam_points::IntegratedICPFactor>(
    target_key, source_key, target, source);
```

**수학적 정의**:
$$E_{ICP} = \sum_{i} \|T \cdot p_i^{source} - p_{nn(i)}^{target}\|^2$$

| 장점 | 단점 |
|------|------|
| 구현 단순 | 평면 영역에서 수렴 불안정 |
| 계산 빠름 | Local minima에 취약 |

### 6.2 Point-to-Plane ICP

```cpp
auto factor = gtsam::make_shared<gtsam_points::IntegratedPointToPlaneICPFactor>(
    target_key, source_key, target, source);
```

**수학적 정의**:
$$E_{P2Plane} = \sum_{i} (n_{nn(i)} \cdot (T \cdot p_i^{source} - p_{nn(i)}^{target}))^2$$

| 장점 | 단점 |
|------|------|
| 평면 영역에서 안정적 | Normal 추정 필요 |
| 빠른 수렴 | 노이즈에 민감 |

### 6.3 GICP (Generalized ICP)

```cpp
auto factor = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(
    target_key, source_key, target, source);
```

**수학적 정의**:
$$E_{GICP} = \sum_{i} d_i^T (\Sigma_i^{target} + R\Sigma_i^{source}R^T)^{-1} d_i$$

여기서 $d_i = T \cdot p_i^{source} - p_{nn(i)}^{target}$

**특징**:
- Distribution-to-Distribution 매칭
- 각 포인트의 공분산을 고려
- Point-to-Point와 Point-to-Plane의 일반화

### 6.4 VGICP (Voxelized GICP)

```cpp
auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(
    target_key, source_key, target_voxelmap, source);
```

**특징**:
- 복셀 기반 데이터 연관
- 다중 분포 대응 (Multi-Distribution Correspondence)
- 메모리 효율적

**복셀맵 생성**:
```cpp
// resolution = 0.5m
auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(0.5);
voxelmap->insert(*frame);
```

### 6.5 Factor 비교 표

| Factor | 대응점 탐색 | 오차 메트릭 | GPU 지원 | 메모리 |
|--------|-------------|-------------|----------|--------|
| ICP | KD-Tree | Point-to-Point | X | 낮음 |
| ICP_PLANE | KD-Tree | Point-to-Plane | X | 낮음 |
| GICP | KD-Tree | Mahalanobis | X | 중간 |
| VGICP | VoxelMap | Mahalanobis | X | 중간 |
| VGICP_GPU | VoxelMap | Mahalanobis | O | 높음 |

---

## 7. 최적화 알고리즘

### 7.1 Factor 생성 함수

```cpp
gtsam::NonlinearFactor::shared_ptr create_factor(
    gtsam::Key target_key,
    gtsam::Key source_key,
    const gtsam_points::PointCloud::ConstPtr& target,
    const gtsam_points::GaussianVoxelMap::ConstPtr& target_voxelmap,
    const gtsam_points::GaussianVoxelMap::ConstPtr& target_voxelmap_gpu,
    const gtsam_points::PointCloud::ConstPtr& source);
```

**파라미터 설명**:
| 파라미터 | 역할 |
|----------|------|
| `target_key` | 타겟 프레임 변수 키 |
| `source_key` | 소스 프레임 변수 키 |
| `target` | 타겟 포인트 클라우드 |
| `target_voxelmap` | VGICP용 CPU 복셀맵 |
| `target_voxelmap_gpu` | VGICP_GPU용 GPU 복셀맵 |
| `source` | 소스 포인트 클라우드 |

### 7.2 Graph 구성

```cpp
void run_optimization() {
    gtsam::NonlinearFactorGraph graph;
    
    // Prior Factor: 첫 번째 포즈 고정
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(
        0, 
        poses.at<gtsam::Pose3>(0), 
        gtsam::noiseModel::Isotropic::Precision(6, 1e6)  // 매우 높은 정밀도
    ));
    
    // Matching Cost Factors 추가
    for (int i = 0; i < 5; i++) {
        int j_end = full_connection ? 5 : std::min(i + 2, 5);
        for (int j = i + 1; j < j_end; j++) {
            auto factor = create_factor(i, j, frames[i], voxelmaps[i], 
                                        voxelmaps_gpu[i], frames[j]);
            graph.add(factor);
        }
    }
}
```

**연결 방식 비교**:

Full Connection (full_connection = true):
```
[0] --- [1] --- [2] --- [3] --- [4]
 |\_______|\_______|\_______|
 |________|\_______|\_______|
 |_________|\_______|
 |__________|\_______|
```

Sequential Connection (full_connection = false):
```
[0] --- [1] --- [2] --- [3] --- [4]
```

### 7.3 Levenberg-Marquardt 최적화

```cpp
gtsam_points::LevenbergMarquardtExtParams lm_params;
lm_params.maxIterations = 100;
lm_params.relativeErrorTol = 1e-5;
lm_params.absoluteErrorTol = 1e-5;
lm_params.callback = [this](const auto& status, const gtsam::Values& values) {
    update_viewer(values);  // 매 반복마다 시각화 업데이트
};

gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, poses, lm_params);
optimized_values = optimizer.optimize();
```

**LM 알고리즘 개요**:
1. 초기 추정값 $x_0$에서 시작
2. 각 반복에서:
   - Jacobian $J$ 및 오차 $e$ 계산
   - $(J^TJ + \lambda I)\Delta x = -J^Te$ 풀기
   - $x_{k+1} = x_k + \Delta x$
3. 수렴 조건 만족 시 종료

### 7.4 iSAM2 최적화

```cpp
gtsam::ISAM2Params isam2_params;
isam2_params.relinearizeSkip = 1;      // 매 스텝 재선형화
isam2_params.setRelinearizeThreshold(0.0);  // 항상 재선형화
gtsam_points::ISAM2Ext isam2(isam2_params);

// 초기 업데이트
auto status = isam2.update(graph, poses);

// 추가 최적화 반복
for (int i = 0; i < 5; i++) {
    auto status = isam2.update();
    update_viewer(isam2.calculateEstimate());
}

optimized_values = isam2.calculateEstimate();
```

**iSAM2 특징**:
- 점진적(Incremental) 최적화
- Bayes Tree 기반 효율적 업데이트
- 온라인 SLAM에 적합

---

## 8. 실행 흐름

### 8.1 전체 실행 시퀀스

```
┌─────────────────────────────────────────────────────────────┐
│                         INITIALIZATION                       │
├─────────────────────────────────────────────────────────────┤
│ 1. Viewer 초기화                                             │
│ 2. Ground Truth 로드 (gt-tum.txt)                           │
│ 3. PCD 파일 로드 (5개)                                       │
│ 4. 각 프레임 처리:                                           │
│    ├── 포인트 변환 (float→double, 동차좌표)                  │
│    ├── 공분산 추정                                           │
│    ├── 법선 추정                                             │
│    ├── PointCloud 생성                                       │
│    └── VoxelMap 생성                                         │
│ 5. UI 콜백 등록                                              │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                      USER INTERACTION                        │
├─────────────────────────────────────────────────────────────┤
│ • noise_scale 조정 → "add noise" 버튼으로 초기 포즈 교란    │
│ • factor_type 선택 (ICP/GICP/VGICP 등)                      │
│ • optimizer_type 선택 (LM/iSAM2)                            │
│ • full_connection 토글                                       │
│ • "optimize" 버튼 클릭                                       │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                      OPTIMIZATION                            │
├─────────────────────────────────────────────────────────────┤
│ 1. NonlinearFactorGraph 생성                                 │
│ 2. PriorFactor 추가 (첫 포즈 고정)                          │
│ 3. Matching Cost Factor 추가 (프레임 쌍마다)                 │
│ 4. 최적화 실행 (LM 또는 iSAM2)                              │
│ 5. 매 반복마다 시각화 업데이트                               │
│ 6. 결과 출력 (Optimized vs GT 비교)                         │
└─────────────────────────────────────────────────────────────┘
```

### 8.2 Factor 연산 흐름

```
create_factor() 호출
         │
         ▼
┌─────────────────────────┐
│ Factor 타입 확인        │
│ (ICP/GICP/VGICP 등)    │
└─────────────────────────┘
         │
         ▼
┌─────────────────────────────────────────┐
│ IntegratedXXXFactor 생성                 │
│ • target_key, source_key 설정           │
│ • target, source 포인트 클라우드 참조   │
│ • correspondence_update_tolerance 설정  │
│ • num_threads 설정                      │
└─────────────────────────────────────────┘
         │
         ▼
  [최적화 과정에서]
         │
         ▼
┌─────────────────────────────────────────┐
│ linearize() 호출 (매 반복)              │
│                                         │
│ 1. calc_delta(): 상대 변환 계산         │
│    delta = T_target^{-1} * T_source     │
│                                         │
│ 2. update_correspondences():            │
│    Source 포인트 → Target 대응점 탐색   │
│                                         │
│ 3. evaluate():                          │
│    오차 및 Hessian/Gradient 계산        │
│                                         │
│ 4. HessianFactor 반환                   │
└─────────────────────────────────────────┘
```

---

## 9. 튜토리얼: 단계별 동작 원리

### 9.1 Step 1: 데이터 준비

**입력 데이터 구조**:
```
/root/workdir/data/pcd/
├── 1710406871.842252000.pcd   # 타임스탬프가 파일명
├── 1710406871.942252000.pcd
├── 1710406872.042252000.pcd
├── 1710406872.142252000.pcd
├── 1710406872.242252000.pcd
└── gt-tum.txt                 # Ground Truth (TUM 형식)
```

**gt-tum.txt 형식**:
```
timestamp tx ty tz qx qy qz qw
1710406871.842252 0.0 0.0 0.0 0.0 0.0 0.0 1.0
1710406871.942252 0.1 0.0 0.0 0.0 0.0 0.0 1.0
...
```

### 9.2 Step 2: 포즈 초기화

```cpp
// 첫 프레임을 원점으로 설정
gtsam::Pose3 W_T_L_origin = W_T_B_origin * T_base_lidar;

// 각 프레임의 상대 포즈 계산
for (int i = 0; i < 5; i++) {
    gtsam::Pose3 W_T_L = W_T_B * T_base_lidar;
    gtsam::Pose3 relative_pose = W_T_L_origin.inverse() * W_T_L;
    
    poses.insert(i, relative_pose);
    poses_gt.insert(i, relative_pose);
}
```

**좌표계 변환 체인**:
```
World (W) → Base (B) → LiDAR (L)

W_T_L = W_T_B * B_T_L (T_base_lidar)

상대 포즈 = L0_T_Li = (W_T_L0)^{-1} * W_T_Li
```

### 9.3 Step 3: 노이즈 추가

```cpp
// UI에서 "add noise" 버튼 클릭 시
for (int i = 1; i < 5; i++) {
    // se(3) Lie algebra에서 랜덤 섭동 생성
    gtsam::Pose3 noise = gtsam::Pose3::Expmap(
        gtsam::Vector6::Random() * pose_noise_scale
    );
    
    // Ground Truth에 노이즈 곱하기
    poses.update<gtsam::Pose3>(i, poses_gt.at<gtsam::Pose3>(i) * noise);
}
```

**Expmap 설명**:
- `Vector6::Random()`: 6차원 랜덤 벡터 $[\omega_x, \omega_y, \omega_z, v_x, v_y, v_z]$
- `Pose3::Expmap()`: se(3) → SE(3) 지수 사상
- 결과: 작은 랜덤 회전 + 이동

### 9.4 Step 4: Factor Graph 구성

```cpp
gtsam::NonlinearFactorGraph graph;

// 1. Prior Factor: 첫 포즈 고정
graph.add(gtsam::PriorFactor<gtsam::Pose3>(
    0,                                    // key
    poses.at<gtsam::Pose3>(0),           // 값
    gtsam::noiseModel::Isotropic::Precision(6, 1e6)  // 노이즈 모델
));

// 2. Matching Cost Factors
for (int i = 0; i < 5; i++) {
    for (int j = i + 1; j < j_end; j++) {
        // target=i, source=j로 Factor 생성
        auto factor = create_factor(i, j, ...);
        graph.add(factor);
    }
}
```

**그래프 구조 (full_connection=true)**:
```
Factor 수: C(5,2) = 10개
- Factor(0,1), Factor(0,2), Factor(0,3), Factor(0,4)
- Factor(1,2), Factor(1,3), Factor(1,4)
- Factor(2,3), Factor(2,4)
- Factor(3,4)
```

### 9.5 Step 5: 최적화 실행

**Levenberg-Marquardt 과정**:

```
반복 k=0:
├── 현재 추정값 x_k로 모든 Factor linearize()
├── Hessian H = ΣH_i, Gradient b = Σb_i 누적
├── (H + λI)Δx = -b 풀기
├── x_{k+1} = x_k ⊕ Δx (SE(3) 업데이트)
└── update_viewer(x_{k+1})

반복 k=1:
├── ...

수렴 조건:
├── |error_{k} - error_{k-1}| / error_{k-1} < relativeErrorTol
└── 또는 |Δx| < absoluteErrorTol
```

### 9.6 Step 6: 결과 분석

```cpp
// 최적화 결과 vs Ground Truth 비교
for (int i = 0; i < 5; i++) {
    gtsam::Pose3 opt_pose = optimized_values.at<gtsam::Pose3>(i);
    gtsam::Pose3 gt_pose = poses_gt.at<gtsam::Pose3>(i);
    
    // 오차 = GT^{-1} * Optimized
    gtsam::Pose3 error = gt_pose.inverse() * opt_pose;
    
    // 이동 오차 (m)
    double trans_error = error.translation().norm();
    
    // 회전 오차 (deg)
    double rot_error = error.rotation().axisAngle().second * 180.0 / M_PI;
}
```

**출력 예시**:
```
--- Results: VGICP ---

Frame 0:
  [Optimized] t: 0 0 0
              R (ypr): 0 0 0 deg
  [GT]        t: 0 0 0
              R (ypr): 0 0 0 deg
  [Error]     t: 0 m
              R: 0 deg

Frame 1:
  [Optimized] t: 0.0998 0.0012 0.0003
              R (ypr): 0.5 0.1 0.2 deg
  [GT]        t: 0.1 0 0
              R (ypr): 0 0 0 deg
  [Error]     t: 0.0012 m
              R: 0.54 deg

--- Summary ---
Mean Translation Error: 0.0023 m
Mean Rotation Error: 0.42 deg
```

---

## 10. 참고 자료

### 10.1 논문 참조

| 알고리즘 | 논문 |
|----------|------|
| ICP | Zhang, "Iterative Point Matching for Registration of Free-Form Curve", IJCV 1994 |
| GICP | Segal et al., "Generalized-ICP", RSS 2005 |
| VGICP | Koide et al., "Voxelized GICP for Fast and Accurate 3D Point Cloud Registration", ICRA 2021 |
| VGICP GPU | Koide et al., "Globally Consistent 3D LiDAR Mapping with GPU-accelerated GICP Matching Cost Factors", RA-L 2021 |

### 10.2 GTSAM 리소스

- [GTSAM 공식 문서](https://gtsam.org/)
- [GTSAM GitHub](https://github.com/borglab/gtsam)
- [gtsam_points GitHub](https://github.com/koide3/gtsam_points)

### 10.3 클래스 다이어그램

```
┌────────────────────────────────────────────────────────────┐
│                   gtsam::NonlinearFactor                   │
│  ┌──────────────────────────────────────────────────────┐  │
│  │ virtual double error(const Values&)                  │  │
│  │ virtual GaussianFactor::shared_ptr linearize(...)    │  │
│  │ KeyVector keys()                                     │  │
│  └──────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────┘
                              △
                              │ 상속
┌────────────────────────────────────────────────────────────┐
│            gtsam_points::IntegratedMatchingCostFactor      │
│  ┌──────────────────────────────────────────────────────┐  │
│  │ Eigen::Isometry3d calc_delta(const Values&)          │  │
│  │ virtual void update_correspondences(...) = 0         │  │
│  │ virtual double evaluate(...) = 0                     │  │
│  │                                                      │  │
│  │ bool is_binary                                       │  │
│  │ Eigen::Isometry3d fixed_target_pose                  │  │
│  └──────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────┘
         △                    △                    △
         │                    │                    │
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│IntegratedICPFactor│ │IntegratedGICPFactor│ │IntegratedVGICPFactor│
│                 │  │                 │  │                 │
│ KD-Tree 기반    │  │ 공분산 고려     │  │ VoxelMap 기반   │
│ Point-to-Point  │  │ Mahalanobis     │  │ 효율적 탐색     │
└─────────────────┘  └─────────────────┘  └─────────────────┘
```

### 10.4 성능 비교 가이드

| 상황 | 추천 Factor | 이유 |
|------|-------------|------|
| 빠른 프로토타이핑 | ICP | 가장 단순, 빠름 |
| 구조화된 환경 | ICP_PLANE | 평면 영역 안정적 |
| 일반적 사용 | GICP | 정확도와 속도 균형 |
| 대규모 맵 | VGICP | 메모리 효율적 |
| 실시간 요구 | VGICP_GPU | GPU 가속 |

---

## 부록: 핵심 코드 스니펫

### A. PointCloud 구조체

```cpp
struct PointCloud {
    size_t num_points;           // 포인트 수
    Eigen::Vector4d* points;     // 포인트 좌표 (x, y, z, 1)
    Eigen::Vector4d* normals;    // 법선 벡터 (nx, ny, nz, 0)
    Eigen::Matrix4d* covs;       // 공분산 행렬
    double* intensities;         // 반사 강도
};
```

### B. GaussianVoxelMap 인터페이스

```cpp
class GaussianVoxelMap {
public:
    virtual double voxel_resolution() const = 0;
    virtual void insert(const PointCloud& frame) = 0;
};
```

### C. Factor 생성 패턴

```cpp
// Binary Factor (두 변수 모두 최적화)
IntegratedGICPFactor(target_key, source_key, target, source);

// Unary Factor (target 고정, source만 최적화)
IntegratedGICPFactor(fixed_target_pose, source_key, target, source);
```

---

*문서 작성일: 2026-01-26*
*gtsam_points 버전: 1.2.0*
