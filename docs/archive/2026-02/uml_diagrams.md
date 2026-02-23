# NDT Factor 구현 - UML 다이어그램

이 문서는 NDT Factor 구현과 Main 코드의 구조를 PlantUML 다이어그램으로 시각화합니다.

---

## 1. NDT Factor 클래스 계층 구조

### 1.1 Factor 상속 계층

```plantuml
@startuml ndt_factor_hierarchy
!theme blueprint
skinparam linetype ortho

title NDT Factor 클래스 계층 구조

package "gtsam" {
  abstract class NonlinearFactor {
    #keys_ : KeyVector
    +{abstract} error(values) : double
    +{abstract} linearize(values) : GaussianFactor
    +{abstract} dim() : size_t
  }
}

package "gtsam_points" {
  abstract class IntegratedMatchingCostFactor {
    #is_binary : bool
    #fixed_target_pose : Isometry3d
    +IntegratedMatchingCostFactor(target_key, source_key)
    +IntegratedMatchingCostFactor(fixed_target_pose, source_key)
    +error(values) : double
    +linearize(values) : GaussianFactor
    +dim() : size_t {return 6}
    +{abstract} update_correspondences(delta) : void
    +{abstract} evaluate(delta, H*, b*) : double
    +memory_usage() : size_t
  }
  
  class "IntegratedNDTFactor_<SourceFrame>" as NDTFactor {
    -target_voxels : shared_ptr<GaussianVoxelMapCPU>
    -source : shared_ptr<SourceFrame>
    -correspondences : mutable vector<GaussianVoxel*>
    -num_threads : int
    -resolution : double
    -outlier_ratio : double
    -regularization_epsilon : double
    -search_mode : NDTSearchMode
    -gauss_d1, gauss_d2 : mutable double
    
    +IntegratedNDTFactor_(target_key, source_key, target_voxels, source)
    +IntegratedNDTFactor_(fixed_target_pose, source_key, target_voxels, source)
    +~IntegratedNDTFactor_()
    
    +set_num_threads(n) : void
    +set_resolution(r) : void
    +set_outlier_ratio(ratio) : void
    +set_regularization_epsilon(eps) : void
    +set_search_mode(mode) : void
    
    +num_inliers() : int
    +inlier_fraction() : double
    +get_target() : const GaussianVoxelMapCPU&
    
    +print(s, formatter) : void
    +memory_usage() : size_t
    +clone() : NonlinearFactor::shared_ptr
    
    #update_correspondences(delta) : void
    #evaluate(delta, H*, b*) : double
  }
  
  class "IntegratedVGICPFactor_<SourceFrame>" as VGICPFactor {
    -target_voxels : shared_ptr<GaussianVoxelMapCPU>
    -source : shared_ptr<SourceFrame>
    +update_correspondences(delta) : void
    +evaluate(delta, H*, b*) : double
  }
  
  class "IntegratedGICPFactor_<T, S>" as GICPFactor {
    -target : shared_ptr<TargetFrame>
    -source : shared_ptr<SourceFrame>
    +update_correspondences(delta) : void
    +evaluate(delta, H*, b*) : double
  }
  
  class IntegratedICPFactor {
    -target : shared_ptr<PointCloud>
    -source : shared_ptr<PointCloud>
    +update_correspondences(delta) : void
    +evaluate(delta, H*, b*) : double
  }
}

NonlinearFactor <|-- IntegratedMatchingCostFactor
IntegratedMatchingCostFactor <|-- NDTFactor
IntegratedMatchingCostFactor <|-- VGICPFactor
IntegratedMatchingCostFactor <|-- GICPFactor
IntegratedMatchingCostFactor <|-- IntegratedICPFactor

note right of IntegratedMatchingCostFactor
  추상 기반 클래스
  (Abstract Base Class)
  
  모든 스캔 매칭 팩터의
  공통 인터페이스 제공
end note

note right of NDTFactor
  본 프로젝트에서 구현한
  NDT Factor
  
  템플릿 인스턴스화:
  - IntegratedNDTFactor (PointCloud)
  - IntegratedNDTFactor_<DummyFrame>
end note

@enduml
```

### 1.2 NDT Factor 데이터 구조

```plantuml
@startuml ndt_data_structures
!theme blueprint

title NDT Factor 데이터 구조 관계

class "IntegratedNDTFactor_<SourceFrame>" as NDTFactor {
  -target_voxels
  -source
  -correspondences
  -num_threads
  -resolution
  -outlier_ratio
  -regularization_epsilon
  -search_mode
}

class GaussianVoxelMapCPU {
  -voxels_ : unordered_map<Voxel, GaussianVoxel>
  -voxel_resolution_ : double
  +insert(frame) : void
  +lookup_voxel(coord) : GaussianVoxel*
  +voxel_coord(point) : Vector3i
  +finalize() : void
}

class GaussianVoxel {
  +mean : Vector4d
  +cov : Matrix4d
  +inv_cov : Matrix4d
  +inv_cov_valid : bool
  +gauss_d1, gauss_d2 : double
  +intensity : double
  +num_points : int
  +finalized : bool
  
  +compute_inverse_covariance(epsilon) : void
  +{static} compute_ndt_params(resolution, ratio, d1, d2) : void
}

class PointCloud {
  +points : vector<Vector4d>
  +normals : vector<Vector4d>
  +covs : vector<Matrix4d>
  +size() : size_t
  +add_points(pts) : void
}

enum NDTSearchMode {
  DIRECT1
  DIRECT7
  DIRECT27
}

NDTFactor *-- GaussianVoxelMapCPU : target_voxels
NDTFactor *-- PointCloud : source
NDTFactor o-- NDTSearchMode : search_mode

GaussianVoxelMapCPU o-- GaussianVoxel : contains many

NDTFactor ..> GaussianVoxel : uses correspondences

note right of GaussianVoxel
  NDT 확장 필드:
  - inv_cov (역공분산 캐시)
  - inv_cov_valid (유효성)
  - gauss_d1, gauss_d2 (NDT 파라미터)
end note

note bottom of NDTSearchMode
  DIRECT1: 현재 복셀만
  DIRECT7: 중심 + 6개 면 이웃 (기본값)
  DIRECT27: 중심 + 26개 전체 이웃
end note

@enduml
```

---

## 2. Main 코드 구조

### 2.1 MatchingCostFactorDemo 클래스

```plantuml
@startuml main_class_structure
!theme blueprint

title Main 코드 - MatchingCostFactorDemo 클래스 구조

class MatchingCostFactorDemo {
  -frames : vector<PointCloud::ConstPtr>
  -voxelmaps : vector<GaussianVoxelMap::ConstPtr>
  -frames_with_ring : vector<PointWithRing>
  -loam_features : vector<LOAMFeatures>
  -poses : Values
  -poses_gt : Values
  
  -pose_noise_scale : float
  -optimizer_type : int
  -optimizer_types : vector<string>
  -factor_type : int
  -factor_types : vector<string>
  -full_connection : bool
  -num_threads : int
  -correspondence_update_tolerance_rot : float
  -correspondence_update_tolerance_trans : float
  
  -optimization_thread : thread
  
  +MatchingCostFactorDemo()
  +~MatchingCostFactorDemo()
  -update_viewer(values) : void
  -create_factor(target_key, source_key, ...) : NonlinearFactor::shared_ptr
  -run_optimization() : void
}

package "gtsam_points::factors" {
  class IntegratedICPFactor
  class IntegratedGICPFactor
  class IntegratedVGICPFactor
  class IntegratedLOAMFactor
  class IntegratedNDTFactor
}

package "gtsam" {
  class NonlinearFactorGraph {
    +add(factor) : void
    +error(values) : double
  }
  
  class Values {
    +insert(key, value) : void
    +at<T>(key) : T
    +update(key, value) : void
  }
  
  class LevenbergMarquardtOptimizer {
    +optimize() : Values
  }
  
  class ISAM2 {
    +update(graph, values) : void
    +calculateEstimate() : Values
  }
}

package "gtsam_points::types" {
  class PointCloud {
    +points : vector<Vector4d>
    +size() : size_t
  }
  
  class GaussianVoxelMapCPU {
    +insert(frame) : void
    +lookup_voxel(coord) : GaussianVoxel*
  }
}

package "visualization" {
  class "guik::LightViewer" as LightViewer {
    +{static} instance() : LightViewer*
    +update_drawable(name, buffer, style) : void
    +register_ui_callback(name, callback) : void
  }
}

MatchingCostFactorDemo ..> IntegratedICPFactor : creates
MatchingCostFactorDemo ..> IntegratedGICPFactor : creates
MatchingCostFactorDemo ..> IntegratedVGICPFactor : creates
MatchingCostFactorDemo ..> IntegratedLOAMFactor : creates
MatchingCostFactorDemo ..> IntegratedNDTFactor : creates

MatchingCostFactorDemo --> NonlinearFactorGraph : builds
MatchingCostFactorDemo --> Values : manages poses
MatchingCostFactorDemo --> LevenbergMarquardtOptimizer : uses
MatchingCostFactorDemo --> ISAM2 : uses

MatchingCostFactorDemo o-- PointCloud : stores frames
MatchingCostFactorDemo o-- GaussianVoxelMapCPU : stores voxelmaps

MatchingCostFactorDemo ..> LightViewer : visualizes with

note right of MatchingCostFactorDemo
  Main 애플리케이션 클래스
  
  지원하는 Factor 타입:
  0: Point-to-Point ICP
  1: Point-to-Plane ICP
  2: GICP
  3: VGICP
  4: LOAM
  5: NDT (추가됨)
  
  Optimizer 타입:
  0: Levenberg-Marquardt
  1: ISAM2
end note

@enduml
```

### 2.2 Main 코드 데이터 흐름

```plantuml
@startuml main_data_flow
!theme blueprint

title Main 코드 - 데이터 처리 흐름

rectangle "데이터 로딩" as loading {
  (PCD 파일) as pcd
  (Ground Truth\nTUM 형식) as gt
}

rectangle "전처리" as preprocessing {
  (포인트 클라우드\n생성) as pc_create
  (공분산 추정) as cov_est
  (법선 벡터 추정) as normal_est
  (LOAM 특징점\n추출) as loam_extract
  (가우시안\n복셀맵 생성) as voxel_create
}

rectangle "최적화" as optimization {
  (팩터 생성) as factor_create
  (팩터 그래프\n구축) as graph_build
  (초기값 설정) as initial_set
  (최적화 실행) as optimize
}

rectangle "시각화" as visualization {
  (결과 표시) as display
  (UI 콜백) as ui_callback
}

pcd --> pc_create
gt --> initial_set

pc_create --> cov_est
cov_est --> normal_est
normal_est --> loam_extract
pc_create --> voxel_create

pc_create --> factor_create
voxel_create --> factor_create
loam_extract --> factor_create

factor_create --> graph_build
graph_build --> optimize
initial_set --> optimize

optimize --> display
display --> ui_callback
ui_callback --> optimize : 파라미터 변경

note right of pc_create
  PointCloudCPU 생성
  - Homogeneous 좌표 (x,y,z,1)
  - Float → Double 변환
end note

note right of cov_est
  estimate_covariances()
  - KD-Tree 기반 이웃 검색
  - 각 점의 공분산 계산
end note

note right of voxel_create
  GaussianVoxelMapCPU
  - resolution = 0.5m
  - 복셀 내 점군 통계 계산
end note

note right of factor_create
  create_factor() 함수
  - ICP, GICP, VGICP,
    LOAM, NDT 중 선택
end note

@enduml
```

---

## 3. NDT Factor 통합 다이어그램

### 3.1 Main 코드와 NDT Factor 통합

```plantuml
@startuml integration_diagram
!theme blueprint
skinparam linetype ortho

title Main 코드 ↔ NDT Factor 통합 구조

package "사용자 코드 (main.cpp)" {
  class MatchingCostFactorDemo {
    +create_factor() : NonlinearFactor::shared_ptr
    +run_optimization() : void
  }
}

package "gtsam_points::factors" {
  abstract class IntegratedMatchingCostFactor {
    +{abstract} update_correspondences(delta)
    +{abstract} evaluate(delta, H*, b*)
  }
  
  class IntegratedNDTFactor {
    +set_num_threads(n)
    +set_resolution(r)
    +set_outlier_ratio(ratio)
    +set_search_mode(mode)
    +num_inliers() : int
  }
}

package "gtsam_points::types" {
  class PointCloud {
    +points : vector<Vector4d>
    +covs : vector<Matrix4d>
  }
  
  class GaussianVoxelMapCPU {
    -voxels_ : unordered_map
    +insert(frame)
    +lookup_voxel(coord)
  }
  
  class GaussianVoxel {
    +mean : Vector4d
    +cov : Matrix4d
    +inv_cov : Matrix4d
    +compute_inverse_covariance(eps)
  }
}

package "gtsam::optimization" {
  class NonlinearFactorGraph {
    +add(factor)
  }
  
  class LevenbergMarquardtOptimizer {
    +optimize() : Values
  }
  
  class Values {
    +insert(key, pose)
    +at<Pose3>(key)
  }
}

MatchingCostFactorDemo ..> IntegratedNDTFactor : creates
MatchingCostFactorDemo ..> NonlinearFactorGraph : adds factor to

IntegratedMatchingCostFactor <|-- IntegratedNDTFactor

IntegratedNDTFactor --> GaussianVoxelMapCPU : uses target_voxels
IntegratedNDTFactor --> PointCloud : uses source
IntegratedNDTFactor ..> GaussianVoxel : accesses via correspondences

GaussianVoxelMapCPU o-- GaussianVoxel : contains

NonlinearFactorGraph o-- IntegratedNDTFactor : contains
LevenbergMarquardtOptimizer --> NonlinearFactorGraph : optimizes
LevenbergMarquardtOptimizer --> Values : updates

MatchingCostFactorDemo --> Values : manages poses

note as N1
  **사용 패턴**:
  
  1. Main 코드가 PCD 로드
  2. PointCloud 생성
  3. GaussianVoxelMapCPU 생성
  4. IntegratedNDTFactor 생성
  5. NonlinearFactorGraph에 추가
  6. LM Optimizer 실행
  7. 최적화된 포즈 획득
end note

@enduml
```

### 3.2 NDT Factor 실행 시퀀스

```plantuml
@startuml ndt_sequence
!theme blueprint

title NDT Factor 실행 시퀀스 다이어그램

actor User
participant "MatchingCostFactorDemo" as Demo
participant "IntegratedNDTFactor" as NDT
participant "GaussianVoxelMapCPU" as VoxelMap
participant "GaussianVoxel" as Voxel
participant "LM Optimizer" as LM

== 초기화 단계 ==

User -> Demo : 생성자 호출
activate Demo

Demo -> Demo : PCD 파일 로드
Demo -> Demo : PointCloud 생성
Demo -> VoxelMap : insert(target_points)
activate VoxelMap
VoxelMap -> Voxel : 통계 계산 (mean, cov)
VoxelMap --> Demo : voxelmap 반환
deactivate VoxelMap

Demo -> NDT : create(target_key, source_key,\ntarget_voxelmap, source_points)
activate NDT
NDT -> NDT : 멤버 변수 초기화
NDT -> NDT : correspondences.resize(source->size())
NDT --> Demo : factor 반환

Demo -> NDT : set_num_threads(4)
Demo -> NDT : set_search_mode(DIRECT7)
Demo -> NDT : set_outlier_ratio(0.1)

Demo -> Demo : graph.add(ndt_factor)
deactivate Demo

== 최적화 단계 ==

User -> Demo : run_optimization()
activate Demo

Demo -> LM : LevenbergMarquardtOptimizer(graph, initial)
activate LM

loop 각 iteration
  LM -> NDT : linearize(values)
  activate NDT
  
  NDT -> NDT : calc_delta(values)
  NDT -> NDT : update_correspondences(delta)
  activate NDT
  
  NDT -> Voxel : compute_inverse_covariance(eps)
  activate Voxel
  Voxel -> Voxel : 고유값 분해
  Voxel -> Voxel : 정규화 및 역행렬 계산
  Voxel --> NDT : inv_cov
  deactivate Voxel
  
  loop 각 source 점
    NDT -> VoxelMap : lookup_voxel(transformed_point)
    activate VoxelMap
    
    alt DIRECT7 모드
      VoxelMap -> VoxelMap : 7개 이웃 검색
      VoxelMap -> VoxelMap : best voxel 선택\n(최소 Mahalanobis 거리)
    else DIRECT1 모드
      VoxelMap -> VoxelMap : 현재 복셀만
    end
    
    VoxelMap --> NDT : voxel pointer
    deactivate VoxelMap
    
    NDT -> NDT : correspondences[i] = voxel
  end
  deactivate NDT
  
  NDT -> NDT : evaluate(delta, H*, b*)
  activate NDT
  
  loop 각 correspondence
    NDT -> NDT : error = transformed_pt - voxel->mean
    NDT -> NDT : score = error^T * inv_cov * error
    NDT -> NDT : 야코비안 계산 (Lie Algebra)
    NDT -> NDT : Hessian 근사 누적
  end
  
  NDT --> LM : JacobianFactor
  deactivate NDT
  deactivate NDT
  
  LM -> LM : Gauss-Newton 스텝 계산
  LM -> LM : values 업데이트
  
  alt 수렴 조건 만족
    LM --> Demo : optimized values
  end
end

deactivate LM

Demo -> NDT : num_inliers()
activate NDT
NDT --> Demo : inlier count
deactivate NDT

Demo -> Demo : update_viewer(result)
Demo --> User : 최적화 완료
deactivate Demo

@enduml
```

---

## 4. 패키지 다이어그램

### 4.1 전체 시스템 패키지 구조

```plantuml
@startuml package_diagram
!theme blueprint

title 전체 시스템 패키지 구조

package "사용자 애플리케이션" {
  [main.cpp]
  [loam_feature.cpp]
  [loam_feature.hpp]
}

package "gtsam_points" {
  package "factors" {
    [integrated_matching_cost_factor.hpp]
    [integrated_ndt_factor.hpp]
    [integrated_ndt_factor_impl.hpp]
    [integrated_vgicp_factor.hpp]
    [integrated_gicp_factor.hpp]
    [integrated_icp_factor.hpp]
    [integrated_loam_factor.hpp]
  }
  
  package "types" {
    [point_cloud_cpu.hpp]
    [gaussian_voxelmap_cpu.hpp]
  }
  
  package "optimizers" {
    [levenberg_marquardt_ext.hpp]
    [isam2_ext.hpp]
  }
  
  package "features" {
    [normal_estimation.hpp]
    [covariance_estimation.hpp]
  }
  
  package "ann" {
    [kdtree.hpp]
  }
}

package "GTSAM" {
  package "geometry" {
    [Pose3.hpp]
    [Rot3.hpp]
  }
  
  package "nonlinear" {
    [NonlinearFactor.hpp]
    [NonlinearFactorGraph.hpp]
    [LevenbergMarquardtOptimizer.hpp]
  }
  
  package "linear" {
    [GaussianFactor.hpp]
    [JacobianFactor.hpp]
  }
}

package "Eigen" {
  [Core]
  [Geometry]
  [Eigenvalues]
}

package "Visualization" {
  [guik::LightViewer]
  [glk::PointCloudBuffer]
}

[main.cpp] --> [integrated_ndt_factor.hpp] : includes
[main.cpp] --> [integrated_vgicp_factor.hpp]
[main.cpp] --> [integrated_gicp_factor.hpp]
[main.cpp] --> [integrated_loam_factor.hpp]
[main.cpp] --> [point_cloud_cpu.hpp]
[main.cpp] --> [gaussian_voxelmap_cpu.hpp]

[integrated_ndt_factor.hpp] --> [integrated_matching_cost_factor.hpp]
[integrated_ndt_factor_impl.hpp] --> [integrated_ndt_factor.hpp]
[gaussian_voxelmap_cpu.hpp] ..> [Eigenvalues] : uses

[integrated_matching_cost_factor.hpp] --> [NonlinearFactor.hpp]
[main.cpp] --> [NonlinearFactorGraph.hpp]
[main.cpp] --> [LevenbergMarquardtOptimizer.hpp]

[main.cpp] --> [guik::LightViewer]

note right of [integrated_ndt_factor.hpp]
  **본 프로젝트에서 구현**
  
  IntegratedNDTFactor_<SourceFrame>
  - NDT 알고리즘 구현
  - DIRECT1/7/27 검색 모드
end note

note right of [gaussian_voxelmap_cpu.hpp]
  **NDT 확장**
  
  GaussianVoxel:
  - inv_cov (역공분산)
  - gauss_d1, gauss_d2
  - compute_inverse_covariance()
end note

@enduml
```

---

## 5. 컴포넌트 다이어그램

### 5.1 NDT Factor 컴포넌트 구조

```plantuml
@startuml component_diagram
!theme blueprint

title NDT Factor 컴포넌트 구조

component "Main Application" {
  [MatchingCostFactorDemo]
}

component "NDT Factor Module" {
  [IntegratedNDTFactor_<T>] as NDTFactor
  [NDTSearchMode] as SearchMode
  [update_correspondences()] as UpdateCorr
  [evaluate()] as Evaluate
}

component "Voxel Map Module" {
  [GaussianVoxelMapCPU] as VoxelMap
  [GaussianVoxel] as Voxel
  [compute_inverse_covariance()] as InvCov
  [compute_ndt_params()] as NDTParams
}

component "Point Cloud Module" {
  [PointCloudCPU] as PointCloud
  [estimate_covariances()] as EstCov
  [estimate_normals()] as EstNormals
}

component "GTSAM Optimization" {
  [NonlinearFactorGraph] as Graph
  [LevenbergMarquardtOptimizer] as LM
  [Values] as Values
}

interface "Factor Interface" as IFactor
interface "Correspondence" as ICorr
interface "Evaluation" as IEval

[MatchingCostFactorDemo] --> IFactor : uses
IFactor <-- NDTFactor : implements

NDTFactor --> ICorr : provides
ICorr <-- UpdateCorr : implements

NDTFactor --> IEval : provides
IEval <-- Evaluate : implements

NDTFactor ..> VoxelMap : uses
NDTFactor ..> PointCloud : uses
NDTFactor o-- SearchMode : configuration

VoxelMap *-- Voxel : contains
Voxel ..> InvCov : computes
Voxel ..> NDTParams : computes

PointCloud ..> EstCov : uses
PointCloud ..> EstNormals : uses

[MatchingCostFactorDemo] --> Graph : builds
Graph --> LM : optimized by
LM --> Values : updates

note right of NDTFactor
  핵심 컴포넌트
  
  - Correspondence 관리
  - Error 계산
  - Jacobian 계산
end note

note right of VoxelMap
  복셀 기반 표현
  
  - 해시맵 기반 인덱싱
  - 가우시안 통계
  - 역공분산 캐싱
end note

@enduml
```

---

## 다이어그램 렌더링 방법

### 1. PlantUML 온라인 에디터

- **URL**: https://www.plantuml.com/plantuml/uml/
- 위 다이어그램 코드를 복사하여 붙여넣기
- 자동으로 렌더링된 이미지 확인

### 2. VS Code 확장

```bash
# PlantUML 확장 설치
code --install-extension jebbs.plantuml

# Java 설치 (필요시)
sudo apt install default-jre

# Graphviz 설치 (필요시)
sudo apt install graphviz
```

**사용법**:
- `.puml` 또는 `.plantuml` 파일 생성
- `Alt+D` 키로 미리보기 실행

### 3. 명령줄 도구

```bash
# PlantUML JAR 다운로드
wget https://github.com/plantuml/plantuml/releases/download/v1.2024.0/plantuml-1.2024.0.jar

# PNG 이미지 생성
java -jar plantuml.jar diagram.puml

# SVG 벡터 이미지 생성
java -jar plantuml.jar -tsvg diagram.puml
```

### 4. Docker 사용

```bash
# PlantUML 서버 실행
docker run -d -p 8080:8080 plantuml/plantuml-server:jetty

# 브라우저에서 http://localhost:8080 접속
```

---

**문서 버전**: 1.0  
**최종 수정일**: 2026-02-13  
**관련 문서**: [ndt_factor_detailed_ko.md](./ndt_factor_detailed_ko.md)
