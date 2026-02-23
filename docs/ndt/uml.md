# NDT Factor UML 다이어그램

**작성일**: 2026-02-19  
**대상**: `gtsam_points` 라이브러리에 통합된 NDT Factor 구현 (NdtCorrespondence 기반)

> **참고**: 모든 다이어그램은 [PlantUML](https://www.plantuml.com/plantuml/uml/)에 붙여넣어 렌더링할 수 있다.

---

## 1. 클래스 상속 계층도

NDT Factor는 GICP/VGICP와 동일한 추상 기반 클래스 `IntegratedMatchingCostFactor`를 상속한다.

```plantuml
@startuml class_hierarchy
skinparam linetype ortho
skinparam classAttributeIconSize 0

title gtsam_points Factor 상속 계층

package "gtsam" #DDDDDD {
  abstract class NonlinearFactor {
    #keys_ : KeyVector
    +{abstract} error(values) : double
    +{abstract} linearize(values) : GaussianFactor::shared_ptr
    +{abstract} dim() : size_t
  }
}

package "gtsam_points" #E8F5E9 {
  abstract class IntegratedMatchingCostFactor {
    #is_binary : bool
    #fixed_target_pose : Isometry3d
    +error(values) : double
    +linearize(values) : GaussianFactor::shared_ptr
    +dim() : size_t {return 6}
    #{abstract} update_correspondences(delta)
    #{abstract} evaluate(delta, H*, b*) : double
    +calc_delta(values) : Isometry3d
  }

  class "IntegratedNDTFactor_<SourceFrame>" as NDT <<신규 구현>> {
    -target_voxels : shared_ptr<GaussianVoxelMapCPU>
    -source : shared_ptr<SourceFrame>
    -correspondences : vector<NdtCorrespondence>
    -num_threads : int
    -resolution : double
    -outlier_ratio : double
    -regularization_epsilon : double
    -search_mode : NDTSearchMode
    -gauss_d1, gauss_d2 : double
    __
    +set_num_threads(n)
    +set_resolution(r)
    +set_outlier_ratio(ratio)
    +set_regularization_epsilon(eps)
    +set_search_mode(mode)
    +num_inliers() : int
    +inlier_fraction() : double
    #update_correspondences(delta)
    #evaluate(delta, H*, b*) : double
  }

  class "IntegratedVGICPFactor_<SourceFrame>" as VGICP {
    -target_voxels : shared_ptr<GaussianVoxelMapCPU>
    -source : shared_ptr<SourceFrame>
    -correspondences : vector<GaussianVoxel*>
    #update_correspondences(delta)
    #evaluate(delta, H*, b*) : double
  }

  class "IntegratedGICPFactor_<T, S>" as GICP {
    #update_correspondences(delta)
    #evaluate(delta, H*, b*) : double
  }

  class IntegratedICPFactor {
    #update_correspondences(delta)
    #evaluate(delta, H*, b*) : double
  }

  class IntegratedLOAMFactor {
    #update_correspondences(delta)
    #evaluate(delta, H*, b*) : double
  }
}

NonlinearFactor <|-- IntegratedMatchingCostFactor
IntegratedMatchingCostFactor <|-- NDT
IntegratedMatchingCostFactor <|-- VGICP
IntegratedMatchingCostFactor <|-- GICP
IntegratedMatchingCostFactor <|-- IntegratedICPFactor
IntegratedMatchingCostFactor <|-- IntegratedLOAMFactor

note right of NDT #FFFFCC
  **NDT vs VGICP 핵심 차이**
  VGICP: GaussianVoxel*를 직접 참조
  NDT: NdtCorrespondence에 inv_cov 캐싱
  (GaussianVoxel에는 inv_cov 필드 없음)
end note
@enduml
```

---

## 2. NDT Factor 데이터 구조 관계도

```plantuml
@startuml data_structures
skinparam classAttributeIconSize 0

title NDT Factor 데이터 구조 관계

class "IntegratedNDTFactor_<SourceFrame>" as NDT {
  -target_voxels : shared_ptr<GaussianVoxelMapCPU>
  -source : shared_ptr<SourceFrame>
  -correspondences : vector<NdtCorrespondence>
  -search_mode : NDTSearchMode
  -resolution : double
  -outlier_ratio : double
  -regularization_epsilon : double
  -gauss_d1, gauss_d2 : double
}

class NdtCorrespondence <<struct>> #FFE0B2 {
  +mean : Vector4d
  +inv_cov : Matrix4d
  +valid : bool
}

class GaussianVoxelMapCPU {
  -voxels_ : unordered_map<coord, GaussianVoxel>
  -voxel_resolution_ : double
  +insert(frame)
  +lookup_voxel(index) : GaussianVoxel&
  +lookup_voxel_index(coord) : int
  +voxel_coord(point) : Vector3i
  +num_voxels() : size_t
}

class GaussianVoxel {
  +mean : Vector4d
  +cov : Matrix4d
  +num_points : int
}

class PointCloud {
  +points : vector<Vector4d>
  +covs : vector<Matrix4d>
  +normals : vector<Vector4d>
  +size() : size_t
}

enum NDTSearchMode {
  DIRECT1  (1개 복셀)
  DIRECT7  (7개 복셀, 기본값)
  DIRECT27 (27개 복셀)
}

NDT *-right-> GaussianVoxelMapCPU : target_voxels
NDT *-left-> PointCloud : source
NDT *-down-> "0..*" NdtCorrespondence : correspondences
NDT o--> NDTSearchMode : search_mode

GaussianVoxelMapCPU o--> "0..*" GaussianVoxel : contains

note bottom of NdtCorrespondence #FFFFCC
  **핵심 설계 결정**
  GaussianVoxel에는 inv_cov가 없으므로
  NdtCorrespondence가 voxel.cov로부터
  계산된 정규화 역공분산을 캐싱한다.

  mean ← GaussianVoxel.mean
  inv_cov ← compute_ndt_inverse_covariance(voxel.cov)
end note

note bottom of GaussianVoxel
  cov만 존재, inv_cov 없음!
  NDT에서 필요한 역공분산은
  NdtCorrespondence에서 관리
end note
@enduml
```

---

## 3. 유틸리티 함수 관계도

```plantuml
@startuml utility_functions
skinparam classAttributeIconSize 0

title NDT 유틸리티 함수 (integrated_ndt_factor.hpp)

package "gtsam_points 네임스페이스" #E8F5E9 {

  class "compute_ndt_inverse_covariance()" as InvCov <<free function>> {
    입력: cov (Matrix4d), epsilon (double)
    출력: Matrix4d (정규화 역공분산)
    __
    1. 고유값 분해 (SelfAdjointEigenSolver)
    2. 소값 클램핑: max(λ_i, ε * λ_max)
    3. 정규화 공분산 재구성
    4. 역행렬 반환
  }

  class "compute_ndt_params()" as Params <<free function>> {
    입력: resolution, outlier_ratio
    출력: d1 (음수), d2 (양수)
    __
    Magnusson 2009, Eq. 6.9-6.10
    c1 = 10 * (1 - outlier_ratio)
    c2 = outlier_ratio / resolution³
    d3 = -log(c2)
    d1 = -log(c1 + c2) - d3
    d2 = -2 * log((-log(c1*exp(-0.5)+c2) - d3) / d1)
  }

  class NdtCorrespondence <<struct>> {
    mean : Vector4d
    inv_cov : Matrix4d
    valid : bool
  }
}

InvCov .right.> NdtCorrespondence : inv_cov 필드에\n결과 저장
Params ..> "IntegratedNDTFactor_" : gauss_d1, gauss_d2에\n결과 저장

note bottom of InvCov
  **성능 최적화**
  update_correspondences()에서
  복셀당 1회만 호출 (사전 캐싱)
  → O(num_voxels) eigendecomp
  (O(num_points × num_neighbors) 아님)
end note
@enduml
```

---

## 4. 실행 시퀀스 다이어그램

```plantuml
@startuml sequence
skinparam sequenceMessageAlign center

title NDT Factor 실행 흐름

actor "main.cpp" as Main
participant "IntegratedNDTFactor_" as NDT
participant "GaussianVoxelMapCPU" as VoxelMap
participant "NdtCorrespondence[]" as Corr
participant "LM Optimizer" as LM

== 1. 팩터 생성 ==

Main -> NDT : new(target_key, source_key,\ntarget_voxels, source)
activate NDT
NDT -> NDT : resolution=1.0, outlier_ratio=0.55
NDT -> NDT : search_mode=DIRECT7
NDT --> Main : factor
deactivate NDT

Main -> NDT : set_search_mode(DIRECT7)
Main -> NDT : set_num_threads(4)

== 2. 최적화 루프 ==

Main -> LM : optimize(graph, initial_values)
activate LM

loop 각 LM iteration

  LM -> NDT : linearize(values)
  activate NDT

  NDT -> NDT : delta = calc_delta(values)

  == 2a. update_correspondences(delta) ==

  NDT -> NDT : compute_ndt_params()\n→ gauss_d1, gauss_d2

  NDT -> VoxelMap : num_voxels()
  activate VoxelMap
  VoxelMap --> NDT : N
  deactivate VoxelMap

  loop v = 0..N-1 (복셀별 역공분산 사전 캐싱)
    NDT -> VoxelMap : lookup_voxel(v).cov
    NDT -> NDT : inv_cov_cache[v] =\ncompute_ndt_inverse_covariance(cov)
  end

  loop i = 0..source.size()-1 (OpenMP 병렬)
    NDT -> NDT : pt = delta * source[i]
    NDT -> VoxelMap : voxel_coord(pt)
    loop 7개 이웃 (DIRECT7)
      NDT -> VoxelMap : lookup_voxel_index(neighbor)
      NDT -> NDT : 마할라노비스 거리 비교
    end
    NDT -> Corr : [i] = {mean, inv_cov, valid=true}
  end

  == 2b. evaluate(delta, H*, b*) ==

  loop i = 0..source.size()-1 (병렬 리듀스)
    NDT -> Corr : corr = [i]
    NDT -> NDT : residual = mean_B - delta*mean_A
    NDT -> NDT : mahal = r^T * inv_cov * r
    NDT -> NDT : e_term = exp(-d2/2 * mahal)
    NDT -> NDT : error = -d1 * (1 - e_term)
    NDT -> NDT : J_target, J_source (Lie algebra)
    NDT -> NDT : scale = -d1 * d2 * e_term
    NDT -> NDT : H += scale * J^T * inv_cov * J
    NDT -> NDT : b += scale * J^T * inv_cov * r
  end

  NDT --> LM : HessianFactor(H, b, error)
  deactivate NDT

  LM -> LM : Gauss-Newton step
  LM -> LM : values 갱신

end

LM --> Main : optimized values
deactivate LM

Main -> NDT : num_inliers()
NDT --> Main : count

@enduml
```

---

## 5. 패키지/파일 의존성 다이어그램

```plantuml
@startuml file_dependency
skinparam componentStyle uml2

title NDT Factor 파일 의존성

package "사용자 코드" #DDDDDD {
  [src/main.cpp] as main
}

package "gtsam_points/factors" #E8F5E9 {
  [integrated_ndt_factor.hpp] as ndt_hpp
  [impl/integrated_ndt_factor_impl.hpp] as ndt_impl
  [integrated_ndt_factor.cpp] as ndt_cpp
  [integrated_matching_cost_factor.hpp] as base_hpp
  [impl/scan_matching_reduction.hpp] as reduction
}

package "gtsam_points/types" #E3F2FD {
  [point_cloud.hpp] as pc
  [gaussian_voxelmap_cpu.hpp] as voxelmap
  [dummy_frame.hpp] as dummy
}

package "gtsam_points/util" #FFF3E0 {
  [parallelism.hpp] as parallel
  [gtsam_migration.hpp] as migration
}

package "gtsam" #F3E5F5 {
  [geometry/Pose3.h] as pose3
  [linear/HessianFactor.h] as hessian
  [nonlinear/NonlinearFactor.h] as nlfactor
}

package "Eigen" #FFEBEE {
  [Core] as eigen_core
  [Eigenvalues] as eigen_eig
}

main --> ndt_hpp : #include

ndt_hpp --> base_hpp
ndt_hpp --> pc
ndt_hpp --> voxelmap
ndt_hpp --> migration
ndt_hpp --> nlfactor
ndt_hpp --> eigen_core
ndt_hpp --> eigen_eig

ndt_impl --> ndt_hpp
ndt_impl --> pose3
ndt_impl --> hessian
ndt_impl --> voxelmap
ndt_impl --> parallel
ndt_impl --> reduction

ndt_cpp --> pc
ndt_cpp --> ndt_hpp
ndt_cpp --> ndt_impl
ndt_cpp --> dummy

base_hpp --> nlfactor

note right of ndt_hpp
  클래스 선언 + 유틸리티 함수
  NdtCorrespondence 정의
  compute_ndt_inverse_covariance()
  compute_ndt_params()
  NDTSearchMode enum
end note

note right of ndt_impl
  update_correspondences() 구현
  evaluate() 구현
  OpenMP/TBB 병렬화
end note

note right of ndt_cpp
  PointCloud, DummyFrame
  명시적 템플릿 인스턴스화
end note
@enduml
```

---

## 6. 컴포넌트 다이어그램 — Main과의 통합

```plantuml
@startuml integration
skinparam componentStyle uml2

title Main 애플리케이션 ↔ NDT Factor 통합

component "main.cpp\nMatchingCostFactorDemo" as demo #DDDDDD {
  [create_factor()] as cf
  [run_optimization()] as opt
}

component "gtsam_points 라이브러리" as lib #E8F5E9 {
  component "NDT Factor" as ndt_module {
    [IntegratedNDTFactor_]
    [NdtCorrespondence]
    [compute_ndt_inverse_covariance()]
    [compute_ndt_params()]
  }
  component "Types" as types {
    [PointCloud]
    [GaussianVoxelMapCPU]
    [GaussianVoxel]
  }
  component "Base" as base {
    [IntegratedMatchingCostFactor]
  }
}

component "GTSAM" as gtsam #F3E5F5 {
  [NonlinearFactorGraph]
  [LevenbergMarquardtOptimizer]
  [ISAM2]
  [Values]
}

cf --> [IntegratedNDTFactor_] : 생성
cf --> [PointCloud] : source 전달
cf --> [GaussianVoxelMapCPU] : target 전달

[IntegratedNDTFactor_] --> [IntegratedMatchingCostFactor] : 상속
[IntegratedNDTFactor_] --> [NdtCorrespondence] : 대응점 캐싱
[IntegratedNDTFactor_] --> [GaussianVoxel] : cov 읽기
[NdtCorrespondence] <.. [compute_ndt_inverse_covariance()] : inv_cov 계산

opt --> [NonlinearFactorGraph] : factor 추가
opt --> [LevenbergMarquardtOptimizer] : 최적화 실행
opt --> [Values] : 포즈 관리

note right of ndt_module
  factor_type == 5 ("NDT")일 때
  create_factor()에서 생성

  기본 설정:
  - search_mode = DIRECT7
  - resolution = 1.0
  - outlier_ratio = 0.55
end note
@enduml
```

---

## 다이어그램 렌더링 방법

### 온라인
- https://www.plantuml.com/plantuml/uml/ 에 코드 블록 내용을 붙여넣기

### VS Code
```bash
code --install-extension jebbs.plantuml
# Alt+D 로 미리보기
```

### CLI
```bash
java -jar plantuml.jar -tsvg uml.md
```

---

**최종 수정일**: 2026-02-19  
**관련 문서**: [README.md](./README.md)
