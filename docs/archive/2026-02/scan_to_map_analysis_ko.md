# Scan-to-Map 구현 분석 및 재설계 제안

**작성일**: 2026년 2월 12일  
**프로젝트**: Bottom-LiDAR Point Cloud Registration  
**상태**: 🔴 현재 구현 문제 발견, 재설계 필요

---

## 📋 목차

1. [문제 상황](#문제-상황)
2. [현재 구현 분석](#현재-구현-분석)
3. [문제점 정리](#문제점-정리)
4. [올바른 이해](#올바른-이해)
5. [재설계 제안](#재설계-제안)
6. [구현 로드맵](#구현-로드맵)

---

## 문제 상황

### 사용자 피드백

> "지금 너가 수정한건 scan-to-map이라는 factor를 따로 추가해서 진행을 했는게 이거는 맞는 방식이 아닌거 같아. 다시 제대로 확인하고 내 코드에 대한 구조를 확실하게 이해하고 진행해."

### 실험 목표

- **현재**: 5개 PCD 파일만 사용하는 scan-to-scan 정합
- **목표**: 더 많은 PCD 파일로 확장 (scan-to-map 방식 적용)
- **알고리즘**: Point-to-Point, Point-to-Plane, GICP, VGICP, LOAM
- **비교 지표**: R (rotation), t (translation) 정확도

---

## 현재 구현 분석

### 기존 시스템 (main.cpp)

#### 알고리즘 5개: Scan-to-Scan 방식

```cpp
// create_factor() - main.cpp L380-445
gtsam::NonlinearFactor::shared_ptr create_factor(
    gtsam::Key target_key,  // 프레임 i
    gtsam::Key source_key,  // 프레임 j
    ...)
{
    if (factor_type == "GICP") {
        // Binary factor: 프레임 i ↔ 프레임 j
        return gtsam::make_shared<IntegratedGICPFactor>(
            target_key, source_key, target, source);
    }
    // ... Point-to-Point, Point-to-Plane, VGICP, LOAM도 동일 구조
}

// run_optimization() - main.cpp L447-614
void run_optimization() {
    gtsam::NonlinearFactorGraph graph;
    
    // 모든 프레임 쌍에 binary factor 추가
    for (int i = 0; i < num_frames; i++) {
        for (int j = i + 1; j < j_end; j++) {
            auto factor = create_factor(i, j, ...);
            graph.add(factor);
        }
    }
    
    // 전체 포즈 동시 최적화 (Batch Optimization)
    gtsam::Values optimized = optimizer.optimize();
}
```

**특징:**
- Binary factors (두 프레임 간 constraint)
- Batch optimization (모든 포즈 동시 최적화)
- Full connection or Sequential connection

---

### 추가된 구현 (scan_to_map.cpp)

#### "Scan-to-Map" 추가 시도

```cpp
// scan_to_map.cpp
class ScanToMapRegistration {
    ScanToMapResult register_frames(...) {
        global_map_ = std::make_shared<GaussianVoxelMapCPU>(0.5);
        
        for (i = 0 to N-1) {
            if (i == 0) {
                // 첫 프레임: 원점 고정
                global_map_->insert(*frame);
            } else {
                // ⚠️ 문제: "Unary VGICP Factor"를 "Scan-to-Map Factor"로 착각
                auto factor = IntegratedVGICPFactor(
                    Pose3::Identity(),  // 맵 포즈 (고정)
                    i,                  // 최적화할 포즈
                    global_map_,        // 글로벌 맵
                    frame);
                
                // 각 프레임마다 개별 최적화 (Sequential)
                graph.add(factor);
                optimized = optimizer.optimize();
                
                // 맵 업데이트
                global_map_->insert(*transformed_frame);
            }
        }
    }
};
```

#### main.cpp 통합

```cpp
// main.cpp L450-507
void run_optimization() {
    if (factor_types[factor_type] == "Scan-to-Map") {
        // ⚠️ 문제: 별도 경로로 분기
        ScanToMapRegistration scan_to_map_reg(0.5);
        auto result = scan_to_map_reg.register_frames(frames, poses);
        update_viewer(result.poses);
        return;  // 기존 batch optimization 건너뜀
    }
    
    // 기존 알고리즘: Batch optimization
    // ...
}
```

---

## 문제점 정리

### 1. 개념적 혼란: "Scan-to-Map Factor"는 존재하지 않음

#### ❌ 잘못된 이해

```
Point-to-Point Factor
Point-to-Plane Factor
GICP Factor
VGICP Factor
LOAM Factor
Scan-to-Map Factor  ← ❌ 이런 개념 없음!
```

#### ✅ 올바른 이해

**Factor Graph 용어:**
- **Node (Variable)**: 최적화할 변수 (예: 포즈 x₀, x₁, x₂, ...)
- **Factor (Constraint)**: 변수 간 관계 (예: x₁과 x₂ 사이의 relative pose)
- **Factor Type**: Binary (2개 변수), Unary (1개 변수), N-ary (N개 변수)

**Point Cloud Registration:**
- **Scan-to-Scan**: 두 스캔 간 정합
- **Scan-to-Map**: 스캔과 누적 맵 간 정합

**결론:**
```
Scan-to-Map은 registration 방법이지, factor type이 아닙니다!
```

---

### 2. 구조적 불일치

| 특성 | 기존 5개 알고리즘 | 추가한 "Scan-to-Map" |
|-----|-----------------|-------------------|
| **Target** | 개별 스캔 (frame j) | 누적 맵 (global_map) |
| **Factor Type** | Binary (i ↔ j) | Unary (i ↔ map) |
| **최적화 방식** | Batch (전체 동시) | Sequential (순차) |
| **Graph 구조** | 복수 노드 + 복수 factor | 각 프레임마다 독립 graph |
| **비교 가능성** | 서로 비교 가능 | ❌ 비교 불가 |

**문제:**
- 기존 알고리즘들과 **근본적으로 다른 프레임워크**
- "각각 비교하는 실험"이라는 목표 달성 불가

---

### 3. Production 코드 (A-LOAM)와의 차이

#### A-LOAM의 Scan-to-Map 방식 (laserMapping.cpp)

```cpp
// A-LOAM: 맵은 graph 밖에서 관리
for (int i = 0; i < num_frames; i++) {
    // 1. Scan-to-Map ICP (graph 밖에서 수행)
    Pose3 scan_to_map_result = icp_against_map(scan_i, local_map);
    
    // 2. 결과를 prior factor로 추가
    graph.add(PriorFactor(i, scan_to_map_result, noise_model));
    
    // 3. Scan-to-Scan factors도 함께 추가 (loop closure 등)
    graph.add(BinaryFactor(i-1, i, ...));
    
    // 4. 전체 최적화
    optimized_poses = optimizer.optimize();
    
    // 5. 맵 업데이트
    local_map += transform(scan_i, optimized_poses[i]);
}
```

**핵심 차이:**
- **A-LOAM**: Scan-to-map은 measurement 제공, factor graph는 여전히 scan-to-scan
- **현재 구현**: Scan-to-map을 factor graph에 직접 통합하려고 시도

---

### 4. 실험 목적과의 불일치

**사용자의 실험 목적:**
> "point-to-point, point-to-plane, g-icp, vg-icp, loam feature를 통해서 정합을 하고 이후 최적화를 통해 R,t를 구해서 각각 비교를 하는 실험"

**현재 상황:**
- 5개 PCD만 사용 → Drift 문제 적음
- 더 많은 PCD 사용하려면 → **Map 필요**

**하지만:**
- 기존 알고리즘: Scan-to-scan binary factors
- 추가한 방식: Map 기반 sequential optimization
- **비교가 공정하지 않음**

---

## 올바른 이해

### Scan-to-Map의 본질

#### 정의

**Scan-to-Map Registration:**
> 새로운 스캔을 이미 구축된 맵에 정합하여 포즈를 추정하는 과정

**Map의 역할:**
- **Reference Frame**: 정합의 target
- **Accumulated Knowledge**: 이전 관측 정보의 누적
- **Not a Graph Node**: 최적화 변수가 아님

#### Factor Graph에서의 위치

```
┌─────────────────────────────────────┐
│  Factor Graph (최적화 문제)           │
│                                     │
│  Variables: x₀, x₁, x₂, ..., xₙ     │
│  Factors: f₁(x₀,x₁), f₂(x₁,x₂), ... │
│                                     │
│  ⚠️ Map은 여기에 없음!               │
└─────────────────────────────────────┘

┌─────────────────────────────────────┐
│  Registration (측정 과정)            │
│                                     │
│  Input: Scan, Map                   │
│  Output: Relative Pose (measurement)│
│                                     │
│  이 결과가 factor의 입력으로 사용됨    │
└─────────────────────────────────────┘
```

---

### 기존 코드 구조의 올바른 해석

#### main.cpp의 설계 철학

```cpp
// main.cpp는 GTSAM Factor Graph 벤치마크
// 목적: 다양한 ICP 알고리즘을 Factor로 통합하여 비교

// 1. 데이터 로딩
for (i in frames) {
    frames[i] = load_pcd(pcd_files[i]);
    voxelmaps[i] = create_voxelmap(frames[i]);  // Target으로 사용
}

// 2. Factor 생성
for (i, j in pairs) {
    // 각 알고리즘별로 다른 factor type
    if (algorithm == "GICP") {
        factor = IntegratedGICPFactor(i, j, frames[i], frames[j]);
    } else if (algorithm == "VGICP") {
        factor = IntegratedVGICPFactor(i, j, voxelmaps[i], frames[j]);
    }
    // ...
    graph.add(factor);
}

// 3. 최적화
optimized_poses = optimizer.optimize();

// 4. 평가
for (i in frames) {
    error = compute_error(optimized_poses[i], ground_truth[i]);
}
```

**핵심:**
- **Frames와 Voxelmaps는 고정된 데이터**
- **최적화 대상은 Poses만**
- **Algorithm은 factor 내부의 error function만 바꿈**

---

## 재설계 제안

### 목표

1. **5개 PCD → 100개 PCD 확장**
2. **Drift 감소를 위한 Map 활용**
3. **기존 알고리즘 구조 유지 (공정한 비교)**
4. **코드 통일성 유지**

---

### 방안 1: Local Map을 Target으로 사용 (추천 ⭐)

#### 개념

기존 binary factor 구조를 유지하되, **target을 individual frame 대신 accumulated map으로 변경**

#### 구현

```cpp
// main.cpp 수정
void run_optimization() {
    gtsam::NonlinearFactorGraph graph;
    
    // Local map 구축
    std::vector<GaussianVoxelMap::Ptr> local_maps(num_frames);
    local_maps[0] = std::make_shared<GaussianVoxelMapCPU>(0.5);
    local_maps[0]->insert(*frames[0]);
    
    for (int i = 1; i < num_frames; i++) {
        // i번째 프레임의 local map = 이전 프레임들의 누적
        local_maps[i] = std::make_shared<GaussianVoxelMapCPU>(0.5);
        
        // 이전 N개 프레임 누적 (sliding window)
        int window_size = 10;  // 또는 adaptive
        for (int j = std::max(0, i - window_size); j < i; j++) {
            // 이미 추정된 포즈로 변환하여 추가
            Eigen::Isometry3d T_j(poses.at<Pose3>(j).matrix());
            std::vector<Eigen::Vector4d> transformed = transform_points(frames[j], T_j);
            local_maps[i]->insert(transformed);
        }
    }
    
    // Factor 생성 (기존과 동일한 구조!)
    for (int i = 1; i < num_frames; i++) {
        // Sequential connection: i-1 → i
        auto factor = create_factor(
            i-1,              // 이전 프레임 (또는 고정 reference)
            i,                // 현재 프레임
            local_maps[i],    // ← Target: Local map
            frames[i]         // Source: 현재 스캔
        );
        graph.add(factor);
    }
    
    // Batch optimization (기존 방식)
    gtsam::Values optimized = optimizer.optimize();
}
```

**장점:**
- ✅ 기존 factor 구조 유지
- ✅ Batch optimization 유지
- ✅ 모든 알고리즘에 동일하게 적용 가능
- ✅ 공정한 비교 가능

**단점:**
- Local map 구축 오버헤드
- 순환 참조 문제 (pose 추정에 사용한 pose로 map 구축)

---

### 방안 2: Incremental Optimization (순차 처리)

#### 개념

기존 batch optimization 대신 incremental 방식 채택

#### 구현

```cpp
void run_optimization_incremental() {
    gtsam::ISAM2 isam;
    GaussianVoxelMapCPU global_map(0.5);
    
    for (int i = 0; i < num_frames; i++) {
        if (i == 0) {
            // 첫 프레임: Prior factor
            graph.add(PriorFactor(0, Pose3::Identity(), ...));
            global_map.insert(*frames[0]);
        } else {
            // Binary factor: i-1 ↔ i
            auto factor = create_factor(
                i-1, i, 
                &global_map,  // 누적 맵을 reference로 사용
                frames[i]);
            graph.add(factor);
            
            // Incremental update
            isam.update(graph, initial_values);
            
            // 최적화된 포즈로 맵 업데이트
            Pose3 optimized_pose_i = isam.calculateEstimate().at<Pose3>(i);
            auto transformed = transform_points(frames[i], optimized_pose_i);
            global_map.insert(transformed);
        }
    }
}
```

**장점:**
- ✅ 실시간 SLAM에 가까운 방식
- ✅ 메모리 효율적
- ✅ Map 업데이트가 자연스러움

**단점:**
- ❌ 기존 batch optimization과 구조가 다름
- ❌ 5개 알고리즘 모두 수정 필요
- ❌ 비교 공정성 문제

---

### 방안 3: Hybrid - Batch with Map-based Initial Guess

#### 개념

Batch optimization은 유지하되, **initial guess를 scan-to-map으로 개선**

#### 구현

```cpp
void run_optimization_hybrid() {
    GaussianVoxelMapCPU global_map(0.5);
    gtsam::Values improved_initial_poses;
    
    // Phase 1: Scan-to-Map으로 초기 추정치 개선
    for (int i = 0; i < num_frames; i++) {
        if (i == 0) {
            improved_initial_poses.insert(0, poses_gt.at<Pose3>(0));
            global_map.insert(*frames[i]);
        } else {
            // Scan-to-map ICP
            Pose3 refined_pose = scan_to_map_icp(
                frames[i], 
                &global_map, 
                poses_gt.at<Pose3>(i)  // GT를 initial guess로
            );
            improved_initial_poses.insert(i, refined_pose);
            
            auto transformed = transform_points(frames[i], refined_pose);
            global_map.insert(transformed);
        }
    }
    
    // Phase 2: Batch optimization (기존 방식)
    gtsam::NonlinearFactorGraph graph;
    for (int i = 0; i < num_frames; i++) {
        for (int j = i + 1; j < j_end; j++) {
            auto factor = create_factor(i, j, frames[i], frames[j]);
            graph.add(factor);
        }
    }
    
    // 개선된 initial guess 사용
    gtsam::Values optimized = optimizer.optimize(improved_initial_poses);
}
```

**장점:**
- ✅ 기존 구조 완전 보존
- ✅ 수렴 속도 향상
- ✅ Drift 감소

**단점:**
- ⚠️ Initial guess의 영향력이 커짐
- ⚠️ "알고리즘 비교"의 순수성 저하

---

### 추천: 방안 1 (Local Map as Target)

#### 이유

1. **실험 목적과 부합**
   - 각 알고리즘의 특성 비교 (error function 차이)
   - Map 활용으로 drift 감소
   - 기존 구조 유지

2. **구현 용이성**
   - `create_factor()` 함수 수정만 필요
   - `run_optimization()` 구조 변경 최소화

3. **확장성**
   - Sliding window 크기 조절 가능
   - Loop closure 추가 용이

---

## 구현 로드맵

### Phase 1: 기존 코드 정리 (1-2시간)

#### 작업 내용
- [ ] `scan_to_map.cpp`, `scan_to_map.hpp` 제거 또는 별도 보관
- [ ] `main.cpp`의 "Scan-to-Map" 분기 제거
- [ ] Factor type을 5개로 복원

#### 목표
기존 scan-to-scan 방식으로 되돌리기

---

### Phase 2: Local Map 구축 로직 추가 (2-3시간)

#### 작업 내용

1. **LocalMapBuilder 클래스 생성**

```cpp
// include/local_map_builder.hpp
class LocalMapBuilder {
public:
    LocalMapBuilder(double voxel_resolution, int window_size);
    
    // 프레임 추가 (pose는 현재 추정치)
    void add_frame(int frame_id, 
                   const PointCloud::Ptr& frame,
                   const gtsam::Pose3& pose);
    
    // i번째 프레임을 위한 local map 생성
    GaussianVoxelMap::Ptr get_local_map(int frame_id);
    
private:
    double voxel_resolution_;
    int window_size_;
    std::map<int, PointCloud::Ptr> frames_;
    std::map<int, gtsam::Pose3> poses_;
};
```

2. **main.cpp 통합**

```cpp
// main.cpp
void run_optimization() {
    LocalMapBuilder map_builder(0.5, 10);  // 10-frame sliding window
    
    // 첫 번째 최적화: Ground truth 기반 local map 구축
    for (int i = 0; i < num_frames; i++) {
        map_builder.add_frame(i, frames[i], poses_gt.at<Pose3>(i));
    }
    
    // Factor 생성
    for (int i = 1; i < num_frames; i++) {
        auto local_map = map_builder.get_local_map(i);
        
        auto factor = create_factor(
            i-1, i,
            local_map,  // ← Local map as target
            frames[i]
        );
        graph.add(factor);
    }
    
    // Batch optimization
    gtsam::Values optimized = optimizer.optimize();
}
```

---

### Phase 3: 반복 최적화 (Iterative Refinement) (3-4시간)

#### 작업 내용

Local map을 최적화된 pose로 재구축하여 정확도 향상

```cpp
void run_optimization_iterative() {
    const int max_iterations = 3;
    gtsam::Values current_poses = poses_gt;  // Initial guess
    
    for (int iter = 0; iter < max_iterations; iter++) {
        spdlog::info("=== Iteration {} ===", iter);
        
        // 1. 현재 pose로 local map 구축
        LocalMapBuilder map_builder(0.5, 10);
        for (int i = 0; i < num_frames; i++) {
            map_builder.add_frame(i, frames[i], current_poses.at<Pose3>(i));
        }
        
        // 2. Factor graph 구축
        gtsam::NonlinearFactorGraph graph;
        for (int i = 1; i < num_frames; i++) {
            auto local_map = map_builder.get_local_map(i);
            auto factor = create_factor(i-1, i, local_map, frames[i]);
            graph.add(factor);
        }
        
        // 3. 최적화
        current_poses = optimizer.optimize(current_poses);
        
        // 4. 수렴 체크
        if (pose_change < threshold) break;
    }
    
    return current_poses;
}
```

---

### Phase 4: 실험 및 평가 (2-3시간)

#### 작업 내용

1. **5개 알고리즘 × Local Map 테스트**
   - Point-to-Point + Local Map
   - Point-to-Plane + Local Map
   - GICP + Local Map
   - VGICP + Local Map
   - LOAM + Local Map

2. **평가 지표**
   - ATE (Absolute Trajectory Error)
   - RPE (Relative Pose Error)
   - 처리 시간

3. **결과 문서화**
   - 각 알고리즘별 성능 비교 표
   - 궤적 시각화
   - 에러 분포 그래프

---

## 요약

### 현재 문제

- ❌ "Scan-to-Map Factor"라는 잘못된 개념
- ❌ 기존 5개 알고리즘과 구조적 불일치
- ❌ 비교 불가능한 실험 설계

### 올바른 이해

- ✅ Scan-to-Map은 registration 방법, factor type 아님
- ✅ Map은 graph의 node가 아님
- ✅ Map은 registration의 target (reference frame)

### 재설계 방향

**추천: Local Map을 Target으로 사용하는 방식**

```
기존: Frame_i ↔ Frame_j
수정: Local_Map_i ↔ Frame_j

Factor 구조는 유지, Target만 변경
```

### 구현 우선순위

1. ⭐ **Phase 1**: 기존 코드 정리 (즉시 시작)
2. ⭐ **Phase 2**: Local Map 구축 (핵심)
3. ⭐⭐ **Phase 3**: 반복 최적화 (정확도 향상)
4. ⭐⭐⭐ **Phase 4**: 실험 및 평가 (최종 목표)

---

**다음 단계**: Phase 1 실행 여부 확인 필요

사용자 승인 후 구현 시작할 준비 완료.
