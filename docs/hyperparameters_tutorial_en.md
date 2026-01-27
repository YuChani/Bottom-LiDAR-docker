# GTSAM-Points Hyperparameter Tutorial

## Overview
This document organizes and explains all hyperparameters used in the point cloud registration demo code with the `gtsam_points` library.

**Demo Program**: `/root/workdir/src/main.cpp`  
**Purpose**: Compare ICP, GICP, VGICP algorithms and evaluate registration accuracy against Ground Truth

---

## 1. Data Configuration

### 1.1 Input Data
```cpp
const std::string data_path = "/root/workdir/data/pcd";
```

| Item | Value | Description |
|------|-------|-------------|
| **PCD Files** | 5 files | Sequential LiDAR frames |
| **Points per Frame** | ~54,400 | Each PCD file |
| **File Format** | Binary PCD | Custom parser without PCL |
| **Point Size** | 48 bytes | x,y,z + intensity + timestamp + ring, etc. |
| **Coordinate Frame** | LiDAR local frame | Sensor-based coordinates |

### 1.2 Ground Truth Data
```cpp
// gt-tum.txt: timestamp tx ty tz qx qy qz qw
std::map<double, gtsam::Pose3> gt_poses;
```

| Item | Value | Description |
|------|-------|-------------|
| **GT Poses** | 12,270 | TUM format |
| **Coordinate Frame** | World ← Base | Robot base frame |
| **Matching Method** | Timestamp-based | PCD filename → closest GT pose |

---

## 2. Extrinsic Calibration

### 2.1 T_base_lidar Transformation
```cpp
// Values from sensor.yaml
Eigen::Vector3d t_base_lidar(0.0, 0.0, 0.124);
Eigen::Quaterniond q_base_lidar(0.0, 0.0, 0.0, 1.0);  // w,x,y,z
gtsam::Pose3 T_base_lidar(gtsam::Rot3(q_base_lidar), t_base_lidar);
```

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Translation (x, y, z)** | `[0.0, 0.0, 0.124]` m | Base → LiDAR displacement |
| **Rotation (quaternion)** | `[w:0, x:0, y:0, z:1]` | 180° yaw rotation |
| **Rotation (ypr)** | `[180°, 0°, 0°]` | Yaw-Pitch-Roll |

### 2.2 Coordinate Transformation Chain
```cpp
W_T_L = W_T_B * T_base_lidar;           // World ← LiDAR
L0_T_Li = W_T_L_origin.inverse() * W_T_L;  // Relative pose w.r.t. first frame
```

| Transform | Meaning | Usage |
|-----------|---------|-------|
| **W_T_B** | World ← Base | Poses from gt-tum.txt |
| **T_base_lidar** | Base ← LiDAR | Sensor extrinsic calibration |
| **W_T_L** | World ← LiDAR | LiDAR absolute pose |
| **L0_T_Li** | LiDAR0 ← LiDARi | Relative pose for optimization |

---

## 3. Point Cloud Preprocessing

### 3.1 Covariance Estimation
```cpp
auto covs = gtsam_points::estimate_covariances(points);
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| **k_neighbors** | 10 | Number of neighbors for KNN search |
| **Algorithm** | PCA-based | Estimate local surface characteristics |

**Purpose**: Calculate uncertainty (covariance) for each point used in GICP

### 3.2 Normal Estimation
```cpp
frame->add_normals(gtsam_points::estimate_normals(frame->points, frame->size()));
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| **k_neighbors** | 10 | Number of neighbors for KNN search |
| **Method** | PCA smallest eigenvector | Local plane normal direction |

**Purpose**: Calculate surface normal vectors for Point-to-Plane ICP

### 3.3 Voxel Map Generation
```cpp
auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(0.5);
voxelmap->insert(*frame);
```

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Voxel Resolution** | `0.5` m | Side length of each voxel |
| **Data Structure** | Gaussian Voxel | Store mean and covariance per voxel |
| **Used By** | VGICP | Fast correspondence search |

---

## 4. ICP Algorithm Parameters

### 4.1 Supported Algorithms
```cpp
factor_types = {"ICP", "ICP_PLANE", "GICP", "VGICP", "VGICP_GPU"};
```

| Algorithm | Description | Features |
|-----------|-------------|----------|
| **ICP** | Point-to-Point | Most basic method |
| **ICP_PLANE** | Point-to-Plane | Uses normal vectors, robust to planes |
| **GICP** | Generalized ICP | Uses point covariances, high accuracy |
| **VGICP** | Voxelized GICP | Uses voxel maps, faster |
| **VGICP_GPU** | GPU-accelerated VGICP | Requires CUDA (currently disabled) |

### 4.2 ICP/ICP_PLANE/GICP Parameters
```cpp
auto factor = gtsam::make_shared<gtsam_points::IntegratedICPFactor>(...);
factor->set_correspondence_update_tolerance(correspondence_update_tolerance_rot, 
                                             correspondence_update_tolerance_trans);
factor->set_num_threads(num_threads);
```

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| **correspondence_update_tolerance_rot** | `0.0` rad | 0.0 - 0.1 | Rotation change threshold (0 = update every iteration) |
| **correspondence_update_tolerance_trans** | `0.0` m | 0.0 - 1.0 | Translation change threshold (0 = update every iteration) |
| **num_threads** | `1` | 1 - 128 | Number of parallel processing threads |
| **max_correspondence_distance** | `1.0` m | (default) | Maximum correspondence distance |

**correspondence_update_tolerance explanation**:
- Value `0.0`: Recalculate correspondences every iteration (accurate but slow)
- Larger value: Recalculate only when pose change is large (faster but less accurate)

### 4.3 VGICP-Specific Parameters
```cpp
auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(
    target_key, source_key, target_voxelmap, source);
factor->set_num_threads(num_threads);
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| **Voxel Resolution** | `0.5` m | Voxelmap resolution (smaller = more precise) |
| **num_threads** | `1` | Number of parallel processing threads |

**Note**: VGICP uses voxel maps, so correspondence_update_tolerance is not supported

---

## 5. Optimization Algorithm Parameters

### 5.1 Supported Optimizers
```cpp
optimizer_types = {"LM", "ISAM2"};
```

### 5.2 Levenberg-Marquardt (LM) Parameters
```cpp
gtsam_points::LevenbergMarquardtExtParams lm_params;
lm_params.maxIterations = 100;
lm_params.relativeErrorTol = 1e-5;
lm_params.absoluteErrorTol = 1e-5;
```

| Parameter | Value | Description |
|-----------|-------|-------------|
| **maxIterations** | `100` | Maximum number of iterations |
| **relativeErrorTol** | `1e-5` | Relative error convergence criterion |
| **absoluteErrorTol** | `1e-5` | Absolute error convergence criterion |

**Convergence conditions**:
- Relative Error < `1e-5` OR
- Absolute Error < `1e-5` OR
- Iterations ≥ `100`

### 5.3 iSAM2 Parameters
```cpp
gtsam::ISAM2Params isam2_params;
isam2_params.relinearizeSkip = 1;
isam2_params.setRelinearizeThreshold(0.0);
```

| Parameter | Value | Description |
|-----------|-------|-------------|
| **relinearizeSkip** | `1` | Relinearization period (1 = every update) |
| **relinearizeThreshold** | `0.0` | Relinearization threshold (0 = always relinearize) |
| **Additional iterations** | `5` | Extra optimization steps after update |

---

## 6. Factor Graph Construction

### 6.1 Prior Factor
```cpp
graph.add(gtsam::PriorFactor<gtsam::Pose3>(
    0, poses.at<gtsam::Pose3>(0), 
    gtsam::noiseModel::Isotropic::Precision(6, 1e6)));
```

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Key** | `0` | Fix first frame |
| **Precision** | `1e6` | Noise precision (very strong constraint) |
| **Dimension** | `6` | SE(3) DOF (3 translation + 3 rotation) |

### 6.2 Registration Factor Connection
```cpp
bool full_connection = true;
int j_end = full_connection ? 5 : std::min(i + 2, 5);
```

| Mode | Description | Factor Count (5 frames) |
|------|-------------|------------------------|
| **full_connection = true** | Connect all frame pairs | 10 (5C2) |
| **full_connection = false** | Connect adjacent frames only | 4 (sequential) |

**Example (full_connection = true)**:
```
Frame 0 ↔ Frame 1, 2, 3, 4  (4 factors)
Frame 1 ↔ Frame 2, 3, 4     (3 factors)
Frame 2 ↔ Frame 3, 4        (2 factors)
Frame 3 ↔ Frame 4           (1 factor)
Total: 10 factors
```

---

## 7. Visualization and UI Parameters

### 7.1 Pose Noise Addition
```cpp
float pose_noise_scale = 0.1;
```

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| **pose_noise_scale** | `0.1` | 0.0 - 10.0 | Random noise magnitude for initial poses (se(3) tangent space) |

**Purpose**: Test convergence performance of optimization algorithms

### 7.2 3D Viewer Settings
```cpp
viewer->enable_vsync();
auto drawable = guik::Rainbow().add("model_matrix", pose);
```

| Setting | Value | Description |
|---------|-------|-------------|
| **VSync** | Enabled | Screen refresh rate synchronization |
| **Coordinate Size** | 5.0 m | Display size of coordinate system per frame |
| **Factor Line Color** | Green (0, 1, 0) | Factor graph connection lines |

---

## 8. Logging Configuration (spdlog)

### 8.1 Logger Initialization
```cpp
auto console = spdlog::stdout_color_mt("demo");
spdlog::set_level(spdlog::level::info);
spdlog::set_pattern("[%^%l%$] %v");
```

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Log Level** | `info` | Output info and above (debug, info, warn, error) |
| **Pattern** | `[%^%l%$] %v` | Color + level + message |
| **Output** | stdout (colored) | Terminal color output |

### 8.2 Output Information
- PCD file loading status
- Timestamp matching results
- Relative pose per frame (translation)
- Optimization results (Optimized vs GT)
- Pose errors (translation error, rotation error)
- Average statistics (Mean Translation Error, Mean Rotation Error)

---

## 9. Hyperparameter Tuning Guide

### 9.1 Improving Accuracy
| Parameter | Adjustment | Effect |
|-----------|-----------|--------|
| **Voxel Resolution** | Decrease (0.5 → 0.2) | Finer matching, increased computation |
| **k_neighbors** | Increase (10 → 20) | More stable covariance/normal estimation |
| **maxIterations** | Increase (100 → 200) | More accurate convergence |
| **full_connection** | true | Optimize all frame pairs, improve global consistency |

### 9.2 Improving Speed
| Parameter | Adjustment | Effect |
|-----------|-----------|--------|
| **Voxel Resolution** | Increase (0.5 → 1.0) | Faster processing, reduced accuracy |
| **num_threads** | Increase (1 → 8) | Speed up with parallel processing |
| **correspondence_update_tolerance** | Increase (0.0 → 0.01) | Reduce correspondence recalculation frequency |
| **full_connection** | false | Connect adjacent frames only, reduce factor count |

### 9.3 Algorithm Selection Guide
| Scenario | Recommended | Reason |
|----------|------------|--------|
| **Real-time Processing** | VGICP | Fast with voxel maps |
| **Highest Accuracy** | GICP | Uses per-point covariances |
| **Planar Environments** | ICP_PLANE | Optimized for planes with normals |
| **Simple Testing** | ICP | Most basic, fastest execution |

---

## 10. Current Configuration Summary

### 10.1 Default Parameters (main.cpp)
```yaml
Data:
  frames: 5
  points_per_frame: ~54400
  
Calibration:
  T_base_lidar:
    translation: [0.0, 0.0, 0.124]
    rotation_ypr: [180°, 0°, 0°]

Preprocessing:
  k_neighbors_covariance: 10
  k_neighbors_normal: 10
  voxel_resolution: 0.5 m

ICP_Factors:
  correspondence_update_tolerance_rot: 0.0 rad
  correspondence_update_tolerance_trans: 0.0 m
  max_correspondence_distance: 1.0 m
  num_threads: 1

Optimization:
  optimizer: LM
  maxIterations: 100
  relativeErrorTol: 1e-5
  absoluteErrorTol: 1e-5

Graph:
  full_connection: true
  prior_precision: 1e6

Visualization:
  pose_noise_scale: 0.1
  coordinate_system_size: 5.0 m

Logging:
  level: info
  color_output: enabled
```

### 10.2 How to Run
```bash
cd /root/workdir/build
./calculate_rt
```

**UI Controls**:
1. `noise_scale` slider: Adjust initial pose noise
2. `add noise` button: Add noise to poses
3. `full connection` checkbox: Factor graph connection mode
4. `num threads` slider: Number of parallel threads
5. `factor type` dropdown: Select ICP/ICP_PLANE/GICP/VGICP
6. `optimizer type` dropdown: Select LM/ISAM2
7. `corr update tolerance` slider: Correspondence update threshold
8. `optimize` button: Run optimization

---

## 11. References

### 11.1 Code Locations
- **Main code**: `/root/workdir/src/main.cpp`
- **PCD parser**: `/root/workdir/include/gtsam_points/include/gtsam_points/util/read_points.hpp`
- **Data**: `/root/workdir/data/pcd/`

### 11.2 Library Documentation
- **gtsam_points**: https://github.com/koide3/gtsam_points
- **GTSAM**: https://gtsam.org/
- **Iridescence**: https://github.com/koide3/iridescence

### 11.3 Paper References
- **ICP**: Besl & McKay, 1992
- **Point-to-Plane ICP**: Chen & Medioni, 1992
- **GICP**: Segal et al., 2009
- **VGICP**: Koide et al., 2021

---

## Change Log
- **2026-01-26**: Initial document creation (based on main.cpp)
