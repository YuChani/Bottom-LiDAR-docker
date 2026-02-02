/**
 * loam_feature.hpp - LOAM Feature Extraction
 * 
 * LOAM (Lidar Odometry and Mapping) 알고리즘의 특징점 추출 기능
 * - Ring 기반 특징 추출 (LOAM 논문 원본 방식)
 * - 3D KNN 기반 특징 추출 (백업용)
 */

#pragma once

#include <vector>
#include <memory>
#include <string>
#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <gtsam_points/types/point_cloud_cpu.hpp>

// ====================================================================
// PCL 커스텀 포인트 타입 정의 (Ring 포함)
// LiDAR의 몇번째 scan인지에 대한 Ring ID 정보 포함
// ====================================================================
struct PointXYZIR
{
  PCL_ADD_POINT4D;      // x, y, z, padding
  float intensity;
  uint16_t ring;        // Ring 정보
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (uint16_t, ring, ring)
)

// ====================================================================
// Ring 정보를 포함한 포인트 구조체 (내부 사용용)
// ====================================================================
struct PointWithRing
{
  Eigen::Vector4d point;  // x, y, z, 1 (homogeneous 좌표)
  uint16_t ring;          // Ring ID
  double angle;           // atan2(y, x) for sorting within ring
};

// ====================================================================
// LOAM 특징점 구조체
// ====================================================================
struct LOAMFeatures
{
  std::shared_ptr<gtsam_points::PointCloudCPU> edge_points;   // Edge 특징점 (높은 곡률)
  std::shared_ptr<gtsam_points::PointCloudCPU> planar_points; // Planar 특징점 (낮은 곡률)
};

// ====================================================================
// PCL을 사용한 PCD 읽기 함수
// Ring 정보를 포함하여 PCD 파일을 읽음
// ====================================================================
std::vector<PointWithRing> read_points_with_ring_pcl(const std::string& pcd_path);

// ====================================================================
// Ring 기반 LOAM feature 추출 (LOAM 논문 원본 방식)
// 
// LOAM 논문의 Curvature 공식:
//   c_i = ||Σ(X_i - X_j)|| / (|S| * ||X_i||)
// 
// 여기서:
//   - X_i: 현재 포인트
//   - X_j: 같은 Ring 내 ±5개 이웃 포인트
//   - |S|: 이웃 포인트 수 (10개)
// 
// Edge: 높은 곡률 (코너, 엣지 부분)
// Planar: 낮은 곡률 (평면 부분)
// ====================================================================
LOAMFeatures extract_loam_features_ring_based(const std::vector<PointWithRing>& points_with_ring);

// ====================================================================
// 3D KNN 기반 LOAM feature 추출 (백업용)
// 
// Ring 정보가 없는 경우 사용
// KD-Tree로 3D 공간에서 최근접 이웃 검색
// 주의: 다른 Ring의 포인트가 섞일 수 있어 정확도 낮음
// ====================================================================
LOAMFeatures extract_loam_features_knn(const gtsam_points::PointCloud::Ptr& cloud);

// ====================================================================
// Normal 벡터 기반 LOAM feature 추출 (휴리스틱 방식)
// 
// 법선 벡터의 방향 특성을 이용한 특징점 분류:
//   - Planar: 법선이 한 축에 거의 평행 (max_axis > 0.9)
//   - Edge: 법선이 여러 축에 분산 (max_axis < 0.7)
// 
// 장점: 빠른 처리 속도 (다운샘플링 적용)
// 단점: 휴리스틱 기반으로 정확도 낮을 수 있음
// ====================================================================
LOAMFeatures extract_loam_features_normal_based(const gtsam_points::PointCloud::Ptr& cloud);

