#pragma once

#include <vector>
#include <memory>
#include <string>
#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <gtsam_points/types/point_cloud_cpu.hpp>


// PCL 커스텀 포인트 타입 정의 (Ring 포함)
// LiDAR의 몇번째 scan인지에 대한 Ring ID 정보 포함

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

// Ring 정보를 포함한 포인트 구조체 (내부 사용용)
struct PointWithRing
{
  Eigen::Vector4d point;  // x, y, z, 1 (homogeneous 좌표)
  uint16_t ring;          // Ring ID
  double angle;           // atan2(y, x) for sorting within ring
};


// LOAM 특징점 구조체
struct LOAMFeatures
{
  std::shared_ptr<gtsam_points::PointCloudCPU> edge_points;   // Edge 특징점 (높은 곡률)
  std::shared_ptr<gtsam_points::PointCloudCPU> planar_points; // Planar 특징점 (낮은 곡률)
};


// PCL을 사용한 PCD 읽기 함수
// Ring 정보를 포함하여 PCD 파일을 읽음
std::vector<PointWithRing> read_points_with_ring_pcl(const std::string& pcd_path);


// Ring 기반 LOAM feature 추출 (LIO-SAM 방식 채용) 이유는 아래와 같음:
// 1. Ring 정보 활용: 각 Ring 별로 포인트를 나누어 처리
// 2. 동일 Ring 내에서 최근접 이웃 검색: 2D 방식으로 곡률 계산
// 3. 정확도 향상: 다른 Ring의 포인트가 섞이지 않아 더 정확한 특징점 추출 가능
// Edge: 높은 곡률 (코너, 엣지 부분)
// Planar: 낮은 곡률 (평면 부분)
LOAMFeatures extract_loam_features_ring_based(const std::vector<PointWithRing>& points_with_ring);


// 3D KNN 기반 LOAM feature 추출 (백업용)
// Ring 정보가 없는 경우 사용
// KD-Tree로 3D 공간에서 최근접 이웃 검색
// 주의: 다른 Ring의 포인트가 섞일 수 있어 정확도 낮음
LOAMFeatures extract_loam_features_knn(const gtsam_points::PointCloud::Ptr& cloud);


