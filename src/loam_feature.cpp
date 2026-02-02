/**
 * loam_feature.cpp - LOAM Feature Extraction Implementation
 * 
 * LOAM (Lidar Odometry and Mapping) 알고리즘의 특징점 추출 기능 구현
 * - Ring 기반 특징 추출 (LOAM 논문 원본 방식)
 * - KNN 기반 특징 추출 (3D 공간 기반)
 * - Normal 벡터 기반 특징 추출 (휴리스틱 방식)
 */

#include "loam_feature.hpp"
#include <spdlog/spdlog.h>
#include <gtsam_points/ann/kdtree.hpp>
#include <gtsam_points/features/normal_estimation.hpp>
#include <algorithm>
#include <map>

/**
 * @brief PCL을 사용한 PCD 읽기 함수
 * @param pcd_path PCD 파일 경로
 * @return Ring 정보를 포함한 포인트 벡터
 */
std::vector<PointWithRing> read_points_with_ring_pcl(const std::string& pcd_path)
{
  std::vector<PointWithRing> points;
  
  pcl::PointCloud<PointXYZIR>::Ptr cloud(new pcl::PointCloud<PointXYZIR>);
  
  if (pcl::io::loadPCDFile<PointXYZIR>(pcd_path, *cloud) == -1)
  {
    spdlog::error("Failed to load PCD file with PCL: {}", pcd_path);
    return points;
  }
  
  points.reserve(cloud->size());
  
  for (const auto& p : cloud->points)
  {
    // NaN 및 원점 필터링
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) 
    {
      continue;
    }
    double dist = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
    if (dist < 0.5 || dist > 150.0) 
    {
      continue;
    }
    
    PointWithRing pt;
    pt.point = Eigen::Vector4d(p.x, p.y, p.z, 1.0);
    pt.ring = p.ring;
    pt.angle = std::atan2(p.y, p.x);
    
    points.push_back(pt);
  }
  
  spdlog::info("PCL: Read {} points with ring info from {}", points.size(), pcd_path);
  return points;
}

/**
 * @brief Ring 기반 LOAM feature 추출 (LOAM 논문 원본 방식)
 * 
 * Step 1: Ring별로 포인트 그룹화 ex) 16채널 LiDAR면 16개 벡터 배열 생성
 * Step 2: 각 Ring 내에서 angle 기준으로 정렬. 배열의 인덱스가 물리적으로 옆에있는 point와 가깝도록 함
 * Step 3: Ring 기반 curvature 계산
 * 
 * LOAM Curvature: c = ||Σ(Xi - Xj)|| / (|S| * ||Xi||)
 * 
 * @param points_with_ring Ring 정보를 포함한 포인트 벡터
 * @return Edge와 Planar 특징점
 */
LOAMFeatures extract_loam_features_ring_based(const std::vector<PointWithRing>& points_with_ring)
{
  LOAMFeatures features;
  features.edge_points = std::make_shared<gtsam_points::PointCloudCPU>();
  features.planar_points = std::make_shared<gtsam_points::PointCloudCPU>();

  if (points_with_ring.size() < 100)
  {
    spdlog::warn("Too few points for LOAM feature extraction: {}", points_with_ring.size());
    return features;
  }

  // Step 1: Ring별로 포인트 그룹화 ex) 16채널 LiDAR면 16개 벡터 배열 생성
  std::map<uint16_t, std::vector<size_t>> ring_groups;
  for (size_t i = 0; i < points_with_ring.size(); i++)
  {
    ring_groups[points_with_ring[i].ring].push_back(i);
  }
  
  spdlog::info("Found {} rings in point cloud", ring_groups.size());

  // Step 2: 각 Ring 내에서 angle 기준으로 정렬. 배열의 인덱스가 물리적으로 옆에있는 point와 가깝도록 함
  for (auto& [ring_id, indices] : ring_groups)
  {
    std::sort(indices.begin(), indices.end(), [&points_with_ring](size_t a, size_t b)
    {
      return points_with_ring[a].angle < points_with_ring[b].angle;
    });
  }

  // Step 3: Ring 기반 curvature 계산
  // 주의: Occlusion 필터링은 너무 공격적일 수 있어 비활성화
  std::vector<std::pair<size_t, float>> index_curvatures;
  index_curvatures.reserve(points_with_ring.size());
  
  const int half_neighbor = 5;  // LOAM 논문: 좌우 5개씩 = 총 10개 이웃
  
  for (const auto& [ring_id, indices] : ring_groups)
  {
    int n = indices.size();
    if (n < half_neighbor * 2 + 1) continue;
    
    // Curvature 계산
    for (int i = half_neighbor; i < n - half_neighbor; i++)
    {
      size_t curr_idx = indices[i];
      Eigen::Vector3d X_i = points_with_ring[curr_idx].point.head<3>();
      
      // 같은 Ring 내에서 ±5개 이웃과의 차이 벡터 합산
      Eigen::Vector3d diff_sum = Eigen::Vector3d::Zero();
      for (int j = -half_neighbor; j <= half_neighbor; j++)
      {
        if (j == 0) continue;
        size_t neighbor_idx = indices[i + j];
        Eigen::Vector3d X_j = points_with_ring[neighbor_idx].point.head<3>();
        // planar : 0에 가까움, edge : 큼
        diff_sum += (X_i - X_j);
      }
      
      // LOAM Curvature: c = ||Σ(Xi - Xj)|| / (|S| * ||Xi||)
      double xi_norm = X_i.norm();
      double c = (xi_norm > 0.1) ? diff_sum.norm() / (2 * half_neighbor * xi_norm) : 0.0;
      
      index_curvatures.emplace_back(curr_idx, static_cast<float>(c));
    }
  }
  
  if (index_curvatures.empty())
  {
    spdlog::warn("No valid curvatures computed");
    return features;
  }
  
  // 디버깅: curvature 분포 확인
  float min_c = index_curvatures[0].second, max_c = index_curvatures[0].second;
  double sum_c = 0.0;
  for (const auto& [idx, c] : index_curvatures)
  {
    min_c = std::min(min_c, c);
    max_c = std::max(max_c, c);
    sum_c += c;
  }
  spdlog::info("Curvature stats: min={:.6f}, max={:.6f}, mean={:.6f}",
                min_c, max_c, sum_c / index_curvatures.size());

  // Step 4: Curvature 기준으로 정렬 (오름차순)
  std::sort(index_curvatures.begin(), index_curvatures.end(),
            [](const auto& a, const auto& b) { return a.second < b.second; });

  // Step 5: 비율 기반 Edge/Planar 선택 (간단하고 안정적)
  const int max_edge = 2000;
  const int max_planar = 4000;
  
  std::vector<Eigen::Vector4d> edge_pts, planar_pts;
  
  int num_valid = index_curvatures.size();
  
  // Planar: 하위 10% (낮은 곡률)
  int planar_count = std::min(max_planar, (int)(num_valid * 0.10));
  for (int i = 0; i < planar_count; i++)
  {
    size_t idx = index_curvatures[i].first;
    planar_pts.push_back(points_with_ring[idx].point);
  }
  
  // Edge: 상위 5% (높은 곡률)  
  int edge_count = std::min(max_edge, (int)(num_valid * 0.05));
  for (int i = num_valid - edge_count; i < num_valid; i++)
  {
    size_t idx = index_curvatures[i].first;
    edge_pts.push_back(points_with_ring[idx].point);
  }
  
  features.edge_points->add_points(edge_pts);
  features.planar_points->add_points(planar_pts);
  
  spdlog::info("LOAM features (Ring-based): {} edge, {} planar from {} points",
                edge_pts.size(), planar_pts.size(), points_with_ring.size());
  
  return features;
}

/**
 * @brief 기존 LOAM feature 추출 함수 (3D KNN 기반 - 백업용)
 * 
 * yuchan_step : 곡률(curvature) 계산 
 * KD-Tree 생성 (최근접 이웃 검색용)
 * 
 * @param cloud 포인트 클라우드
 * @return Edge와 Planar 특징점
 */
LOAMFeatures extract_loam_features_knn(const gtsam_points::PointCloud::Ptr& cloud)
{
  LOAMFeatures features;
  features.edge_points = std::make_shared<gtsam_points::PointCloudCPU>();
  features.planar_points = std::make_shared<gtsam_points::PointCloudCPU>();

  int num_points = cloud->size();
  if (num_points < 100)
  {
    spdlog::warn("Too few points for LOAM feature extraction: {}", num_points);
    return features;
  }
  
  // yuchan_step : 곡률(curvature) 계산 
  // KD-Tree 생성 (최근접 이웃 검색용)
  gtsam_points::KdTree kdtree(cloud->points, cloud->size());
  // curvature 계산용 벡터
  std::vector<float> curvatures(num_points);
  for(int i = 0; i<num_points; i++)
  {
    // KNN search(k=11, 자기 자신 포함되므로 +1)
    std::array<size_t, 11> neighbors;
    std::array<double, 11> sq_dists;
    size_t num_found = kdtree.knn_search(cloud->points[i].data(), 11, neighbors.data(), sq_dists.data());

    // 현재 포인트
    Eigen::Vector3d X_i = cloud->points[i].head<3>();
    // 이웃 포인트들과의 차이 벡터 합산 (자기 자신 제외)
    Eigen::Vector3d diff_sum = Eigen::Vector3d::Zero();
    int valid_neighbors = 0;
    for(size_t j = 0; j < num_found; j++)
    {
      size_t idx = neighbors[j];
      if (idx == static_cast<size_t>(i)) continue;  // 자기 자신 제외
      Eigen::Vector3d X_j = cloud->points[idx].head<3>();
      diff_sum += (X_i - X_j);
      valid_neighbors++;
    }
    // Curvature 계산: c = ||sum(Xi - Xj)|| / (|S| * ||Xi||)
    // LOAM 논문: c = ||Σ(Xi - Xj)|| / (|S| * ||Xi||)
    double c = (valid_neighbors > 0) ? diff_sum.norm() / (valid_neighbors * X_i.norm()) : 0.0;
    curvatures[i] = c;
  }

  // Curvature 기반 정렬 및 선택
  std::vector<std::pair<int, float>> index_curvatures;
  for(int i = 0; i < num_points; i++)
  {
    index_curvatures.emplace_back(i, curvatures[i]);
  }
  // Curvature 오름차순 정렬(낮은 곡률 -> 높은 곡률)
  std::sort(index_curvatures.begin(), index_curvatures.end(),
            [](const auto& a, const auto& b) { return a.second < b.second; });
  // edge / planar 특정점 선택
  const int max_edge = 5000;
  const int max_planar = 10000;

  std::vector<Eigen::Vector4d> edge_pts, planar_pts;

  // Planar 하위 4% 낮은 곡률
  int planar_count = std::min(max_planar, (int)(num_points * 0.04));
  for (int i = 0; i < planar_count; i++)
  {
      int idx = index_curvatures[i].first;
      planar_pts.push_back(cloud->points[idx]);
  }

  // Edge 상위 2% 높은 곡률
  int edge_count = std::min(max_edge, (int)(num_points * 0.02));
  for (int i = num_points - edge_count; i < num_points; i++)
  {
      int idx = index_curvatures[i].first;
      edge_pts.push_back(cloud->points[idx]);
  }
  
  features.edge_points->add_points(edge_pts);
  features.planar_points->add_points(planar_pts);
  
  spdlog::info("LOAM features (Curvature-based): {} edge, {} planar from {} points",
                edge_pts.size(), planar_pts.size(), num_points);

  if (num_points < 5000)
  {
    spdlog::warn("Too few points for reliable LOAM features: {}", num_points);
    // 경고만 출력하고 계속 진행 (Fallback 없음)
  }

  return features;
}

/**
 * @brief Normal 벡터 기반 LOAM feature 추출 (휴리스틱 방식)
 * 
 * yuchan_법선벡터 기반 추출
 * 
 * 법선 벡터의 최대 축 성분 기준으로 특징점 분류:
 *   - Planar: 법선 벡터가 한 축에 거의 평행한 경우 (max_axis > 0.9)
 *   - Edge: 법선 벡터가 어느 한 축에도 크게 치우치지 않은 경우 (max_axis < 0.7)
 * 
 * @param cloud 포인트 클라우드
 * @return Edge와 Planar 특징점
 */
LOAMFeatures extract_loam_features_normal_based(const gtsam_points::PointCloud::Ptr& cloud)
{
  LOAMFeatures features;
  features.edge_points = std::make_shared<gtsam_points::PointCloudCPU>();
  features.planar_points = std::make_shared<gtsam_points::PointCloudCPU>();

  int num_points = cloud->size();
  if (num_points < 100)
  {
    spdlog::warn("Too few points for LOAM feature extraction: {}", num_points);
    return features;
  }

  // yuchan_법선벡터 기반 추출
  std::vector<Eigen::Vector4d> normals = gtsam_points::estimate_normals(cloud->points, cloud->size());

  const int downsample_factor = 5;  // 5개중 1개만 샘플링(다운샘플링 -> 속도향상)
  const int max_edge = 2000;  // 최대 Edge 특징점 수
  const int max_planar = 4000;  // 최대 Planar 특징점 수
  
  std::vector<Eigen::Vector4d> edge_pts, planar_pts;
  edge_pts.reserve(max_edge);
  planar_pts.reserve(max_planar);
  
  // 법선 벡터 기반 특징점 선택(이 부분이 문제이지 않을까? 집중해서 살펴볼 것)
  for (int i = 0; i < num_points && (edge_pts.size() < max_edge || planar_pts.size() < max_planar); i += downsample_factor) 
  {
    if (normals.empty() || i >= (int)normals.size()) continue;
    // 법선 벡터 정규화(방향만 가짐)
    Eigen::Vector3d normal = normals[i].head<3>().normalized();
    
    if (normal.hasNaN() || normal.squaredNorm() < 0.5) continue; // Nan 에러 체크
    
    // 휴리스틱 : 법선 벡터의 최대 축 성분 기준으로 특징점 분류
    // x, y, z축 중에서 절대값이 가장 큰 축 성분을 기준으로 판단
    double max_axis = std::max({std::abs(normal.x()), std::abs(normal.y()), std::abs(normal.z())});
    
    // planar 특징점 : 법선 벡터가 한 축에 거의 평행한 경우 (max_axis > 0.9)
    if (max_axis > 0.9 && planar_pts.size() < max_planar) 
    {
      planar_pts.push_back(cloud->points[i]);
    }
    // edge 특징점 : 법선 벡터가 어느 한 축에도 크게 치우치지 않은 경우 (max_axis < 0.7)
    else if (max_axis < 0.7 && edge_pts.size() < max_edge) 
    {
      edge_pts.push_back(cloud->points[i]);
    }
  }
  
  // Fallback: 균일 샘플링
  if (edge_pts.size() < 100 || planar_pts.size() < 200) 
  {
    spdlog::info("Normal-based selection insufficient, using fallback sampling");
    edge_pts.clear();
    planar_pts.clear();
    
    int edge_step = std::max(1, num_points / max_edge);
    int planar_step = std::max(1, num_points / max_planar);
    
    for (int i = 0; i < num_points && edge_pts.size() < max_edge; i += edge_step) 
    {
      edge_pts.push_back(cloud->points[i]);
    }
    for (int i = 0; i < num_points && planar_pts.size() < max_planar; i += planar_step) 
    {
      planar_pts.push_back(cloud->points[i]);
    }
  }

  features.edge_points->add_points(edge_pts);
  features.planar_points->add_points(planar_pts);
  
  spdlog::info("LOAM features (Normal-based): {} edge, {} planar from {} points", 
                edge_pts.size(), planar_pts.size(), num_points);

  if (num_points < 5000)
  {
    spdlog::warn("Too few points for reliable LOAM features: {}", num_points);
    // 경고만 출력하고 계속 진행 (Fallback 없음)
  }

  return features;
}
