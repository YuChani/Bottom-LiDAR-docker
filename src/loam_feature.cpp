#include "loam_feature.hpp"
#include <spdlog/spdlog.h>
#include <gtsam_points/ann/kdtree.hpp>
#include <gtsam_points/features/normal_estimation.hpp>
#include <algorithm>
#include <map>


// yuchan : PCL을 사용한 PCD 읽기 함수. 데이터 로딩(Pre-Porcessing)
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
    // 데이터 NaN값 필터링
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) 
    {
      continue;
    }
    // Range기반 필터링. -0.5m : base가 찍힌 noise일수있음 / 150m : 너무 멀면 point 간격 넓어서 의미없음
    double dist = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
    if (dist < 0.5 || dist > 150.0) 
    {
      continue;
    }
    // Angle 계산. Ring 내에서 점들을 회전 순서대로 정렬할때 사용
    PointWithRing pt;
    pt.point = Eigen::Vector4d(p.x, p.y, p.z, 1.0);
    pt.ring = p.ring;
    pt.angle = std::atan2(p.y, p.x);
    
    points.push_back(pt);
  }
  
  spdlog::info("PCL: Read {} points with ring info from {}", points.size(), pcd_path);

  return points;
}

// yuchan : Ring 기반 LOAM feature 추출
LOAMFeatures extract_loam_features_ring_based(const std::vector<PointWithRing>& points_with_ring)
{
  // 결과 저장할 gtsam_points 포인트 클라우드 초기화
  LOAMFeatures features;
  features.edge_points = std::make_shared<gtsam_points::PointCloudCPU>();
  features.planar_points = std::make_shared<gtsam_points::PointCloudCPU>();

  if (points_with_ring.size() < 100)
  {
    spdlog::warn("Too few points for LOAM feature extraction");
    return features;
  }
  // Ring별로 포인트 그룹화 및 정렬. LOAM은 스캔 라인 상의 연속성을 이용하기 때문에 Ring별 처리 해야함
  std::map<uint16_t, std::vector<size_t>> ring_groups;

  for (size_t i = 0; i < points_with_ring.size(); i++)
  {
    ring_groups[points_with_ring[i].ring].push_back(i);
  }

  spdlog::info("Found {} rings in point cloud", ring_groups.size());
  // 각 Ring 내에서 포인트를 회전 각도 기준으로 정렬 -> 인덱스 i-1, i+1이 실제 공간상의 왼쪽/오른쪽 이웃이 되도록
  for (auto& [ring_id, indices] : ring_groups)
  {
    std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b)
    {
      return points_with_ring[a].angle < points_with_ring[b].angle;
    });
  }

  // Ring 별로 edge, planar 포인트 추출
  std::vector<Eigen::Vector4d> edge_pts, planar_pts;

  for (const auto& [ring_id, indices] : ring_groups)
  {
    int cloudSize = indices.size();
    if (cloudSize < 100) 
    {
      continue; // 너무 적은 포인트는 무시
    }

    std::vector<float> ranges(cloudSize);
    std::vector<float> curvatures(cloudSize, 0.0f);
    std::vector<int> picked(cloudSize, 0);

    for (int i = 0; i < cloudSize; i++)
    {
      Eigen::Vector3d X = points_with_ring[indices[i]].point.head<3>();
      ranges[i] = X.norm();
    }

    // yuchan : 곡률(Curvature) 계산
    // LIO-SAM 방식의 LOAM Feature 추출(Edge/Planar 특징점 추출)
    // 현재 LIO-SAM 코드의 방식을 채용함. (차이->제곱->합산) 방식 / LOAM 방식은 (차이->합산->제곱) 방식
    // 주변 점들과의 거리 차이가 클수록 곡률이 큼 -> edge point일 가능성 높음
    // region_curvatures 벡터에 (인덱스, 곡률값) 쌍으로 저장하고, 내림차순 정렬하여 특징점 후보 선정(큰값 -> 작은값 순서)
    // curv < 0.1 : planar point 후보 -> 작은값 / curv > 0.1 : edge point 후보 -> 큰값
    // edge는 2개, planar는 4개씩 추출하는 why? -> edge는 상대적으로 적고, planar는 넓은 영역에서 많이 나올 수 있기 때문
    for (int i = 5; i < cloudSize - 5; i++)
    {
      float diffRange = ranges[i-5] + ranges[i-4] + ranges[i-3] + ranges[i-2] + ranges[i-1]
                      - ranges[i] * 10.0f
                      + ranges[i+1] + ranges[i+2] + ranges[i+3] + ranges[i+4] + ranges[i+5];
      
      curvatures[i] = diffRange * diffRange;
    }
    // Occlusion 확인 : 특징점 후보 마킹(point에 마킹해서 제거할 포인트 표시)
    // 이 방식으로 장애물뒤에 있는데도 찍힌 point(ghost point)들을 제거가 가능함. 이거 안하면 이상한 point들이 Edge 특징점으로 뽑혀서 이상해짐
    for (int i = 5; i < cloudSize - 5; i++)
    {
      float depth1 = ranges[i];
      float depth2 = ranges[i + 1];
      // Occlusion : 인접한 두 점의 depth차이가 크면 장애물 경계일 가능성 있음 -> 주변점들 마킹(여기서는 0.3m로 설정)
      if (std::abs(depth1 - depth2) > 0.3)
      {
        // depth1이 더 멀리있는 경우 : depth1 쪽이 장애물 뒤쪽일 가능성. 불안정한 point들이므로 왼쪽 5개 점 마킹
        if (depth1 - depth2 > 0)
        {
          for (int k = -5; k <= 0; k++)
            if (i + k >= 0 && i + k < cloudSize) picked[i + k] = 1;
        }
        // depth2가 더 멀리있는 경우 : depth2 쪽이 장애물 뒤쪽일 가능성. 불안정한 point들이므로 오른쪽 6개 점 마킹
        else
        {
          for (int k = 1; k <= 6; k++)
            if (i + k < cloudSize) picked[i + k] = 1;
        }
      }
      // Parrelle Beam : 레이저가 벽면과 거의 평행하게 입사되었을때 찍힌 점들. 연속된 점들 중에서 급격한 거리 변화가 있으면 주변점들 마킹
      float diff1 = std::abs(ranges[i - 1] - ranges[i]);
      float diff2 = std::abs(ranges[i + 1] - ranges[i]);

      if (diff1 > 0.02 * ranges[i] && diff2 > 0.02 * ranges[i])
      {
        picked[i] = 1;  // 1로 표시된 점들은 특징점 후보에서 제외하기 위한 마킹
      }
    }
    // 6개 영역으로 나누어 각 영역에서 edge, planar 특징점 추출 -> 나눠진 영역안에서 특징점이 확보됨.
    const int num_regions = 6;
    int region_size = cloudSize / num_regions;

    for (int region = 0; region < num_regions; region++)
    {
      int sp = region * region_size;
      int ep = (region == num_regions - 1) ? cloudSize - 5 : (region + 1) * region_size;
      // Curvature 계산에 사용된 앞뒤 point 5개는 제외시킴 -> 곡률 계산이 부정확할 수 있음
      if (sp >= ep || sp < 5 || ep >= cloudSize - 5) 
      {
        continue;
      }
      // 해당 region내에서 곡률값과 인덱스 쌍 저장(후보군 저장)
      std::vector<std::pair<int, float>> region_curvatures;
      for (int i = sp; i < ep; i++)
      {
        if (i < 5 || i >= cloudSize - 5) 
        {
          continue;
        }
        region_curvatures.emplace_back(i, curvatures[i]);
      }
      
      if (region_curvatures.empty()) 
      {
        continue;
      }
      // 정렬 : 곡률 큰 순서대로 정렬. 내림차순 (큰값 -> 작은값 순서)
      std::sort(region_curvatures.begin(), region_curvatures.end(),[](auto& a, auto& b) { return a.second > b.second; });
      // Edge point 추출하는 부분(곡률 큰 순서 -> 뒤에서부터 탐색함)
      int edge_count = 0;
      for (auto& [idx, curv] : region_curvatures)
      {
        // 곡률이 0.1 이상인 점들 중에서 아직 선택되지 않은 점들을 edge 특징점으로 선택(곡률이 0.1보다 큼)
        if (picked[idx] == 0 && curv > 0.1)
        {
          edge_pts.push_back(points_with_ring[indices[idx]].point); // 높은 곡률부터 선택
          picked[idx] = 1;
          // 선택된 점 주변은 선택하는거 방지함 -> 너무 가까운 점들이 다 edge로 뽑히는거 방지
          for (int k = 1; k <= 5; k++)
          {
            if (idx - k >= 0) 
            {
              picked[idx - k] = 1;
            }

            if (idx + k < cloudSize) 
            {
              picked[idx + k] = 1;
            }
          }

          edge_count++;
          if (edge_count >= 2) 
          {
            break; // 각 영역당 최대 2개의 edge 특징점만 선택
          }
        }
      }

      // Planar feature point 추출 (곡률 작은 순서 -> 앞에서부터 탐색)
      // region_curvatures는 내림차순으로 정렬되어 있으므로 뒤집어서(reverse) 오름차순으로 만듦 (작은값 -> 큰값 순서)
      std::reverse(region_curvatures.begin(), region_curvatures.end());
      int planar_count = 0;
      for (auto& [idx, curv] : region_curvatures)
      {
        // 곡률이 0.1 이하인 점들 중에서 아직 선택되지 않은 점들을 planar 특징점으로 선택(곡률이 0.1보다 작음)
        if (picked[idx] == 0 && curv < 0.1)
        {
          planar_pts.push_back(points_with_ring[indices[idx]].point);
          picked[idx] = 1;
          // 선택된 점 주변은 선택하는거 방지함 -> 너무 가까운 점들이 다 planar로 뽑히는거 방지
          for (int k = 1; k <= 5; k++)
          {
            if (idx - k >= 0) 
            {
              picked[idx - k] = 1;
            }
            if (idx + k < cloudSize) 
            {
              picked[idx + k] = 1;
            }
          }

          planar_count++;
          if (planar_count >= 4) 
          {
            break; // 각 영역당 최대 4개의 planar 특징점만 선택
          }
        }
      }
    }
  }

  features.edge_points->add_points(edge_pts);
  features.planar_points->add_points(planar_pts);

  spdlog::info("LOAM features (LIO-SAM): {} edge, {} planar from {} points",
                edge_pts.size(), planar_pts.size(), points_with_ring.size());

  return features;
}

// yuchan : KNN 기반 LOAM feature 추출
// 혹시나 Ring 정보가 없는 포인트 클라우드에 대해서도 LOAM 특징점 추출을 지원하기 위해 구현을 해둠
// 지울까?
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
  
  gtsam_points::KdTree kdtree(cloud->points, cloud->size());
  std::vector<float> curvatures(num_points);

  for(int i = 0; i<num_points; i++)
  {
    std::array<size_t, 11> neighbors;
    std::array<double, 11> sq_dists;
    size_t num_found = kdtree.knn_search(cloud->points[i].data(), 11, neighbors.data(), sq_dists.data());

    if (num_found < 5) 
    {
      curvatures[i] = 0.0;
      continue;
    }

    Eigen::Vector3d X_i = cloud->points[i].head<3>();
    Eigen::Vector3d diff_sum = Eigen::Vector3d::Zero();

    int valid_neighbors = 0;

    for(size_t j = 0; j < num_found; j++)
    {
      size_t idx = neighbors[j];
      if (idx == static_cast<size_t>(i)) 
      {
        continue;
      }
      Eigen::Vector3d X_j = cloud->points[idx].head<3>();
      diff_sum += (X_i - X_j);
      valid_neighbors++;
    }

    double xi_norm = X_i.norm();
    double c = (valid_neighbors > 0 && xi_norm > 0.1) ? diff_sum.norm() / (valid_neighbors * xi_norm) : 0.0;
    
    curvatures[i] = c;
  }

  std::vector<std::pair<int, float>> index_curvatures;

  for(int i = 0; i < num_points; i++)
  {
    index_curvatures.emplace_back(i, curvatures[i]);
  }

  std::sort(index_curvatures.begin(), index_curvatures.end(), [](const auto& a, const auto& b) { return a.second < b.second; });
  const int max_edge = 5000;
  const int max_planar = 10000;

  std::vector<Eigen::Vector4d> edge_pts, planar_pts;

  int planar_count = std::min(max_planar, (int)(num_points * 0.04));

  for (int i = 0; i < planar_count; i++)
  {
      int idx = index_curvatures[i].first;
      planar_pts.push_back(cloud->points[idx]);
  }

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
  }

  return features;
}
