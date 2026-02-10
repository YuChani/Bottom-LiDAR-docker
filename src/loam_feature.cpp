#include "loam_feature.hpp"
#include <spdlog/spdlog.h>
#include <gtsam_points/ann/kdtree.hpp>
#include <gtsam_points/features/normal_estimation.hpp>
#include <algorithm>
#include <map>

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

LOAMFeatures extract_loam_features_ring_based(const std::vector<PointWithRing>& points_with_ring)
{
  LOAMFeatures features;
  features.edge_points = std::make_shared<gtsam_points::PointCloudCPU>();
  features.planar_points = std::make_shared<gtsam_points::PointCloudCPU>();

  if (points_with_ring.size() < 100)
  {
    spdlog::warn("Too few points for LOAM feature extraction");
    return features;
  }

  std::map<uint16_t, std::vector<size_t>> ring_groups;

  for (size_t i = 0; i < points_with_ring.size(); i++)
  {
    ring_groups[points_with_ring[i].ring].push_back(i);
  }

  spdlog::info("Found {} rings in point cloud", ring_groups.size());

  for (auto& [ring_id, indices] : ring_groups)
  {
    std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b)
    {
      return points_with_ring[a].angle < points_with_ring[b].angle;
    });
  }

  std::vector<Eigen::Vector4d> edge_pts, planar_pts;

  for (const auto& [ring_id, indices] : ring_groups)
  {
    int n = indices.size();
    if (n < 100) 
    {
      continue;
    }

    std::vector<float> ranges(n);
    std::vector<float> curvatures(n, 0.0f);
    std::vector<int> picked(n, 0);

    for (int i = 0; i < n; i++)
    {
      Eigen::Vector3d X = points_with_ring[indices[i]].point.head<3>();
      ranges[i] = X.norm();
    }

    for (int i = 5; i < n - 5; i++)
    {
      float diff_sq_sum = 0.0f;
      for (int j = -5; j <= 5; j++)
      {
        if (j == 0) 
        {
          continue;
        }

        float diff = ranges[i] - ranges[i + j];
        diff_sq_sum += diff * diff;
      }
      curvatures[i] = diff_sq_sum;
    }

    for (int i = 5; i < n - 5; i++)
    {
      float depth1 = ranges[i];
      float depth2 = ranges[i + 1];

      if (std::abs(depth1 - depth2) > 0.3)
      {
        if (depth1 - depth2 > 0)
        {
          for (int k = -5; k <= 0; k++)
            if (i + k >= 0 && i + k < n) picked[i + k] = 1;
        }

        else
        {
          for (int k = 1; k <= 6; k++)
            if (i + k < n) picked[i + k] = 1;
        }
      }

      float diff1 = std::abs(ranges[i - 1] - ranges[i]);
      float diff2 = std::abs(ranges[i + 1] - ranges[i]);

      if (diff1 > 0.02 * ranges[i] && diff2 > 0.02 * ranges[i])
      {
        picked[i] = 1;
      }
    }

    const int num_regions = 6;
    int region_size = n / num_regions;

    for (int region = 0; region < num_regions; region++)
    {
      int sp = region * region_size;
      int ep = (region == num_regions - 1) ? n - 5 : (region + 1) * region_size;

      if (sp >= ep || sp < 5 || ep >= n - 5) 
      {
        continue;
      }

      std::vector<std::pair<int, float>> region_curvatures;
      for (int i = sp; i < ep; i++)
      {
        if (i < 5 || i >= n - 5) 
        {
          continue;
        }
        region_curvatures.emplace_back(i, curvatures[i]);
      }
      
      if (region_curvatures.empty()) 
      {
        continue;
      }
      
      std::sort(region_curvatures.begin(), region_curvatures.end(),
                [](auto& a, auto& b) { return a.second > b.second; });

      int edge_count = 0;
      for (auto& [idx, curv] : region_curvatures)
      {
        if (picked[idx] == 0 && curv > 0.1)
        {
          edge_pts.push_back(points_with_ring[indices[idx]].point);
          picked[idx] = 1;

          for (int k = 1; k <= 5; k++)
          {
            if (idx - k >= 0) picked[idx - k] = 1;
            if (idx + k < n) picked[idx + k] = 1;
          }

          edge_count++;
          if (edge_count >= 2) 
          {
            break;
          }
        }
      }

      std::reverse(region_curvatures.begin(), region_curvatures.end());
      int planar_count = 0;
      for (auto& [idx, curv] : region_curvatures)
      {
        if (picked[idx] == 0 && curv < 0.1)
        {
          planar_pts.push_back(points_with_ring[indices[idx]].point);
          picked[idx] = 1;

          for (int k = 1; k <= 5; k++)
          {
            if (idx - k >= 0) picked[idx - k] = 1;
            if (idx + k < n) picked[idx + k] = 1;
          }

          planar_count++;
          if (planar_count >= 4) 
          {
            break;
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
