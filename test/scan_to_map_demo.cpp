/**
 * scan_to_map_demo.cpp - Scan-to-Map Registration Demo with Oxford Dataset
 * 
 * Demonstrates incremental scan-to-map SLAM using GTSAM-Points VGICP factors.
 * Each new scan is registered against an accumulated global map.
 */

#include <chrono>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <algorithm>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam_points/config.hpp>
#include <gtsam_points/util/read_points.hpp>
#include <gtsam_points/features/covariance_estimation.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>

#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class ScanToMapDemo
{
public:
  ScanToMapDemo()
  {
    auto console = spdlog::stdout_color_mt("scan_to_map");
    spdlog::set_default_logger(console);
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%^%l%$] %v");
    
    const std::string data_path = "/root/dataset/Oxford_spires/lidar-clouds";
    const int max_frames = 100;
    
    spdlog::info("=== Scan-to-Map SLAM Demo ===");
    spdlog::info("Dataset: {}", data_path);
    spdlog::info("Max frames: {}", max_frames);
    
    std::vector<std::string> pcd_files;
    namespace fs = std::filesystem;
    for (const auto& entry : fs::directory_iterator(data_path))
    {
      if (entry.path().extension() == ".pcd")
      {
        pcd_files.push_back(entry.path().string());
      }
    }
    std::sort(pcd_files.begin(), pcd_files.end());
    
    if (pcd_files.empty())
    {
      spdlog::error("No PCD files found in {}", data_path);
      return;
    }
    
    int num_frames = std::min(max_frames, (int)pcd_files.size());
    spdlog::info("Found {} PCD files, loading {} frames", pcd_files.size(), num_frames);
    
    global_map = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(0.5);
    
    for (int i = 0; i < num_frames; i++)
    {
      spdlog::info("Processing frame {}/{}", i + 1, num_frames);
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_files[i], *pcl_cloud) == -1)
      {
        spdlog::error("Failed to load {}", pcd_files[i]);
        continue;
      }
      
      std::vector<Eigen::Vector4d> points(pcl_cloud->size());
      for (size_t j = 0; j < pcl_cloud->size(); j++)
      {
        points[j] << pcl_cloud->points[j].x, pcl_cloud->points[j].y, pcl_cloud->points[j].z, 1.0;
      }
      
      auto frame = std::make_shared<gtsam_points::PointCloudCPU>();
      frame->add_points(points);
      frame->add_covs(gtsam_points::estimate_covariances(points));
      
      gtsam::Pose3 estimated_pose;
      
      if (i == 0)
      {
        estimated_pose = gtsam::Pose3::Identity();
        spdlog::info("  Frame 0: Initialized at origin");
      }
      else
      {
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initial;
        
        initial.insert(i, poses.at<gtsam::Pose3>(i-1));
        
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(i, poses.at<gtsam::Pose3>(i-1), gtsam::noiseModel::Isotropic::Precision(6, 1e3)));
        
        auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(
          gtsam::Pose3::Identity(),
          i,
          global_map,
          frame
        );
        factor->set_num_threads(4);
        graph.add(factor);
        
        gtsam_points::LevenbergMarquardtExtParams lm_params;
        lm_params.setMaxIterations(20);
        lm_params.setRelativeErrorTol(1e-5);
        lm_params.setAbsoluteErrorTol(1e-5);
        lm_params.setVerbosity("SILENT");
        
        gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, initial, lm_params);
        gtsam::Values result = optimizer.optimize();
        
        estimated_pose = result.at<gtsam::Pose3>(i);
        
        auto t = estimated_pose.translation();
        auto ypr = estimated_pose.rotation().ypr() * 180.0 / M_PI;
        spdlog::info("  Optimized pose: t=[{:.3f}, {:.3f}, {:.3f}], ypr=[{:.3f}, {:.3f}, {:.3f}]",
                     t.x(), t.y(), t.z(), ypr.x(), ypr.y(), ypr.z());
      }
      
      poses.insert(i, estimated_pose);
      
      std::vector<Eigen::Vector4d> transformed_points(frame->size());
      Eigen::Isometry3d pose_iso(estimated_pose.matrix());
      for (size_t j = 0; j < frame->size(); j++)
      {
        transformed_points[j] = pose_iso * frame->points[j];
      }
      
      auto transformed_frame = std::make_shared<gtsam_points::PointCloudCPU>();
      transformed_frame->add_points(transformed_points);
      auto transformed_covs = gtsam_points::estimate_covariances(transformed_points);
      transformed_frame->add_covs(transformed_covs);
      
      global_map->insert(*transformed_frame);
      
      spdlog::info("  Map updated with {} points", transformed_points.size());
    }
    
    spdlog::info("=== Scan-to-Map Completed ===");
    spdlog::info("Total frames processed: {}", poses.size());
    
    save_trajectory("/root/workdir/scan_to_map_trajectory.txt");
    
    spdlog::info("Trajectory saved to /root/workdir/scan_to_map_trajectory.txt");
  }
  
  void save_trajectory(const std::string& filename)
  {
    std::ofstream ofs(filename);
    if (!ofs)
    {
      spdlog::error("Failed to open {}", filename);
      return;
    }
    
    for (const auto& key_value : poses)
    {
      gtsam::Pose3 pose = key_value.value.cast<gtsam::Pose3>();
      auto t = pose.translation();
      auto q = pose.rotation().toQuaternion();
      
      ofs << key_value.key << " "
          << t.x() << " " << t.y() << " " << t.z() << " "
          << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    }
    
    spdlog::info("Saved {} poses", poses.size());
  }

private:
  gtsam::Values poses;
  std::shared_ptr<gtsam_points::GaussianVoxelMapCPU> global_map;
};

int main(int argc, char** argv)
{
  ScanToMapDemo demo;
  return 0;
}
