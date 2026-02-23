/**
 * test_all_factors.cpp - Headless 6-Factor Registration Benchmark
 * 
 * GUI 없이 6가지 registration 방식(P2P, P2Pl, GICP, VGICP, LOAM, NDT)을 
 * 순차적으로 실행하고 R,t 오차를 비교 출력합니다.
 */

#include <chrono>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <algorithm>
#include <map>
#include <cmath>
#include <vector>
#include <string>
#include <iomanip>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <Eigen/Core>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam_points/config.hpp>
#include <gtsam_points/util/read_points.hpp>
#include <gtsam_points/features/normal_estimation.hpp>
#include <gtsam_points/features/covariance_estimation.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/ann/kdtree.hpp>

#include <gtsam_points/factors/integrated_icp_factor.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/factors/integrated_loam_factor.hpp>
#include <ndt/integrated_ndt_factor.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>

#include "loam_feature.hpp"

namespace fs = std::filesystem;

struct FrameResult {
  int frame_id;
  double trans_error;
  double rot_error;
  Eigen::Vector3d opt_t;
  Eigen::Vector3d gt_t;
  Eigen::Vector3d opt_ypr;
  Eigen::Vector3d gt_ypr;
};

struct FactorResult {
  std::string name;
  std::vector<FrameResult> frame_results;
  double mean_trans_error;
  double mean_rot_error;
  double max_trans_error;
  double max_rot_error;
  double optimization_time_ms;
  int num_iterations;
};

class DataLoader {
public:
  std::vector<gtsam_points::PointCloud::Ptr> frames;
  std::vector<gtsam_points::GaussianVoxelMap::Ptr> voxelmaps;
  std::vector<LOAMFeatures> loam_features;
  gtsam::Values poses_gt;
  gtsam::Values poses_noisy;
  int num_frames;

  bool load(const std::string& data_path, double noise_scale = 0.1) {
    Eigen::Vector3d t_base_lidar(0.0, 0.0, 0.124);
    Eigen::Quaterniond q_base_lidar(0.0, 0.0, 0.0, 1.0);
    gtsam::Pose3 T_base_lidar(gtsam::Rot3(q_base_lidar), t_base_lidar);

    std::map<double, gtsam::Pose3> gt_poses;
    {
      std::ifstream ifs(data_path + "/gt-tum.txt");
      if (!ifs) {
        spdlog::error("Failed to open {}/gt-tum.txt", data_path);
        return false;
      }
      double timestamp;
      gtsam::Vector3 trans;
      gtsam::Quaternion quat;
      while (ifs >> timestamp >> trans.x() >> trans.y() >> trans.z() 
                 >> quat.x() >> quat.y() >> quat.z() >> quat.w()) {
        gt_poses[timestamp] = gtsam::Pose3(gtsam::Rot3(quat), trans);
      }
      spdlog::info("Loaded {} GT poses", gt_poses.size());
    }

    std::vector<std::string> pcd_files;
    for (const auto& entry : fs::directory_iterator(data_path)) {
      if (entry.path().extension() == ".pcd") {
        pcd_files.push_back(entry.path().string());
      }
    }
    std::sort(pcd_files.begin(), pcd_files.end());

    if (pcd_files.size() < 2) {
      spdlog::error("Need at least 2 PCD files, found {}", pcd_files.size());
      return false;
    }

    int max_frames = 100;
    num_frames = std::min(max_frames, (int)pcd_files.size());
    spdlog::info("Loading {} frames...", num_frames);

    frames.resize(num_frames);
    voxelmaps.resize(num_frames);

    auto find_closest_pose = [&gt_poses](double target_ts) -> gtsam::Pose3 {
      auto it = gt_poses.lower_bound(target_ts);
      if (it == gt_poses.end()) return gt_poses.rbegin()->second;
      if (it == gt_poses.begin()) return it->second;
      auto prev_it = std::prev(it);
      if (std::abs(it->first - target_ts) < std::abs(prev_it->first - target_ts))
        return it->second;
      return prev_it->second;
    };

    std::string first_filename = fs::path(pcd_files[0]).stem().string();
    double first_timestamp = std::stod(first_filename);
    gtsam::Pose3 W_T_B_origin = find_closest_pose(first_timestamp);
    gtsam::Pose3 W_T_L_origin = W_T_B_origin * T_base_lidar;

    for (int i = 0; i < num_frames; i++) {
      spdlog::info("  Loading frame {}/{}: {}", i + 1, num_frames, pcd_files[i]);

      std::string filename = fs::path(pcd_files[i]).stem().string();
      double pcd_timestamp = std::stod(filename);

      gtsam::Pose3 W_T_B = find_closest_pose(pcd_timestamp);
      gtsam::Pose3 W_T_L = W_T_B * T_base_lidar;
      gtsam::Pose3 relative_pose = W_T_L_origin.inverse() * W_T_L;

      poses_gt.insert(i, relative_pose);

      auto points_with_ring = read_points_with_ring_pcl(pcd_files[i]);
      if (points_with_ring.empty()) {
        spdlog::error("Failed to read: {}", pcd_files[i]);
        return false;
      }

      std::vector<Eigen::Vector3f> points_f;
      points_f.reserve(points_with_ring.size());
      for (const auto& p : points_with_ring) {
        points_f.push_back(p.point.head<3>().cast<float>());
      }

      std::vector<Eigen::Vector4d> points(points_f.size());
      std::transform(points_f.begin(), points_f.end(), points.begin(), 
        [](const Eigen::Vector3f& p) {
          return (Eigen::Vector4d() << p.cast<double>(), 1.0).finished();
        });

      auto covs = gtsam_points::estimate_covariances(points);

      auto frame = std::make_shared<gtsam_points::PointCloudCPU>();
      frame->add_points(points);
      frame->add_covs(covs);
      frame->add_normals(gtsam_points::estimate_normals(frame->points, frame->size()));
      frames[i] = frame;

      auto features = extract_loam_features_ring_based(points_with_ring);
      loam_features.push_back(features);

      auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(0.5);
      voxelmap->insert(*frame);
      voxelmaps[i] = voxelmap;
    }

    spdlog::info("Adding noise (scale={})...", noise_scale);
    for (int i = 0; i < num_frames; i++) {
      if (i == 0) {
        poses_noisy.insert(i, poses_gt.at<gtsam::Pose3>(i));
      } else {
        gtsam::Pose3 noise = gtsam::Pose3::Expmap(gtsam::Vector6::Random() * noise_scale);
        poses_noisy.insert(i, poses_gt.at<gtsam::Pose3>(i) * noise);
      }
    }

    spdlog::info("Data loading complete. {} frames ready.", num_frames);
    return true;
  }
};

gtsam::NonlinearFactor::shared_ptr create_factor(
  const std::string& factor_type,
  gtsam::Key target_key,
  gtsam::Key source_key,
  const gtsam_points::PointCloud::ConstPtr& target,
  const gtsam_points::GaussianVoxelMap::ConstPtr& target_voxelmap,
  const gtsam_points::PointCloud::ConstPtr& source,
  const LOAMFeatures& target_loam,
  const LOAMFeatures& source_loam,
  int num_threads = 4)
{
  if (factor_type == "Point-to-Point") {
    auto factor = gtsam::make_shared<gtsam_points::IntegratedICPFactor>(target_key, source_key, target, source);
    factor->set_num_threads(num_threads);
    return factor;
  }
  else if (factor_type == "Point-to-Plane") {
    auto factor = gtsam::make_shared<gtsam_points::IntegratedPointToPlaneICPFactor>(target_key, source_key, target, source);
    factor->set_num_threads(num_threads);
    return factor;
  }
  else if (factor_type == "GICP") {
    auto factor = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(target_key, source_key, target, source);
    factor->set_num_threads(num_threads);
    return factor;
  }
  else if (factor_type == "VGICP") {
    auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(target_key, source_key, target_voxelmap, source);
    factor->set_num_threads(num_threads);
    return factor;
  }
  else if (factor_type == "LOAM") {
    auto factor = gtsam::make_shared<gtsam_points::IntegratedLOAMFactor>(
      target_key, source_key,
      target_loam.edge_points, target_loam.planar_points,
      source_loam.edge_points, source_loam.planar_points);
    factor->set_enable_correspondence_validation(true);
    factor->set_max_correspondence_distance(2.0, 2.0);
    factor->set_num_threads(num_threads);
    return factor;
  }
  else if (factor_type == "NDT") {
    auto factor = gtsam::make_shared<ndt::IntegratedNDTFactor>(target_key, source_key, target_voxelmap, source);
    factor->set_num_threads(num_threads);
    factor->set_search_mode(ndt::NDTSearchMode::DIRECT7);
    factor->set_outlier_ratio(0.1);
    factor->set_regularization_epsilon(1e-3);
    return factor;
  }
  
  spdlog::error("Unknown factor type: {}", factor_type);
  return nullptr;
}

FactorResult run_experiment(
  const std::string& factor_type,
  const DataLoader& data,
  bool full_connection = true,
  int num_threads = 4)
{
  FactorResult result;
  result.name = factor_type;
  
  spdlog::info("========================================");
  spdlog::info("Running experiment: {}", factor_type);
  spdlog::info("========================================");

  gtsam::NonlinearFactorGraph graph;
  
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
    0, data.poses_noisy.at<gtsam::Pose3>(0), 
    gtsam::noiseModel::Isotropic::Precision(6, 1e6)));

  for (int i = 0; i < data.num_frames; i++) {
    int j_end = full_connection ? data.num_frames : std::min(i + 2, data.num_frames);
    for (int j = i + 1; j < j_end; j++) {
      auto factor = create_factor(
        factor_type, i, j,
        data.frames[i], data.voxelmaps[i],
        data.frames[j],
        data.loam_features[i], data.loam_features[j],
        num_threads);
      if (factor) graph.add(factor);
    }
  }

  gtsam_points::LevenbergMarquardtExtParams lm_params;
  lm_params.maxIterations = 100;
  lm_params.relativeErrorTol = 1e-5;
  lm_params.absoluteErrorTol = 1e-5;

  int iteration_count = 0;
  lm_params.callback = [&iteration_count](
    const gtsam_points::LevenbergMarquardtOptimizationStatus& status, 
    const gtsam::Values& values) {
    iteration_count++;
    spdlog::info("  Iter {}: {}", iteration_count, status.to_string());
  };

  auto t_start = std::chrono::high_resolution_clock::now();
  
  gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, data.poses_noisy, lm_params);
  gtsam::Values optimized = optimizer.optimize();
  
  auto t_end = std::chrono::high_resolution_clock::now();
  result.optimization_time_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start).count() / 1e6;
  result.num_iterations = iteration_count;

  double total_trans_error = 0.0;
  double total_rot_error = 0.0;
  double max_trans = 0.0;
  double max_rot = 0.0;

  for (int i = 0; i < data.num_frames; i++) {
    gtsam::Pose3 opt_pose = optimized.at<gtsam::Pose3>(i);
    gtsam::Pose3 gt_pose = data.poses_gt.at<gtsam::Pose3>(i);
    gtsam::Pose3 error = gt_pose.inverse() * opt_pose;

    double t_err = error.translation().norm();
    double r_err = error.rotation().axisAngle().second * 180.0 / M_PI;

    FrameResult fr;
    fr.frame_id = i;
    fr.trans_error = t_err;
    fr.rot_error = r_err;
    fr.opt_t = opt_pose.translation();
    fr.gt_t = gt_pose.translation();
    fr.opt_ypr = opt_pose.rotation().ypr() * 180.0 / M_PI;
    fr.gt_ypr = gt_pose.rotation().ypr() * 180.0 / M_PI;
    result.frame_results.push_back(fr);

    total_trans_error += t_err;
    total_rot_error += r_err;
    max_trans = std::max(max_trans, t_err);
    max_rot = std::max(max_rot, r_err);

    spdlog::info("Frame {}:", i);
    spdlog::info("  [Opt] t: [{:.6f}, {:.6f}, {:.6f}]", fr.opt_t.x(), fr.opt_t.y(), fr.opt_t.z());
    spdlog::info("        R(ypr): [{:.3f}, {:.3f}, {:.3f}] deg", fr.opt_ypr.x(), fr.opt_ypr.y(), fr.opt_ypr.z());
    spdlog::info("  [GT]  t: [{:.6f}, {:.6f}, {:.6f}]", fr.gt_t.x(), fr.gt_t.y(), fr.gt_t.z());
    spdlog::info("        R(ypr): [{:.3f}, {:.3f}, {:.3f}] deg", fr.gt_ypr.x(), fr.gt_ypr.y(), fr.gt_ypr.z());
    spdlog::info("  [Err] t: {:.6f} m, R: {:.6f} deg", t_err, r_err);
  }

  result.mean_trans_error = total_trans_error / data.num_frames;
  result.mean_rot_error = total_rot_error / data.num_frames;
  result.max_trans_error = max_trans;
  result.max_rot_error = max_rot;

  spdlog::info("--- {} Summary ---", factor_type);
  spdlog::info("Mean Trans Error: {:.6f} m", result.mean_trans_error);
  spdlog::info("Mean Rot Error:   {:.6f} deg", result.mean_rot_error);
  spdlog::info("Max Trans Error:  {:.6f} m", result.max_trans_error);
  spdlog::info("Max Rot Error:    {:.6f} deg", result.max_rot_error);
  spdlog::info("Time:             {:.1f} ms ({} iterations)", result.optimization_time_ms, result.num_iterations);
  spdlog::info("========================================\n");

  return result;
}

void print_comparison_table(const std::vector<FactorResult>& results) {
  spdlog::info("╔══════════════════════════════════════════════════════════════════════════════════════╗");
  spdlog::info("║                    6-Factor Registration Benchmark Results                          ║");
  spdlog::info("╠══════════════════╦═══════════════╦═══════════════╦═══════════════╦═══════════════════╣");
  spdlog::info("║ Factor Type      ║ Mean T(m)     ║ Mean R(deg)   ║ Max T(m)      ║ Time(ms)/Iters   ║");
  spdlog::info("╠══════════════════╬═══════════════╬═══════════════╬═══════════════╬═══════════════════╣");

  for (const auto& r : results) {
    spdlog::info("║ {:<16s} ║ {:>13.6f} ║ {:>13.6f} ║ {:>13.6f} ║ {:>8.1f} / {:>4d}   ║",
      r.name, r.mean_trans_error, r.mean_rot_error, r.max_trans_error, 
      r.optimization_time_ms, r.num_iterations);
  }

  spdlog::info("╚══════════════════╩═══════════════╩═══════════════╩═══════════════╩═══════════════════╝");
}

void print_per_frame_comparison(const std::vector<FactorResult>& results) {
  if (results.empty()) return;
  
  int num_frames = results[0].frame_results.size();
  
  spdlog::info("\n=== Per-Frame Translation Error (m) ===");
  std::string header = "Frame |";
  for (const auto& r : results) {
    header += " " + r.name;
    int pad = 14 - (int)r.name.size();
    if (pad > 0) header += std::string(pad, ' ');
    header += "|";
  }
  spdlog::info("{}", header);
  
  for (int i = 0; i < num_frames; i++) {
    std::string line = "  " + std::to_string(i) + "   |";
    for (const auto& r : results) {
      char buf[32];
      snprintf(buf, sizeof(buf), " %13.6f |", r.frame_results[i].trans_error);
      line += buf;
    }
    spdlog::info("{}", line);
  }
  
  spdlog::info("\n=== Per-Frame Rotation Error (deg) ===");
  header = "Frame |";
  for (const auto& r : results) {
    header += " " + r.name;
    int pad = 14 - (int)r.name.size();
    if (pad > 0) header += std::string(pad, ' ');
    header += "|";
  }
  spdlog::info("{}", header);
  
  for (int i = 0; i < num_frames; i++) {
    std::string line = "  " + std::to_string(i) + "   |";
    for (const auto& r : results) {
      char buf[32];
      snprintf(buf, sizeof(buf), " %13.6f |", r.frame_results[i].rot_error);
      line += buf;
    }
    spdlog::info("{}", line);
  }
}

int main(int argc, char** argv) {
  auto console = spdlog::stdout_color_mt("benchmark");
  spdlog::set_default_logger(console);
  spdlog::set_level(spdlog::level::info);
  spdlog::set_pattern("[%^%l%$] %v");

  const std::string data_path = "data/pcd";
  const double noise_scale = 0.1;
  const int num_threads = 4;
  const bool full_connection = true;

  spdlog::info("=== 6-Factor LiDAR Registration Benchmark ===");
  spdlog::info("Data path: {}", data_path);
  spdlog::info("Noise scale: {}", noise_scale);
  spdlog::info("Threads: {}", num_threads);
  spdlog::info("Full connection: {}\n", full_connection);

  DataLoader data;
  if (!data.load(data_path, noise_scale)) {
    spdlog::error("Data loading failed!");
    return 1;
  }

  std::vector<std::string> factor_names = {
    "Point-to-Point", "Point-to-Plane", "GICP", "VGICP", "LOAM", "NDT"
  };

  std::vector<FactorResult> all_results;
  
  for (const auto& name : factor_names) {
    try {
      auto result = run_experiment(name, data, full_connection, num_threads);
      all_results.push_back(result);
    } catch (const std::exception& e) {
      spdlog::error("Factor {} FAILED: {}", name, e.what());
      FactorResult fail;
      fail.name = name + " (FAILED)";
      fail.mean_trans_error = -1;
      fail.mean_rot_error = -1;
      fail.max_trans_error = -1;
      fail.max_rot_error = -1;
      fail.optimization_time_ms = -1;
      fail.num_iterations = -1;
      all_results.push_back(fail);
    }
  }

  spdlog::info("\n\n");
  print_comparison_table(all_results);
  print_per_frame_comparison(all_results);

  spdlog::info("\n=== Benchmark Complete ===");
  return 0;
}
