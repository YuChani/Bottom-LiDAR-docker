#include <chrono>
#include <thread>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <algorithm>
#include <map>
#include <cmath>
#include <boost/format.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam_points/config.hpp>
#include <gtsam_points/util/read_points.hpp>
#include <gtsam_points/features/normal_estimation.hpp>
#include <gtsam_points/features/covariance_estimation.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>

#ifdef GTSAM_POINTS_USE_CUDA
#include <gtsam_points/types/point_cloud_gpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu_create.hpp>
#endif

#include <gtsam_points/factors/integrated_icp_factor.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_points/optimizers/isam2_ext.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/optimizers/linearization_hook.hpp>

#include <glk/thin_lines.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

class MatchingCostFactorDemo {
public:
  MatchingCostFactorDemo() {
    auto viewer = guik::LightViewer::instance();
    viewer->enable_vsync();

    const std::string data_path = "/root/workdir/data/pcd";
    
    // Define T_base_lidar from sensor.yaml
    // T_base_lidar_t_xyz_q_xyzw: [0.0, 0.0, 0.124, 0.0, 0.0, 1.0, 0.0]
    // t = [0, 0, 0.124], q_xyzw = [0, 0, 1, 0] -> Eigen q(w,x,y,z) = (0, 0, 0, 1)
    Eigen::Vector3d t_base_lidar(0.0, 0.0, 0.124);
    Eigen::Quaterniond q_base_lidar(0.0, 0.0, 0.0, 1.0);  // w, x, y, z order in Eigen
    gtsam::Pose3 T_base_lidar(gtsam::Rot3(q_base_lidar), t_base_lidar);
    std::cout << "T_base_lidar translation: " << T_base_lidar.translation().transpose() << std::endl;
    std::cout << "T_base_lidar rotation (ypr): " << T_base_lidar.rotation().ypr().transpose() << std::endl;

    // Read gt-tum.txt and build timestamp -> pose map
    std::map<double, gtsam::Pose3> gt_poses;
    {
      std::ifstream ifs(data_path + "/gt-tum.txt");
      if (!ifs) {
        std::cerr << "error: failed to open " << data_path + "/gt-tum.txt" << std::endl;
        abort();
      }
      
      double timestamp;
      gtsam::Vector3 trans;
      gtsam::Quaternion quat;
      while (ifs >> timestamp >> trans.x() >> trans.y() >> trans.z() >> quat.x() >> quat.y() >> quat.z() >> quat.w()) {
        gt_poses[timestamp] = gtsam::Pose3(gtsam::Rot3(quat), trans);
      }
      std::cout << "Loaded " << gt_poses.size() << " poses from gt-tum.txt" << std::endl;
    }

#ifdef GTSAM_POINTS_USE_CUDA
    std::cout << "Register GPU linearization hook" << std::endl;
    gtsam_points::LinearizationHook::register_hook([] { return gtsam_points::create_nonlinear_factor_set_gpu(); });
#endif

    // Read test data
    frames.resize(5);
    voxelmaps.resize(5);
    voxelmaps_gpu.resize(5);

    // Get list of PCD files sorted by name
    std::vector<std::string> pcd_files;
    {
      namespace fs = std::filesystem;
      for (const auto& entry : fs::directory_iterator(data_path)) 
      {
        if (entry.path().extension() == ".pcd") 
        {
          pcd_files.push_back(entry.path().string());
        }
      }
      std::sort(pcd_files.begin(), pcd_files.end());
    }
    
    if (pcd_files.size() < 5) 
    {
      std::cerr << "error: expected at least 5 PCD files, found " << pcd_files.size() << std::endl;
      abort();
    }

    // Lambda to find closest pose by timestamp
    auto find_closest_pose = [&gt_poses](double target_ts) -> gtsam::Pose3 {
      auto it = gt_poses.lower_bound(target_ts);
      if (it == gt_poses.end()) {
        return gt_poses.rbegin()->second;
      }
      if (it == gt_poses.begin()) {
        return it->second;
      }
      auto prev_it = std::prev(it);
      if (std::abs(it->first - target_ts) < std::abs(prev_it->first - target_ts)) {
        return it->second;
      }
      return prev_it->second;
    };

    // Get first frame's pose as origin (in LiDAR frame)
    namespace fs = std::filesystem;
    std::string first_filename = fs::path(pcd_files[0]).stem().string();
    double first_timestamp = std::stod(first_filename);
    gtsam::Pose3 W_T_B_origin = find_closest_pose(first_timestamp);
    gtsam::Pose3 W_T_L_origin = W_T_B_origin * T_base_lidar;  // World <- LiDAR (origin)

    for (int i = 0; i < 5; i++) 
    {
      const std::string points_path = pcd_files[i];
      std::cout << "loading " << points_path << std::endl;

      // Extract timestamp from PCD filename (e.g., "1710406871.842252000.pcd")
      std::string filename = fs::path(points_path).stem().string();  // "1710406871.842252000"
      double pcd_timestamp = std::stod(filename);
      
      // Find closest pose from gt-tum.txt
      gtsam::Pose3 W_T_B = find_closest_pose(pcd_timestamp);  // World <- Base
      gtsam::Pose3 W_T_L = W_T_B * T_base_lidar;              // World <- LiDAR
      
      // Make relative to origin LiDAR frame: L0_T_Li
      gtsam::Pose3 relative_pose = W_T_L_origin.inverse() * W_T_L;
      
      poses.insert(i, relative_pose);
      poses_gt.insert(i, relative_pose);
      std::cout << "  matched timestamp: " << std::fixed << pcd_timestamp << std::endl;
      std::cout << "  relative pose (L0_T_Li): " << relative_pose.translation().transpose() << std::endl;

      auto points_f = gtsam_points::read_points(points_path);
      if (points_f.empty()) {
        std::cerr << "error: failed to read points " << points_path << std::endl;
        abort();
      }
      
      // Debug: print point stats
      Eigen::Vector3f min_pt = points_f[0], max_pt = points_f[0];
      for (const auto& p : points_f) {
        min_pt = min_pt.cwiseMin(p);
        max_pt = max_pt.cwiseMax(p);
      }
      std::cout << "  loaded " << points_f.size() << " points" << std::endl;
      std::cout << "  bounds: [" << min_pt.transpose() << "] to [" << max_pt.transpose() << "]" << std::endl;

      // Points are in sensor local frame - keep as is
      // The pose will transform them to world frame during visualization and optimization
      std::vector<Eigen::Vector4d> points(points_f.size());
      std::transform(points_f.begin(), points_f.end(), points.begin(), [](const Eigen::Vector3f& p) {
        return (Eigen::Vector4d() << p.cast<double>(), 1.0).finished();
      });
      auto covs = gtsam_points::estimate_covariances(points);

#ifndef GTSAM_POINTS_USE_CUDA
      std::cout << "Create CPU frame" << std::endl;
      auto frame = std::make_shared<gtsam_points::PointCloudCPU>();
#else
      std::cout << "Create GPU frame" << std::endl;
      auto frame = std::make_shared<gtsam_points::PointCloudGPU>();
#endif
      frame->add_points(points);
      frame->add_covs(covs);
      frame->add_normals(gtsam_points::estimate_normals(frame->points, frame->size()));
      frames[i] = frame;

      // voxelmap 사이즈 즉 resolution 
      auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(0.5);
      voxelmap->insert(*frame);
      voxelmaps[i] = voxelmap;

#ifdef GTSAM_POINTS_USE_CUDA
      auto voxelmap_gpu = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(0.5);
      voxelmap_gpu->insert(*frame);
      voxelmaps_gpu[i] = voxelmap_gpu;
#endif

      viewer->update_drawable(
        "frame_" + std::to_string(i), 
        std::make_shared<glk::PointCloudBuffer>(frame->points, frame->size()), 
        guik::Rainbow().add("model_matrix", Eigen::Isometry3f(relative_pose.matrix().cast<float>())));
    }

    update_viewer(poses);

    pose_noise_scale = 0.1;

    optimizer_type = 0;
    optimizer_types.push_back("LM");
    optimizer_types.push_back("ISAM2");

    factor_type = 0;
    factor_types.push_back("ICP");
    factor_types.push_back("ICP_PLANE");
    factor_types.push_back("GICP");
    factor_types.push_back("VGICP");
#ifdef GTSAM_POINTS_USE_CUDA
    factor_types.push_back("VGICP_GPU");
#endif

    full_connection = true;
    num_threads = 1;

    correspondence_update_tolerance_rot = 0.0f;
    correspondence_update_tolerance_trans = 0.0f;

    viewer->register_ui_callback("control", [this] {
      // Add noise on the initial poses
      ImGui::DragFloat("noise_scale", &pose_noise_scale, 0.01f, 0.0f);
      if (ImGui::Button("add noise")) {
        for (int i = 1; i < 5; i++) {
          gtsam::Pose3 noise = gtsam::Pose3::Expmap(gtsam::Vector6::Random() * pose_noise_scale);
          poses.update<gtsam::Pose3>(i, poses_gt.at<gtsam::Pose3>(i) * noise);
        }
        update_viewer(poses);
      }

      // Optimization configurations
      ImGui::Separator();
      ImGui::Checkbox("full connection", &full_connection);
      ImGui::DragInt("num threads", &num_threads, 1, 1, 128);
      ImGui::Combo("factor type", &factor_type, factor_types.data(), factor_types.size());
      ImGui::Combo("optimizer type", &optimizer_type, optimizer_types.data(), optimizer_types.size());

      ImGui::DragFloat("corr update tolerance rot", &correspondence_update_tolerance_rot, 0.001f, 0.0f, 0.1f);
      ImGui::DragFloat("corr update tolerance trans", &correspondence_update_tolerance_trans, 0.01f, 0.0f, 1.0f);

      // Run optimization
      if (ImGui::Button("optimize")) {
        if (optimization_thread.joinable()) {
          optimization_thread.join();
        }
        optimization_thread = std::thread([this] { run_optimization(); });
      }
    });
  }

  ~MatchingCostFactorDemo() {
    if (optimization_thread.joinable()) 
    {
      optimization_thread.join();
    }
  }

  void update_viewer(const gtsam::Values& values) 
  {
    guik::LightViewer::instance()->invoke([=] 
      {
      auto viewer = guik::LightViewer::instance();

      std::vector<Eigen::Vector3f> factor_lines;
      for (int i = 0; i < 5; i++) 
      {
        Eigen::Isometry3f pose(values.at<gtsam::Pose3>(i).matrix().cast<float>());

        auto drawable = viewer->find_drawable("frame_" + std::to_string(i));
        drawable.first->add("model_matrix", pose);
        viewer->update_drawable(
          "coord_" + std::to_string(i),
          glk::Primitives::coordinate_system(),
          guik::VertexColor(pose * Eigen::UniformScaling<float>(5.0f)));

        int j_end = full_connection ? 5 : std::min(i + 2, 5);

        for (int j = i + 1; j < j_end; j++) 
        {
          factor_lines.push_back(values.at<gtsam::Pose3>(i).translation().cast<float>());
          factor_lines.push_back(values.at<gtsam::Pose3>(j).translation().cast<float>());
        }
      }

      viewer->update_drawable("factors", std::make_shared<glk::ThinLines>(factor_lines), guik::FlatColor(0.0f, 1.0f, 0.0f, 1.0f));
    });
  }

  gtsam::NonlinearFactor::shared_ptr create_factor(
    gtsam::Key target_key,
    gtsam::Key source_key,
    const gtsam_points::PointCloud::ConstPtr& target,
    const gtsam_points::GaussianVoxelMap::ConstPtr& target_voxelmap,
    const gtsam_points::GaussianVoxelMap::ConstPtr& target_voxelmap_gpu,
    const gtsam_points::PointCloud::ConstPtr& source) {

    if (factor_types[factor_type] == std::string("ICP")) 
    {
      auto factor = gtsam::make_shared<gtsam_points::IntegratedICPFactor>(target_key, source_key, target, source);
      factor->set_correspondence_update_tolerance(correspondence_update_tolerance_rot, correspondence_update_tolerance_trans);
      factor->set_num_threads(num_threads);
      return factor;
    } 

    else if (factor_types[factor_type] == std::string("ICP_PLANE")) 
    {
      auto factor = gtsam::make_shared<gtsam_points::IntegratedPointToPlaneICPFactor>(target_key, source_key, target, source);
      factor->set_correspondence_update_tolerance(correspondence_update_tolerance_rot, correspondence_update_tolerance_trans);
      factor->set_num_threads(num_threads);
      return factor;
    } 

    else if (factor_types[factor_type] == std::string("GICP")) 
    {
      auto factor = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(target_key, source_key, target, source);
      factor->set_correspondence_update_tolerance(correspondence_update_tolerance_rot, correspondence_update_tolerance_trans);
      factor->set_num_threads(num_threads);
      return factor;
    } 

    else if (factor_types[factor_type] == std::string("VGICP")) 
    {
      auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(target_key, source_key, target_voxelmap, source);
      factor->set_num_threads(num_threads);
      return factor;
    } 
    else if (factor_types[factor_type] == std::string("VGICP_GPU")) 
    {
#ifdef GTSAM_POINTS_USE_CUDA
      return gtsam::make_shared<gtsam_points::IntegratedVGICPFactorGPU>(target_key, source_key, target_voxelmap_gpu, source);
#endif
    }

    std::cerr << "error: unknown factor type " << factor_types[factor_type] << std::endl;
    return nullptr;
  }

  void run_optimization() {
    gtsam::NonlinearFactorGraph graph;
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, poses.at<gtsam::Pose3>(0), gtsam::noiseModel::Isotropic::Precision(6, 1e6)));

    // Create factors
    for (int i = 0; i < 5; i++) {
      // If full_connection == false, factors are only created between consecutive frames
      int j_end = full_connection ? 5 : std::min(i + 2, 5);
      for (int j = i + 1; j < j_end; j++) {
        auto factor = create_factor(i, j, frames[i], voxelmaps[i], voxelmaps_gpu[i], frames[j]);
        graph.add(factor);
      }
    }

    std::cout << "\n========================================" << std::endl;
    std::cout << "Factor Type: " << factor_types[factor_type] << std::endl;
    std::cout << "Optimizer: " << optimizer_types[optimizer_type] << std::endl;
    std::cout << "========================================" << std::endl;

    gtsam::Values optimized_values;

    // Levenberg-Marquardt optimization
    if (optimizer_types[optimizer_type] == std::string("LM")) {
      gtsam_points::LevenbergMarquardtExtParams lm_params;
      lm_params.maxIterations = 100;
      lm_params.relativeErrorTol = 1e-5;
      lm_params.absoluteErrorTol = 1e-5;
      lm_params.callback = [this](const gtsam_points::LevenbergMarquardtOptimizationStatus& status, const gtsam::Values& values) {
        guik::LightViewer::instance()->append_text(status.to_string());
        update_viewer(values);
      };

      gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, poses, lm_params);
      optimized_values = optimizer.optimize();
    }
    // iSAM2 optimization
    else if (optimizer_types[optimizer_type] == std::string("ISAM2")) {
      gtsam::ISAM2Params isam2_params;
      isam2_params.relinearizeSkip = 1;
      isam2_params.setRelinearizeThreshold(0.0);
      gtsam_points::ISAM2Ext isam2(isam2_params);

      auto t1 = std::chrono::high_resolution_clock::now();
      auto status = isam2.update(graph, poses);
      update_viewer(isam2.calculateEstimate());
      guik::LightViewer::instance()->append_text(status.to_string());

      for (int i = 0; i < 5; i++) {
        auto status = isam2.update();
        update_viewer(isam2.calculateEstimate());
        guik::LightViewer::instance()->append_text(status.to_string());
      }

      auto t2 = std::chrono::high_resolution_clock::now();
      double msec = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6;
      guik::LightViewer::instance()->append_text((boost::format("total:%.3f[msec]") % msec).str());
      
      optimized_values = isam2.calculateEstimate();
    }

    // Print comparison: Optimized vs GT
    std::cout << "\n--- Results: " << factor_types[factor_type] << " ---" << std::endl;
    for (int i = 0; i < 5; i++) 
    {
      gtsam::Pose3 opt_pose = optimized_values.at<gtsam::Pose3>(i);
      gtsam::Pose3 gt_pose = poses_gt.at<gtsam::Pose3>(i);
      
      // Compute error
      gtsam::Pose3 error = gt_pose.inverse() * opt_pose;
      
      std::cout << "\nFrame " << i << ":" << std::endl;
      std::cout << "  [Optimized] t: " << opt_pose.translation().transpose() << std::endl;
      std::cout << "              R (ypr): " << opt_pose.rotation().ypr().transpose() * 180.0 / M_PI << " deg" << std::endl;
      std::cout << "  [GT]        t: " << gt_pose.translation().transpose() << std::endl;
      std::cout << "              R (ypr): " << gt_pose.rotation().ypr().transpose() * 180.0 / M_PI << " deg" << std::endl;
      std::cout << "  [Error]     t: " << error.translation().norm() << " m" << std::endl;
      std::cout << "              R: " << error.rotation().axisAngle().second * 180.0 / M_PI << " deg" << std::endl;
    }
    
    // Summary statistics
    double total_trans_error = 0.0;
    double total_rot_error = 0.0;
    for (int i = 0; i < 5; i++) 
    {
      gtsam::Pose3 opt_pose = optimized_values.at<gtsam::Pose3>(i);
      gtsam::Pose3 gt_pose = poses_gt.at<gtsam::Pose3>(i);
      gtsam::Pose3 error = gt_pose.inverse() * opt_pose;
      total_trans_error += error.translation().norm();
      total_rot_error += error.rotation().axisAngle().second * 180.0 / M_PI;
    }
    std::cout << "\n--- Summary ---" << std::endl;
    std::cout << "Mean Translation Error: " << total_trans_error / 5.0 << " m" << std::endl;
    std::cout << "Mean Rotation Error: " << total_rot_error / 5.0 << " deg" << std::endl;
    std::cout << "========================================\n" << std::endl;
  }

private:
  float pose_noise_scale;

  std::vector<const char*> factor_types;
  int factor_type;
  bool full_connection;
  int num_threads;

  std::vector<const char*> optimizer_types;
  int optimizer_type;

  float correspondence_update_tolerance_rot;
  float correspondence_update_tolerance_trans;

  std::thread optimization_thread;

  gtsam::Values poses;
  gtsam::Values poses_gt;
  std::vector<gtsam_points::PointCloud::Ptr> frames;
  std::vector<gtsam_points::GaussianVoxelMap::Ptr> voxelmaps;
  std::vector<gtsam_points::GaussianVoxelMap::Ptr> voxelmaps_gpu;
};

int main(int argc, char** argv) {
  MatchingCostFactorDemo demo;
  guik::LightViewer::instance()->spin();
  return 0;
}