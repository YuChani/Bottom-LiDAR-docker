/**
 * main.cpp - LiDAR Point Cloud Registration Demo
 * 
 * 상세 문서는 what_main.md 파일을 참조하세요.
 */

#include <chrono>
#include <thread>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <algorithm>
#include <map>
#include <cmath>
#include <boost/format.hpp>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/fmt/ostr.h>

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

#ifdef GTSAM_POINTS_USE_CUDA
#include <gtsam_points/types/point_cloud_gpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu_create.hpp>
#endif

#include <gtsam_points/factors/integrated_icp_factor.hpp>
#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/factors/integrated_loam_factor.hpp>
#include <gtsam_points/optimizers/isam2_ext.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/optimizers/linearization_hook.hpp>

#include <glk/thin_lines.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

// LOAM Feature Extraction (Ring 기반 / KNN 기반)
#include "loam_feature.hpp"

class MatchingCostFactorDemo
{
public:
  MatchingCostFactorDemo()
  {
    // Logging 초기화
    auto console = spdlog::stdout_color_mt("demo");
    spdlog::set_default_logger(console);
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%^%l%$] %v");
    
    // 3D Viewer 초기화
    auto viewer = guik::LightViewer::instance();
    viewer->enable_vsync();
    // yuchan_step data loader(pcd데이터파일 경로)
    const std::string data_path = "/root/workdir/data/pcd";

    // yuchan_step : Extrinsic: Base ← LiDAR (t = [0, 0, 0.124], R = I)
    // Base에서 LiDAR로 포즈를 변환시키는 부분 (sensor.yaml에서 값을 가져옴. 하드코딩임)
    Eigen::Vector3d t_base_lidar(0.0, 0.0, 0.124);
    Eigen::Quaterniond q_base_lidar(0.0, 0.0, 0.0, 1.0);
    gtsam::Pose3 T_base_lidar(gtsam::Rot3(q_base_lidar), t_base_lidar);
    spdlog::info("T_base_lidar translation: [{:.3f}, {:.3f}, {:.3f}]", 
                 T_base_lidar.translation().x(), T_base_lidar.translation().y(), T_base_lidar.translation().z());
    auto ypr = T_base_lidar.rotation().ypr();
    spdlog::info("T_base_lidar rotation (ypr): [{:.3f}, {:.3f}, {:.3f}]", ypr.x(), ypr.y(), ypr.z());

    // Ground Truth 포즈 로드 (TUM format)
    // gt_tum.txt : tx, yz, yz, qw, qx, qy, qz 형태로 구성되어있음
    std::map<double, gtsam::Pose3> gt_poses;
    {
      std::ifstream ifs(data_path + "/gt-tum.txt");
      if (!ifs)
      {
        spdlog::error("Failed to open {}/gt-tum.txt", data_path);
        abort();
      }
      
      double timestamp;
      gtsam::Vector3 trans;
      gtsam::Quaternion quat;
      while (ifs >> timestamp >> trans.x() >> trans.y() >> trans.z() >> quat.x() >> quat.y() >> quat.z() >> quat.w())
      {
        gt_poses[timestamp] = gtsam::Pose3(gtsam::Rot3(quat), trans);
      }
      spdlog::info("Loaded {} poses from gt-tum.txt", gt_poses.size());
    }

#ifdef GTSAM_POINTS_USE_CUDA
    spdlog::info("Register GPU linearization hook");
    gtsam_points::LinearizationHook::register_hook([] { return gtsam_points::create_nonlinear_factor_set_gpu(); });
#endif

    // PCD 파일 목록 수집 및 정렬
    int max_frames = 100;
    
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
    
    if (pcd_files.size() < 2)
    {
      spdlog::error("Expected at least 2 PCD files, found {}", pcd_files.size());
      abort();
    }
    
    int num_frames = std::min(max_frames, (int)pcd_files.size());
    spdlog::info("Found {} PCD files, loading {} frames", pcd_files.size(), num_frames);
    
    frames.resize(num_frames);
    voxelmaps.resize(num_frames);

    // Timestamp → Pose 매칭 함수
    auto find_closest_pose = [&gt_poses](double target_ts) -> gtsam::Pose3
    {
      auto it = gt_poses.lower_bound(target_ts);
      if (it == gt_poses.end()) return gt_poses.rbegin()->second;
      if (it == gt_poses.begin()) return it->second;
      auto prev_it = std::prev(it);
      if (std::abs(it->first - target_ts) < std::abs(prev_it->first - target_ts))
        return it->second;
      return prev_it->second;
    };

    // yuchan_step : Origin 프레임 설정
    // world좌표에서 첫 프레임의 LiDAR 포즈를 원점으로 설정
    // 수식 : W_T_L_origin = W_T_B_origin * T_base_lidar(sensor.yaml 변수명과 동일 T_base_lidar만)
    namespace fs = std::filesystem;
    std::string first_filename = fs::path(pcd_files[0]).stem().string();
    double first_timestamp = std::stod(first_filename);
    gtsam::Pose3 W_T_B_origin = find_closest_pose(first_timestamp);
    gtsam::Pose3 W_T_L_origin = W_T_B_origin * T_base_lidar;

    // 각 프레임 로드 및 전처리
    for (int i = 0; i < num_frames; i++)
    {
      const std::string points_path = pcd_files[i];
      spdlog::info("Loading {}", points_path);

      std::string filename = fs::path(points_path).stem().string();
      double pcd_timestamp = std::stod(filename);
      
      // 상대 포즈 계산: L0_T_Li = W_T_L_origin^(-1) * W_T_L_i
      // 현재 프레임 i의 절대포즈 W_T_B_i 계산
      gtsam::Pose3 W_T_B = find_closest_pose(pcd_timestamp);
      gtsam::Pose3 W_T_L = W_T_B * T_base_lidar;
      gtsam::Pose3 relative_pose = W_T_L_origin.inverse() * W_T_L;
      
      poses.insert(i, relative_pose);   // initial guess값으로 사용
      poses_gt.insert(i, relative_pose);  // ground truth로 사용(나중에 에러 비교용)
      
      spdlog::info("  Matched timestamp: {:.6f}", pcd_timestamp);
      auto t = relative_pose.translation();
      spdlog::info("  Relative pose (L0_T_Li): [{:.3f}, {:.3f}, {:.3f}]", t.x(), t.y(), t.z());

      // 포인트 클라우드 로드 (PCL 사용, Ring 정보 포함)
      auto points_with_ring = read_points_with_ring_pcl(points_path);
      if (points_with_ring.empty())
      {
        spdlog::error("Failed to read points from {}", points_path);
        abort();
      }
      
      // Ring 정보 저장 (LOAM feature 추출용)
      frames_with_ring.push_back(points_with_ring);
      
      // 기존 호환성을 위해 points_f도 생성
      std::vector<Eigen::Vector3f> points_f;
      points_f.reserve(points_with_ring.size());
      for (const auto& p : points_with_ring)
      {
        points_f.push_back(p.point.head<3>().cast<float>());
      }
      
      Eigen::Vector3f min_pt = points_f[0], max_pt = points_f[0];
      for (const auto& p : points_f)
      {
        min_pt = min_pt.cwiseMin(p);
        max_pt = max_pt.cwiseMax(p);
      }
      spdlog::info("  Loaded {} points", points_f.size());

      // yuchan_step : float → double 변환 (homogeneous 좌표)
      // 3차원 점(x,y,z)를 4차원 homogeneous 좌표 (x,y,z,1)로 변환. R과t를 하나의 4x4행렬로 표현해서 선형변환.(p'=R*p+t)
      std::vector<Eigen::Vector4d> points(points_f.size());
      std::transform(points_f.begin(), points_f.end(), points.begin(), [](const Eigen::Vector3f& p)
      {
        return (Eigen::Vector4d() << p.cast<double>(), 1.0).finished();
      });

      // yuchan_step : 공분산(covariance) 행렬 추정
      // 가우시안 복셀맵에서 사용될 공분산 행렬을 각 포인트에 대해 추정. KD-tree 기반 최근접 이웃 검색 사용
      // plane : 납작한 타원 / edge : 길쭉한 타원 / corner : 거의 구형
      auto covs = gtsam_points::estimate_covariances(points);

#ifndef GTSAM_POINTS_USE_CUDA
      spdlog::debug("Create CPU frame");
      auto frame = std::make_shared<gtsam_points::PointCloudCPU>();
#else
      spdlog::debug("Create GPU frame");
      auto frame = std::make_shared<gtsam_points::PointCloudGPU>();
#endif
      frame->add_points(points);
      frame->add_covs(covs);

      // yuchan_step : 법선 벡터 추정
      // p2pl : point-to-plane ICP에서 사용되는 법선 벡터 추정 방법(e = n^Tp+d)
      // LOAM 특징점 추출에도 사용됨(curvature 계산용)
      frame->add_normals(gtsam_points::estimate_normals(frame->points, frame->size()));
      frames[i] = frame;

      // yuchan_step : LOAM 특징점 추출 (Ring 기반)
      spdlog::info("  Extracting LOAM features (Ring-based)...");
      auto features = extract_loam_features_ring_based(points_with_ring);
      loam_features.push_back(features);
      spdlog::info("Edge points: {}, Planar points: {}", features.edge_points->size(), features.planar_points->size());

      // yuchan_step : 가우시안 복셀맵 생성 (resolution = 0.5m)
      // VG-ICP에서 사용하는 r값을 설정하는 부분
      auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(0.5);
      voxelmap->insert(*frame);
      voxelmaps[i] = voxelmap;

#ifdef GTSAM_POINTS_USE_CUDA
      auto voxelmap_gpu = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(0.5);
      voxelmap_gpu->insert(*frame);
      voxelmaps_gpu[i] = voxelmap_gpu;
#endif

      // 시각화 업데이트
      viewer->update_drawable(
        "frame_" + std::to_string(i), 
        std::make_shared<glk::PointCloudBuffer>(frame->points, frame->size()), 
        guik::Rainbow().add("model_matrix", Eigen::Isometry3f(relative_pose.matrix().cast<float>())));
    }

    update_viewer(poses);

    // yuchan_step : UI 파라미터 초기화
    pose_noise_scale = 0.1; // 포즈 노이즈 스케일. GT에 인위적인 노이즈 추가

    optimizer_type = 0; // 기본 최적화 알고리즘은 0 : LM or 1이면 ISAM2
    optimizer_types.push_back("LM");
    optimizer_types.push_back("ISAM2");

    factor_type = 2;  // 기본 팩터 최적화 알고리즘 타입은 2 : GICP
    factor_types.push_back("Point-to-Point"); // 0 : Point-to-point ICP
    factor_types.push_back("Point-to-Plane"); // 1 : Point-to-plane ICP
    factor_types.push_back("GICP"); // 2 : GICP
    factor_types.push_back("VGICP"); // 3 : VG-ICP
    factor_types.push_back("LOAM"); // 4 : LOAM

#ifdef GTSAM_POINTS_USE_CUDA
    factor_types.push_back("VGICP_GPU"); // 5 : VG-ICP GPU
#endif
    full_connection = true;
    num_threads = 1;

    correspondence_update_tolerance_rot = 0.0f;
    correspondence_update_tolerance_trans = 0.0f;

    // UI 콜백 등록
    viewer->register_ui_callback("control", [this]
    {
      int num_frames = frames.size();
      
      ImGui::DragFloat("noise_scale", &pose_noise_scale, 0.01f, 0.0f);
      if (ImGui::Button("add noise"))
      {
        for (int i = 1; i < num_frames; i++)
        {
          gtsam::Pose3 noise = gtsam::Pose3::Expmap(gtsam::Vector6::Random() * pose_noise_scale);
          poses.update<gtsam::Pose3>(i, poses_gt.at<gtsam::Pose3>(i) * noise);
        }
        update_viewer(poses);
      }

      ImGui::Separator();
      ImGui::Checkbox("full connection", &full_connection);
      ImGui::DragInt("num threads", &num_threads, 1, 1, 128);
      ImGui::Combo("factor type", &factor_type, factor_types.data(), factor_types.size());
      ImGui::Combo("optimizer type", &optimizer_type, optimizer_types.data(), optimizer_types.size());

      ImGui::DragFloat("corr update tolerance rot", &correspondence_update_tolerance_rot, 0.001f, 0.0f, 0.1f);
      ImGui::DragFloat("corr update tolerance trans", &correspondence_update_tolerance_trans, 0.01f, 0.0f, 1.0f);

      if (ImGui::Button("optimize"))
      {
        if (optimization_thread.joinable())
        {
          optimization_thread.join();
        }
        optimization_thread = std::thread([this]
        {
          run_optimization();
        });
      }
    });
  }

  ~MatchingCostFactorDemo()
  {
    if (optimization_thread.joinable())
    {
      optimization_thread.join();
    }
  }

  // yuchan_step : 뷰어 업데이트 함수(ImGui)
  void update_viewer(const gtsam::Values& values)
  {
    guik::LightViewer::instance()->invoke([=]
    {
      auto viewer = guik::LightViewer::instance();

      std::vector<Eigen::Vector3f> factor_lines;  // graph의 edge를 그림
      int num_frames = values.size();
      for (int i = 0; i < num_frames; i++)
      {
        Eigen::Isometry3f pose(values.at<gtsam::Pose3>(i).matrix().cast<float>());
        // yuchan_step : pose 변환
        auto drawable = viewer->find_drawable("frame_" + std::to_string(i));
        drawable.first->add("model_matrix", pose);
        // RGB축 그림
        viewer->update_drawable(
          "coord_" + std::to_string(i),
          glk::Primitives::coordinate_system(),
          guik::VertexColor(pose * Eigen::UniformScaling<float>(5.0f)));
        
        // 프레임끼리 연결 되어있는지 초록선으로 그림. full_connection이면 모든 프레임과 연결, 아니면 인접한 프레임과만 연결
        int j_end = full_connection ? num_frames : std::min(i + 2, num_frames);
        for (int j = i + 1; j < j_end; j++)
        {
          factor_lines.push_back(values.at<gtsam::Pose3>(i).translation().cast<float>());
          factor_lines.push_back(values.at<gtsam::Pose3>(j).translation().cast<float>());
        }
      }

      viewer->update_drawable("factors", std::make_shared<glk::ThinLines>(factor_lines), guik::FlatColor(0.0f, 1.0f, 0.0f, 1.0f));
    });
  }

  // yuchan_step : 팩터 생성 함수
  // target : 기준 프레임, source : 변환될 프레임
  gtsam::NonlinearFactor::shared_ptr create_factor(
    gtsam::Key target_key,  // 기준포즈
    gtsam::Key source_key, // 변환될 포즈
    const gtsam_points::PointCloud::ConstPtr& target,
    const gtsam_points::GaussianVoxelMap::ConstPtr& target_voxelmap,
    const gtsam_points::GaussianVoxelMap::ConstPtr& target_voxelmap_gpu,
    const gtsam_points::PointCloud::ConstPtr& source)
  {
    // yuchan : Point-to-point ICP 팩터 생성
    // correspondence_update_tolerance_rot, correspondence_update_tolerance_trans
    // cost function = Σ || p_target - (R * p_source + t) ||^2
    if (factor_types[factor_type] == std::string("ICP"))
    {
      auto factor = gtsam::make_shared<gtsam_points::IntegratedICPFactor>(target_key, source_key, target, source);
      factor->set_correspondence_update_tolerance(correspondence_update_tolerance_rot, correspondence_update_tolerance_trans);
      factor->set_num_threads(num_threads);
      return factor;
    }
    // yuchan : Point-to-plane ICP 팩터 생성
    // cost function = Σ (n^T * (p_target - (R * p_source + t)))^2
    else if (factor_types[factor_type] == std::string("ICP_PLANE"))
    {
      auto factor = gtsam::make_shared<gtsam_points::IntegratedPointToPlaneICPFactor>(target_key, source_key, target, source);
      factor->set_correspondence_update_tolerance(correspondence_update_tolerance_rot, correspondence_update_tolerance_trans);
      factor->set_num_threads(num_threads);
      return factor;
    }
    // yuchan : GICP 팩터 생성
    // cost function = Σ (p_target - (R * p_source + t))^T * (Σ_target + R * Σ_source * R^T)^(-1) * (p_target - (R * p_source + t))
    else if (factor_types[factor_type] == std::string("GICP"))
    {
      auto factor = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(target_key, source_key, target, source);
      factor->set_correspondence_update_tolerance(correspondence_update_tolerance_rot, correspondence_update_tolerance_trans);
      factor->set_num_threads(num_threads);
      return factor;
    }
    // yuchan : VG-ICP 팩터 생성
    // cost function = Σ (p_target - (R * p_source + t))^T * (Σ_voxel_target + R * Σ_voxel_source * R^T)^(-1) * (p_target - (R * p_source + t))
    else if (factor_types[factor_type] == std::string("VGICP"))
    {
      auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(target_key, source_key, target_voxelmap, source);
      factor->set_num_threads(num_threads);
      return factor;
    }
    // yuchan : LOAM 팩터 생성
    // LOAM 특징점(Edge, Planar points) 기반의 팩터 생성
    // Edge : Point-to-line distance 최소화 / Planar : Point-to-plane distance 최소화
    else if (factor_types[factor_type] == std::string("LOAM"))
    {
      auto factor = gtsam::make_shared<gtsam_points::IntegratedLOAMFactor>(
        target_key, source_key, 
        loam_features[target_key].edge_points, loam_features[target_key].planar_points,
        loam_features[source_key].edge_points, loam_features[source_key].planar_points);
      
      factor->set_enable_correspondence_validation(true); // correspondence 검증 활성화
      factor->set_max_correspondence_distance(2.0, 2.0);
      factor->set_correspondence_update_tolerance(correspondence_update_tolerance_rot, correspondence_update_tolerance_trans);
      factor->set_num_threads(num_threads);
      return factor;
    }

    spdlog::error("Unknown factor type: {}", factor_types[factor_type]);
    return nullptr;
  }

  void run_optimization()
  {
    int num_frames = frames.size();
    
    gtsam::NonlinearFactorGraph graph;
    
    // Prior Factor: 첫 번째 포즈 고정
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, poses.at<gtsam::Pose3>(0), gtsam::noiseModel::Isotropic::Precision(6, 1e6)));

    // Registration Factors 추가
    for (int i = 0; i < num_frames; i++)
    {
      int j_end = full_connection ? num_frames : std::min(i + 2, num_frames);
      for (int j = i + 1; j < j_end; j++)
      {
        auto factor = create_factor(i, j, frames[i], voxelmaps[i], nullptr, frames[j]);
        graph.add(factor);
      }
    }

    spdlog::info("========================================");
    spdlog::info("Frames: {}", num_frames);
    spdlog::info("Factor Type: {}", factor_types[factor_type]);
    spdlog::info("Optimizer: {}", optimizer_types[optimizer_type]);
    spdlog::info("========================================");

    gtsam::Values optimized_values;

    // optmizer 선택 및 실행(LM or ISAM2)
    if (optimizer_types[optimizer_type] == std::string("LM"))
    {
      gtsam_points::LevenbergMarquardtExtParams lm_params;
      lm_params.maxIterations = 100;
      lm_params.relativeErrorTol = 1e-5;
      lm_params.absoluteErrorTol = 1e-5;
      
      lm_params.callback = [this](const gtsam_points::LevenbergMarquardtOptimizationStatus& status, const gtsam::Values& values)
      {
        guik::LightViewer::instance()->append_text(status.to_string());
        spdlog::info("{}", status.to_string());
        update_viewer(values);
      };

      gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, poses, lm_params);
      optimized_values = optimizer.optimize();
    }
    else if (optimizer_types[optimizer_type] == std::string("ISAM2"))
    {
      gtsam::ISAM2Params isam2_params;
      isam2_params.relinearizeSkip = 1;
      isam2_params.setRelinearizeThreshold(0.0);
      gtsam_points::ISAM2Ext isam2(isam2_params);

      auto t1 = std::chrono::high_resolution_clock::now();
      
      auto status = isam2.update(graph, poses);
      update_viewer(isam2.calculateEstimate());
      guik::LightViewer::instance()->append_text(status.to_string());

      for (int i = 0; i < 5; i++)
      {
        auto status = isam2.update();
        update_viewer(isam2.calculateEstimate());
        guik::LightViewer::instance()->append_text(status.to_string());
      }

      auto t2 = std::chrono::high_resolution_clock::now();
      double msec = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6;
      guik::LightViewer::instance()->append_text((boost::format("total:%.3f[msec]") % msec).str());
      
      optimized_values = isam2.calculateEstimate();
    }

    // 결과 분석 및 출력
    spdlog::info("--- Results: {} ---", factor_types[factor_type]);
    for (int i = 0; i < num_frames; i++)
    {
      gtsam::Pose3 opt_pose = optimized_values.at<gtsam::Pose3>(i);
      gtsam::Pose3 gt_pose = poses_gt.at<gtsam::Pose3>(i);
      
      gtsam::Pose3 error = gt_pose.inverse() * opt_pose;
      
      auto opt_t = opt_pose.translation();
      auto opt_ypr = opt_pose.rotation().ypr() * 180.0 / M_PI;
      auto gt_t = gt_pose.translation();
      auto gt_ypr = gt_pose.rotation().ypr() * 180.0 / M_PI;
      
      spdlog::info("Frame {}:", i);
      spdlog::info("  [Optimized] t: [{:.6f}, {:.6f}, {:.6f}]", opt_t.x(), opt_t.y(), opt_t.z());
      spdlog::info("              R (ypr): [{:.3f}, {:.3f}, {:.3f}] deg", opt_ypr.x(), opt_ypr.y(), opt_ypr.z());
      spdlog::info("  [GT]        t: [{:.6f}, {:.6f}, {:.6f}]", gt_t.x(), gt_t.y(), gt_t.z());
      spdlog::info("              R (ypr): [{:.3f}, {:.3f}, {:.3f}] deg", gt_ypr.x(), gt_ypr.y(), gt_ypr.z());
      spdlog::info("  [Error]     t: {:.6f} m", error.translation().norm());
      spdlog::info("              R: {:.6f} deg", error.rotation().axisAngle().second * 180.0 / M_PI);
    }
    
    // Summary Statistics
    double total_trans_error = 0.0;
    double total_rot_error = 0.0;
    for (int i = 0; i < num_frames; i++)
    {
      gtsam::Pose3 opt_pose = optimized_values.at<gtsam::Pose3>(i);
      gtsam::Pose3 gt_pose = poses_gt.at<gtsam::Pose3>(i);
      gtsam::Pose3 error = gt_pose.inverse() * opt_pose;
      total_trans_error += error.translation().norm();
      total_rot_error += error.rotation().axisAngle().second * 180.0 / M_PI;
    }
    spdlog::info("--- Summary ---");
    spdlog::info("Mean Translation Error: {:.6f} m", total_trans_error / num_frames);
    spdlog::info("Mean Rotation Error: {:.6f} deg", total_rot_error / num_frames);
    spdlog::info("========================================");
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

  gtsam::Values poses;  // initial guess(현재 추정중인 경로)
  gtsam::Values poses_gt; // ground truth 경로
  
  std::vector<gtsam_points::PointCloud::Ptr> frames;  // point, 법선, 공분산 정보 가지고있음
  std::vector<gtsam_points::GaussianVoxelMap::Ptr> voxelmaps; // VG-ICP용 복셀맵
  
  std::vector<LOAMFeatures> loam_features;
  
  // yuchan : 각 프레임의 Ring 정보를 저장
  std::vector<std::vector<PointWithRing>> frames_with_ring;
};


int main(int argc, char** argv)
{
  MatchingCostFactorDemo demo;
  guik::LightViewer::instance()->spin();
  return 0;
}