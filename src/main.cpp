/**
 * main.cpp - LiDAR Point Cloud Registration Demo
 * 
 * 상세 문서는 what_main.md 파일을 참조하세요.
 * 
 * 실행 모드:
 *   GUI 모드 (기본):  ./lidar_registration_benchmark
 *   Headless 모드:    ./lidar_registration_benchmark --headless
 *     → GUI 없이 6가지 factor를 순차 실행하고 R,t 오차를 비교 출력
 */

#include <chrono>
#include <thread>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <algorithm>
#include <map>
#include <cmath>
#include <cstring>
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
// NDT factor from gtsam_points library
#include <gtsam_points/factors/integrated_ndt_factor.hpp>

#include <gtsam_points/optimizers/isam2_ext.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/optimizers/linearization_hook.hpp>

#include <glk/thin_lines.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

// LOAM Feature Extraction (Ring 기반 / KNN 기반)
#include "loam_feature.hpp"
#include "featureExtraction.hpp"


class MatchingCostFactorDemo
{
public:
  MatchingCostFactorDemo(bool headless_mode = false) : headless(headless_mode)
  {
    // Logging 초기화
    auto console = spdlog::stdout_color_mt("demo");
    spdlog::set_default_logger(console);
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%^%l%$] %v");
    
    // 3D Viewer 초기화 (GUI 모드에서만)
    guik::LightViewer* viewer = nullptr;
    if (!headless)
    {
      viewer = guik::LightViewer::instance();
      viewer->enable_vsync();
    }

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

      // yuchan_step : LOAM 특징점 추출 (Ring 기반) — 비활성화 (LIO-SAM 디버깅 중)
      // spdlog::info("  Extracting LOAM features (Ring-based)...");
      // auto features = extract_loam_features_ring_based(points_with_ring);
      // loam_features.push_back(features);
      // spdlog::info("Edge points: {}, Planar points: {}", features.edge_points->size(), features.planar_points->size());
      // yuchan_step : LIO-SAM 기반 LOAM 특징점 추출
      spdlog::info("  Extracting LOAM features (LIO-SAM)...");
      auto features_liosam = feature_extractor.process(points_with_ring);
      loam_features_liosam.push_back(features_liosam);
      spdlog::info("LIO-SAM Edge points: {}, Planar points: {}", features_liosam.edge_points->size(), features_liosam.planar_points->size());

      // yuchan_step : 가우시안 복셀맵 생성 (resolution = 1.0m)
      // VG-ICP에서 사용하는 r값을 설정하는 부분
      // Koide fast_gicp 기본값: 1.0m, small_gicp 벤치마크: 2.0m (outdoor)
      auto voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(1.0);
      voxelmap->insert(*frame);
      voxelmaps[i] = voxelmap;

#ifdef GTSAM_POINTS_USE_CUDA
      auto voxelmap_gpu = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(1.0);
      voxelmap_gpu->insert(*frame);
      voxelmaps_gpu[i] = voxelmap_gpu;
#endif

      // 시각화 업데이트 (GUI 모드에서만)
      if (!headless && viewer)
      {
        // ============================================================
        // [일반 포인트 클라우드 시각화 설정]
        // ============================================================
        // ● 색상 모드 (아래 중 택1):
        //   guik::VertexColor()                        — 포인트별 개별 색상 (add_color 필요)
        //   guik::Rainbow()                             — Z축 높이 기반 무지개 색상 (자동)
        //   guik::FlatColor(r, g, b, a)                 — 단색 (0.0~1.0, a=투명도)
        //     예: FlatColor(0.7, 0.7, 0.7, 1.0)        — 회색
        //     예: FlatColor(1.0, 0.0, 0.0, 0.5)        — 반투명 빨강
        //
        // ● 포인트 크기:
        //   .set_point_size(float)                      — 미터 단위 크기 (기본: metric 모드)
        //     예: 0.02f(매우 작음), 0.05f(작음), 0.1f(중간), 0.2f(큼)
        //
        // ● 포인트 모양:
        //   .set_point_shape_circle()                   — 원형 (기본값)
        //   .set_point_shape_rectangle()                — 사각형
        //
        // ● 크기 모드:
        //   .set_point_scale_metric()                   — 미터 단위 (3D 공간 크기, 기본값)
        //   .set_point_scale_screenspace()               — 픽셀 단위 (화면 크기 고정)
        //
        // ● Z축 회색 그라데이션 파라미터 (현재 사용 중):
        //   gray = GRAY_MIN + GRAY_RANGE * t  (t: 0~1 정규화된 높이)
        //   GRAY_MIN: 가장 낮은 포인트의 밝기 (0.0=검정, 1.0=흰색)
        //   GRAY_RANGE: 밝기 변화폭 (GRAY_MIN + GRAY_RANGE = 가장 높은 포인트 밝기)
        // ============================================================
        constexpr float GRAY_MIN = 0.3f;    // 가장 낮은 포인트 밝기 (어두움)
        constexpr float GRAY_RANGE = 0.7f;   // 밝기 변화폭 (0.3~1.0)
        constexpr float POINT_SIZE = 0.1f;  // 일반 포인트 크기 (미터)
        
        // Z축 높이 기반 회색 그라데이션 색상 생성
        auto pcb = std::make_shared<glk::PointCloudBuffer>(frame->points, frame->size());
        float z_min = std::numeric_limits<float>::max();
        float z_max = std::numeric_limits<float>::lowest();

        for (int p = 0; p < frame->size(); p++) 
        {
          float z = frame->points[p].z();
          z_min = std::min(z_min, z);
          z_max = std::max(z_max, z);
        }

        float z_range = (z_max - z_min > 1e-6f) ? (z_max - z_min) : 1.0f;
        std::vector<Eigen::Vector4f> colors(frame->size());

        for (int p = 0; p < frame->size(); p++) 
        {
          float t = (frame->points[p].z() - z_min) / z_range;
          float gray = GRAY_MIN + GRAY_RANGE * t;
          colors[p] = Eigen::Vector4f(gray, gray, gray, 1.0f);
        }

        pcb->add_color(colors);
        viewer->update_drawable(
          "frame_" + std::to_string(i), pcb,
          guik::VertexColor().set_point_size(POINT_SIZE).add("model_matrix", Eigen::Isometry3f(relative_pose.matrix().cast<float>())));
      }
    }

    if (!headless) update_viewer(poses);

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
    // factor_types.push_back("LOAM"); // 4 : LOAM
    factor_types.push_back("NDT"); // 5 : NDT
    factor_types.push_back("LOAM_LIOSAM"); // 6 : LOAM (LIO-SAM feature extraction)

#ifdef GTSAM_POINTS_USE_CUDA
    factor_types.push_back("VGICP_GPU"); // 6 : VG-ICP GPU (if CUDA enabled)
#endif
    full_connection = true;
    num_threads = 1;

    correspondence_update_tolerance_rot = 0.0f;
    correspondence_update_tolerance_trans = 0.0f;

    // UI 콜백 등록
    if (!headless && viewer)
    {
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


        ImGui::Separator();
        ImGui::Checkbox("Show LIO-SAM Features", &show_liosam_features);
        if (show_liosam_features) {
          ImGui::TextColored(ImVec4(1,0,0,1), "Red = Edge");
          ImGui::SameLine();
          ImGui::TextColored(ImVec4(0,0,1,1), "Blue = Planar");
        }

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
    if (headless) return;

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

      // 프레임 연결선 색상: FlatColor(R, G, B, Alpha(투명도)) — 현재 초록색
      viewer->update_drawable("factors", std::make_shared<glk::ThinLines>(factor_lines), guik::FlatColor(0.0f, 1.0f, 0.0f, 1.0f));

      // LIO-SAM feature 시각화 (edge=빨강, planar=파랑)
      if (show_liosam_features) 
      {
        for (int i = 0; i < num_frames; i++) 
        {
          Eigen::Isometry3f pose(values.at<gtsam::Pose3>(i).matrix().cast<float>());

          // 포인트 시각화 설정
          // 색상: FlatColor(R, G, B, Alpha(투명도)) 
          // 크기: set_point_size(float)
          // 모양: set_point_shape_rectangle() 또는 set_point_shape_circle()
          constexpr float EDGE_R = 1.0f, EDGE_G = 0.0f, EDGE_B = 0.0f, EDGE_A = 1.0f;  // 빨강
          constexpr float EDGE_POINT_SIZE = 0.15f;  // Edge 포인트 크기 (미터)
          // EDGE_SHAPE: rectangle(사각형) 또는 circle(원형)
          if (i < (int)loam_features_liosam.size() && loam_features_liosam[i].edge_points && loam_features_liosam[i].edge_points->size() > 0) 
          {
            auto edge_pcb = std::make_shared<glk::PointCloudBuffer>(loam_features_liosam[i].edge_points->points, loam_features_liosam[i].edge_points->size());
            viewer->update_drawable(
              "liosam_edge_" + std::to_string(i), edge_pcb,
              guik::FlatColor(EDGE_R, EDGE_G, EDGE_B, EDGE_A).set_point_size(EDGE_POINT_SIZE).set_point_shape_rectangle().add("model_matrix", pose));
          }

          constexpr float PLANAR_R = 0.0f, PLANAR_G = 0.5f, PLANAR_B = 1.0f, PLANAR_A = 1.0f;  // 하늘색
          constexpr float PLANAR_POINT_SIZE = 0.15f;  // Planar 포인트 크기 (미터)
          // PLANAR_SHAPE: circle(원형) 또는 rectangle(사각형)
          if (i < (int)loam_features_liosam.size() && loam_features_liosam[i].planar_points && loam_features_liosam[i].planar_points->size() > 0) 
          {
            auto planar_pcb = std::make_shared<glk::PointCloudBuffer>(loam_features_liosam[i].planar_points->points, loam_features_liosam[i].planar_points->size());
            viewer->update_drawable(
              "liosam_planar_" + std::to_string(i), planar_pcb,
              guik::FlatColor(PLANAR_R, PLANAR_G, PLANAR_B, PLANAR_A).set_point_size(PLANAR_POINT_SIZE).set_point_shape_circle().add("model_matrix", pose));
          }
        }
      } 
      else 
      {
        // feature 시각화 제거
        for (int i = 0; i < num_frames; i++) 
        {
          viewer->remove_drawable("liosam_edge_" + std::to_string(i));
          viewer->remove_drawable("liosam_planar_" + std::to_string(i));
        }
      }
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
    if (factor_types[factor_type] == std::string("Point-to-Point"))
    {
      auto factor = gtsam::make_shared<gtsam_points::IntegratedICPFactor>(target_key, source_key, target, source);
      factor->set_correspondence_update_tolerance(correspondence_update_tolerance_rot, correspondence_update_tolerance_trans);
      factor->set_num_threads(num_threads);
      return factor;
    }
    // yuchan : Point-to-plane ICP 팩터 생성
    // cost function = Σ (n^T * (p_target - (R * p_source + t)))^2
    else if (factor_types[factor_type] == std::string("Point-to-Plane"))
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
      factor->set_correspondence_update_tolerance(correspondence_update_tolerance_rot, correspondence_update_tolerance_trans);
      factor->set_num_threads(num_threads);
      return factor;
    }
    // yuchan : LOAM 팩터 생성 — 비활성화 (Ring-based feature 추출 비활성화됨)
    // else if (factor_types[factor_type] == std::string("LOAM"))
    // {
    //   auto factor = gtsam::make_shared<gtsam_points::IntegratedLOAMFactor>(
    //     target_key, source_key, 
    //     loam_features[target_key].edge_points, loam_features[target_key].planar_points,
    //     loam_features[source_key].edge_points, loam_features[source_key].planar_points);
    //   factor->set_enable_correspondence_validation(true);
    //   factor->set_max_correspondence_distance(2.0, 2.0);
    //   factor->set_correspondence_update_tolerance(0.005, 0.02);
    //   factor->set_num_threads(num_threads);
    //   return factor;
    // }
    // yuchan : LOAM_LIOSAM 팩터 생성 (LIO-SAM 방식의 특징점 사용)
    else if (factor_types[factor_type] == std::string("LOAM_LIOSAM"))
    {
      auto factor = gtsam::make_shared<gtsam_points::IntegratedLOAMFactor>(
        target_key, source_key, 
        loam_features_liosam[target_key].edge_points, loam_features_liosam[target_key].planar_points,
        loam_features_liosam[source_key].edge_points, loam_features_liosam[source_key].planar_points);
      
      factor->set_enable_correspondence_validation(true);
      factor->set_max_correspondence_distance(2.0, 2.0);
      factor->set_correspondence_update_tolerance(0.005, 0.02);
      factor->set_num_threads(num_threads);
      return factor;
    }
    else if (factor_types[factor_type] == std::string("NDT"))
    {
      auto factor = gtsam::make_shared<gtsam_points::IntegratedNDTFactor_<gtsam_points::PointCloud>>(
        target_key, source_key, target_voxelmap, source);
      factor->set_num_threads(num_threads);
      factor->set_search_mode(gtsam_points::NDTSearchMode::DIRECT7);
      factor->set_outlier_ratio(0.01);  // 0.1→0.01: d2 감소로 gradient 감쇠 더 완화
      factor->set_regularization_epsilon(1e-3);
      factor->set_correspondence_update_tolerance(correspondence_update_tolerance_rot, correspondence_update_tolerance_trans);
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
    auto bench_start = std::chrono::high_resolution_clock::now();

    // optmizer 선택 및 실행(LM or ISAM2)
    if (optimizer_types[optimizer_type] == std::string("LM"))
    {
      gtsam_points::LevenbergMarquardtExtParams lm_params;
      lm_params.maxIterations = 100;
      lm_params.relativeErrorTol = 1e-5;
      lm_params.absoluteErrorTol = 1e-5;
      lm_params.lambdaLowerBound = 1e-6;  // lambda가 0으로 수렴하여 undamped GN으로 퇴화하는 것을 방지
      
      lm_params.callback = [this](const gtsam_points::LevenbergMarquardtOptimizationStatus& status, const gtsam::Values& values)
      {
        spdlog::info("{}", status.to_string());
        if (!headless)
        {
          guik::LightViewer::instance()->append_text(status.to_string());
          update_viewer(values);
        }
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
      if (!headless)
      {
        update_viewer(isam2.calculateEstimate());
        guik::LightViewer::instance()->append_text(status.to_string());
      }

      for (int i = 0; i < 5; i++)
      {
        auto status = isam2.update();
        if (!headless)
        {
          update_viewer(isam2.calculateEstimate());
          guik::LightViewer::instance()->append_text(status.to_string());
        }
      }

      auto t2 = std::chrono::high_resolution_clock::now();
      double msec = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e6;
      spdlog::info("ISAM2 total: {:.3f} ms", msec);
      if (!headless)
      {
        guik::LightViewer::instance()->append_text((boost::format("total:%.3f[msec]") % msec).str());
      }
      
      optimized_values = isam2.calculateEstimate();
    }

    auto bench_end = std::chrono::high_resolution_clock::now();
    last_total_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(bench_end - bench_start).count() / 1e6;

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
    double max_trans_error = 0.0;
    double max_rot_error = 0.0;
    
    for (int i = 0; i < num_frames; i++)
    {
      gtsam::Pose3 opt_pose = optimized_values.at<gtsam::Pose3>(i);
      gtsam::Pose3 gt_pose = poses_gt.at<gtsam::Pose3>(i);
      gtsam::Pose3 error = gt_pose.inverse() * opt_pose;
      double te = error.translation().norm();
      double re = error.rotation().axisAngle().second * 180.0 / M_PI;
      total_trans_error += te;
      total_rot_error += re;
      if (te > max_trans_error) max_trans_error = te;
      if (re > max_rot_error) max_rot_error = re;
    }
    last_mean_trans_error = total_trans_error / num_frames;
    last_mean_rot_error = total_rot_error / num_frames;
    last_max_trans_error = max_trans_error;
    last_max_rot_error = max_rot_error;
    spdlog::info("--- Summary ---");
    spdlog::info("Mean Translation Error: {:.6f} m", last_mean_trans_error);
    spdlog::info("Mean Rotation Error: {:.6f} deg", last_mean_rot_error);
    spdlog::info("Max  Translation Error: {:.6f} m", last_max_trans_error);
    spdlog::info("Max  Rotation Error: {:.6f} deg", last_max_rot_error);
    spdlog::info("Optimization Time: {:.3f} ms", last_total_ms);
    spdlog::info("========================================");
  }

private:
  bool headless;
  bool show_liosam_features = true;  // LIO-SAM feature 시각화 토글
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
  std::vector<LOAMFeatures> loam_features_liosam;
  FeatureExtraction feature_extractor;
  
  // yuchan : 각 프레임의 Ring 정보를 저장
  std::vector<std::vector<PointWithRing>> frames_with_ring;

  double last_mean_trans_error = 0.0;
  double last_mean_rot_error = 0.0;
  double last_max_trans_error = 0.0;
  double last_max_rot_error = 0.0;
  double last_total_ms = 0.0;

public:
  void run_all_factors_headless()   // 시각화 없이 모든 팩터 타입 최적화 실행해서 결과 비교하는 함수 -> ./lidar_registration_demo --headless 옵션으로 실행
  {
    int num_frames = frames.size();

    spdlog::info("\n");
    spdlog::info("╔══════════════════════════════════════════════════════════╗");
    spdlog::info("║       Headless 6-Factor Registration Benchmark           ║");
    spdlog::info("╚══════════════════════════════════════════════════════════╝");

    gtsam::Values poses_noisy;
    poses_noisy.insert(0, poses_gt.at<gtsam::Pose3>(0));
    for (int i = 1; i < num_frames; i++)
    {
      gtsam::Pose3 noise = gtsam::Pose3::Expmap(gtsam::Vector6::Random() * pose_noise_scale);
      poses_noisy.insert(i, poses_gt.at<gtsam::Pose3>(i) * noise);
    }

    spdlog::info("Noise scale: {:.3f}", pose_noise_scale);
    spdlog::info("Frames: {}", num_frames);
    spdlog::info("Optimizer: LM");
    spdlog::info("Full connection: {}", full_connection ? "true" : "false");
    spdlog::info("");

    struct Result {
      std::string name;
      double mean_t, mean_r, max_t, max_r, ms;
    };
    std::vector<Result> results;

    int num_factors = static_cast<int>(factor_types.size());
    #ifdef GTSAM_POINTS_USE_CUDA
    num_factors = std::min(num_factors, 6);
    #endif

    for (int fi = 0; fi < num_factors; fi++)
    {
      // LOAM (Ring-based)은 비활성화 상태이므로 스킵
      if (std::string(factor_types[fi]) == "LOAM") {
        spdlog::info("Skipping {} (disabled)", factor_types[fi]);
        continue;
      }

      factor_type = fi;
      spdlog::info("────────────────────────────────────────");
      spdlog::info("Running: {}", factor_types[fi]);
      spdlog::info("────────────────────────────────────────");

      poses = gtsam::Values();
      for (int i = 0; i < num_frames; i++)
      {
        poses.insert(i, poses_noisy.at<gtsam::Pose3>(i));
      }

      optimizer_type = 0;
      run_optimization();

      results.push_back({factor_types[fi], last_mean_trans_error, last_mean_rot_error,
                          last_max_trans_error, last_max_rot_error, last_total_ms});
    }

    spdlog::info("\n");
    spdlog::info("╔══════════════════════════════════════════════════════════════════════════════════════╗");
    spdlog::info("║                        Final Comparison Table                                        ║");
    spdlog::info("╠════════════════╦═══════════════╦═══════════════╦═══════════════╦═══════════════╦═════╣");
    spdlog::info("║ Factor         ║ Mean T (m)    ║ Mean R (deg)  ║ Max T (m)     ║ Max R (deg)   ║ ms  ║");
    spdlog::info("╠════════════════╬═══════════════╬═══════════════╬═══════════════╬═══════════════╬═════╣");
    for (const auto& r : results)
    {
      spdlog::info("║ {:14s} ║ {:13.6f} ║ {:13.6f} ║ {:13.6f} ║ {:13.6f} ║{:5.0f}║",
                   r.name, r.mean_t, r.mean_r, r.max_t, r.max_r, r.ms);
    }
    spdlog::info("╚════════════════╩═══════════════╩═══════════════╩═══════════════╩═══════════════╩═════╝");
  }
};


int main(int argc, char** argv)
{
  bool headless_mode = false;
  for (int i = 1; i < argc; i++)
  {
    if (std::strcmp(argv[i], "--headless") == 0) headless_mode = true;
  }

  MatchingCostFactorDemo demo(headless_mode);

  if (headless_mode)
  {
    demo.run_all_factors_headless();
  }
  else
  {
    guik::LightViewer::instance()->spin();
  }
  return 0;
}