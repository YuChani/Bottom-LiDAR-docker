#include "scan_to_map.hpp"

#include <chrono>
#include <spdlog/spdlog.h>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/features/covariance_estimation.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>

ScanToMapRegistration::ScanToMapRegistration(double voxel_resolution)
  : voxel_resolution_(voxel_resolution),
    num_threads_(4),
    max_iterations_(20),
    tolerance_(1e-5),
    global_map_(nullptr)
{
  spdlog::info("[Scan-to-Map] 초기화: 복셀 해상도 = {:.2f}m", voxel_resolution_);
}

ScanToMapRegistration::~ScanToMapRegistration()
{
}

void ScanToMapRegistration::set_num_threads(int num_threads)
{
  num_threads_ = num_threads;
  spdlog::info("[Scan-to-Map] OpenMP 스레드 수 설정: {}", num_threads_);
}

void ScanToMapRegistration::set_max_iterations(int max_iter)
{
  max_iterations_ = max_iter;
  spdlog::info("[Scan-to-Map] 최대 반복 횟수 설정: {}", max_iterations_);
}

void ScanToMapRegistration::set_convergence_tolerance(double tolerance)
{
  tolerance_ = tolerance;
  spdlog::info("[Scan-to-Map] 수렴 허용 오차 설정: {:.2e}", tolerance_);
}

std::shared_ptr<gtsam_points::GaussianVoxelMapCPU> ScanToMapRegistration::get_global_map() const
{
  return global_map_;
}

ScanToMapResult ScanToMapRegistration::register_frames(
  const std::vector<gtsam_points::PointCloud::Ptr>& frames,
  const gtsam::Values& initial_poses)
{
  ScanToMapResult result;
  result.total_frames = frames.size();
  result.successful_frames = 0;
  result.optimization_times.reserve(frames.size());
  result.iterations.reserve(frames.size());
  result.final_errors.reserve(frames.size());
  
  if (frames.empty())
  {
    spdlog::error("[Scan-to-Map] 프레임이 비어있습니다");
    return result;
  }
  
  spdlog::info("[Scan-to-Map] 총 {} 프레임 정합 시작", frames.size());
  spdlog::info("[Scan-to-Map] 파라미터: 스레드={}, 최대반복={}, 허용오차={:.2e}", 
               num_threads_, max_iterations_, tolerance_);
  
  global_map_ = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(voxel_resolution_);
  
  for (size_t i = 0; i < frames.size(); i++)
  {
    auto frame = frames[i];
    if (!frame || frame->size() == 0)
    {
      spdlog::warn("[Scan-to-Map] 프레임 {}: 포인트 없음, 건너뜀", i);
      result.optimization_times.push_back(0.0);
      result.iterations.push_back(0);
      result.final_errors.push_back(-1.0);
      continue;
    }
    
    spdlog::info("[Scan-to-Map] === 프레임 {} / {} ===", i, frames.size() - 1);
    spdlog::info("[Scan-to-Map]   포인트 수: {}", frame->size());
    
    auto t1 = std::chrono::high_resolution_clock::now();
    
    gtsam::Pose3 estimated_pose;
    int final_iterations = 0;
    double final_error = 0.0;
    
    if (i == 0)
    {
      estimated_pose = gtsam::Pose3::Identity();
      result.poses.insert(i, estimated_pose);
      spdlog::info("[Scan-to-Map]   원점으로 초기화");
      
      global_map_->insert(*frame);
      spdlog::info("[Scan-to-Map]   글로벌 맵 초기화 완료");
      
      result.successful_frames++;
      result.optimization_times.push_back(0.0);
      result.iterations.push_back(0);
      result.final_errors.push_back(0.0);
    }
    else
    {
      gtsam::Pose3 initial_guess = initial_poses.at<gtsam::Pose3>(i);
      auto init_t = initial_guess.translation();
      spdlog::info("[Scan-to-Map]   초기 추정: [{:.3f}, {:.3f}, {:.3f}]", 
                   init_t.x(), init_t.y(), init_t.z());
      
      gtsam::NonlinearFactorGraph graph;
      gtsam::Values initial;
      initial.insert(i, initial_guess);
      
      graph.add(gtsam::PriorFactor<gtsam::Pose3>(
        i, initial_guess,
        gtsam::noiseModel::Isotropic::Precision(6, 1e3)));
      
      auto factor = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(
        gtsam::Pose3::Identity(),
        i,
        global_map_,
        frame
      );
      factor->set_num_threads(num_threads_);
      graph.add(factor);
      
      gtsam_points::LevenbergMarquardtExtParams lm_params;
      lm_params.maxIterations = max_iterations_;
      lm_params.relativeErrorTol = tolerance_;
      lm_params.absoluteErrorTol = tolerance_;
      lm_params.setVerbosityLM("SILENT");
      
      gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, initial, lm_params);
      gtsam::Values optimized = optimizer.optimize();
      
      estimated_pose = optimized.at<gtsam::Pose3>(i);
      result.poses.insert(i, estimated_pose);
      
      final_iterations = optimizer.iterations();
      final_error = optimizer.error();
      
      auto opt_t = estimated_pose.translation();
      spdlog::info("[Scan-to-Map]   최적화 완료: [{:.3f}, {:.3f}, {:.3f}]", 
                   opt_t.x(), opt_t.y(), opt_t.z());
      spdlog::info("[Scan-to-Map]   반복 횟수: {}, 최종 에러: {:.6f}", 
                   final_iterations, final_error);
      
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
      
      global_map_->insert(*transformed_frame);
      spdlog::info("[Scan-to-Map]   글로벌 맵 업데이트 완료");
      
      result.successful_frames++;
    }
    
    auto t2 = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() / 1000.0;
    
    result.optimization_times.push_back(elapsed);
    result.iterations.push_back(final_iterations);
    result.final_errors.push_back(final_error);
    
    spdlog::info("[Scan-to-Map]   소요 시간: {:.3f}초", elapsed);
  }
  
  spdlog::info("[Scan-to-Map] ========================================");
  spdlog::info("[Scan-to-Map] 정합 완료: {}/{} 프레임 성공", 
               result.successful_frames, result.total_frames);
  
  double total_time = 0.0;
  for (double t : result.optimization_times) total_time += t;
  spdlog::info("[Scan-to-Map] 총 소요 시간: {:.3f}초 (평균 {:.3f}초/프레임)", 
               total_time, total_time / frames.size());
  spdlog::info("[Scan-to-Map] ========================================");
  
  return result;
}
