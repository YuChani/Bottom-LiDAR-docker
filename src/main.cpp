#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>

#include <gtsam_points/factors/integrated_gicp_factor.hpp>
#include <gtsam_points/factors/integrated_icp_factor.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam_points/features/covariance_estimation.hpp>
#include <gtsam_points/features/normal_estimation.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

static std::vector<Eigen::Vector4d> read_pcd_xyz(const std::string& path) 
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) == -1) 
  {
    return {};
  }

  std::vector<Eigen::Vector4d> points;
  points.reserve(cloud->size());
  for (const auto& p : cloud->points) 
  {
    points.emplace_back(p.x, p.y, p.z, 1.0);
  }
  return points;
}

static void print_pose_rt(const gtsam::Pose3& T) 
{
  const auto R = T.rotation().matrix();
  const auto t = T.translation();

  std::cout << "R:\n" << R << "\n";
  std::cout << "t: [" << t.x() << ", " << t.y() << ", " << t.z() << "]\n";
}

int main(int argc, char** argv) 
{
  // 기본값 설정
  std::string target_pcd = "/root/workdir/data/my_dataset/1710406871.842252000.pcd";
  std::string source_pcd = "/root/workdir/data/my_dataset/1710406880.032087000.pcd";
  std::string method = "vgicp";

  // 인자가 주어진 경우 사용
  if (argc >= 2) 
  {
    target_pcd = argv[1];
  }
  if (argc >= 3) 
  {
    source_pcd = argv[2];
  }
  if (argc >= 4) 
  {
    method = argv[3];
  }

  std::cerr << "Loading target: " << target_pcd << "\n";
  std::cerr << "Loading source: " << source_pcd << "\n";
  std::cerr << "Method: " << method << "\n";

  auto target_pts = read_pcd_xyz(target_pcd);
  auto source_pts = read_pcd_xyz(source_pcd);

  if (target_pts.empty() || source_pts.empty()) 
  {
    std::cerr << "[Error] Failed to load PCD(s) (empty point set)." << std::endl;
    return 1;
  }

  auto target = std::make_shared<gtsam_points::PointCloudCPU>();
  target->add_points(target_pts);
  target->add_covs(gtsam_points::estimate_covariances(target_pts));
  target->add_normals(gtsam_points::estimate_normals(target_pts));

  auto source = std::make_shared<gtsam_points::PointCloudCPU>();
  source->add_points(source_pts);
  source->add_covs(gtsam_points::estimate_covariances(source_pts));
  source->add_normals(gtsam_points::estimate_normals(source_pts));

  auto target_voxelmap = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(1.0);
  target_voxelmap->insert(*target);

  const gtsam::Key k0 = 0;
  const gtsam::Key k1 = 1;

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values init;
  init.insert(k0, gtsam::Pose3());
  init.insert(k1, gtsam::Pose3());

  graph.add(gtsam::PriorFactor<gtsam::Pose3>(k0, gtsam::Pose3(), gtsam::noiseModel::Isotropic::Precision(6, 1e6)));

  gtsam::NonlinearFactor::shared_ptr factor;
  if (method == "icp") 
  {
    auto f = gtsam::make_shared<gtsam_points::IntegratedICPFactor>(k0, k1, target, source);
    f->set_num_threads(4);
    factor = f;
  } 
  else if (method == "gicp") 
  {
    auto f = gtsam::make_shared<gtsam_points::IntegratedGICPFactor>(k0, k1, target, source);
    f->set_num_threads(4);
    factor = f;
  } 
  else if (method == "vgicp") 
  {
    auto f = gtsam::make_shared<gtsam_points::IntegratedVGICPFactor>(k0, k1, target_voxelmap, source);
    f->set_num_threads(4);
    factor = f;
  } 
  else 
  {
    std::cerr << "[Error] Unknown method: " << method << std::endl;
    return 2;
  }
  graph.add(factor);

  gtsam_points::LevenbergMarquardtExtParams params;
  gtsam_points::LevenbergMarquardtOptimizerExt optimizer(graph, init, params);
  auto result = optimizer.optimize();

  const auto T01 = result.at<gtsam::Pose3>(k0).between(result.at<gtsam::Pose3>(k1));
  std::cout << "T_target_source (Pose3):\n" << T01 << "\n";
  print_pose_rt(T01);
  return 0;
}
