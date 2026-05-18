#pragma once

#include <filesystem>
#include <vector>

#include <Eigen/Core>

#include "lidar_slam/kitti_loader.hpp"
#include "lidar_slam/mapping.hpp"
#include "lidar_slam/odometry.hpp"
#include "lidar_slam/preprocess.hpp"

namespace lidar_slam {

struct SlamConfig {
  PreprocessConfig preprocess;
  IcpConfig icp;
  MapConfig map;

  int start_frame = 0;
  int end_frame = 100;
  bool strict_icp = false;
};

struct FrameStats {
  int frame = 0;
  bool icp_ran = false;
  bool icp_converged = false;
  double icp_fitness = 0.0;
  std::size_t map_points = 0;
  double preprocess_ms = 0.0;
  double icp_ms = 0.0;
};

struct SlamResult {
  std::vector<Eigen::Matrix4f> poses;
  std::vector<int> frame_ids;
  PointCloudPtr global_map;
  std::vector<FrameStats> stats;
};

class SlamRunner {
 public:
  explicit SlamRunner(SlamConfig config = {});

  SlamResult run(const std::filesystem::path& sequence_dir);

  static void savePoses(const std::filesystem::path& path,
                        const std::vector<int>& frame_ids,
                        const std::vector<Eigen::Matrix4f>& poses);

 private:
  SlamConfig config_;
};

}  // namespace lidar_slam
