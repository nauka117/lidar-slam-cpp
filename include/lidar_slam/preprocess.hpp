#pragma once

#include "lidar_slam/kitti_loader.hpp"

namespace lidar_slam {

struct PreprocessConfig {
  bool passthrough_enabled = false;
  std::string passthrough_field = "z";
  float passthrough_min = -2.5f;
  float passthrough_max = 2.5f;

  float voxel_leaf = 0.3f;

  int sor_mean_k = 20;
  double sor_stddev_mul = 1.0;
};

struct ProcessResult {
  PointCloudPtr cloud;
  std::size_t points_before = 0;
  std::size_t points_after = 0;
  double time_ms = 0.0;
};

class PreprocessPipeline {
 public:
  explicit PreprocessPipeline(PreprocessConfig config = {});

  ProcessResult run(const PointCloudPtr& input) const;

  const PreprocessConfig& config() const { return config_; }

 private:
  PreprocessConfig config_;
};

}  // namespace lidar_slam
