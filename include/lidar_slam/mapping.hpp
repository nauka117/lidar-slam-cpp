#pragma once

#include <filesystem>

#include <Eigen/Core>

#include "lidar_slam/kitti_loader.hpp"

namespace lidar_slam {

struct MapConfig {
  float voxel_leaf = 0.5f;
  int voxel_every_n_frames = 10;
};

class MapAccumulator {
 public:
  explicit MapAccumulator(MapConfig config = {});

  void insert(const PointCloudPtr& cloud, const Eigen::Matrix4f& world_pose);
  void finalize();

  const PointCloudPtr& map() const { return global_map_; }
  std::size_t pointCount() const { return global_map_ ? global_map_->size() : 0; }

  void savePcd(const std::filesystem::path& path) const;

 private:
  void voxelDownsample();

  MapConfig config_;
  PointCloudPtr global_map_;
  int frames_since_voxel_ = 0;
};

}  // namespace lidar_slam
