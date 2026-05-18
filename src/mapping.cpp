#include "lidar_slam/mapping.hpp"

#include <stdexcept>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

namespace lidar_slam {

MapAccumulator::MapAccumulator(MapConfig config) : config_(std::move(config)) {
  global_map_.reset(new PointCloud);
}

void MapAccumulator::insert(const PointCloudPtr& cloud, const Eigen::Matrix4f& world_pose) {
  if (!cloud || cloud->empty()) {
    return;
  }

  PointCloudPtr transformed(new PointCloud);
  pcl::transformPointCloud(*cloud, *transformed, world_pose);

  *global_map_ += *transformed;
  global_map_->width = static_cast<std::uint32_t>(global_map_->size());
  global_map_->height = 1;

  ++frames_since_voxel_;
  if (config_.voxel_every_n_frames > 0 &&
      frames_since_voxel_ >= config_.voxel_every_n_frames) {
    voxelDownsample();
    frames_since_voxel_ = 0;
  }
}

void MapAccumulator::voxelDownsample() {
  if (!global_map_ || global_map_->empty()) {
    return;
  }

  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setInputCloud(global_map_);
  voxel.setLeafSize(config_.voxel_leaf, config_.voxel_leaf, config_.voxel_leaf);
  PointCloudPtr filtered(new PointCloud);
  voxel.filter(*filtered);
  global_map_ = filtered;
}

void MapAccumulator::finalize() {
  voxelDownsample();
  frames_since_voxel_ = 0;
}

void MapAccumulator::savePcd(const std::filesystem::path& path) const {
  if (!global_map_ || global_map_->empty()) {
    throw std::runtime_error("Cannot save empty map");
  }
  std::filesystem::create_directories(path.parent_path());
  if (pcl::io::savePCDFileBinary(path.string(), *global_map_) != 0) {
    throw std::runtime_error("Failed to write map: " + path.string());
  }
}

}  // namespace lidar_slam
