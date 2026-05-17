#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_slam {

using PointCloud = pcl::PointCloud<pcl::PointXYZI>;
using PointCloudPtr = PointCloud::Ptr;

class KittiLoader {
 public:
  static constexpr std::size_t kBytesPerPoint = 16;

  static PointCloudPtr loadBin(const std::filesystem::path& path);
  static PointCloudPtr loadFrame(const std::filesystem::path& sequence_dir, int frame_id);

  static std::filesystem::path framePath(const std::filesystem::path& sequence_dir, int frame_id);
  static int countFrames(const std::filesystem::path& sequence_dir);
};

}  // namespace lidar_slam
