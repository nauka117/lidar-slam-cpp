#include "lidar_slam/kitti_loader.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>

namespace lidar_slam {

namespace {

void checkFileSize(std::size_t byte_count) {
  if (byte_count == 0) {
    throw std::runtime_error("KITTI .bin file is empty");
  }
  if (byte_count % KittiLoader::kBytesPerPoint != 0) {
    std::ostringstream oss;
    oss << "KITTI .bin size " << byte_count
        << " is not a multiple of " << KittiLoader::kBytesPerPoint;
    throw std::runtime_error(oss.str());
  }
}

}  // namespace

PointCloudPtr KittiLoader::loadBin(const std::filesystem::path& path) {
  if (!std::filesystem::is_regular_file(path)) {
    throw std::runtime_error("File not found: " + path.string());
  }

  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file) {
    throw std::runtime_error("Cannot open: " + path.string());
  }

  const auto byte_count = static_cast<std::size_t>(file.tellg());
  checkFileSize(byte_count);
  file.seekg(0);

  const std::size_t point_count = byte_count / kBytesPerPoint;
  std::vector<float> buffer(point_count * 4);
  file.read(reinterpret_cast<char*>(buffer.data()),
            static_cast<std::streamsize>(byte_count));
  if (!file) {
    throw std::runtime_error("Read failed: " + path.string());
  }

  PointCloudPtr cloud(new PointCloud);
  cloud->resize(point_count);
  cloud->width = static_cast<std::uint32_t>(point_count);
  cloud->height = 1;
  cloud->is_dense = true;

  for (std::size_t i = 0; i < point_count; ++i) {
    auto& p = cloud->points[i];
    p.x = buffer[4 * i + 0];
    p.y = buffer[4 * i + 1];
    p.z = buffer[4 * i + 2];
    p.intensity = buffer[4 * i + 3];
  }

  return cloud;
}

std::filesystem::path KittiLoader::framePath(const std::filesystem::path& sequence_dir,
                                             int frame_id) {
  std::ostringstream name;
  name << std::setw(6) << std::setfill('0') << frame_id << ".bin";
  return sequence_dir / "velodyne" / name.str();
}

PointCloudPtr KittiLoader::loadFrame(const std::filesystem::path& sequence_dir,
                                     int frame_id) {
  return loadBin(framePath(sequence_dir, frame_id));
}

int KittiLoader::countFrames(const std::filesystem::path& sequence_dir) {
  const auto velodyne = sequence_dir / "velodyne";
  if (!std::filesystem::is_directory(velodyne)) {
    throw std::runtime_error("Missing velodyne directory: " + velodyne.string());
  }

  int count = 0;
  for (const auto& entry : std::filesystem::directory_iterator(velodyne)) {
    if (entry.is_regular_file() && entry.path().extension() == ".bin") {
      ++count;
    }
  }
  return count;
}

}  // namespace lidar_slam
