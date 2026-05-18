#include "lidar_slam/slam_runner.hpp"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace lidar_slam {

SlamRunner::SlamRunner(SlamConfig config) : config_(std::move(config)) {}

SlamResult SlamRunner::run(const std::filesystem::path& sequence_dir) {
  const int available = KittiLoader::countFrames(sequence_dir);
  if (config_.end_frame >= available) {
    std::cerr << "Warning: end_frame " << config_.end_frame << " >= available " << available
              << ", clamping to " << available - 1 << '\n';
    config_.end_frame = available - 1;
  }
  if (config_.start_frame < 0 || config_.start_frame > config_.end_frame) {
    throw std::runtime_error("Invalid frame range");
  }

  SlamResult result;
  PreprocessPipeline preprocess(config_.preprocess);
  IcpOdometry odometry(config_.icp);
  MapAccumulator map(config_.map);

  auto loadAndPreprocess = [&](int frame_id, FrameStats& stats) -> PointCloudPtr {
    const auto t0 = std::chrono::steady_clock::now();
    auto raw = KittiLoader::loadFrame(sequence_dir, frame_id);
    auto proc = preprocess.run(raw);
    const auto t1 = std::chrono::steady_clock::now();
    stats.preprocess_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    return proc.cloud;
  };

  FrameStats stats0;
  stats0.frame = config_.start_frame;
  auto current = loadAndPreprocess(config_.start_frame, stats0);

  result.poses.push_back(Eigen::Matrix4f::Identity());
  result.frame_ids.push_back(config_.start_frame);
  map.insert(current, result.poses.back());
  stats0.map_points = map.pointCount();
  stats0.icp_ran = false;
  result.stats.push_back(stats0);

  PointCloudPtr previous = current;

  for (int frame = config_.start_frame + 1; frame <= config_.end_frame; ++frame) {
    FrameStats stats;
    stats.frame = frame;
    current = loadAndPreprocess(frame, stats);

    const auto t0 = std::chrono::steady_clock::now();
    const auto icp_result = odometry.estimate(current, previous);
    const auto t1 = std::chrono::steady_clock::now();
    stats.icp_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    stats.icp_ran = true;
    stats.icp_converged = icp_result.converged;
    stats.icp_fitness = icp_result.fitness;

    if (!icp_result.converged) {
      std::cerr << "Warning: ICP did not converge at frame " << frame << '\n';
      if (config_.strict_icp) {
        throw std::runtime_error("ICP failed in strict mode at frame " + std::to_string(frame));
      }
    }

    const Eigen::Matrix4f T_new = result.poses.back() * icp_result.transform;
    result.poses.push_back(T_new);
    result.frame_ids.push_back(frame);

    map.insert(current, T_new);
    stats.map_points = map.pointCount();
    result.stats.push_back(stats);

    previous = current;
  }

  map.finalize();
  result.global_map = map.map();

  return result;
}

void SlamRunner::savePoses(const std::filesystem::path& path,
                           const std::vector<int>& frame_ids,
                           const std::vector<Eigen::Matrix4f>& poses) {
  if (frame_ids.size() != poses.size()) {
    throw std::runtime_error("frame_ids and poses size mismatch");
  }

  std::filesystem::create_directories(path.parent_path());
  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("Cannot open poses file: " + path.string());
  }

  out << "# frame tx ty tz r00 r01 r02 r10 r11 r12 r20 r21 r22\n";
  out << std::fixed << std::setprecision(6);

  for (std::size_t i = 0; i < poses.size(); ++i) {
    const auto& T = poses[i];
    out << frame_ids[i] << ' ' << T(0, 3) << ' ' << T(1, 3) << ' ' << T(2, 3);
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        out << ' ' << T(row, col);
      }
    }
    out << '\n';
  }
}

}  // namespace lidar_slam
