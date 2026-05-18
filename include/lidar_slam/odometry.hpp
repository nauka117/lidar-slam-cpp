#pragma once

#include <Eigen/Core>

#include "lidar_slam/kitti_loader.hpp"

namespace lidar_slam {

struct IcpConfig {
  double max_correspondence_distance = 1.0;
  int max_iterations = 50;
  double transformation_epsilon = 1e-8;
  double euclidean_fitness_epsilon = 1e-6;
};

struct IcpResult {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  bool converged = false;
  double fitness = 0.0;
};

class IcpOdometry {
 public:
  explicit IcpOdometry(IcpConfig config = {});

  IcpResult estimate(const PointCloudPtr& source, const PointCloudPtr& target,
                     const Eigen::Matrix4f& initial_guess = Eigen::Matrix4f::Identity()) const;

  const IcpConfig& config() const { return config_; }

 private:
  IcpConfig config_;
};

}  // namespace lidar_slam
