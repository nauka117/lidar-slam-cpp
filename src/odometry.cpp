#include "lidar_slam/odometry.hpp"

#include <stdexcept>

#include <pcl/registration/icp.h>

namespace lidar_slam {

IcpOdometry::IcpOdometry(IcpConfig config) : config_(std::move(config)) {}

IcpResult IcpOdometry::estimate(const PointCloudPtr& source, const PointCloudPtr& target,
                                const Eigen::Matrix4f& initial_guess) const {
  if (!source || !target || source->empty() || target->empty()) {
    throw std::runtime_error("ICP: empty source or target cloud");
  }

  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setInputSource(source);
  icp.setInputTarget(target);
  icp.setMaxCorrespondenceDistance(config_.max_correspondence_distance);
  icp.setMaximumIterations(config_.max_iterations);
  icp.setTransformationEpsilon(config_.transformation_epsilon);
  icp.setEuclideanFitnessEpsilon(config_.euclidean_fitness_epsilon);

  PointCloud aligned;
  icp.align(aligned, initial_guess);

  IcpResult result;
  result.converged = icp.hasConverged();
  result.fitness = icp.getFitnessScore();
  result.transform = icp.getFinalTransformation();

  return result;
}

}  // namespace lidar_slam
