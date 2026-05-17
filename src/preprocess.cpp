#include "lidar_slam/preprocess.hpp"
#include <chrono>
#include <stdexcept>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

namespace lidar_slam {

PreprocessPipeline::PreprocessPipeline(PreprocessConfig config) : config_(std::move(config)) {}

ProcessResult PreprocessPipeline::run(const PointCloudPtr& input) const {
  ProcessResult result;
  if (!input || input->empty()) {
    throw std::runtime_error("Input cloud is empty");
  }

  result.points_before = input->size();
  const auto t0 = std::chrono::steady_clock::now();

  PointCloudPtr current(new PointCloud(*input));

  if (config_.passthrough_enabled) {
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(current);
    pass.setFilterFieldName(config_.passthrough_field);
    pass.setFilterLimits(config_.passthrough_min, config_.passthrough_max);
    PointCloudPtr filtered(new PointCloud);
    pass.filter(*filtered);
    current = filtered;
  }

  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setInputCloud(current);
  voxel.setLeafSize(config_.voxel_leaf, config_.voxel_leaf, config_.voxel_leaf);
  PointCloudPtr downsampled(new PointCloud);
  voxel.filter(*downsampled);
  current = downsampled;

  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud(current);
  sor.setMeanK(config_.sor_mean_k);
  sor.setStddevMulThresh(config_.sor_stddev_mul);
  PointCloudPtr cleaned(new PointCloud);
  sor.filter(*cleaned);

  const auto t1 = std::chrono::steady_clock::now();
  result.cloud = cleaned;
  result.points_after = cleaned->size();
  result.time_ms =
      std::chrono::duration<double, std::milli>(t1 - t0).count();

  return result;
}

}  // namespace lidar_slam
