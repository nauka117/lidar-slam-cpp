#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <stdexcept>

#include <pcl/io/pcd_io.h>

#include "lidar_slam/kitti_loader.hpp"
#include "lidar_slam/preprocess.hpp"

#if defined(LIDAR_SLAM_HAS_VISUALIZATION)
#include <pcl/visualization/pcl_visualizer.h>
#endif

namespace {

struct Options {
  std::filesystem::path sequence;
  int start = 0;
  int end = 50;
  float voxel = 0.3f;
  std::filesystem::path output_dir = "output/processed";
  bool save_pcd = false;
  bool visualize = false;
  int visualize_frame = -1;
  bool passthrough = false;
};

void printUsage(const char* prog) {
  std::cerr
      << "Usage: " << prog << " --sequence <path/to/sequences/00> [options]\n"
      << "  --sequence PATH   KITTI sequence directory (contains velodyne/)\n"
      << "  --start N         First frame index (default: 0)\n"
      << "  --end N           Last frame index inclusive (default: 50)\n"
      << "  --voxel SIZE      VoxelGrid leaf size in meters (default: 0.3)\n"
      << "  --out DIR         Output directory for PCD (default: output/processed)\n"
      << "  --save-pcd        Write processed clouds to --out\n"
      << "  --visualize       Show one frame in PCL viewer (host with display)\n"
      << "  --frame N         Frame to visualize (default: --end)\n"
      << "  --passthrough     Enable z passthrough filter before voxel\n"
      << "  -h, --help        Show this help\n";
}

std::optional<Options> parseArgs(int argc, char** argv) {
  Options opt;
  if (argc < 2) {
    return std::nullopt;
  }

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto needValue = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error(std::string("Missing value for ") + name);
      }
      return argv[++i];
    };

    if (arg == "-h" || arg == "--help") {
      printUsage(argv[0]);
      std::exit(0);
    } else if (arg == "--sequence") {
      opt.sequence = needValue("--sequence");
    } else if (arg == "--start") {
      opt.start = std::stoi(needValue("--start"));
    } else if (arg == "--end") {
      opt.end = std::stoi(needValue("--end"));
    } else if (arg == "--voxel") {
      opt.voxel = std::stof(needValue("--voxel"));
    } else if (arg == "--out") {
      opt.output_dir = needValue("--out");
    } else if (arg == "--save-pcd") {
      opt.save_pcd = true;
    } else if (arg == "--visualize") {
      opt.visualize = true;
    } else if (arg == "--frame") {
      opt.visualize_frame = std::stoi(needValue("--frame"));
    } else if (arg == "--passthrough") {
      opt.passthrough = true;
    } else {
      throw std::runtime_error("Unknown argument: " + arg);
    }
  }

  if (opt.sequence.empty()) {
    return std::nullopt;
  }
  if (opt.start < 0 || opt.end < opt.start) {
    throw std::runtime_error("Invalid frame range");
  }
  return opt;
}

#if defined(LIDAR_SLAM_HAS_VISUALIZATION)
void showCloud(const lidar_slam::PointCloudPtr& raw,
               const lidar_slam::PointCloudPtr& processed, int frame_id) {
  pcl::visualization::PCLVisualizer viewer("kitti_preprocess");
  viewer.setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> raw_color(raw, 180,
                                                                              180, 180);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> proc_color(processed, 0,
                                                                              255, 0);
  viewer.addPointCloud<pcl::PointXYZI>(raw, raw_color, "raw");
  viewer.addPointCloud<pcl::PointXYZI>(processed, proc_color, "processed");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
                                          "raw");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
                                          "processed");
  viewer.addText("Frame " + std::to_string(frame_id) + "  gray=raw  green=processed", 10, 10,
                 "label");
  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
  }
}
#endif

}  // namespace

int main(int argc, char** argv) {
  try {
    const auto parsed = parseArgs(argc, argv);
    if (!parsed) {
      printUsage(argv[0]);
      return 1;
    }
  Options opt = *parsed;

  if (!std::filesystem::is_directory(opt.sequence)) {
    std::cerr << "Sequence directory not found: " << opt.sequence << '\n';
    return 1;
  }

  const int available = lidar_slam::KittiLoader::countFrames(opt.sequence);
  if (opt.end >= available) {
    std::cerr << "Warning: --end " << opt.end << " >= frame count " << available
              << "; clamping to " << available - 1 << '\n';
    opt.end = available - 1;
  }

  lidar_slam::PreprocessConfig cfg;
  cfg.voxel_leaf = opt.voxel;
  cfg.passthrough_enabled = opt.passthrough;
  const lidar_slam::PreprocessPipeline pipeline(cfg);

  if (opt.save_pcd) {
    std::filesystem::create_directories(opt.output_dir);
  }

  std::cout << "frame  points_before  points_after  time_ms\n";

  lidar_slam::PointCloudPtr vis_raw;
  lidar_slam::PointCloudPtr vis_proc;
  int vis_id = opt.visualize_frame >= 0 ? opt.visualize_frame : opt.end;

  for (int frame = opt.start; frame <= opt.end; ++frame) {
    const auto raw = lidar_slam::KittiLoader::loadFrame(opt.sequence, frame);
    const auto result = pipeline.run(raw);

    std::cout << std::setw(5) << frame << ' ' << std::setw(14) << result.points_before << ' '
              << std::setw(13) << result.points_after << ' ' << std::setw(7)
              << std::fixed << std::setprecision(2) << result.time_ms << '\n';

    if (opt.save_pcd) {
      std::ostringstream name;
      name << std::setw(6) << std::setfill('0') << frame << ".pcd";
      const auto path = opt.output_dir / name.str();
      pcl::io::savePCDFileBinary(path.string(), *result.cloud);
    }

    if (opt.visualize && frame == vis_id) {
      vis_raw = raw;
      vis_proc = result.cloud;
    }
  }

  if (opt.visualize) {
#if defined(LIDAR_SLAM_HAS_VISUALIZATION)
    if (!vis_raw || !vis_proc) {
      std::cerr << "Visualization frame not in processed range.\n";
      return 1;
    }
    showCloud(vis_raw, vis_proc, vis_id);
#else
    std::cerr << "Built without PCL visualization support.\n";
    return 1;
#endif
  }

  return 0;
  } catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << '\n';
    return 1;
  }
}
