#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <stdexcept>

#include <pcl/io/pcd_io.h>

#include "lidar_slam/slam_runner.hpp"

#if defined(LIDAR_SLAM_HAS_VISUALIZATION)
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#endif

namespace {

struct Options {
  std::filesystem::path sequence;
  int start = 0;
  int end = 100;
  float voxel = 0.3f;
  double icp_max_dist = 1.0;
  int icp_iter = 50;
  float map_voxel = 0.5f;
  int map_voxel_every = 10;
  std::filesystem::path output_dir = "output/slam";
  bool save_poses = true;
  bool save_map = true;
  bool visualize = false;
  bool strict_icp = false;
};

void printUsage(const char* prog) {
  std::cerr << "Usage: " << prog << " --sequence <path/to/sequences/00> [options]\n"
            << "  --sequence PATH      KITTI sequence directory\n"
            << "  --start N            First frame (default: 0)\n"
            << "  --end N              Last frame inclusive (default: 100)\n"
            << "  --voxel SIZE         Preprocess voxel leaf m (default: 0.3)\n"
            << "  --icp-max-dist D     ICP max correspondence m (default: 1.0)\n"
            << "  --icp-iter N         ICP max iterations (default: 50)\n"
            << "  --map-voxel SIZE     Global map voxel m (default: 0.5)\n"
            << "  --map-voxel-every N  Voxelize map every N frames (default: 10)\n"
            << "  --out DIR            Output directory (default: output/slam)\n"
            << "  --no-save-poses      Skip poses.txt\n"
            << "  --no-save-map        Skip global_map.pcd\n"
            << "  --visualize          Show map and trajectory\n"
            << "  --strict             Abort if ICP fails to converge\n"
            << "  -h, --help           Show help\n";
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
    } else if (arg == "--icp-max-dist") {
      opt.icp_max_dist = std::stod(needValue("--icp-max-dist"));
    } else if (arg == "--icp-iter") {
      opt.icp_iter = std::stoi(needValue("--icp-iter"));
    } else if (arg == "--map-voxel") {
      opt.map_voxel = std::stof(needValue("--map-voxel"));
    } else if (arg == "--map-voxel-every") {
      opt.map_voxel_every = std::stoi(needValue("--map-voxel-every"));
    } else if (arg == "--out") {
      opt.output_dir = needValue("--out");
    } else if (arg == "--no-save-poses") {
      opt.save_poses = false;
    } else if (arg == "--no-save-map") {
      opt.save_map = false;
    } else if (arg == "--visualize") {
      opt.visualize = true;
    } else if (arg == "--strict") {
      opt.strict_icp = true;
    } else {
      throw std::runtime_error("Unknown argument: " + arg);
    }
  }

  if (opt.sequence.empty()) {
    return std::nullopt;
  }
  return opt;
}

#if defined(LIDAR_SLAM_HAS_VISUALIZATION)
void visualize(const lidar_slam::SlamResult& result) {
  pcl::visualization::PCLVisualizer viewer("kitti_slam");
  viewer.setBackgroundColor(0, 0, 0);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> map_color(
      result.global_map, 200, 200, 200);
  viewer.addPointCloud<pcl::PointXYZI>(result.global_map, map_color, "map");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
                                          "map");

  for (std::size_t i = 1; i < result.poses.size(); ++i) {
    const Eigen::Vector3f a = result.poses[i - 1].block<3, 1>(0, 3);
    const Eigen::Vector3f b = result.poses[i].block<3, 1>(0, 3);
    pcl::PointXYZ pa(a.x(), a.y(), a.z());
    pcl::PointXYZ pb(b.x(), b.y(), b.z());
    viewer.addLine(pa, pb, 0.0, 1.0, 0.0, "traj_" + std::to_string(i));
  }

  viewer.addText("gray=map  green=trajectory", 10, 10, "legend");
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
    const Options opt = *parsed;

    if (!std::filesystem::is_directory(opt.sequence)) {
      std::cerr << "Sequence not found: " << opt.sequence << '\n';
      return 1;
    }

    lidar_slam::SlamConfig cfg;
    cfg.start_frame = opt.start;
    cfg.end_frame = opt.end;
    cfg.strict_icp = opt.strict_icp;
    cfg.preprocess.voxel_leaf = opt.voxel;
    cfg.icp.max_correspondence_distance = opt.icp_max_dist;
    cfg.icp.max_iterations = opt.icp_iter;
    cfg.map.voxel_leaf = opt.map_voxel;
    cfg.map.voxel_every_n_frames = opt.map_voxel_every;

    lidar_slam::SlamRunner runner(cfg);
    const auto result = runner.run(opt.sequence);

    std::cout << "frame  icp  converged  fitness  map_points  pre_ms  icp_ms\n";
    for (const auto& s : result.stats) {
      std::cout << s.frame << ' ' << (s.icp_ran ? 1 : 0) << ' ' << (s.icp_converged ? 1 : 0)
                << ' ' << s.icp_fitness << ' ' << s.map_points << ' ' << s.preprocess_ms << ' '
                << s.icp_ms << '\n';
    }

    std::filesystem::create_directories(opt.output_dir);

    if (opt.save_poses) {
      const auto poses_path = opt.output_dir / "poses.txt";
      lidar_slam::SlamRunner::savePoses(poses_path, result.frame_ids, result.poses);
      std::cout << "Wrote " << poses_path << '\n';
    }

    if (opt.save_map) {
      const auto map_path = opt.output_dir / "global_map.pcd";
      if (pcl::io::savePCDFileBinary(map_path.string(), *result.global_map) != 0) {
        throw std::runtime_error("Failed to save map");
      }
      std::cout << "Wrote " << map_path << " (" << result.global_map->size() << " points)\n";
    }

    if (opt.visualize) {
#if defined(LIDAR_SLAM_HAS_VISUALIZATION)
      visualize(result);
#else
      std::cerr << "Built without visualization.\n";
      return 1;
#endif
    }

    return 0;
  } catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << '\n';
    return 1;
  }
}
