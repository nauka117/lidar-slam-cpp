cmake_minimum_required(VERSION 3.16)
project(lidar_slam LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(MPI REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)

add_library(lidar_slam
  src/kitti_loader.cpp
  src/preprocess.cpp
  src/odometry.cpp
  src/mapping.cpp
  src/slam_runner.cpp
)
target_include_directories(lidar_slam
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE ${PCL_INCLUDE_DIRS}
)
target_link_libraries(lidar_slam PUBLIC ${PCL_LIBRARIES})
target_compile_definitions(lidar_slam PUBLIC ${PCL_DEFINITIONS})
target_compile_options(lidar_slam PRIVATE -Wall -Wextra -Wpedantic)

add_executable(kitti_preprocess apps/kitti_preprocess.cpp)
target_link_libraries(kitti_preprocess PRIVATE lidar_slam)

add_executable(kitti_slam apps/kitti_slam.cpp)
target_link_libraries(kitti_slam PRIVATE lidar_slam)

option(LIDAR_SLAM_ENABLE_VISUALIZATION "Build with PCL visualizer" ON)
if(LIDAR_SLAM_ENABLE_VISUALIZATION)
  find_package(PCL REQUIRED COMPONENTS visualization)
  target_compile_definitions(kitti_preprocess PRIVATE LIDAR_SLAM_HAS_VISUALIZATION=1)
  target_compile_definitions(kitti_slam PRIVATE LIDAR_SLAM_HAS_VISUALIZATION=1)
  target_link_libraries(kitti_preprocess PRIVATE pcl_visualization)
  target_link_libraries(kitti_slam PRIVATE pcl_visualization)
endif()
