# Docker

Работает на любом Linux, где установлены Docker и Docker Compose. Образ на **Ubuntu 24.04** (пакеты через `apt`); pacman на хосте не нужен.

## Требования

- Docker, Docker Compose plugin
- Клон репозитория
- KITTI Odometry Velodyne, распакованный в:

  `data/data_odometry_velodyne/dataset/sequences/00/velodyne/`

## Сборка и запуск SLAM

```bash
git clone https://github.com/nauka117/lidar-slam-cpp lidar_slam && cd lidar_slam
docker compose build
docker compose run --rm lidar_slam
```

По умолчанию: кадры `0–50`, sequence `00`, выход в `output/slam/`.

## Свои параметры

```bash
docker compose run --rm lidar_slam bash -c \
  'rm -rf build-docker &&
   cmake -S . -B build-docker -DCMAKE_BUILD_TYPE=Release &&
   cmake --build build-docker -j$(nproc) &&
   ./build-docker/kitti_slam \
     --sequence data/data_odometry_velodyne/dataset/sequences/00 \
     --start 0 --end 100 --voxel 0.3 --out output/slam'
```

В контейнере используется `build-docker/`, чтобы не конфликтовать с нативной сборкой в `build/`.

## Препроцессинг отдельно

```bash
docker compose run --rm lidar_slam bash -c \
  'rm -rf build-docker &&
   cmake -S . -B build-docker -DCMAKE_BUILD_TYPE=Release &&
   cmake --build build-docker -j$(nproc) &&
   ./build-docker/kitti_preprocess \
     --sequence data/data_odometry_velodyne/dataset/sequences/00 \
     --start 0 --end 20 --voxel 0.3'
```

## Визуализация

В контейнере без GUI. На хосте (после прогона), если установлен PCL:

```bash
pcl_viewer output/slam/global_map.pcd
```

Или нативная сборка на хосте с флагом `--visualize` (см. [slam.md](slam.md)).
