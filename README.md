# lidar_slam

LiDAR SLAM на C++ и PCL: KITTI Velodyne, препроцессинг, ICP-одометрия, построение карты.

## Зависимости (Arch)

```bash
sudo pacman -S --needed base-devel cmake ninja pcl eigen3 boost openmpi
```

## Сборка

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

## Данные

KITTI Odometry в `data/data_odometry_velodyne/dataset/`.

```bash
./scripts/verify_kitti.sh
```

## Утилиты

### Препроцессинг

```bash
./build/kitti_preprocess \
  --sequence data/data_odometry_velodyne/dataset/sequences/00 \
  --start 0 --end 50 --voxel 0.3
```

### SLAM (ICP + карта)

Обрабатывается только диапазон `--start` … `--end` (по умолчанию 0–100):

```bash
./build/kitti_slam \
  --sequence data/data_odometry_velodyne/dataset/sequences/00 \
  --start 0 --end 100 --voxel 0.3 --icp-max-dist 1.0 \
  --out output/slam
```

Выход: `output/slam/poses.txt`, `output/slam/global_map.pcd`.

Визуализация карты и траектории:

```bash
./build/kitti_slam ... --visualize
```

### Эксперименты с параметрами

```bash
./scripts/run_slam_experiments.sh
```

## Документация

- [docs/research/](docs/research/) — обзор алгоритмов
- [docs/setup/arch.md](docs/setup/arch.md) — окружение
- [docs/setup/datasets.md](docs/setup/datasets.md) — KITTI
- [docs/setup/slam.md](docs/setup/slam.md) — ICP и карта
- [docs/setup/docker.md](docs/setup/docker.md) — Docker
