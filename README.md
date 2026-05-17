# lidar_slam

LiDAR SLAM на C++ и PCL: загрузка KITTI Velodyne, препроцессинг облаков точек.

## Зависимости (Arch)

```bash
sudo pacman -S --needed base-devel cmake ninja pcl eigen3 boost
```

## Сборка

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

## Данные

KITTI Odometry Velodyne в `data/data_odometry_velodyne/dataset/`. Проверка:

```bash
./scripts/verify_kitti.sh
```

## Запуск

Обрабатывается только указанный диапазон кадров (по умолчанию 0–50), не весь датасет:

```bash
./build/kitti_preprocess \
  --sequence data/data_odometry_velodyne/dataset/sequences/00 \
  --start 0 --end 50 --voxel 0.3
```

Сохранение PCD:

```bash
./build/kitti_preprocess \
  --sequence data/data_odometry_velodyne/dataset/sequences/00 \
  --start 0 --end 10 --save-pcd --out output/processed
```

## Документация

- [docs/research/](docs/research/) — обзор SLAM и алгоритмов
- [docs/setup/arch.md](docs/setup/arch.md) — окружение Arch
- [docs/setup/datasets.md](docs/setup/datasets.md) — KITTI
- [docs/setup/docker.md](docs/setup/docker.md) — Docker
