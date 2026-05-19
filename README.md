# lidar_slam

LiDAR SLAM на **C++** и **PCL**: KITTI Velodyne, препроцессинг, ICP-одометрия, построение карты.  
**ROS не используется.** ОС: **Linux** (нативно или через Docker).

---

## Быстрый старт (Docker, любой Linux)

Подходит для проверки на машине **без pacman** — образ на Ubuntu, сборка и зависимости только внутри контейнера.

**Нужно:** Git, Docker, Docker Compose.

### 1. Клонировать репозиторий

```bash
git clone <URL> lidar_slam
cd lidar_slam
```

### 2. Положить данные KITTI

Скачать [KITTI Odometry Velodyne](https://www.cvlibs.net/datasets/kitti/eval_odometry.php) (`data_odometry_velodyne.zip`, ~80 GB), распаковать так, чтобы существовал путь:

```text
data/data_odometry_velodyne/dataset/sequences/00/velodyne/000000.bin
```

Проверка:

```bash
ls data/data_odometry_velodyne/dataset/sequences/00/velodyne/000000.bin
```

Можно передать уже распакованную папку `data/` архивом, если полный датасет не скачивать.

Подробнее о формате: [docs/setup/datasets.md](docs/setup/datasets.md).

### 3. Собрать образ и запустить SLAM

```bash
docker compose build
docker compose run --rm lidar_slam
```

По умолчанию: sequence `00`, кадры **0–50** (~1–3 минуты, зависит от CPU).

**Результаты на хосте:**

| Файл | Содержание |
|------|------------|
| `output/slam/poses.txt` | Траектория (ICP-одометрия) |
| `output/slam/global_map.pcd` | Глобальная карта (merge облаков) |

Дополнительно: [docs/setup/docker.md](docs/setup/docker.md).

### 4. Визуализация карты (на хосте, с GUI)

```bash
pcl_viewer output/slam/global_map.pcd
```

Ubuntu/Debian: `sudo apt install pcl-tools`  
Альтернатива: CloudCompare, или нативный запуск с `--visualize` (см. ниже).

### 5. Свой диапазон кадров

```bash
docker compose run --rm lidar_slam bash -c \
  'rm -rf build-docker && cmake -S . -B build-docker -DCMAKE_BUILD_TYPE=Release &&
   cmake --build build-docker -j4 &&
   ./build-docker/kitti_slam \
     --sequence data/data_odometry_velodyne/dataset/sequences/00 \
     --start 0 --end 100 --voxel 0.3 --icp-max-dist 1.0 \
     --out output/slam'
```

Сборка в Docker идёт в `build-docker/`, нативная — в `build/` (не смешивать).

---

## Нативная сборка (Arch Linux)
Если другой дистрибутив, задача сводится к тому чтоб найти пакеты в своих package manager'ах
```bash
sudo pacman -S --needed base-devel cmake ninja git eigen3 boost openmpi
paru -S pcl
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

Проверка данных:

```bash
./scripts/verify_kitti.sh
```

### Препроцессинг

```bash
./build/kitti_preprocess \
  --sequence data/data_odometry_velodyne/dataset/sequences/00 \
  --start 0 --end 50 --voxel 0.3
```

### SLAM (ICP + карта)

```bash
./build/kitti_slam \
  --sequence data/data_odometry_velodyne/dataset/sequences/00 \
  --start 0 --end 100 \
  --voxel 0.3 --icp-max-dist 1.0 \
  --out output/slam
```

Визуализация карты и траектории (нужен дисплей):

```bash
./build/kitti_slam ... --visualize
```

Длинный прогон (вся sequence `00`, ~40–60 мин):

```bash
./build/kitti_slam \
  --sequence data/data_odometry_velodyne/dataset/sequences/00 \
  --start 0 --end 4540 \
  --out output/slam_long
```

Подробнее: [docs/setup/arch.md](docs/setup/arch.md), [docs/setup/slam.md](docs/setup/slam.md).

### Эксперименты с параметрами

```bash
./scripts/run_slam_experiments.sh
```

---

## Порядок чтения документации

1. [README.md](README.md) — этот файл, запуск  
2. [docs/research/](docs/research/) — этап 1: теория (SLAM, LOAM, ICP, PCL)  
3. [docs/setup/datasets.md](docs/setup/datasets.md) — KITTI  
4. [docs/setup/slam.md](docs/setup/slam.md) — ICP, позы, карта  
5. Исходники: `src/odometry.cpp`, `src/mapping.cpp`, `src/slam_runner.cpp`

---

## Структура проекта

```text
include/lidar_slam/     библиотека
apps/                   kitti_preprocess, kitti_slam
docs/research/          обзор алгоритмов
docs/setup/             установка, Docker, датасеты
docker/                 Dockerfile
scripts/                verify_kitti.sh, run_slam_experiments.sh
data/                   KITTI (не в git)
```

---

## Ссылки

- [docs/setup/docker.md](docs/setup/docker.md) — Docker
- [docs/setup/datasets.md](docs/setup/datasets.md) — KITTI
- [docs/setup/slam.md](docs/setup/slam.md) — ICP и карта
- [docs/setup/arch.md](docs/setup/arch.md) — Arch без Docker
