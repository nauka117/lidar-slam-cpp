# Docker

Сборка и запуск без GUI (headless). Данные монтируются с хоста.

```bash
docker compose build
docker compose run --rm lidar_slam
```

По умолчанию в `docker-compose.yml` обрабатываются кадры `0–10` sequence `00`.

Свой диапазон:

```bash
docker compose run --rm lidar_slam bash -c \
  "cmake --build build -j$(nproc) && ./build/kitti_preprocess \
   --sequence data/data_odometry_velodyne/dataset/sequences/00 \
   --start 0 --end 50 --voxel 0.3"
```

`PCLVisualizer` в контейнере без проброса `DISPLAY` не используется; визуализацию запускайте на хосте.
