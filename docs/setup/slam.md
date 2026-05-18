# LiDAR SLAM (ICP + map merge)

## Pipeline

1. Загрузка KITTI `.bin`, препроцессинг (VoxelGrid + SOR)
2. **Scan-to-scan ICP:** target = кадр $k$, source = кадр $k+1$
3. Позы: $T_0 = I$, $T_{k+1} = T_k \cdot \Delta T$ (матрица из `pcl::IterativeClosestPoint::getFinalTransformation()`)
4. Карта: трансформировать каждый скан в мир и добавить в `global_map`, периодический VoxelGrid на карте

Loop closure и глобальная оптимизация не реализованы — на длинных траекториях ожидается дрейф.

## Сборка

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

## Запуск

```bash
./build/kitti_slam \
  --sequence data/data_odometry_velodyne/dataset/sequences/00 \
  --start 0 --end 100 \
  --voxel 0.3 \
  --icp-max-dist 1.0 \
  --out output/slam
```

Результаты:

- `poses.txt` — строка на кадр: `frame tx ty tz` + 9 элементов вращения (3×3 row-major)
- `global_map.pcd` — объединённая карта

Визуализация (нужен дисплей):

```bash
./build/kitti_slam ... --visualize
```

## Параметры

| Флаг | Default | Описание |
|------|---------|----------|
| `--voxel` | 0.3 | Leaf size препроцессинга (м) |
| `--icp-max-dist` | 1.0 | Max correspondence distance ICP |
| `--icp-iter` | 50 | Итерации ICP |
| `--map-voxel` | 0.5 | Voxel карты (м) |
| `--map-voxel-every` | 10 | Периодичность voxel на карте |
| `--strict` | off | Остановка при несходимости ICP |

## Эксперименты

```bash
./scripts/run_slam_experiments.sh
```

Логи в `output/experiments/`.

| Прогон | Смысл |
|--------|--------|
| baseline | seq 00, voxel 0.3, icp 1.0 |
| fine_voxel | voxel 0.2 |
| coarse_voxel | voxel 0.5 |
| tight_icp | icp-max-dist 0.5 |
| loose_icp | icp-max-dist 2.0 |

Другая последовательность: `--sequence .../sequences/05 --end 80`.
