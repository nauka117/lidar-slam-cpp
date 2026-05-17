# Датасеты

## KITTI Odometry (основной)

- Страница: https://www.cvlibs.net/datasets/kitti/eval_odometry.php
- Velodyne: `data_odometry_velodyne.zip` (~80 GB, все последовательности)

### Структура в этом репозитории

```text
data/data_odometry_velodyne/dataset/sequences/00/velodyne/000000.bin
```

Sequence `00` содержит 4541 кадр. Формат `.bin`: `float32` — `x, y, z, reflectivity`, 16 байт на точку.

### Что реально обрабатывать

| Цель | Рекомендация |
|------|----------------|
| Разработка, отладка | `sequences/00`, кадры `0–50` |
| Расширенный тест | `00`, `--end 4540` |
| Seq. `01`–`21` | Только для сравнительных экспериментов |

Дополнительно (для метрик позже): `data_odometry_calib.zip`, `data_odometry_poses.zip` — распаковать в ту же `dataset/`.

## Другие наборы (справочно)

- **nuScenes** — мультисенсор, автономное вождение
- **MulRan** — мультисессии, loop closure
- **Ford Campus** — лидар + GPS

Код загрузки ориентирован на KITTI `.bin`.
