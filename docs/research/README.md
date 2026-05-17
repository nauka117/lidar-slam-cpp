# LiDAR SLAM — обзорные материалы

Краткая выжимка по теории лидарного SLAM, трём классам алгоритмов и библиотеке PCL (C++).

## Содержание


| Файл                                                 | Тема                                                                     |
| ---------------------------------------------------- | ------------------------------------------------------------------------ |
| [01-slam-basics.md](01-slam-basics.md)               | Основы SLAM, одометрия, картирование, loop closure, лидарные особенности |
| [02-feature-based-slam.md](02-feature-based-slam.md) | SLAM на признаках: LOAM, LeGO-LOAM, LIO-SAM                              |
| [03-direct-slam.md](03-direct-slam.md)               | Прямое сопоставление: ICP, GICP, NDT                                     |
| [04-voxel-based-slam.md](04-voxel-based-slam.md)     | Воксельные карты: OctoMap, TSDF                                          |
| [05-pcl-library.md](05-pcl-library.md)               | Point Cloud Library                                                      |


## Порядок чтения

1. [01-slam-basics.md](01-slam-basics.md) — общая рамка.
2. [02](02-feature-based-slam.md), [03](03-direct-slam.md), [04](04-voxel-based-slam.md) — три подхода к SLAM (порядок произвольный).
3. [05-pcl-library.md](05-pcl-library.md) — инструментарий для работы с облаками точек.

## Источники

- Zhang J., Singh S. *LOAM: Lidar Odometry and Mapping in Real-time.* RSS 2014.
- Hornung A. et al. *OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees.* Autonomous Robots, 2013.
- Rusu R. B., Cousins S. *3D is here: Point Cloud Library (PCL).* ICRA 2011.
- [Point Cloud Library](https://pointclouds.org/documentation/)
- [KITTI Odometry](https://www.cvlibs.net/datasets/kitti/eval_odometry.php)

