# Среда разработки (Arch Linux)

## Пакеты

```bash
sudo pacman -S --needed base-devel cmake ninja gdb eigen3 boost openmpi
paru -S pcl
```

PCL на Arch тянет VTK с MPI — в `CMakeLists.txt` включён язык C и `find_package(MPI)` до `find_package(PCL)`.

Проверка PCL:

```bash
pacman -Q pcl
pkg-config --modversion pcl_common
```

## Сборка

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build -j$(nproc)
```

## Запуск препроцессинга

```bash
./scripts/verify_kitti.sh

./build/kitti_preprocess \
  --sequence data/data_odometry_velodyne/dataset/sequences/00 \
  --start 0 --end 50 --voxel 0.3
```

Полный архив Velodyne на диске не означает прогон всех кадров: по умолчанию обрабатывается только диапазон `--start` … `--end`.

## Визуализация

`--visualize` требует графическую сессию (Wayland/X11) и сборку PCL с модулем `visualization`.
