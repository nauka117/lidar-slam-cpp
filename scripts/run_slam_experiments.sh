#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="$ROOT/build/kitti_slam"
SEQ="${KITTI_SEQ:-$ROOT/data/data_odometry_velodyne/dataset/sequences/00}"
OUT="$ROOT/output/experiments"
END="${SLAM_END:-100}"

if [[ ! -x "$BIN" ]]; then
  echo "Build first: cmake --build $ROOT/build" >&2
  exit 1
fi

mkdir -p "$OUT"

run_exp() {
  local name="$1"
  shift
  echo "=== $name ==="
  "$BIN" --sequence "$SEQ" --start 0 --end "$END" --out "$OUT/$name" "$@" \
    | tee "$OUT/${name}.log"
}

run_exp baseline --voxel 0.3 --icp-max-dist 1.0
run_exp fine_voxel --voxel 0.2 --icp-max-dist 1.0
run_exp coarse_voxel --voxel 0.5 --icp-max-dist 1.0
run_exp tight_icp --voxel 0.3 --icp-max-dist 0.5
run_exp loose_icp --voxel 0.3 --icp-max-dist 2.0

echo "Done. Results in $OUT"
