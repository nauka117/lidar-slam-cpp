#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SEQ="${1:-$ROOT/data/data_odometry_velodyne/dataset/sequences/00}"

if [[ ! -d "$SEQ/velodyne" ]]; then
  echo "FAIL: velodyne not found under $SEQ" >&2
  exit 1
fi

FIRST="$SEQ/velodyne/000000.bin"
if [[ ! -f "$FIRST" ]]; then
  echo "FAIL: missing $FIRST" >&2
  exit 1
fi

COUNT=$(find "$SEQ/velodyne" -name '*.bin' | wc -l)
SIZE=$(stat -c '%s' "$FIRST")
echo "OK: sequence=$SEQ"
echo "    frames=$COUNT"
echo "    000000.bin bytes=$SIZE (expect multiple of 16)"
if (( SIZE % 16 != 0 )); then
  echo "FAIL: invalid .bin size" >&2
  exit 1
fi
