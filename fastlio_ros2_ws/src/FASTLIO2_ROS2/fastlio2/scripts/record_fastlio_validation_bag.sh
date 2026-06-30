#!/usr/bin/env bash
set -euo pipefail

OUTPUT_ROOT="${1:-$HOME/fastlio_validation_bags}"
SESSION_NAME="${2:-fastlio_nav_$(date +%Y%m%d_%H%M%S)}"
OUTPUT_DIR="${OUTPUT_ROOT}/${SESSION_NAME}"

mkdir -p "${OUTPUT_ROOT}"

echo "Recording FASTLIO2 validation rosbag:"
echo "  output: ${OUTPUT_DIR}"
echo
echo "Raw topics:"
echo "  /livox/lidar"
echo "  /livox/imu"
echo "  /imu/data"
echo
echo "FASTLIO2 output topics are also recorded when available:"
echo "  /fastlio2/lio_odom"
echo "  /fastlio2/lio_path"
echo "  /fastlio2/body_cloud"
echo "  /fastlio2/world_cloud"
echo
echo "Stop with Ctrl-C after the route is complete."

ros2 bag record \
  /livox/lidar \
  /livox/imu \
  /imu/data \
  /fastlio2/lio_odom \
  /fastlio2/lio_path \
  /fastlio2/body_cloud \
  /fastlio2/world_cloud \
  /tf \
  /tf_static \
  -o "${OUTPUT_DIR}"
