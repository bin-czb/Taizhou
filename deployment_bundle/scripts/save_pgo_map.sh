#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 1 ]; then
  echo "Usage: $0 <output_map_dir>"
  exit 1
fi

OUT_DIR="$1"
mkdir -p "${OUT_DIR}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
LIVO_WS="${ROOT_DIR}/fast_livo2_ros2_ws"
FASTLIO_WS="${ROOT_DIR}/fastlio_ros2_ws"

conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
export ROS_LOG_DIR=/tmp/pgo_save_map_logs

source /opt/ros/humble/setup.bash
source "${LIVO_WS}/install/setup.bash"
source "${FASTLIO_WS}/install_clean/setup.bash"

ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '${OUT_DIR}', save_patches: true}"

