#!/usr/bin/env bash
set -eo pipefail

OUTPUT_ROOT="${1:-${HOME}/fast_livo2_validation_bags}"
SESSION_NAME="${2:-site_$(date +%Y%m%d_%H%M%S)}"

IMAGE_TOPIC="${IMAGE_TOPIC:-/left_camera/image}"
LIDAR_TOPIC="${LIDAR_TOPIC:-/livox/lidar}"
LIVOX_IMU_TOPIC="${LIVOX_IMU_TOPIC:-/livox/imu}"
IMU_TOPIC="${IMU_TOPIC:-/imu/data}"
LIDAR_TYPE="${LIDAR_TYPE:-livox_ros_driver2/msg/CustomMsg}"
RECORD_LIVOX_IMU="${RECORD_LIVOX_IMU:-1}"
RECORD_TF="${RECORD_TF:-1}"
RECORD_FASTLIO_OUTPUTS="${RECORD_FASTLIO_OUTPUTS:-0}"
RECORD_ODOM="${RECORD_ODOM:-0}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
BAG_GROUP="${BAG_GROUP:-auto}"
BAG_STORAGE="${BAG_STORAGE:-sqlite3}"
BAG_COMPRESSION="${BAG_COMPRESSION:-none}"

OUTPUT_ROOT="${OUTPUT_ROOT/#\~/${HOME}}"

if [ "${BAG_GROUP}" = "auto" ]; then
  if [ "${RECORD_ODOM}" = "1" ]; then
    BAG_GROUP="04_system_with_odom"
  elif [ "${RECORD_FASTLIO_OUTPUTS}" = "1" ]; then
    BAG_GROUP="03_system_with_fastlio"
  else
    BAG_GROUP="01_raw_sensor_inputs"
  fi
fi

if [ -n "${BAG_GROUP}" ] && [ "${BAG_GROUP}" != "." ]; then
  OUTPUT_ROOT="${OUTPUT_ROOT}/${BAG_GROUP}"
fi

OUTPUT_PATH="${OUTPUT_ROOT}/${SESSION_NAME}"
SNAPSHOT_PATH="${OUTPUT_ROOT}/${SESSION_NAME}_calibration_snapshot"

if [ -e "${OUTPUT_PATH}" ]; then
  echo "Output path already exists: ${OUTPUT_PATH}" >&2
  exit 2
fi

mkdir -p "${OUTPUT_ROOT}" "${SNAPSHOT_PATH}"

PKG_PREFIX="$(ros2 pkg prefix fast_livo2_ros2)"
PKG_SHARE="${PKG_PREFIX}/share/fast_livo2_ros2"

if [ -d "${PKG_SHARE}/config" ]; then
  cp "${PKG_SHARE}"/config/*.yaml "${SNAPSHOT_PATH}/" 2>/dev/null || true
fi

cat > "${SNAPSHOT_PATH}/recording_manifest.txt" <<EOF
FAST-LIVO2 validation-data recording manifest
created_at: $(date --iso-8601=seconds)
output_bag: ${OUTPUT_PATH}
image_topic: ${IMAGE_TOPIC}
lidar_topic: ${LIDAR_TOPIC}
lidar_type: ${LIDAR_TYPE}
livox_imu_topic: ${LIVOX_IMU_TOPIC}
record_livox_imu: ${RECORD_LIVOX_IMU}
imu_topic: ${IMU_TOPIC}
record_tf: ${RECORD_TF}
record_fastlio_outputs: ${RECORD_FASTLIO_OUTPUTS}
record_odom: ${RECORD_ODOM}
odom_topic: ${ODOM_TOPIC}
bag_group: ${BAG_GROUP}
bag_storage: ${BAG_STORAGE}
bag_compression: ${BAG_COMPRESSION}

Expected for current rig:
  camera: 1280x720, /left_camera/image, sensor_msgs/msg/Image
  lidar:  Livox Mid360, /livox/lidar, livox_ros_driver2/msg/CustomMsg, about 10 Hz
  livox_imu: Livox built-in IMU, /livox/imu, sensor_msgs/msg/Imu, diagnostic/comparison stream
  external_imu: HWT905/HWT601-compatible RS485, /imu/data, sensor_msgs/msg/Imu, about 150 Hz
  fastlio2_core_inputs: /livox/lidar plus /livox/imu or /imu/data
  fastlio2_outputs_when_running: /fastlio2/lio_odom /fastlio2/lio_path /fastlio2/body_cloud /fastlio2/world_cloud

Notes:
  Record raw topics only. Do not replace /livox/lidar with PointCloud2 if the
  downstream FAST-LIVO2 system expects Livox CustomMsg and per-point offset_time.
  For FASTLIO2 replay/debug, raw /livox/lidar and one IMU topic are sufficient.
  For online FASTLIO2 output inspection, set RECORD_FASTLIO_OUTPUTS=1.
  For odom fusion/chassis comparison, set RECORD_ODOM=1 and make sure /odom exists.
EOF

echo "Checking required recording topics..."
ros2 run fast_livo2_ros2 check_record_topics.py \
  --image-topic "${IMAGE_TOPIC}" \
  --lidar-topic "${LIDAR_TOPIC}" \
  --lidar-type "${LIDAR_TYPE}" \
  --imu-topic "${IMU_TOPIC}"

echo
echo "Quick rate check. Warnings here do not stop recording, but should be read."
timeout 6s ros2 topic hz "${IMAGE_TOPIC}" || true
timeout 6s ros2 topic hz "${LIDAR_TOPIC}" || true
if [ "${RECORD_LIVOX_IMU}" = "1" ]; then
  timeout 6s ros2 topic hz "${LIVOX_IMU_TOPIC}" || true
fi
timeout 6s ros2 topic hz "${IMU_TOPIC}" || true

record_args=(--storage "${BAG_STORAGE}" --output "${OUTPUT_PATH}")

if [ -n "${MAX_BAG_SIZE:-}" ]; then
  record_args+=(--max-bag-size "${MAX_BAG_SIZE}")
fi

if [ "${BAG_COMPRESSION}" != "none" ]; then
  record_args+=(--compression-mode file --compression-format "${BAG_COMPRESSION}")
fi

topics=("${IMAGE_TOPIC}" "${LIDAR_TOPIC}" "${IMU_TOPIC}")
if [ "${RECORD_LIVOX_IMU}" = "1" ]; then
  topics+=("${LIVOX_IMU_TOPIC}")
fi
if [ "${RECORD_TF}" = "1" ]; then
  topics+=(/tf /tf_static)
fi
if [ "${RECORD_FASTLIO_OUTPUTS}" = "1" ]; then
  topics+=(/fastlio2/lio_odom /fastlio2/lio_path /fastlio2/body_cloud /fastlio2/world_cloud)
fi
if [ "${RECORD_ODOM}" = "1" ]; then
  topics+=("${ODOM_TOPIC}")
fi

echo
echo "Recording FAST-LIVO validation rosbag:"
echo "  output:   ${OUTPUT_PATH}"
echo "  snapshot: ${SNAPSHOT_PATH}"
echo "  topics:   ${topics[*]}"
echo
echo "Field guide:"
echo "  1. Keep the rig still for 5-10 seconds at the beginning."
echo "  2. Walk/drive the real navigation route at validation speed."
echo "  3. Include turns, stops, and loop closures if the scene allows."
echo "  4. Avoid violent motion until hardware/software time sync is verified."
echo "  5. Stop with Ctrl-C after the route is complete."
echo "  6. FASTLIO2 core replay needs /livox/lidar plus /livox/imu or /imu/data."
echo "  7. Set RECORD_FASTLIO_OUTPUTS=1 only when FASTLIO2 is running during recording."
echo

ros2 bag record "${record_args[@]}" "${topics[@]}"
