#!/usr/bin/env bash
set -euo pipefail

conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
export ROS_LOG_DIR=/tmp/fastlio_bag_play_170718_logs

source /opt/ros/humble/setup.bash
ros2 bag play "$HOME/fast_livo2_validation_bags/01_raw_sensor_inputs/site_nav_20260624_170718" \
  --topics /livox/lidar /imu/data

