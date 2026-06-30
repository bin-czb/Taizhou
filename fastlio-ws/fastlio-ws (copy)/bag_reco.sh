#!/bin/bash

# 生成时间戳以确保每次录制的 bag 文件夹名称唯一
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="sensor_data_${TIMESTAMP}"

echo "========================================"
echo " 开始录制 ROS 2 Bag..."
echo " 保存目录: ./${BAG_NAME}"
echo " 按 Ctrl+C 停止录制"
echo "========================================"

# 执行录制命令并指定输出文件夹及所有需要的 topics
ros2 bag record -o "${BAG_NAME}" \
    /livox/imu \
    /livox/lidar \
    /odom \
    /odom_path \
    /tf \
    /tf_static
