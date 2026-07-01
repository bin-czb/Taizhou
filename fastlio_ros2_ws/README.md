# FASTLIO2 ROS2 工作空间部署入口

本工作空间是当前小车系统中的 FASTLIO2/建图/定位/底盘 CAN 通讯工作空间：

```text
fastlio_ros2_ws/
  src/FASTLIO2_ROS2/fastlio2      # LiDAR-IMU 里程计
  src/FASTLIO2_ROS2/pgo           # 回环与位姿图优化
  src/FASTLIO2_ROS2/localizer     # 地图重定位
  src/FASTLIO2_ROS2/hba           # 地图优化
  src/FASTLIO2_ROS2/interface     # 自定义服务接口
  src/FASTLIO2_ROS2/chassis_can   # USB-CAN 底盘通讯与 /odom 发布
```

如果只下载这个工作空间，不能直接完整复现“雷达、相机、外置 IMU、USB-CAN、FASTLIO2”的全部功能。另一台 Ubuntu 22.04 + ROS 2 Humble 电脑还需要额外安装：

- ROS 2 Humble 基础环境；
- Livox-SDK2；
- livox_ros_driver2，用于发布 `/livox/lidar` 和 `/livox/imu`；
- 外置 HWT601/HWT905 RS485 IMU 驱动，用于发布 `/imu/data`；
- 可选 USB 相机驱动，用于发布 `/left_camera/image`；
- 可选 `can-utils` 和 SocketCAN 内核驱动，用于 USB-CAN 调试。

详细部署步骤见：

```text
README_DEPLOY.md
```

平时使用时，优先使用干净构建目录：

```bash
cd "/home/czb/Tai Zhou/fastlio_ros2_ws"
conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
source /opt/ros/humble/setup.bash
source install_clean/setup.bash
```

