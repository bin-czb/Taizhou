# 上传清单

检查日期：2026-07-01

目标：把当前系统迁移到另一台 Ubuntu 22.04 + ROS 2 Humble 电脑，并能复现雷达、IMU、相机录包、FASTLIO2、底盘 CAN 里程计功能。

## 必须上传

### 1. FASTLIO2 主工作空间

上传：

```text
/home/czb/Tai Zhou/fastlio_ros2_ws
```

建议只上传源码和文档，排除重新构建可生成的目录：

```text
保留：
  fastlio_ros2_ws/README.md
  fastlio_ros2_ws/README_DEPLOY.md
  fastlio_ros2_ws/src/

排除：
  fastlio_ros2_ws/build/
  fastlio_ros2_ws/build_clean/
  fastlio_ros2_ws/install/
  fastlio_ros2_ws/install_clean/
  fastlio_ros2_ws/log/
  fastlio_ros2_ws/log_clean/
```

当前源码大小约 1.2M，整个工作空间约 43M。

### 2. 传感器驱动与录包工作空间

上传：

```text
/home/czb/Tai Zhou/fast_livo2_ros2_ws
```

建议上传这些内容：

```text
fast_livo2_ros2_ws/README.md
fast_livo2_ros2_ws/.clangd
fast_livo2_ros2_ws/src/
fast_livo2_ros2_ws/scripts/
fast_livo2_ros2_ws/calibration/
```

其中 `src/` 包含：

```text
fast_livo2_ros2_ws/src/fast_livo2_ros2
fast_livo2_ros2_ws/src/livox_ros_driver2
```

排除：

```text
fast_livo2_ros2_ws/build/
fast_livo2_ros2_ws/install/
fast_livo2_ros2_ws/log/
```

可选上传：

```text
fast_livo2_ros2_ws/third_party/LiDAR_IMU_Init
```

不建议上传，除非要重新打印 AprilTag 全套码图：

```text
fast_livo2_ros2_ws/third_party/apriltag-imgs
```

当前 `fast_livo2_ros2_ws/src` 约 1.5M，`calibration` 约 213M，`third_party/apriltag-imgs` 约 652M。

### 3. 标定文件

如果已经上传整个 `fast_livo2_ros2_ws/calibration/`，下面这些已经包含在里面。若不传整个 calibration，至少单独上传：

```text
/home/czb/Tai Zhou/fast_livo2_ros2_ws/calibration/recommended_lidar_imu_extrinsic_20260622.yaml
/home/czb/Tai Zhou/fast_livo2_ros2_ws/calibration/li_init_lidar_imu_result_20260622_150hz.yaml
/home/czb/Tai Zhou/fast_livo2_ros2_ws/calibration/manual_lidar_imu_extrinsic.yaml
/home/czb/Tai Zhou/fast_livo2_ros2_ws/calibration/manual_lidar_camera_extrinsic.yaml
/home/czb/Tai Zhou/fast_livo2_ros2_ws/calibration/lidar_camera_planes.yaml
/home/czb/Tai Zhou/fast_livo2_ros2_ws/calibration/imu_static_analysis.yaml
```

### 4. Livox 配置文件

这些在 `fast_livo2_ros2_ws/src/livox_ros_driver2/config/` 中。至少需要保留：

```text
/home/czb/Tai Zhou/fast_livo2_ros2_ws/src/livox_ros_driver2/config/MID360_config.json
/home/czb/Tai Zhou/fast_livo2_ros2_ws/src/livox_ros_driver2/config/MID360s_config.json
/home/czb/Tai Zhou/fast_livo2_ros2_ws/src/livox_ros_driver2/config/MID360_current_192_168_1_183_config.json
/home/czb/Tai Zhou/fast_livo2_ros2_ws/src/livox_ros_driver2/config/MID360s_current_192_168_1_183_config.json
/home/czb/Tai Zhou/fast_livo2_ros2_ws/src/livox_ros_driver2/config/MID360_sdk_current_192_168_1_183.json
```

新电脑上雷达或电脑 IP 改变时，需要重新检查这些 json。

## 建议上传

### 5. 两个主要验证 rosbag

用于离线验证系统能否跑通：

```text
/home/czb/fast_livo2_validation_bags/01_raw_sensor_inputs/site_nav_20260624_170718
/home/czb/fast_livo2_validation_bags/01_raw_sensor_inputs/site_nav_20260624_170718_calibration_snapshot
/home/czb/fast_livo2_validation_bags/01_raw_sensor_inputs/site_nav_20260624_172258
/home/czb/fast_livo2_validation_bags/01_raw_sensor_inputs/site_nav_20260624_172258_calibration_snapshot
```

体积：

```text
site_nav_20260624_170718                         17G
site_nav_20260624_170718_calibration_snapshot   40K
site_nav_20260624_172258                         14G
site_nav_20260624_172258_calibration_snapshot   40K
```

### 6. LI-Init 标定 rosbag

如果后续要复查外置 IMU 时间偏移或外参，建议上传：

```text
/home/czb/fast_livo2_validation_bags/02_lidar_imu_init/lidar_imu_init_150hz_20260622_145550
```

体积约 1.1G。

### 7. 已生成地图

如果另一台电脑要测试重定位，上传：

```text
/home/czb/Tai Zhou/offline_maps
```

体积约 23M，包含：

```text
external_imu_test_170718
external_imu_test_172258
external_imu_transition_test
```

### 8. CAN 协议 PDF

上传最新协议：

```text
/home/czb/下载/小车CAN.pdf
```

旧协议可选：

```text
/home/czb/桌面/小车CAN通讯(1).pdf
```

### 9. 离线安装依赖源码

如果另一台电脑没有网络，建议额外上传：

```text
/home/czb/Livox-SDK2
```

体积约 109M。若新电脑可以联网，则可按 `README_DEPLOY.md` 重新 `git clone`，不必上传。

## 不建议上传

这些目录建议在新电脑重新生成：

```text
*/build/
*/install/
*/log/
*/build_clean/
*/install_clean/
*/log_clean/
~/.ros/log/
~/.cache/
```

不建议上传整个：

```text
/home/czb
```

里面包含大量无关项目、缓存、安装包和个人配置。

## 已存在旧压缩包

当前已有：

```text
/home/czb/Tai Zhou/release_archives/taizhou_system_20260626.tar.zst
/home/czb/Tai Zhou/release_archives/taizhou_validation_bags_20260626.tar.zst.part_aa ... part_ae
```

注意：这些包是 2026-06-26 生成的，而 2026-07-01 又修改了 `fastlio_ros2_ws/README.md` 和 `README_DEPLOY.md`，因此如果要保证最新部署文档和代码状态，建议重新打包。
