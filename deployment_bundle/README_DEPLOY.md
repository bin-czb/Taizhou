# Tai Zhou FASTLIO2 / Sensor System Deployment Guide

本文档用于把当前电脑上的传感器采集、FASTLIO2、PGO 建图、外置 IMU 回放验证环境迁移到另一台电脑。

目标系统：

```text
Ubuntu 22.04
ROS 2 Humble
```

当前迁移包包含两部分：

```text
1. 系统源码与配置包
   fast_livo2_ros2_ws
   fastlio_ros2_ws
   deployment_bundle

2. 两个验证 rosbag 包
   site_nav_20260624_170718
   site_nav_20260624_172258
```

建议在新电脑上解压到：

```text
/home/<用户名>/Tai Zhou/
```

最终目录结构建议保持为：

```text
Tai Zhou/
├── deployment_bundle/
├── fast_livo2_ros2_ws/
└── fastlio_ros2_ws/
```

如果你的用户名不是 `czb`，没有关系，只要三个目录并排即可。本文档中的脚本会按相对路径查找工作空间。

---

## 1. 压缩包解压与校验

如果压缩包被分卷，例如：

```text
taizhou_system_20260626.tar.zst.part_aa
taizhou_system_20260626.tar.zst.part_ab
...
```

先校验：

```bash
cd "<压缩包所在目录>"
sha256sum -c SHA256SUMS.txt
```

解压系统包：

```bash
mkdir -p "$HOME/Tai Zhou"
cat taizhou_system_20260626.tar.zst.part_* | tar --zstd -xpf - -C "$HOME/Tai Zhou"
```

如果没有分卷，是单个 `.tar.zst`：

```bash
mkdir -p "$HOME/Tai Zhou"
tar --zstd -xpf taizhou_system_20260626.tar.zst -C "$HOME/Tai Zhou"
```

解压 rosbag 包：

```bash
mkdir -p "$HOME/fast_livo2_validation_bags"
cat taizhou_validation_bags_20260626.tar.zst.part_* | tar --zstd -xpf - -C "$HOME/fast_livo2_validation_bags"
```

解压后两个 bag 推荐位置：

```text
$HOME/fast_livo2_validation_bags/01_raw_sensor_inputs/site_nav_20260624_170718
$HOME/fast_livo2_validation_bags/01_raw_sensor_inputs/site_nav_20260624_172258
```

---

## 2. 系统内容说明

### 2.1 `fast_livo2_ros2_ws`

用于传感器启动、数据录制、标定文件管理。

主要包：

```text
fast_livo2_ros2      相机、外置 IMU、录包脚本、标定辅助脚本
livox_ros_driver2    Livox Mid360 ROS 2 驱动
```

关键话题：

```text
/left_camera/image   sensor_msgs/msg/Image
/livox/lidar         livox_ros_driver2/msg/CustomMsg
/livox/imu           sensor_msgs/msg/Imu
/imu/data            sensor_msgs/msg/Imu
```

### 2.2 `fastlio_ros2_ws`

用于 FASTLIO2 建图、PGO、重定位、HBA、底盘 CAN 节点。

主要包：

```text
fastlio2      激光惯性里程计
pgo           位姿图与地图保存
localizer     已有地图上的 ICP 重定位
hba           地图优化
interface     自定义服务
chassis_can   底盘 CAN 通讯与 odom
```

FASTLIO2 主要输出：

```text
/fastlio2/lio_odom
/fastlio2/lio_path
/fastlio2/body_cloud
/fastlio2/world_cloud
TF: map -> body
```

---

## 3. 必装依赖

先安装 ROS 2 Humble，建议 Desktop 版。

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git wget curl \
  python3-pip python3-serial python3-yaml python3-numpy python3-opencv \
  python3-colcon-common-extensions python3-rosdep \
  v4l-utils can-utils zstd pigz \
  libpcl-dev libeigen3-dev libyaml-cpp-dev libgtsam-dev \
  ros-humble-desktop \
  ros-humble-rviz2 \
  ros-humble-pcl-conversions \
  ros-humble-tf2-ros \
  ros-humble-message-filters \
  ros-humble-rosbag2 \
  ros-humble-rosbag2-storage-default-plugins \
  ros-humble-cv-bridge \
  ros-humble-image-transport
```

如果 `rosdep` 未初始化：

```bash
sudo rosdep init
rosdep update
```

运行 ROS 时避免 Conda 污染：

```bash
conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
export PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
```

---

## 4. Livox-SDK2 安装

`livox_ros_driver2` 依赖 Livox-SDK2。新电脑上需要安装一次。

```bash
cd "$HOME"
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir -p build
cd build
cmake ..
make -j"$(nproc)"
sudo make install
sudo ldconfig
```

如果无法联网，可以从旧电脑复制已下载的 Livox-SDK2 源码目录到新电脑后执行同样的 `cmake/make/install`。

---

## 5. Sophus 安装

FASTLIO2 依赖 Sophus。当前工程使用 Sophus 1.22.10。

```bash
cd /tmp
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.22.10
mkdir -p build
cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make -j"$(nproc)"
sudo make install
sudo ldconfig
```

---

## 6. 一键构建

确认目录结构如下：

```text
Tai Zhou/
├── deployment_bundle/
├── fast_livo2_ros2_ws/
└── fastlio_ros2_ws/
```

然后执行：

```bash
cd "$HOME/Tai Zhou/deployment_bundle/scripts"
bash build_all.sh
```

该脚本会做：

```text
1. 构建 fast_livo2_ros2_ws
2. 构建 fastlio_ros2_ws 到 build_clean/install_clean
3. 固定 Python 为 /usr/bin/python3，避免 Conda Python 破坏 rclpy
```

如果你要手动构建：

```bash
cd "$HOME/Tai Zhou/fast_livo2_ros2_ws"
conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash

cd "$HOME/Tai Zhou/fastlio_ros2_ws"
conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
source /opt/ros/humble/setup.bash
source "$HOME/Tai Zhou/fast_livo2_ros2_ws/install/setup.bash"
colcon --log-base log_clean build \
  --symlink-install \
  --build-base build_clean \
  --install-base install_clean \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install_clean/setup.bash
```

---

## 7. 传感器环境

### 7.1 Livox Mid360 网络

当前配置：

```text
PC 有线网卡 IP: 192.168.1.50/24
Mid360 IP:      192.168.1.183
Livox config:   fast_livo2_ros2_ws/src/livox_ros_driver2/config/MID360s_config.json
```

设置 PC 有线 IP，连接名需要按新电脑实际修改：

```bash
nmcli connection show

sudo nmcli connection modify "<有线连接名>" \
  ipv4.method manual \
  ipv4.addresses 192.168.1.50/24

sudo nmcli connection up "<有线连接名>"
```

检查：

```bash
ping 192.168.1.183
```

启动 Livox：

```bash
cd "$HOME/Tai Zhou/fast_livo2_ros2_ws"
conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

检查：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

### 7.2 USB 相机

查设备：

```bash
v4l2-ctl --list-devices
```

启动：

```bash
cd "$HOME/Tai Zhou/fast_livo2_ros2_ws"
conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run fast_livo2_ros2 start_usb_camera.sh \
  0 1280 720 30.0 /left_camera/image
```

如果 `/dev/video0` 不对，把命令中的 `0` 改成实际相机编号。

### 7.3 HWT601/HWT905 RS485 外置 IMU

当前配置：

```text
port: /dev/ttyUSB0
baudrate: 115200
address: 80 decimal, 0x50 hex
rate: 150 Hz
accel_range_g: 4.0
topic: /imu/data
frame_id: imu_link
```

启动：

```bash
cd "$HOME/Tai Zhou/fast_livo2_ros2_ws"
conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch fast_livo2_ros2 hwt601_485_imu.launch.py \
  port:=/dev/ttyUSB0 \
  baudrate:=115200 \
  address:=80 \
  rate_hz:=150.0 \
  accel_range_g:=4.0
```

检查：

```bash
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
```

如果 USB 权限不足：

```bash
sudo usermod -aG dialout "$USER"
```

执行后需要重新登录。

---

## 8. 录包

先分别启动 Livox、相机、外置 IMU，然后录包：

```bash
cd "$HOME/Tai Zhou/fast_livo2_ros2_ws"
conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run fast_livo2_ros2 record_fast_livo_site_bag.sh \
  "$HOME/fast_livo2_validation_bags" \
  "site_nav_$(date +%Y%m%d_%H%M%S)"
```

录包会检查：

```text
/left_camera/image
/livox/lidar
/imu/data
```

并记录：

```text
/left_camera/image
/livox/lidar
/imu/data
/livox/imu
/tf
/tf_static
```

录制建议：

```text
1. 开始保持静止 5-10 秒。
2. 慢速运动，避免突然甩动。
3. 包含直线、转弯、停止。
4. Ctrl-C 后等待 rosbag 输出 Recording stopped，再断电。
```

---

## 9. 使用外置 IMU 跑 FASTLIO2 + RViz

当前外置 IMU 配置：

```text
fastlio_ros2_ws/src/FASTLIO2_ROS2/fastlio2/config/lio_mid360_hwt905_external_imu.yaml
imu_topic: /imu/data
lidar_topic: /livox/lidar
imu_acc_scale: 1.0
imu_time_lag_to_lidar_s: -0.053331
```

启动 FASTLIO2 + RViz：

```bash
cd "$HOME/Tai Zhou/fastlio_ros2_ws"
conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
export ROS_LOG_DIR=/tmp/fastlio_external_imu_logs

source /opt/ros/humble/setup.bash
source "$HOME/Tai Zhou/fast_livo2_ros2_ws/install/setup.bash"
source install_clean/setup.bash

ros2 launch fastlio2 lio_launch.py \
  config_path:="$HOME/Tai Zhou/fastlio_ros2_ws/src/FASTLIO2_ROS2/fastlio2/config/lio_mid360_hwt905_external_imu.yaml" \
  use_rviz:=true
```

RViz 里重点看：

```text
/fastlio2/world_cloud
/fastlio2/body_cloud
/fastlio2/lio_path
/fastlio2/lio_odom
```

另开终端检查：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/Tai Zhou/fast_livo2_ros2_ws/install/setup.bash"
source "$HOME/Tai Zhou/fastlio_ros2_ws/install_clean/setup.bash"

ros2 topic hz /fastlio2/lio_odom
ros2 topic echo /fastlio2/lio_odom --once
```

---

## 10. 回放两个验证包

不要直接把两个包硬接成一条连续建图流。两包之间有时间缺口，如果设备在中间移动过，FASTLIO2 状态会不连续。

### 10.1 第一包

终端 1 启动 FASTLIO2 + RViz，见第 9 节。

终端 2 播放：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/Tai Zhou/fast_livo2_ros2_ws/install/setup.bash"

ros2 bag play "$HOME/fast_livo2_validation_bags/01_raw_sensor_inputs/site_nav_20260624_170718" \
  --topics /livox/lidar /imu/data
```

### 10.2 第二包

先 Ctrl-C 关闭 FASTLIO2 和 bag play。重新启动 FASTLIO2 + RViz，再播放第二包：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/Tai Zhou/fast_livo2_ros2_ws/install/setup.bash"

ros2 bag play "$HOME/fast_livo2_validation_bags/01_raw_sensor_inputs/site_nav_20260624_172258" \
  --topics /livox/lidar /imu/data
```

已验证情况：

```text
site_nav_20260624_170718:
  /fastlio2/lio_odom 约 10 Hz，外置 IMU 跑通较稳定。

site_nav_20260624_172258:
  /fastlio2/lio_odom 约 9-11 Hz，但 IMU/LiDAR 覆盖等待较多，质量弱于第一包。
```

---

## 11. PGO 保存地图

FASTLIO2 单独运行只发布 odom/path/cloud。要保存 `map.pcd`，需要启动 PGO。

终端 1：启动 FASTLIO2，见第 9 节，建议 `use_rviz:=false` 或者按需开 RViz。

终端 2：启动 PGO。

```bash
cd "$HOME/Tai Zhou/fastlio_ros2_ws"
conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
source /opt/ros/humble/setup.bash
source "$HOME/Tai Zhou/fast_livo2_ros2_ws/install/setup.bash"
source install_clean/setup.bash

ros2 run pgo pgo_node --ros-args \
  -p config_path:="$HOME/Tai Zhou/fastlio_ros2_ws/src/FASTLIO2_ROS2/pgo/config/pgo.yaml"
```

终端 3：播放 bag。

终端 4：保存地图。

```bash
mkdir -p "$HOME/Tai Zhou/offline_maps/site_nav_170718"

ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '$HOME/Tai Zhou/offline_maps/site_nav_170718', save_patches: true}"
```

输出：

```text
map.pcd
poses.txt
patches/*.pcd
```

注意：不要直接用 `ros2 launch pgo pgo_launch.py` 跑外置 IMU，因为它默认会启动 `lio.yaml`，容易切回默认内置 IMU配置。推荐分开启动 FASTLIO2 和 PGO。

---

## 12. 内置 IMU 与外置 IMU 配置区别

外置 IMU：

```text
config: lio_mid360_hwt905_external_imu.yaml
imu_topic: /imu/data
imu_acc_scale: 1.0
imu_time_lag_to_lidar_s: -0.053331
```

Livox 内置 IMU：

```text
config: lio_mid360_builtin_imu.yaml
imu_topic: /livox/imu
imu_acc_scale: 9.80665
imu_time_lag_to_lidar_s: 0.0
```

不要混用。如果用错配置，常见表现是：

```text
1. lio_odom 不输出。
2. 点云撕裂。
3. 轨迹快速发散。
4. 剧烈运动后无法恢复。
```

---

## 13. 已知限制与排查

### 13.1 剧烈运动会飞

FASTLIO2 是连续里程计，不是全局定位系统。主节点飞了以后没有自动找回原位置的机制。

原因通常包括：

```text
1. IMU 量程饱和。
2. LiDAR-IMU 时间同步误差。
3. LiDAR-IMU 外参误差。
4. 场景特征少或点云退化。
5. 初始化时没有静止。
```

建议：

```text
1. 启动后静止 5-10 秒。
2. 先慢速运动验证。
3. 提高 IMU 量程，例如 8g/16g，视硬件支持而定。
4. 重新做 LiDAR-IMU 外参和时间偏移标定。
5. 使用 localizer 做已有地图重定位，不要指望 FASTLIO2 主节点自动恢复。
```

### 13.2 rclpy / Python 报错

如果看到：

```text
ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
```

基本是 Conda Python 污染了 ROS2。

处理：

```bash
conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
which python3
```

应输出：

```text
/usr/bin/python3
```

### 13.3 Livox 无数据

检查：

```bash
ping 192.168.1.183
ip addr
ros2 topic list | grep livox
```

确认 `MID360s_config.json` 中：

```text
host_ip:  192.168.1.50
lidar_ip: 192.168.1.183
```

### 13.4 外置 IMU 没数据

检查：

```bash
ls -l /dev/ttyUSB*
groups
ros2 run fast_livo2_ros2 probe_hwt601_485.py
```

需要 `dialout` 权限。

### 13.5 第二包质量弱于第一包

已验证：

```text
第一包 /imu/data 约 128 Hz，/livox/lidar 约 10 Hz。
第二包 /imu/data 约 115 Hz，/livox/lidar 约 7.8 Hz，IMU/LiDAR 覆盖等待更多。
```

所以第二包可以跑通，但质量不如第一包。

---

## 14. 底盘 CAN 节点

底盘包在：

```text
fastlio_ros2_ws/src/FASTLIO2_ROS2/chassis_can
```

CAN 波特率：

```text
100000
```

启动 CAN：

```bash
cd "$HOME/Tai Zhou/fastlio_ros2_ws"
conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run chassis_can setup_can.sh can0 100000
```

启动底盘通讯与 odom：

```bash
ros2 launch chassis_can chassis_can.launch.py \
  interface:=can0 \
  listen_only:=false \
  enable_on_start:=true \
  query_on_start:=true \
  query_period_s:=1.0 \
  feedback_parse_mode:=small_board \
  feedback_byte_order:=big \
  feedback_crc_enabled:=false
```

当前 8 字节反馈解析：

```text
byte0-1 左角度，高字节在前
byte2-3 右角度，高字节在前
byte4-5 左速度，高字节在前
byte6-7 右速度，高字节在前
```

示例：

```text
00 5C -> 0x005C -> 92
01 13 -> 0x0113 -> 275
```

原始量话题以十六进制字符串发布：

```text
/left_motor/angle
/right_motor/angle
/left_motor/speed
/right_motor/speed
/chassis_can/decoded_hex
```

物理量和里程计仍为数值：

```text
/chassis/vx
/chassis/wz
/odom
```

---

## 15. 推荐迁移后验收顺序

1. 构建两个工作空间。
2. 不接传感器，先回放第一包，确认 `/fastlio2/lio_odom` 输出。
3. 回放第二包，确认也能输出，但允许有少量 warning。
4. 接 Livox，确认 `/livox/lidar` 和 `/livox/imu`。
5. 接外置 IMU，确认 `/imu/data`。
6. 接相机，确认 `/left_camera/image`。
7. 现场录一个短包。
8. 用外置 IMU 配置回放短包看 RViz。
9. 再接底盘 CAN 和 `/odom`。

---

## 16. 当前已验证的离线结果

在原电脑上已验证：

```text
第一包:
  /home/czb/fast_livo2_validation_bags/01_raw_sensor_inputs/site_nav_20260624_170718
  FASTLIO2 外置 IMU跑通，PGO 保存地图成功。

第二包:
  /home/czb/fast_livo2_validation_bags/01_raw_sensor_inputs/site_nav_20260624_172258
  FASTLIO2 外置 IMU跑通，PGO 保存地图成功。

不建议:
  两个包直接硬接成一条连续建图流。
```

原电脑测试输出：

```text
offline_maps/external_imu_test_170718/map.pcd
offline_maps/external_imu_test_172258/map.pcd
```

这些输出只用于证明链路，不一定是最终地图。
