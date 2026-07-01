# FASTLIO2 ROS2 小车系统部署说明

本文档用于把当前 `fastlio_ros2_ws` 部署到另一台 Ubuntu 22.04 + ROS 2 Humble 电脑，并尽量复现当前机器上的功能。

## 1. 重要结论

`fastlio_ros2_ws` 不是完全独立的传感器工作空间。它包含 FASTLIO2、回环、重定位、地图优化、底盘 CAN 通讯代码；但雷达驱动、外置 IMU 驱动、相机启动脚本通常来自另一个传感器工作空间。

因此迁移时至少需要准备两类内容：

```text
1. fastlio_ros2_ws
   FASTLIO2 / PGO / Localizer / HBA / chassis_can

2. 传感器驱动环境
   Livox-SDK2
   livox_ros_driver2
   外置 IMU 驱动，发布 /imu/data
   可选 USB 相机，发布 /left_camera/image
```

当前 FASTLIO2 主要输入话题：

```text
/livox/lidar   livox_ros_driver2/msg/CustomMsg
/livox/imu     sensor_msgs/msg/Imu，Livox MID360 内置 IMU
/imu/data      sensor_msgs/msg/Imu，外置 HWT601/HWT905 RS485 IMU
/odom          nav_msgs/msg/Odometry，可选，来自 chassis_can
```

当前 FASTLIO2 主要输出话题：

```text
/fastlio2/lio_odom
/fastlio2/lio_path
/fastlio2/body_cloud
/fastlio2/world_cloud
TF: map -> body
```

底盘 CAN 节点主要输出：

```text
/odom
/wheel_odom_twist
/chassis/vx
/chassis/wz
/left_track/velocity
/right_track/velocity
/chassis_can/decoded_hex
```

## 2. 推荐目录结构

建议在新电脑上保持类似目录，路径中可以有空格，但命令必须加引号：

```text
/home/<user>/Tai Zhou/fastlio_ros2_ws
/home/<user>/Tai Zhou/fast_livo2_ros2_ws       # 如果你迁移了传感器工作空间
```

如果只迁移 `fastlio_ros2_ws`，也可以单独安装 Livox 驱动：

```text
/home/<user>/ws_livox
```

下面命令假设工作空间路径为：

```bash
FASTLIO_WS="$HOME/Tai Zhou/fastlio_ros2_ws"
SENSOR_WS="$HOME/Tai Zhou/fast_livo2_ros2_ws"
```

如果你的实际路径不同，请对应替换。

## 3. 基础系统环境

系统要求：

```text
Ubuntu 22.04
ROS 2 Humble
```

建议先关闭 conda 对 ROS Python 的影响：

```bash
conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
```

安装常用依赖：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git wget curl \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  libpcl-dev libeigen3-dev libyaml-cpp-dev libgtsam-dev \
  can-utils net-tools \
  ros-humble-desktop \
  ros-humble-pcl-conversions ros-humble-pcl-ros \
  ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-eigen \
  ros-humble-message-filters ros-humble-rosidl-default-generators \
  ros-humble-rviz2
```

初始化 rosdep：

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

## 4. 安装 Sophus

FASTLIO2 依赖 Sophus。当前机器使用的是 Sophus 1.22.10：

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
```

如果已经安装过 Sophus，可以检查：

```bash
ls /usr/local/share/sophus/cmake
```

## 5. 安装 Livox-SDK2

MID360 需要 Livox-SDK2：

```bash
cd "$HOME"
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir -p build
cd build
cmake ..
make -j"$(nproc)"
sudo make install
```

## 6. 安装 livox_ros_driver2

如果你已经迁移了 `fast_livo2_ros2_ws` 并且里面包含 `livox_ros_driver2`，可以直接编译那个工作空间。

如果没有，单独创建 Livox 工作空间：

```bash
mkdir -p "$HOME/ws_livox/src"
cd "$HOME/ws_livox/src"
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd livox_ros_driver2
source /opt/ros/humble/setup.bash
./build.sh humble
```

启动前检查或修改 MID360 配置文件，例如：

```text
livox_ros_driver2/config/MID360_config.json
livox_ros_driver2/config/MID360s_config.json
```

需要保证电脑网卡和 MID360 在同一网段。当前机器曾经识别到 MID360 地址为：

```text
192.168.1.183
```

新电脑上请以实际雷达 IP 和网卡 IP 为准。

## 7. 外置 IMU 与相机

外置 IMU 目标话题：

```text
/imu/data  sensor_msgs/msg/Imu
```

当前使用习惯参数：

```text
port: /dev/ttyUSB0
baudrate: 115200
address: 80，也就是 0x50
rate_hz: 150.0
accel_range_g: 4.0
```

如果你迁移了原来的传感器工作空间，可以这样启动：

```bash
cd "$SENSOR_WS"
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

USB 相机目标话题：

```text
/left_camera/image  sensor_msgs/msg/Image
```

如果你迁移了原来的相机启动脚本，可以这样启动：

```bash
cd "$SENSOR_WS"
conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run fast_livo2_ros2 start_usb_camera.sh \
  0 1280 720 30.0 /left_camera/image
```

注意：`fastlio_ros2_ws` 本身不依赖相机；相机主要用于 FAST-LIVO 或采集数据集。

## 8. 编译 fastlio_ros2_ws

进入工作空间：

```bash
cd "$FASTLIO_WS"
conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
source /opt/ros/humble/setup.bash
```

如果 Livox 驱动在独立工作空间：

```bash
source "$HOME/ws_livox/install/setup.bash"
```

如果 Livox 驱动在传感器工作空间：

```bash
source "$SENSOR_WS/install/setup.bash"
```

安装 ROS 依赖：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

使用干净构建目录编译：

```bash
colcon --log-base log_clean build \
  --symlink-install \
  --build-base build_clean \
  --install-base install_clean \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

编译完成后 source：

```bash
source install_clean/setup.bash
```

检查包是否存在：

```bash
ros2 pkg list | grep -E "fastlio2|pgo|localizer|hba|interface|chassis_can"
```

## 9. 启动 Livox MID360

如果使用迁移过来的传感器工作空间：

```bash
cd "$SENSOR_WS"
conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

如果使用单独的 `ws_livox`：

```bash
cd "$HOME/ws_livox"
conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

检查话题：

```bash
ros2 topic list | grep -E "/livox/lidar|/livox/imu"
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

正常期望：

```text
/livox/lidar  约 10 Hz
/livox/imu    约 200 Hz，视驱动配置而定
```

## 10. 启动 FASTLIO2

### 10.1 使用 MID360 内置 IMU

这是当前更推荐的基线方式：

```bash
cd "$FASTLIO_WS"
conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
export ROS_LOG_DIR=/tmp/fastlio_builtin_imu_logs

source /opt/ros/humble/setup.bash
source "$SENSOR_WS/install/setup.bash" 2>/dev/null || source "$HOME/ws_livox/install/setup.bash"
source install_clean/setup.bash

ros2 launch fastlio2 lio_launch.py \
  config_path:="$FASTLIO_WS/src/FASTLIO2_ROS2/fastlio2/config/lio_mid360_builtin_imu.yaml" \
  use_rviz:=true
```

该配置使用：

```text
LiDAR: /livox/lidar
IMU:   /livox/imu
imu_acc_scale: 9.80665
imu_time_lag_to_lidar_s: 0.0
```

### 10.2 使用外置 HWT601/HWT905 IMU

先启动外置 IMU，使其发布 `/imu/data`，再启动 FASTLIO2：

```bash
cd "$FASTLIO_WS"
conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
export ROS_LOG_DIR=/tmp/fastlio_external_imu_logs

source /opt/ros/humble/setup.bash
source "$SENSOR_WS/install/setup.bash" 2>/dev/null || source "$HOME/ws_livox/install/setup.bash"
source install_clean/setup.bash

ros2 launch fastlio2 lio_launch.py \
  config_path:="$FASTLIO_WS/src/FASTLIO2_ROS2/fastlio2/config/lio_mid360_hwt905_external_imu.yaml" \
  use_rviz:=true
```

该配置使用：

```text
LiDAR: /livox/lidar
IMU:   /imu/data
imu_acc_scale: 1.0
imu_time_lag_to_lidar_s: 0.0
```

当前外置 IMU 主配置默认不加时间补偿，因为实测加入 LI-Init 的 53.331 ms 偏移后效果变差。

保留两个实验配置用于现场对比：

```text
lio_mid360_hwt905_external_imu_time_plus53ms.yaml
lio_mid360_hwt905_external_imu_time_minus53ms.yaml
```

## 11. 离线 rosbag 回放

内置 IMU 回放：

```bash
ros2 bag play "/path/to/bag" \
  --topics /livox/lidar /livox/imu
```

外置 IMU 回放：

```bash
ros2 bag play "/path/to/bag" \
  --topics /livox/lidar /imu/data
```

如果要看相机数据是否录入：

```bash
ros2 bag info "/path/to/bag"
ros2 bag play "/path/to/bag" --topics /left_camera/image
```

## 12. 启动 PGO 保存地图

启动 FASTLIO2 后，另开终端启动 PGO：

```bash
cd "$FASTLIO_WS"
conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
source /opt/ros/humble/setup.bash
source "$SENSOR_WS/install/setup.bash" 2>/dev/null || source "$HOME/ws_livox/install/setup.bash"
source install_clean/setup.bash

ros2 launch pgo pgo_launch.py
```

保存地图：

```bash
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '$HOME/fastlio_maps/test_map', save_patches: true}"
```

## 13. 启动底盘 CAN 与 /odom

安装 CAN 调试工具：

```bash
sudo apt install -y can-utils
```

打开 can0，当前底盘协议波特率为 100000：

```bash
cd "$FASTLIO_WS"
conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
source /opt/ros/humble/setup.bash
source install_clean/setup.bash

ros2 run chassis_can setup_can.sh can0 100000
ip -details link show can0
```

只接收不主动下发：

```bash
ROS_LOG_DIR=/tmp/chassis_can_logs ros2 launch chassis_can chassis_can.launch.py \
  interface:=can0 \
  listen_only:=true \
  enable_on_start:=false \
  query_on_start:=false \
  wheel_base:=0.80 \
  wheel_radius:=0.22
```

主动开启并周期请求反馈：

```bash
ROS_LOG_DIR=/tmp/chassis_can_logs ros2 launch chassis_can chassis_can.launch.py \
  interface:=can0 \
  listen_only:=false \
  enable_on_start:=true \
  query_on_start:=true \
  query_period_s:=1.0 \
  wheel_base:=0.80 \
  wheel_radius:=0.22 \
  gear_ratio:=1.0 \
  speed_scale:=1.0 \
  left_sign:=1.0 \
  right_sign:=1.0
```

检查 odom：

```bash
ros2 topic echo /odom --once
ros2 topic echo /wheel_odom_twist --once
ros2 topic echo /chassis_can/decoded_hex
```

底盘尺寸参数说明：

```text
wheel_base   左右轮/履带中心距，当前实测 0.80 m
wheel_radius 驱动轮半径，轮直径 44 cm，所以半径 0.22 m
gear_ratio   电机到驱动轮减速比，未知时先用 1.0
speed_scale  协议速度原始值到 rpm 的比例，未知时先用 1.0
left_sign/right_sign 前进方向符号，方向反了就改成 -1.0
```

## 14. 常用检查命令

检查 ROS 环境：

```bash
echo "$ROS_DISTRO"
echo "$ROS_DOMAIN_ID"
which python3
python3 --version
```

检查关键话题：

```bash
ros2 topic list | grep -E "/livox/lidar|/livox/imu|/imu/data|/left_camera/image|/odom|/fastlio2/lio_odom"
```

检查消息频率：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /imu/data
ros2 topic hz /fastlio2/lio_odom
```

检查 FASTLIO2 配置是否读对：

```bash
ros2 launch fastlio2 lio_launch.py \
  config_path:="$FASTLIO_WS/src/FASTLIO2_ROS2/fastlio2/config/lio_mid360_builtin_imu.yaml" \
  use_rviz:=false
```

启动日志应显示类似：

```text
IMU topic: /livox/imu, LiDAR topic: /livox/lidar
```

## 15. 常见问题

### 15.1 `Package 'fastlio2' not found`

通常是没有 source 当前工作空间：

```bash
cd "$FASTLIO_WS"
source /opt/ros/humble/setup.bash
source install_clean/setup.bash
ros2 pkg list | grep fastlio2
```

### 15.2 `Package 'chassis_can' not found`

同样是 source 了错误工作空间，或者没有重新编译：

```bash
cd "$FASTLIO_WS"
source /opt/ros/humble/setup.bash
source install_clean/setup.bash
ros2 pkg list | grep chassis_can
```

### 15.3 `not found: ... install/interface/share/interface/local_setup.bash`

这是旧 `install/` 目录里的残留环境提示。优先使用当前干净构建：

```bash
source install_clean/setup.bash
```

如果新电脑从源码重新构建，一般不会出现这个提示。

### 15.4 `ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`

这是 conda Python 覆盖 ROS 2 Python 导致的。启动前执行：

```bash
conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
source /opt/ros/humble/setup.bash
```

确认 Python 是系统 Python：

```bash
which python3
python3 --version
```

应优先使用 `/usr/bin/python3` 和 Python 3.10。

### 15.5 `candump` 或 `cansend` 找不到

安装：

```bash
sudo apt install -y can-utils
```

### 15.6 `cansend can0 ...` 出现 `write: No buffer space available`

常见原因：

```text
1. can0 没有正常 up；
2. 波特率不对，当前底盘是 100000；
3. CAN_H/CAN_L 接反；
4. 没有终端电阻或总线没有 ACK；
5. USB-CAN 驱动不匹配。
```

先检查：

```bash
ip -details link show can0
```

期望看到：

```text
state ERROR-ACTIVE
bitrate 100000
```

### 15.7 外置 IMU 剧烈运动后定位飞掉

优先排查：

```text
1. 外置 IMU 与雷达外参是否准确；
2. IMU 时间戳是否稳定；
3. 外置 IMU 加速度单位是否为 m/s^2；
4. 运动是否超过 IMU 或 FASTLIO2 初始化假设；
5. 开始录制或启动时是否静止 5-10 秒。
```

当前外置 IMU 主配置默认不加 53 ms 时间补偿，因为实测效果更差。

## 16. 最小验收流程

新电脑部署完成后，按这个顺序验收：

```text
1. 启动 Livox 驱动，确认 /livox/lidar 和 /livox/imu 有频率。
2. 如果用外置 IMU，启动 IMU 驱动，确认 /imu/data 有频率。
3. 启动 FASTLIO2 内置 IMU 配置，看 RViz 是否连续建图。
4. 回放已有 rosbag，确认 /fastlio2/lio_odom 连续输出。
5. 如接底盘，打开 can0，启动 chassis_can，确认 /odom 输出。
6. 最后再启动 PGO 保存地图。
```

推荐先以内置 IMU 跑通整套 LiDAR-IMU 里程计，再切外置 IMU 调参。

