# FAST-LIVO2 Data Acquisition Workspace

This workspace is for ROS2 Humble sensor bring-up, calibration management, and
FAST-LIVO2-compatible rosbag recording.

Current boundary: this workspace does not run the final FAST-LIVO2 localization
algorithm. It records raw camera, Livox, and IMU data with the calibration files
needed by the downstream localization/navigation system.

## Current Sensor Contract

| Sensor | Topic | ROS2 type | Expected rate |
| --- | --- | --- | --- |
| USB RGB camera | `/left_camera/image` | `sensor_msgs/msg/Image` | 30 Hz |
| Livox Mid360 | `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | 10 Hz |
| Livox built-in IMU | `/livox/imu` | `sensor_msgs/msg/Imu` | driver dependent |
| HWT905/HWT601 RS485 IMU | `/imu/data` | `sensor_msgs/msg/Imu` | 150 Hz |

Use Livox `CustomMsg` for recording. It preserves per-point `offset_time`,
which is required for proper LiDAR motion compensation.

## Calibration In Use

Main config:

```text
src/fast_livo2_ros2/config/mid360_rgb_apriltag.yaml
src/fast_livo2_ros2/config/camera_pinhole_rgb.yaml
src/fast_livo2_ros2/config/current_calibration_summary.yaml
```

Camera intrinsics currently selected:

```text
resolution: 1280 x 720
fx fy cx cy: 715.799008 708.732198 647.586881 342.577746
k1 k2 p1 p2: -0.054322996 0.090084176 -0.004535688 0.004554663
source: calibration/camera_intrinsic/checkerboard_20260618_190320
```

LiDAR to camera, manual first-pass value:

```yaml
Rcl: [0.0, -1.0,  0.0,
      0.0,  0.0, -1.0,
      1.0,  0.0,  0.0]
Pcl: [0.0, -0.06, -0.065]
```

Convention:

```text
p_camera = Rcl * p_lidar + Pcl
```

LiDAR to IMU, current recommendation:

```yaml
extrinsic_R: [0.009167, -0.999957, 0.001393,
              0.999900,  0.009182, 0.010762,
             -0.010774,  0.001294, 0.999941]
extrinsic_T: [0.0, 0.0, 0.08]
```

Convention:

```text
p_imu = extrinsic_R * p_lidar + extrinsic_T
```

Why this combination:

- Rotation comes from official LI-Init on the 150 Hz IMU bag.
- Translation uses mechanical measurement because LI-Init estimated
  `z=0.124-0.131 m`, larger than the measured top-of-LiDAR to bottom-of-IMU
  distance and therefore not trusted as final translation.
- LI-Init estimated `Time Lag IMU to LiDAR = -0.053331 s`. The current
  FASTLIO2 external-IMU config applies it as `imu_time_lag_to_lidar_s:
  -0.053331`, using `imu_timestamp -= imu_time_lag_to_lidar_s`.

Detailed records:

```text
calibration/li_init_lidar_imu_result_20260622_150hz.yaml
calibration/recommended_lidar_imu_extrinsic_20260622.yaml
calibration/manual_lidar_camera_extrinsic.yaml
calibration/manual_lidar_imu_extrinsic.yaml
```

## Environment Setup On A New Computer

Use Ubuntu 22.04 with ROS2 Humble.

Install common dependencies:

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git python3-pip python3-serial python3-yaml \
  python3-numpy python3-opencv v4l-utils \
  ros-humble-desktop ros-humble-image-tools ros-humble-rviz2 \
  ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins
```

Install or copy Livox-SDK2:

```bash
cd ~/Livox-SDK2
mkdir -p build
cd build
cmake ..
make -j
sudo make install
sudo ldconfig
```

Copy this workspace to the new computer, then build:

```bash
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
```

Avoid Conda while running ROS2:

```bash
conda deactivate 2>/dev/null || true
unset PYTHONPATH
unset PYTHONHOME
unset AMENT_TRACE_SETUP_FILES
export PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
```

## Sensor Environment

### Livox Mid360

Current network:

```text
PC Ethernet IP: 192.168.1.50/24
Mid360 IP:      192.168.1.183
interface:      enp8s0
```

Set the PC wired IP if needed:

```bash
sudo nmcli connection modify '有线连接 1' \
  ipv4.method manual ipv4.addresses 192.168.1.50/24 ipv4.gateway 192.168.1.1
sudo nmcli connection up '有线连接 1'
```

Check link:

```bash
ros2 run fast_livo2_ros2 check_livox_mid360_link.sh enp8s0 192.168.1.50
ping -I enp8s0 192.168.1.183
```

Start LiDAR:

```bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

Expected:

```bash
ros2 topic hz /livox/lidar
ros2 topic echo --once /livox/lidar
```

### USB RGB Camera

Find the camera id:

```bash
v4l2-ctl --list-devices
```

Start with the calibrated resolution:

```bash
ros2 run fast_livo2_ros2 start_usb_camera.sh 2 1280 720 30.0 /left_camera/image
```

If the device id changes, replace `2`.

### RS485 IMU

Current settings:

```text
port: /dev/ttyUSB0
baudrate: 115200
address: 80 decimal, 0x50 hex
rate: 150 Hz
accel range: 4 g
```

Start IMU:

```bash
ros2 launch fast_livo2_ros2 hwt601_485_imu.launch.py \
  port:=/dev/ttyUSB0 \
  baudrate:=115200 \
  address:=80 \
  rate_hz:=150.0 \
  accel_range_g:=4.0
```

If the port or address changes:

```bash
ros2 run fast_livo2_ros2 probe_hwt601_485.py
```

## Field Recording For Navigation Validation

Use four terminals. In every terminal:

```bash
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Terminal 1, LiDAR:

```bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

Terminal 2, camera:

```bash
ros2 run fast_livo2_ros2 start_usb_camera.sh 2 1280 720 30.0 /left_camera/image
```

Terminal 3, IMU:

```bash
ros2 launch fast_livo2_ros2 hwt601_485_imu.launch.py \
  port:=/dev/ttyUSB0 baudrate:=115200 address:=80 rate_hz:=150.0 accel_range_g:=4.0
```

Terminal 4, preflight and record:

```bash
ros2 run fast_livo2_ros2 check_record_topics.py

ros2 run fast_livo2_ros2 record_fast_livo_site_bag.sh \
  "$HOME/fast_livo2_validation_bags" \
  "site_nav_$(date +%Y%m%d_%H%M%S)"
```

The recording script writes:

```text
~/fast_livo2_validation_bags/site_nav_YYYYMMDD_HHMMSS/
~/fast_livo2_validation_bags/site_nav_YYYYMMDD_HHMMSS_calibration_snapshot/
```

The snapshot folder contains the camera and extrinsic config used for that bag.
It copies all `src/fast_livo2_ros2/config/*.yaml` files from the installed
package, including `current_calibration_summary.yaml`.

Field motion guide:

```text
1. Keep the rig still for 5-10 seconds at the beginning.
2. Traverse the real navigation route at validation speed.
3. Include turns, stops, and at least one loop if the site allows it.
4. Avoid violent motion until time synchronization is verified.
5. Record enough data for the downstream system: usually 2-10 minutes per route.
```

Verify after recording:

```bash
ros2 bag info "$HOME/fast_livo2_validation_bags/site_nav_YYYYMMDD_HHMMSS"
```

The bag must contain:

```text
/left_camera/image
/livox/lidar
/livox/imu
/imu/data
```

Optional:

```text
/tf
/tf_static
```

## Important Validation Notes

- Different rates are normal: LiDAR 10 Hz, IMU 150 Hz, camera 30 Hz.
- Different rates do not mean the sensors are synchronized.
- The current bag showed IMU messages recorded almost immediately after their
  header stamps, while LiDAR messages were recorded about 105 ms after header
  stamps. This is consistent with a completed 10 Hz scan being published after
  scan end while its header stamp is near scan start.
- The current FASTLIO2 external-IMU config applies LI-Init's `-0.053331 s`
  time lag. Repeat LI-Init on one or two more bags before treating it as final.
- For downstream FAST-LIVO validation, first try the recommended LiDAR-IMU
  extrinsic above. If maps show motion-direction ghosting, test `T_li_z` values
  `0.08`, `0.10`, and `0.11 m` before accepting the LI-Init `0.124-0.131 m`
  translation.

## Useful Documentation

```text
src/fast_livo2_ros2/README.md
src/fast_livo2_ros2/docs/data_recording.md
src/fast_livo2_ros2/docs/livox_mid360_bringup.md
src/fast_livo2_ros2/docs/hwt601_485_imu_bringup.md
src/fast_livo2_ros2/docs/camera_intrinsic_calibration.md
src/fast_livo2_ros2/docs/imu_calibration.md
src/fast_livo2_ros2/docs/li_init_workflow.md
src/fast_livo2_ros2/docs/system_integration.md
```
