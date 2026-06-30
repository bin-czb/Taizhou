# FASTLIO2 Mid360 Deployment Notes

This workspace is the clean deployment copy for the later navigation system:

```text
/home/czb/Tai Zhou/fastlio_ros2_ws
```

The original imported folder had spaces and parentheses in its path
(`fastlio-ws (copy)`), which breaks `ament_cmake_python` during build. Keep the
clean workspace path for this machine and future migration.

## What This System Contains

Packages:

```text
fastlio2    LiDAR-inertial odometry
pgo         loop closure and pose graph optimization
localizer   ICP relocalization against a saved map
hba         map optimization
interface   custom service definitions
```

The core runtime node is `fastlio2/lio_node`.

Inputs:

```text
/livox/lidar   livox_ros_driver2/msg/CustomMsg
/livox/imu     sensor_msgs/msg/Imu, optional Livox built-in IMU
/imu/data      sensor_msgs/msg/Imu, optional external HWT905 RS485 IMU
```

Main outputs:

```text
/fastlio2/lio_odom
/fastlio2/lio_path
/fastlio2/body_cloud
/fastlio2/world_cloud
TF: map -> body
```

It does not publish `/odom` directly. If a downstream navigation stack needs
`/odom`, remap `/fastlio2/lio_odom` or add a wrapper after frame conventions are
fixed.

## Chassis And Odometry Code Search

This repository does not contain low-level chassis communication code. I found no
`cmd_vel`, serial, CAN, SocketCAN, motor, Modbus, or controller driver code.

It does contain an odometry wrapper/fusion script:

```text
fastlio2/launch/odom_fusion_node.py
```

That script subscribes to `/fastlio2/lio_odom` and `/odom`, looks up
`body <- base_link`, and publishes:

```text
/fused_odom
/fused_path
/odom_path
/lio_path
/lio_base_odom
```

So `/odom` in this project is expected to come from another system, usually a
wheel odometry or chassis driver. It is not generated here.

## Dependencies

Already present on this machine:

```text
ROS2 Humble
PCL
GTSAM
livox_ros_driver2 from /home/czb/Tai Zhou/fast_livo2_ros2_ws
Sophus 1.22.10 in /usr/local/share/sophus/cmake
```

Build status on this machine:

```text
interface   built
fastlio2    built
pgo         built
localizer   built
hba         built
```

Sophus installation command, only needed on a fresh machine:

```bash
cd /tmp
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.22.10
mkdir -p build
cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make -j$(nproc)
sudo make install
```

Then build:

```bash
cd "/home/czb/Tai Zhou/fastlio_ros2_ws"
conda deactivate 2>/dev/null || true
unset PYTHONPATH PYTHONHOME
export PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
source "/home/czb/Tai Zhou/fast_livo2_ros2_ws/install/setup.bash"
colcon --log-base log_clean build \
  --symlink-install \
  --build-base build_clean \
  --install-base install_clean \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install_clean/setup.bash
```

Build all map/localization modules later:

```bash
colcon --log-base log_clean build \
  --symlink-install \
  --build-base build_clean \
  --install-base install_clean \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

Why the `build_clean/install_clean` paths are used: the first local build was
accidentally configured with Anaconda Python 3.13, which generated stale
`cpython-313` build rules. A clean build base avoids that without deleting old
files.

## Sensor Configuration Choice

Two FASTLIO2 configs were added.

Livox built-in IMU:

```text
fastlio2/config/lio_mid360_builtin_imu.yaml
```

Use this only after `/livox/imu` is confirmed to publish real samples. The
Livox ROS driver comments show Livox IMU acceleration is in `g`, so this config
uses:

```yaml
imu_acc_scale: 9.80665
```

External HWT905 RS485 IMU:

```text
fastlio2/config/lio_mid360_hwt905_external_imu.yaml
```

Use this when running with `/imu/data`. The HWT905 driver publishes acceleration
in `m/s^2`, so this config uses:

```yaml
imu_acc_scale: 1.0
```

The external IMU extrinsic is:

```text
p_imu = r_il * p_lidar + t_il
t_il = [0.0, 0.0, 0.08]
```

Rotation and translation come from:

```text
/home/czb/Tai Zhou/fast_livo2_ros2_ws/calibration/recommended_lidar_imu_extrinsic_20260622.yaml
```

## Start Sensors

Terminal 1, Mid360:

```bash
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

Terminal 2, external HWT905 IMU if used:

```bash
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fast_livo2_ros2 hwt601_485_imu.launch.py \
  port:=/dev/ttyUSB0 baudrate:=115200 address:=80 rate_hz:=150.0 accel_range_g:=4.0
```

Check topics:

```bash
cd "/home/czb/Tai Zhou/fastlio_ros2_ws"
source /opt/ros/humble/setup.bash
source "/home/czb/Tai Zhou/fast_livo2_ros2_ws/install/setup.bash"
source install/setup.bash
ros2 run fastlio2 check_fastlio_topics.sh
```

## Start FASTLIO2

Built-in IMU:

```bash
cd "/home/czb/Tai Zhou/fastlio_ros2_ws"
source /opt/ros/humble/setup.bash
source "/home/czb/Tai Zhou/fast_livo2_ros2_ws/install/setup.bash"
source install/setup.bash
ros2 launch fastlio2 lio_launch.py \
  config_path:="$(ros2 pkg prefix fastlio2)/share/fastlio2/config/lio_mid360_builtin_imu.yaml"
```

External HWT905 IMU:

```bash
cd "/home/czb/Tai Zhou/fastlio_ros2_ws"
source /opt/ros/humble/setup.bash
source "/home/czb/Tai Zhou/fast_livo2_ros2_ws/install/setup.bash"
source install/setup.bash
ros2 launch fastlio2 lio_launch.py \
  config_path:="$(ros2 pkg prefix fastlio2)/share/fastlio2/config/lio_mid360_hwt905_external_imu.yaml"
```

For headless testing:

```bash
ros2 launch fastlio2 lio_launch.py use_rviz:=false
```

## Record A Validation Bag

Record raw data and FASTLIO2 outputs:

```bash
cd "/home/czb/Tai Zhou/fastlio_ros2_ws"
source /opt/ros/humble/setup.bash
source "/home/czb/Tai Zhou/fast_livo2_ros2_ws/install/setup.bash"
source install/setup.bash
ros2 run fastlio2 record_fastlio_validation_bag.sh
```

The bag is saved under:

```text
~/fastlio_validation_bags/<session_name>
```

For later algorithm replay, the minimum useful raw topics are:

```text
/livox/lidar
/livox/imu or /imu/data
/tf
/tf_static
```

For this vehicle, record both IMUs for now until `/livox/imu` quality is
confirmed:

```text
/livox/imu
/imu/data
```

## First Acceptance Test

1. Start LiDAR and chosen IMU.
2. Confirm topic rates:
   - `/livox/lidar`: about 10 Hz
   - `/livox/imu`: expected high rate if built-in IMU is enabled
   - `/imu/data`: about 150 Hz for HWT905
3. Start FASTLIO2 while the rig is still for 5-10 seconds.
4. Move slowly in a feature-rich corridor.
5. Check:
   - `/fastlio2/lio_odom` is continuous;
   - `/fastlio2/world_cloud` is not obviously doubled or torn;
   - the path handles a 90 degree turn without sudden flips.

If motion causes ghosting, first check time offset and IMU choice before tuning
map parameters.
