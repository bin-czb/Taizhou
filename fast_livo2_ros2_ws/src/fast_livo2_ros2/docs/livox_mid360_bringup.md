# Livox Mid360 Bring-Up

This workspace contains a migrated copy of `livox_ros_driver2` from:

```text
/home/czb/pythonProject01/ws_livox/src/livox_ros_driver2
```

The Livox SDK2 library is already available system-wide:

```text
/usr/local/lib/liblivox_lidar_sdk_shared.so
/usr/local/include/livox_lidar_api.h
```

The source SDK remains at:

```text
/home/czb/Livox-SDK2
```

## Build

Build from this workspace only:

```bash
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
```

Do not source `/home/czb/pythonProject01/ws_livox/install/setup.bash` for this workspace anymore.

## Packages

Expected packages after build:

```text
livox_ros_driver2
fast_livo2_ros2
```

Check:

```bash
colcon list
ros2 pkg prefix livox_ros_driver2
ros2 interface show livox_ros_driver2/msg/CustomMsg
```

## Mid360 Network Config

The default Mid360 ROS2 config is set for the current sensor found on the wired link:

```text
src/livox_ros_driver2/config/MID360_config.json
```

Important fields:

```json
"host_net_info": {
"host_ip": "192.168.1.50",
"lidar_ip": ["192.168.1.183"]
}
```

Current sensor:

```text
computer Ethernet IP: 192.168.1.50/24
current Mid-360 IP:   192.168.1.183
sensor MAC observed:  88:29:85:8a:dc:82
```

This matches the quick-start rule `192.168.1.1XX`, so the current sensor SN likely ends with `83`.

The previous fixed-IP profile was moved to:

```text
src/livox_ros_driver2/config/MID360_legacy_192_168_10_153_config.json
```

Use the legacy file only if you intentionally reconnect the older unit.

For direct official SDK testing without ROS2, use:

```text
src/livox_ros_driver2/config/MID360_sdk_current_192_168_1_183.json
```

## Launch

The current sensor reports SDK device type `35`, which maps to the SDK2 `Mid360s` branch even though the product is still reported as Mid-360. Use the Mid360s launch file for this unit:

```bash
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/livox_launch_logs ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

The launch file uses:

```text
xfer_format = 1
frame_id = livox_frame
publish_freq = 10.0
```

`xfer_format = 1` publishes Livox custom point clouds, which is the preferred input for FAST-LIVO2 because it preserves per-point `offset_time`.

For this sensor, `msg_MID360_launch.py` can discover the device at SDK level, but the ROS driver path that completes configuration and reaches publisher creation is the `msg_MID360s_launch.py` path.

## Expected Topics

After the LiDAR is connected and configured:

```bash
ros2 topic list
ros2 topic hz /livox/lidar
ros2 topic echo --once /livox/lidar
```

FAST-LIVO2 expects:

```text
/livox/lidar  livox_ros_driver2/msg/CustomMsg
```

If the driver also publishes Livox IMU, treat it as a diagnostic stream for now. The current FAST-LIVO2 plan uses an external IMU on `/imu/data`.

## Network Bring-Up Checklist

The current Mid360 profile uses:

```text
computer Ethernet IP: 192.168.1.50/24
Mid360 IP:            192.168.1.183
interface:            enp8s0
```

The wired connection named `有线连接 1` has been configured as a static Livox profile:

```bash
nmcli connection show '有线连接 1'
```

Useful checks:

```bash
ros2 run fast_livo2_ros2 check_livox_mid360_link.sh enp8s0 192.168.1.50
ip -br addr
nmcli -f GENERAL,WIRED-PROPERTIES,IP4 device show enp8s0
ip route show table main
ip neigh show dev enp8s0
```

Expected healthy signs:

```text
enp8s0 has 192.168.1.50/24
192.168.1.0/24 route points to enp8s0 without linkdown
WIRED-PROPERTIES.CARRIER is on
the neighbor table shows 192.168.1.183 with MAC 88:29:85:8a:dc:82
ping -I enp8s0 192.168.1.183 succeeds
```

If `ros2 launch livox_ros_driver2 msg_MID360_launch.py` prints `Init lds lidar success!` but `/livox/lidar` is not published, check the physical Ethernet/PoE/power path first.

## Current Status

The driver builds in this workspace without depending on the old `ws_livox` install space. The active config has been cleaned for the new Mid360 and no longer contains the previous unit's fixed IP.

Runtime validation on 2026-06-18:

```text
Livox ROS driver:        starts
Livox SDK init:          success
ROS config mode:         current fixed sensor IP 192.168.1.183
Official SDK quickstart: detects sensor
Sensor SN:               ARMCNCQ0037083
SDK dev_type:            35
enp8s0 carrier:          on
enp8s0 speed/duplex:     100 Mbps full duplex
ping current sensor:     192.168.1.183 reachable
```

Manual cross-check from `Livox_Mid-360_Quick_Start_Guide_multi.pdf`:

```text
PC static IP:       192.168.1.50/24
Factory sensor IP:  192.168.1.1XX, where XX is the last two digits of the SN
Current sensor SN:  ARMCNCQ0037083, so factory IP is 192.168.1.183
Power input:        9-27 V DC on the external power branch, red positive and black negative
RJ45 branch:        Ethernet only, do not connect any PoE device to the RJ45 connector
M12 connector:      tighten the female lock nut until it fully mates with the male connector face, with no gap
Data port:          100 BASE-TX Ethernet
```

The ROS2 driver CMake was patched to link the generated `livox_ros_driver2__rosidl_typesupport_cpp` library. Without this, the driver can connect to the sensor but crashes when creating the `/livox/lidar` publisher for `livox_ros_driver2/msg/CustomMsg`.
