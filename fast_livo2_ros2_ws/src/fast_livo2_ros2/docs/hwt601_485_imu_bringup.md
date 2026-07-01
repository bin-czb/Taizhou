# HWT601-AGV-485 IMU Bringup

This sensor is treated as a likely HWT601/WitMotion-compatible RS485 Modbus
IMU/inclinometer until the exact datasheet is available.

## Wiring

- Use a USB-RS485 adapter, not a normal USB-TTL adapter.
- Connect sensor `A/+` to adapter `A/+`, and sensor `B/-` to adapter `B/-`.
- If there is no response, swap A and B once.
- Power the sensor from the voltage printed on its label or manual.
- Linux should expose the adapter as `/dev/ttyUSB*` or `/dev/serial/by-id/*`.

## Probe

```bash
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run fast_livo2_ros2 probe_hwt601_485.py
```

If the port is known:

```bash
ros2 run fast_livo2_ros2 probe_hwt601_485.py \
  --ports /dev/ttyUSB0 \
  --baudrates 9600,115200,57600,38400,19200 \
  --addresses 0x50,0x01,0x02,0x03,0x04,0x05 \
  --accel-range-g 4
```

Expected success line:

```text
FOUND port=/dev/ttyUSB0 baud=115200 address=0x50 ...
```

## Publish `/imu/data`

Use the `FOUND` port, baudrate, and address:

```bash
ros2 launch fast_livo2_ros2 hwt601_485_imu.launch.py \
  port:=/dev/ttyUSB0 \
  baudrate:=115200 \
  address:=80 \
  rate_hz:=150.0 \
  accel_range_g:=4.0
```

`address:=80` is decimal for `0x50`.

Verify:

```bash
ros2 topic hz /imu/data
ros2 topic echo --once /imu/data
```

For the current HWT905/HWT601-compatible RS485 unit, probing found `/dev/ttyUSB0`,
`115200`, and address `0x50`. The current validation-recording target is 150 Hz.
At rest, acceleration magnitude should be close to
`9.8 m/s^2`, and angular velocity should be close to zero. If the acceleration
magnitude is near `39.2 m/s^2`, the driver is using `16g`; set
`accel_range_g:=4.0`.

## Notes For FAST-LIVO2

This quick driver is for bringing up the data stream. Before formal
localization, confirm:

- exact protocol and scaling from the vendor datasheet;
- sensor axis direction relative to LiDAR;
- IMU-LiDAR extrinsic in `extrinsic_R/extrinsic_T`;
- time offset/synchronization.
