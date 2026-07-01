# IMU Calibration And Verification

This workspace uses the HWT601-AGV-485 / RS485 IMU as the external IMU stream
on `/imu/data`. The current job is data quality and rosbag readiness; formal
FAST-LIVO2 localization still needs LiDAR-IMU extrinsic and timing validation.

## What FAST-LIVO2 Expects

FAST-LIVO2 reads these IMU-related fields from its YAML files:

- `common/imu_topic`
- `extrin_calib/extrinsic_T`
- `extrin_calib/extrinsic_R`
- `time_offset/imu_time_offset`
- `imu/imu_en`
- `imu/imu_int_frame`
- `imu/acc_cov`, `imu/gyr_cov`
- `imu/b_acc_cov`, `imu/b_gyr_cov`

In the official IMU processor these extrinsics are stored as
`Lid_rot_to_IMU` and `Lid_offset_to_IMU`, so treat `extrinsic_R/T` as the
LiDAR-to-IMU transform.

FAST-LIVO2 itself does not provide a standalone IMU calibration utility. For
rigorous LiDAR-IMU extrinsic and time-offset calibration, use HKU-MARS
LiDAR_IMU_Init (LI-Init) or an equivalent LiDAR-IMU calibration pipeline.

## Step 1: Probe The RS485 IMU

```bash
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run fast_livo2_ros2 probe_hwt601_485.py \
  --ports /dev/ttyUSB0 \
  --baudrates 9600,115200,57600,38400,19200 \
  --addresses 0x50,0x01,0x02,0x03,0x04,0x05 \
  --accel-range-g 4
```

Current known values for this unit:

- port: `/dev/ttyUSB0`
- baudrate: `115200`
- address: `0x50` / decimal `80`
- acceleration range: `4 g`

## Step 2: Publish `/imu/data`

```bash
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch fast_livo2_ros2 hwt601_485_imu.launch.py \
  port:=/dev/ttyUSB0 \
  baudrate:=115200 \
  address:=80 \
  rate_hz:=150.0 \
  accel_range_g:=4.0
```

Verify in another terminal:

```bash
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic hz /imu/data
ros2 topic echo --once /imu/data
```

Acceptance checks:

- frequency is stable near the configured rate;
- `linear_acceleration` is in `m/s^2`;
- `angular_velocity` is in `rad/s`;
- static acceleration norm is close to `9.80665 m/s^2`;
- static angular velocity is close to zero.

## Step 3: Static Bias And Noise Check

Keep the whole sensor rig completely still for 60 seconds:

```bash
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run fast_livo2_ros2 analyze_imu_static.py \
  --imu-topic /imu/data \
  --duration 60 \
  --output calibration/imu_static_analysis.yaml
```

The output records:

- sample count and measured rate;
- acceleration mean/std;
- acceleration norm mean/std;
- gyro mean/std;
- a practical static gyro-bias estimate.

If `accel_norm_mean_mps2` is around `39.2`, the driver is using the wrong
acceleration range. Use `accel_range_g:=4.0` for the current unit.

## Step 4: Axis Direction Check

Do this before writing IMU-LiDAR extrinsics:

1. Place the rig still with one IMU face upward.
2. Read `/imu/data` and record which acceleration axis is about `+9.8` or
   `-9.8`.
3. Repeat with another face upward.
4. Slowly rotate the rig around each IMU axis and check the sign of
   `angular_velocity`.

This tells you the IMU coordinate frame relative to the LiDAR frame. Without
that axis mapping, `extrinsic_R` is guesswork.

## Step 5: LiDAR-IMU Extrinsic And Time Offset

Current axis check:

```text
IMU +Y is aligned with the camera/LiDAR forward direction.
IMU +X points to the camera-right direction.
IMU is mounted below the LiDAR by about 80 mm by mechanical measurement.
```

Current recommended first-pass LiDAR-to-IMU extrinsic:

```yaml
extrinsic_R: [0.009167, -0.999957, 0.001393,
              0.999900,  0.009182, 0.010762,
             -0.010774,  0.001294, 0.999941]
extrinsic_T: [0.0, 0.0, 0.08]
```

This uses LI-Init rotation and mechanical translation. LI-Init estimated
translation `z=0.124-0.131 m`, but that is larger than the measured
top-of-LiDAR to bottom-of-IMU distance, so it is not trusted as the final
translation.

Time offset:

```yaml
imu_time_offset: 0.0
```

LI-Init estimated `Time Lag IMU to LiDAR = -0.053331 s` once. Keep that as a
candidate until repeated bags confirm it and the downstream sign convention is
checked.

Recommended route for further refinement:

1. Record LiDAR and IMU together.
2. Start with the rig still for more than 5 seconds.
3. Add roll, pitch, yaw, forward/back, left/right, and up/down excitation for
   60-120 seconds.
4. Run LI-Init with `mean_acc_norm` near `9.805` for this external IMU.
5. Compare LI-Init against mechanical limits before writing the result into the
   downstream FAST-LIVO2 YAML.

## Step 6: Record A Calibration Bag

Use the Livox message format required by the downstream system. For FAST-LIVO2,
prefer Livox `CustomMsg` because it preserves per-point time.

```bash
cd "/home/czb/Tai Zhou/fast_livo2_ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 bag record /livox/lidar /imu/data \
  -o calibration/lidar_imu_$(date +%Y%m%d_%H%M%S)
```

For the final validation bag, record all three sensors:

```bash
ros2 run fast_livo2_ros2 record_fast_livo_site_bag.sh
```
