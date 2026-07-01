#!/usr/bin/env python3
"""Convert a ROS 2 Livox/IMU bag into a ROS 1 bag accepted by LI-Init.

This script fixes two things that generic conversion often gets wrong:
  * ROS 1 message MD5 sums, especially Header/Imu and Livox CustomMsg.
  * ROS 1 binary serialization instead of writing ROS 2 CDR bytes.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
from rosbags.rosbag1 import Writer
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore


def _register_livox_types(typestore, package: str, msg_dir: Path) -> None:
    types = {}
    types.update(
        get_types_from_msg(
            (msg_dir / "CustomPoint.msg").read_text(),
            f"{package}/msg/CustomPoint",
        )
    )
    types.update(
        get_types_from_msg(
            (msg_dir / "CustomMsg.msg").read_text(),
            f"{package}/msg/CustomMsg",
        )
    )
    typestore.register(types)


def _make_header_factory(ros1):
    header_cls = ros1.types["std_msgs/msg/Header"]
    time_cls = ros1.types["builtin_interfaces/msg/Time"]

    def make_header(src_header, seq: int):
        return header_cls(
            seq,
            time_cls(src_header.stamp.sec, src_header.stamp.nanosec),
            src_header.frame_id,
        )

    return make_header


def convert_bag(args: argparse.Namespace) -> None:
    input_bag = Path(args.input_bag).expanduser().resolve()
    output_bag = Path(args.output_bag).expanduser().resolve()
    msg_dir = Path(args.livox_msg_dir).expanduser().resolve()

    if not input_bag.exists():
        raise FileNotFoundError(input_bag)
    if not msg_dir.exists():
        raise FileNotFoundError(msg_dir)
    if output_bag.exists():
        if not args.force:
            raise FileExistsError(f"{output_bag} exists; pass --force to overwrite")
        output_bag.unlink()
    output_bag.parent.mkdir(parents=True, exist_ok=True)

    ros2 = get_typestore(Stores.ROS2_HUMBLE)
    _register_livox_types(ros2, args.source_livox_package, msg_dir)

    ros1 = get_typestore(Stores.ROS1_NOETIC)
    _register_livox_types(ros1, args.target_livox_package, msg_dir)

    make_header = _make_header_factory(ros1)
    imu_cls = ros1.types["sensor_msgs/msg/Imu"]
    quat_cls = ros1.types["geometry_msgs/msg/Quaternion"]
    vec3_cls = ros1.types["geometry_msgs/msg/Vector3"]
    livox_cls = ros1.types[f"{args.target_livox_package}/msg/CustomMsg"]

    counts = {args.lidar_topic: 0, args.imu_topic: 0}
    seqs = {args.lidar_topic: 0, args.imu_topic: 0}
    start_wall = time.monotonic()

    with Reader(input_bag) as reader, Writer(output_bag) as writer:
        source_connections = {
            c.topic: c
            for c in reader.connections
            if c.topic in (args.lidar_topic, args.imu_topic)
        }
        missing = [
            topic
            for topic in (args.lidar_topic, args.imu_topic)
            if topic not in source_connections
        ]
        if missing:
            raise RuntimeError(f"Missing required topics in ROS 2 bag: {missing}")

        out_connections = {
            args.lidar_topic: writer.add_connection(
                args.lidar_topic,
                f"{args.target_livox_package}/msg/CustomMsg",
                typestore=ros1,
            ),
            args.imu_topic: writer.add_connection(
                args.imu_topic,
                "sensor_msgs/msg/Imu",
                typestore=ros1,
            ),
        }

        read_connections = [source_connections[args.lidar_topic], source_connections[args.imu_topic]]
        for conn, timestamp, raw in reader.messages(connections=read_connections):
            src_msg = ros2.deserialize_cdr(raw, conn.msgtype)
            seq = seqs[conn.topic]
            seqs[conn.topic] += 1

            if conn.topic == args.imu_topic:
                dst_msg = imu_cls(
                    make_header(src_msg.header, seq),
                    quat_cls(
                        src_msg.orientation.x,
                        src_msg.orientation.y,
                        src_msg.orientation.z,
                        src_msg.orientation.w,
                    ),
                    src_msg.orientation_covariance,
                    vec3_cls(
                        src_msg.angular_velocity.x,
                        src_msg.angular_velocity.y,
                        src_msg.angular_velocity.z,
                    ),
                    src_msg.angular_velocity_covariance,
                    vec3_cls(
                        src_msg.linear_acceleration.x,
                        src_msg.linear_acceleration.y,
                        src_msg.linear_acceleration.z,
                    ),
                    src_msg.linear_acceleration_covariance,
                )
                dst_raw = ros1.serialize_ros1(dst_msg, "sensor_msgs/msg/Imu")
            else:
                # Keep source point objects to avoid copying tens of millions of points.
                dst_msg = livox_cls(
                    make_header(src_msg.header, seq),
                    src_msg.timebase,
                    src_msg.point_num,
                    src_msg.lidar_id,
                    np.asarray(src_msg.rsvd, dtype=np.uint8),
                    src_msg.points,
                )
                dst_raw = ros1.serialize_ros1(
                    dst_msg,
                    f"{args.target_livox_package}/msg/CustomMsg",
                )

            writer.write(out_connections[conn.topic], timestamp, dst_raw)
            counts[conn.topic] += 1

            total = counts[args.lidar_topic] + counts[args.imu_topic]
            if args.max_messages and total >= args.max_messages:
                break
            if args.progress_every > 0 and total % args.progress_every == 0:
                elapsed = time.monotonic() - start_wall
                print(
                    f"converted {total} messages "
                    f"(lidar={counts[args.lidar_topic]}, imu={counts[args.imu_topic]}) "
                    f"in {elapsed:.1f}s",
                    flush=True,
                )

    elapsed = time.monotonic() - start_wall
    print(f"written: {output_bag}")
    print(f"counts: lidar={counts[args.lidar_topic]}, imu={counts[args.imu_topic]}")
    print(f"elapsed_s: {elapsed:.1f}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_bag", help="ROS 2 bag directory")
    parser.add_argument("output_bag", help="Output ROS 1 .bag path")
    parser.add_argument("--lidar-topic", default="/livox/lidar")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--source-livox-package", default="livox_ros_driver2")
    parser.add_argument("--target-livox-package", default="livox_ros_driver")
    parser.add_argument(
        "--livox-msg-dir",
        default="third_party/livox_ros_driver/msg",
        help="Directory containing CustomMsg.msg and CustomPoint.msg",
    )
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--max-messages", type=int, default=0)
    parser.add_argument("--progress-every", type=int, default=5000)
    return parser.parse_args()


def main() -> int:
    convert_bag(parse_args())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
