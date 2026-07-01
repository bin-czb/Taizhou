#!/usr/bin/python3
"""Check the raw sensor topics required for validation rosbag recording."""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node


class TopicChecker(Node):
    def __init__(self, required_topics: dict[str, str]) -> None:
        super().__init__("record_topic_checker")
        self.required_topics = required_topics

    def wait_for_discovery(self, timeout_sec: float) -> dict[str, list[str]]:
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        found: dict[str, list[str]] = {}

        while True:
            rclpy.spin_once(self, timeout_sec=0.1)
            found = dict(self.get_topic_names_and_types())
            if all(expected in found.get(topic, []) for topic, expected in self.required_topics.items()):
                return found
            if time.monotonic() >= deadline:
                return found

    def run(self, timeout_sec: float) -> int:
        found = self.wait_for_discovery(timeout_sec)
        ok = True
        for topic, expected_type in self.required_topics.items():
            types = found.get(topic, [])
            if expected_type in types:
                self.get_logger().info(f"OK   {topic}: {expected_type}")
                continue
            ok = False
            if types:
                self.get_logger().error(
                    f"BAD  {topic}: expected {expected_type}, observed {', '.join(types)}"
                )
            else:
                self.get_logger().error(f"MISS {topic}: expected {expected_type}")
        return 0 if ok else 2


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image-topic", default="/left_camera/image")
    parser.add_argument("--lidar-topic", default="/livox/lidar")
    parser.add_argument(
        "--lidar-type",
        default="livox_ros_driver2/msg/CustomMsg",
        choices=["livox_ros_driver2/msg/CustomMsg", "sensor_msgs/msg/PointCloud2"],
    )
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    args = parser.parse_args()

    required_topics = {
        args.image_topic: "sensor_msgs/msg/Image",
        args.lidar_topic: args.lidar_type,
        args.imu_topic: "sensor_msgs/msg/Imu",
    }

    rclpy.init()
    node = TopicChecker(required_topics)
    try:
        return node.run(args.timeout_sec)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
