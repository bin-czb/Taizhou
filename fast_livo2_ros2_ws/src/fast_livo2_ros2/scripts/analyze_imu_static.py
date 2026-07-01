#!/usr/bin/python3
"""Collect static IMU samples and report bias/noise sanity checks."""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import Imu


G = 9.80665


class StaticImuAnalyzer(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("static_imu_analyzer")
        self.args = args
        self.start_time: Optional[float] = None
        self.samples: list[tuple[float, np.ndarray, np.ndarray]] = []
        qos = rclpy.qos.qos_profile_sensor_data
        self.create_subscription(Imu, args.imu_topic, self._imu_cb, qos)

    def _imu_cb(self, msg: Imu) -> None:
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1.0e-9
        if stamp <= 0.0:
            stamp = time.time()
        if self.start_time is None:
            self.start_time = stamp
        accel = np.array(
            [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ],
            dtype=np.float64,
        )
        gyro = np.array(
            [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ],
            dtype=np.float64,
        )
        self.samples.append((stamp, accel, gyro))

    def elapsed(self) -> float:
        if self.start_time is None or not self.samples:
            return 0.0
        return self.samples[-1][0] - self.start_time


def _format_vec(vec: np.ndarray, precision: int = 6) -> list[float]:
    return [round(float(v), precision) for v in vec.reshape(-1)]


def _analyze(samples: list[tuple[float, np.ndarray, np.ndarray]]) -> dict:
    stamps = np.array([item[0] for item in samples], dtype=np.float64)
    accels = np.vstack([item[1] for item in samples])
    gyros = np.vstack([item[2] for item in samples])
    duration = float(stamps[-1] - stamps[0]) if len(stamps) > 1 else 0.0
    hz = float((len(stamps) - 1) / duration) if duration > 0.0 and len(stamps) > 1 else 0.0

    accel_mean = accels.mean(axis=0)
    gyro_mean = gyros.mean(axis=0)
    accel_std = accels.std(axis=0, ddof=1) if len(samples) > 1 else np.zeros(3)
    gyro_std = gyros.std(axis=0, ddof=1) if len(samples) > 1 else np.zeros(3)
    accel_norm = np.linalg.norm(accels, axis=1)
    gravity_axis = int(np.argmax(np.abs(accel_mean)))

    return {
        "samples": int(len(samples)),
        "duration_s": duration,
        "rate_hz": hz,
        "accel_mean_mps2": accel_mean,
        "accel_std_mps2": accel_std,
        "accel_norm_mean_mps2": float(accel_norm.mean()),
        "accel_norm_std_mps2": float(accel_norm.std(ddof=1)) if len(samples) > 1 else 0.0,
        "gyro_mean_radps": gyro_mean,
        "gyro_std_radps": gyro_std,
        "gyro_mean_degps": np.rad2deg(gyro_mean),
        "gyro_std_degps": np.rad2deg(gyro_std),
        "gravity_axis_index": gravity_axis,
        "gravity_axis_name": ["x", "y", "z"][gravity_axis],
        "gravity_axis_sign": float(np.sign(accel_mean[gravity_axis])),
        "accel_norm_error_mps2": float(accel_norm.mean() - G),
    }


def _write_yaml(path: Path, args: argparse.Namespace, result: dict) -> None:
    output = {
        "method": "static_imu_analysis",
        "imu_topic": args.imu_topic,
        "samples": result["samples"],
        "duration_s": round(result["duration_s"], 6),
        "rate_hz": round(result["rate_hz"], 3),
        "accel_mean_mps2": _format_vec(result["accel_mean_mps2"]),
        "accel_std_mps2": _format_vec(result["accel_std_mps2"]),
        "accel_norm_mean_mps2": round(result["accel_norm_mean_mps2"], 6),
        "accel_norm_std_mps2": round(result["accel_norm_std_mps2"], 6),
        "accel_norm_error_mps2": round(result["accel_norm_error_mps2"], 6),
        "gyro_mean_radps": _format_vec(result["gyro_mean_radps"]),
        "gyro_std_radps": _format_vec(result["gyro_std_radps"]),
        "gyro_mean_degps": _format_vec(result["gyro_mean_degps"]),
        "gyro_std_degps": _format_vec(result["gyro_std_degps"]),
        "gravity_axis": {
            "axis": result["gravity_axis_name"],
            "sign": result["gravity_axis_sign"],
        },
        "notes": [
            "Keep the IMU completely static while collecting this file.",
            "gyro_mean_radps is a useful static gyro bias estimate.",
            "accel_norm_mean_mps2 should be close to 9.80665 for a normal external IMU.",
        ],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(output, sort_keys=False), encoding="utf-8")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--min-samples", type=int, default=200)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("calibration/imu_static_analysis.yaml"),
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    rclpy.init()
    node = StaticImuAnalyzer(args)
    print(f"Collecting static IMU samples from {args.imu_topic} for {args.duration:.1f} s")
    print("Keep the IMU completely still.")
    try:
        while rclpy.ok() and node.elapsed() < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.samples and len(node.samples) % 100 == 0:
                print(f"samples={len(node.samples)} elapsed={node.elapsed():.1f} s", end="\r")
        print()
        if len(node.samples) < args.min_samples:
            print(f"ERROR: only {len(node.samples)} samples collected < {args.min_samples}")
            return 2
        result = _analyze(node.samples)
        _write_yaml(args.output, args, result)
        print(f"samples: {result['samples']}")
        print(f"rate_hz: {result['rate_hz']:.3f}")
        print(f"accel_mean_mps2: {_format_vec(result['accel_mean_mps2'])}")
        print(f"accel_norm_mean_mps2: {result['accel_norm_mean_mps2']:.6f}")
        print(f"accel_norm_error_mps2: {result['accel_norm_error_mps2']:.6f}")
        print(f"gyro_mean_radps: {_format_vec(result['gyro_mean_radps'])}")
        print(f"gyro_mean_degps: {_format_vec(result['gyro_mean_degps'])}")
        print(
            f"gravity_axis: {result['gravity_axis_name']} "
            f"sign={result['gravity_axis_sign']:+.0f}"
        )
        print(f"written: {args.output}")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
