#!/usr/bin/env python3
"""Compute FAST-LIVO2 Rcl/Pcl from AprilTag-camera and LiDAR-board poses.

Input YAML format:

scenes:
  - name: scene_01
    T_camera_board:
      rotation: [1, 0, 0, 0, 1, 0, 0, 0, 1]
      translation: [0.0, 0.0, 2.0]
    T_lidar_board:
      rotation: [1, 0, 0, 0, 1, 0, 0, 0, 1]
      translation: [0.1, 0.0, 2.1]

The script assumes both transforms map board-frame points into the named sensor frame:
  p_camera = T_camera_board * p_board
  p_lidar = T_lidar_board * p_board

It outputs LiDAR -> Camera:
  p_camera = Rcl * p_lidar + Pcl
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml


def _as_rotation(values: Any) -> np.ndarray:
    rot = np.asarray(values, dtype=float)
    if rot.shape == (9,):
        rot = rot.reshape(3, 3)
    if rot.shape != (3, 3):
        raise ValueError("rotation must contain 9 row-major values or a 3x3 matrix")
    return rot


def _as_translation(values: Any) -> np.ndarray:
    trans = np.asarray(values, dtype=float).reshape(-1)
    if trans.shape != (3,):
        raise ValueError("translation must contain 3 values")
    return trans


def _transform_from_dict(data: dict[str, Any]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = _as_rotation(data["rotation"])
    transform[:3, 3] = _as_translation(data["translation"])
    return transform


def _project_to_so3(rot: np.ndarray) -> np.ndarray:
    u, _, vt = np.linalg.svd(rot)
    result = u @ vt
    if np.linalg.det(result) < 0:
        u[:, -1] *= -1.0
        result = u @ vt
    return result


def _rotation_to_quaternion(rot: np.ndarray) -> np.ndarray:
    trace = np.trace(rot)
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        quat = np.array(
            [
                0.25 * s,
                (rot[2, 1] - rot[1, 2]) / s,
                (rot[0, 2] - rot[2, 0]) / s,
                (rot[1, 0] - rot[0, 1]) / s,
            ]
        )
    else:
        idx = int(np.argmax(np.diag(rot)))
        if idx == 0:
            s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
            quat = np.array(
                [
                    (rot[2, 1] - rot[1, 2]) / s,
                    0.25 * s,
                    (rot[0, 1] + rot[1, 0]) / s,
                    (rot[0, 2] + rot[2, 0]) / s,
                ]
            )
        elif idx == 1:
            s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
            quat = np.array(
                [
                    (rot[0, 2] - rot[2, 0]) / s,
                    (rot[0, 1] + rot[1, 0]) / s,
                    0.25 * s,
                    (rot[1, 2] + rot[2, 1]) / s,
                ]
            )
        else:
            s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
            quat = np.array(
                [
                    (rot[1, 0] - rot[0, 1]) / s,
                    (rot[0, 2] + rot[2, 0]) / s,
                    (rot[1, 2] + rot[2, 1]) / s,
                    0.25 * s,
                ]
            )
    return quat / np.linalg.norm(quat)


def _quaternion_to_rotation(quat: np.ndarray) -> np.ndarray:
    quat = quat / np.linalg.norm(quat)
    w, x, y, z = quat
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _average_rotations(rotations: list[np.ndarray]) -> np.ndarray:
    accum = np.zeros((4, 4))
    reference = _rotation_to_quaternion(rotations[0])
    for rot in rotations:
        quat = _rotation_to_quaternion(rot)
        if np.dot(quat, reference) < 0.0:
            quat *= -1.0
        accum += np.outer(quat, quat)
    _, eigenvectors = np.linalg.eigh(accum)
    return _quaternion_to_rotation(eigenvectors[:, -1])


def solve_lidar_to_camera(scenes: list[dict[str, Any]]) -> tuple[np.ndarray, np.ndarray, list[float]]:
    transforms = []
    for scene in scenes:
        t_camera_board = _transform_from_dict(scene["T_camera_board"])
        t_lidar_board = _transform_from_dict(scene["T_lidar_board"])
        t_camera_lidar = t_camera_board @ np.linalg.inv(t_lidar_board)
        t_camera_lidar[:3, :3] = _project_to_so3(t_camera_lidar[:3, :3])
        transforms.append(t_camera_lidar)

    rotations = [transform[:3, :3] for transform in transforms]
    translations = [transform[:3, 3] for transform in transforms]
    rcl = _average_rotations(rotations)
    pcl = np.mean(translations, axis=0)

    residuals = []
    for transform in transforms:
        rot_error = rcl.T @ transform[:3, :3]
        angle = math.acos(max(-1.0, min(1.0, (np.trace(rot_error) - 1.0) / 2.0)))
        trans_error = np.linalg.norm(transform[:3, 3] - pcl)
        residuals.append(float(trans_error + angle))

    return rcl, pcl, residuals


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_yaml", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    data = yaml.safe_load(args.input_yaml.read_text(encoding="utf-8"))
    scenes = data.get("scenes", [])
    if len(scenes) < 3:
        raise SystemExit("Need at least 3 scenes for a useful multi-scene estimate.")

    rcl, pcl, residuals = solve_lidar_to_camera(scenes)
    output = {
        "extrin_calib": {
            "Rcl": [float(v) for v in rcl.reshape(-1)],
            "Pcl": [float(v) for v in pcl],
        },
        "residuals": residuals,
        "notes": "Rcl/Pcl are LiDAR -> Camera: p_camera = Rcl * p_lidar + Pcl",
    }

    rendered = yaml.safe_dump(output, sort_keys=False)
    if args.output:
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
