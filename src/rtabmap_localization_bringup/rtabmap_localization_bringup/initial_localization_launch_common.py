"""Shared random-spawn helpers for initial-localization baseline launches."""

import math
import os
import random

import cv2
import numpy as np
import yaml
from launch.substitutions import LaunchConfiguration


ROBOT_SAFE_RADIUS_M = 1.0


def pick_random_free_pose(map_yaml_path: str, seed: int) -> tuple[float, float, float]:
    """Match the ORB navigation launch's free-space spawn distribution."""
    yaml_dir = os.path.dirname(os.path.abspath(map_yaml_path))
    with open(map_yaml_path, "r", encoding="utf-8") as stream:
        meta = yaml.safe_load(stream)

    image = cv2.imread(os.path.join(yaml_dir, meta["image"]), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f"无法读取地图图片: {meta['image']}")

    resolution = float(meta["resolution"])
    origin_x = float(meta["origin"][0])
    origin_y = float(meta["origin"][1])
    image_norm = image.astype(np.float32) / 255.0
    occupancy = image_norm if meta.get("negate", 0) else 1.0 - image_norm
    free_mask = occupancy < float(meta.get("free_thresh", 0.196))
    occupied_mask = occupancy > float(meta.get("occupied_thresh", 0.65))
    valid_free = free_mask & ~occupied_mask

    border = int(ROBOT_SAFE_RADIUS_M / resolution) + 1
    padded = np.pad(valid_free.astype(np.uint8), border, constant_values=0)
    distance = cv2.distanceTransform(padded, cv2.DIST_L2, 5)[
        border:-border, border:-border
    ]
    rows, cols = np.where(distance >= ROBOT_SAFE_RADIUS_M / resolution)
    if len(rows) == 0:
        raise RuntimeError("地图中没有满足机器人安全半径的出生位置")

    generator = random.Random(seed)
    index = generator.randrange(len(rows))
    row = int(rows[index])
    col = int(cols[index])
    yaw = generator.uniform(-math.pi, math.pi)
    height, width = image.shape
    x = origin_x + (col + 0.5) * resolution
    y = origin_y + (height - row - 0.5) * resolution
    clearance = float(distance[row, col] * resolution)
    print(
        f"[random_spawn] seed={seed} 地图 {width}×{height} @ {resolution:.3f}m/格, "
        f"空闲候选 {len(rows)} 个, 选中 px=({col},{row}) clearance={clearance:.2f}m "
        f"→ world=({x:.4f}, {y:.4f}, {math.degrees(yaw):.4f}°)",
        flush=True,
    )
    return x, y, yaw


def set_random_spawn(context, *args, **kwargs):
    """Populate spawn launch configurations from a reproducible seed."""
    map_yaml = LaunchConfiguration("static_map_yaml").perform(context)
    seed = int(LaunchConfiguration("spawn_seed").perform(context))
    x, y, yaw = pick_random_free_pose(map_yaml, seed)
    context.launch_configurations["random_spawn_x"] = f"{x:.6f}"
    context.launch_configurations["random_spawn_y"] = f"{y:.6f}"
    context.launch_configurations["random_spawn_yaw"] = f"{yaw:.6f}"
    return []
