#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gazebo_sensors_only.launch.py -- 仅启动 Gazebo + 传感器，不启动任何定位/导航节点.
用于独立测试 kidnapped_robot_finder 等外部定位模块.
"""

import math
import os
import random

import cv2
import numpy as np
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ROBOT_SAFE_RADIUS_M = 0.35
MIN_CLEARANCE_PREFERRED_M = 0.60


def _pick_random_free_pose(map_yaml_path: str, safe_radius_m: float = ROBOT_SAFE_RADIUS_M):
    yaml_dir = os.path.dirname(os.path.abspath(map_yaml_path))
    with open(map_yaml_path, "r", encoding="utf-8") as f:
        meta = yaml.safe_load(f)

    pgm_path = os.path.join(yaml_dir, meta["image"])
    image = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f"无法读取地图图片: {pgm_path}")

    resolution = float(meta["resolution"])
    origin_x = float(meta["origin"][0])
    origin_y = float(meta["origin"][1])
    negate = meta.get("negate", 0)
    free_thresh = float(meta.get("free_thresh", 0.196))
    occupied_thresh = float(meta.get("occupied_thresh", 0.65))
    mode = meta.get("mode", "trinary")

    height, width = image.shape
    image_norm = image.astype(np.float32) / 255.0
    if negate:
        occ_prob = image_norm
    else:
        occ_prob = 1.0 - image_norm

    if mode == "trinary":
        free_mask = occ_prob < free_thresh
        occupied_mask = occ_prob > occupied_thresh
    else:
        free_mask = occ_prob < free_thresh
        occupied_mask = occ_prob >= occupied_thresh

    valid_free_mask = free_mask & ~occupied_mask
    # 在图像四周填充 0（障碍物），防止 distanceTransform 的默认边界反射
    # 把地图边缘像素错误地当作"安全空旷区域"导致生成点跑到地图外面
    border = int(max(safe_radius_m, MIN_CLEARANCE_PREFERRED_M) / resolution) + 1
    padded = np.pad(valid_free_mask.astype(np.uint8), border, mode='constant', constant_values=0)
    distance_cells_full = cv2.distanceTransform(padded, cv2.DIST_L2, 5)
    distance_cells = distance_cells_full[border:-border, border:-border]
    safe_cells = float(safe_radius_m / resolution)
    preferred_cells = float(MIN_CLEARANCE_PREFERRED_M / resolution)

    eligible_mask = distance_cells >= safe_cells
    preferred_mask = distance_cells >= preferred_cells
    candidate_mask = preferred_mask if np.any(preferred_mask) else eligible_mask

    free_rows, free_cols = np.where(candidate_mask)
    if len(free_rows) == 0:
        raise RuntimeError("地图中没有足够的安全空闲区域！")

    idx = random.randint(0, len(free_rows) - 1)
    px = free_cols[idx]
    py = free_rows[idx]
    x_m = origin_x + (px + 0.5) * resolution
    y_m = origin_y + ((height - py - 1) + 0.5) * resolution
    yaw_rad = random.uniform(-math.pi, math.pi)
    clearance_m = float(distance_cells[py, px] * resolution)

    print(f"[random_spawn] 地图 {width}×{height} @ {resolution:.3f}m/格, "
          f"空闲候选 {len(free_rows)} 个, "
          f"选中 px=({px},{py}) clearance={clearance_m:.2f}m "
          f"→ world=({x_m:.2f}, {y_m:.2f}, {math.degrees(yaw_rad):.1f}°)")
    return x_m, y_m, yaw_rad


def _set_random_spawn(context, *args, **kwargs):
    map_yaml_path = LaunchConfiguration("static_map_yaml").perform(context)
    print(f"[random_spawn] 读取地图: {map_yaml_path}")
    x, y, yaw = _pick_random_free_pose(map_yaml_path)
    context.launch_configurations["random_spawn_x"] = f"{x:.4f}"
    context.launch_configurations["random_spawn_y"] = f"{y:.4f}"
    context.launch_configurations["random_spawn_yaw"] = f"{yaw:.4f}"
    return []


def generate_launch_description():
    gazebo_launch = os.path.join(
        get_package_share_directory("gazebo_modele"),
        "launch",
        "gazebo.launch.py",
    )
    default_static_map_yaml = os.path.join(
        get_package_share_directory("nav_slam"),
        "map",
        "localization_10m.yaml",
    )

    world_name = LaunchConfiguration("world_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_moving_obstacle = LaunchConfiguration("start_moving_obstacle")
    static_map_yaml = LaunchConfiguration("static_map_yaml")
    random_spawn_x = LaunchConfiguration("random_spawn_x", default="0.0")
    random_spawn_y = LaunchConfiguration("random_spawn_y", default="0.0")
    random_spawn_yaw = LaunchConfiguration("random_spawn_yaw", default="0.0")

    return LaunchDescription([
        DeclareLaunchArgument("world_name", default_value="localization_10m.world"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("start_moving_obstacle", default_value="false"),
        DeclareLaunchArgument("static_map_yaml", default_value=default_static_map_yaml),

        # 随机选空闲位姿
        OpaqueFunction(function=_set_random_spawn),

        # Gazebo + robot_state_publisher + 传感器
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                "world_name": world_name,
                "start_moving_obstacle": start_moving_obstacle,
                "use_sim_time": use_sim_time,
                "spawn_x": random_spawn_x,
                "spawn_y": random_spawn_y,
                "spawn_z": "0.03",
                "spawn_yaw": random_spawn_yaw,
            }.items(),
        ),
    ])
