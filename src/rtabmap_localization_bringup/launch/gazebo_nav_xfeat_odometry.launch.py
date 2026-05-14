#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import random
import yaml

import cv2
import numpy as np

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ---- 机器人安全半径（米），用于避开障碍物 ----
ROBOT_SAFE_RADIUS_M = 0.25


def _pick_random_free_pose(map_yaml_path: str, safe_radius_m: float = ROBOT_SAFE_RADIUS_M):
    """
    读取 PGM+YAML 地图，随机选取空闲区域的一个位姿。
    返回 (x_m, y_m, yaw_rad)，均为世界坐标系（与地图 origin 对齐）。
    确保机器人安全半径内无障碍物。
    """
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

    # 构建占用栅格：0=free, 100=occupied, -1=unknown
    if mode == "trinary":
        if negate:
            free_mask = (image / 255.0) <= free_thresh
            occupied_mask = (image / 255.0) >= occupied_thresh
        else:
            free_mask = (image / 255.0) <= free_thresh
            occupied_mask = (image / 255.0) >= occupied_thresh
    else:
        # raw/scale: 简单阈值
        if negate:
            free_mask = (image / 255.0) <= free_thresh
        else:
            free_mask = (image / 255.0) <= free_thresh

    # 腐蚀安全半径区域
    safe_cells = int(math.ceil(safe_radius_m / resolution))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * safe_cells + 1, 2 * safe_cells + 1))
    free_mask_eroded = cv2.erode(free_mask.astype(np.uint8), kernel).astype(bool)

    # 收集所有空闲栅格坐标
    free_rows, free_cols = np.where(free_mask_eroded)
    if len(free_rows) == 0:
        raise RuntimeError("地图中没有足够的安全空闲区域！请检查 safe_radius_m 或地图。")

    # 随机选一个
    idx = random.randint(0, len(free_rows) - 1)
    px = free_cols[idx]
    py = free_rows[idx]

    # 栅格坐标 → 世界坐标 (地图中心在 origin + size/2)
    x_m = origin_x + px * resolution
    y_m = origin_y + py * resolution
    yaw_rad = random.uniform(-math.pi, math.pi)

    print(f"[random_spawn] 地图 {width}×{height} @ {resolution:.3f}m/格, "
          f"空闲候选 {len(free_rows)} 个, "
          f"选中 px=({px},{py}) → world=({x_m:.2f}, {y_m:.2f}, {math.degrees(yaw_rad):.1f}°)")

    return x_m, y_m, yaw_rad


def _set_random_spawn(context, *args, **kwargs):
    """OpaqueFunction: 读取地图，随机选空闲位姿，写入 launch_configurations。"""
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
    nav_launch = os.path.join(
        get_package_share_directory("nav_slam"),
        "launch",
        "2dpoints.launch.py",
    )
    default_static_map_yaml = os.path.join(
        get_package_share_directory("nav_slam"),
        "map",
        "gpt.yaml",
    )

    world_name = LaunchConfiguration("world_name")
    start_moving_obstacle = LaunchConfiguration("start_moving_obstacle")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_nav_rviz = LaunchConfiguration("start_nav_rviz")
    start_web_ui = LaunchConfiguration("start_web_ui")
    use_static_map = LaunchConfiguration("use_static_map")
    static_map_yaml = LaunchConfiguration("static_map_yaml")
    web_host = LaunchConfiguration("web_host")
    web_port = LaunchConfiguration("web_port")
    web_image_topic = LaunchConfiguration("web_image_topic")
    use_pointcloud_obstacles = LaunchConfiguration("use_pointcloud_obstacles")
    use_dynamic_obstacle_points = LaunchConfiguration("use_dynamic_obstacle_points")
    start_hotspot = LaunchConfiguration("start_hotspot")
    hotspot_connection_name = LaunchConfiguration("hotspot_connection_name")
    hotspot_ssid = LaunchConfiguration("hotspot_ssid")
    hotspot_password = LaunchConfiguration("hotspot_password")
    hotspot_ifname = LaunchConfiguration("hotspot_ifname")
    top_k = LaunchConfiguration("top_k")
    detection_threshold = LaunchConfiguration("detection_threshold")
    min_score = LaunchConfiguration("min_score")
    depth_min_m = LaunchConfiguration("depth_min_m")
    depth_max_m = LaunchConfiguration("depth_max_m")
    xfeat_repo_dir = LaunchConfiguration("xfeat_repo_dir")
    xfeat_weights_path = LaunchConfiguration("xfeat_weights_path")
    match_min_cossim = LaunchConfiguration("match_min_cossim")
    min_pnp_points = LaunchConfiguration("min_pnp_points")
    min_inliers = LaunchConfiguration("min_inliers")
    pnp_reproj_error = LaunchConfiguration("pnp_reproj_error")
    pnp_iterations = LaunchConfiguration("pnp_iterations")
    odom_topic = LaunchConfiguration("odom_topic")
    delta_odom_topic = LaunchConfiguration("delta_odom_topic")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    camera_frame = LaunchConfiguration("camera_frame")
    publish_tf = LaunchConfiguration("publish_tf")
    fused_odom_topic = LaunchConfiguration("fused_odom_topic")
    correction_gain_xy = LaunchConfiguration("correction_gain_xy")
    correction_gain_yaw = LaunchConfiguration("correction_gain_yaw")
    max_delta_translation_diff_m = LaunchConfiguration("max_delta_translation_diff_m")
    max_delta_yaw_diff_deg = LaunchConfiguration("max_delta_yaw_diff_deg")
    # 随机 spawn 位姿（由 OpaqueFunction 填入）
    random_spawn_x = LaunchConfiguration("random_spawn_x", default="0.0")
    random_spawn_y = LaunchConfiguration("random_spawn_y", default="0.0")
    random_spawn_yaw = LaunchConfiguration("random_spawn_yaw", default="0.0")

    return LaunchDescription([
        DeclareLaunchArgument("world_name", default_value="gpt.world"),
        DeclareLaunchArgument("start_moving_obstacle", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("start_nav_rviz", default_value="true"),
        DeclareLaunchArgument("start_web_ui", default_value="false"),
        DeclareLaunchArgument("use_static_map", default_value="true"),
        DeclareLaunchArgument("static_map_yaml", default_value=default_static_map_yaml),
        DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("web_port", default_value="8080"),
        DeclareLaunchArgument("web_image_topic", default_value="/camera/camera/image_raw"),
        DeclareLaunchArgument("use_pointcloud_obstacles", default_value="true"),
        DeclareLaunchArgument("use_dynamic_obstacle_points", default_value="false"),
        DeclareLaunchArgument("start_hotspot", default_value="false"),
        DeclareLaunchArgument("hotspot_connection_name", default_value="dashgo-hotspot"),
        DeclareLaunchArgument("hotspot_ssid", default_value="Dashgo-Robot"),
        DeclareLaunchArgument("hotspot_password", default_value="dashgo12345"),
        DeclareLaunchArgument("hotspot_ifname", default_value=""),
        DeclareLaunchArgument("top_k", default_value="768"),
        DeclareLaunchArgument("detection_threshold", default_value="0.05"),
        DeclareLaunchArgument("min_score", default_value="0.0"),
        DeclareLaunchArgument("depth_min_m", default_value="0.2"),
        DeclareLaunchArgument("depth_max_m", default_value="8.0"),
        DeclareLaunchArgument("xfeat_repo_dir", default_value="/home/xu/project/XFeat"),
        DeclareLaunchArgument("xfeat_weights_path", default_value=""),
        DeclareLaunchArgument("match_min_cossim", default_value="0.65"),
        DeclareLaunchArgument("min_pnp_points", default_value="6"),
        DeclareLaunchArgument("min_inliers", default_value="4"),
        DeclareLaunchArgument("pnp_reproj_error", default_value="8.0"),
        DeclareLaunchArgument("pnp_iterations", default_value="200"),
        DeclareLaunchArgument("odom_topic", default_value="/xfeat/odom"),
        DeclareLaunchArgument("delta_odom_topic", default_value="/xfeat/delta_odom"),
        DeclareLaunchArgument("odom_frame", default_value="xfeat_odom"),
        DeclareLaunchArgument("base_frame", default_value="base_footprint"),
        DeclareLaunchArgument("camera_frame", default_value="camera_optical_frame"),
        DeclareLaunchArgument("publish_tf", default_value="false"),
        DeclareLaunchArgument("fused_odom_topic", default_value="/localized_odom"),
        DeclareLaunchArgument("correction_gain_xy", default_value="0.15"),
        DeclareLaunchArgument("correction_gain_yaw", default_value="0.10"),
        DeclareLaunchArgument("max_delta_translation_diff_m", default_value="0.20"),
        DeclareLaunchArgument("max_delta_yaw_diff_deg", default_value="20.0"),
        # 0. 随机选空闲位姿（必须先于 gazebo 和 AMCL）
        OpaqueFunction(function=_set_random_spawn),
        # 1. 启动 Gazebo（用随机位姿 spawn 机器人）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                "world_name": world_name,
                "start_moving_obstacle": start_moving_obstacle,
                "use_sim_time": use_sim_time,
                "spawn_x": random_spawn_x,
                "spawn_y": random_spawn_y,
                "spawn_yaw": random_spawn_yaw,
            }.items(),
        ),
        TimerAction(
            period=2.0,
            actions=[
                # 2. 静态地图服务器 → 发布干净地图到 /map
                Node(
                    package="nav_slam",
                    executable="static_map_server",
                    name="static_map_server",
                    output="screen",
                    parameters=[{
                        "use_sim_time": use_sim_time,
                        "map_yaml_path": static_map_yaml,
                        "publish_period_sec": 1.0,
                        "frame_id": "map",
                    }],
                ),
                # 2.5. 一次性地图转发 → 避免 AMCL 反复重建似然场
                Node(
                    package="nav_slam",
                    executable="map_once_relay",
                    name="map_once_relay",
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_time}],
                ),
                # 2.6. 激光扫描转点云 → TF→map 帧 → /mapokk，直接供 map_pub 使用
                Node(
                    package="nav_slam",
                    executable="laser_scan_to_points",
                    name="laser_scan_to_points",
                    output="screen",
                    parameters=[{
                        "use_sim_time": use_sim_time,
                        "scan_topic": "/scan",
                        "output_topic": "/mapokk",
                        "target_frame": "map",
                    }],
                ),
                # 3. odom TF 桥接 → 监听融合里程计, 发布 odom→base_footprint TF
                Node(
                    package="gazebo_modele",
                    executable="odom_tf_bridge",
                    name="odom_tf_bridge",
                    output="screen",
                    parameters=[{
                        "use_sim_time": use_sim_time,
                        "odom_topic": fused_odom_topic,
                    }],
                ),
                # 4. XFeat 视觉里程计
                Node(
                    package="rtabmap_localization_bringup",
                    executable="xfeat_rgbd_odometry",
                    name="xfeat_rgbd_odometry",
                    output="screen",
                    parameters=[{
                        "rgb_topic": "/camera/camera/image_raw",
                        "depth_topic": "/camera/camera/depth/image_raw",
                        "camera_info_topic": "/camera/camera/camera_info",
                        "xfeat_repo_dir": xfeat_repo_dir,
                        "xfeat_weights_path": xfeat_weights_path,
                        "top_k": top_k,
                        "detection_threshold": detection_threshold,
                        "min_score": min_score,
                        "min_depth_m": depth_min_m,
                        "max_depth_m": depth_max_m,
                        "sync_queue_size": 10,
                        "odom_topic": odom_topic,
                        "delta_odom_topic": delta_odom_topic,
                        "odom_frame": odom_frame,
                        "base_frame": base_frame,
                        "camera_frame": camera_frame,
                        "publish_tf": publish_tf,
                        "match_min_cossim": match_min_cossim,
                        "min_pnp_points": min_pnp_points,
                        "min_inliers": min_inliers,
                        "pnp_reproj_error": pnp_reproj_error,
                        "pnp_iterations": pnp_iterations,
                    }],
                ),
                # 5. 里程计融合 → 轮式 + XFeat → /localized_odom
                Node(
                    package="rtabmap_localization_bringup",
                    executable="odom_fusion_node",
                    name="odom_fusion_node",
                    output="screen",
                    parameters=[{
                        "base_odom_topic": "/odom",
                        "xfeat_delta_topic": delta_odom_topic,
                        "output_odom_topic": fused_odom_topic,
                        "correction_gain_xy": correction_gain_xy,
                        "correction_gain_yaw": correction_gain_yaw,
                        "max_delta_translation_diff_m": max_delta_translation_diff_m,
                        "max_delta_yaw_diff_deg": max_delta_yaw_diff_deg,
                    }],
                ),
                # 6. AMCL 全局定位（不设初始位姿，通过 global localization 自主收敛）
                Node(
                    package="nav2_amcl",
                    executable="amcl",
                    name="amcl",
                    output="screen",
                    parameters=[{
                        "use_sim_time": use_sim_time,
                        "base_frame_id": "base_footprint",
                        "global_frame_id": "map",
                        "odom_frame_id": "odom",
                        "scan_topic": "/scan",
                        "min_particles": 1000,
                        "max_particles": 15000,
                        "pf_err": 0.01,
                        "pf_z": 0.99,
                        "recovery_alpha_fast": 0.001,
                        "recovery_alpha_slow": 0.001,
                        "resample_interval": 1,
                        "save_pose_rate": 0.5,
                        "update_min_a": 0.2,
                        "update_min_d": 0.25,
                        "robot_model_type": "nav2_amcl::DifferentialMotionModel",
                        "alpha1": 0.2,
                        "alpha2": 0.2,
                        "alpha3": 0.2,
                        "alpha4": 0.2,
                        "laser_model_type": "likelihood_field",
                        "laser_max_range": 100.0,
                        "laser_min_range": -1.0,
                        "z_hit": 0.5,
                        "z_max": 0.05,
                        "z_rand": 0.5,
                        "z_short": 0.05,
                        "sigma_hit": 0.2,
                        "lambda_short": 0.1,
                        "laser_likelihood_max_dist": 2.0,
                        "do_beamskip": False,
                        "max_beams": 120,
                        "tf_broadcast": False,
                        "transform_tolerance": 2.0,
                    }],
                    remappings=[
                        ("/scan", "/scan"),
                        ("/map", "/map_for_amcl"),
                    ],
                ),
                # 7. 生命周期管理器 → 自动激活 AMCL
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_amcl",
                    output="screen",
                    parameters=[{
                        "use_sim_time": use_sim_time,
                        "node_names": ["amcl"],
                        "autostart": True,
                    }],
                ),
                # 7.5. AMCL 全局定位触发器 → 等待 AMCL 激活后将粒子散布到全地图
                Node(
                    package="nav_slam",
                    executable="amcl_global_localize",
                    name="amcl_global_localize",
                    output="screen",
                    parameters=[{
                        "use_sim_time": use_sim_time,
                    }],
                ),
                # 8. AMCL 收敛后锁定 map→odom 静态 TF，后续导航不再依赖 AMCL
                Node(
                    package="nav_slam",
                    executable="amcl_init_bridge",
                    name="amcl_init_bridge",
                    output="screen",
                    parameters=[{
                        "use_sim_time": use_sim_time,
                        "cov_xy_threshold": 0.005,
                        "cov_yaw_threshold": 0.003,
                    }],
                ),
                # 9. 导航核心
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav_launch),
                    launch_arguments={
                        "start_nav_rviz": start_nav_rviz,
                        "start_web_ui": start_web_ui,
                        "publish_robot_model": "true",
                        "use_sim_time": use_sim_time,
                        "use_static_map": use_static_map,
                        "static_map_yaml": static_map_yaml,
                        "web_host": web_host,
                        "web_port": web_port,
                        "web_image_topic": web_image_topic,
                        "scan_topic": "/scan",
                        "pointcloud_topic": "",
                        "use_pointcloud_obstacles": use_pointcloud_obstacles,
                        "use_dynamic_obstacle_points": use_dynamic_obstacle_points,
                        "start_hotspot": start_hotspot,
                        "hotspot_connection_name": hotspot_connection_name,
                        "hotspot_ssid": hotspot_ssid,
                        "hotspot_password": hotspot_password,
                        "hotspot_ifname": hotspot_ifname,
                        "map_odom_topic": fused_odom_topic,
                        "control_odom_topic": fused_odom_topic,
                        "start_odom_map_tf": "false",
                    }.items(),
                ),
            ],
        ),
    ])
