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
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ---- 随机出生安全参数 ----
ROBOT_SAFE_RADIUS_M = 0.35
MIN_CLEARANCE_PREFERRED_M = 2.0


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

    # 把 unknown 一并视为不可出生区域，只允许已知 free 区域
    valid_free_mask = free_mask & ~occupied_mask

    # 距离变换：只在离障碍足够远的 free 区域中出生，并优先更空旷的位置
    # 在图像四周填充 0（障碍物），防止 distanceTransform 的默认边界反射
    # 把地图边缘像素错误地当作"安全空旷区域"导致生成点跑到地图外面
    border = int(max(safe_radius_m, MIN_CLEARANCE_PREFERRED_M) / resolution) + 1
    padded = np.pad(valid_free_mask.astype(np.uint8), border, mode='constant', constant_values=0)
    distance_cells_full = cv2.distanceTransform(padded, cv2.DIST_L2, 5)
    distance_cells = distance_cells_full[border:-border, border:-border]
    preferred_cells = float(MIN_CLEARANCE_PREFERRED_M / resolution)

    # 只允许生在大空旷区域（>= MIN_CLEARANCE_PREFERRED_M），不 fallback 到小区域
    preferred_mask = distance_cells >= preferred_cells
    candidate_mask = preferred_mask

    # 收集所有候选空闲栅格坐标
    free_rows, free_cols = np.where(candidate_mask)
    if len(free_rows) == 0:
        raise RuntimeError(
            f"地图中没有足够的大空旷区域（需要 ≥{MIN_CLEARANCE_PREFERRED_M:.2f}m 净空），"
            f"当前最小安全半径 {safe_radius_m:.2f}m。请检查地图或降低 MIN_CLEARANCE_PREFERRED_M。")

    # 随机选一个
    idx = random.randint(0, len(free_rows) - 1)
    px = free_cols[idx]
    py = free_rows[idx]

    # 图像像素中心 → 地图世界坐标
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
    """OpaqueFunction: 读取地图，随机选空闲位姿，写入 launch_configurations。"""
    map_yaml_path = LaunchConfiguration("static_map_yaml").perform(context)
    print(f"[random_spawn] 读取地图: {map_yaml_path}")
    x, y, yaw = _pick_random_free_pose(map_yaml_path)
    context.launch_configurations["random_spawn_x"] = f"{x:.4f}"
    context.launch_configurations["random_spawn_y"] = f"{y:.4f}"
    context.launch_configurations["random_spawn_yaw"] = f"{yaw:.4f}"
    return []


def _build_timed_actions(context, *args, **kwargs):
    """构建 TimerAction 内部的节点列表。"""
    start_odom_map_tf = "false"

    actions = [
        # 1.5. 激光 ORB 全局定位 → 锁定 map→odom 静态 TF
        Node(
            package="nav_slam",
            executable="lidar_global_localize",
            name="lidar_global_localize",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "map_yaml_path": LaunchConfiguration("static_map_yaml"),
            }],
        ),
        # 2. 静态地图服务器
        Node(
            package="nav_slam",
            executable="static_map_server",
            name="static_map_server",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "map_yaml_path": LaunchConfiguration("static_map_yaml"),
                "publish_period_sec": 1.0,
                "frame_id": "map",
            }],
        ),
        # 2.5. 一次性地图转发
        Node(
            package="nav_slam",
            executable="map_once_relay",
            name="map_once_relay",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
        # 2.6. 激光扫描转点云
        Node(
            package="nav_slam",
            executable="laser_scan_to_points",
            name="laser_scan_to_points",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "scan_topic": "/scan",
                "output_topic": "/mapokk",
                "target_frame": "map",
            }],
        ),
        # 3. odom TF 桥接
        Node(
            package="gazebo_modele",
            executable="odom_tf_bridge",
            name="odom_tf_bridge",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "odom_topic": "/odom",
            }],
        ),
        # 3.5. odom→map 坐标转发
        Node(
            package="nav_slam",
            executable="odom_to_map_relay",
            name="odom_to_map_relay",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "odom_topic": "/localized_odom",
                "output_topic": "/odom_in_map",
            }],
        ),
        # 3.6. Gazebo 真实位姿 vs 计算位姿 对比记录
        Node(
            package="nav_slam",
            executable="pose_logger",
            name="pose_logger",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "output_path": os.path.join(
                    os.path.expanduser("~"), "project", "位姿对比", "pose_comparison.csv"),
                "log_hz": 5.0,
            }],
        ),
        # 4. XFeat 视觉里程计（修正增益已置零，保留节点供后续可选使用）
        Node(
            package="rtabmap_localization_bringup",
            executable="xfeat_rgbd_odometry",
            name="xfeat_rgbd_odometry",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "rgb_topic": "/camera/camera/image_raw",
                "depth_topic": "/camera/camera/depth/image_raw",
                "camera_info_topic": "/camera/camera/camera_info",
                "xfeat_weights_path": LaunchConfiguration("xfeat_weights_path"),
                "top_k": LaunchConfiguration("top_k"),
                "detection_threshold": LaunchConfiguration("detection_threshold"),
                "min_score": LaunchConfiguration("min_score"),
                "min_depth_m": LaunchConfiguration("depth_min_m"),
                "max_depth_m": LaunchConfiguration("depth_max_m"),
                "sync_queue_size": 10,
                "odom_topic": LaunchConfiguration("odom_topic"),
                "delta_odom_topic": LaunchConfiguration("delta_odom_topic"),
                "odom_frame": LaunchConfiguration("odom_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "camera_frame": LaunchConfiguration("camera_frame"),
                "publish_tf": LaunchConfiguration("publish_tf"),
                "match_min_cossim": LaunchConfiguration("match_min_cossim"),
                "min_pnp_points": LaunchConfiguration("min_pnp_points"),
                "min_inliers": LaunchConfiguration("min_inliers"),
                "pnp_reproj_error": LaunchConfiguration("pnp_reproj_error"),
                "pnp_iterations": LaunchConfiguration("pnp_iterations"),
            }],
        ),
        # 4.5. 激光扫描 -> ORB 地图匹配 持续定位修正（可通过参数关闭）
    ]
    if LaunchConfiguration("start_orb_matcher").perform(context) == "true":
        actions.append(
            Node(
                package="nav_slam",
                executable="orb_map_matcher",
                name="orb_map_matcher",
                output="screen",
                parameters=[{
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "map_yaml_path": LaunchConfiguration("static_map_yaml"),
                    "scan_topic": "/scan",
                    "odom_topic": "/localized_odom",
                    "delta_odom_topic": LaunchConfiguration("orb_delta_topic"),
                    "base_frame": LaunchConfiguration("base_frame"),
                    "match_period_sec": LaunchConfiguration("orb_match_period_sec"),
                    "lidar_max_range": 8.0,
                    "map_resolution": 0.05,
                    "max_iterations": LaunchConfiguration("orb_max_iterations"),
                    "min_f1_score": LaunchConfiguration("orb_min_f1_score"),
                    "correction_gain_xy": LaunchConfiguration("orb_gain_xy"),
                    "correction_gain_yaw": LaunchConfiguration("orb_gain_yaw"),
                }],
            )
        )
    actions.extend([
        # 4.6. 导航目标到达后全局重定位编排器
        Node(
            package="nav_slam",
            executable="nav_goal_relocalizer",
            name="nav_goal_relocalizer",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "min_relocalize_interval_sec": LaunchConfiguration(
                    "relocalize_interval_sec"),
                "orb_disable_duration_sec": LaunchConfiguration(
                    "orb_disable_duration_sec"),
            }],
        ),
        # 5. 里程计融合（使用 ORB 修正源）
        Node(
            package="rtabmap_localization_bringup",
            executable="odom_fusion_node",
            name="odom_fusion_node",
            output="screen",
            parameters=[{
                "base_odom_topic": "/odom",
                "xfeat_delta_topic": LaunchConfiguration("orb_delta_topic"),
                "output_odom_topic": LaunchConfiguration("fused_odom_topic"),
                "correction_gain_xy": 1.0,
                "correction_gain_yaw": 1.0,
                "use_imu_yaw": LaunchConfiguration("use_imu_yaw"),
                "max_delta_translation_diff_m": 0.50,
                "max_delta_yaw_diff_deg": 45.0,
                "xfeat_timeout_sec": 5.0,
            }],
        ),
    ])

    # 9. 导航核心
    nav_launch_file = os.path.join(
        get_package_share_directory("nav_slam"),
        "launch",
        "2dpoints.launch.py",
    )
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch_file),
            launch_arguments={
                "start_nav_rviz": LaunchConfiguration("start_nav_rviz"),
                "start_web_ui": LaunchConfiguration("start_web_ui"),
                "publish_robot_model": "false",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "use_static_map": LaunchConfiguration("use_static_map"),
                "static_map_yaml": LaunchConfiguration("static_map_yaml"),
                "web_host": LaunchConfiguration("web_host"),
                "web_port": LaunchConfiguration("web_port"),
                "web_image_topic": LaunchConfiguration("web_image_topic"),
                "scan_topic": "/scan",
                "pointcloud_topic": "",
                "use_pointcloud_obstacles": LaunchConfiguration("use_pointcloud_obstacles"),
                "use_dynamic_obstacle_points": LaunchConfiguration("use_dynamic_obstacle_points"),
                "start_hotspot": LaunchConfiguration("start_hotspot"),
                "hotspot_connection_name": LaunchConfiguration("hotspot_connection_name"),
                "hotspot_ssid": LaunchConfiguration("hotspot_ssid"),
                "hotspot_password": LaunchConfiguration("hotspot_password"),
                "hotspot_ifname": LaunchConfiguration("hotspot_ifname"),
                "map_odom_topic": LaunchConfiguration("fused_odom_topic"),
                "control_odom_topic": "/odom_in_map",
                "start_odom_map_tf": start_odom_map_tf,
            }.items(),
        )
    )

    return [TimerAction(period=2.0, actions=actions)]


def generate_launch_description():
    gazebo_launch = os.path.join(
        get_package_share_directory("gazebo_modele"),
        "launch",
        "gazebo.launch.py",
    )
    default_static_map_yaml = os.path.join(
        get_package_share_directory("nav_slam"),
        "map",
        "dashgo_slam_map.yaml",
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
    use_imu_yaw = LaunchConfiguration("use_imu_yaw")
    max_delta_translation_diff_m = LaunchConfiguration("max_delta_translation_diff_m")
    max_delta_yaw_diff_deg = LaunchConfiguration("max_delta_yaw_diff_deg")
    # ORB 地图匹配参数
    orb_delta_topic = LaunchConfiguration("orb_delta_topic")
    orb_match_period_sec = LaunchConfiguration("orb_match_period_sec")
    orb_max_iterations = LaunchConfiguration("orb_max_iterations")
    orb_min_f1_score = LaunchConfiguration("orb_min_f1_score")
    orb_gain_xy = LaunchConfiguration("orb_gain_xy")
    orb_gain_yaw = LaunchConfiguration("orb_gain_yaw")
    # 导航到达目标点后重定位间隔
    relocalize_interval_sec = LaunchConfiguration("relocalize_interval_sec")
    orb_disable_duration_sec = LaunchConfiguration("orb_disable_duration_sec")
    start_orb_matcher = LaunchConfiguration("start_orb_matcher")
    # 随机 spawn 位姿（由 OpaqueFunction 填入）
    random_spawn_x = LaunchConfiguration("random_spawn_x", default="0.0")
    random_spawn_y = LaunchConfiguration("random_spawn_y", default="0.0")
    random_spawn_yaw = LaunchConfiguration("random_spawn_yaw", default="0.0")

    return LaunchDescription([
        # ---- 0. 强制 ROS 2 仅用回环接口，避免插网线后 DDS 多播扰乱 TF 通信 ----
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        # ---- 0.5. 启动前清理：避免上次 Ctrl+C 残留导致第二次启动卡顿 ----
        ExecuteProcess(
            cmd=["bash", os.path.join(
                get_package_share_directory("rtabmap_localization_bringup"),
                "..", "..", "..", "..", "cleanup.sh",
            )],
            name="pre_launch_cleanup",
            output="screen",
        ),
        DeclareLaunchArgument("world_name", default_value="small_house.world"),
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
        DeclareLaunchArgument("use_imu_yaw", default_value="true"),
        DeclareLaunchArgument("max_delta_translation_diff_m", default_value="0.20"),
        DeclareLaunchArgument("max_delta_yaw_diff_deg", default_value="20.0"),
        # ORB 地图匹配参数
        DeclareLaunchArgument("orb_delta_topic", default_value="/orb/delta_odom"),
        DeclareLaunchArgument("orb_match_period_sec", default_value="2.0"),
        DeclareLaunchArgument("orb_max_iterations", default_value="50"),
        DeclareLaunchArgument("orb_min_f1_score", default_value="30.0"),
        DeclareLaunchArgument("orb_gain_xy", default_value="0.6"),
        DeclareLaunchArgument("orb_gain_yaw", default_value="0.5"),
        DeclareLaunchArgument("use_amcl", default_value="true"),
        DeclareLaunchArgument("relocalize_interval_sec", default_value="10.0"),
        DeclareLaunchArgument("orb_disable_duration_sec", default_value="10.0"),
        DeclareLaunchArgument("start_orb_matcher", default_value="true"),
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
                "spawn_z": "0.03",
                "spawn_yaw": random_spawn_yaw,
            }.items(),
        ),
        OpaqueFunction(function=_build_timed_actions),
    ])