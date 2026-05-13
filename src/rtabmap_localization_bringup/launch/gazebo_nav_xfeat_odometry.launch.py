#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
        DeclareLaunchArgument("use_pointcloud_obstacles", default_value="false"),
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                "world_name": world_name,
                "start_moving_obstacle": start_moving_obstacle,
                "use_sim_time": use_sim_time,
            }.items(),
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="gazebo_modele",
                    executable="odom_tf_bridge",
                    name="odom_tf_bridge",
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_time}],
                ),
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
                        "map_odom_topic": "/odom",
                        "control_odom_topic": fused_odom_topic,
                        "start_odom_map_tf": "true",
                    }.items(),
                ),
            ],
        ),
    ])
