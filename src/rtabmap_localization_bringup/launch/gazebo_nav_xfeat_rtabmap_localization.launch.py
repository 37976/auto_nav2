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
    rtabmap_launch = os.path.join(
        get_package_share_directory("rtabmap_launch"),
        "launch",
        "rtabmap.launch.py",
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
    database_path = LaunchConfiguration("database_path")
    localization = LaunchConfiguration("localization")
    initial_pose = LaunchConfiguration("initial_pose")
    rtabmap_args = LaunchConfiguration("rtabmap_args")
    start_rtabmapviz = LaunchConfiguration("start_rtabmapviz")
    start_rtabmap_rviz = LaunchConfiguration("start_rtabmap_rviz")
    output_rate_hz = LaunchConfiguration("output_rate_hz")
    top_k = LaunchConfiguration("top_k")
    detection_threshold = LaunchConfiguration("detection_threshold")
    min_score = LaunchConfiguration("min_score")
    depth_max_m = LaunchConfiguration("depth_max_m")
    xfeat_repo_dir = LaunchConfiguration("xfeat_repo_dir")
    xfeat_weights_path = LaunchConfiguration("xfeat_weights_path")
    xfeat_log_level = LaunchConfiguration("xfeat_log_level")
    rtabmap_log_level = LaunchConfiguration("rtabmap_log_level")

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
        DeclareLaunchArgument(
            "database_path",
            default_value=os.path.expanduser("~/.ros/rtabmap_auto_nav2_xfeat_gazebo.db"),
        ),
        DeclareLaunchArgument("localization", default_value="true"),
        DeclareLaunchArgument("initial_pose", default_value="0 0 0 0 0 0"),
        DeclareLaunchArgument(
            "rtabmap_args",
            default_value="--Mem/IncrementalMemory false --Mem/InitWMWithAllNodes true",
        ),
        DeclareLaunchArgument("start_rtabmapviz", default_value="false"),
        DeclareLaunchArgument("start_rtabmap_rviz", default_value="false"),
        DeclareLaunchArgument("output_rate_hz", default_value="0.5"),
        DeclareLaunchArgument("top_k", default_value="160"),
        DeclareLaunchArgument("detection_threshold", default_value="0.05"),
        DeclareLaunchArgument("min_score", default_value="0.05"),
        DeclareLaunchArgument("depth_max_m", default_value="8.0"),
        DeclareLaunchArgument("xfeat_repo_dir", default_value=os.path.expanduser("~/project/XFeat")),
        DeclareLaunchArgument("xfeat_weights_path", default_value=""),
        DeclareLaunchArgument("xfeat_log_level", default_value="error"),
        DeclareLaunchArgument("rtabmap_log_level", default_value="error"),
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
                    executable="xfeat_rtabmap_bridge",
                    name="xfeat_rtabmap_bridge",
                    output="screen",
                    arguments=["--ros-args", "--log-level", xfeat_log_level],
                    parameters=[{
                        "rgb_topic": "/camera/camera/image_raw",
                        "depth_topic": "/camera/camera/depth/image_raw",
                        "camera_info_topic": "/camera/camera/camera_info",
                        "output_rgbd_topic": "/xfeat/rgbd_image",
                        "xfeat_repo_dir": xfeat_repo_dir,
                        "xfeat_weights_path": xfeat_weights_path,
                        "top_k": top_k,
                        "detection_threshold": detection_threshold,
                        "min_score": min_score,
                        "depth_max_m": depth_max_m,
                        "output_rate_hz": output_rate_hz,
                        "sync_queue_size": 10,
                    }],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(rtabmap_launch),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                        "localization": localization,
                        "database_path": database_path,
                        "initial_pose": initial_pose,
                        "rtabmap_args": rtabmap_args,
                        "frame_id": "base_link",
                        "odom_topic": "/odom",
                        "visual_odometry": "false",
                        "subscribe_scan": "true",
                        "scan_topic": "/scan",
                        "subscribe_scan_cloud": "false",
                        "subscribe_rgbd": "true",
                        "rgbd_sync": "false",
                        "rgbd_topic": "/xfeat/rgbd_image",
                        "publish_tf_map": "true",
                        "rviz": start_rtabmap_rviz,
                        "rtabmap_viz": start_rtabmapviz,
                        "approx_sync": "true",
                        "queue_size": "30",
                        "log_level": rtabmap_log_level,
                        "odom_log_level": rtabmap_log_level,
                    }.items(),
                ),
                Node(
                    package="gazebo_modele",
                    executable="localized_odom_bridge",
                    name="localized_odom_bridge",
                    output="screen",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {"input_odom_topic": "/odom"},
                        {"output_odom_topic": "/localized_odom"},
                        {"map_frame": "map"},
                        {"odom_frame": "odom"},
                    ],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav_launch),
                    launch_arguments={
                        "start_nav_rviz": start_nav_rviz,
                        "start_web_ui": start_web_ui,
                        "publish_robot_model": "false",
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
                        "odom_topic": "/localized_odom",
                        "start_odom_map_tf": "false",
                    }.items(),
                ),
            ],
        ),
    ])
