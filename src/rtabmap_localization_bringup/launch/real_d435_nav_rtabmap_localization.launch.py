#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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

    try:
        realsense_launch = os.path.join(
            get_package_share_directory("realsense2_camera"),
            "launch",
            "rs_launch.py",
        )
    except PackageNotFoundError:
        return LaunchDescription([
            LogInfo(msg="realsense2_camera is not installed. Cannot start real D435 RTAB-Map launch."),
        ])

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_camera = LaunchConfiguration("start_camera")
    serial_no = LaunchConfiguration("serial_no")
    camera_name = LaunchConfiguration("camera_name")
    camera_namespace = LaunchConfiguration("camera_namespace")
    color_profile = LaunchConfiguration("color_profile")
    depth_profile = LaunchConfiguration("depth_profile")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    start_nav_rviz = LaunchConfiguration("start_nav_rviz")
    start_web_ui = LaunchConfiguration("start_web_ui")
    use_static_map = LaunchConfiguration("use_static_map")
    static_map_yaml = LaunchConfiguration("static_map_yaml")
    web_host = LaunchConfiguration("web_host")
    web_port = LaunchConfiguration("web_port")
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
    rgb_topic = LaunchConfiguration("rgb_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    rtabmap_queue_size = LaunchConfiguration("rtabmap_queue_size")
    rtabmap_approx_sync = LaunchConfiguration("rtabmap_approx_sync")
    frame_id = LaunchConfiguration("frame_id")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("serial_no", default_value=""),
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("camera_namespace", default_value="camera"),
        DeclareLaunchArgument("color_profile", default_value="640,480,30"),
        DeclareLaunchArgument("depth_profile", default_value="640,480,30"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("frame_id", default_value="base_footprint"),
        DeclareLaunchArgument("start_nav_rviz", default_value="true"),
        DeclareLaunchArgument("start_web_ui", default_value="false"),
        DeclareLaunchArgument("use_static_map", default_value="true"),
        DeclareLaunchArgument("static_map_yaml", default_value=default_static_map_yaml),
        DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("web_port", default_value="8080"),
        DeclareLaunchArgument("use_pointcloud_obstacles", default_value="false"),
        DeclareLaunchArgument("use_dynamic_obstacle_points", default_value="false"),
        DeclareLaunchArgument("start_hotspot", default_value="false"),
        DeclareLaunchArgument("hotspot_connection_name", default_value="dashgo-hotspot"),
        DeclareLaunchArgument("hotspot_ssid", default_value="Dashgo-Robot"),
        DeclareLaunchArgument("hotspot_password", default_value="dashgo12345"),
        DeclareLaunchArgument("hotspot_ifname", default_value=""),
        DeclareLaunchArgument(
            "database_path",
            default_value=os.path.expanduser("~/.ros/rtabmap_auto_nav2_real.db"),
        ),
        DeclareLaunchArgument("localization", default_value="true"),
        DeclareLaunchArgument("initial_pose", default_value="0 0 0 0 0 0"),
        DeclareLaunchArgument("rtabmap_args", default_value=""),
        DeclareLaunchArgument("rgb_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info"),
        DeclareLaunchArgument("rtabmap_queue_size", default_value="10"),
        DeclareLaunchArgument("rtabmap_approx_sync", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            condition=IfCondition(start_camera),
            launch_arguments={
                "serial_no": serial_no,
                "camera_name": camera_name,
                "camera_namespace": camera_namespace,
                "enable_color": "true",
                "enable_depth": "true",
                "enable_infra1": "false",
                "enable_infra2": "false",
                "enable_gyro": "false",
                "enable_accel": "false",
                "rgb_camera.color_profile": color_profile,
                "depth_module.depth_profile": depth_profile,
                "enable_pointcloud": "false",
                "align_depth.enable": "true",
                "initial_reset": "true",
                "publish_tf": "true",
                "publish_odom_tf": "false",
            }.items(),
        ),
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(rtabmap_launch),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                        "localization": localization,
                        "database_path": database_path,
                        "initial_pose": initial_pose,
                        "rtabmap_args": rtabmap_args,
                        "frame_id": frame_id,
                        "odom_topic": odom_topic,
                        "visual_odometry": "false",
                        "subscribe_scan": "true",
                        "scan_topic": scan_topic,
                        "subscribe_scan_cloud": "false",
                        "rgb_topic": rgb_topic,
                        "depth_topic": depth_topic,
                        "camera_info_topic": camera_info_topic,
                        "approx_sync": rtabmap_approx_sync,
                        "queue_size": rtabmap_queue_size,
                        "publish_tf_map": "true",
                        "rviz": "false",
                        "rtabmapviz": "false",
                    }.items(),
                ),
                Node(
                    package="gazebo_modele",
                    executable="localized_odom_bridge",
                    name="localized_odom_bridge",
                    output="screen",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {"input_odom_topic": odom_topic},
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
                        "publish_robot_model": "true",
                        "use_sim_time": use_sim_time,
                        "use_static_map": use_static_map,
                        "static_map_yaml": static_map_yaml,
                        "web_host": web_host,
                        "web_port": web_port,
                        "web_image_topic": rgb_topic,
                        "scan_topic": scan_topic,
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
