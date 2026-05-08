#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rtabmap_launch = os.path.join(
        get_package_share_directory("rtabmap_launch"),
        "launch",
        "rtabmap.launch.py",
    )

    try:
        realsense_launch = os.path.join(
            get_package_share_directory("realsense2_camera"),
            "launch",
            "rs_launch.py",
        )
    except PackageNotFoundError:
        return LaunchDescription([
            LogInfo(msg="realsense2_camera is not installed. Cannot start D435-only RTAB-Map launch."),
        ])

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_camera = LaunchConfiguration("start_camera")
    serial_no = LaunchConfiguration("serial_no")
    camera_name = LaunchConfiguration("camera_name")
    camera_namespace = LaunchConfiguration("camera_namespace")
    color_profile = LaunchConfiguration("color_profile")
    depth_profile = LaunchConfiguration("depth_profile")
    localization = LaunchConfiguration("localization")
    database_path = LaunchConfiguration("database_path")
    initial_pose = LaunchConfiguration("initial_pose")
    rtabmap_args = LaunchConfiguration("rtabmap_args")
    rgb_topic = LaunchConfiguration("rgb_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    frame_id = LaunchConfiguration("frame_id")
    approx_sync = LaunchConfiguration("approx_sync")
    queue_size = LaunchConfiguration("queue_size")
    start_rtabmapviz = LaunchConfiguration("start_rtabmapviz")
    start_rviz = LaunchConfiguration("start_rviz")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("serial_no", default_value=""),
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("camera_namespace", default_value="camera"),
        DeclareLaunchArgument("color_profile", default_value="640,480,30"),
        DeclareLaunchArgument("depth_profile", default_value="640,480,30"),
        DeclareLaunchArgument("localization", default_value="false"),
        DeclareLaunchArgument(
            "database_path",
            default_value=os.path.expanduser("~/.ros/rtabmap_d435_only.db"),
        ),
        DeclareLaunchArgument("initial_pose", default_value="0 0 0 0 0 0"),
        DeclareLaunchArgument("rtabmap_args", default_value=""),
        DeclareLaunchArgument("rgb_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info"),
        DeclareLaunchArgument("frame_id", default_value="camera_link"),
        DeclareLaunchArgument("approx_sync", default_value="true"),
        DeclareLaunchArgument("queue_size", default_value="10"),
        DeclareLaunchArgument("start_rtabmapviz", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="false"),
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
                        "visual_odometry": "true",
                        "icp_odometry": "false",
                        "subscribe_scan": "false",
                        "subscribe_scan_cloud": "false",
                        "rgb_topic": rgb_topic,
                        "depth_topic": depth_topic,
                        "camera_info_topic": camera_info_topic,
                        "approx_sync": approx_sync,
                        "queue_size": queue_size,
                        "publish_tf_map": "true",
                        "rviz": start_rviz,
                        "rtabmap_viz": start_rtabmapviz,
                    }.items(),
                ),
            ],
        ),
    ])
