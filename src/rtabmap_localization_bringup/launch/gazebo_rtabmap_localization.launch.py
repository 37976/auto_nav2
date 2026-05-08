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
        "gazebo_sim_only.launch.py",
    )
    rtabmap_launch = os.path.join(
        get_package_share_directory("rtabmap_launch"),
        "launch",
        "rtabmap.launch.py",
    )

    world_name = LaunchConfiguration("world_name")
    start_moving_obstacle = LaunchConfiguration("start_moving_obstacle")
    use_sim_time = LaunchConfiguration("use_sim_time")
    localization = LaunchConfiguration("localization")
    database_path = LaunchConfiguration("database_path")
    rtabmap_args = LaunchConfiguration("rtabmap_args")
    start_rviz = LaunchConfiguration("start_rviz")

    return LaunchDescription([
        DeclareLaunchArgument("world_name", default_value="gpt.world"),
        DeclareLaunchArgument("start_moving_obstacle", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("localization", default_value="false"),
        DeclareLaunchArgument(
            "database_path",
            default_value=os.path.expanduser("~/.ros/rtabmap_auto_nav2_gazebo.db"),
        ),
        DeclareLaunchArgument("rtabmap_args", default_value="--delete_db_on_start"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
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
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(rtabmap_launch),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                        "localization": localization,
                        "database_path": database_path,
                        "rtabmap_args": rtabmap_args,
                        "frame_id": "base_link",
                        "odom_topic": "/odom",
                        "visual_odometry": "false",
                        "subscribe_scan_cloud": "true",
                        "scan_cloud_topic": "/points_raw",
                        "rgb_topic": "/camera/camera/image_raw",
                        "depth_topic": "/camera/camera/depth/image_raw",
                        "camera_info_topic": "/camera/camera/camera_info",
                        "approx_sync": "true",
                        "queue_size": "30",
                        "publish_tf_map": "true",
                        "rviz": start_rviz,
                        "rtabmapviz": "false",
                    }.items(),
                ),
            ],
        ),
    ])
