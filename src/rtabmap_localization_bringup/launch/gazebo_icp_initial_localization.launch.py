#!/usr/bin/env python3
"""Gazebo + sensors + one-shot global ICP, without navigation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rtabmap_localization_bringup.initial_localization_launch_common import (
    set_random_spawn,
)


def generate_launch_description():
    gazebo_launch = os.path.join(
        get_package_share_directory("gazebo_modele"), "launch", "gazebo.launch.py"
    )
    default_map = os.path.join(
        get_package_share_directory("nav_slam"), "map", "dashgo_slam_map.yaml"
    )
    world_name = LaunchConfiguration("world_name")
    map_yaml = LaunchConfiguration("static_map_yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    spawn_x = LaunchConfiguration("random_spawn_x", default="0.0")
    spawn_y = LaunchConfiguration("random_spawn_y", default="0.0")
    spawn_yaw = LaunchConfiguration("random_spawn_yaw", default="0.0")

    return LaunchDescription([
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
        DeclareLaunchArgument("world_name", default_value="small_house.world"),
        DeclareLaunchArgument("static_map_yaml", default_value=default_map),
        DeclareLaunchArgument("spawn_seed", default_value="1"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("start_gazebo_gui", default_value="false"),
        OpaqueFunction(function=set_random_spawn),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                "world_name": world_name,
                "start_moving_obstacle": "false",
                "use_sim_time": use_sim_time,
                "start_gazebo_gui": LaunchConfiguration("start_gazebo_gui"),
                "spawn_x": spawn_x,
                "spawn_y": spawn_y,
                "spawn_z": "0.03",
                "spawn_yaw": spawn_yaw,
            }.items(),
        ),
        Node(
            package="nav_slam",
            executable="icp_global_localizer",
            name="icp_global_localizer",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "map_yaml_path": map_yaml,
            }],
        ),
    ])
