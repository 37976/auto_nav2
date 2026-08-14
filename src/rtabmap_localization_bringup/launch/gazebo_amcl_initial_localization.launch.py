#!/usr/bin/env python3
"""Gazebo + sensors + AMCL global initialization, without navigation."""

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
from launch_ros.parameter_descriptions import ParameterValue

from rtabmap_localization_bringup.initial_localization_launch_common import (
    set_random_spawn,
)


def generate_launch_description():
    gazebo_launch = os.path.join(
        get_package_share_directory("gazebo_modele"), "launch", "gazebo.launch.py"
    )
    nav_slam_share = get_package_share_directory("nav_slam")
    default_map = os.path.join(nav_slam_share, "map", "dashgo_slam_map.yaml")
    amcl_config = os.path.join(nav_slam_share, "config", "amcl_params.yaml")

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
        DeclareLaunchArgument("localization_timeout_sec", default_value="90.0"),
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
            package="gazebo_modele",
            executable="odom_tf_bridge",
            name="odom_tf_bridge",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time, "odom_topic": "/odom"}],
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "yaml_filename": map_yaml,
            }],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[amcl_config, {"use_sim_time": use_sim_time}],
            remappings=[("scan", "/scan"), ("map", "/map")],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["map_server", "amcl"],
            }],
        ),
        Node(
            package="nav_slam",
            executable="amcl_initial_localizer",
            name="amcl_initial_localizer",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "timeout_sec": ParameterValue(
                    LaunchConfiguration("localization_timeout_sec"), value_type=float
                ),
            }],
        ),
    ])
