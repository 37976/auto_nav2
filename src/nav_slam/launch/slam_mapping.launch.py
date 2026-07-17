#!/usr/bin/env python3
"""
slam_mapping.launch.py — 2D 雷达建图 (仿真版).

启动 Gazebo + slam_toolbox (online_async) + slam_controller + RViz.
用键盘/手柄控制机器人建图，完成后通过 service 保存地图:

    ros2 service call /slam_controller/save_map std_srvs/srv/Trigger
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_slam_share = get_package_share_directory("nav_slam")
    gazebo_share = get_package_share_directory("gazebo_modele")

    gazebo_launch = os.path.join(gazebo_share, "launch", "gazebo.launch.py")
    slam_params_file = os.path.join(nav_slam_share, "config", "slam_toolbox_params.yaml")
    rviz_config = os.path.join(nav_slam_share, "config", "rviz.rviz")

    # --- Arguments ---
    world_name = LaunchConfiguration("world_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params = LaunchConfiguration("slam_params_file")
    scan_topic = LaunchConfiguration("scan_topic")
    start_rviz = LaunchConfiguration("start_rviz")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    return LaunchDescription([
        DeclareLaunchArgument("world_name", default_value="small_house.world",
                              description="Gazebo world file"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("slam_params_file", default_value=slam_params_file,
                              description="slam_toolbox parameter file"),
        DeclareLaunchArgument("scan_topic", default_value="/scan",
                              description="LiDAR scan topic"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("start_moving_obstacle", default_value="false"),
        DeclareLaunchArgument("spawn_x", default_value="5.0",
                              description="robot spawn X (m)"),
        DeclareLaunchArgument("spawn_y", default_value="-5.0",
                              description="robot spawn Y (m)"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0",
                              description="robot spawn yaw (rad)"),

        # Gazebo + 传感器
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                "world_name": world_name,
                "use_sim_time": use_sim_time,
                "start_moving_obstacle": LaunchConfiguration("start_moving_obstacle"),
                "spawn_x": spawn_x,
                "spawn_y": spawn_y,
                "spawn_yaw": spawn_yaw,
            }.items(),
        ),

        # 等待 Gazebo 启动
        TimerAction(
            period=3.0,
            actions=[
                # odom → base_footprint TF
                Node(
                    package="gazebo_modele",
                    executable="odom_tf_bridge",
                    name="odom_tf_bridge",
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_time}],
                ),
                # slam_toolbox (online_async)
                Node(
                    package="slam_toolbox",
                    executable="async_slam_toolbox_node",
                    name="slam_toolbox",
                    output="screen",
                    parameters=[slam_params],
                    remappings=[
                        ("/scan", scan_topic),
                        ("/map", "/slam_map"),
                    ],
                ),
                # slam_controller: slam_map → combined_grid + save 服务
                Node(
                    package="nav_slam",
                    executable="slam_controller",
                    name="slam_controller",
                    output="screen",
                ),
                # RViz
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    condition=IfCondition(start_rviz),
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_time}],
                    arguments=["-d", rviz_config, "-f", "map"],
                ),
            ],
        ),
    ])
