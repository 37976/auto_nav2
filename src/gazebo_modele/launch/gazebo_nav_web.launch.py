#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_modele'),
        'launch',
        'gazebo.launch.py',
    )
    nav_launch = os.path.join(
        get_package_share_directory('nav_slam'),
        'launch',
        '2dpoints.launch.py',
    )
    default_static_map_yaml = os.path.join(
        get_package_share_directory('nav_slam'),
        'map',
        'gpt.yaml',
    )

    start_nav_rviz = LaunchConfiguration('start_nav_rviz')
    start_web_ui = LaunchConfiguration('start_web_ui')
    use_static_map = LaunchConfiguration('use_static_map')
    static_map_yaml = LaunchConfiguration('static_map_yaml')
    world_name = LaunchConfiguration('world_name')
    start_moving_obstacle = LaunchConfiguration('start_moving_obstacle')
    web_host = LaunchConfiguration('web_host')
    web_port = LaunchConfiguration('web_port')
    web_image_topic = LaunchConfiguration('web_image_topic')
    use_pointcloud_obstacles = LaunchConfiguration('use_pointcloud_obstacles')
    use_dynamic_obstacle_points = LaunchConfiguration('use_dynamic_obstacle_points')
    start_hotspot = LaunchConfiguration('start_hotspot')
    hotspot_connection_name = LaunchConfiguration('hotspot_connection_name')
    hotspot_ssid = LaunchConfiguration('hotspot_ssid')
    hotspot_password = LaunchConfiguration('hotspot_password')
    hotspot_ifname = LaunchConfiguration('hotspot_ifname')

    return LaunchDescription([
        DeclareLaunchArgument('start_nav_rviz', default_value='true'),
        DeclareLaunchArgument('start_web_ui', default_value='true'),
        DeclareLaunchArgument('use_static_map', default_value='true'),
        DeclareLaunchArgument('static_map_yaml', default_value=default_static_map_yaml),
        DeclareLaunchArgument('world_name', default_value='gpt.world'),
        DeclareLaunchArgument('start_moving_obstacle', default_value='false'),
        DeclareLaunchArgument('web_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('web_image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('use_pointcloud_obstacles', default_value='true'),
        DeclareLaunchArgument('use_dynamic_obstacle_points', default_value='true'),
        DeclareLaunchArgument('start_hotspot', default_value='false'),
        DeclareLaunchArgument('hotspot_connection_name', default_value='dashgo-hotspot'),
        DeclareLaunchArgument('hotspot_ssid', default_value='Dashgo-Robot'),
        DeclareLaunchArgument('hotspot_password', default_value='dashgo12345'),
        DeclareLaunchArgument('hotspot_ifname', default_value=''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
                    launch_arguments={
                        'world_name': world_name,
                        'start_moving_obstacle': start_moving_obstacle,
                        'use_sim_time': 'true',
                    }.items(),
        ),
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav_launch),
                    launch_arguments={
                        'start_nav_rviz': start_nav_rviz,
                        'start_web_ui': start_web_ui,
                        'publish_robot_model': 'false',
                        'use_sim_time': 'true',
                        'use_static_map': use_static_map,
                        'static_map_yaml': static_map_yaml,
                        'web_host': web_host,
                        'web_port': web_port,
                        'web_image_topic': web_image_topic,
                        'use_pointcloud_obstacles': use_pointcloud_obstacles,
                        'use_dynamic_obstacle_points': use_dynamic_obstacle_points,
                        'start_hotspot': start_hotspot,
                        'hotspot_connection_name': hotspot_connection_name,
                        'hotspot_ssid': hotspot_ssid,
                        'hotspot_password': hotspot_password,
                        'hotspot_ifname': hotspot_ifname,
                    }.items(),
                ),
            ],
        ),
    ])
