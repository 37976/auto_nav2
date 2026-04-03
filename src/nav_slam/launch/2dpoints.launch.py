#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = get_package_share_directory('nav_slam')
    rviz_config_file = os.path.join(config_dir, 'config', 'rviz.rviz')
    default_static_map_yaml = os.path.join(config_dir, 'map', 'gpt.yaml')
    web_control_launch = os.path.join(
        get_package_share_directory('dashgo_web_control'),
        'launch',
        'web_control.launch.py',
    )

    start_nav_rviz = LaunchConfiguration('start_nav_rviz')
    start_web_ui = LaunchConfiguration('start_web_ui')
    use_static_map = LaunchConfiguration('use_static_map')
    static_map_yaml = LaunchConfiguration('static_map_yaml')
    web_host = LaunchConfiguration('web_host')
    web_port = LaunchConfiguration('web_port')
    web_image_topic = LaunchConfiguration('web_image_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
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
        DeclareLaunchArgument('web_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('web_image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('scan_topic', default_value=''),
        DeclareLaunchArgument('pointcloud_topic', default_value='/points_raw'),
        DeclareLaunchArgument('start_hotspot', default_value='false'),
        DeclareLaunchArgument('hotspot_connection_name', default_value='dashgo-hotspot'),
        DeclareLaunchArgument('hotspot_ssid', default_value='Dashgo-Robot'),
        DeclareLaunchArgument('hotspot_password', default_value='dashgo12345'),
        DeclareLaunchArgument('hotspot_ifname', default_value=''),
        Node(
            package='nav2_voronoi_planner',
            executable='voronoi_node',
            name='voronoi',
            output='screen',
            parameters=[{
                'robot_radius': 0.14,
                'occ_threshold': 15,
                'trunk_safety_penalty_scale': 0.06,
            }],
        ),
        Node(
            package='nav_slam',
            executable='map_pub',
            name='map_pub',
            output='screen',
            parameters=[{
                'use_static_map': use_static_map,
                'static_map_yaml': static_map_yaml,
                'dynamic_obstacle_timeout': 0.2,
                'obstacle_radius': 0.05,
                'clear_radius': 0.14,
                'projection_gap_fill_cells': 0,
                'accumulate_pointcloud_obstacles': False,
            }],
        ),
        Node(
            package='nav_slam',
            executable='odom_map_tf',
            name='odom_map_tf',
            output='screen',
        ),
        Node(
            package='nav_slam',
            executable='points_pub_map',
            name='points_pub_map',
            output='screen',
            parameters=[{'frame_id': 'map'}],
        ),
        Node(
            package='nav_slam',
            executable='start_nav',
            name='start_nav',
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(web_control_launch),
            condition=IfCondition(start_web_ui),
            launch_arguments={
                'host': web_host,
                'port': web_port,
                'image_topic': web_image_topic,
                'scan_topic': scan_topic,
                'pointcloud_topic': pointcloud_topic,
                'start_hotspot': start_hotspot,
                'hotspot_connection_name': hotspot_connection_name,
                'hotspot_ssid': hotspot_ssid,
                'hotspot_password': hotspot_password,
                'hotspot_ifname': hotspot_ifname,
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(start_nav_rviz),
            output='screen',
            arguments=['-d', rviz_config_file],
        ),
    ])
