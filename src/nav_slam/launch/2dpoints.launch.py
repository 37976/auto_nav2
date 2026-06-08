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
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_dir = get_package_share_directory('nav_slam')
    robot_model_dir = get_package_share_directory('gazebo_modele')
    rviz_config_file = os.path.join(config_dir, 'config', 'rviz.rviz')
    robot_model_file = os.path.join(robot_model_dir, 'urdf', 'model.urdf')
    default_static_map_yaml = os.path.join(config_dir, 'map', 'voronoi_50m.yaml')
    web_control_launch = os.path.join(
        get_package_share_directory('dashgo_web_control'),
        'launch',
        'web_control.launch.py',
    )
    with open(robot_model_file, 'r', encoding='utf-8') as robot_model_stream:
        robot_description = robot_model_stream.read()

    start_nav_rviz = LaunchConfiguration('start_nav_rviz')
    start_web_ui = LaunchConfiguration('start_web_ui')
    publish_robot_model = LaunchConfiguration('publish_robot_model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_static_map = LaunchConfiguration('use_static_map')
    static_map_yaml = LaunchConfiguration('static_map_yaml')
    web_host = LaunchConfiguration('web_host')
    web_port = LaunchConfiguration('web_port')
    web_image_topic = LaunchConfiguration('web_image_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    use_pointcloud_obstacles = LaunchConfiguration('use_pointcloud_obstacles')
    use_dynamic_obstacle_points = LaunchConfiguration('use_dynamic_obstacle_points')
    map_odom_topic = LaunchConfiguration('map_odom_topic')
    control_odom_topic = LaunchConfiguration('control_odom_topic')
    start_odom_map_tf = LaunchConfiguration('start_odom_map_tf')
    start_hotspot = LaunchConfiguration('start_hotspot')
    hotspot_connection_name = LaunchConfiguration('hotspot_connection_name')
    hotspot_ssid = LaunchConfiguration('hotspot_ssid')
    hotspot_password = LaunchConfiguration('hotspot_password')
    hotspot_ifname = LaunchConfiguration('hotspot_ifname')

    return LaunchDescription([
        DeclareLaunchArgument('start_nav_rviz', default_value='true'),
        DeclareLaunchArgument('start_web_ui', default_value='true'),
        DeclareLaunchArgument('publish_robot_model', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_static_map', default_value='true'),
        DeclareLaunchArgument('static_map_yaml', default_value=default_static_map_yaml),
        DeclareLaunchArgument('web_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('web_image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('scan_topic', default_value=''),
        DeclareLaunchArgument('pointcloud_topic', default_value='/points_raw'),
        DeclareLaunchArgument('use_pointcloud_obstacles', default_value='true'),
        DeclareLaunchArgument('use_dynamic_obstacle_points', default_value='true'),
        DeclareLaunchArgument('map_odom_topic', default_value='/odom'),
        DeclareLaunchArgument('control_odom_topic', default_value='/odom'),
        DeclareLaunchArgument('start_odom_map_tf', default_value='true'),
        DeclareLaunchArgument('start_hotspot', default_value='false'),
        DeclareLaunchArgument('hotspot_connection_name', default_value='dashgo-hotspot'),
        DeclareLaunchArgument('hotspot_ssid', default_value='Dashgo-Robot'),
        DeclareLaunchArgument('hotspot_password', default_value='dashgo12345'),
        DeclareLaunchArgument('hotspot_ifname', default_value=''),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            condition=IfCondition(publish_robot_model),
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'publish_frequency': 30.0,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'odom_topic': map_odom_topic,
            }],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=IfCondition(publish_robot_model),
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'rate': 30.0,
                'use_gui': False,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
        Node(
            package='nav2_voronoi_planner',
            executable='voronoi_node',
            name='voronoi',
            output='screen',
            remappings=[('/odom', '/odom_in_map')],
            parameters=[{
                'robot_radius': 0.14,
                'clearance_margin': 0.03,
                'occ_threshold': 15,
                'trunk_safety_penalty_scale': 0.06,
                'connector_candidate_count': 0,
                'path_smoothing_control_step': 2,
                'stable_map_replan_period_ms': 3000.0,
                'map_significant_change_cells': 50,
                'path_obstacle_check_distance_m': 3.0,
                'path_switch_min_improvement_m': 0.5,
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
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
                'dynamic_obstacle_timeout': 0.6,
                'obstacle_radius': 0.08,
                'clear_radius': 0.16,
                'projection_gap_fill_cells': 1,
                'accumulate_pointcloud_obstacles': False,
                'use_pointcloud_obstacles': ParameterValue(
                    use_pointcloud_obstacles, value_type=bool),
                'use_dynamic_obstacle_points': ParameterValue(
                    use_dynamic_obstacle_points, value_type=bool),
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'odom_topic': map_odom_topic,
            }],
        ),
        Node(
            package='nav_slam',
            executable='odom_map_tf',
            name='odom_map_tf',
            condition=IfCondition(start_odom_map_tf),
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
        Node(
            package='nav_slam',
            executable='points_pub_map',
            name='points_pub_map',
            output='screen',
            parameters=[{
                'frame_id': 'map',
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'odom_topic': map_odom_topic,
            }],
        ),
        Node(
            package='nav_slam',
            executable='start_nav',
            name='start_nav',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'odom_topic': control_odom_topic,
            }],
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
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
            arguments=['-d', rviz_config_file],
        ),
    ])
