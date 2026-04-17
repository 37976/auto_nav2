#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name_in_model = 'fishbot'
    package_name = 'gazebo_modele'

    pkg_share = FindPackageShare(package=package_name)
    pkg_share_dir = get_package_share_directory(package_name)
    world_name = LaunchConfiguration('world_name')
    start_moving_obstacle = LaunchConfiguration('start_moving_obstacle')
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_model_path = PathJoinSubstitution([pkg_share, 'urdf', 'model.urdf'])
    gazebo_world_path = PathJoinSubstitution([pkg_share, 'world', world_name])
    robot_model_file = os.path.join(pkg_share_dir, 'urdf', 'model.urdf')
    with open(robot_model_file, 'r', encoding='utf-8') as robot_model_stream:
        robot_description = robot_model_stream.read()

    start_gazebo_cmd = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            gazebo_world_path,
        ],
        output='screen')

    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model, '-file', urdf_model_path],
        output='screen')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 30.0,
            'use_sim_time': use_sim_time,
        }],
    )
    moving_obstacle_cmd = Node(
        package='gazebo_modele',
        executable='moving_obstacle_controller',
        name='moving_obstacle_controller',
        condition=IfCondition(start_moving_obstacle),
        output='screen',
        parameters=[{
            'enabled': True,
            'entity_name': 'dynamic_box_1',
            'reference_frame': 'world',
            'publish_rate': 20.0,
            'base_x': 0.0,
            'base_y': -6.0,
            'base_z': 0.5,
            'amp_x': 3.0,
            'amp_y': 0.0,
            'period': 10.0,
            'yaw': 0.0,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='gpt.world'),
        DeclareLaunchArgument('start_moving_obstacle', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('IGN_IP', '127.0.0.1'),
        start_gazebo_cmd,
        spawn_entity_cmd,
        start_robot_state_publisher_cmd,
        moving_obstacle_cmd,
    ])
