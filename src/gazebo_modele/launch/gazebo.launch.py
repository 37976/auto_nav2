#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name_in_model = 'fishbot'
    package_name = 'gazebo_modele'

    pkg_share = FindPackageShare(package=package_name)
    world_name = LaunchConfiguration('world_name')
    urdf_model_path = PathJoinSubstitution([pkg_share, 'urdf', 'model.urdf'])
    gazebo_world_path = PathJoinSubstitution([pkg_share, 'world', world_name])

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
        arguments=[urdf_model_path]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[urdf_model_path]
    )
    moving_obstacle_cmd = Node(
        package='gazebo_modele',
        executable='moving_obstacle_controller',
        name='moving_obstacle_controller',
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
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='gpt.world'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        start_gazebo_cmd,
        spawn_entity_cmd,
        start_robot_state_publisher_cmd,
        joint_state_publisher_node,
        moving_obstacle_cmd,
    ])
