#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_modele'),
        'launch',
        'gazebo.launch.py',
    )

    world_name = LaunchConfiguration('world_name')
    start_moving_obstacle = LaunchConfiguration('start_moving_obstacle')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='gpt.world'),
        DeclareLaunchArgument('start_moving_obstacle', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'world_name': world_name,
                'start_moving_obstacle': start_moving_obstacle,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
    ])
