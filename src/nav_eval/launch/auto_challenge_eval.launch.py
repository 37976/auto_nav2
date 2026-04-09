#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo_nav_launch = os.path.join(
        get_package_share_directory("gazebo_modele"),
        "launch",
        "gazebo_nav_web.launch.py",
    )
    eval_launch = os.path.join(
        get_package_share_directory("nav_eval"),
        "launch",
        "eval.launch.py",
    )
    challenge_map_yaml = os.path.join(
        get_package_share_directory("nav_slam"),
        "map",
        "challenge_maze.yaml",
    )

    output_dir = LaunchConfiguration("output_dir")
    goals = LaunchConfiguration("goals")
    random_goals = LaunchConfiguration("random_goals")
    random_goal_count = LaunchConfiguration("random_goal_count")
    random_seed = LaunchConfiguration("random_seed")
    random_goal_min_clearance = LaunchConfiguration("random_goal_min_clearance")
    random_goal_min_start_distance = LaunchConfiguration("random_goal_min_start_distance")
    random_goal_min_between_distance = LaunchConfiguration("random_goal_min_between_distance")
    random_goal_max_attempts = LaunchConfiguration("random_goal_max_attempts")
    goal_tolerance = LaunchConfiguration("goal_tolerance")
    goal_timeout_sec = LaunchConfiguration("goal_timeout_sec")
    start_delay_sec = LaunchConfiguration("start_delay_sec")
    between_goal_delay_sec = LaunchConfiguration("between_goal_delay_sec")
    start_nav_rviz = LaunchConfiguration("start_nav_rviz")
    start_web_ui = LaunchConfiguration("start_web_ui")
    start_hotspot = LaunchConfiguration("start_hotspot")

    return LaunchDescription([
        DeclareLaunchArgument("output_dir", default_value="~/auto_nav2_eval/challenge_maze_auto"),
        DeclareLaunchArgument(
            "goals",
            default_value=(
                "5.0,0.0,0.0; 5.0,5.0,0.0; -5.0,5.0,0.0; "
                "-10.0,-5.0,0.0; 10.0,-10.0,0.0; 15.0,10.0,0.0"
            ),
        ),
        DeclareLaunchArgument("random_goals", default_value="true"),
        DeclareLaunchArgument("random_goal_count", default_value="6"),
        DeclareLaunchArgument("random_seed", default_value="-1"),
        DeclareLaunchArgument("random_goal_min_clearance", default_value="0.60"),
        DeclareLaunchArgument("random_goal_min_start_distance", default_value="1.0"),
        DeclareLaunchArgument("random_goal_min_between_distance", default_value="1.0"),
        DeclareLaunchArgument("random_goal_max_attempts", default_value="20000"),
        DeclareLaunchArgument("goal_tolerance", default_value="0.20"),
        DeclareLaunchArgument("goal_timeout_sec", default_value="180.0"),
        DeclareLaunchArgument("start_delay_sec", default_value="10.0"),
        DeclareLaunchArgument("between_goal_delay_sec", default_value="2.0"),
        DeclareLaunchArgument("start_nav_rviz", default_value="true"),
        DeclareLaunchArgument("start_web_ui", default_value="false"),
        DeclareLaunchArgument("start_hotspot", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_nav_launch),
            launch_arguments={
                "world_name": "challenge_maze.world",
                "start_moving_obstacle": "false",
                "static_map_yaml": challenge_map_yaml,
                "start_nav_rviz": start_nav_rviz,
                "start_web_ui": start_web_ui,
                "start_hotspot": start_hotspot,
                "use_pointcloud_obstacles": "true",
                "use_dynamic_obstacle_points": "true",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(eval_launch),
            launch_arguments={
                "output_dir": output_dir,
                "goal_tolerance": goal_tolerance,
                "timeout_sec": goal_timeout_sec,
            }.items(),
        ),
        Node(
            package="nav_eval",
            executable="auto_goal_runner",
            name="auto_goal_runner",
            output="screen",
            parameters=[{
                "goals": goals,
                "random_goals": ParameterValue(random_goals, value_type=bool),
                "random_goal_count": ParameterValue(random_goal_count, value_type=int),
                "random_seed": ParameterValue(random_seed, value_type=int),
                "random_goal_min_clearance": ParameterValue(
                    random_goal_min_clearance, value_type=float),
                "random_goal_min_start_distance": ParameterValue(
                    random_goal_min_start_distance, value_type=float),
                "random_goal_min_between_distance": ParameterValue(
                    random_goal_min_between_distance, value_type=float),
                "random_goal_max_attempts": ParameterValue(
                    random_goal_max_attempts, value_type=int),
                "goal_tolerance": ParameterValue(goal_tolerance, value_type=float),
                "goal_timeout_sec": ParameterValue(goal_timeout_sec, value_type=float),
                "start_delay_sec": ParameterValue(start_delay_sec, value_type=float),
                "between_goal_delay_sec": ParameterValue(
                    between_goal_delay_sec, value_type=float),
                "goal_topic": "/goal_pose",
                "mode_topic": "/control_mode",
                "odom_topic": "/odom",
                "map_topic": "/combined_grid",
                "done_topic": "/nav_eval/done",
            }],
        ),
    ])
