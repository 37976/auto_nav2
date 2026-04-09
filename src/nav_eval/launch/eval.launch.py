#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    output_dir = LaunchConfiguration("output_dir")
    goal_tolerance = LaunchConfiguration("goal_tolerance")
    timeout_sec = LaunchConfiguration("timeout_sec")
    robot_radius = LaunchConfiguration("robot_radius")
    obstacle_threshold = LaunchConfiguration("obstacle_threshold")
    clearance_search_radius = LaunchConfiguration("clearance_search_radius")
    collision_clearance = LaunchConfiguration("collision_clearance")
    near_collision_clearance = LaunchConfiguration("near_collision_clearance")
    stuck_speed_threshold = LaunchConfiguration("stuck_speed_threshold")

    return LaunchDescription([
        DeclareLaunchArgument("output_dir", default_value="~/auto_nav2_eval"),
        DeclareLaunchArgument("goal_tolerance", default_value="0.20"),
        DeclareLaunchArgument("timeout_sec", default_value="180.0"),
        DeclareLaunchArgument("robot_radius", default_value="0.12"),
        DeclareLaunchArgument("obstacle_threshold", default_value="15"),
        DeclareLaunchArgument("clearance_search_radius", default_value="2.0"),
        DeclareLaunchArgument("collision_clearance", default_value="0.0"),
        DeclareLaunchArgument("near_collision_clearance", default_value="0.08"),
        DeclareLaunchArgument("stuck_speed_threshold", default_value="0.02"),
        Node(
            package="nav_eval",
            executable="nav_eval_node",
            name="nav_eval",
            output="screen",
            parameters=[{
                "output_dir": output_dir,
                "goal_tolerance": ParameterValue(goal_tolerance, value_type=float),
                "timeout_sec": ParameterValue(timeout_sec, value_type=float),
                "robot_radius": ParameterValue(robot_radius, value_type=float),
                "obstacle_threshold": ParameterValue(obstacle_threshold, value_type=int),
                "clearance_search_radius": ParameterValue(
                    clearance_search_radius, value_type=float),
                "collision_clearance": ParameterValue(
                    collision_clearance, value_type=float),
                "near_collision_clearance": ParameterValue(
                    near_collision_clearance, value_type=float),
                "stuck_speed_threshold": ParameterValue(
                    stuck_speed_threshold, value_type=float),
                "odom_topic": "/odom",
                "cmd_vel_topic": "/cmd_vel",
                "goal_topic": "/goal_pose",
                "path_topic": "/path",
                "map_topic": "/combined_grid",
            }],
        ),
    ])
