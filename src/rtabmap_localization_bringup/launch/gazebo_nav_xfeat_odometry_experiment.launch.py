#!/usr/bin/env python3
"""Run the navigation ablation experiment with a reproducible spawn pose."""

import os
import random

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


def _seed_spawn_random(context, *args, **kwargs):
    """Seed the random module before the core launch selects a free pose."""
    seed = int(LaunchConfiguration("experiment_seed").perform(context))
    random.seed(seed)
    print(f"[experiment] 固定出生位姿随机种子: {seed}")
    return []


def generate_launch_description():
    """Wrap the unchanged navigation launch with experiment-only arguments."""
    package_share = get_package_share_directory("rtabmap_localization_bringup")
    core_launch = os.path.join(
        package_share, "launch", "gazebo_nav_xfeat_odometry.launch.py"
    )

    start_orb_matcher = LaunchConfiguration("start_orb_matcher")
    consistent_matches = LaunchConfiguration("orb_required_consistent_matches")
    match_period = LaunchConfiguration("orb_match_period_sec")
    consistent_translation = LaunchConfiguration(
        "orb_consistent_translation_m"
    )
    consistent_yaw = LaunchConfiguration("orb_consistent_yaw_deg")
    max_tracking_innovation_translation = LaunchConfiguration(
        "orb_max_tracking_innovation_translation_m"
    )
    max_tracking_innovation_yaw = LaunchConfiguration(
        "orb_max_tracking_innovation_yaw_deg"
    )
    max_correction_linear = LaunchConfiguration(
        "orb_max_correction_linear_mps"
    )
    max_correction_angular = LaunchConfiguration(
        "orb_max_correction_angular_degps"
    )
    goal_relocalization_enabled = LaunchConfiguration(
        "goal_relocalization_enabled")
    start_nav_rviz = LaunchConfiguration("start_nav_rviz")
    start_gazebo_gui = LaunchConfiguration("start_gazebo_gui")
    start_pose_logger = LaunchConfiguration("start_pose_logger")

    return LaunchDescription([
        DeclareLaunchArgument("experiment_seed", default_value="10001"),
        DeclareLaunchArgument("start_orb_matcher", default_value="true"),
        DeclareLaunchArgument(
            "orb_required_consistent_matches", default_value="2"
        ),
        DeclareLaunchArgument("orb_match_period_sec", default_value="2.0"),
        DeclareLaunchArgument(
            "orb_consistent_translation_m", default_value="0.30"
        ),
        DeclareLaunchArgument("orb_consistent_yaw_deg", default_value="5.0"),
        DeclareLaunchArgument(
            "orb_max_tracking_innovation_translation_m", default_value="0.0"
        ),
        DeclareLaunchArgument(
            "orb_max_tracking_innovation_yaw_deg", default_value="0.0"
        ),
        DeclareLaunchArgument(
            "orb_max_correction_linear_mps", default_value="0.20"
        ),
        DeclareLaunchArgument(
            "orb_max_correction_angular_degps", default_value="12.0"
        ),
        DeclareLaunchArgument(
            "goal_relocalization_enabled", default_value="true"
        ),
        DeclareLaunchArgument("start_nav_rviz", default_value="true"),
        DeclareLaunchArgument("start_gazebo_gui", default_value="false"),
        DeclareLaunchArgument("start_pose_logger", default_value="true"),
        OpaqueFunction(function=_seed_spawn_random),
        SetEnvironmentVariable(
            "AUTO_NAV2_GOAL_RELOCALIZATION_ENABLED",
            goal_relocalization_enabled,
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core_launch),
            launch_arguments={
                "start_orb_matcher": start_orb_matcher,
                "orb_required_consistent_matches": consistent_matches,
                "orb_match_period_sec": match_period,
                "orb_consistent_translation_m": consistent_translation,
                "orb_consistent_yaw_deg": consistent_yaw,
                "orb_max_tracking_innovation_translation_m": (
                    max_tracking_innovation_translation
                ),
                "orb_max_tracking_innovation_yaw_deg": (
                    max_tracking_innovation_yaw
                ),
                "orb_max_correction_linear_mps": max_correction_linear,
                "orb_max_correction_angular_degps": max_correction_angular,
                "start_nav_rviz": start_nav_rviz,
                "start_gazebo_gui": start_gazebo_gui,
                "start_pose_logger": start_pose_logger,
            }.items(),
        ),
    ])
