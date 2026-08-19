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
    goal_relocalization_enabled = LaunchConfiguration(
        "goal_relocalization_enabled")

    return LaunchDescription([
        DeclareLaunchArgument("experiment_seed", default_value="10001"),
        DeclareLaunchArgument("start_orb_matcher", default_value="true"),
        DeclareLaunchArgument(
            "orb_required_consistent_matches", default_value="2"
        ),
        DeclareLaunchArgument(
            "goal_relocalization_enabled", default_value="true"
        ),
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
            }.items(),
        ),
    ])
