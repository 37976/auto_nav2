#!/usr/bin/env python3
"""Run one complete continuous-ORB ablation experiment."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


MODES = {
    "odom_imu": {
        "start_orb": "false", "core_matches": "2", "metadata_matches": 0,
        "innovation_gate": False,
    },
    "orb_single_observation": {
        "start_orb": "true", "core_matches": "1", "metadata_matches": 1,
        "innovation_gate": False,
    },
    "orb_quality_gated": {
        "start_orb": "true", "core_matches": "2", "metadata_matches": 2,
        "innovation_gate": True,
    },
}


def _build_experiment(context, *args, **kwargs):
    mode = LaunchConfiguration("experiment_mode").perform(context)
    if mode not in MODES:
        choices = ", ".join(MODES)
        raise ValueError(f"experiment_mode must be one of: {choices}")
    config = MODES[mode]
    seed = int(LaunchConfiguration("experiment_seed").perform(context))
    run_name = LaunchConfiguration("run_name").perform(context).strip()
    if not run_name:
        run_name = f"{mode}_{seed}"
    if config["innovation_gate"]:
        max_tracking_innovation_translation = LaunchConfiguration(
            "orb_max_tracking_innovation_translation_m"
        )
        max_tracking_innovation_yaw = LaunchConfiguration(
            "orb_max_tracking_innovation_yaw_deg"
        )
        metadata_max_tracking_innovation_translation = (
            max_tracking_innovation_translation
        )
        metadata_max_tracking_innovation_yaw = max_tracking_innovation_yaw
    else:
        max_tracking_innovation_translation = "0.0"
        max_tracking_innovation_yaw = "0.0"
        metadata_max_tracking_innovation_translation = 0.0
        metadata_max_tracking_innovation_yaw = 0.0

    package_share = get_package_share_directory("rtabmap_localization_bringup")
    navigation_launch = os.path.join(
        package_share,
        "launch",
        "gazebo_nav_xfeat_odometry_experiment.launch.py",
    )
    common_orb_arguments = {
        "orb_match_period_sec": LaunchConfiguration("orb_match_period_sec"),
        "orb_consistent_translation_m": LaunchConfiguration(
            "orb_consistent_translation_m"
        ),
        "orb_consistent_yaw_deg": LaunchConfiguration(
            "orb_consistent_yaw_deg"
        ),
        "orb_max_tracking_innovation_translation_m": (
            max_tracking_innovation_translation
        ),
        "orb_max_tracking_innovation_yaw_deg": max_tracking_innovation_yaw,
        "orb_max_correction_linear_mps": LaunchConfiguration(
            "orb_max_correction_linear_mps"
        ),
        "orb_max_correction_angular_degps": LaunchConfiguration(
            "orb_max_correction_angular_degps"
        ),
    }
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch),
        launch_arguments={
            "experiment_seed": str(seed),
            "start_orb_matcher": config["start_orb"],
            "orb_required_consistent_matches": config["core_matches"],
            "goal_relocalization_enabled": "false",
            "start_nav_rviz": LaunchConfiguration("start_nav_rviz"),
            "start_gazebo_gui": LaunchConfiguration("start_gazebo_gui"),
            "start_pose_logger": "false",
            **common_orb_arguments,
        }.items(),
    )

    logger = Node(
        package="nav_slam",
        executable="nav_experiment_logger",
        name="nav_experiment_logger",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "output_dir": LaunchConfiguration("output_dir"),
            "run_name": run_name,
            "experiment_group": mode,
            "estimate_topic": "/odom_in_map",
            "align_initial_pose": True,
            "rpe_interval_sec": 1.0,
            "position_tolerance_m": LaunchConfiguration(
                "position_tolerance_m"
            ),
            "yaw_tolerance_deg": LaunchConfiguration("yaw_tolerance_deg"),
            "wait_for_initial_global_correction": True,
            "stop_on_goal_reached": False,
            "stop_on_experiment_done": True,
            "experiment_seed": seed,
            "continuous_orb_enabled": config["start_orb"] == "true",
            "orb_match_period_sec": LaunchConfiguration(
                "orb_match_period_sec"
            ),
            "orb_required_consistent_matches": config["metadata_matches"],
            "orb_consistent_translation_m": LaunchConfiguration(
                "orb_consistent_translation_m"
            ),
            "orb_consistent_yaw_deg": LaunchConfiguration(
                "orb_consistent_yaw_deg"
            ),
            "orb_max_tracking_innovation_translation_m": (
                metadata_max_tracking_innovation_translation
            ),
            "orb_max_tracking_innovation_yaw_deg": (
                metadata_max_tracking_innovation_yaw
            ),
            "orb_max_correction_linear_mps": LaunchConfiguration(
                "orb_max_correction_linear_mps"
            ),
            "orb_max_correction_angular_degps": LaunchConfiguration(
                "orb_max_correction_angular_degps"
            ),
        }],
    )
    goal_runner = Node(
        package="nav_slam",
        executable="nav_experiment_goal_runner",
        name="nav_experiment_goal_runner",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "goals": LaunchConfiguration("goals"),
            "start_delay_sec": LaunchConfiguration("start_delay_sec"),
            "between_goal_delay_sec": LaunchConfiguration(
                "between_goal_delay_sec"
            ),
            "goal_timeout_sec": LaunchConfiguration("goal_timeout_sec"),
        }],
    )
    stop_when_logger_exits = RegisterEventHandler(OnProcessExit(
        target_action=logger,
        on_exit=[
            EmitEvent(event=Shutdown(reason="experiment logger finished"))
        ],
    ))
    stop_if_goal_runner_crashes = RegisterEventHandler(OnProcessExit(
        target_action=goal_runner,
        on_exit=[
            EmitEvent(event=Shutdown(reason="experiment goal runner exited"))
        ],
    ))
    delayed_experiment_nodes = TimerAction(
        period=4.0, actions=[logger, goal_runner]
    )
    return [
        navigation,
        stop_when_logger_exits,
        stop_if_goal_runner_crashes,
        delayed_experiment_nodes,
    ]


def generate_launch_description():
    """Declare paper experiment settings and construct the selected mode."""
    return LaunchDescription([
        DeclareLaunchArgument(
            "experiment_mode", default_value="orb_quality_gated"
        ),
        DeclareLaunchArgument("experiment_seed", default_value="10001"),
        DeclareLaunchArgument("run_name", default_value=""),
        DeclareLaunchArgument(
            "output_dir", default_value="/home/xu/auto_nav2_eval/orb_drift"
        ),
        DeclareLaunchArgument(
            "goals",
            default_value=(
                "7.85,-4.06,0; -8.60,-3.31,180; "
                "8.50,2.34,90; 4.00,-1.16,-157"
            ),
        ),
        DeclareLaunchArgument("position_tolerance_m", default_value="0.50"),
        DeclareLaunchArgument("yaw_tolerance_deg", default_value="5.0"),
        DeclareLaunchArgument("orb_match_period_sec", default_value="2.0"),
        DeclareLaunchArgument(
            "orb_consistent_translation_m", default_value="0.30"
        ),
        DeclareLaunchArgument("orb_consistent_yaw_deg", default_value="5.0"),
        DeclareLaunchArgument(
            "orb_max_tracking_innovation_translation_m", default_value="0.50"
        ),
        DeclareLaunchArgument(
            "orb_max_tracking_innovation_yaw_deg", default_value="5.0"
        ),
        DeclareLaunchArgument(
            "orb_max_correction_linear_mps", default_value="0.20"
        ),
        DeclareLaunchArgument(
            "orb_max_correction_angular_degps", default_value="12.0"
        ),
        DeclareLaunchArgument("start_delay_sec", default_value="2.0"),
        DeclareLaunchArgument("between_goal_delay_sec", default_value="1.0"),
        DeclareLaunchArgument("goal_timeout_sec", default_value="180.0"),
        DeclareLaunchArgument("start_nav_rviz", default_value="true"),
        DeclareLaunchArgument("start_gazebo_gui", default_value="false"),
        OpaqueFunction(function=_build_experiment),
    ])
