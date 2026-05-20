#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rtabmap_launch = os.path.join(
        get_package_share_directory("rtabmap_launch"),
        "launch",
        "rtabmap.launch.py",
    )

    try:
        realsense_launch = os.path.join(
            get_package_share_directory("realsense2_camera"),
            "launch",
            "rs_launch.py",
        )
    except PackageNotFoundError:
        return LaunchDescription([
            LogInfo(msg="realsense2_camera is not installed. Cannot start D435-only XFeat RTAB-Map launch."),
        ])

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_camera = LaunchConfiguration("start_camera")
    serial_no = LaunchConfiguration("serial_no")
    camera_name = LaunchConfiguration("camera_name")
    camera_namespace = LaunchConfiguration("camera_namespace")
    color_profile = LaunchConfiguration("color_profile")
    depth_profile = LaunchConfiguration("depth_profile")
    localization = LaunchConfiguration("localization")
    database_path = LaunchConfiguration("database_path")
    initial_pose = LaunchConfiguration("initial_pose")
    rtabmap_args = LaunchConfiguration("rtabmap_args")
    rgb_topic = LaunchConfiguration("rgb_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    frame_id = LaunchConfiguration("frame_id")
    start_rtabmapviz = LaunchConfiguration("start_rtabmapviz")
    start_rviz = LaunchConfiguration("start_rviz")
    output_rate_hz = LaunchConfiguration("output_rate_hz")
    top_k = LaunchConfiguration("top_k")
    detection_threshold = LaunchConfiguration("detection_threshold")
    min_score = LaunchConfiguration("min_score")
    depth_scale = LaunchConfiguration("depth_scale")
    depth_min_m = LaunchConfiguration("depth_min_m")
    depth_max_m = LaunchConfiguration("depth_max_m")
    xfeat_repo_dir = LaunchConfiguration("xfeat_repo_dir")
    xfeat_weights_path = LaunchConfiguration("xfeat_weights_path")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("serial_no", default_value=""),
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("camera_namespace", default_value="camera"),
        DeclareLaunchArgument("color_profile", default_value="640,480,30"),
        DeclareLaunchArgument("depth_profile", default_value="640,480,30"),
        DeclareLaunchArgument("localization", default_value="true"),
        DeclareLaunchArgument(
            "database_path",
            default_value=os.path.expanduser("~/.ros/rtabmap_d435_xfeat.db"),
        ),
        DeclareLaunchArgument("initial_pose", default_value="0 0 0 0 0 0"),
        DeclareLaunchArgument(
            "rtabmap_args",
            default_value="--Mem/IncrementalMemory false --Mem/InitWMWithAllNodes true",
        ),
        DeclareLaunchArgument("rgb_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info"),
        DeclareLaunchArgument("frame_id", default_value="camera_link"),
        DeclareLaunchArgument("start_rtabmapviz", default_value="false"),
        DeclareLaunchArgument("start_rviz", default_value="false"),
        DeclareLaunchArgument("output_rate_hz", default_value="0.5"),
        DeclareLaunchArgument("top_k", default_value="160"),
        DeclareLaunchArgument("detection_threshold", default_value="0.05"),
        DeclareLaunchArgument("min_score", default_value="0.05"),
        DeclareLaunchArgument("depth_scale", default_value="0.001"),
        DeclareLaunchArgument("depth_min_m", default_value="0.2"),
        DeclareLaunchArgument("depth_max_m", default_value="3.0"),
        DeclareLaunchArgument("xfeat_repo_dir", default_value=os.path.expanduser("~/project/XFeat")),
        DeclareLaunchArgument("xfeat_weights_path", default_value=""),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            condition=IfCondition(start_camera),
            launch_arguments={
                "serial_no": serial_no,
                "camera_name": camera_name,
                "camera_namespace": camera_namespace,
                "enable_color": "true",
                "enable_depth": "true",
                "enable_infra1": "false",
                "enable_infra2": "false",
                "enable_gyro": "false",
                "enable_accel": "false",
                "rgb_camera.color_profile": color_profile,
                "depth_module.depth_profile": depth_profile,
                "enable_pointcloud": "false",
                "align_depth.enable": "true",
                "initial_reset": "true",
                "publish_tf": "true",
                "publish_odom_tf": "false",
            }.items(),
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="rtabmap_localization_bringup",
                    executable="xfeat_rtabmap_bridge",
                    name="xfeat_rtabmap_bridge",
                    output="screen",
                    parameters=[{
                        "rgb_topic": rgb_topic,
                        "depth_topic": depth_topic,
                        "camera_info_topic": camera_info_topic,
                        "output_rgbd_topic": "/xfeat/rgbd_image",
                        "xfeat_repo_dir": xfeat_repo_dir,
                        "xfeat_weights_path": xfeat_weights_path,
                        "top_k": top_k,
                        "detection_threshold": detection_threshold,
                        "min_score": min_score,
                        "depth_scale": depth_scale,
                        "depth_min_m": depth_min_m,
                        "depth_max_m": depth_max_m,
                        "output_rate_hz": output_rate_hz,
                        "sync_queue_size": 10,
                    }],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(rtabmap_launch),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                        "localization": localization,
                        "database_path": database_path,
                        "initial_pose": initial_pose,
                        "rtabmap_args": rtabmap_args,
                        "frame_id": frame_id,
                        "visual_odometry": "false",
                        "icp_odometry": "false",
                        "subscribe_scan": "false",
                        "subscribe_scan_cloud": "false",
                        "subscribe_rgbd": "true",
                        "rgbd_sync": "false",
                        "rgbd_topic": "/xfeat/rgbd_image",
                        "publish_tf_map": "true",
                        "rviz": start_rviz,
                        "rtabmap_viz": start_rtabmapviz,
                        "approx_sync": "true",
                        "queue_size": "30",
                    }.items(),
                ),
            ],
        ),
    ])
