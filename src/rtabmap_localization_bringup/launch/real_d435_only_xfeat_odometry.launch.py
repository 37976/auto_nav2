#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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

    start_camera = LaunchConfiguration("start_camera")
    serial_no = LaunchConfiguration("serial_no")
    camera_name = LaunchConfiguration("camera_name")
    camera_namespace = LaunchConfiguration("camera_namespace")
    color_profile = LaunchConfiguration("color_profile")
    depth_profile = LaunchConfiguration("depth_profile")
    rgb_topic = LaunchConfiguration("rgb_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    top_k = LaunchConfiguration("top_k")
    detection_threshold = LaunchConfiguration("detection_threshold")
    min_score = LaunchConfiguration("min_score")
    depth_scale = LaunchConfiguration("depth_scale")
    depth_min_m = LaunchConfiguration("depth_min_m")
    depth_max_m = LaunchConfiguration("depth_max_m")
    xfeat_repo_dir = LaunchConfiguration("xfeat_repo_dir")
    xfeat_weights_path = LaunchConfiguration("xfeat_weights_path")
    sync_queue_size = LaunchConfiguration("sync_queue_size")
    odom_topic = LaunchConfiguration("odom_topic")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    camera_frame = LaunchConfiguration("camera_frame")
    publish_tf = LaunchConfiguration("publish_tf")
    match_min_cossim = LaunchConfiguration("match_min_cossim")
    min_pnp_points = LaunchConfiguration("min_pnp_points")
    min_inliers = LaunchConfiguration("min_inliers")
    pnp_reproj_error = LaunchConfiguration("pnp_reproj_error")
    pnp_iterations = LaunchConfiguration("pnp_iterations")

    return LaunchDescription([
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("serial_no", default_value=""),
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("camera_namespace", default_value="camera"),
        DeclareLaunchArgument("color_profile", default_value="640,480,30"),
        DeclareLaunchArgument("depth_profile", default_value="640,480,30"),
        DeclareLaunchArgument("rgb_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info"),
        DeclareLaunchArgument("top_k", default_value="256"),
        DeclareLaunchArgument("detection_threshold", default_value="0.05"),
        DeclareLaunchArgument("min_score", default_value="0.08"),
        DeclareLaunchArgument("depth_scale", default_value="0.001"),
        DeclareLaunchArgument("depth_min_m", default_value="0.2"),
        DeclareLaunchArgument("depth_max_m", default_value="3.0"),
        DeclareLaunchArgument("sync_queue_size", default_value="10"),
        DeclareLaunchArgument("xfeat_repo_dir", default_value=os.path.expanduser("~/project/XFeat")),
        DeclareLaunchArgument("xfeat_weights_path", default_value=""),
        DeclareLaunchArgument("odom_topic", default_value="/xfeat/odom"),
        DeclareLaunchArgument("odom_frame", default_value="xfeat_odom"),
        DeclareLaunchArgument("base_frame", default_value="base_footprint"),
        DeclareLaunchArgument("camera_frame", default_value="camera_link"),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        DeclareLaunchArgument("match_min_cossim", default_value="0.82"),
        DeclareLaunchArgument("min_pnp_points", default_value="12"),
        DeclareLaunchArgument("min_inliers", default_value="10"),
        DeclareLaunchArgument("pnp_reproj_error", default_value="4.0"),
        DeclareLaunchArgument("pnp_iterations", default_value="200"),
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
        Node(
            package="rtabmap_localization_bringup",
            executable="xfeat_rgbd_odometry",
            name="xfeat_rgbd_odometry",
            output="screen",
            parameters=[{
                "rgb_topic": rgb_topic,
                "depth_topic": depth_topic,
                "camera_info_topic": camera_info_topic,
                "xfeat_repo_dir": xfeat_repo_dir,
                "xfeat_weights_path": xfeat_weights_path,
                "top_k": top_k,
                "detection_threshold": detection_threshold,
                "min_score": min_score,
                "depth_scale": depth_scale,
                "min_depth_m": depth_min_m,
                "max_depth_m": depth_max_m,
                "sync_queue_size": sync_queue_size,
                "odom_topic": odom_topic,
                "odom_frame": odom_frame,
                "base_frame": base_frame,
                "camera_frame": camera_frame,
                "publish_tf": publish_tf,
                "match_min_cossim": match_min_cossim,
                "min_pnp_points": min_pnp_points,
                "min_inliers": min_inliers,
                "pnp_reproj_error": pnp_reproj_error,
                "pnp_iterations": pnp_iterations,
            }],
        ),
    ])
