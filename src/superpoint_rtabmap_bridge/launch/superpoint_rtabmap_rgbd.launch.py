#!/usr/bin/env python3

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    try:
        realsense_launch = os.path.join(
            get_package_share_directory("realsense2_camera"),
            "launch",
            "rs_launch.py",
        )
    except PackageNotFoundError:
        return LaunchDescription([
            LogInfo(msg="realsense2_camera is not installed. Cannot start D435 camera with SuperPoint + RTAB-Map launch."),
        ])

    bridge_node = Node(
        package="superpoint_rtabmap_bridge",
        executable="superpoint_rtabmap_bridge_node",
        name="superpoint_rtabmap_bridge_node",
        output="screen",
        parameters=[{
            "rgb_topic": LaunchConfiguration("rgb_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "output_rgbd_topic": LaunchConfiguration("output_rgbd_topic"),
            "superpoint_model_file": LaunchConfiguration("superpoint_model_file"),
            "max_features": LaunchConfiguration("max_features"),
            "pyramid_levels": LaunchConfiguration("pyramid_levels"),
            "scale_factor": LaunchConfiguration("scale_factor"),
            "confidence_threshold": LaunchConfiguration("confidence_threshold"),
            "depth_scale": LaunchConfiguration("depth_scale"),
            "depth_min_m": LaunchConfiguration("depth_min_m"),
            "depth_max_m": LaunchConfiguration("depth_max_m"),
            "sync_queue_size": LaunchConfiguration("sync_queue_size"),
            "force_gray": LaunchConfiguration("force_gray"),
            "output_rate_hz": LaunchConfiguration("output_rate_hz"),
        }],
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("rtabmap_launch"),
                "launch",
                "rtabmap.launch.py",
            ])
        ]),
        launch_arguments={
            "subscribe_rgbd": "true",
            "rgbd_sync": "false",
            "rgbd_topic": LaunchConfiguration("output_rgbd_topic"),
            "visual_odometry": LaunchConfiguration("visual_odometry"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "subscribe_scan": LaunchConfiguration("subscribe_scan"),
            "scan_topic": LaunchConfiguration("scan_topic"),
            "localization": LaunchConfiguration("localization"),
            "database_path": LaunchConfiguration("database_path"),
            "frame_id": LaunchConfiguration("frame_id"),
            "rtabmap_args": LaunchConfiguration("rtabmap_args"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "rviz": LaunchConfiguration("start_rviz"),
            "rtabmap_viz": LaunchConfiguration("start_rtabmapviz"),
            "queue_size": LaunchConfiguration("rtabmap_queue_size"),
            "approx_sync": LaunchConfiguration("rtabmap_approx_sync"),
        }.items(),
    )

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
        DeclareLaunchArgument("output_rgbd_topic", default_value="/superpoint/rgbd_image"),
        DeclareLaunchArgument(
            "superpoint_model_file",
            default_value=os.path.expanduser("~/project/sp_orb_slam_localization/vendor/superpoint_orb_slam3/weights/superpoint.ts"),
        ),
        DeclareLaunchArgument("max_features", default_value="400"),
        DeclareLaunchArgument("pyramid_levels", default_value="1"),
        DeclareLaunchArgument("scale_factor", default_value="1.2"),
        DeclareLaunchArgument("confidence_threshold", default_value="0.001"),
        DeclareLaunchArgument("depth_scale", default_value="0.001"),
        DeclareLaunchArgument("depth_min_m", default_value="0.15"),
        DeclareLaunchArgument("depth_max_m", default_value="2.0"),
        DeclareLaunchArgument("sync_queue_size", default_value="10"),
        DeclareLaunchArgument("force_gray", default_value="true"),
        DeclareLaunchArgument("output_rate_hz", default_value="1.0"),
        DeclareLaunchArgument("localization", default_value="false"),
        DeclareLaunchArgument("database_path", default_value="~/.ros/rtabmap_superpoint.db"),
        DeclareLaunchArgument("frame_id", default_value="camera_link"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("visual_odometry", default_value="true"),
        DeclareLaunchArgument("subscribe_scan", default_value="false"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("rtabmap_args", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_rviz", default_value="false"),
        DeclareLaunchArgument("start_rtabmapviz", default_value="true"),
        DeclareLaunchArgument("rtabmap_queue_size", default_value="10"),
        DeclareLaunchArgument("rtabmap_approx_sync", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            condition=IfCondition(LaunchConfiguration("start_camera")),
            launch_arguments={
                "serial_no": LaunchConfiguration("serial_no"),
                "camera_name": LaunchConfiguration("camera_name"),
                "camera_namespace": LaunchConfiguration("camera_namespace"),
                "enable_color": "true",
                "enable_depth": "true",
                "enable_infra1": "false",
                "enable_infra2": "false",
                "enable_gyro": "false",
                "enable_accel": "false",
                "rgb_camera.color_profile": LaunchConfiguration("color_profile"),
                "depth_module.depth_profile": LaunchConfiguration("depth_profile"),
                "enable_pointcloud": "false",
                "align_depth.enable": "true",
                "initial_reset": "true",
                "publish_tf": "true",
                "publish_odom_tf": "false",
            }.items(),
        ),
        bridge_node,
        rtabmap_launch,
    ])
