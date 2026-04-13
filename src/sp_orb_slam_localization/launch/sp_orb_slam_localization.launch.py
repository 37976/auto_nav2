from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = FindPackageShare("sp_orb_slam_localization")
    return LaunchDescription([
        DeclareLaunchArgument("sensor_mode", default_value="rgbd"),
        DeclareLaunchArgument("image_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/camera/camera/aligned_depth_to_color/image_raw",
        ),
        DeclareLaunchArgument("settings_file", default_value=""),
        DeclareLaunchArgument(
            "vocabulary_file",
            default_value=PathJoinSubstitution(
                [package_share, "vendor", "Vocabulary", "ORBvoc.txt"]
            ),
        ),
        DeclareLaunchArgument(
            "superpoint_model_file",
            default_value=PathJoinSubstitution(
                [package_share, "vendor", "weights", "superpoint.ts"]
            ),
        ),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("child_frame", default_value="camera_link"),
        DeclareLaunchArgument("publish_tf", default_value="false"),
        DeclareLaunchArgument("max_depth_age_sec", default_value="1.0"),
        DeclareLaunchArgument("localization_only", default_value="true"),
        Node(
            package="sp_orb_slam_localization",
            executable="superpoint_localization_node",
            name="superpoint_localization_node",
            output="screen",
            parameters=[{
                "sensor_mode": LaunchConfiguration("sensor_mode"),
                "image_topic": LaunchConfiguration("image_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "settings_file": LaunchConfiguration("settings_file"),
                "vocabulary_file": LaunchConfiguration("vocabulary_file"),
                "superpoint_model_file": LaunchConfiguration("superpoint_model_file"),
                "map_frame": LaunchConfiguration("map_frame"),
                "child_frame": LaunchConfiguration("child_frame"),
                "publish_tf": LaunchConfiguration("publish_tf"),
                "max_depth_age_sec": LaunchConfiguration("max_depth_age_sec"),
                "localization_only": LaunchConfiguration("localization_only"),
            }],
        ),
    ])
