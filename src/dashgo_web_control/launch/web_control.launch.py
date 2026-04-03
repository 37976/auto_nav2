import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_qr_popup_script():
    matches = glob.glob(
        os.path.join(
            get_package_share_directory("dashgo_web_control"),
            "..",
            "..",
            "lib",
            "python*",
            "site-packages",
            "dashgo_web_control",
            "show_web_qr_popup.py",
        )
    )
    if not matches:
        raise FileNotFoundError("Unable to locate show_web_qr_popup.py")
    return matches[0]


def resolve_hotspot_script():
    matches = glob.glob(
        os.path.join(
            get_package_share_directory("dashgo_web_control"),
            "..",
            "..",
            "lib",
            "python*",
            "site-packages",
            "dashgo_web_control",
            "hotspot_manager.py",
        )
    )
    if not matches:
        raise FileNotFoundError("Unable to locate hotspot_manager.py")
    return matches[0]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8080"),
            DeclareLaunchArgument("image_topic", default_value="/camera/camera/color/image_raw"),
            DeclareLaunchArgument("scan_topic", default_value="/scan_filtered"),
            DeclareLaunchArgument("pointcloud_topic", default_value=""),
            DeclareLaunchArgument("start_hotspot", default_value="false"),
            DeclareLaunchArgument("hotspot_connection_name", default_value="dashgo-hotspot"),
            DeclareLaunchArgument("hotspot_ssid", default_value="Dashgo-Robot"),
            DeclareLaunchArgument("hotspot_password", default_value="dashgo12345"),
            DeclareLaunchArgument("hotspot_ifname", default_value=""),
            TimerAction(
                period=0.3,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "python3",
                            resolve_hotspot_script(),
                            "--connection-name",
                            LaunchConfiguration("hotspot_connection_name"),
                            "--ssid",
                            LaunchConfiguration("hotspot_ssid"),
                            "--password",
                            LaunchConfiguration("hotspot_password"),
                            "--ifname",
                            LaunchConfiguration("hotspot_ifname"),
                        ],
                        output="screen",
                    ),
                ],
                condition=IfCondition(LaunchConfiguration("start_hotspot")),
            ),
            Node(
                package="dashgo_web_control",
                executable="web_control_node",
                name="dashgo_web_control",
                output="screen",
                parameters=[
                    {
                        "host": LaunchConfiguration("host"),
                        "port": LaunchConfiguration("port"),
                        "image_topic": LaunchConfiguration("image_topic"),
                        "scan_topic": LaunchConfiguration("scan_topic"),
                        "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
                    }
                ],
            ),
            TimerAction(
                period=2.3,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "python3",
                            resolve_qr_popup_script(),
                        ],
                        additional_env={
                            "DASHGO_WEB_UI_HOST": LaunchConfiguration("host"),
                            "DASHGO_WEB_UI_PORT": LaunchConfiguration("port"),
                            "DASHGO_HOTSPOT_ENABLED": LaunchConfiguration("start_hotspot"),
                            "DASHGO_HOTSPOT_SSID": LaunchConfiguration("hotspot_ssid"),
                            "DASHGO_HOTSPOT_PASSWORD": LaunchConfiguration("hotspot_password"),
                        },
                        output="screen",
                    ),
                ],
            ),
        ]
    )
