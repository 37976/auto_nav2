#!/usr/bin/env python3
import time
import rclpy
from std_srvs.srv import Empty
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import os
import numpy as np
import cv2
from global_localizer import kidnap_solver as ks
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import yaml

class LaserScanFilter(Node):

    def __init__(self):
        super().__init__('global_localizer_node')

        self.max_range = 8.0  # max range of lidar in meters
        self.resolution = 0.05  # 5 cm per pixel

        self.min_distance = None
        self.scan_image = None
        self.map_image = None
        self.config_data = None

        self.scan_topic = "/scan" # default scan topic
        self.map_file_path = ""
        self.map_origin = (0.0, 0.0)

        self.max_iterations = 30
        self.stop_search_threshold_f1 = 50

        self.lidar_max_range = 8.0
        self.map_resolution = 0.05 # m/px

        self.load_parameters()
        self.load_map_file()

        self.image_size = int((2 * self.max_range) / self.resolution)  # image width and height in pixels
        self.origin_offset = int(self.max_range / self.resolution)  # origin offset in pixels
        
        self.subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        self.srv = self.create_service(Empty, 'global_localization_srv', self.global_localization_callback)
        self.get_logger().info('Global Localization Service is ready.')


    def scan_callback(self, msg: LaserScan):
        # Initialize a black image
        image = np.zeros((self.image_size, self.image_size), dtype=np.uint8)
        
        min_distance = math.inf
        # Convert lidar points to pixels and draw on the image
        for i, range_val in enumerate(msg.ranges):
            if 0 < range_val < self.max_range:  # Ignore invalid or out-of-range values
                angle = msg.angle_min + i * msg.angle_increment
                # Convert polar coordinates (range, angle) to Cartesian (x, y)
                x = range_val * np.cos(angle)
                y = range_val * np.sin(angle)
                
                # Convert from meters to pixels
                px = int((x / self.resolution) + self.origin_offset)
                py = int((y / self.resolution) + self.origin_offset)
                min_distance = min(min_distance, range_val)
                # Draw the point on the image as a white circle with radius 1
                cv2.circle(image, (px, py), radius=1, color=255, thickness=-1)

        #print(f"Min distance: {min_distance}")

        self.scan_image = image.copy()
        self.min_distance = min_distance

    def load_config_file(self):
        package_share_directory = get_package_share_directory('global_localizer')
        yaml_file_path = Path(package_share_directory) / 'config' / 'config.yaml'
        try:
            with open(yaml_file_path, 'r') as file:
                self.config_data = yaml.safe_load(file)
        except Exception as e:
            self.get_logger().error(f"Error reading YAML file: {e}")
            exit()

    def load_parameters(self):
        # ROS 参数优先，config.yaml 作为 fallback
        self.load_config_file()

        self.declare_parameter("map_file_path", self.config_data.get("map_file_path", ""))
        self.declare_parameter("scan_topic", self.config_data.get("scan_topic", "/scan"))
        self.declare_parameter("max_iterations", self.config_data.get("max_iterations", 30))
        self.declare_parameter("stop_search_threshold_f1", self.config_data.get("stop_search_threshold_f1", 50))
        self.declare_parameter("lidar_max_range", self.config_data.get("lidar_max_range", 8.0))
        self.declare_parameter("map_resolution", self.config_data.get("map_resolution", 0.05))

        self.map_file_path = str(self.get_parameter("map_file_path").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.max_iterations = int(self.get_parameter("max_iterations").value)
        self.stop_search_threshold_f1 = float(self.get_parameter("stop_search_threshold_f1").value)
        self.lidar_max_range = float(self.get_parameter("lidar_max_range").value)
        self.map_resolution = float(self.get_parameter("map_resolution").value)

        self.get_logger().info(f"Scan topic: {self.scan_topic}")
        self.get_logger().info(f"Map file path: {self.map_file_path}")
        self.get_logger().info(f"Max iterations: {self.max_iterations}")
        self.get_logger().info(f"Stop search threshold: {self.stop_search_threshold_f1}")
        self.get_logger().info(f"Lidar max range: {self.lidar_max_range}")
        self.get_logger().info(f"Map resolution: {self.map_resolution}")

    def load_map_file(self):
        map_file_path = self.map_file_path
        self.get_logger().info(f"Map file path: {map_file_path}")

        if not Path(map_file_path).exists():
            self.get_logger().error(f"Map file not found: {map_file_path}")
            exit()

        self.map_image = cv2.imread(map_file_path, cv2.IMREAD_GRAYSCALE)

        # 尝试读取同名的 YAML 获取 origin / resolution
        yaml_path = os.path.splitext(map_file_path)[0] + ".yaml"
        if not os.path.exists(yaml_path):
            # 某些地图用 .yml 后缀
            yaml_path = os.path.splitext(map_file_path)[0] + ".yml"
        if os.path.exists(yaml_path):
            with open(yaml_path, "r", encoding="utf-8") as f:
                meta = yaml.safe_load(f)
            origin = meta.get("origin", [0.0, 0.0, 0.0])
            self.map_origin = (float(origin[0]), float(origin[1]))
            self.map_resolution = float(meta.get("resolution", self.map_resolution))
            self.get_logger().info(
                f"从 YAML 读取: origin={self.map_origin}, resolution={self.map_resolution}"
            )
        else:
            self.get_logger().warn("未找到对应 YAML，origin 默认为 (0,0)")

    def global_localization_callback(self, request, response):
        self.get_logger().info('Global Localization Service triggered.')
        if self.min_distance is not None:
            self.get_logger().info('Found last laser-scan data. Matching in progress....')
            
            # Solve the kidnap problem
            result = ks.solve_kidnap(self.scan_image, self.map_image, self.min_distance,
                                     map_resolution=self.map_resolution,
                                     map_origin=self.map_origin,
                                     max_iterations=self.max_iterations,
                                     stop_search_threshold=self.stop_search_threshold_f1,
                                     lidar_range=self.lidar_max_range)
            if result is not None:
                x, y, yaw, f1 = result
                self.get_logger().info(
                    f"定位结果: x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}° F1={f1:.1f}"
                )
        else:
            self.get_logger().error('No laser-scan data found. Please make-sure there are laser scans available, and retry.')
        return response

def main(args=None):
    print("\n\n** Starting the localization service!!**\n\n")
    rclpy.init(args=args)
    laser_scan_filter = LaserScanFilter()
    rclpy.spin(laser_scan_filter)
    laser_scan_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

