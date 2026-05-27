#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lidar_global_localize.py -- 基于 kidnapped_robot_finder 的 ORB 匹配全局定位.

完全对齐 find_robot.py 的处理方式: cv2.imread 直接加载 PGM, 同款 scan 渲染, 同款 solve_kidnap 调用.
区别: 自动触发(收到 scan + map 后立即执行), 发布 /initialpose 给 AMCL.
"""

import math
import os
import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from sensor_msgs.msg import LaserScan

# 动态 import kidnapped_robot_finder
import sys
_base = os.path.dirname(os.path.abspath(__file__))
for _ in range(6):
    _base = os.path.dirname(_base)
_krf_path = os.path.join(_base, "src", "kidnapped_robot_finder")
if os.path.isdir(_krf_path) and _krf_path not in sys.path:
    sys.path.insert(0, _krf_path)
from global_localizer import kidnap_solver


def _quat_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class LidarGlobalLocalize(Node):
    def __init__(self):
        super().__init__("lidar_global_localize")

        # ---- 参数, 对齐 find_robot.py load_parameters ----
        self.declare_parameter("map_file_path", "")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("max_iterations", 30)
        self.declare_parameter("stop_search_threshold_f1", 50.0)
        self.declare_parameter("lidar_max_range", 8.0)
        self.declare_parameter("map_resolution", 0.05)
        # 兼容旧参数名 map_yaml_path, 有则优先
        self.declare_parameter("map_yaml_path", "")

        map_yaml = str(self.get_parameter("map_yaml_path").value)
        map_file = str(self.get_parameter("map_file_path").value)

        if map_yaml and not map_file:
            map_file = self._pgm_from_yaml(map_yaml)
            if map_file:
                self._map_yaml_path = map_yaml
            else:
                self.get_logger().error(f"无法从 YAML 定位 PGM, fallback 到空路径")
        elif map_file:
            self._map_yaml_path = ""
        else:
            self.get_logger().fatal("map_file_path 或 map_yaml_path 至少提供一个")
            raise RuntimeError("缺少地图路径参数")

        self._map_file_path = map_file

        self._scan_topic = str(self.get_parameter("scan_topic").value)
        self._max_iter = int(self.get_parameter("max_iterations").value)
        self._stop_thresh = float(self.get_parameter("stop_search_threshold_f1").value)
        self._max_range = float(self.get_parameter("lidar_max_range").value)
        self._map_resolution = float(self.get_parameter("map_resolution").value)
        self._map_origin = (0.0, 0.0)

        self._map_image = None
        self._map_ready = False
        self._scan_msg = None
        self._scan_image = None
        self._min_distance = None
        self._localized = False
        self._scan_count = 0

        # ---- 地图加载: 对齐 find_robot.py load_map_file ----
        self._load_map_file()

        # ---- 扫描图尺寸: 对齐 find_robot.py __init__ ----
        self._image_size = int((2 * self._max_range) / self._map_resolution)
        self._origin_offset = int(self._max_range / self._map_resolution)

        self._scan_sub = self.create_subscription(
            LaserScan, self._scan_topic, self._scan_callback, 10)
        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)

        # AMCL 是 lifecycle 节点, 启动有延迟, 需要重复发布确保它收到
        self._pending_pose = None   # (x, y, yaw)
        self._repub_count = 0
        self._repub_timer = self.create_timer(0.5, self._repub_initial_pose)

        self.get_logger().info(
            f"kidnapped_robot_finder ORB 匹配定位已就绪, 地图: {self._map_file_path}")

    # ============== 地图(对齐 find_robot.py load_map_file) ==============

    @staticmethod
    def _pgm_from_yaml(yaml_path):
        """从 YAML 里读 image 字段, 返回 PGM 绝对路径."""
        if not yaml_path or not os.path.exists(yaml_path):
            return ""
        yaml_dir = os.path.dirname(os.path.abspath(yaml_path))
        with open(yaml_path, "r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)
        pgm_rel = meta.get("image", "")
        if not pgm_rel:
            return ""
        return os.path.join(yaml_dir, pgm_rel)

    def _load_map_file(self):
        map_path = self._map_file_path
        self.get_logger().info(f"加载地图: {map_path}")

        if not os.path.exists(map_path):
            self.get_logger().error(f"地图文件不存在: {map_path}")
            return

        # 完全对齐 find_robot.py: cv2.imread 直接读
        self._map_image = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
        if self._map_image is None:
            self.get_logger().error(f"无法读取地图图片: {map_path}")
            return

        # 尝试读取 YAML 获取 origin / resolution (对齐 find_robot.py)
        yaml_path = self._map_yaml_path
        if not yaml_path:
            yaml_path = os.path.splitext(map_path)[0] + ".yaml"
        if not os.path.exists(yaml_path):
            yaml_path = os.path.splitext(map_path)[0] + ".yml"

        if os.path.exists(yaml_path):
            with open(yaml_path, "r", encoding="utf-8") as f:
                meta = yaml.safe_load(f)
            origin = meta.get("origin", [0.0, 0.0, 0.0])
            self._map_origin = (float(origin[0]), float(origin[1]))
            self._map_resolution = float(meta.get("resolution", self._map_resolution))
            self.get_logger().info(
                f"从 YAML 读取: origin={self._map_origin}, resolution={self._map_resolution}")

        self._map_ready = True
        self.get_logger().info(
            f"地图已加载: {self._map_image.shape[1]}×{self._map_image.shape[0]}")

        if self._scan_image is not None and not self._localized:
            self._run_localization()

    # ============== 激光(完全对齐 find_robot.py scan_callback) ==============

    def _scan_callback(self, msg: LaserScan):
        if self._localized:
            return
        self._scan_msg = msg

        image = np.zeros((self._image_size, self._image_size), dtype=np.uint8)
        min_distance = math.inf

        for i, range_val in enumerate(msg.ranges):
            if 0 < range_val < self._max_range:
                angle = msg.angle_min + i * msg.angle_increment
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                px = int((x / self._map_resolution) + self._origin_offset)
                py = int((y / self._map_resolution) + self._origin_offset)
                min_distance = min(min_distance, range_val)
                cv2.circle(image, (px, py), radius=1, color=255, thickness=-1)

        self._scan_image = image
        self._min_distance = min_distance
        self._scan_count += 1

        if self._map_ready and self._scan_count >= 3:
            self._run_localization()

    # ============== 定位(完全对齐 find_robot.py global_localization_callback) ==============

    def _run_localization(self):
        self._localized = True

        scan = self._scan_msg
        self.get_logger().info("渲染雷达图像...")
        self.get_logger().info(
            f"最小距离: {self._min_distance:.2f}m, 调用 kidnap_solver...")

        try:
            result = kidnap_solver.solve_kidnap(
                self._scan_image,
                self._map_image,
                self._min_distance,
                map_resolution=self._map_resolution,
                map_origin=self._map_origin,
                max_iterations=self._max_iter,
                stop_search_threshold=self._stop_thresh,
                lidar_range=self._max_range,
            )
            if result is None:
                self.get_logger().error("定位失败: 未找到候选位姿")
                return
            x, y, yaw, f1 = result
            # 补偿 cv2.flip(scan, 1) 造成的 180° yaw 偏移
            yaw = math.atan2(math.sin(yaw + math.pi), math.cos(yaw + math.pi))
        except Exception as e:
            self.get_logger().error(f"定位失败: {e}")
            return

        self.get_logger().info(
            f"定位完成: x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}° F1={f1:.1f}"
        )
        self._publish_initial_pose(x, y, yaw)

    # ============== 发布 ==============

    def _publish_initial_pose(self, x, y, yaw):
        self._pending_pose = (x, y, yaw)
        self._repub_count = 0

    def _repub_initial_pose(self):
        if self._pending_pose is None:
            return
        x, y, yaw = self._pending_pose
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = _quat_from_yaw(yaw)
        msg.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.03,
        ]
        self._pose_pub.publish(msg)
        self._repub_count += 1
        self.get_logger().info(
            f"已发布 /initialpose 给 AMCL ({self._repub_count}/5)")
        if self._repub_count >= 5:
            self._pending_pose = None
            self._repub_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = LidarGlobalLocalize()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
