#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lidar_global_localize.py -- ORB 匹配一次性全局定位, 锁定 map->odom 静态 TF.
对齐 find_robot.py: cv2.imread 加载 PGM, 同款 scan 渲染, 同款 solve_kidnap.
"""

import math
import os
import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from tf2_ros import StaticTransformBroadcaster

import sys
_base = os.path.dirname(os.path.abspath(__file__))
for _ in range(6):
    _base = os.path.dirname(_base)
_krf_path = os.path.join(_base, "src", "kidnapped_robot_finder")
if os.path.isdir(_krf_path) and _krf_path not in sys.path:
    sys.path.insert(0, _krf_path)
from global_localizer import kidnap_solver  # type: ignore[import-unresolved]


class LidarGlobalLocalize(Node):
    def __init__(self):
        super().__init__("lidar_global_localize")

        self.declare_parameter("map_file_path", "")
        self.declare_parameter("map_yaml_path", "")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("max_iterations", 250)
        self.declare_parameter("stop_search_threshold_f1", 50.0)
        self.declare_parameter("lidar_max_range", 8.0)
        self.declare_parameter("map_resolution", 0.05)

        map_yaml = str(self.get_parameter("map_yaml_path").value)
        map_file = str(self.get_parameter("map_file_path").value)

        if map_yaml and not map_file:
            map_file = self._pgm_from_yaml(map_yaml)
            if map_file:
                self._map_yaml_path = map_yaml
            else:
                self.get_logger().error("无法从 YAML 定位 PGM")
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
        self._map_pose_offset = (0.0, 0.0)

        self._map_image = None
        self._map_ready = False
        self._scan_image = None
        self._min_distance = None
        self._scan_count = 0
        self._localized = False
        self._cancel_requested = False
        self._latest_odom = None

        self._load_map_file()

        self._image_size = int((2 * self._max_range) / self._map_resolution)
        self._origin_offset = int(self._max_range / self._map_resolution)

        self._scan_sub = self.create_subscription(
            LaserScan, self._scan_topic, self._scan_callback, 10)
        self._odom_sub = self.create_subscription(
            Odometry, "/localized_odom", self._odom_callback, 10)
        self._tf_broadcaster = StaticTransformBroadcaster(self)

        self._pending_pose = None
        self._repub_count = 0
        self._repub_timer = self.create_timer(0.5, self._publish_map_odom_tf)

        # 重定位服务：允许外部节点在导航到达目标点后触发全局重定位
        self._relocalize_srv = self.create_service(
            Empty, "/trigger_relocalize", self._relocalize_callback)

        # 取消重定位服务：允许外部节点在重定位期间收到新目标时截断定位
        self._cancel_relocalize_srv = self.create_service(
            Empty, "/cancel_relocalize", self._cancel_relocalize_callback)

        # 立即发布默认 TF, 建立 map 帧, 避免 ORB 计算期间系统瘫痪
        self._publish_default_tf()

        self.get_logger().info(
            f"ORB 全局定位已就绪, 地图: {self._map_file_path}"
            ", 服务 /trigger_relocalize, /cancel_relocalize 可用")

    # ============== 地图 ==============

    @staticmethod
    def _pgm_from_yaml(yaml_path):
        if not yaml_path or not os.path.exists(yaml_path):
            return ""
        yaml_dir = os.path.dirname(os.path.abspath(yaml_path))
        with open(yaml_path, "r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)
        pgm_rel = meta.get("image", "")
        return os.path.join(yaml_dir, pgm_rel) if pgm_rel else ""

    def _load_map_file(self):
        map_path = self._map_file_path
        self.get_logger().info(f"加载地图: {map_path}")
        if not os.path.exists(map_path):
            self.get_logger().error(f"地图文件不存在: {map_path}")
            return
        self._map_image = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
        if self._map_image is None:
            self.get_logger().error(f"无法读取地图图片: {map_path}")
            return

        yaml_path = self._map_yaml_path
        if not yaml_path:
            yaml_path = os.path.splitext(map_path)[0] + ".yaml"
        if not os.path.exists(yaml_path):
            yaml_path = os.path.splitext(map_path)[0] + ".yml"
        if os.path.exists(yaml_path):
            with open(yaml_path, "r", encoding="utf-8") as f:
                meta = yaml.safe_load(f)
            origin = meta.get("origin", [0.0, 0.0, 0.0])
            pose_offset = meta.get("localization_pose_offset", [0.0, 0.0])
            self._map_origin = (float(origin[0]), float(origin[1]))
            self._map_pose_offset = (float(pose_offset[0]), float(pose_offset[1]))
            self._map_resolution = float(meta.get("resolution", self._map_resolution))
            self.get_logger().info(
                f"从 YAML 读取: origin={self._map_origin}, resolution={self._map_resolution}, "
                f"pose_offset={self._map_pose_offset}")

        self._map_ready = True
        self.get_logger().info(
            f"地图已加载: {self._map_image.shape[1]}×{self._map_image.shape[0]}")

    # ============== 激光 ==============

    def _scan_callback(self, msg: LaserScan):
        if self._localized or self._cancel_requested:
            return
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

    # ============== 定位 ==============

    def _publish_default_tf(self):
        t = TransformStamped()
        t.header.stamp = rclpy.time.Time(seconds=0, nanoseconds=0).to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(t)
        self.get_logger().info("已发送默认 map->odom TF (0,0,0), 等待 ORB 定位...")

    def _relocalize_callback(self, request, response):
        """外部触发重定位：重置状态，重新收集激光扫描并执行全局定位。"""
        if not self._map_ready:
            self.get_logger().warn("重定位请求被拒绝: 地图未就绪")
            return response

        self.get_logger().info("收到重定位请求, 重置定位状态...")
        self._cancel_requested = False
        self._localized = False
        self._scan_count = 0
        self._scan_image = None
        self._pending_pose = None
        self._repub_count = 0

        # 如果之前的定时器已被取消，重新创建
        if self._repub_timer.is_canceled():
            self._repub_timer = self.create_timer(0.5, self._publish_map_odom_tf)

        self.get_logger().info("重定位状态已重置, 等待激光扫描触发定位...")
        return response

    def _cancel_relocalize_callback(self, request, response):
        """取消正在进行的重定位：停止扫描收集，丢弃定位结果，保持当前 map->odom TF 不变。

        只影响当前正在执行的定位流程（扫描收集 / ORB 求解），不阻止未来的重定位请求。
        """
        self.get_logger().info("收到取消重定位请求")
        if self._localized:
            # 当前未在重定位（可能 trigger 还在异步传输中，尚未处理）
            # 设置取消标志，trigger 到达后会立即清除，但扫描收集和求解会被跳过
            self.get_logger().info("当前未在重定位中, 设置取消标志以防 trigger 即将到达")
        self._cancel_requested = True
        self._localized = True  # 停止扫描收集
        self._scan_count = 0
        self._scan_image = None
        self._pending_pose = None
        self._repub_count = 0
        if not self._repub_timer.is_canceled():
            self._repub_timer.cancel()
        self.get_logger().info("重定位已取消, 保持当前 map->odom TF")
        return response

    def _run_localization(self):
        if self._cancel_requested:
            self.get_logger().info("重定位已被取消, 丢弃当前定位结果")
            return
        self._localized = True
        self.get_logger().info(f"ORB 匹配中... min_dist={self._min_distance:.2f}m")

        try:
            result = kidnap_solver.solve_kidnap(
                self._scan_image, self._map_image, self._min_distance,
                map_resolution=self._map_resolution,
                map_origin=self._map_origin,
                map_pose_offset=self._map_pose_offset,
                max_iterations=self._max_iter,
                stop_search_threshold=self._stop_thresh,
                lidar_range=self._max_range,
            )
            if result is None:
                self.get_logger().error("定位失败: 未找到候选位姿, 将重试")
                self._localized = False
                return
            x, y, yaw, f1 = result
            yaw = math.atan2(math.sin(yaw + math.pi), math.cos(yaw + math.pi))
        except Exception as e:
            self.get_logger().error(f"定位失败: {e}, 将重试")
            self._localized = False
            return

        # 求解器可能耗时较长，再次检查是否已被取消
        if self._cancel_requested:
            self.get_logger().info("重定位已被取消, 丢弃求解结果")
            return

        self.get_logger().info(
            f"定位完成: x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}° F1={f1:.1f}"
        )
        self._pending_pose = (x, y, yaw)
        self._repub_count = 0

    # ============== 发布 ==============

    def _odom_callback(self, msg: Odometry):
        self._latest_odom = msg

    def _publish_map_odom_tf(self):
        if self._pending_pose is None or self._latest_odom is None:
            return
        if self._repub_count >= 5:
            self._repub_timer.cancel()
            return
        map_x, map_y, map_yaw = self._pending_pose
        odom = self._latest_odom
        q = odom.pose.pose.orientation
        odom_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # tf_yaw = map_yaw - odom_yaw
        tf_yaw = math.atan2(math.sin(map_yaw - odom_yaw),
                            math.cos(map_yaw - odom_yaw))

        # StaticTransform 语义: map = R(tf_yaw) * odom + tf_translation
        # 因此 tf_translation = map - R(tf_yaw) * odom
        odom_x = odom.pose.pose.position.x
        odom_y = odom.pose.pose.position.y
        c = math.cos(tf_yaw)
        s = math.sin(tf_yaw)
        rotated_odom_x = c * odom_x - s * odom_y
        rotated_odom_y = s * odom_x + c * odom_y

        t = TransformStamped()
        t.header.stamp = rclpy.time.Time(seconds=0, nanoseconds=0).to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = map_x - rotated_odom_x
        t.transform.translation.y = map_y - rotated_odom_y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(tf_yaw * 0.5)
        t.transform.rotation.w = math.cos(tf_yaw * 0.5)
        self._tf_broadcaster.sendTransform(t)

        self._repub_count += 1
        self.get_logger().info(
            f"已发布 map->odom 静态 TF: x={t.transform.translation.x:.3f} "
            f"y={t.transform.translation.y:.3f} yaw={math.degrees(tf_yaw):.1f}° "
            f"({self._repub_count}/5)"
        )


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
