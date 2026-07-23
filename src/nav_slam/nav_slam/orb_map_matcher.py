#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
orb_map_matcher.py -- 持续 ORB 扫描-地图匹配, 周期性纠正里程计漂移。

将当前激光扫描渲染为图像, 与地图的局部区域进行 ORB 特征匹配,
获得机器人在 map 帧下的绝对位姿, 转换为局部修正增量发布给 odom_fusion_node。
"""

import math
import os
import sys
from typing import Optional

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node, Timer
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool

# ---- 复用 kidnapped_robot_finder ----
_base = os.path.dirname(os.path.abspath(__file__))
for _ in range(6):
    _base = os.path.dirname(_base)
_krf_path = os.path.join(_base, "src", "kidnapped_robot_finder")
if os.path.isdir(_krf_path) and _krf_path not in sys.path:
    sys.path.insert(0, _krf_path)
from global_localizer import kidnap_solver


def _yaw_from_quaternion(q: Quaternion) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class OrbMapMatcher(Node):
    def __init__(self) -> None:
        super().__init__("orb_map_matcher")

        self.declare_parameter("map_yaml_path", "")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/localized_odom")
        self.declare_parameter("delta_odom_topic", "/orb/delta_odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("match_period_sec", 2.0)
        self.declare_parameter("lidar_max_range", 8.0)
        self.declare_parameter("map_resolution", 0.05)
        self.declare_parameter("max_iterations", 50)
        self.declare_parameter("min_f1_score", 30.0)
        self.declare_parameter("correction_gain_xy", 0.3)
        self.declare_parameter("correction_gain_yaw", 0.3)

        map_yaml = str(self.get_parameter("map_yaml_path").value)
        if not map_yaml:
            self.get_logger().fatal("map_yaml_path 必须提供")
            raise RuntimeError("缺少 map_yaml_path")

        self._scan_topic = str(self.get_parameter("scan_topic").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._delta_topic = str(self.get_parameter("delta_odom_topic").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._period = float(self.get_parameter("match_period_sec").value)
        self._max_range = float(self.get_parameter("lidar_max_range").value)
        self._map_resolution = float(self.get_parameter("map_resolution").value)
        self._max_iter = int(self.get_parameter("max_iterations").value)
        self._min_f1 = float(self.get_parameter("min_f1_score").value)
        self._gain_xy = float(self.get_parameter("correction_gain_xy").value)
        self._gain_yaw = float(self.get_parameter("correction_gain_yaw").value)

        # ---- 加载地图 ----
        self._map_image, self._map_origin, self._map_pose_offset = self._load_map(map_yaml)
        self._image_size = int(2.0 * self._max_range / self._map_resolution)
        self._origin_offset = int(self._max_range / self._map_resolution)

        # ---- 状态 ----
        self._latest_scan: Optional[LaserScan] = None
        self._latest_odom: Optional[Odometry] = None
        self._match_count = 0
        self._success_count = 0
        self._enabled = True

        # ---- 订阅 ----
        self._scan_sub = self.create_subscription(
            LaserScan, self._scan_topic, self._scan_cb, 10)
        self._odom_sub = self.create_subscription(
            Odometry, self._odom_topic, self._odom_cb, 10)

        # ---- 发布 ----
        self._delta_pub = self.create_publisher(Odometry, self._delta_topic, 10)

        # ---- 定时匹配 ----
        self._timer = self.create_timer(self._period, self._match_timer_cb)

        # ---- 暂停/恢复服务 (重定位期间暂停，避免新旧TF冲突) ----
        self._enable_srv = self.create_service(
            SetBool, "/enable_orb_matcher", self._enable_cb)

        self.get_logger().info(
            f"ORB 持续匹配就绪: 每 {self._period}s, "
            f"max_iter={self._max_iter}, min_f1={self._min_f1}, "
            f"gain_xy={self._gain_xy}, gain_yaw={self._gain_yaw}"
        )

    # ==================== 地图 ====================

    @staticmethod
    def _pgm_from_yaml(yaml_path: str) -> str:
        yaml_dir = os.path.dirname(os.path.abspath(yaml_path))
        with open(yaml_path, "r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)
        pgm_rel = meta.get("image", "")
        return os.path.join(yaml_dir, pgm_rel) if pgm_rel else ""

    def _load_map(
        self, map_yaml: str
    ) -> tuple[np.ndarray, tuple[float, float], tuple[float, float]]:
        pgm_path = self._pgm_from_yaml(map_yaml)
        if not os.path.exists(pgm_path):
            raise FileNotFoundError(f"地图不存在: {pgm_path}")
        map_img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if map_img is None:
            raise RuntimeError(f"无法读取地图: {pgm_path}")

        with open(map_yaml, "r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)
        origin = (float(meta["origin"][0]), float(meta["origin"][1]))
        pose_offset = tuple(float(value) for value in meta.get(
            "localization_pose_offset", [0.0, 0.0]))
        self.get_logger().info(
            f"地图加载: {map_img.shape[1]}×{map_img.shape[0]}, "
            f"origin={origin}, resolution={self._map_resolution}, "
            f"pose_offset={pose_offset}"
        )
        return map_img, origin, pose_offset

    # ==================== 回调 ====================

    def _scan_cb(self, msg: LaserScan) -> None:
        self._latest_scan = msg

    def _odom_cb(self, msg: Odometry) -> None:
        self._latest_odom = msg

    # ==================== 暂停/恢复 ====================

    def _enable_cb(self, request, response):
        self._enabled = request.data
        response.success = True
        response.message = (
            f"ORB matcher {'已恢复' if self._enabled else '已暂停'}")
        self.get_logger().info(response.message)
        return response

    # ==================== 定时匹配 ====================

    def _match_timer_cb(self) -> None:
        if not self._enabled:
            return
        if self._latest_scan is None or self._latest_odom is None:
            return

        self._match_count += 1

        # 1. 渲染扫描图像
        scan_img, min_dist = self._render_scan(self._latest_scan)

        # 2. 裁剪地图局部区域
        map_crop, crop_origin_m = self._crop_map_around_pose(self._latest_odom)

        # 3. ORB 匹配
        try:
            result = kidnap_solver.solve_kidnap(
                scan_img, map_crop, min_dist,
                map_resolution=self._map_resolution,
                map_origin=crop_origin_m,
                map_pose_offset=self._map_pose_offset,
                max_iterations=self._max_iter,
                stop_search_threshold=self._min_f1,
                lidar_range=self._max_range,
            )
        except Exception as exc:
            self.get_logger().warn(f"ORB 匹配异常: {exc}")
            return

        if result is None:
            self.get_logger().info(
                f"[{self._match_count}] 匹配失败 (无候选)",
                throttle_duration_sec=2.0,
            )
            return

        map_x, map_y, map_yaw, f1 = result
        self._success_count += 1

        # 4. 计算修正增量
        delta = self._compute_delta(map_x, map_y, map_yaw, self._latest_odom)
        if delta is None:
            return

        # 5. 发布
        self._publish_delta(self._latest_scan.header, delta)

        self.get_logger().info(
            f"[{self._match_count}] 匹配成功: F1={f1:.1f}% "
            f"map=({map_x:.2f},{map_y:.2f},{math.degrees(map_yaw):.1f}°) "
            f"corr=({delta['dx']:.3f},{delta['dy']:.3f},{math.degrees(delta['dyaw']):.1f}°) "
            f"成功率={self._success_count}/{self._match_count}"
        )

    # ==================== 扫描渲染 ====================

    def _render_scan(self, msg: LaserScan) -> tuple[np.ndarray, float]:
        """将 LaserScan 渲染为 320×320 灰度图像, 返回 (image, min_distance_m)."""
        image = np.zeros((self._image_size, self._image_size), dtype=np.uint8)
        min_dist = math.inf
        for i, r in enumerate(msg.ranges):
            if not (0.0 < r < self._max_range):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            px = int(x / self._map_resolution + self._origin_offset)
            py = int(y / self._map_resolution + self._origin_offset)
            if 0 <= px < self._image_size and 0 <= py < self._image_size:
                min_dist = min(min_dist, r)
                cv2.circle(image, (px, py), radius=1, color=255, thickness=-1)
        return image, float(min_dist)

    # ==================== 地图裁剪 ====================

    def _crop_map_around_pose(
        self, odom: Odometry
    ) -> tuple[np.ndarray, tuple[float, float]]:
        """
        裁剪当前估计位姿周围的地图区域。

        返回:
            map_crop: 裁剪后的灰度图
            crop_origin_m: 裁剪区域左下角在 map 帧下的坐标 (x, y) 米
        """
        px = (odom.pose.pose.position.x - self._map_origin[0]) / self._map_resolution
        py = (odom.pose.pose.position.y - self._map_origin[1]) / self._map_resolution
        # map 图像坐标系: y 轴翻转
        py_img = self._map_image.shape[0] - int(py)

        # 裁剪半边长 (像素): 覆盖扫描范围 + 余量
        half_px = int(self._max_range / self._map_resolution) + 20  # 8m + 1m margin
        x1 = max(0, int(px) - half_px)
        x2 = min(self._map_image.shape[1], int(px) + half_px)
        y1 = max(0, py_img - half_px)
        y2 = min(self._map_image.shape[0], py_img + half_px)

        map_crop = self._map_image[y1:y2, x1:x2]

        # 裁剪区域左下角在 map 帧下的世界坐标
        crop_origin_m = (
            self._map_origin[0] + x1 * self._map_resolution,
            self._map_origin[1] + (self._map_image.shape[0] - y2) * self._map_resolution,
        )

        return map_crop, crop_origin_m

    # ==================== 修正量计算 ====================

    def _compute_delta(
        self,
        map_x: float, map_y: float, map_yaw: float,
        odom: Odometry,
    ) -> Optional[dict]:
        """根据 ORB 匹配结果和当前估计位姿计算局部修正增量."""
        est_x = odom.pose.pose.position.x
        est_y = odom.pose.pose.position.y
        est_yaw = _yaw_from_quaternion(odom.pose.pose.orientation)

        # map 帧下的位姿误差
        err_x = map_x - est_x
        err_y = map_y - est_y
        err_yaw = _wrap_angle(map_yaw - est_yaw)

        # 转为 base_footprint 帧的局部增量 (P 控制器)
        c = math.cos(est_yaw)
        s = math.sin(est_yaw)
        local_dx = self._gain_xy * (c * err_x + s * err_y)
        local_dy = self._gain_xy * (-s * err_x + c * err_y)
        local_dyaw = self._gain_yaw * err_yaw

        return {"dx": local_dx, "dy": local_dy, "dyaw": local_dyaw}

    # ==================== 发布 ====================

    def _publish_delta(self, header, delta: dict) -> None:
        odom = Odometry()
        odom.header = header
        odom.header.frame_id = self._base_frame
        odom.child_frame_id = f"{self._base_frame}_delta"
        odom.pose.pose.position.x = delta["dx"]
        odom.pose.pose.position.y = delta["dy"]
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = _quaternion_from_yaw(delta["dyaw"])
        self._delta_pub.publish(odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OrbMapMatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
