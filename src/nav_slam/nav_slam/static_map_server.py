#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
static_map_server.py — 将静态 PGM+YAML 地图发布到 /map 话题。
用于给 AMCL 提供干净的全局定位地图（不含动态障碍物）。
"""

import os

import cv2
import numpy as np
import rclpy
import yaml
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node


class StaticMapServer(Node):
    def __init__(self) -> None:
        super().__init__("static_map_server")

        self.declare_parameter("map_yaml_path", "")
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("frame_id", "map")

        self._map_yaml_path = str(self.get_parameter("map_yaml_path").value)
        self._publish_period_sec = max(0.1, float(self.get_parameter("publish_period_sec").value))
        self._frame_id = str(self.get_parameter("frame_id").value)

        if not self._map_yaml_path:
            self.get_logger().fatal("map_yaml_path 参数为空，无法加载地图。")
            raise RuntimeError("map_yaml_path is required")

        self._grid_msg = self._load_map(self._map_yaml_path)
        self._pub = self.create_publisher(OccupancyGrid, "/map", rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value)
        self._timer = self.create_timer(self._publish_period_sec, self._publish_map)

        self.get_logger().info(
            f"静态地图已加载: {self._grid_msg.info.width}×{self._grid_msg.info.height} "
            f"@ {self._grid_msg.info.resolution:.3f} m/格"
        )

    def _load_map(self, yaml_path: str) -> OccupancyGrid:
        yaml_dir = os.path.dirname(os.path.abspath(yaml_path))

        with open(yaml_path, "r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)

        image_rel = meta["image"]
        pgm_path = os.path.join(yaml_dir, image_rel)
        image = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise FileNotFoundError(f"无法读取地图图片: {pgm_path}")

        resolution = float(meta["resolution"])
        origin = meta.get("origin", [0.0, 0.0, 0.0])
        negate = meta.get("negate", 0)
        free_thresh = float(meta.get("free_thresh", 0.196))
        occupied_thresh = float(meta.get("occupied_thresh", 0.65))
        mode = meta.get("mode", "trinary")

        height, width = image.shape

        if mode == "trinary":
            data = -1 * np.ones((height, width), dtype=np.int8)
            if negate:
                data[(image / 255.0) <= free_thresh] = 0
                data[(image / 255.0) >= occupied_thresh] = 100
            else:
                data[(image / 255.0) >= occupied_thresh] = 100
                data[(image / 255.0) <= free_thresh] = 0
        elif mode == "scale":
            data = np.zeros((height, width), dtype=np.int8)
            if negate:
                free_mask = (image / 255.0) <= free_thresh
                data[free_mask] = 0
                data[~free_mask] = ((1.0 - image[~free_mask] / 255.0) * 100).astype(np.int8).clip(0, 100)
            else:
                occupied_mask = (image / 255.0) >= occupied_thresh
                data[occupied_mask] = 100
                data[~occupied_mask] = ((image[~occupied_mask] / 255.0) * 100).astype(np.int8).clip(0, 100)
        else:  # raw
            data = image.astype(np.int8).copy()

        grid = OccupancyGrid()
        grid.header.frame_id = self._frame_id
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info = MapMetaData()
        grid.info.width = width
        grid.info.height = height
        grid.info.resolution = resolution
        grid.info.origin.position.x = float(origin[0])
        grid.info.origin.position.y = float(origin[1])
        grid.info.origin.position.z = float(origin[2])
        grid.info.origin.orientation.w = 1.0
        grid.data = data.flatten().tolist()
        return grid

    def _publish_map(self) -> None:
        self._grid_msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._grid_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = StaticMapServer()
        rclpy.spin(node)
    except Exception as exc:
        print(f"static_map_server 异常: {exc}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
