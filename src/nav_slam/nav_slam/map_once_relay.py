#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
map_once_relay.py — 从 /map 接收第一帧地图后，以 transient_local QoS 转发一次到私有话题，
随后取消订阅，避免 AMCL 因 /map 持续发布而反复重建似然场。
不影响 /map 其他订阅者的正常使用。
"""

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class MapOnceRelay(Node):
    def __init__(self):
        super().__init__("map_once_relay")

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(OccupancyGrid, "/map_for_amcl", map_qos)
        self._sub = self.create_subscription(OccupancyGrid, "/map", self._on_map, 10)

        self.get_logger().info("等待 /map 首帧地图，转发一次后即停止监听...")

    def _on_map(self, msg: OccupancyGrid):
        self._pub.publish(msg)
        self.get_logger().info("地图已转发到 /map_for_amcl，取消 /map 订阅")
        self.destroy_subscription(self._sub)


def main(args=None):
    rclpy.init(args=args)
    node = MapOnceRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
