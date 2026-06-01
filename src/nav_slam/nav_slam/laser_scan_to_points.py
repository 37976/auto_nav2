#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
laser_scan_to_points.py — 将 LaserScan 通过 TF 转到 map 帧，发布 PointCloud2 到 /mapokk。
直接为 map_pub 动态障碍物管线提供 map 帧点云，绕过 points_pub_map 的坐标系 bug。
"""

import math

import numpy as np
import rclpy
import rclpy.time
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


class LaserScanToPoints(Node):
    def __init__(self):
        super().__init__("laser_scan_to_points")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("output_topic", "/mapokk")
        self.declare_parameter("min_range", 0.1)
        self.declare_parameter("max_range", 100.0)
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("tf_timeout_sec", 0.5)

        scan_topic = str(self.get_parameter("scan_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._min_range = float(self.get_parameter("min_range").value)
        self._max_range = float(self.get_parameter("max_range").value)
        self._target_frame = str(self.get_parameter("target_frame").value)
        tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self._tf_timeout = rclpy.duration.Duration(seconds=tf_timeout_sec)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._sub = self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        self._pub = self.create_publisher(PointCloud2, output_topic, 10)

        self._tf_ok = False
        self._fail_count = 0

        self.get_logger().info(
            f"监听 {scan_topic} → TF→{self._target_frame} → {output_topic}"
        )

    def _on_scan(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return

        source_frame = msg.header.frame_id
        if not source_frame:
            self.get_logger().warn("LaserScan header.frame_id 为空，跳过")
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                source_frame,
                rclpy.time.Time(),
                self._tf_timeout,
            )
        except TransformException as e:
            self._fail_count += 1
            if self._fail_count % 50 == 1:
                self.get_logger().warn(
                    f"TF 查找失败 ({source_frame}→{self._target_frame}): {e} "
                    f"(已失败 {self._fail_count} 次)"
                )
            return

        if not self._tf_ok:
            self._tf_ok = True
            self.get_logger().info(
                f"TF 链路已打通 ({source_frame}→{self._target_frame})，开始转发点云"
            )

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        rot = _quat_to_rot(qx, qy, qz, qw)

        points = []
        angle = msg.angle_min
        for i in range(n):
            r = msg.ranges[i]
            if not (msg.range_min <= r <= msg.range_max):
                angle += msg.angle_increment
                continue
            if r < self._min_range or r > self._max_range:
                angle += msg.angle_increment
                continue

            px = r * math.cos(angle)
            py = r * math.sin(angle)

            mx = rot[0] * px + rot[1] * py + tx
            my = rot[3] * px + rot[4] * py + ty
            mz = rot[6] * px + rot[7] * py + tz

            points.append([mx, my, mz])
            angle += msg.angle_increment

        if not points:
            return

        cloud = np.array(points, dtype=np.float32)

        header = msg.header
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._target_frame
        cloud_msg = pc2.create_cloud_xyz32(header, cloud)
        self._pub.publish(cloud_msg)


def _quat_to_rot(qx, qy, qz, qw):
    return (
        1 - 2 * (qy * qy + qz * qz),
        2 * (qx * qy - qw * qz),
        2 * (qx * qz + qw * qy),
        2 * (qx * qy + qw * qz),
        1 - 2 * (qx * qx + qz * qz),
        2 * (qy * qz - qw * qx),
        2 * (qx * qz - qw * qy),
        2 * (qy * qz + qw * qx),
        1 - 2 * (qx * qx + qy * qy),
    )


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToPoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
