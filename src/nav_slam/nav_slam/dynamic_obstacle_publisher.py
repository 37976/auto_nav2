#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header


class DynamicObstaclePublisher(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_publisher')

        self.declare_parameter('enabled', True)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('point_spacing', 0.08)
        self.declare_parameter('z_height', 0.2)
        self.declare_parameter('base_xs', [0.0])
        self.declare_parameter('base_ys', [0.0])
        self.declare_parameter('amp_xs', [2.0])
        self.declare_parameter('amp_ys', [0.0])
        self.declare_parameter('periods', [10.0])
        self.declare_parameter('radii', [0.35])
        self.declare_parameter('phase_offsets', [0.0])

        self.enabled = self.get_parameter('enabled').get_parameter_value().bool_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.point_spacing = self.get_parameter('point_spacing').get_parameter_value().double_value
        self.z_height = self.get_parameter('z_height').get_parameter_value().double_value
        self.base_xs = list(self.get_parameter('base_xs').get_parameter_value().double_array_value)
        self.base_ys = list(self.get_parameter('base_ys').get_parameter_value().double_array_value)
        self.amp_xs = list(self.get_parameter('amp_xs').get_parameter_value().double_array_value)
        self.amp_ys = list(self.get_parameter('amp_ys').get_parameter_value().double_array_value)
        self.periods = list(self.get_parameter('periods').get_parameter_value().double_array_value)
        self.radii = list(self.get_parameter('radii').get_parameter_value().double_array_value)
        self.phase_offsets = list(
            self.get_parameter('phase_offsets').get_parameter_value().double_array_value)

        self.obstacle_count = min(
            len(self.base_xs),
            len(self.base_ys),
            len(self.amp_xs),
            len(self.amp_ys),
            len(self.periods),
            len(self.radii),
            len(self.phase_offsets),
        )

        self.publisher = self.create_publisher(PointCloud2, '/dynamic_obstacle_points', 10)
        self.timer = self.create_timer(1.0 / max(self.publish_rate, 1.0), self.timer_callback)
        self.start_time = self.get_clock().now()

        self.get_logger().info(
            f'DynamicObstaclePublisher started, enabled={self.enabled}, count={self.obstacle_count}')

    def build_disk_points(self, center_x, center_y, radius):
        if radius <= 0.0:
            return []

        spacing = max(self.point_spacing, 0.02)
        radius_sq = radius * radius
        points = []

        steps = int(math.ceil(radius / spacing))
        for ix in range(-steps, steps + 1):
            dx = ix * spacing
            for iy in range(-steps, steps + 1):
                dy = iy * spacing
                if dx * dx + dy * dy <= radius_sq:
                    points.append((center_x + dx, center_y + dy, self.z_height))

        return points

    def timer_callback(self):
        if not self.enabled or self.obstacle_count == 0:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        points = []

        for i in range(self.obstacle_count):
            period = max(self.periods[i], 0.1)
            phase = 2.0 * math.pi * elapsed / period + self.phase_offsets[i]
            center_x = self.base_xs[i] + self.amp_xs[i] * math.sin(phase)
            center_y = self.base_ys[i] + self.amp_ys[i] * math.sin(phase)
            points.extend(self.build_disk_points(center_x, center_y, self.radii[i]))

        if not points:
            return

        msg_header = Header()
        msg_header.stamp = self.get_clock().now().to_msg()
        msg_header.frame_id = 'map'
        cloud_msg = pc2.create_cloud_xyz32(msg_header, points)
        self.publisher.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstaclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
