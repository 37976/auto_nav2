#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2


class ObstacleGridNode(Node):
    def __init__(self):
        super().__init__('obstacle_grid_node')

        # 声明并获取参数
        self.declare_parameter('grid_width', 60.0)
        self.declare_parameter('grid_height', 60.0)
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('min_height', 0.1)
        self.declare_parameter('max_height', 1.0)
        self.declare_parameter('obstacle_radius', 0.2)
        self.declare_parameter('projection_gap_fill_cells', 2)
        self.declare_parameter('dynamic_obstacle_timeout', 0.8)

        # 新增：静态地图参数
        self.declare_parameter('use_static_map', True)
        self.declare_parameter('static_map_yaml', '/home/xu/automatic-navigation/src/nav_slam/map/gpt.yaml')

        self.grid_width = self.get_parameter('grid_width').get_parameter_value().double_value
        self.grid_height = self.get_parameter('grid_height').get_parameter_value().double_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.min_height = self.get_parameter('min_height').get_parameter_value().double_value
        self.max_height = self.get_parameter('max_height').get_parameter_value().double_value
        self.obstacle_radius = self.get_parameter('obstacle_radius').get_parameter_value().double_value
        self.projection_gap_fill_cells = self.get_parameter(
            'projection_gap_fill_cells').get_parameter_value().integer_value
        self.dynamic_obstacle_timeout = self.get_parameter(
            'dynamic_obstacle_timeout').get_parameter_value().double_value

        self.use_static_map = self.get_parameter('use_static_map').get_parameter_value().bool_value
        self.static_map_yaml = self.get_parameter('static_map_yaml').get_parameter_value().string_value

        # 初始化障碍物和膨胀层集合
        self.obstacles = set()
        self.dilated_obstacles_layer1 = set()
        self.dilated_obstacles_layer2 = set()
        self.dilated_obstacles_layer3 = set()

        # 静态底图缓存
        self.static_grid_data = None

        # 初始化OccupancyGrid消息
        self.grid_combined = OccupancyGrid()
        self.grid_combined.header.frame_id = 'map'
        self.grid_combined.info.width = int(self.grid_width / self.resolution)
        self.grid_combined.info.height = int(self.grid_height / self.resolution)
        self.grid_combined.info.resolution = self.resolution
        self.grid_combined.info.origin.position.x = -self.grid_width / 2
        self.grid_combined.info.origin.position.y = -self.grid_height / 2
        self.grid_combined.info.origin.position.z = 0.0
        self.grid_combined.data = [-1] * (self.grid_combined.info.width * self.grid_combined.info.height)

        # 如果启用静态地图，启动时先加载一次
        if self.use_static_map:
            self.load_static_map(self.static_map_yaml)

        # 创建订阅者和发布者
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/mapokk', self.pointcloud_callback, 10
        )
        self.dynamic_obstacle_sub = self.create_subscription(
            PointCloud2, '/dynamic_obstacle_points', self.dynamic_obstacle_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.grid_combined_pub = self.create_publisher(
            OccupancyGrid, '/combined_grid', 10
        )

        self.odom_data = None
        self.last_pointcloud_time = None
        self.last_dynamic_obstacle_time = None
        self.pointcloud_obstacles = set()
        self.pointcloud_dilated_obstacles_layer1 = set()
        self.pointcloud_dilated_obstacles_layer2 = set()
        self.pointcloud_dilated_obstacles_layer3 = set()
        self.dynamic_obstacle_points = set()
        self.dynamic_dilated_obstacles_layer1 = set()
        self.dynamic_dilated_obstacles_layer2 = set()
        self.dynamic_dilated_obstacles_layer3 = set()

        # 定时发布，保证即使没有点云也能看到静态地图
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info('ObstacleGridNode started.')
        if self.use_static_map:
            self.get_logger().info(f'Use static map: {self.static_map_yaml}')

    def timer_callback(self):
        changed = False
        if self.last_pointcloud_time is not None:
            elapsed = (
                self.get_clock().now() - self.last_pointcloud_time).nanoseconds / 1e9
            if elapsed > self.dynamic_obstacle_timeout and (
                self.pointcloud_obstacles or self.pointcloud_dilated_obstacles_layer1 or
                self.pointcloud_dilated_obstacles_layer2 or self.pointcloud_dilated_obstacles_layer3
            ):
                self.pointcloud_obstacles = set()
                self.pointcloud_dilated_obstacles_layer1 = set()
                self.pointcloud_dilated_obstacles_layer2 = set()
                self.pointcloud_dilated_obstacles_layer3 = set()
                changed = True

        if self.last_dynamic_obstacle_time is not None:
            elapsed = (
                self.get_clock().now() - self.last_dynamic_obstacle_time).nanoseconds / 1e9
            if elapsed > self.dynamic_obstacle_timeout and (
                self.dynamic_obstacle_points or self.dynamic_dilated_obstacles_layer1 or
                self.dynamic_dilated_obstacles_layer2 or self.dynamic_dilated_obstacles_layer3
            ):
                self.dynamic_obstacle_points = set()
                self.dynamic_dilated_obstacles_layer1 = set()
                self.dynamic_dilated_obstacles_layer2 = set()
                self.dynamic_dilated_obstacles_layer3 = set()
                changed = True

        if changed:
            self.refresh_dynamic_layers()
            self.update_combined_grid()
            return

        self.grid_combined.header.stamp = self.get_clock().now().to_msg()
        self.grid_combined.header.frame_id = 'map'
        self.grid_combined_pub.publish(self.grid_combined)

    def odom_callback(self, msg):
        self.odom_data = msg

    def build_obstacle_layers(self, points):
        map_origin_x = self.grid_combined.info.origin.position.x
        map_origin_y = self.grid_combined.info.origin.position.y
        radius_cells = max(1, int(self.obstacle_radius / self.resolution))

        new_obstacles = set()
        new_dilated_obstacles_layer1 = set()
        new_dilated_obstacles_layer2 = set()
        new_dilated_obstacles_layer3 = set()
        previous_cell = None

        for x, y, z in points:
            if not (self.min_height <= z <= self.max_height):
                previous_cell = None
                continue

            center_x = int((x - map_origin_x) / self.resolution)
            center_y = int((y - map_origin_y) / self.resolution)

            if not (
                0 <= center_x < self.grid_combined.info.width and
                0 <= center_y < self.grid_combined.info.height
            ):
                previous_cell = None
                continue

            index = center_y * self.grid_combined.info.width + center_x
            new_obstacles.add(index)

            if previous_cell is not None and self.projection_gap_fill_cells > 0:
                prev_x, prev_y = previous_cell
                gap_cells = max(abs(center_x - prev_x), abs(center_y - prev_y)) - 1
                if 0 < gap_cells <= self.projection_gap_fill_cells:
                    for fill_x, fill_y in self.iter_grid_line(prev_x, prev_y, center_x, center_y):
                        fill_index = fill_y * self.grid_combined.info.width + fill_x
                        new_obstacles.add(fill_index)

            previous_cell = (center_x, center_y)

            for layer, dilated_set in enumerate([
                new_dilated_obstacles_layer1,
                new_dilated_obstacles_layer2,
                new_dilated_obstacles_layer3
            ]):
                rr = (layer + 1) * radius_cells
                for dx in range(-rr, rr + 1):
                    for dy in range(-rr, rr + 1):
                        if dx ** 2 + dy ** 2 <= rr ** 2:
                            grid_x = center_x + dx
                            grid_y = center_y + dy
                            if 0 <= grid_x < self.grid_combined.info.width and 0 <= grid_y < self.grid_combined.info.height:
                                index = grid_y * self.grid_combined.info.width + grid_x
                                dilated_set.add(index)

        return (
            new_obstacles,
            new_dilated_obstacles_layer1,
            new_dilated_obstacles_layer2,
            new_dilated_obstacles_layer3,
        )

    def refresh_dynamic_layers(self):
        self.obstacles = self.pointcloud_obstacles | self.dynamic_obstacle_points
        self.dilated_obstacles_layer1 = (
            self.pointcloud_dilated_obstacles_layer1 | self.dynamic_dilated_obstacles_layer1
        )
        self.dilated_obstacles_layer2 = (
            self.pointcloud_dilated_obstacles_layer2 | self.dynamic_dilated_obstacles_layer2
        )
        self.dilated_obstacles_layer3 = (
            self.pointcloud_dilated_obstacles_layer3 | self.dynamic_dilated_obstacles_layer3
        )

    def iter_grid_line(self, x0, y0, x1, y1):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0
        while True:
            yield x, y
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def pointcloud_callback(self, msg):
        if self.odom_data is None:
            return

        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        (
            self.pointcloud_obstacles,
            self.pointcloud_dilated_obstacles_layer1,
            self.pointcloud_dilated_obstacles_layer2,
            self.pointcloud_dilated_obstacles_layer3,
        ) = self.build_obstacle_layers(points)
        self.last_pointcloud_time = self.get_clock().now()
        self.refresh_dynamic_layers()
        self.update_combined_grid()

    def dynamic_obstacle_callback(self, msg):
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        (
            self.dynamic_obstacle_points,
            self.dynamic_dilated_obstacles_layer1,
            self.dynamic_dilated_obstacles_layer2,
            self.dynamic_dilated_obstacles_layer3,
        ) = self.build_obstacle_layers(points)
        self.last_dynamic_obstacle_time = self.get_clock().now()
        self.refresh_dynamic_layers()
        self.update_combined_grid()

    def update_combined_grid(self):
        # 关键改动：先恢复静态底图，而不是每次整图刷成 [1]
        if self.use_static_map and self.static_grid_data is not None:
            self.grid_combined.data = list(self.static_grid_data)
        else:
            self.grid_combined.data = [0] * (self.grid_combined.info.width * self.grid_combined.info.height)

        # 标记障碍物
        for index in self.obstacles:
            if 0 <= index < len(self.grid_combined.data):
                self.grid_combined.data[index] = 100

        # 三层膨胀：都用 0~100 范围，别再用负数
        for index in self.dilated_obstacles_layer1 - self.obstacles:
            if 0 <= index < len(self.grid_combined.data) and self.grid_combined.data[index] == 0:
                self.grid_combined.data[index] = 30

        for index in self.dilated_obstacles_layer2 - self.dilated_obstacles_layer1:
            if 0 <= index < len(self.grid_combined.data) and self.grid_combined.data[index] == 0:
                self.grid_combined.data[index] = 15

        for index in self.dilated_obstacles_layer3 - self.dilated_obstacles_layer2:
            if 0 <= index < len(self.grid_combined.data) and self.grid_combined.data[index] == 0:
                self.grid_combined.data[index] = 5

        self.grid_combined.header.stamp = self.get_clock().now().to_msg()
        self.grid_combined.header.frame_id = 'map'
        self.grid_combined_pub.publish(self.grid_combined)

    def load_static_map(self, yaml_path):
        if not os.path.exists(yaml_path):
            self.get_logger().error(f'静态地图 yaml 不存在: {yaml_path}')
            return

        try:
            with open(yaml_path, 'r', encoding='utf-8') as f:
                map_yaml = yaml.safe_load(f)

            image_path = map_yaml['image']
            if not os.path.isabs(image_path):
                image_path = os.path.join(os.path.dirname(yaml_path), image_path)

            resolution = float(map_yaml['resolution'])
            origin = map_yaml['origin']
            occupied_thresh = float(map_yaml.get('occupied_thresh', 0.65))
            free_thresh = float(map_yaml.get('free_thresh', 0.196))
            negate = int(map_yaml.get('negate', 0))

            img = self.read_pgm(image_path)
            if img is None:
                self.get_logger().error(f'静态地图 pgm 读取失败: {image_path}')
                return

            img = np.flipud(img)
            h, w = img.shape

            self.grid_combined.info.width = w
            self.grid_combined.info.height = h
            self.grid_combined.info.resolution = resolution
            self.grid_combined.info.origin.position.x = float(origin[0])
            self.grid_combined.info.origin.position.y = float(origin[1])
            self.grid_combined.info.origin.position.z = 0.0

            data = []
            for y in range(h):
                for x in range(w):
                    pixel = img[y, x]

                    if negate == 0:
                        occ = (255.0 - pixel) / 255.0
                    else:
                        occ = pixel / 255.0

                    if occ >= occupied_thresh:
                        data.append(100)
                    elif occ <= free_thresh:
                        data.append(0)
                    else:
                        data.append(-1)

            self.grid_combined.data = list(data)
            self.static_grid_data = list(data)

            # 同步尺寸参数，后面点云叠加也用这份图
            self.grid_width = self.grid_combined.info.width * self.grid_combined.info.resolution
            self.grid_height = self.grid_combined.info.height * self.grid_combined.info.resolution
            self.resolution = self.grid_combined.info.resolution

            self.get_logger().info(
                f'静态地图加载成功: {image_path}, size={w}x{h}, resolution={resolution}'
            )

        except Exception as e:
            self.get_logger().error(f'加载静态地图失败: {e}')

    def read_pgm(self, filename):
        try:
            with open(filename, 'rb') as f:
                magic = f.readline().strip()
                if magic != b'P5':
                    self.get_logger().error('目前只支持 P5 格式 pgm')
                    return None

                line = f.readline()
                while line.startswith(b'#'):
                    line = f.readline()

                width, height = map(int, line.split())
                maxval = int(f.readline().strip())

                if maxval > 255:
                    self.get_logger().error('暂不支持 16bit pgm')
                    return None

                data = np.frombuffer(f.read(width * height), dtype=np.uint8)
                img = data.reshape((height, width))
                return img

        except Exception as e:
            self.get_logger().error(f'读取 pgm 失败: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleGridNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
