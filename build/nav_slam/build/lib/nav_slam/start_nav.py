#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
import math
import numpy as np
from scipy.spatial import KDTree


class PurePursuitController:
    def __init__(self, lookahead_distance):
        self.lookahead_distance = lookahead_distance

    def calculate_steering_angle(self, vehicle_pose, path_points):
        closest_point_idx = KDTree(path_points[:, :2]).query(vehicle_pose[:2])[1]

        target_point = path_points[-1]
        for i in range(len(path_points) - closest_point_idx):
            lookahead_point_idx = closest_point_idx + i
            target_point = path_points[lookahead_point_idx]
            dx = target_point[0] - vehicle_pose[0]
            dy = target_point[1] - vehicle_pose[1]
            distance_to_target = math.sqrt(dx**2 + dy**2)
            if distance_to_target >= self.lookahead_distance:
                break

        dx = target_point[0] - vehicle_pose[0]
        dy = target_point[1] - vehicle_pose[1]
        target_angle = math.atan2(dy, dx)

        steering_angle = target_angle - vehicle_pose[2]
        while steering_angle > math.pi:
            steering_angle -= 2 * math.pi
        while steering_angle < -math.pi:
            steering_angle += 2 * math.pi

        return steering_angle, target_point


class PathFollowingNode(Node):
    def __init__(self):
        super().__init__('path_following_node')

        # 前视距离调大一点，避免太敏感
        self.pure_pursuit = PurePursuitController(lookahead_distance=0.8)

        self.path_points = None
        self.current_odom = None
        self.path_received = False

        # 上一时刻控制量，用于平滑
        self.last_linear_x = 0.0
        self.last_angular_z = 0.0

        # 控制参数，可继续调
        self.max_speed = 0.5          # 原来太快了，这里先降到 0.5
        self.min_speed = 0.08         # 小转弯时允许慢速爬行
        self.max_angular = 0.8        # 限制最大角速度，避免猛甩
        self.angular_gain = 1.2       # 角速度比例系数，不要直接等于 steering_angle
        self.linear_acc_limit = 0.03  # 每次回调线速度最大变化量
        self.angular_acc_limit = 0.08 # 每次回调角速度最大变化量

        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odometry_callback, 10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_subscriber = self.create_subscription(
            Path, '/path', self.path_callback, 10
        )

    def path_callback(self, msg):
        self.path_points_list = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]

        if len(self.path_points_list) < 2:
            self.get_logger().warn("Received path is too short.")
            return

        self.path_points = np.array(self.path_points_list)
        assert self.path_points.ndim == 2, "path_points must be a 2D array"

        self.path_points = self.interpolate_path(self.path_points, segment_length=0.08)
        self.path_received = True

    def interpolate_path(self, points, segment_length=0.1):
        interpolated_points = []
        for i in range(len(points) - 1):
            start_point = points[i]
            end_point = points[i + 1]
            distance = np.linalg.norm(end_point - start_point)
            num_points = int(distance / segment_length) + 1
            t_values = np.linspace(0, 1, num_points)
            interpolated_segment = (
                start_point + (end_point - start_point)[np.newaxis, :] * t_values[:, np.newaxis]
            )
            interpolated_points.append(interpolated_segment)

        return np.vstack(interpolated_points)

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def smooth_value(self, target, current, step):
        if target > current:
            return min(target, current + step)
        else:
            return max(target, current - step)

    def odometry_callback(self, msg):
        if not self.path_received or self.path_points is None:
            return

        self.current_odom = msg
        pose = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self.quaternion_to_yaw(msg.pose.pose.orientation)
        ]

        steering_angle, target_point = self.pure_pursuit.calculate_steering_angle(
            pose, self.path_points
        )

        distance_to_end = np.linalg.norm(np.array(pose[:2]) - self.path_points[-1])

        cmd_vel_msg = Twist()

        # 到终点就停车
        if distance_to_end < 0.2:
            target_speed = 0.0
            target_angular = 0.0
            self.path_received = False
        else:
            # 角速度：比例控制 + 限幅
            target_angular = self.angular_gain * steering_angle
            target_angular = self.clamp(target_angular, -self.max_angular, self.max_angular)

            # 线速度：转弯越大，速度越小
            angle_abs = abs(target_angular)
            target_speed = self.max_speed * (1.0 - angle_abs / self.max_angular)

            # 给一个最低速度，避免停在那里不动
            target_speed = self.clamp(target_speed, self.min_speed, self.max_speed)

            # 快到终点时进一步减速
            if distance_to_end < 0.6:
                target_speed *= 0.5

        # 速度平滑，避免突然加速/突然猛转
        self.last_linear_x = self.smooth_value(
            target_speed, self.last_linear_x, self.linear_acc_limit
        )
        self.last_angular_z = self.smooth_value(
            target_angular, self.last_angular_z, self.angular_acc_limit
        )

        cmd_vel_msg.linear.x = self.last_linear_x
        cmd_vel_msg.angular.z = self.last_angular_z
        self.cmd_vel_publisher.publish(cmd_vel_msg)

        # self.get_logger().info(
        #     f"v={self.last_linear_x:.2f}, w={self.last_angular_z:.2f}, "
        #     f"dist={distance_to_end:.2f}, steer={steering_angle:.2f}"
        # )


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()