#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import math
import numpy as np
from scipy.spatial import KDTree
from std_msgs.msg import String


class PurePursuitController:
    def __init__(self, lookahead_distance):
        self.lookahead_distance = lookahead_distance
        self.last_closest_idx = 0

    def reset(self):
        self.last_closest_idx = 0

    def calculate_steering_angle(self, vehicle_pose, path_points, visibility_checker=None):
        # Keep progress mostly monotonic. A global nearest-point search can jump
        # to a nearby but wrong branch when the path folds back in a maze.
        search_start = max(0, self.last_closest_idx - 5)
        search_points = path_points[search_start:, :2]
        closest_offset = KDTree(search_points).query(vehicle_pose[:2])[1]
        closest_point_idx = search_start + int(closest_offset)
        self.last_closest_idx = max(self.last_closest_idx, closest_point_idx)

        target_idx = closest_point_idx
        target_visible = True
        visible_idx = None
        traveled = 0.0
        for idx in range(closest_point_idx + 1, len(path_points)):
            traveled += float(np.linalg.norm(path_points[idx] - path_points[idx - 1]))
            candidate = path_points[idx]
            candidate_visible = (
                visibility_checker is None or
                visibility_checker(
                    vehicle_pose[0], vehicle_pose[1], candidate[0], candidate[1]
                )
            )

            if candidate_visible:
                visible_idx = idx
                target_idx = idx

            if traveled >= self.lookahead_distance:
                if candidate_visible:
                    target_idx = idx
                elif visible_idx is not None:
                    target_idx = visible_idx
                else:
                    target_visible = False
                break

        if visible_idx is None and target_idx == closest_point_idx:
            target_visible = (
                visibility_checker is None or
                visibility_checker(
                    vehicle_pose[0], vehicle_pose[1],
                    path_points[target_idx][0], path_points[target_idx][1]
                )
            )

        target_point = path_points[target_idx]
        dx = target_point[0] - vehicle_pose[0]
        dy = target_point[1] - vehicle_pose[1]
        actual_lookahead = math.hypot(dx, dy)

        alpha = math.atan2(dy, dx) - vehicle_pose[2]
        while alpha > math.pi:
            alpha -= 2 * math.pi
        while alpha < -math.pi:
            alpha += 2 * math.pi

        if actual_lookahead < 0.02:
            curvature = 0.0
        else:
            curvature = 2.0 * math.sin(alpha) / actual_lookahead

        return curvature, alpha, actual_lookahead, target_point, self.last_closest_idx, target_idx, target_visible


class PathFollowingNode(Node):
    def __init__(self):
        super().__init__('path_following_node')

        # 前视距离缩小，紧贴路径，避免脱离
        self.pure_pursuit = PurePursuitController(lookahead_distance=0.4)

        self.path_points = None
        self.current_odom = None
        self.latest_map = None
        self.path_received = False
        self.control_mode = "nav"
        self.last_path_progress_idx = 0
        self.last_progress_time = self.get_clock().now()
        self.last_progress_distance = None
        self.last_progress_heading_error = None
        self.recovery_state = None
        self.recovery_end_time = self.get_clock().now()
        self.recovery_angular = 0.0

        # 上一时刻控制量，用于平滑
        self.last_linear_x = 0.0
        self.last_angular_z = 0.0
        self.last_debug_log_time = self.get_clock().now()
        self._last_control_time = None

        # 室内稳定导航参数
        self.max_speed = 0.40         # 直道最高速度 (m/s)
        self.min_speed = 0.08         # 转弯时最低速度 (m/s)
        self.max_angular = 0.60       # 最大角速度 (rad/s)
        self.k_cte = 0.5              # CTE 修正增益 (1/s)，控制横向回正强度
        self.max_curve_for_slowdown = 6.0  # 路径曲率阈值 (1/m)，超过此值速度降至最低
        self.max_linear_acceleration = 0.25   # 线加速度上限 (m/s²)
        self.max_angular_acceleration = 0.80  # 角加速度上限 (rad/s²)
        self.rotate_in_place_angle = 1.2
        self.rotate_in_place_exit = 0.4   # 退出原地转向的阈值，小于进入阈值，防止掉头画弧
        self.in_rotate_in_place = False
        self.rotate_in_place_speed = 0.35
        self.stuck_timeout = 8.0
        self.progress_index_epsilon = 4
        self.progress_distance_epsilon = 0.10
        self.progress_heading_epsilon = 0.20
        self.stuck_linear_velocity_epsilon = 0.02
        self.stuck_angular_velocity_epsilon = 0.10
        self.recovery_reverse_duration = 1.2
        self.recovery_turn_duration = 2.0
        self.recovery_reverse_speed = -0.08
        self.recovery_turn_speed = 0.55
        self.obstacle_threshold = 15
        self.target_line_clearance = 0.12
        self.relaxed_target_line_clearance = 0.07
        self.blocked_crawl_speed = 0.05
        self.blocked_crawl_distance = 0.30

        self.declare_parameter('odom_topic', '/odom')
        odom_topic = self.get_parameter('odom_topic').value
        self.odom_subscriber = self.create_subscription(
            Odometry, odom_topic, self.odometry_callback, 10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_subscriber = self.create_subscription(
            Path, '/path', self.path_callback, 10
        )
        self.map_subscriber = self.create_subscription(
            OccupancyGrid, '/combined_grid', self.map_callback, 10
        )
        self.mode_subscriber = self.create_subscription(
            String, '/control_mode', self.mode_callback, 10
        )

    def mode_callback(self, msg):
        new_mode = msg.data.strip().lower()
        if new_mode not in ("manual", "nav", "pause"):
            self.get_logger().warn(f"Unknown control mode: {msg.data}")
            return

        if new_mode == self.control_mode:
            return

        previous_mode = self.control_mode
        self.control_mode = new_mode

        if new_mode == "manual":
            self.path_points = None
            self.path_received = False
        self.stop_robot()
        self.get_logger().info(
            f"Control mode switched: {previous_mode} -> {self.control_mode}")

    def path_callback(self, msg):
        if self.control_mode == "manual":
            return

        self.path_points_list = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]

        if len(self.path_points_list) < 2:
            self.get_logger().warn(
                f"Received path is too short: {len(self.path_points_list)} point(s). Stop robot."
            )
            self.path_points = None
            self.path_received = False
            self.stop_robot()
            return

        self.path_points = np.array(self.path_points_list)
        assert self.path_points.ndim == 2, "path_points must be a 2D array"

        self.path_points = self.interpolate_path(self.path_points, segment_length=0.08)
        if len(self.path_points) < 2:
            self.get_logger().warn("Interpolated path is too short. Stop robot.")
            self.path_points = None
            self.path_received = False
            self.stop_robot()
            return

        self.path_received = True
        anchor_idx = 0
        if self.current_odom is not None:
            anchor_idx = self.find_nearest_path_index(
                self.path_points,
                self.current_odom.pose.pose.position.x,
                self.current_odom.pose.pose.position.y,
            )
        self.pure_pursuit.reset()
        self.pure_pursuit.last_closest_idx = anchor_idx
        self.last_path_progress_idx = anchor_idx
        self.last_progress_time = self.get_clock().now()
        self.last_progress_distance = None
        self.last_progress_heading_error = None
        self.recovery_state = None
        self.in_rotate_in_place = False
        self.get_logger().info(
            f"Received new path: raw={len(self.path_points_list)}, "
            f"interpolated={len(self.path_points)}, anchor_idx={anchor_idx}"
        )

    def map_callback(self, msg):
        self.latest_map = msg

    def interpolate_path(self, points, segment_length=0.1):
        interpolated_points = [points[0]]
        for i in range(1, len(points)):
            start_point = points[i - 1]
            end_point = points[i]
            distance = np.linalg.norm(end_point - start_point)
            if distance < 1e-6:
                continue

            num_segments = max(int(np.ceil(distance / segment_length)), 1)
            t_values = np.linspace(0, 1, num_segments + 1)[1:]
            interpolated_segment = (
                start_point + (end_point - start_point)[np.newaxis, :] * t_values[:, np.newaxis]
            )
            interpolated_points.extend(interpolated_segment)

        return np.asarray(interpolated_points)

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

    def compute_steering_to_point(self, vehicle_pose, target_point):
        dx = target_point[0] - vehicle_pose[0]
        dy = target_point[1] - vehicle_pose[1]
        target_angle = math.atan2(dy, dx)

        steering_angle = target_angle - vehicle_pose[2]
        while steering_angle > math.pi:
            steering_angle -= 2 * math.pi
        while steering_angle < -math.pi:
            steering_angle += 2 * math.pi

        return steering_angle

    def find_nearest_path_index(self, path_points, world_x, world_y):
        if path_points is None or len(path_points) == 0:
            return 0
        deltas = path_points[:, :2] - np.array([world_x, world_y])
        distances_sq = np.einsum('ij,ij->i', deltas, deltas)
        return int(np.argmin(distances_sq))

    def find_relaxed_crawl_target(self, vehicle_pose, closest_idx):
        if self.path_points is None or len(self.path_points) == 0:
            return None, None

        traveled = 0.0
        best_idx = None
        for idx in range(max(closest_idx, 0) + 1, len(self.path_points)):
            traveled += float(np.linalg.norm(self.path_points[idx] - self.path_points[idx - 1]))
            candidate = self.path_points[idx]
            if self.is_line_free(
                vehicle_pose[0], vehicle_pose[1], candidate[0], candidate[1],
                clearance=self.relaxed_target_line_clearance,
            ):
                best_idx = idx

            if traveled >= self.blocked_crawl_distance:
                break

        if best_idx is None:
            return None, None

        target_point = self.path_points[best_idx]
        steering_angle = self.compute_steering_to_point(vehicle_pose, target_point)
        return target_point, steering_angle
    
    def compute_cte(self, vehicle_pose, path_points, closest_idx):
        if closest_idx >= len(path_points) - 1:
            return 0.0
        p1 = path_points[closest_idx]
        p2 = path_points[closest_idx + 1]
        seg_x = p2[0] - p1[0]
        seg_y = p2[1] - p1[1]
        seg_len = math.hypot(seg_x, seg_y)
        if seg_len < 1e-6:
            return 0.0
        # positive = robot is RIGHT of path direction (standard Stanley convention)
        return -(seg_x * (vehicle_pose[1] - p1[1]) - seg_y * (vehicle_pose[0] - p1[0])) / seg_len

    def stop_robot(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0

        # 清零内部平滑状态，避免残留速度
        self.last_linear_x = 0.0
        self.last_angular_z = 0.0
        self.recovery_state = None

        # 连续发几次零速度，更稳
        for _ in range(5):
            self.cmd_vel_publisher.publish(cmd_vel_msg)

    def odometry_callback(self, msg):
        now = self.get_clock().now()
        control_dt = 0.0
        if self._last_control_time is not None:
            control_dt = max(
                0.0, min(0.1, (now - self._last_control_time).nanoseconds * 1e-9)
            )
        self._last_control_time = now

        if self.control_mode in ("manual", "pause"):
            return

        if not self.path_received or self.path_points is None:
            self.stop_robot()
            return

        self.current_odom = msg
        pose = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self.quaternion_to_yaw(msg.pose.pose.orientation)
        ]

        if self.publish_recovery_command_if_needed():
            return

        (
            curvature,
            alpha,
            actual_lookahead,
            target_point,
            closest_idx,
            target_idx,
            target_visible,
        ) = self.pure_pursuit.calculate_steering_angle(
            pose, self.path_points, self.is_line_free
        )

        distance_to_end = np.linalg.norm(np.array(pose[:2]) - self.path_points[-1])
        odom_linear = float(msg.twist.twist.linear.x)
        odom_angular = float(msg.twist.twist.angular.z)
        cte = self.compute_cte(pose, self.path_points, closest_idx)
        if self.update_progress_watchdog(
            distance_to_end, closest_idx, alpha, odom_linear, odom_angular
        ):
            return

        cmd_vel_msg = Twist()

        # 到终点就停车
        if distance_to_end < 0.2:
            self.path_received = False
            self.path_points = None
            self.stop_robot()
            return
        else:
            # 原地转向滞回逻辑：进入和退出用不同阈值。
            # 进入阈值大(1.2rad~69°)，退出阈值小(0.4rad~23°)，
            # 防止掉头时剩几十度就提前退出原地旋转，画出弧线。
            crawl_target = None
            crawl_steering = 0.0
            if not target_visible:
                crawl_target, crawl_steering = self.find_relaxed_crawl_target(pose, closest_idx)
                if crawl_target is not None:
                    heading_error = abs(crawl_steering)
                else:
                    heading_error = abs(alpha)
            else:
                heading_error = abs(alpha)

            if heading_error >= self.rotate_in_place_angle:
                self.in_rotate_in_place = True
            elif heading_error < self.rotate_in_place_exit:
                self.in_rotate_in_place = False

            if self.in_rotate_in_place:
                steer_ref = crawl_steering if (not target_visible and crawl_target is not None) else alpha
                target_angular = math.copysign(self.rotate_in_place_speed, steer_ref)
                target_speed = 0.0
            elif not target_visible:
                if crawl_target is not None:
                    target_speed = self.blocked_crawl_speed
                    crawl_dx = crawl_target[0] - pose[0]
                    crawl_dy = crawl_target[1] - pose[1]
                    crawl_dist = math.hypot(crawl_dx, crawl_dy)
                    if crawl_dist > 0.02:
                        target_angular = target_speed * 2.0 * math.sin(crawl_steering) / crawl_dist
                    else:
                        target_angular = 0.0
                    target_angular = self.clamp(target_angular, -self.max_angular, self.max_angular)
                else:
                    target_angular = 0.0
                    target_speed = 0.0
            else:
                # 速度基于路径曲率前馈（不用当前角速度指令反馈）
                curve_abs = abs(curvature)
                target_speed = self.max_speed * max(0.0, 1.0 - curve_abs / self.max_curve_for_slowdown)
                target_speed = self.clamp(target_speed, self.min_speed, self.max_speed)
                if distance_to_end < 0.6:
                    target_speed *= 0.5

                # Pure Pursuit 曲率 + Stanley 型 CTE 修正
                alpha_adjusted = alpha + math.atan2(self.k_cte * cte, target_speed + 0.01)
                kappa_corrected = 2.0 * math.sin(alpha_adjusted) / max(actual_lookahead, 0.02)
                target_angular = target_speed * kappa_corrected
                target_angular = self.clamp(target_angular, -self.max_angular, self.max_angular)

        # 速度平滑，避免突然加速/突然猛转
        self.last_linear_x = self.smooth_value(
            target_speed, self.last_linear_x, self.max_linear_acceleration * control_dt
        )
        self.last_angular_z = self.smooth_value(
            target_angular, self.last_angular_z, self.max_angular_acceleration * control_dt
        )

        cmd_vel_msg.linear.x = self.last_linear_x
        cmd_vel_msg.angular.z = self.last_angular_z
        self.cmd_vel_publisher.publish(cmd_vel_msg)

        if (now - self.last_debug_log_time).nanoseconds / 1e9 >= 2.0:
            self.last_debug_log_time = now
            self.get_logger().debug(
                f"tracking path: mode={self.control_mode}, v={self.last_linear_x:.2f}, "
                f"w={self.last_angular_z:.2f}, odom_v={odom_linear:.2f}, "
                f"odom_w={odom_angular:.2f}, dist={distance_to_end:.2f}, "
                f"alpha={alpha:.2f}, curv={curvature:.3f}, cte={cte:.3f}, idx={closest_idx}/{len(self.path_points)}, "
                f"target_idx={target_idx}, visible={target_visible}, "
                f"target=({target_point[0]:.2f}, {target_point[1]:.2f})"
            )

    def is_line_free(self, start_x, start_y, end_x, end_y, clearance=None):
        grid = self.latest_map
        if grid is None or grid.info.resolution <= 0.0 or not grid.data:
            return True

        if clearance is None:
            clearance = self.target_line_clearance

        distance = math.hypot(end_x - start_x, end_y - start_y)
        step = max(float(grid.info.resolution) * 0.5, 0.02)
        steps = max(1, int(math.ceil(distance / step)))

        for i in range(1, steps + 1):
            ratio = i / steps
            x = start_x + (end_x - start_x) * ratio
            y = start_y + (end_y - start_y) * ratio
            if not self.is_world_area_free(x, y, clearance):
                return False
        return True

    def is_world_area_free(self, world_x, world_y, clearance):
        grid = self.latest_map
        resolution = float(grid.info.resolution)
        center = self.world_to_cell(world_x, world_y)
        if center is None:
            return False

        center_x, center_y = center
        radius_cells = max(0, int(math.ceil(clearance / resolution)))
        for cell_y in range(center_y - radius_cells, center_y + radius_cells + 1):
            for cell_x in range(center_x - radius_cells, center_x + radius_cells + 1):
                if math.hypot(cell_x - center_x, cell_y - center_y) * resolution > clearance:
                    continue
                if not self.is_free_cell(cell_x, cell_y):
                    return False
        return True

    def world_to_cell(self, world_x, world_y):
        grid = self.latest_map
        origin_x = float(grid.info.origin.position.x)
        origin_y = float(grid.info.origin.position.y)
        resolution = float(grid.info.resolution)
        cell_x = int((world_x - origin_x) / resolution)
        cell_y = int((world_y - origin_y) / resolution)

        if (
            cell_x < 0 or cell_y < 0 or
            cell_x >= int(grid.info.width) or cell_y >= int(grid.info.height)
        ):
            return None
        return cell_x, cell_y

    def is_free_cell(self, cell_x, cell_y):
        grid = self.latest_map
        width = int(grid.info.width)
        height = int(grid.info.height)
        if cell_x < 0 or cell_y < 0 or cell_x >= width or cell_y >= height:
            return False

        value = int(grid.data[cell_y * width + cell_x])
        return 0 <= value < self.obstacle_threshold

    def update_progress_watchdog(
        self, distance_to_end, closest_idx, steering_angle, odom_linear, odom_angular
    ):
        now = self.get_clock().now()
        heading_error = abs(steering_angle)

        if self.last_progress_distance is None:
            self.last_progress_distance = distance_to_end
            self.last_path_progress_idx = closest_idx
            self.last_progress_heading_error = heading_error
            self.last_progress_time = now
            return False

        index_progress = closest_idx - self.last_path_progress_idx
        distance_progress = self.last_progress_distance - distance_to_end
        heading_progress = 0.0
        if self.last_progress_heading_error is not None:
            heading_progress = self.last_progress_heading_error - heading_error

        active_linear_motion = abs(odom_linear) >= self.stuck_linear_velocity_epsilon
        active_rotation = (
            heading_error >= self.rotate_in_place_angle * 0.7 and
            abs(odom_angular) >= self.stuck_angular_velocity_epsilon
        )
        if (
            index_progress >= self.progress_index_epsilon or
            distance_progress >= self.progress_distance_epsilon or
            heading_progress >= self.progress_heading_epsilon or
            active_linear_motion or
            active_rotation
        ):
            self.last_path_progress_idx = closest_idx
            self.last_progress_distance = distance_to_end
            self.last_progress_heading_error = heading_error
            self.last_progress_time = now
            return False

        elapsed = (now - self.last_progress_time).nanoseconds / 1e9
        if elapsed < self.stuck_timeout:
            return False

        # Re-anchor the controller and run a short recovery maneuver. This helps
        # when the robot is physically wedged while still receiving non-zero cmd_vel.
        last_cmd_linear = self.last_linear_x
        last_cmd_angular = self.last_angular_z
        self.pure_pursuit.last_closest_idx = max(0, closest_idx - 10)
        self.last_path_progress_idx = closest_idx
        self.last_progress_distance = distance_to_end
        self.last_progress_heading_error = heading_error
        self.last_progress_time = now
        self.last_linear_x = 0.0
        self.last_angular_z = 0.0
        self.start_recovery(steering_angle)
        publishers = self.describe_cmd_vel_publishers()
        self.get_logger().warn(
            "Path tracking made no progress for "
            f"{elapsed:.1f}s; start recovery at idx={closest_idx}, "
            f"dist={distance_to_end:.2f}, steer={steering_angle:.2f}, "
            f"last_cmd=({last_cmd_linear:.2f}, {last_cmd_angular:.2f}), "
            f"odom=({odom_linear:.2f}, {odom_angular:.2f}), "
            f"cmd_vel_publishers={publishers}."
        )
        return True

    def describe_cmd_vel_publishers(self):
        try:
            infos = self.get_publishers_info_by_topic('/cmd_vel')
        except Exception as exc:
            return f"unknown: {exc}"

        if not infos:
            return "none"

        names = []
        for info in infos:
            namespace = info.node_namespace.rstrip('/')
            if namespace:
                names.append(f"{namespace}/{info.node_name}")
            else:
                names.append(info.node_name)
        return ",".join(sorted(set(names)))

    def start_recovery(self, steering_angle):
        self.recovery_state = "reverse"
        self.recovery_end_time = self.get_clock().now() + Duration(
            seconds=self.recovery_reverse_duration)
        self.recovery_angular = math.copysign(
            self.recovery_turn_speed,
            steering_angle if abs(steering_angle) > 1e-3 else 1.0
        )
        self.publish_cmd(self.recovery_reverse_speed, 0.0)

    def publish_recovery_command_if_needed(self):
        if self.recovery_state is None:
            return False

        now = self.get_clock().now()
        if now >= self.recovery_end_time:
            if self.recovery_state == "reverse":
                self.recovery_state = "turn"
                self.recovery_end_time = now + Duration(seconds=self.recovery_turn_duration)
            else:
                self.recovery_state = None
                self.last_linear_x = 0.0
                self.last_angular_z = 0.0
                self.get_logger().warn("Recovery finished; resume path tracking.")
                return False

        if self.recovery_state == "reverse":
            self.publish_cmd(self.recovery_reverse_speed, 0.0)
        elif self.recovery_state == "turn":
            self.publish_cmd(0.0, self.recovery_angular)
        return True

    def publish_cmd(self, linear_x, angular_z):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = float(linear_x)
        cmd_vel_msg.angular.z = float(angular_z)
        self.last_linear_x = cmd_vel_msg.linear.x
        self.last_angular_z = cmd_vel_msg.angular.z
        self.cmd_vel_publisher.publish(cmd_vel_msg)


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
