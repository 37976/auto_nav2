#!/usr/bin/env python3

import math
import random
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from std_msgs.msg import String


@dataclass
class Goal:
    x: float
    y: float
    yaw: float = 0.0


def parse_goals(value: str) -> List[Goal]:
    goals = []
    for item in value.split(";"):
        item = item.strip()
        if not item:
            continue
        parts = [part.strip() for part in item.split(",")]
        if len(parts) not in (2, 3):
            raise ValueError(f"目标点格式错误: {item}")
        x = float(parts[0])
        y = float(parts[1])
        yaw = float(parts[2]) if len(parts) == 3 else 0.0
        goals.append(Goal(x=x, y=y, yaw=yaw))
    if not goals:
        raise ValueError("目标点列表为空")
    return goals


class AutoGoalRunner(Node):
    def __init__(self):
        super().__init__("auto_goal_runner")

        self.declare_parameter(
            "goals",
            "5.0,0.0,0.0; 5.0,5.0,0.0; -5.0,5.0,0.0; "
            "-10.0,-5.0,0.0; 10.0,-10.0,0.0; 15.0,10.0,0.0",
        )
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("mode_topic", "/control_mode")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("map_topic", "/combined_grid")
        self.declare_parameter("done_topic", "/nav_eval/done")
        self.declare_parameter("random_goals", True)
        self.declare_parameter("random_goal_count", 6)
        self.declare_parameter("random_seed", -1)
        self.declare_parameter("random_goal_min_clearance", 0.35)
        self.declare_parameter("random_goal_min_start_distance", 1.0)
        self.declare_parameter("random_goal_min_between_distance", 1.0)
        self.declare_parameter("random_goal_max_attempts", 20000)
        self.declare_parameter("obstacle_threshold", 15)
        self.declare_parameter("goal_tolerance", 0.20)
        self.declare_parameter("goal_timeout_sec", 180.0)
        self.declare_parameter("start_delay_sec", 8.0)
        self.declare_parameter("between_goal_delay_sec", 2.0)
        self.declare_parameter("wait_for_map", True)
        self.declare_parameter("wait_for_odom", True)

        self.random_goals = bool(self.get_parameter("random_goals").value)
        self.random_goal_count = int(self.get_parameter("random_goal_count").value)
        self.random_seed = int(self.get_parameter("random_seed").value)
        self.random_goal_min_clearance = float(
            self.get_parameter("random_goal_min_clearance").value)
        self.random_goal_min_start_distance = float(
            self.get_parameter("random_goal_min_start_distance").value)
        self.random_goal_min_between_distance = float(
            self.get_parameter("random_goal_min_between_distance").value)
        self.random_goal_max_attempts = int(
            self.get_parameter("random_goal_max_attempts").value)
        self.obstacle_threshold = int(self.get_parameter("obstacle_threshold").value)
        self.goals = [] if self.random_goals else parse_goals(
            str(self.get_parameter("goals").value)
        )
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.goal_timeout_sec = float(self.get_parameter("goal_timeout_sec").value)
        self.start_delay_sec = float(self.get_parameter("start_delay_sec").value)
        self.between_goal_delay_sec = float(self.get_parameter("between_goal_delay_sec").value)
        self.wait_for_map = bool(self.get_parameter("wait_for_map").value)
        self.wait_for_odom = bool(self.get_parameter("wait_for_odom").value)

        self.goal_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("goal_topic").value), 10)
        self.mode_pub = self.create_publisher(
            String, str(self.get_parameter("mode_topic").value), 10)
        self.done_pub = self.create_publisher(
            String, str(self.get_parameter("done_topic").value), 10)
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self.odom_callback, 20)
        self.create_subscription(
            OccupancyGrid, str(self.get_parameter("map_topic").value), self.map_callback, 5)

        self.latest_odom: Optional[Odometry] = None
        self.latest_map: Optional[OccupancyGrid] = None
        self.has_map = False
        self.goal_index = -1
        self.active_goal: Optional[Goal] = None
        self.active_goal_start_time: Optional[float] = None
        self.active_goal_timed_out = False
        self.next_goal_time = time.monotonic() + self.start_delay_sec
        self.done = False
        self.last_wait_log_time = 0.0
        self.timer = self.create_timer(0.2, self.timer_callback)

        if self.random_goals:
            self.get_logger().info(
                f"自动目标点评估已启动，将从地图自由区域随机生成 "
                f"{self.random_goal_count} 个目标点。"
            )
        else:
            self.get_logger().info(
                f"自动目标点评估已启动，共 {len(self.goals)} 个目标点。"
            )

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg
        self.has_map = True

    def timer_callback(self):
        if self.done:
            return

        if self.wait_for_map and not self.has_map:
            self.log_waiting("等待 /combined_grid 地图...")
            return
        if self.wait_for_odom and self.latest_odom is None:
            self.log_waiting("等待 /odom 里程计...")
            return
        if self.random_goals and not self.goals:
            if not self.generate_random_goals():
                self.done = True
                self.publish_done()
            return

        now = time.monotonic()
        if self.active_goal is None:
            if now < self.next_goal_time:
                return
            self.publish_next_goal()
            return

        if self.latest_odom is None:
            return

        pose = self.latest_odom.pose.pose.position
        distance = math.hypot(pose.x - self.active_goal.x, pose.y - self.active_goal.y)
        if distance <= self.goal_tolerance:
            self.get_logger().info(
                f"目标 {self.goal_index + 1}/{len(self.goals)} 已到达，"
                f"距离 {distance:.2f} m。"
            )
            self.active_goal = None
            self.active_goal_start_time = None
            self.active_goal_timed_out = False
            self.next_goal_time = now + self.between_goal_delay_sec
            return

        elapsed = now - self.active_goal_start_time
        if elapsed >= self.goal_timeout_sec and not self.active_goal_timed_out:
            self.active_goal_timed_out = True
            self.get_logger().warn(
                f"目标 {self.goal_index + 1}/{len(self.goals)} 超时，"
                f"当前距离 {distance:.2f} m；继续等待该目标完成。"
            )

    def publish_next_goal(self):
        next_index = self.goal_index + 1
        if next_index >= len(self.goals):
            self.done = True
            self.publish_done()
            self.get_logger().info("全部自动目标点已完成，已通知评估节点写报告。")
            return

        self.goal_index = next_index
        goal = self.goals[self.goal_index]
        self.active_goal = goal
        self.active_goal_start_time = time.monotonic()
        self.active_goal_timed_out = False

        mode_msg = String()
        mode_msg.data = "nav"
        self.mode_pub.publish(mode_msg)

        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal.x
        goal_msg.pose.position.y = goal.y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.z = math.sin(goal.yaw / 2.0)
        goal_msg.pose.orientation.w = math.cos(goal.yaw / 2.0)
        self.goal_pub.publish(goal_msg)

        self.get_logger().info(
            f"发送目标 {self.goal_index + 1}/{len(self.goals)}: "
            f"x={goal.x:.2f}, y={goal.y:.2f}, yaw={goal.yaw:.2f}"
        )

    def generate_random_goals(self) -> bool:
        grid = self.latest_map
        if grid is None:
            return False

        width = int(grid.info.width)
        height = int(grid.info.height)
        resolution = float(grid.info.resolution)
        if width <= 0 or height <= 0 or resolution <= 0.0 or not grid.data:
            self.get_logger().error("地图无效，无法随机生成目标点。")
            return False

        rng = random.Random(None if self.random_seed < 0 else self.random_seed)
        origin_x = float(grid.info.origin.position.x)
        origin_y = float(grid.info.origin.position.y)
        goals = []

        start_x = None
        start_y = None
        if self.latest_odom is not None:
            pose = self.latest_odom.pose.pose.position
            start_x = float(pose.x)
            start_y = float(pose.y)

        for _ in range(max(1, self.random_goal_max_attempts)):
            cell_x = rng.randrange(width)
            cell_y = rng.randrange(height)
            if not self.is_safe_goal_cell(grid, cell_x, cell_y):
                continue

            x = origin_x + (cell_x + 0.5) * resolution
            y = origin_y + (cell_y + 0.5) * resolution
            if start_x is not None:
                if math.hypot(x - start_x, y - start_y) < self.random_goal_min_start_distance:
                    continue
            if any(
                math.hypot(x - goal.x, y - goal.y) < self.random_goal_min_between_distance
                for goal in goals
            ):
                continue

            goals.append(Goal(x=x, y=y, yaw=rng.uniform(-math.pi, math.pi)))
            if len(goals) >= self.random_goal_count:
                break

        if not goals:
            self.get_logger().error(
                "没有找到满足安全距离的随机目标点，请减小 "
                "random_goal_min_clearance 或检查 /combined_grid。"
            )
            return False

        if len(goals) < self.random_goal_count:
            self.get_logger().warn(
                f"只生成了 {len(goals)}/{self.random_goal_count} 个安全随机目标点。"
            )

        self.goals = goals
        for index, goal in enumerate(self.goals, start=1):
            self.get_logger().info(
                f"随机目标 {index}/{len(self.goals)}: "
                f"x={goal.x:.2f}, y={goal.y:.2f}, yaw={goal.yaw:.2f}"
            )
        return True

    def is_safe_goal_cell(self, grid: OccupancyGrid, cell_x: int, cell_y: int) -> bool:
        width = int(grid.info.width)
        height = int(grid.info.height)
        resolution = float(grid.info.resolution)
        if not self.is_free_cell(grid, cell_x, cell_y):
            return False

        clearance_cells = max(0, int(math.ceil(self.random_goal_min_clearance / resolution)))
        for y in range(cell_y - clearance_cells, cell_y + clearance_cells + 1):
            for x in range(cell_x - clearance_cells, cell_x + clearance_cells + 1):
                if x < 0 or y < 0 or x >= width or y >= height:
                    return False
                if math.hypot(x - cell_x, y - cell_y) * resolution > self.random_goal_min_clearance:
                    continue
                if not self.is_free_cell(grid, x, y):
                    return False
        return True

    def is_free_cell(self, grid: OccupancyGrid, cell_x: int, cell_y: int) -> bool:
        width = int(grid.info.width)
        height = int(grid.info.height)
        if cell_x < 0 or cell_y < 0 or cell_x >= width or cell_y >= height:
            return False

        value = int(grid.data[cell_y * width + cell_x])
        return 0 <= value < self.obstacle_threshold

    def publish_done(self):
        msg = String()
        msg.data = "done"
        self.done_pub.publish(msg)

    def log_waiting(self, text: str):
        now = time.monotonic()
        if now - self.last_wait_log_time >= 3.0:
            self.last_wait_log_time = now
            self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = AutoGoalRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
