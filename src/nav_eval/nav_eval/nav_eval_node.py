#!/usr/bin/env python3

import csv
import math
import os
import statistics
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from std_msgs.msg import String


@dataclass
class Trial:
    trial_id: int
    goal_x: float
    goal_y: float
    start_wall_time: float
    start_x: Optional[float] = None
    start_y: Optional[float] = None
    end_x: Optional[float] = None
    end_y: Optional[float] = None
    last_x: Optional[float] = None
    last_y: Optional[float] = None
    last_odom_wall_time: Optional[float] = None
    distance_traveled_m: float = 0.0
    min_goal_distance_m: Optional[float] = None
    final_goal_distance_m: Optional[float] = None
    min_clearance_m: Optional[float] = None
    collision: bool = False
    near_collision_count: int = 0
    replan_count: int = 0
    latest_plan_length_m: Optional[float] = None
    cmd_samples: int = 0
    cmd_delta_linear_sum: float = 0.0
    cmd_delta_angular_sum: float = 0.0
    last_cmd_linear: Optional[float] = None
    last_cmd_angular: Optional[float] = None
    max_abs_linear: float = 0.0
    max_abs_angular: float = 0.0
    stuck_time_s: float = 0.0
    timed_out: bool = False
    status: str = "active"
    duration_s: Optional[float] = None
    notes: str = ""


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def path_length(path: Path) -> float:
    total = 0.0
    last = None
    for pose_stamped in path.poses:
        pose = pose_stamped.pose.position
        if last is not None:
            total += math.hypot(pose.x - last[0], pose.y - last[1])
        last = (pose.x, pose.y)
    return total


class NavEvalNode(Node):
    def __init__(self):
        super().__init__("nav_eval")

        self.declare_parameter("output_dir", "~/auto_nav2_eval")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("path_topic", "/path")
        self.declare_parameter("map_topic", "/combined_grid")
        self.declare_parameter("done_topic", "/nav_eval/done")
        self.declare_parameter("goal_tolerance", 0.20)
        self.declare_parameter("timeout_sec", 180.0)
        self.declare_parameter("robot_radius", 0.12)
        self.declare_parameter("obstacle_threshold", 15)
        self.declare_parameter("clearance_search_radius", 2.0)
        self.declare_parameter("collision_clearance", 0.0)
        self.declare_parameter("near_collision_clearance", 0.08)
        self.declare_parameter("stuck_speed_threshold", 0.02)

        self.output_dir = os.path.expanduser(str(self.get_parameter("output_dir").value))
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.robot_radius = float(self.get_parameter("robot_radius").value)
        self.obstacle_threshold = int(self.get_parameter("obstacle_threshold").value)
        self.clearance_search_radius = float(self.get_parameter("clearance_search_radius").value)
        self.collision_clearance = float(self.get_parameter("collision_clearance").value)
        self.near_collision_clearance = float(
            self.get_parameter("near_collision_clearance").value)
        self.stuck_speed_threshold = float(self.get_parameter("stuck_speed_threshold").value)

        self.current_trial: Optional[Trial] = None
        self.completed_trials = []
        self.next_trial_id = 1
        self.latest_odom = None
        self.latest_map: Optional[OccupancyGrid] = None
        self.report_written = False

        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self.odom_callback,
            50,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self.cmd_vel_callback,
            50,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("goal_topic").value),
            self.goal_callback,
            10,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("path_topic").value),
            self.path_callback,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self.map_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("done_topic").value),
            self.done_callback,
            10,
        )
        self.create_timer(0.5, self.timer_callback)

        self.get_logger().info(
            "Nav evaluation started. Send multiple /goal_pose goals; "
            f"reports will be written to {self.output_dir}"
        )

    def done_callback(self, _msg: String):
        self.get_logger().info("收到自动评估完成信号，开始写入评估报告。")
        self.write_report()

    def goal_callback(self, msg: PoseStamped):
        now = time.monotonic()
        if self.current_trial is not None:
            self.finish_current_trial("replaced", "新目标到达，上一目标未完成")

        trial = Trial(
            trial_id=self.next_trial_id,
            goal_x=float(msg.pose.position.x),
            goal_y=float(msg.pose.position.y),
            start_wall_time=now,
        )
        self.next_trial_id += 1

        if self.latest_odom is not None:
            pose = self.latest_odom.pose.pose.position
            trial.start_x = float(pose.x)
            trial.start_y = float(pose.y)
            trial.last_x = trial.start_x
            trial.last_y = trial.start_y
            trial.last_odom_wall_time = now
            trial.min_goal_distance_m = math.hypot(
                trial.start_x - trial.goal_x,
                trial.start_y - trial.goal_y,
            )

        self.current_trial = trial
        self.get_logger().info(
            f"Trial {trial.trial_id} started: goal=({trial.goal_x:.2f}, {trial.goal_y:.2f})"
        )

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg
        trial = self.current_trial
        if trial is None:
            return

        now = time.monotonic()
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        trial.end_x = x
        trial.end_y = y

        if trial.start_x is None:
            trial.start_x = x
            trial.start_y = y
        if trial.last_x is not None and trial.last_y is not None:
            step = math.hypot(x - trial.last_x, y - trial.last_y)
            trial.distance_traveled_m += step
            self.update_stuck_time(trial, step, now)
        trial.last_x = x
        trial.last_y = y
        trial.last_odom_wall_time = now

        goal_distance = math.hypot(x - trial.goal_x, y - trial.goal_y)
        trial.final_goal_distance_m = goal_distance
        if trial.min_goal_distance_m is None:
            trial.min_goal_distance_m = goal_distance
        else:
            trial.min_goal_distance_m = min(trial.min_goal_distance_m, goal_distance)

        clearance = self.estimate_clearance(x, y)
        if clearance is not None:
            if trial.min_clearance_m is None:
                trial.min_clearance_m = clearance
            else:
                trial.min_clearance_m = min(trial.min_clearance_m, clearance)
            if clearance <= self.collision_clearance:
                trial.collision = True
            if clearance <= self.near_collision_clearance:
                trial.near_collision_count += 1

        if goal_distance <= self.goal_tolerance:
            status = "timeout" if trial.timed_out else "success"
            notes = "超时后进入目标容差范围" if trial.timed_out else "进入目标容差范围"
            self.finish_current_trial(status, notes)

    def update_stuck_time(self, trial: Trial, step: float, now: float):
        if trial.last_odom_wall_time is None:
            return
        dt = now - trial.last_odom_wall_time
        if dt <= 0.0 or dt > 2.0:
            return
        moving_slowly = step / dt <= self.stuck_speed_threshold
        far_from_goal = (
            trial.final_goal_distance_m is None or
            trial.final_goal_distance_m > self.goal_tolerance
        )
        if moving_slowly and far_from_goal:
            trial.stuck_time_s += dt

    def cmd_vel_callback(self, msg: Twist):
        trial = self.current_trial
        if trial is None:
            return

        linear = float(msg.linear.x)
        angular = float(msg.angular.z)
        trial.cmd_samples += 1
        trial.max_abs_linear = max(trial.max_abs_linear, abs(linear))
        trial.max_abs_angular = max(trial.max_abs_angular, abs(angular))
        if trial.last_cmd_linear is not None:
            trial.cmd_delta_linear_sum += abs(linear - trial.last_cmd_linear)
            trial.cmd_delta_angular_sum += abs(angular - trial.last_cmd_angular)
        trial.last_cmd_linear = linear
        trial.last_cmd_angular = angular

    def path_callback(self, msg: Path):
        trial = self.current_trial
        if trial is None or len(msg.poses) == 0:
            return
        trial.replan_count += 1
        trial.latest_plan_length_m = path_length(msg)

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    def timer_callback(self):
        trial = self.current_trial
        if trial is None:
            return
        elapsed = time.monotonic() - trial.start_wall_time
        if elapsed >= self.timeout_sec and not trial.timed_out:
            trial.timed_out = True
            trial.notes = f"超过 {self.timeout_sec:.1f} 秒，继续等待目标完成"
            self.get_logger().warn(
                f"Trial {trial.trial_id} timed out at {elapsed:.1f}s; "
                "continuing until the goal is reached."
            )

    def estimate_clearance(self, x: float, y: float) -> Optional[float]:
        grid = self.latest_map
        if grid is None or grid.info.resolution <= 0.0:
            return None

        resolution = float(grid.info.resolution)
        width = int(grid.info.width)
        height = int(grid.info.height)
        origin_x = float(grid.info.origin.position.x)
        origin_y = float(grid.info.origin.position.y)
        if width <= 0 or height <= 0 or not grid.data:
            return None

        center_x = int((x - origin_x) / resolution)
        center_y = int((y - origin_y) / resolution)
        if center_x < 0 or center_y < 0 or center_x >= width or center_y >= height:
            return None

        search_cells = max(1, int(math.ceil(self.clearance_search_radius / resolution)))
        min_distance = None
        y_min = max(0, center_y - search_cells)
        y_max = min(height - 1, center_y + search_cells)
        x_min = max(0, center_x - search_cells)
        x_max = min(width - 1, center_x + search_cells)

        for gy in range(y_min, y_max + 1):
            row_offset = gy * width
            cell_y = origin_y + (gy + 0.5) * resolution
            for gx in range(x_min, x_max + 1):
                value = grid.data[row_offset + gx]
                if value < self.obstacle_threshold:
                    continue
                cell_x = origin_x + (gx + 0.5) * resolution
                distance = math.hypot(cell_x - x, cell_y - y) - self.robot_radius
                if min_distance is None or distance < min_distance:
                    min_distance = distance

        return min_distance

    def finish_current_trial(self, status: str, notes: str):
        trial = self.current_trial
        if trial is None:
            return
        if status == "unfinished" and trial.timed_out:
            status = "timeout"
            notes = f"超过 {self.timeout_sec:.1f} 秒，评估结束时仍未到达"
        trial.status = status
        trial.notes = notes
        trial.duration_s = time.monotonic() - trial.start_wall_time
        self.completed_trials.append(trial)
        self.current_trial = None
        self.get_logger().info(
            f"Trial {trial.trial_id} finished: {status}, "
            f"duration={trial.duration_s:.1f}s, distance={trial.distance_traveled_m:.2f}m"
        )

    def write_report(self):
        if self.report_written:
            return
        self.report_written = True

        if self.current_trial is not None:
            self.finish_current_trial("unfinished", "评估节点停止")

        os.makedirs(self.output_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(self.output_dir, f"nav_eval_{stamp}.csv")
        md_path = os.path.join(self.output_dir, f"nav_eval_{stamp}.md")

        rows = [self.trial_to_row(trial) for trial in self.completed_trials]
        fieldnames = self.csv_fieldnames()

        with open(csv_path, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

        with open(md_path, "w", encoding="utf-8") as md_file:
            md_file.write(self.make_markdown_report(rows, csv_path))

        self.get_logger().info(f"Navigation evaluation CSV written: {csv_path}")
        self.get_logger().info(f"Navigation evaluation summary written: {md_path}")

    def trial_to_row(self, trial: Trial):
        cmd_delta_count = max(trial.cmd_samples - 1, 0)
        avg_cmd_delta_linear = (
            trial.cmd_delta_linear_sum / cmd_delta_count if cmd_delta_count else 0.0
        )
        avg_cmd_delta_angular = (
            trial.cmd_delta_angular_sum / cmd_delta_count if cmd_delta_count else 0.0
        )
        return {
            "序号": trial.trial_id,
            "状态": self.status_text(trial.status),
            "目标X": self.fmt(trial.goal_x),
            "目标Y": self.fmt(trial.goal_y),
            "起点X": self.fmt(trial.start_x),
            "起点Y": self.fmt(trial.start_y),
            "终点X": self.fmt(trial.end_x),
            "终点Y": self.fmt(trial.end_y),
            "耗时_秒": self.fmt(trial.duration_s),
            "实际路程_米": self.fmt(trial.distance_traveled_m),
            "最终目标距离_米": self.fmt(trial.final_goal_distance_m),
            "最小目标距离_米": self.fmt(trial.min_goal_distance_m),
            "最小障碍净距_米": self.fmt(trial.min_clearance_m),
            "是否碰撞": "是" if trial.collision else "否",
            "近碰次数": trial.near_collision_count,
            "重规划次数": trial.replan_count,
            "最新规划路径长度_米": self.fmt(trial.latest_plan_length_m),
            "速度样本数": trial.cmd_samples,
            "平均线速度变化": self.fmt(avg_cmd_delta_linear),
            "平均角速度变化": self.fmt(avg_cmd_delta_angular),
            "最大线速度绝对值": self.fmt(trial.max_abs_linear),
            "最大角速度绝对值": self.fmt(trial.max_abs_angular),
            "停滞时间_秒": self.fmt(trial.stuck_time_s),
            "是否超时": "是" if trial.timed_out or trial.status == "timeout" else "否",
            "备注": trial.notes,
        }

    def make_markdown_report(self, rows, csv_path: str) -> str:
        total = len(rows)
        successes = sum(1 for row in rows if row["状态"] == "成功")
        timeouts = sum(1 for row in rows if row["是否超时"] == "是")
        collisions = sum(1 for row in rows if row["是否碰撞"] == "是")
        success_rate = (successes / total * 100.0) if total else 0.0

        success_durations = [
            float(row["耗时_秒"]) for row in rows
            if row["状态"] == "成功" and row["耗时_秒"] != ""
        ]
        distances = [
            float(row["实际路程_米"]) for row in rows
            if row["实际路程_米"] != ""
        ]
        clearances = [
            float(row["最小障碍净距_米"]) for row in rows
            if row["最小障碍净距_米"] != ""
        ]

        lines = [
            "# 导航评估报告",
            "",
            f"- CSV 文件: `{csv_path}`",
            f"- 目标次数: {total}",
            f"- 成功次数: {successes}",
            f"- 成功率: {success_rate:.1f}%",
            f"- 超时次数: {timeouts}",
            f"- 碰撞次数: {collisions}",
            f"- 成功目标平均耗时: {self.mean_text(success_durations)} 秒",
            f"- 平均实际路程: {self.mean_text(distances)} 米",
            f"- 平均最小障碍净距: {self.mean_text(clearances)} 米",
            "",
            "| 序号 | 状态 | 是否超时 | 目标点 | 耗时(秒) | 实际路程(米) | 最小障碍净距(米) | 重规划次数 | 停滞时间(秒) | 是否碰撞 |",
            "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
        ]
        for row in rows:
            lines.append(
                f"| {row['序号']} | {row['状态']} | {row['是否超时']} | "
                f"({row['目标X']}, {row['目标Y']}) | {row['耗时_秒']} | "
                f"{row['实际路程_米']} | {row['最小障碍净距_米']} | "
                f"{row['重规划次数']} | {row['停滞时间_秒']} | {row['是否碰撞']} |"
            )
        lines.append("")
        return "\n".join(lines)

    @staticmethod
    def csv_fieldnames():
        return [
            "序号",
            "状态",
            "目标X",
            "目标Y",
            "起点X",
            "起点Y",
            "终点X",
            "终点Y",
            "耗时_秒",
            "实际路程_米",
            "最终目标距离_米",
            "最小目标距离_米",
            "最小障碍净距_米",
            "是否碰撞",
            "近碰次数",
            "重规划次数",
            "最新规划路径长度_米",
            "速度样本数",
            "平均线速度变化",
            "平均角速度变化",
            "最大线速度绝对值",
            "最大角速度绝对值",
            "停滞时间_秒",
            "是否超时",
            "备注",
        ]

    @staticmethod
    def status_text(status: str) -> str:
        return {
            "active": "进行中",
            "success": "成功",
            "timeout": "超时",
            "replaced": "被新目标替换",
            "unfinished": "未完成",
        }.get(status, status)

    @staticmethod
    def fmt(value) -> str:
        if value is None:
            return ""
        return f"{float(value):.4f}"

    @staticmethod
    def mean_text(values) -> str:
        if not values:
            return "n/a"
        return f"{statistics.mean(values):.3f}"

    def destroy_node(self):
        self.write_report()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavEvalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
