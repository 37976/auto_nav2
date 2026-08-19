#!/usr/bin/env python3
"""Publish one reproducible goal sequence after initial global localization."""

import json
import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Empty, String


@dataclass(frozen=True)
class Goal:
    """One map-frame navigation goal."""

    x: float
    y: float
    yaw_rad: float


def parse_goals(value: str) -> list[Goal]:
    """Parse ``x,y,yaw_deg;...`` into map-frame goals."""
    goals = []
    for item in value.split(";"):
        item = item.strip()
        if not item:
            continue
        parts = [part.strip() for part in item.split(",")]
        if len(parts) != 3:
            raise ValueError(
                f"Invalid goal {item!r}; expected x,y,yaw_deg"
            )
        goals.append(Goal(
            x=float(parts[0]),
            y=float(parts[1]),
            yaw_rad=math.radians(float(parts[2])),
        ))
    if not goals:
        raise ValueError("The goals parameter must contain at least one goal")
    return goals


class NavExperimentGoalRunner(Node):
    """Send a fixed goal sequence and report completion or timeout."""

    def __init__(self) -> None:
        super().__init__("nav_experiment_goal_runner")
        self.declare_parameter("goals", "")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("goal_reached_topic", "/goal_reached")
        self.declare_parameter("experiment_done_topic", "/nav_experiment/done")
        self.declare_parameter(
            "global_correction_event_topic",
            "/localization/map_odom_correction_event",
        )
        self.declare_parameter("start_delay_sec", 2.0)
        self.declare_parameter("between_goal_delay_sec", 1.0)
        self.declare_parameter("goal_timeout_sec", 180.0)

        self._goals = parse_goals(str(self.get_parameter("goals").value))
        self._start_delay_sec = max(
            0.0, float(self.get_parameter("start_delay_sec").value)
        )
        self._between_goal_delay_sec = max(
            0.0, float(self.get_parameter("between_goal_delay_sec").value)
        )
        self._goal_timeout_sec = max(
            1.0, float(self.get_parameter("goal_timeout_sec").value)
        )

        self._goal_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter("goal_topic").value), 10
        )
        self._done_pub = self.create_publisher(
            String, str(self.get_parameter("experiment_done_topic").value), 10
        )
        self.create_subscription(
            Empty,
            str(self.get_parameter("goal_reached_topic").value),
            self._goal_reached_callback,
            10,
        )
        event_qos = QoSProfile(
            depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(
            String,
            str(self.get_parameter("global_correction_event_topic").value),
            self._correction_event_callback,
            event_qos,
        )

        self._initial_global_applied = False
        self._goal_index = -1
        self._goals_reached = 0
        self._goal_active = False
        self._finished = False
        self._next_goal_at: Optional[float] = None
        self._active_goal_started_at: Optional[float] = None
        self.create_timer(0.1, self._timer_callback)
        self.get_logger().info(
            f"固定目标序列节点已启动，共 {len(self._goals)} 个目标；"
            "等待首次全局 ORB 定位"
        )

    def _correction_event_callback(self, msg: String) -> None:
        if self._initial_global_applied or self._finished:
            return
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if event.get("status") != "global_applied":
            return
        self._initial_global_applied = True
        self._next_goal_at = self._now_sec() + self._start_delay_sec
        self.get_logger().info(
            f"首次全局定位已应用，{self._start_delay_sec:.1f}s 后发送第一个目标"
        )

    def _goal_reached_callback(self, _msg: Empty) -> None:
        if not self._goal_active or self._finished:
            return
        self._goal_active = False
        self._active_goal_started_at = None
        self._goals_reached += 1
        reached = self._goals_reached
        self.get_logger().info(f"目标 {reached}/{len(self._goals)} 已到达")
        if reached >= len(self._goals):
            self._finish("completed", "all_goals_reached")
            return
        self._next_goal_at = self._now_sec() + self._between_goal_delay_sec

    def _timer_callback(self) -> None:
        if self._finished or not self._initial_global_applied:
            return
        now_sec = self._now_sec()
        if self._goal_active:
            assert self._active_goal_started_at is not None
            elapsed = now_sec - self._active_goal_started_at
            if elapsed >= self._goal_timeout_sec:
                self._finish("aborted", "goal_timeout")
            return
        if self._next_goal_at is None or now_sec < self._next_goal_at:
            return
        self._publish_next_goal(now_sec)

    def _publish_next_goal(self, now_sec: float) -> None:
        self._goal_index += 1
        goal = self._goals[self._goal_index]
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x = goal.x
        message.pose.position.y = goal.y
        message.pose.orientation.z = math.sin(goal.yaw_rad * 0.5)
        message.pose.orientation.w = math.cos(goal.yaw_rad * 0.5)
        self._goal_pub.publish(message)
        self._goal_active = True
        self._active_goal_started_at = now_sec
        self._next_goal_at = None
        self.get_logger().info(
            f"发送目标 {self._goal_index + 1}/{len(self._goals)}："
            f"x={goal.x:.2f}, y={goal.y:.2f}, "
            f"yaw={math.degrees(goal.yaw_rad):.1f}deg"
        )

    def _finish(self, status: str, reason: str) -> None:
        self._finished = True
        event = String()
        event.data = json.dumps({
            "status": status,
            "reason": reason,
            "goals_total": len(self._goals),
            "goals_reached": self._goals_reached,
        })
        self._done_pub.publish(event)
        self.get_logger().info(
            f"目标序列结束：status={status}, reason={reason}"
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavExperimentGoalRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
