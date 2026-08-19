#!/usr/bin/env python3
"""Record one manual navigation experiment and its paper-ready trajectory metrics."""

import csv
import json
import math
import os
import statistics
import time
from collections import deque
from datetime import datetime
from typing import Any, Deque, Optional

import rclpy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Empty, String


def _yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _percent(numerator: int, denominator: int) -> Optional[float]:
    return None if denominator == 0 else 100.0 * numerator / denominator


def _rms(values: list[float]) -> Optional[float]:
    return None if not values else math.sqrt(sum(value * value for value in values) / len(values))


def _percentile(values: list[float], percentile: float) -> Optional[float]:
    if not values:
        return None
    values = sorted(values)
    index = (len(values) - 1) * percentile / 100.0
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    return values[lower] + (values[upper] - values[lower]) * (index - lower)


def _composite_pose_error(
    position_error_m: float,
    yaw_error_deg: float,
    position_tolerance_m: float,
    yaw_tolerance_deg: float,
) -> float:
    """Return the equally weighted, dimensionless combined pose error."""
    return math.sqrt(0.5 * (
        (position_error_m / position_tolerance_m) ** 2
        + (yaw_error_deg / yaw_tolerance_deg) ** 2
    ))


def _sample_standard_deviation(values: list[float]) -> Optional[float]:
    """Return sample standard deviation, or no value for fewer than two runs."""
    return statistics.stdev(values) if len(values) >= 2 else None


def _joint_accuracy_satisfied(
    position_error_m: float,
    yaw_error_deg: float,
    position_tolerance_m: float,
    yaw_tolerance_deg: float,
) -> bool:
    """Return whether both absolute pose-error requirements are satisfied."""
    return (
        position_error_m <= position_tolerance_m
        and abs(yaw_error_deg) <= yaw_tolerance_deg
    )


def _pose_difference(
    first: tuple[float, float, float],
    second: tuple[float, float, float],
) -> tuple[float, float]:
    """Return planar and absolute wrapped-yaw differences between two poses."""
    return (
        math.hypot(second[0] - first[0], second[1] - first[1]),
        abs(math.degrees(_wrap_angle(second[2] - first[2]))),
    )


def _interpolate_pose(
    history: Deque[tuple[float, float, float, float]],
    stamp_sec: float,
    max_sync_sec: float,
) -> Optional[tuple[float, float, float, float]]:
    """Interpolate a pose at ``stamp_sec``; return x, y, yaw and bracket span."""
    if not history:
        return None
    first = history[0]
    last = history[-1]
    if stamp_sec <= first[0]:
        delta = first[0] - stamp_sec
        return (*first[1:], delta) if delta <= max_sync_sec else None
    if stamp_sec >= last[0]:
        delta = stamp_sec - last[0]
        return (*last[1:], delta) if delta <= max_sync_sec else None

    previous = first
    for current in list(history)[1:]:
        if current[0] < stamp_sec:
            previous = current
            continue
        span = current[0] - previous[0]
        if span <= 0.0 or span > 2.0 * max_sync_sec:
            return None
        ratio = (stamp_sec - previous[0]) / span
        yaw_delta = _wrap_angle(current[3] - previous[3])
        return (
            previous[1] + ratio * (current[1] - previous[1]),
            previous[2] + ratio * (current[2] - previous[2]),
            _wrap_angle(previous[3] + ratio * yaw_delta),
            span,
        )
    return None


def _relative_pose(
    first: dict[str, float], second: dict[str, float], prefix: str
) -> tuple[float, float, float]:
    """Return the second SE(2) pose relative to the first pose."""
    dx = second[f"{prefix}_x_m"] - first[f"{prefix}_x_m"]
    dy = second[f"{prefix}_y_m"] - first[f"{prefix}_y_m"]
    first_yaw = first[f"{prefix}_yaw"]
    cos_yaw = math.cos(first_yaw)
    sin_yaw = math.sin(first_yaw)
    return (
        cos_yaw * dx + sin_yaw * dy,
        -sin_yaw * dx + cos_yaw * dy,
        _wrap_angle(second[f"{prefix}_yaw"] - first_yaw),
    )


def _interpolate_sample_pose(
    samples: list[dict[str, float]],
    start_index: int,
    stamp_sec: float,
    prefix: str,
) -> Optional[dict[str, float]]:
    """Interpolate one stored trajectory at an exact RPE target timestamp."""
    previous = samples[start_index]
    for current in samples[start_index + 1:]:
        if current["time_s"] < stamp_sec:
            previous = current
            continue
        span = current["time_s"] - previous["time_s"]
        if span <= 0.0:
            return None
        ratio = (stamp_sec - previous["time_s"]) / span
        previous_yaw = previous[f"{prefix}_yaw"]
        yaw_delta = _wrap_angle(current[f"{prefix}_yaw"] - previous_yaw)
        return {
            f"{prefix}_x_m": previous[f"{prefix}_x_m"] + ratio * (
                current[f"{prefix}_x_m"] - previous[f"{prefix}_x_m"]
            ),
            f"{prefix}_y_m": previous[f"{prefix}_y_m"] + ratio * (
                current[f"{prefix}_y_m"] - previous[f"{prefix}_y_m"]
            ),
            f"{prefix}_yaw": _wrap_angle(previous_yaw + ratio * yaw_delta),
        }
    return None


class NavExperimentLogger(Node):
    """Record truth, estimates, and map-to-odom correction events without navigation control."""

    def __init__(self) -> None:
        super().__init__("nav_experiment_logger")
        self.declare_parameter(
            "output_dir", os.path.join(os.path.expanduser("~"), "auto_nav2_eval")
        )
        self.declare_parameter("run_name", "")
        self.declare_parameter("experiment_group", "unspecified")
        self.declare_parameter("sample_hz", 5.0)
        self.declare_parameter("robot_model_name", "fishbot")
        self.declare_parameter("ground_truth_topic", "/gazebo/model_states")
        self.declare_parameter("estimate_topic", "/odom_in_map")
        self.declare_parameter("auto_estimate_wait_sec", 2.0)
        self.declare_parameter("orb_event_topic", "/orb/match_event")
        self.declare_parameter(
            "correction_event_topic", "/localization/map_odom_correction_event")
        self.declare_parameter("align_initial_pose", True)
        self.declare_parameter("rpe_interval_sec", 1.0)
        self.declare_parameter("position_tolerance_m", 0.50)
        self.declare_parameter("yaw_tolerance_deg", 5.0)
        self.declare_parameter("ground_truth_history_sec", 10.0)
        self.declare_parameter("max_ground_truth_sync_sec", 0.10)
        self.declare_parameter("wait_for_initial_global_correction", True)
        self.declare_parameter("initial_estimate_settle_sec", 0.5)
        self.declare_parameter("initial_estimate_stability_translation_m", 0.5)
        self.declare_parameter("initial_estimate_stability_yaw_deg", 15.0)
        self.declare_parameter("stop_on_goal_reached", True)
        self.declare_parameter("goal_reached_topic", "/goal_reached")
        self.declare_parameter("stop_on_experiment_done", True)
        self.declare_parameter("experiment_done_topic", "/nav_experiment/done")
        self.declare_parameter("experiment_seed", -1)
        self.declare_parameter("continuous_orb_enabled", True)
        self.declare_parameter("orb_match_period_sec", 2.0)
        self.declare_parameter("orb_required_consistent_matches", 2)
        self.declare_parameter("orb_consistent_translation_m", 0.30)
        self.declare_parameter("orb_consistent_yaw_deg", 5.0)
        self.declare_parameter("orb_max_tracking_innovation_translation_m", 0.0)
        self.declare_parameter("orb_max_tracking_innovation_yaw_deg", 0.0)
        self.declare_parameter("orb_max_correction_linear_mps", 0.20)
        self.declare_parameter("orb_max_correction_angular_degps", 12.0)

        output_dir = str(self.get_parameter("output_dir").value)
        run_name = os.path.basename(str(self.get_parameter("run_name").value).strip())
        if not run_name:
            run_name = datetime.now().strftime("nav_%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(output_dir, run_name)
        os.makedirs(self.run_dir, exist_ok=False)

        self.robot_model_name = str(self.get_parameter("robot_model_name").value)
        self.experiment_group = (
            str(self.get_parameter("experiment_group").value).strip() or "unspecified"
        )
        self.align_initial_pose = bool(self.get_parameter("align_initial_pose").value)
        self.rpe_interval_sec = max(0.1, float(self.get_parameter("rpe_interval_sec").value))
        self.position_tolerance_m = float(
            self.get_parameter("position_tolerance_m").value
        )
        self.yaw_tolerance_deg = float(
            self.get_parameter("yaw_tolerance_deg").value
        )
        if self.position_tolerance_m <= 0.0 or self.yaw_tolerance_deg <= 0.0:
            raise ValueError("position_tolerance_m and yaw_tolerance_deg must be positive")
        self.estimate_topic = str(self.get_parameter("estimate_topic").value)
        self.auto_estimate_wait_sec = max(
            0.0, float(self.get_parameter("auto_estimate_wait_sec").value)
        )
        self._ground_truth_history_sec = max(
            1.0, float(self.get_parameter("ground_truth_history_sec").value)
        )
        self._max_ground_truth_sync_sec = max(
            0.001, float(self.get_parameter("max_ground_truth_sync_sec").value)
        )
        self._wait_for_initial_global = bool(
            self.get_parameter("wait_for_initial_global_correction").value
        )
        self._initial_estimate_settle_sec = max(
            0.0, float(self.get_parameter("initial_estimate_settle_sec").value)
        )
        self._initial_estimate_stability_translation_m = max(
            0.0,
            float(self.get_parameter(
                "initial_estimate_stability_translation_m"
            ).value),
        )
        self._initial_estimate_stability_yaw_deg = max(
            0.0,
            float(self.get_parameter("initial_estimate_stability_yaw_deg").value),
        )
        self._stop_on_goal_reached = bool(
            self.get_parameter("stop_on_goal_reached").value
        )
        self._stop_on_experiment_done = bool(
            self.get_parameter("stop_on_experiment_done").value
        )
        self._experiment_metadata = {
            "experiment_seed": int(self.get_parameter("experiment_seed").value),
            "continuous_orb_enabled": bool(
                self.get_parameter("continuous_orb_enabled").value
            ),
            "orb_match_period_sec": float(
                self.get_parameter("orb_match_period_sec").value
            ),
            "orb_required_consistent_matches": int(
                self.get_parameter("orb_required_consistent_matches").value
            ),
            "orb_consistent_translation_m": float(
                self.get_parameter("orb_consistent_translation_m").value
            ),
            "orb_consistent_yaw_deg": float(
                self.get_parameter("orb_consistent_yaw_deg").value
            ),
            "orb_max_tracking_innovation_translation_m": float(
                self.get_parameter(
                    "orb_max_tracking_innovation_translation_m"
                ).value
            ),
            "orb_max_tracking_innovation_yaw_deg": float(
                self.get_parameter("orb_max_tracking_innovation_yaw_deg").value
            ),
            "orb_max_correction_linear_mps": float(
                self.get_parameter("orb_max_correction_linear_mps").value
            ),
            "orb_max_correction_angular_degps": float(
                self.get_parameter("orb_max_correction_angular_degps").value
            ),
        }
        self._ground_truth: Optional[tuple[float, float, float]] = None
        self._ground_truth_history: Deque[tuple[float, float, float, float]] = deque()
        self._estimate: Optional[tuple[float, float, float, float]] = None
        self._estimates: dict[str, tuple[float, float, float, float]] = {}
        self._active_estimate_topic: Optional[str] = None
        self._model_states_received = False
        self._model_names: list[str] = []
        self._started_monotonic = time.monotonic()
        self._alignment: Optional[tuple[float, float, float]] = None
        self._samples: list[dict[str, float]] = []
        self._orb_events: list[dict[str, Any]] = []
        self._correction_events: list[dict[str, Any]] = []
        self._last_sample_stamp: Optional[float] = None
        self._first_sample_stamp: Optional[float] = None
        self._initial_global_applied = not self._wait_for_initial_global
        self._initial_global_received_stamp: Optional[float] = None
        self._initial_estimate_candidate: Optional[tuple[float, float, float]] = None
        self._initial_estimate_stabilized = not self._wait_for_initial_global
        self._goals_reached = 0
        self._goals_total_reported: Optional[int] = None
        self._goal_sequence_status: Optional[str] = None
        self._finished_reason = "manual_stop"
        self._recording_finished = False

        self._trajectory_file = open(
            os.path.join(self.run_dir, "trajectory.csv"), "w", newline="", encoding="utf-8"
        )
        self._trajectory_writer = csv.DictWriter(
            self._trajectory_file,
            fieldnames=[
                "time_s", "elapsed_s", "gt_sync_span_s",
                "gt_x_m", "gt_y_m", "gt_yaw_deg",
                "estimate_x_m", "estimate_y_m", "estimate_yaw_deg",
                "aligned_x_m", "aligned_y_m", "aligned_yaw_deg",
                "position_error_m", "yaw_error_signed_deg", "yaw_error_deg",
                "composite_pose_error", "joint_accuracy_satisfied",
            ],
        )
        self._trajectory_writer.writeheader()
        self._orb_file, self._orb_writer = self._open_event_csv("orb_events.csv")
        self._correction_file, self._correction_writer = self._open_correction_csv()

        self.create_subscription(
            ModelStates,
            str(self.get_parameter("ground_truth_topic").value),
            self._ground_truth_callback,
            20,
        )
        if self.estimate_topic == "auto":
            self.create_subscription(
                Odometry,
                "/odom_in_map",
                lambda msg: self._estimate_callback(msg, "/odom_in_map"),
                20,
            )
            self.create_subscription(
                Odometry,
                "/odom",
                lambda msg: self._estimate_callback(msg, "/odom"),
                20,
            )
        else:
            self.create_subscription(
                Odometry,
                self.estimate_topic,
                lambda msg: self._estimate_callback(msg, self.estimate_topic),
                20,
            )
        self.create_subscription(
            String, str(self.get_parameter("orb_event_topic").value), self._orb_event_callback, 20
        )
        correction_event_qos = QoSProfile(
            depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(
            String,
            str(self.get_parameter("correction_event_topic").value),
            self._correction_event_callback,
            correction_event_qos,
        )
        self.create_subscription(
            Empty,
            str(self.get_parameter("goal_reached_topic").value),
            self._goal_reached_callback,
            10,
        )
        if self._stop_on_experiment_done:
            self.create_subscription(
                String,
                str(self.get_parameter("experiment_done_topic").value),
                self._experiment_done_callback,
                10,
            )
        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        sample_hz = max(0.1, float(self.get_parameter("sample_hz").value))
        self.create_timer(1.0 / sample_hz, self._sample, clock=steady_clock)
        self.create_timer(2.0, self._print_status, clock=steady_clock)
        self.get_logger().info(
            f"导航试验记录已启动：{self.run_dir}；"
            f"position/yaw tolerance={self.position_tolerance_m:.2f}m/"
            f"{self.yaw_tolerance_deg:.1f}deg"
        )

    def _open_event_csv(self, filename: str):
        file = open(os.path.join(self.run_dir, filename), "w", newline="", encoding="utf-8")
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "stamp_sec", "attempt", "status", "elapsed_ms", "f1", "timed_out",
                "delta_diff_m", "yaw_diff_deg",
            ],
        )
        writer.writeheader()
        return file, writer

    def _open_correction_csv(self):
        file = open(
            os.path.join(self.run_dir, "correction_events.csv"), "w", newline="", encoding="utf-8"
        )
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "stamp_sec", "source", "status", "target_x_m", "target_y_m",
                "target_yaw_deg", "translation_innovation_m", "yaw_innovation_deg",
                "tracking_translation_innovation_m",
                "tracking_yaw_innovation_deg",
            ],
        )
        writer.writeheader()
        return file, writer

    def _ground_truth_callback(self, msg: ModelStates) -> None:
        self._model_states_received = True
        self._model_names = list(msg.name)
        try:
            index = msg.name.index(self.robot_model_name)
        except ValueError:
            return
        pose = msg.pose[index]
        self._ground_truth = (
            pose.position.x,
            pose.position.y,
            _yaw_from_quaternion(pose.orientation),
        )
        stamp_sec = self.get_clock().now().nanoseconds * 1e-9
        self._ground_truth_history.append((stamp_sec, *self._ground_truth))
        cutoff = stamp_sec - self._ground_truth_history_sec
        while self._ground_truth_history[0][0] < cutoff:
            self._ground_truth_history.popleft()

    def _estimate_callback(self, msg: Odometry, topic: str) -> None:
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        pose = msg.pose.pose
        estimate = (
            pose.position.x,
            pose.position.y,
            _yaw_from_quaternion(pose.orientation),
            stamp_sec,
        )
        self._estimates[topic] = estimate
        if (
            self.estimate_topic != "auto"
            or topic == "/odom_in_map"
            or topic == self._active_estimate_topic
        ):
            self._select_estimate_topic(topic)

    def _select_estimate_topic(self, topic: str) -> None:
        if topic not in self._estimates:
            return
        if self._active_estimate_topic == topic:
            self._estimate = self._estimates[topic]
            return
        if self._samples:
            return
        self._active_estimate_topic = topic
        self._estimate = self._estimates[topic]
        self.get_logger().info(f"估计轨迹来源：{topic}")

    def _print_status(self) -> None:
        if self.estimate_topic == "auto" and self._active_estimate_topic is None:
            if "/odom_in_map" in self._estimates:
                self._select_estimate_topic("/odom_in_map")
            elif (
                "/odom" in self._estimates
                and time.monotonic() - self._started_monotonic >= self.auto_estimate_wait_sec
            ):
                self._select_estimate_topic("/odom")

        missing = []
        if self._ground_truth is None:
            if self._model_states_received:
                names = ", ".join(self._model_names[:8]) or "（空）"
                missing.append(f"模型 {self.robot_model_name!r}（当前模型：{names}）")
            else:
                missing.append("/gazebo/model_states 真值")
        if self._estimate is None:
            expected = (
                "/odom_in_map 或 /odom"
                if self.estimate_topic == "auto"
                else self.estimate_topic
            )
            missing.append(f"{expected} 估计里程计")
        if not self._initial_global_applied:
            missing.append("首次全局 ORB map→odom 应用事件")
        if missing:
            self.get_logger().warn("等待 " + "；".join(missing))
            return
        self.get_logger().info(
            f"正在记录：{len(self._samples)} 个轨迹样本，估计来源 {self._active_estimate_topic}"
        )

    def _parse_event(self, msg: String) -> Optional[dict[str, Any]]:
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("忽略无法解析的实验事件")
            return None
        return event if isinstance(event, dict) else None

    def _orb_event_callback(self, msg: String) -> None:
        if self._recording_finished:
            return
        event = self._parse_event(msg)
        if event is None:
            return
        self._orb_events.append(event)
        self._write_event(self._orb_writer, event)
        self._orb_file.flush()

    def _correction_event_callback(self, msg: String) -> None:
        if self._recording_finished:
            return
        event = self._parse_event(msg)
        if event is None:
            return
        if event.get("status") == "global_applied" and not self._initial_global_applied:
            self._initial_global_applied = True
            self._initial_global_received_stamp = (
                self.get_clock().now().nanoseconds * 1e-9
            )
            self._initial_estimate_candidate = None
            self._initial_estimate_stabilized = False
            self._alignment = None
            self._last_sample_stamp = None
            self._first_sample_stamp = None
            self.get_logger().info("首次全局定位已应用，开始记录实验轨迹")
        self._correction_events.append(event)
        self._write_event(self._correction_writer, event)
        self._correction_file.flush()

    @staticmethod
    def _write_event(writer: csv.DictWriter, event: dict[str, Any]) -> None:
        writer.writerow({key: event.get(key, "") for key in writer.fieldnames})

    def _sample(self) -> None:
        if (
            self._recording_finished
            or not self._initial_global_applied
            or self._estimate is None
        ):
            return
        estimate_x, estimate_y, estimate_yaw, stamp_sec = self._estimate
        if stamp_sec <= 0.0:
            stamp_sec = self.get_clock().now().nanoseconds * 1e-9
        if not self._initial_estimate_stabilized:
            if self._initial_global_received_stamp is None:
                return
            if stamp_sec < (
                self._initial_global_received_stamp
                + self._initial_estimate_settle_sec
            ):
                return
            current_pose = (estimate_x, estimate_y, estimate_yaw)
            if self._initial_estimate_candidate is None:
                self._initial_estimate_candidate = current_pose
                return
            translation_diff, yaw_diff = _pose_difference(
                self._initial_estimate_candidate, current_pose
            )
            self._initial_estimate_candidate = current_pose
            if (
                translation_diff
                > self._initial_estimate_stability_translation_m
                or yaw_diff > self._initial_estimate_stability_yaw_deg
            ):
                self.get_logger().warn(
                    "首次全局定位后的估计位姿仍在跳变，继续等待稳定："
                    f"translation={translation_diff:.2f}m, yaw={yaw_diff:.1f}deg"
                )
                return
            self._initial_estimate_stabilized = True
            self.get_logger().info("全局定位后的估计位姿已稳定，建立首帧对齐")
        if self._last_sample_stamp is not None and stamp_sec <= self._last_sample_stamp:
            return
        synchronized_truth = _interpolate_pose(
            self._ground_truth_history,
            stamp_sec,
            self._max_ground_truth_sync_sec,
        )
        if synchronized_truth is None:
            return
        gt_x, gt_y, gt_yaw, gt_sync_span = synchronized_truth

        if self._alignment is None:
            if self.align_initial_pose:
                yaw_offset = _wrap_angle(gt_yaw - estimate_yaw)
                cos_yaw = math.cos(yaw_offset)
                sin_yaw = math.sin(yaw_offset)
                offset_x = gt_x - (cos_yaw * estimate_x - sin_yaw * estimate_y)
                offset_y = gt_y - (sin_yaw * estimate_x + cos_yaw * estimate_y)
            else:
                offset_x, offset_y, yaw_offset = 0.0, 0.0, 0.0
            self._alignment = (offset_x, offset_y, yaw_offset)

        offset_x, offset_y, yaw_offset = self._alignment
        cos_yaw = math.cos(yaw_offset)
        sin_yaw = math.sin(yaw_offset)
        aligned_x = offset_x + cos_yaw * estimate_x - sin_yaw * estimate_y
        aligned_y = offset_y + sin_yaw * estimate_x + cos_yaw * estimate_y
        aligned_yaw = _wrap_angle(estimate_yaw + yaw_offset)
        position_error = math.hypot(aligned_x - gt_x, aligned_y - gt_y)
        yaw_error_signed = math.degrees(_wrap_angle(aligned_yaw - gt_yaw))
        yaw_error = abs(yaw_error_signed)
        composite_error = _composite_pose_error(
            position_error,
            yaw_error,
            self.position_tolerance_m,
            self.yaw_tolerance_deg,
        )
        joint_satisfied = _joint_accuracy_satisfied(
            position_error,
            yaw_error_signed,
            self.position_tolerance_m,
            self.yaw_tolerance_deg,
        )
        if self._first_sample_stamp is None:
            self._first_sample_stamp = stamp_sec
        elapsed_s = stamp_sec - self._first_sample_stamp
        sample = {
            "time_s": stamp_sec, "elapsed_s": elapsed_s,
            "gt_sync_span_s": gt_sync_span,
            "gt_x_m": gt_x, "gt_y_m": gt_y, "gt_yaw": gt_yaw,
            "estimate_x_m": estimate_x, "estimate_y_m": estimate_y, "estimate_yaw": estimate_yaw,
            "aligned_x_m": aligned_x, "aligned_y_m": aligned_y, "aligned_yaw": aligned_yaw,
            "position_error_m": position_error,
            "yaw_error_signed_deg": yaw_error_signed,
            "yaw_error_deg": yaw_error,
            "composite_pose_error": composite_error,
            "joint_accuracy_satisfied": joint_satisfied,
        }
        self._samples.append(sample)
        self._last_sample_stamp = stamp_sec
        if len(self._samples) == 1:
            self.get_logger().info("已写入第一个轨迹样本。")
        self._trajectory_writer.writerow({
            "time_s": f"{stamp_sec:.6f}", "elapsed_s": f"{elapsed_s:.6f}",
            "gt_sync_span_s": f"{gt_sync_span:.6f}",
            "gt_x_m": f"{gt_x:.6f}", "gt_y_m": f"{gt_y:.6f}",
            "gt_yaw_deg": f"{math.degrees(gt_yaw):.3f}", "estimate_x_m": f"{estimate_x:.6f}",
            "estimate_y_m": f"{estimate_y:.6f}",
            "estimate_yaw_deg": f"{math.degrees(estimate_yaw):.3f}",
            "aligned_x_m": f"{aligned_x:.6f}", "aligned_y_m": f"{aligned_y:.6f}",
            "aligned_yaw_deg": f"{math.degrees(aligned_yaw):.3f}",
            "position_error_m": f"{position_error:.6f}",
            "yaw_error_signed_deg": f"{yaw_error_signed:.3f}",
            "yaw_error_deg": f"{yaw_error:.3f}",
            "composite_pose_error": f"{composite_error:.6f}",
            "joint_accuracy_satisfied": int(joint_satisfied),
        })
        self._trajectory_file.flush()

    def _goal_reached_callback(self, _msg: Empty) -> None:
        if self._recording_finished:
            return
        self._goals_reached += 1
        if not self._stop_on_goal_reached:
            return
        self._finish_recording("first_goal_reached", "检测到首次到点")

    def _experiment_done_callback(self, msg: String) -> None:
        if self._recording_finished:
            return
        event = self._parse_event(msg)
        if event is None:
            status = msg.data.strip() or "completed"
            event = {"status": status}
        self._goal_sequence_status = str(event.get("status", "completed"))
        if event.get("goals_total") is not None:
            self._goals_total_reported = int(event["goals_total"])
        if event.get("goals_reached") is not None:
            self._goals_reached = max(
                self._goals_reached, int(event["goals_reached"])
            )
        completed = self._goal_sequence_status == "completed"
        reason = "goal_sequence_completed" if completed else "goal_sequence_aborted"
        message = "目标序列已全部完成" if completed else "目标序列提前终止"
        self._finish_recording(reason, message)

    def _finish_recording(self, reason: str, message: str) -> None:
        self._sample()
        self._finished_reason = reason
        self._recording_finished = True
        self.get_logger().info(f"{message}，已冻结实验数据并生成汇总")
        rclpy.shutdown()

    def _rpe_values(self) -> tuple[list[float], list[float]]:
        position_errors, yaw_errors = [], []
        for index, first in enumerate(self._samples):
            target_time = first["time_s"] + self.rpe_interval_sec
            gt_second = _interpolate_sample_pose(
                self._samples, index, target_time, "gt"
            )
            est_second = _interpolate_sample_pose(
                self._samples, index, target_time, "aligned"
            )
            if gt_second is None or est_second is None:
                continue
            gt_dx, gt_dy, gt_dyaw = _relative_pose(first, gt_second, "gt")
            est_dx, est_dy, est_dyaw = _relative_pose(
                first, est_second, "aligned"
            )
            position_errors.append(math.hypot(est_dx - gt_dx, est_dy - gt_dy))
            yaw_errors.append(abs(math.degrees(_wrap_angle(est_dyaw - gt_dyaw))))
        return position_errors, yaw_errors

    def _summary(self) -> dict[str, Any]:
        position_errors = [sample["position_error_m"] for sample in self._samples]
        yaw_errors = [sample["yaw_error_deg"] for sample in self._samples]
        composite_errors = [
            sample["composite_pose_error"] for sample in self._samples
        ]
        joint_satisfied_count = sum(
            bool(sample["joint_accuracy_satisfied"]) for sample in self._samples
        )
        rpe_position, rpe_yaw = self._rpe_values()
        orb_attempts = len(self._orb_events)
        matched = sum(event.get("status") == "matched" for event in self._orb_events)
        timed_out = sum(bool(event.get("timed_out")) for event in self._orb_events)
        accepted = sum(
            event.get("status") == "accepted" and event.get("source") == "orb"
            for event in self._correction_events
        )
        held = sum(
            event.get("status") == "held" and event.get("source") == "orb"
            for event in self._correction_events
        )
        innovation_rejected = sum(
            event.get("status") == "innovation_rejected"
            and event.get("source") == "orb"
            for event in self._correction_events
        )
        global_applied = sum(
            event.get("status") == "global_applied" for event in self._correction_events
        )
        elapsed_ms = [
            float(event["elapsed_ms"])
            for event in self._orb_events
            if "elapsed_ms" in event
        ]
        accepted_corrections = [
            event for event in self._correction_events
            if event.get("status") == "accepted" and event.get("source") == "orb"
        ]
        translation_innovations = [
            abs(float(event["translation_innovation_m"]))
            for event in accepted_corrections
            if event.get("translation_innovation_m") is not None
        ]
        yaw_innovations = [
            abs(float(event["yaw_innovation_deg"]))
            for event in accepted_corrections
            if event.get("yaw_innovation_deg") is not None
        ]
        endpoint_error = position_errors[-1] if position_errors else None
        gt_sync_spans = [sample["gt_sync_span_s"] for sample in self._samples]
        completion_valid = (
            self._finished_reason == "first_goal_reached"
            or (
                self._finished_reason == "goal_sequence_completed"
                and self._goal_sequence_status == "completed"
                and self._goals_total_reported is not None
                and self._goals_reached == self._goals_total_reported
            )
        )
        paper_statistics_eligible = (
            self._initial_global_applied
            and self._initial_estimate_stabilized
            and len(self._samples) >= 2
            and completion_valid
        )
        return {
            "run_name": os.path.basename(self.run_dir),
            "experiment_group": self.experiment_group,
            "sample_count": len(self._samples),
            "goals_reached": self._goals_reached,
            "goals_total": self._goals_total_reported,
            "goal_sequence_status": self._goal_sequence_status,
            "finished_reason": self._finished_reason,
            "paper_statistics_eligible": paper_statistics_eligible,
            "estimate_topic": self._active_estimate_topic,
            "ground_truth_sync": "timestamp_interpolation",
            "ground_truth_sync_span_mean_s": (
                None if not gt_sync_spans else sum(gt_sync_spans) / len(gt_sync_spans)
            ),
            "ground_truth_sync_span_max_s": (
                None if not gt_sync_spans else max(gt_sync_spans)
            ),
            "alignment": "initial_pose" if self.align_initial_pose else "none",
            "waited_for_initial_global_correction": self._wait_for_initial_global,
            "initial_global_correction_observed": self._initial_global_applied,
            "initial_estimate_stabilized": self._initial_estimate_stabilized,
            "initial_estimate_settle_sec": self._initial_estimate_settle_sec,
            "position_tolerance_m": self.position_tolerance_m,
            "yaw_tolerance_deg": self.yaw_tolerance_deg,
            "ate_rmse_m": _rms(position_errors),
            "ate_mean_m": (
                None if not position_errors else sum(position_errors) / len(position_errors)
            ),
            "ate_p95_m": _percentile(position_errors, 95.0),
            "yaw_mean_deg": None if not yaw_errors else sum(yaw_errors) / len(yaw_errors),
            "yaw_p95_deg": _percentile(yaw_errors, 95.0),
            "rpe_interval_sec": self.rpe_interval_sec,
            "rpe_translation_rmse_m": _rms(rpe_position),
            "rpe_yaw_rmse_deg": _rms(rpe_yaw),
            "endpoint_error_m": endpoint_error,
            "composite_pose_error_rmse": _rms(composite_errors),
            "composite_pose_error_p95": _percentile(composite_errors, 95.0),
            "joint_accuracy_rate_pct": _percent(
                joint_satisfied_count, len(self._samples)
            ),
            "max_accepted_correction_translation_m": (
                max(translation_innovations) if translation_innovations else None
            ),
            "max_accepted_correction_yaw_deg": (
                max(yaw_innovations) if yaw_innovations else None
            ),
            "orb_attempts": orb_attempts, "orb_matched": matched, "orb_timeout": timed_out,
            "orb_correction_accepted": accepted,
            "orb_observation_held": held,
            "orb_innovation_rejected": innovation_rejected,
            "global_correction_applied": global_applied,
            "orb_correction_acceptance_rate_pct": _percent(accepted, matched),
            "timeout_rate_pct": _percent(timed_out, orb_attempts),
            "orb_mean_compute_ms": (
                None if not elapsed_ms else sum(elapsed_ms) / len(elapsed_ms)
            ),
            **self._experiment_metadata,
        }

    def destroy_node(self) -> bool:
        summary = self._summary()
        with open(os.path.join(self.run_dir, "summary.json"), "w", encoding="utf-8") as file:
            json.dump(summary, file, ensure_ascii=False, indent=2)
        self._write_aggregate_summary()
        self._trajectory_file.close()
        self._orb_file.close()
        self._correction_file.close()
        print(json.dumps(summary, ensure_ascii=False))
        return super().destroy_node()

    def _write_aggregate_summary(self) -> None:
        output_dir = os.path.dirname(self.run_dir)
        summaries = []
        for name in sorted(os.listdir(output_dir)):
            path = os.path.join(output_dir, name, "summary.json")
            if not os.path.isfile(path):
                continue
            with open(path, encoding="utf-8") as file:
                summaries.append(json.load(file))
        if not summaries:
            return
        fieldnames = list(dict.fromkeys(
            key for summary in summaries for key in summary.keys()
        ))
        with open(
            os.path.join(output_dir, "all_runs_summary.csv"),
            "w",
            newline="",
            encoding="utf-8",
        ) as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(summaries)
        self._write_group_summary(output_dir, summaries)

    @staticmethod
    def _write_group_summary(output_dir: str, summaries: list[dict[str, Any]]) -> None:
        metric_names = [
            "ate_rmse_m", "ate_mean_m", "ate_p95_m", "yaw_p95_deg",
            "rpe_translation_rmse_m", "rpe_yaw_rmse_deg", "endpoint_error_m",
            "composite_pose_error_rmse", "composite_pose_error_p95",
            "joint_accuracy_rate_pct", "max_accepted_correction_translation_m",
            "max_accepted_correction_yaw_deg", "orb_correction_acceptance_rate_pct",
            "timeout_rate_pct", "orb_mean_compute_ms",
        ]
        grouped: dict[str, list[dict[str, Any]]] = {}
        for summary in summaries:
            grouped.setdefault(summary.get("experiment_group", "unspecified"), []).append(summary)
        rows = []
        for group, group_summaries in sorted(grouped.items()):
            eligible_summaries = [
                summary for summary in group_summaries
                if summary.get("paper_statistics_eligible") is True
                and summary.get("initial_estimate_stabilized") is True
            ]
            row = {
                "experiment_group": group,
                "run_count": len(eligible_summaries),
                "run_count_total": len(group_summaries),
                "run_count_eligible": len(eligible_summaries),
                "sample_count_total": sum(
                    summary.get("sample_count", 0) for summary in group_summaries
                ),
                "orb_attempts_total": sum(
                    summary.get("orb_attempts", 0) for summary in group_summaries
                ),
                "orb_correction_accepted_total": sum(
                    summary.get("orb_correction_accepted", 0) for summary in group_summaries
                ),
                "orb_observation_held_total": sum(
                    summary.get("orb_observation_held", 0) for summary in group_summaries
                ),
                "orb_innovation_rejected_total": sum(
                    summary.get("orb_innovation_rejected", 0)
                    for summary in group_summaries
                ),
                "orb_timeout_total": sum(
                    summary.get("orb_timeout", 0) for summary in group_summaries
                ),
            }
            for metric in metric_names:
                values = [
                    float(summary[metric])
                    for summary in eligible_summaries
                    if summary.get(metric) is not None
                ]
                row[f"{metric}_mean"] = None if not values else sum(values) / len(values)
                row[f"{metric}_std"] = _sample_standard_deviation(values)
            rows.append(row)
        with open(
            os.path.join(output_dir, "group_summary.csv"),
            "w",
            newline="",
            encoding="utf-8",
        ) as file:
            writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavExperimentLogger()
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
