#!/usr/bin/env python3
"""Record one manual navigation experiment and its paper-ready trajectory metrics."""

import csv
import json
import math
import os
import time
from datetime import datetime
from typing import Any, Optional

import rclpy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


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
        self.declare_parameter("estimate_topic", "auto")
        self.declare_parameter("auto_estimate_wait_sec", 2.0)
        self.declare_parameter("orb_event_topic", "/orb/match_event")
        self.declare_parameter(
            "correction_event_topic", "/localization/map_odom_correction_event")
        self.declare_parameter("align_initial_pose", True)
        self.declare_parameter("rpe_interval_sec", 1.0)

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
        self.estimate_topic = str(self.get_parameter("estimate_topic").value)
        self.auto_estimate_wait_sec = max(
            0.0, float(self.get_parameter("auto_estimate_wait_sec").value)
        )
        self._ground_truth: Optional[tuple[float, float, float]] = None
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

        self._trajectory_file = open(
            os.path.join(self.run_dir, "trajectory.csv"), "w", newline="", encoding="utf-8"
        )
        self._trajectory_writer = csv.DictWriter(
            self._trajectory_file,
            fieldnames=[
                "time_s", "gt_x_m", "gt_y_m", "gt_yaw_deg",
                "estimate_x_m", "estimate_y_m", "estimate_yaw_deg",
                "aligned_x_m", "aligned_y_m", "aligned_yaw_deg",
                "position_error_m", "yaw_error_deg",
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
        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        sample_hz = max(0.1, float(self.get_parameter("sample_hz").value))
        self.create_timer(1.0 / sample_hz, self._sample, clock=steady_clock)
        self.create_timer(2.0, self._print_status, clock=steady_clock)
        self.get_logger().info(
            f"人工导航试验记录已启动：{self.run_dir}（Ctrl+C 停止并生成 summary.json）"
        )

    def _open_event_csv(self, filename: str):
        file = open(os.path.join(self.run_dir, filename), "w", newline="", encoding="utf-8")
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "stamp_sec", "status", "elapsed_ms", "timed_out", "delta_diff_m", "yaw_diff_deg"
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
                "target_yaw_deg", "yaw_innovation_deg",
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
        event = self._parse_event(msg)
        if event is None:
            return
        self._orb_events.append(event)
        self._write_event(self._orb_writer, event)

    def _correction_event_callback(self, msg: String) -> None:
        event = self._parse_event(msg)
        if event is None:
            return
        self._correction_events.append(event)
        self._write_event(self._correction_writer, event)

    @staticmethod
    def _write_event(writer: csv.DictWriter, event: dict[str, Any]) -> None:
        writer.writerow({key: event.get(key, "") for key in writer.fieldnames})

    def _sample(self) -> None:
        if self._ground_truth is None or self._estimate is None:
            return
        gt_x, gt_y, gt_yaw = self._ground_truth
        estimate_x, estimate_y, estimate_yaw, stamp_sec = self._estimate
        if stamp_sec <= 0.0:
            stamp_sec = self.get_clock().now().nanoseconds * 1e-9

        if self._alignment is None:
            yaw_offset = _wrap_angle(gt_yaw - estimate_yaw) if self.align_initial_pose else 0.0
            cos_yaw = math.cos(yaw_offset)
            sin_yaw = math.sin(yaw_offset)
            offset_x = gt_x - (cos_yaw * estimate_x - sin_yaw * estimate_y)
            offset_y = gt_y - (sin_yaw * estimate_x + cos_yaw * estimate_y)
            self._alignment = (offset_x, offset_y, yaw_offset)

        offset_x, offset_y, yaw_offset = self._alignment
        cos_yaw = math.cos(yaw_offset)
        sin_yaw = math.sin(yaw_offset)
        aligned_x = offset_x + cos_yaw * estimate_x - sin_yaw * estimate_y
        aligned_y = offset_y + sin_yaw * estimate_x + cos_yaw * estimate_y
        aligned_yaw = _wrap_angle(estimate_yaw + yaw_offset)
        position_error = math.hypot(aligned_x - gt_x, aligned_y - gt_y)
        yaw_error = abs(math.degrees(_wrap_angle(aligned_yaw - gt_yaw)))
        sample = {
            "time_s": stamp_sec, "gt_x_m": gt_x, "gt_y_m": gt_y, "gt_yaw": gt_yaw,
            "estimate_x_m": estimate_x, "estimate_y_m": estimate_y, "estimate_yaw": estimate_yaw,
            "aligned_x_m": aligned_x, "aligned_y_m": aligned_y, "aligned_yaw": aligned_yaw,
            "position_error_m": position_error, "yaw_error_deg": yaw_error,
        }
        self._samples.append(sample)
        if len(self._samples) == 1:
            self.get_logger().info("已写入第一个轨迹样本。")
        self._trajectory_writer.writerow({
            "time_s": f"{stamp_sec:.6f}", "gt_x_m": f"{gt_x:.6f}", "gt_y_m": f"{gt_y:.6f}",
            "gt_yaw_deg": f"{math.degrees(gt_yaw):.3f}", "estimate_x_m": f"{estimate_x:.6f}",
            "estimate_y_m": f"{estimate_y:.6f}",
            "estimate_yaw_deg": f"{math.degrees(estimate_yaw):.3f}",
            "aligned_x_m": f"{aligned_x:.6f}", "aligned_y_m": f"{aligned_y:.6f}",
            "aligned_yaw_deg": f"{math.degrees(aligned_yaw):.3f}",
            "position_error_m": f"{position_error:.6f}",
            "yaw_error_deg": f"{yaw_error:.3f}",
        })
        self._trajectory_file.flush()

    def _rpe_values(self) -> tuple[list[float], list[float]]:
        position_errors, yaw_errors = [], []
        for index, first in enumerate(self._samples):
            target_time = first["time_s"] + self.rpe_interval_sec
            second = next(
                (
                    sample
                    for sample in self._samples[index + 1:]
                    if sample["time_s"] >= target_time
                ),
                None,
            )
            if second is None:
                continue
            gt_dx = second["gt_x_m"] - first["gt_x_m"]
            gt_dy = second["gt_y_m"] - first["gt_y_m"]
            est_dx = second["aligned_x_m"] - first["aligned_x_m"]
            est_dy = second["aligned_y_m"] - first["aligned_y_m"]
            position_errors.append(math.hypot(est_dx - gt_dx, est_dy - gt_dy))
            gt_dyaw = _wrap_angle(second["gt_yaw"] - first["gt_yaw"])
            est_dyaw = _wrap_angle(second["aligned_yaw"] - first["aligned_yaw"])
            yaw_errors.append(abs(math.degrees(_wrap_angle(est_dyaw - gt_dyaw))))
        return position_errors, yaw_errors

    def _summary(self) -> dict[str, Any]:
        position_errors = [sample["position_error_m"] for sample in self._samples]
        yaw_errors = [sample["yaw_error_deg"] for sample in self._samples]
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
        global_applied = sum(
            event.get("status") == "global_applied" for event in self._correction_events
        )
        elapsed_ms = [
            float(event["elapsed_ms"])
            for event in self._orb_events
            if "elapsed_ms" in event
        ]
        endpoint_error = position_errors[-1] if position_errors else None
        return {
            "run_name": os.path.basename(self.run_dir),
            "experiment_group": self.experiment_group,
            "sample_count": len(self._samples),
            "alignment": "initial_pose" if self.align_initial_pose else "none",
            "ate_rmse_m": _rms(position_errors),
            "ate_mean_m": (
                None if not position_errors else sum(position_errors) / len(position_errors)
            ),
            "ate_p95_m": _percentile(position_errors, 95.0),
            "yaw_mean_deg": None if not yaw_errors else sum(yaw_errors) / len(yaw_errors),
            "rpe_interval_sec": self.rpe_interval_sec,
            "rpe_translation_rmse_m": _rms(rpe_position),
            "rpe_yaw_rmse_deg": _rms(rpe_yaw),
            "endpoint_error_m": endpoint_error,
            "orb_attempts": orb_attempts, "orb_matched": matched, "orb_timeout": timed_out,
            "orb_correction_accepted": accepted,
            "orb_observation_held": held,
            "global_correction_applied": global_applied,
            "orb_correction_acceptance_rate_pct": _percent(accepted, matched),
            "timeout_rate_pct": _percent(timed_out, orb_attempts),
            "orb_mean_compute_ms": (
                None if not elapsed_ms else sum(elapsed_ms) / len(elapsed_ms)
            ),
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
            "ate_rmse_m", "ate_mean_m", "ate_p95_m", "rpe_translation_rmse_m",
            "rpe_yaw_rmse_deg", "endpoint_error_m", "orb_correction_acceptance_rate_pct",
            "timeout_rate_pct", "orb_mean_compute_ms",
        ]
        grouped: dict[str, list[dict[str, Any]]] = {}
        for summary in summaries:
            grouped.setdefault(summary.get("experiment_group", "unspecified"), []).append(summary)
        rows = []
        for group, group_summaries in sorted(grouped.items()):
            row = {
                "experiment_group": group,
                "run_count": len(group_summaries),
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
                "orb_timeout_total": sum(
                    summary.get("orb_timeout", 0) for summary in group_summaries
                ),
            }
            for metric in metric_names:
                values = [
                    float(summary[metric])
                    for summary in group_summaries
                    if summary.get(metric) is not None
                ]
                row[f"{metric}_mean"] = None if not values else sum(values) / len(values)
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
