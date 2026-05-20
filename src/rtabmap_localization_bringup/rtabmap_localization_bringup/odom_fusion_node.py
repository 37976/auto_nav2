#!/usr/bin/env python3

import csv
import math
import os
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import String


def _yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class OdomFusionNode(Node):
    def __init__(self) -> None:
        super().__init__("odom_fusion_node")
        self.declare_parameter("base_odom_topic", "/odom")
        self.declare_parameter("xfeat_delta_topic", "/xfeat/delta_odom")
        self.declare_parameter("output_odom_topic", "/localized_odom")
        self.declare_parameter("correction_gain_xy", 0.15)
        self.declare_parameter("correction_gain_yaw", 0.10)
        self.declare_parameter("xfeat_timeout_sec", 1.0)
        self.declare_parameter("log_period_sec", 0.5)
        self.declare_parameter("max_delta_translation_diff_m", 0.20)
        self.declare_parameter("max_delta_yaw_diff_deg", 20.0)
        self.declare_parameter("csv_log_path", os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "..", "..", "..", "XFeat_pose", "sim_odom_fusion_debug.csv")))
        self.declare_parameter("path_topic", "/path")
        self.declare_parameter("control_mode_topic", "/control_mode")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("nav_idle_timeout_sec", 2.0)

        self.base_odom_topic = str(self.get_parameter("base_odom_topic").value)
        self.xfeat_delta_topic = str(self.get_parameter("xfeat_delta_topic").value)
        self.output_odom_topic = str(self.get_parameter("output_odom_topic").value)
        self.gain_xy = float(self.get_parameter("correction_gain_xy").value)
        self.gain_yaw = float(self.get_parameter("correction_gain_yaw").value)
        self.xfeat_timeout_sec = float(self.get_parameter("xfeat_timeout_sec").value)
        self.log_period_sec = max(0.1, float(self.get_parameter("log_period_sec").value))
        self.max_delta_translation_diff_m = float(self.get_parameter("max_delta_translation_diff_m").value)
        self.max_delta_yaw_diff_rad = math.radians(float(self.get_parameter("max_delta_yaw_diff_deg").value))
        self.csv_log_path = str(self.get_parameter("csv_log_path").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.control_mode_topic = str(self.get_parameter("control_mode_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.nav_idle_timeout_sec = max(0.5, float(self.get_parameter("nav_idle_timeout_sec").value))

        self.xfeat_delta: Optional[Odometry] = None
        self.last_xfeat_stamp_sec: Optional[float] = None
        self.last_log_sec: Optional[float] = None
        self.prev_base_odom: Optional[Odometry] = None
        self.fused_x: Optional[float] = None
        self.fused_y: Optional[float] = None
        self.fused_yaw: Optional[float] = None
        self._last_fusion_status = "base_only"
        self._last_status_details = ""
        self._last_csv_row = None
        self.control_mode = "nav"
        self.path_active = False
        self.last_path_sec: Optional[float] = None
        self.last_nonzero_cmd_sec: Optional[float] = None

        self.create_subscription(Odometry, self.base_odom_topic, self._base_odom_cb, 20)
        self.create_subscription(Odometry, self.xfeat_delta_topic, self._xfeat_delta_cb, 20)
        self.create_subscription(Path, self.path_topic, self._path_cb, 10)
        self.create_subscription(String, self.control_mode_topic, self._control_mode_cb, 10)
        self.create_subscription(Twist, self.cmd_vel_topic, self._cmd_vel_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 20)

        self.get_logger().info(
            f"Fusing {self.base_odom_topic} with local delta {self.xfeat_delta_topic} -> {self.output_odom_topic}"
        )
        self._init_csv_log()

    def _xfeat_delta_cb(self, msg: Odometry) -> None:
        self.xfeat_delta = msg
        self.last_xfeat_stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def _base_odom_cb(self, msg: Odometry) -> None:
        if self.prev_base_odom is None or self.fused_x is None or self.fused_y is None or self.fused_yaw is None:
            self.prev_base_odom = msg
            self.fused_x = float(msg.pose.pose.position.x)
            self.fused_y = float(msg.pose.pose.position.y)
            self.fused_yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
            self.odom_pub.publish(msg)
            return
        fused = self._fuse(msg)
        self.odom_pub.publish(fused)
        self._log_fused_pose(msg, fused)
        self.prev_base_odom = msg

    def _path_cb(self, msg: Path) -> None:
        self.path_active = len(msg.poses) >= 2
        if self.path_active:
            self.last_path_sec = self.get_clock().now().nanoseconds * 1e-9

    def _control_mode_cb(self, msg: String) -> None:
        self.control_mode = msg.data.strip().lower()

    def _cmd_vel_cb(self, msg: Twist) -> None:
        if abs(float(msg.linear.x)) > 1e-3 or abs(float(msg.angular.z)) > 1e-3:
            self.last_nonzero_cmd_sec = self.get_clock().now().nanoseconds * 1e-9

    def _fuse(self, base_msg: Odometry) -> Odometry:
        assert self.prev_base_odom is not None
        assert self.fused_x is not None and self.fused_y is not None and self.fused_yaw is not None

        fused = Odometry()
        fused.header = base_msg.header
        fused.child_frame_id = base_msg.child_frame_id
        fused.twist = base_msg.twist
        fused.pose = base_msg.pose

        prev_base_x = float(self.prev_base_odom.pose.pose.position.x)
        prev_base_y = float(self.prev_base_odom.pose.pose.position.y)
        prev_base_yaw = _yaw_from_quaternion(self.prev_base_odom.pose.pose.orientation)
        base_x = float(base_msg.pose.pose.position.x)
        base_y = float(base_msg.pose.pose.position.y)
        base_yaw = _yaw_from_quaternion(base_msg.pose.pose.orientation)

        delta_global_x = base_x - prev_base_x
        delta_global_y = base_y - prev_base_y
        cos_yaw = math.cos(prev_base_yaw)
        sin_yaw = math.sin(prev_base_yaw)
        base_local_dx = cos_yaw * delta_global_x + sin_yaw * delta_global_y
        base_local_dy = -sin_yaw * delta_global_x + cos_yaw * delta_global_y
        base_delta_yaw = _wrap_angle(base_yaw - prev_base_yaw)

        corrected_dx = base_local_dx
        corrected_dy = base_local_dy
        corrected_dyaw = base_delta_yaw
        xfeat_dx = 0.0
        xfeat_dy = 0.0
        xfeat_dyaw = 0.0
        delta_diff = 0.0
        yaw_diff = 0.0
        self._last_fusion_status = "base_only"
        self._last_status_details = (
            f"dx={base_local_dx:.3f} dy={base_local_dy:.3f} dyaw={math.degrees(base_delta_yaw):.1f}deg"
        )

        if self.xfeat_delta is not None and self.last_xfeat_stamp_sec is not None:
            base_stamp_sec = float(base_msg.header.stamp.sec) + float(base_msg.header.stamp.nanosec) * 1e-9
            if base_stamp_sec - self.last_xfeat_stamp_sec <= self.xfeat_timeout_sec:
                xfeat_dx = float(self.xfeat_delta.pose.pose.position.x)
                xfeat_dy = float(self.xfeat_delta.pose.pose.position.y)
                xfeat_dyaw = _yaw_from_quaternion(self.xfeat_delta.pose.pose.orientation)
                delta_diff = math.hypot(xfeat_dx - base_local_dx, xfeat_dy - base_local_dy)
                yaw_diff = abs(_wrap_angle(xfeat_dyaw - base_delta_yaw))
                if (
                    delta_diff <= self.max_delta_translation_diff_m
                    and yaw_diff <= self.max_delta_yaw_diff_rad
                ):
                    corrected_dx = base_local_dx + self.gain_xy * (xfeat_dx - base_local_dx)
                    corrected_dy = base_local_dy + self.gain_xy * (xfeat_dy - base_local_dy)
                    corrected_dyaw = base_delta_yaw + self.gain_yaw * _wrap_angle(xfeat_dyaw - base_delta_yaw)
                    self._last_fusion_status = "fused"
                    self._last_status_details = (
                        f"base(dx={base_local_dx:.3f},dy={base_local_dy:.3f},dyaw={math.degrees(base_delta_yaw):.1f}deg) "
                        f"xfeat(dx={xfeat_dx:.3f},dy={xfeat_dy:.3f},dyaw={math.degrees(xfeat_dyaw):.1f}deg)"
                    )
                else:
                    self._last_fusion_status = "rejected"
                    self._last_status_details = (
                        f"delta_diff={delta_diff:.3f}m yaw_diff={math.degrees(yaw_diff):.1f}deg "
                        f"base(dx={base_local_dx:.3f},dy={base_local_dy:.3f},dyaw={math.degrees(base_delta_yaw):.1f}deg) "
                        f"xfeat(dx={xfeat_dx:.3f},dy={xfeat_dy:.3f},dyaw={math.degrees(xfeat_dyaw):.1f}deg)"
                    )
            else:
                age = base_stamp_sec - self.last_xfeat_stamp_sec
                self._last_fusion_status = "base_only"
                self._last_status_details = f"xfeat_timeout={age:.2f}s"
        elif self.xfeat_delta is None:
            self._last_fusion_status = "base_only"
            self._last_status_details = "xfeat_missing"

        world_dx = math.cos(self.fused_yaw) * corrected_dx - math.sin(self.fused_yaw) * corrected_dy
        world_dy = math.sin(self.fused_yaw) * corrected_dx + math.cos(self.fused_yaw) * corrected_dy
        self.fused_x += world_dx
        self.fused_y += world_dy
        self.fused_yaw = _wrap_angle(self.fused_yaw + corrected_dyaw)

        self._last_csv_row = {
            "stamp_sec": float(base_msg.header.stamp.sec) + float(base_msg.header.stamp.nanosec) * 1e-9,
            "status": self._last_fusion_status,
            "base_dx": base_local_dx,
            "base_dy": base_local_dy,
            "base_dyaw_deg": math.degrees(base_delta_yaw),
            "xfeat_dx": xfeat_dx,
            "xfeat_dy": xfeat_dy,
            "xfeat_dyaw_deg": math.degrees(xfeat_dyaw),
            "delta_diff_m": delta_diff,
            "yaw_diff_deg": math.degrees(yaw_diff),
            "fused_x": self.fused_x,
            "fused_y": self.fused_y,
            "fused_yaw_deg": math.degrees(self.fused_yaw),
        }

        fused.pose.pose.position.x = self.fused_x
        fused.pose.pose.position.y = self.fused_y
        fused.pose.pose.position.z = base_msg.pose.pose.position.z
        fused.pose.pose.orientation = _quaternion_from_yaw(self.fused_yaw)
        return fused

    def _log_fused_pose(self, base: Odometry, fused: Odometry) -> None:
        if not self._should_log_pose():
            return
        now_sec = float(fused.header.stamp.sec) + float(fused.header.stamp.nanosec) * 1e-9
        if self.last_log_sec is not None and now_sec - self.last_log_sec < self.log_period_sec:
            return
        self.last_log_sec = now_sec
        self._append_csv_log()
        self.get_logger().info(f"{self._last_fusion_status} | {self._last_status_details}")

    def _init_csv_log(self) -> None:
        csv_dir = os.path.dirname(self.csv_log_path)
        if csv_dir:
            os.makedirs(csv_dir, exist_ok=True)
        with open(self.csv_log_path, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    "时间戳_秒",
                    "融合状态",
                    "底盘增量_dx_米",
                    "底盘增量_dy_米",
                    "底盘增量_dyaw_度",
                    "XFeat增量_dx_米",
                    "XFeat增量_dy_米",
                    "XFeat增量_dyaw_度",
                    "平移差值_米",
                    "偏航差值_度",
                    "融合后_x_米",
                    "融合后_y_米",
                    "融合后_yaw_度",
                ]
            )

    def _append_csv_log(self) -> None:
        if self._last_csv_row is None:
            return
        with open(self.csv_log_path, "a", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    f"{self._last_csv_row['stamp_sec']:.6f}",
                    self._last_csv_row["status"],
                    f"{self._last_csv_row['base_dx']:.6f}",
                    f"{self._last_csv_row['base_dy']:.6f}",
                    f"{self._last_csv_row['base_dyaw_deg']:.3f}",
                    f"{self._last_csv_row['xfeat_dx']:.6f}",
                    f"{self._last_csv_row['xfeat_dy']:.6f}",
                    f"{self._last_csv_row['xfeat_dyaw_deg']:.3f}",
                    f"{self._last_csv_row['delta_diff_m']:.6f}",
                    f"{self._last_csv_row['yaw_diff_deg']:.3f}",
                    f"{self._last_csv_row['fused_x']:.6f}",
                    f"{self._last_csv_row['fused_y']:.6f}",
                    f"{self._last_csv_row['fused_yaw_deg']:.3f}",
                ]
            )

    def _should_log_pose(self) -> bool:
        if self.control_mode != "nav":
            return False
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.path_active:
            return True
        if self.last_path_sec is None:
            return False
        recent_path = (now_sec - self.last_path_sec) <= self.nav_idle_timeout_sec
        recent_motion = (
            self.last_nonzero_cmd_sec is not None
            and (now_sec - self.last_nonzero_cmd_sec) <= self.nav_idle_timeout_sec
        )
        return recent_path or recent_motion


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
