#!/usr/bin/env python3
"""Run one AMCL global-initialization trial and report its converged pose."""

import json
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty


def _yaw(orientation) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


class AmclInitialLocalizer(Node):
    """Trigger AMCL global localization, rotate in place, and detect convergence."""

    def __init__(self) -> None:
        super().__init__("amcl_initial_localizer")
        self.declare_parameter("pose_topic", "/amcl_pose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("service_name", "/reinitialize_global_localization")
        self.declare_parameter("result_topic", "/initial_localization/result")
        self.declare_parameter("angular_speed_rps", 0.35)
        self.declare_parameter("timeout_sec", 90.0)
        self.declare_parameter("cov_xy_threshold", 0.05)
        self.declare_parameter("cov_yaw_threshold", 0.05)
        self.declare_parameter("consecutive_covariance_samples", 8)

        self._service_name = str(self.get_parameter("service_name").value)
        self._angular_speed = float(self.get_parameter("angular_speed_rps").value)
        self._timeout_sec = float(self.get_parameter("timeout_sec").value)
        self._cov_xy = float(self.get_parameter("cov_xy_threshold").value)
        self._cov_yaw = float(self.get_parameter("cov_yaw_threshold").value)
        self._required_covariance_count = max(
            2, int(self.get_parameter("consecutive_covariance_samples").value)
        )

        self._latest: Optional[PoseWithCovarianceStamped] = None
        self._good_covariance_count = 0
        self._started_ns: Optional[int] = None
        self._requested = False
        self._finished = False
        self._client = self.create_client(Empty, self._service_name)
        self._cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self._result_pub = self.create_publisher(
            String, str(self.get_parameter("result_topic").value), 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter("pose_topic").value),
            self._pose_callback,
            20,
        )
        self.create_timer(0.1, self._tick)
        self.get_logger().info(f"等待 AMCL 全局定位服务 {self._service_name}")

    def _tick(self) -> None:
        if self._finished:
            return
        if not self._requested:
            if not self._client.service_is_ready():
                self._client.wait_for_service(timeout_sec=0.0)
                return
            future = self._client.call_async(Empty.Request())
            future.add_done_callback(self._service_response)
            self._requested = True
            self._started_ns = self.get_clock().now().nanoseconds
            self.get_logger().info("AMCL 全局粒子已请求，开始原地旋转采集激光")
            return

        self._publish_rotation()
        if self._elapsed_sec() >= self._timeout_sec:
            self._finish("failed", reason="timeout")

    def _service_response(self, future) -> None:
        try:
            future.result()
        except Exception as exc:
            self._finish("failed", reason=f"service_error:{exc}")

    def _pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        if not self._requested or self._finished:
            return
        self._latest = msg
        covariance = msg.pose.covariance
        if (
            covariance[0] > self._cov_xy
            or covariance[7] > self._cov_xy
            or covariance[35] > self._cov_yaw
        ):
            self._good_covariance_count = 0
            return
        self._good_covariance_count += 1
        if self._good_covariance_count >= self._required_covariance_count:
            self._finish("converged")

    def _publish_rotation(self) -> None:
        command = Twist()
        command.angular.z = self._angular_speed
        self._cmd_pub.publish(command)

    def _stop(self) -> None:
        if rclpy.ok():
            self._cmd_pub.publish(Twist())

    def _elapsed_sec(self) -> float:
        if self._started_ns is None:
            return 0.0
        return (self.get_clock().now().nanoseconds - self._started_ns) * 1e-9

    def _finish(self, status: str, reason: str = "") -> None:
        if self._finished:
            return
        self._finished = True
        self._stop()
        result = {
            "method": "amcl",
            "status": status,
            "reason": reason,
            "elapsed_ms": self._elapsed_sec() * 1000.0,
        }
        if self._latest is not None:
            pose = self._latest.pose.pose
            covariance = self._latest.pose.covariance
            result.update(
                x=pose.position.x,
                y=pose.position.y,
                yaw_deg=math.degrees(_yaw(pose.orientation)),
                cov_x=covariance[0],
                cov_y=covariance[7],
                cov_yaw=covariance[35],
            )
        message = String()
        message.data = json.dumps(result, ensure_ascii=False)
        self._result_pub.publish(message)
        print("[INITIAL_LOCALIZATION_RESULT] " + message.data, flush=True)

    def destroy_node(self) -> bool:
        self._stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AmclInitialLocalizer()
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
