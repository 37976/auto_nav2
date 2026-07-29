#!/usr/bin/env python3
"""
Quality-gated asynchronous map-to-odom correction.

ORB observations describe ``base_frame`` in ``map_frame`` at the LaserScan
timestamp.  This node pairs that observation with the saved local odometry at
the same timestamp, derives one map->odom transform, and then moves the
published transform towards it at bounded linear and angular rates.  Local
odometry is never modified.
"""

import json
import math
from collections import deque
from typing import Deque, Optional

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def _yaw(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _stamp(msg: Odometry) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


class MapOdomCorrector(Node):
    def __init__(self) -> None:
        super().__init__("map_odom_corrector")
        self.declare_parameter("odom_topic", "/localized_odom")
        self.declare_parameter("match_pose_topic", "/orb/match_pose")
        self.declare_parameter("initial_pose_topic", "/lidar_global/match_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        # Global ORB can take more than 10 seconds; retain the scan-time odom
        # long enough to apply its delayed observation.
        self.declare_parameter("state_history_sec", 30.0)
        self.declare_parameter("max_observation_sync_sec", 0.15)
        self.declare_parameter("required_consistent_matches", 2)
        self.declare_parameter("consistent_translation_m", 0.30)
        self.declare_parameter("consistent_yaw_deg", 5.0)
        self.declare_parameter("max_correction_linear_mps", 0.20)
        self.declare_parameter("max_correction_angular_degps", 12.0)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter(
            "global_correction_applied_topic",
            "/localization/global_correction_applied")
        self.declare_parameter(
            "correction_event_topic",
            "/localization/map_odom_correction_event")

        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._match_topic = str(self.get_parameter("match_pose_topic").value)
        self._initial_topic = str(self.get_parameter("initial_pose_topic").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._history_sec = float(self.get_parameter("state_history_sec").value)
        self._max_sync_sec = float(self.get_parameter("max_observation_sync_sec").value)
        self._required_matches = max(
            1, int(self.get_parameter("required_consistent_matches").value))
        self._consistent_translation = float(self.get_parameter("consistent_translation_m").value)
        self._consistent_yaw = math.radians(float(self.get_parameter("consistent_yaw_deg").value))
        self._max_linear = float(self.get_parameter("max_correction_linear_mps").value)
        self._max_angular = math.radians(float(
            self.get_parameter("max_correction_angular_degps").value))
        self._publish_hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self._global_applied_topic = str(self.get_parameter(
            "global_correction_applied_topic").value)
        self._correction_event_topic = str(self.get_parameter(
            "correction_event_topic").value)

        self._history: Deque[Odometry] = deque()
        self._pending_target: Optional[tuple[float, float, float]] = None
        self._pending_count = 0
        self._tracking_confirmed = False
        self._current = (0.0, 0.0, 0.0)
        self._target = self._current
        self._last_publish_sec: Optional[float] = None

        self.create_subscription(Odometry, self._odom_topic, self._odom_cb, 50)
        self.create_subscription(Odometry, self._match_topic, self._match_cb, 10)
        self.create_subscription(Odometry, self._initial_topic, self._initial_cb, 10)
        self._tf_pub = TransformBroadcaster(self)
        event_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._global_applied_pub = self.create_publisher(
            String, self._global_applied_topic, 10)
        self._correction_event_pub = self.create_publisher(
            String, self._correction_event_topic, event_qos)
        self.create_timer(1.0 / self._publish_hz, self._publish_tf)
        self.get_logger().info(
            f"map->odom corrector: odom={self._odom_topic}, matches={self._match_topic}, "
            f"consistent_matches={self._required_matches}")

    def _odom_cb(self, msg: Odometry) -> None:
        self._history.append(msg)
        cutoff = _stamp(msg) - self._history_sec
        while self._history and _stamp(self._history[0]) < cutoff:
            self._history.popleft()

    def _odom_at(self, stamp_sec: float) -> Optional[Odometry]:
        if not self._history:
            return None
        candidate = min(self._history, key=lambda odom: abs(_stamp(odom) - stamp_sec))
        return candidate if abs(_stamp(candidate) - stamp_sec) <= self._max_sync_sec else None

    @staticmethod
    def _target_from_observation(
        observation: Odometry, odom: Odometry
    ) -> tuple[float, float, float]:
        map_yaw = _yaw(observation.pose.pose.orientation)
        odom_yaw = _yaw(odom.pose.pose.orientation)
        tf_yaw = _wrap(map_yaw - odom_yaw)
        c = math.cos(tf_yaw)
        s = math.sin(tf_yaw)
        odom_x = odom.pose.pose.position.x
        odom_y = odom.pose.pose.position.y
        return (
            observation.pose.pose.position.x - (c * odom_x - s * odom_y),
            observation.pose.pose.position.y - (s * odom_x + c * odom_y),
            tf_yaw,
        )

    def _initial_cb(self, msg: Odometry) -> None:
        target = self._observation_target(msg)
        if target is None:
            return
        self._current = target
        self._target = target
        self._pending_target = None
        self._pending_count = 0
        self._tracking_confirmed = False
        self._publish_global_applied()
        self._publish_correction_event("global_applied", "global", target, msg)
        self.get_logger().info(
            "Applied initial global localization map->odom transform "
            f"at scan stamp {_stamp(msg):.3f}s, tf_yaw={math.degrees(target[2]):.1f}deg")

    def _match_cb(self, msg: Odometry) -> None:
        target = self._observation_target(msg)
        if target is None:
            return
        if self._required_matches == 1:
            self._target = target
            self._pending_target = None
            self._pending_count = 0
            self._tracking_confirmed = True
            self._publish_correction_event("accepted", "orb", target, msg)
            self.get_logger().info("Accepted single-observation ORB tracking update")
            return
        if self._tracking_confirmed and self._targets_consistent(target, self._target):
            self._target = target
            self._publish_correction_event("accepted", "orb", target, msg)
            self.get_logger().info(
                "Accepted consistent ORB tracking update: "
                f"yaw_innovation={math.degrees(_wrap(target[2] - self._current[2])):.1f}deg")
            return

        # The first update, and every abrupt target change, needs a second
        # independent observation.  Once tracking is stable, small corrections
        # do not wait for another slow ORB solve.
        self._tracking_confirmed = False
        if (self._pending_target is None
                or not self._targets_consistent(target, self._pending_target)):
            self._pending_target = target
            self._pending_count = 1
            self._publish_correction_event("held", "orb", target, msg)
            self.get_logger().info("ORB observation held for consistency confirmation")
            return
        self._pending_count += 1
        self._pending_target = target
        if self._pending_count < self._required_matches:
            return
        self._target = target
        self._pending_count = 0
        self._pending_target = None
        self._tracking_confirmed = True
        self._publish_correction_event("accepted", "orb", target, msg)
        self.get_logger().info(
            f"Accepted ORB map->odom target: x={target[0]:.3f} y={target[1]:.3f} "
            f"yaw={math.degrees(target[2]):.1f}deg "
            f"innovation={math.degrees(_wrap(target[2] - self._current[2])):.1f}deg")

    def _publish_global_applied(self) -> None:
        message = String()
        message.data = "applied"
        self._global_applied_pub.publish(message)

    def _publish_correction_event(
        self, status: str, source: str, target: tuple[float, float, float], msg: Odometry
    ) -> None:
        event = String()
        event.data = json.dumps({
            "stamp_sec": _stamp(msg),
            "status": status,
            "source": source,
            "target_x_m": target[0],
            "target_y_m": target[1],
            "target_yaw_deg": math.degrees(target[2]),
            "yaw_innovation_deg": math.degrees(_wrap(target[2] - self._current[2])),
        })
        self._correction_event_pub.publish(event)

    def _observation_target(self, msg: Odometry) -> Optional[tuple[float, float, float]]:
        if msg.header.frame_id != self._map_frame:
            self.get_logger().warn("Ignoring observation outside map frame")
            return None
        odom = self._odom_at(_stamp(msg))
        if odom is None:
            self.get_logger().warn(
                f"Ignoring unsynchronized map observation at {_stamp(msg):.3f}s; "
                f"history window is {self._history_sec:.1f}s")
            return None
        return self._target_from_observation(msg, odom)

    def _targets_consistent(
        self, first: tuple[float, float, float], second: tuple[float, float, float]
    ) -> bool:
        translation_ok = math.hypot(
            first[0] - second[0], first[1] - second[1]
        ) <= self._consistent_translation
        return translation_ok and (
            abs(_wrap(first[2] - second[2])) <= self._consistent_yaw)

    def _publish_tf(self) -> None:
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9
        if self._last_publish_sec is not None:
            dt = max(0.0, now_sec - self._last_publish_sec)
            dx = self._target[0] - self._current[0]
            dy = self._target[1] - self._current[1]
            distance = math.hypot(dx, dy)
            step = self._max_linear * dt
            if distance > step > 0.0:
                ratio = step / distance
                x = self._current[0] + dx * ratio
                y = self._current[1] + dy * ratio
            else:
                x, y = self._target[0], self._target[1]
            yaw_error = _wrap(self._target[2] - self._current[2])
            yaw_step = self._max_angular * dt
            yaw = self._current[2] + max(-yaw_step, min(yaw_step, yaw_error))
            self._current = (x, y, _wrap(yaw))
        self._last_publish_sec = now_sec

        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = self._map_frame
        transform.child_frame_id = self._odom_frame
        transform.transform.translation.x = self._current[0]
        transform.transform.translation.y = self._current[1]
        transform.transform.rotation.z = math.sin(self._current[2] * 0.5)
        transform.transform.rotation.w = math.cos(self._current[2] * 0.5)
        self._tf_pub.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapOdomCorrector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
