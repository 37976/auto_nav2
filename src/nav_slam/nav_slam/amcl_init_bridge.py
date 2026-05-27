#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
amcl_init_bridge.py — AMCL 初始定位收敛后，锁定并发布静态 map→odom TF。

工作流程:
1. 启动时等待 AMCL 收敛（/amcl_pose 协方差足够小）
2. 同时读取 /localized_odom（odom 框架内的机器人位姿）
3. 计算 map→odom = map_pose * inv(odom_pose)
4. 发布为静态 TF，之后 AMCL 不再需要
"""

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


def _yaw_from_quat(x, y, z, w):
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def _normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class AmclInitBridge(Node):
    def __init__(self):
        super().__init__("amcl_init_bridge")

        self.declare_parameter("cov_xy_threshold", 0.002)
        self.declare_parameter("cov_yaw_threshold", 0.001)
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("localized_odom_topic", "/localized_odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")

        self._cov_xy_threshold = float(self.get_parameter("cov_xy_threshold").value)
        self._cov_yaw_threshold = float(self.get_parameter("cov_yaw_threshold").value)
        self._amcl_pose_topic = str(self.get_parameter("amcl_pose_topic").value)
        self._localized_odom_topic = str(self.get_parameter("localized_odom_topic").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)

        self._latest_amcl: Optional[PoseWithCovarianceStamped] = None
        self._latest_odom: Optional[Odometry] = None
        self._locked = False

        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self._amcl_pose_topic,
            self._amcl_cb,
            10,
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            self._localized_odom_topic,
            self._odom_cb,
            10,
        )
        self._tf_broadcaster = StaticTransformBroadcaster(self)

        # 启动时先发 identity TF，保证 RViz 机器人模型可见
        self._publish_identity_tf()
        self._identity_timer = self.create_timer(2.0, self._publish_identity_tf)

        # 定期打印收敛状态，方便诊断
        self._status_timer = self.create_timer(3.0, self._log_status)

        self.get_logger().info(
            f"等待 AMCL 收敛 (cov_xy<{self._cov_xy_threshold}, cov_yaw<{self._cov_yaw_threshold})..."
        )

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self._latest_amcl = msg
        self._try_lock()

    def _odom_cb(self, msg: Odometry):
        self._latest_odom = msg
        self._try_lock()

    def _log_status(self):
        if self._locked:
            return
        if self._latest_amcl is None:
            self.get_logger().info(
                f"等待 /amcl_pose ...", throttle_duration_sec=5.0
            )
        elif self._latest_odom is None:
            self.get_logger().info(
                f"已收到 /amcl_pose，等待 /localized_odom ...", throttle_duration_sec=5.0
            )
        else:
            cov = self._latest_amcl.pose.covariance
            self.get_logger().info(
                f"AMCL cov: x={cov[0]:.6f} (需<{self._cov_xy_threshold}) "
                f"y={cov[7]:.6f} (需<{self._cov_xy_threshold}) "
                f"yaw={cov[35]:.6f} (需<{self._cov_yaw_threshold})"
            )

    def _try_lock(self):
        if self._locked:
            return
        if self._latest_amcl is None or self._latest_odom is None:
            return

        # 协方差矩阵: 6x6 行优先，索引 0=x, 7=y, 35=yaw(6D), 但在 nav2 中 AMCL 只填 xy+θ
        cov = self._latest_amcl.pose.covariance
        cov_x = cov[0]
        cov_y = cov[7]
        cov_yaw = cov[35]

        if cov_x >= self._cov_xy_threshold or cov_y >= self._cov_xy_threshold:
            return
        if cov_yaw >= self._cov_yaw_threshold:
            return

        self._locked = True
        # 停止发送 identity TF 和状态日志
        self._identity_timer.cancel()
        self._status_timer.cancel()
        self._publish_static_tf()

    def _publish_identity_tf(self):
        """AMCL 未收敛时发 identity map→odom，保证 RViz 显示机器人模型"""
        if self._locked:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._map_frame
        t.child_frame_id = self._odom_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(t)

    def _publish_static_tf(self):
        amcl = self._latest_amcl
        odom = self._latest_odom

        # 机器人在地图坐标系下的位姿
        x_m = amcl.pose.pose.position.x
        y_m = amcl.pose.pose.position.y
        yaw_m = _yaw_from_quat(
            amcl.pose.pose.orientation.x,
            amcl.pose.pose.orientation.y,
            amcl.pose.pose.orientation.z,
            amcl.pose.pose.orientation.w,
        )

        # 机器人在 odom 坐标系下的位姿
        x_o = odom.pose.pose.position.x
        y_o = odom.pose.pose.position.y
        yaw_o = _yaw_from_quat(
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        )

        # T_map_odom = T_map_base * inv(T_odom_base)
        cos_d = math.cos(yaw_m - yaw_o)
        sin_d = math.sin(yaw_m - yaw_o)
        t_x = x_m - (cos_d * x_o - sin_d * y_o)
        t_y = y_m - (sin_d * x_o + cos_d * y_o)
        t_yaw = _normalize_angle(yaw_m - yaw_o)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._map_frame
        t.child_frame_id = self._odom_frame
        t.transform.translation.x = t_x
        t.transform.translation.y = t_y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(t_yaw * 0.5)
        t.transform.rotation.w = math.cos(t_yaw * 0.5)

        self._tf_broadcaster.sendTransform(t)

        self.get_logger().info(
            f"✅ AMCL 已收敛！map→odom 静态 TF 已锁定: "
            f"x={t_x:.3f} y={t_y:.3f} yaw={math.degrees(t_yaw):.1f}°"
        )
        self.get_logger().info("AMCL 使命完成，后续导航由 XFeat 融合里程计接管。")
        self.get_logger().info(
            f"AMCL cov: x={self._latest_amcl.pose.covariance[0]:.6f} "
            f"y={self._latest_amcl.pose.covariance[7]:.6f} "
            f"yaw={self._latest_amcl.pose.covariance[35]:.6f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = AmclInitBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
