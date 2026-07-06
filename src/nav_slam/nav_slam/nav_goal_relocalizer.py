#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
nav_goal_relocalizer.py -- 导航目标点到达后触发全局重定位。

监听 /goal_reached 话题 (来自 voronoi planner),
到达目标点后:
  1. 暂停 orb_map_matcher (避免新旧 TF 冲突)
  2. 调用 /trigger_relocalize 触发全局重定位
  3. 延迟后恢复 orb_map_matcher
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from std_srvs.srv import Empty as EmptySrv
from std_srvs.srv import SetBool


class NavGoalRelocalizer(Node):
    def __init__(self):
        super().__init__("nav_goal_relocalizer")

        # 防抖动：两次重定位之间的最小间隔 (秒)
        self.declare_parameter("min_relocalize_interval_sec", 10.0)
        self._min_interval = float(
            self.get_parameter("min_relocalize_interval_sec").value)

        # 重定位期间暂停 ORB 修正的时长 (秒)，应 > 定位耗时+TF发布耗时
        self.declare_parameter("orb_disable_duration_sec", 10.0)
        self._disable_duration = float(
            self.get_parameter("orb_disable_duration_sec").value)

        # 上次重定位时间
        self._last_relocalize_time = self.get_clock().now()

        # ---- /trigger_relocalize 服务 ----
        self.get_logger().info("等待 /trigger_relocalize 服务...")
        self._relocalize_cli = self.create_client(
            EmptySrv, "/trigger_relocalize")
        while not self._relocalize_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info(
                "/trigger_relocalize 服务尚未就绪, 继续等待...")
        self.get_logger().info("/trigger_relocalize 服务已就绪")

        # ---- /enable_orb_matcher 服务（可选，ORB matcher 不启动时自动跳过） ----
        self._orb_enable_cli = self.create_client(
            SetBool, "/enable_orb_matcher")
        if self._orb_enable_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("/enable_orb_matcher 服务已就绪")
            self._has_orb_matcher = True
        else:
            self.get_logger().warn(
                "/enable_orb_matcher 服务不可用, ORB 暂停/恢复功能已禁用")
            self._has_orb_matcher = False

        # ---- /goal_reached 话题 ----
        self._goal_reached_sub = self.create_subscription(
            Empty, "/goal_reached", self._goal_reached_callback, 10)

        # 恢复 ORB 的定时器
        self._reenable_timer = self.create_timer(
            self._disable_duration, self._reenable_orb_cb)
        self._reenable_timer.cancel()  # 初始不启动

        self.get_logger().info(
            f"导航目标重定位器已就绪, 最小重定位间隔: {self._min_interval}s, "
            f"ORB 暂停时长: {self._disable_duration}s")

    # ==================== 目标到达 ====================

    def _goal_reached_callback(self, msg: Empty):
        now = self.get_clock().now()
        elapsed = (now - self._last_relocalize_time).nanoseconds * 1e-9

        if elapsed < self._min_interval:
            self.get_logger().info(
                f"距离上次重定位仅 {elapsed:.1f}s, "
                f"小于最小间隔 {self._min_interval}s, 跳过")
            return

        self.get_logger().info("导航目标已到达, 触发重定位...")
        self._last_relocalize_time = now

        # 1. 暂停 ORB 持续修正
        self._set_orb_enabled(False)

        # 2. 触发重定位
        self._call_relocalize()

        # 3. 延迟后恢复 ORB
        self._reenable_timer.reset()

    # ==================== ORB 暂停/恢复 ====================

    def _set_orb_enabled(self, enabled: bool):
        if not self._has_orb_matcher:
            return
        if not self._orb_enable_cli.service_is_ready():
            self.get_logger().warn(
                "/enable_orb_matcher 服务不可用, 无法切换 ORB 状态")
            return
        req = SetBool.Request()
        req.data = enabled
        self._orb_enable_cli.call_async(req)

    def _reenable_orb_cb(self):
        self._set_orb_enabled(True)
        self._reenable_timer.cancel()

    # ==================== 重定位 ====================

    def _call_relocalize(self):
        if not self._relocalize_cli.service_is_ready():
            self.get_logger().warn(
                "/trigger_relocalize 服务不可用, 无法触发重定位")
            self._set_orb_enabled(True)  # 恢复 ORB
            return

        req = EmptySrv.Request()
        future = self._relocalize_cli.call_async(req)
        future.add_done_callback(self._relocalize_response_cb)

    def _relocalize_response_cb(self, future):
        try:
            future.result()
            self.get_logger().info(
                "全局重定位已触发, ORB 修正已暂停, "
                f"{self._disable_duration}s 后自动恢复")
        except Exception as e:
            self.get_logger().error(f"重定位请求失败: {e}, 立即恢复 ORB")
            self._set_orb_enabled(True)


def main(args=None):
    rclpy.init(args=args)
    node = NavGoalRelocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
