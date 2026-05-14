#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
amcl_global_localize.py -- 等待 AMCL 就绪后触发全局定位。

工作流程:
1. 轮询等待 AMCL 的 /reinitialize_global_localization 服务就绪
2. 服务就绪后调用 Empty 请求，将所有粒子均匀分布到地图自由空间
3. AMCL 随后通过激光扫描自主收敛到正确位姿
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


class AmclGlobalLocalize(Node):
    def __init__(self):
        super().__init__("amcl_global_localize")

        self.declare_parameter("service_name", "/reinitialize_global_localization")
        self.declare_parameter("retry_period_sec", 1.0)
        self.declare_parameter("max_retries", 60)

        self._service_name = str(self.get_parameter("service_name").value)
        self._retry_period_sec = float(self.get_parameter("retry_period_sec").value)
        self._max_retries = int(self.get_parameter("max_retries").value)

        self._retry_count = 0
        self._done = False

        self._timer = self.create_timer(self._retry_period_sec, self._try_global_localize)
        self.get_logger().info(
            f"等待 AMCL 服务 {self._service_name} 就绪 (最多重试 {self._max_retries} 次)..."
        )

    def _try_global_localize(self):
        if self._done:
            return

        self._retry_count += 1
        if self._retry_count > self._max_retries:
            self.get_logger().error(
                f"超过最大重试次数 {self._max_retries}，放弃全局定位初始化。"
                f"请确认 AMCL 已启动且 lifecycle_manager 已将其激活。"
            )
            self._timer.cancel()
            return

        client = self.create_client(Empty, self._service_name)

        if not client.wait_for_service(timeout_sec=0.5):
            self.get_logger().debug(
                f"服务 {self._service_name} 尚未就绪 ({self._retry_count}/{self._max_retries})"
            )
            return

        self.get_logger().info(f"服务 {self._service_name} 已就绪，触发全局定位...")
        req = Empty.Request()
        future = client.call_async(req)
        future.add_done_callback(self._handle_response)
        self._timer.cancel()

    def _handle_response(self, future):
        self._done = True
        try:
            future.result()
            self.get_logger().info(
                "全局定位已触发。AMCL 粒子已均匀分布到地图自由空间，"
                "机器人将通过激光扫描自主收敛定位。"
            )
        except Exception as e:
            self.get_logger().error(f"全局定位服务调用失败: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = AmclGlobalLocalize()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
