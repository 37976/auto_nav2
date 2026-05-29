#!/usr/bin/env python3
"""pose_logger.py -- 记录 Gazebo 真实位姿 vs 导航计算位姿, 导出 CSV 对比表."""

import csv
import math
import os
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class PoseLogger(Node):
    def __init__(self):
        super().__init__("pose_logger")

        self.declare_parameter("output_path",
                               os.path.join(os.path.expanduser("~"),
                                            "project", "位姿对比",
                                            "pose_comparison.csv"))
        self.declare_parameter("log_hz", 5.0)
        output = str(self.get_parameter("output_path").value)
        log_hz = float(self.get_parameter("log_hz").value)

        os.makedirs(os.path.dirname(output), exist_ok=True)
        self._csv = open(output, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._csv)
        self._writer.writerow([
            "sim_time_s",
            "gt_x_m", "gt_y_m", "gt_yaw_deg",
            "calc_x_m", "calc_y_m", "calc_yaw_deg",
            "error_xy_m", "error_yaw_deg",
        ])

        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        self._odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_cb, 10)
        self._calc_sub = self.create_subscription(
            Odometry, "/odom_in_map", self._calc_cb, 10)

        self._latest_odom = None
        self._latest_calc = None

        self._timer = self.create_timer(1.0 / log_hz, self._log)
        self.get_logger().info(f"位姿对比记录已启动 → {output}")

    def _odom_cb(self, msg: Odometry):
        self._latest_odom = msg

    def _calc_cb(self, msg: Odometry):
        self._latest_calc = msg

    def _yaw(self, q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _log(self):
        if self._latest_odom is None or self._latest_calc is None:
            return

        # 查 map→odom TF, 把 Gazebo 地面真值从 odom 系转到 map 系
        try:
            tf = self._tf_buf.lookup_transform(
                "map", "odom", rclpy.time.Time())
        except TransformException:
            return

        t = tf.transform.translation
        yaw_tf = self._yaw(tf.transform.rotation)

        odom = self._latest_odom
        odom_x = odom.pose.pose.position.x
        odom_y = odom.pose.pose.position.y
        odom_yaw = self._yaw(odom.pose.pose.orientation)

        gt_x = t.x + (odom_x * math.cos(yaw_tf) - odom_y * math.sin(yaw_tf))
        gt_y = t.y + (odom_x * math.sin(yaw_tf) + odom_y * math.cos(yaw_tf))
        gt_yaw = math.degrees(
            math.atan2(math.sin(yaw_tf + odom_yaw),
                       math.cos(yaw_tf + odom_yaw)))

        calc = self._latest_calc
        calc_x = calc.pose.pose.position.x
        calc_y = calc.pose.pose.position.y
        calc_yaw = math.degrees(self._yaw(calc.pose.pose.orientation))

        err_xy = math.hypot(gt_x - calc_x, gt_y - calc_y)
        err_yaw = abs(gt_yaw - calc_yaw)
        if err_yaw > 180.0:
            err_yaw = 360.0 - err_yaw

        sim_t = odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9

        self._writer.writerow([
            f"{sim_t:.3f}",
            f"{gt_x:.4f}", f"{gt_y:.4f}", f"{gt_yaw:.2f}",
            f"{calc_x:.4f}", f"{calc_y:.4f}", f"{calc_yaw:.2f}",
            f"{err_xy:.4f}", f"{err_yaw:.2f}",
        ])
        self._csv.flush()

    def destroy_node(self):
        self._csv.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
