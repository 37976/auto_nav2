#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lidar_global_localize.py -- 静止机器人 2D 激光全局定位。

评分：对每个候选位姿，比较雷达实际距离与地图期望距离的差距。
差距越小，位姿越正确。

1. 预计算 8 方向距离表（DP 横扫，O(w×h×8)）
2. 粗搜索：查表比对期望距离 vs 实际距离，快速过滤
3. 精搜索：对 top-K 真实射线投射，精确评分
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan


def _quat_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


# 8 个方向的 (dx, dy) 和步长（格数）
_DIRS = [
    (1, 0, 1.0),     # 0°
    (1, 1, 1.414),   # 45°
    (0, 1, 1.0),     # 90°
    (-1, 1, 1.414),  # 135°
    (-1, 0, 1.0),    # 180°
    (-1, -1, 1.414), # 225°
    (0, -1, 1.0),    # 270°
    (1, -1, 1.414),  # 315°
]
_N_DIRS = len(_DIRS)
_DIR_ANGLE = 2.0 * math.pi / _N_DIRS  # 45° per bin


class LidarGlobalLocalize(Node):
    def __init__(self):
        super().__init__("lidar_global_localize")

        self.declare_parameter("map_topic", "/map_for_amcl")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("coarse_step_xy", 1.0)
        self.declare_parameter("coarse_step_yaw_deg", 15.0)
        self.declare_parameter("fine_step_xy", 0.1)
        self.declare_parameter("fine_step_yaw_deg", 3.0)
        self.declare_parameter("fine_radius_xy", 1.0)
        self.declare_parameter("fine_radius_yaw_deg", 20.0)
        self.declare_parameter("top_k", 1)
        self.declare_parameter("laser_max_range", 30.0)
        self.declare_parameter("laser_min_range", 0.5)
        self.declare_parameter("range_match_max_diff", 3.0)

        self._map_topic = str(self.get_parameter("map_topic").value)
        self._scan_topic = str(self.get_parameter("scan_topic").value)
        self._coarse_step_xy = float(self.get_parameter("coarse_step_xy").value)
        self._coarse_step_yaw = math.radians(float(self.get_parameter("coarse_step_yaw_deg").value))
        self._fine_step_xy = float(self.get_parameter("fine_step_xy").value)
        self._fine_step_yaw = math.radians(float(self.get_parameter("fine_step_yaw_deg").value))
        self._fine_radius_xy = float(self.get_parameter("fine_radius_xy").value)
        self._fine_radius_yaw = math.radians(float(self.get_parameter("fine_radius_yaw_deg").value))
        self._top_k = int(self.get_parameter("top_k").value)
        self._laser_max_range = float(self.get_parameter("laser_max_range").value)
        self._laser_min_range = float(self.get_parameter("laser_min_range").value)
        self._range_max_diff = float(self.get_parameter("range_match_max_diff").value)

        # 地图数据
        self._range_tables = None   # [8][h][w] 方向距离表（单位：格）
        self._occupied_mask = None
        self._free_mask = None
        self._map_origin_x = 0.0
        self._map_origin_y = 0.0
        self._map_resolution = 1.0
        self._map_height = 0
        self._map_width = 0
        self._scan_msg = None
        self._localized = False

        self._map_sub = self.create_subscription(OccupancyGrid, self._map_topic, self._on_map, 10)
        self._scan_sub = self.create_subscription(LaserScan, self._scan_topic, self._on_scan, 10)
        self._pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.get_logger().info(f"等待 {self._map_topic} 和 {self._scan_topic}...")

    # ============== 地图加载 & 预计算 ==============

    def _on_map(self, msg: OccupancyGrid):
        if self._occupied_mask is not None:
            return
        self._map_resolution = msg.info.resolution
        self._map_origin_x = msg.info.origin.position.x
        self._map_origin_y = msg.info.origin.position.y
        self._map_width = msg.info.width
        self._map_height = msg.info.height

        grid = np.array(msg.data, dtype=np.int8).reshape(self._map_height, self._map_width)
        self._occupied_mask = (grid >= 65)
        self._free_mask = (grid >= 0) & (grid <= 45)

        self.get_logger().info(
            f"地图: {self._map_width}×{self._map_height} @ {self._map_resolution:.3f}m, "
            f"free={np.sum(self._free_mask)} occupied={np.sum(self._occupied_mask)}"
        )

        self._precompute_range_tables()
        if self._scan_msg is not None and not self._localized:
            self._run_localization()

    def _precompute_range_tables(self):
        """对 8 方向计算距离表：range_tables[d][y][x] = 从 (x,y) 沿方向 d 到最近墙的格数。"""
        self.get_logger().info("预计算 8 方向距离表...")
        h, w = self._map_height, self._map_width
        occ = self._occupied_mask
        self._range_tables = np.zeros((_N_DIRS, h, w), dtype=np.float32)

        for d, (dx, dy, step) in enumerate(_DIRS):
            tbl = self._range_tables[d]
            # 确定遍历顺序（从"远端"向"近端"扫描）
            if dx >= 0:
                xs = range(w - 1, -1, -1)
            else:
                xs = range(0, w)
            if dy >= 0:
                ys = range(h - 1, -1, -1)
            else:
                ys = range(0, h)

            for y in ys:
                for x in xs:
                    if occ[y, x]:
                        tbl[y, x] = 0.0
                    else:
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < w and 0 <= ny < h:
                            tbl[y, x] = tbl[ny, nx] + step
                        else:
                            tbl[y, x] = 999.0  # 超出地图

        max_val = max(np.max(tbl[tbl < 999.0]) for t in self._range_tables if np.any(t < 999.0))
        self.get_logger().info(f"方向距离表完成，最大距离: {max_val:.0f} 格")

    def _on_scan(self, msg: LaserScan):
        if self._localized:
            return
        self._scan_msg = msg
        if self._range_tables is not None:
            self._run_localization()

    # ============== 搜索主流程 ==============

    def _run_localization(self):
        self._localized = True
        self.get_logger().info("开始全局定位...")

        scan = self._scan_msg
        ranges = np.array(scan.ranges, dtype=np.float32)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))
        valid = (ranges > self._laser_min_range) & (ranges < min(scan.range_max, self._laser_max_range))
        ranges, angles = ranges[valid], angles[valid]
        if len(ranges) > 180:
            step = len(ranges) // 180
            ranges, angles = ranges[::step], angles[::step]
        self.get_logger().info(f"有效激光点: {len(ranges)}")

        # ---- 粗搜索 ----
        self.get_logger().info("粗搜索...")
        cx, cy, cyaw = self._build_search_grid(self._coarse_step_xy, self._coarse_step_yaw)
        self.get_logger().info(f"候选数: {len(cx)}")
        scores = self._score_coarse(cx, cy, cyaw, ranges, angles)
        top_idx = np.argsort(scores)[:self._top_k]
        self.get_logger().info(
            f"top{self._top_k}: {[f'{scores[i]:.1f}' for i in top_idx]}"
        )

        # ---- 精搜索（真实射线投射）----
        best = (None, None, None, float("inf"))
        for idx in top_idx:
            bx, by, byaw = cx[idx], cy[idx], cyaw[idx]
            fxs = np.arange(bx - self._fine_radius_xy, bx + self._fine_radius_xy + 1e-9, self._fine_step_xy)
            fys = np.arange(by - self._fine_radius_xy, by + self._fine_radius_xy + 1e-9, self._fine_step_xy)
            fys_ = np.arange(byaw - self._fine_radius_yaw, byaw + self._fine_radius_yaw + 1e-9, self._fine_step_yaw)
            fxx, fyy, fyyaw = np.meshgrid(fxs, fys, fys_, indexing="ij")
            fxx, fyy, fyyaw = fxx.ravel(), fyy.ravel(), fyyaw.ravel()
            fscores = self._score_fine(fxx, fyy, fyyaw, ranges, angles)
            mi = int(np.argmin(fscores))
            if fscores[mi] < best[3]:
                best = (float(fxx[mi]), float(fyy[mi]), float(fyyaw[mi]), float(fscores[mi]))

        bx, by, byaw, bscore = best
        self.get_logger().info(
            f"定位完成: x={bx:.3f} y={by:.3f} yaw={math.degrees(byaw):.1f}° score={bscore:.1f}"
        )
        self._publish_initial_pose(bx, by, byaw)

    # ============== 粗搜索：查方向距离表比对 ==============

    def _build_search_grid(self, step_xy, step_yaw):
        x_min = self._map_origin_x
        x_max = self._map_origin_x + self._map_width * self._map_resolution
        y_min = self._map_origin_y
        y_max = self._map_origin_y + self._map_height * self._map_resolution
        xs, ys = np.arange(x_min, x_max, step_xy), np.arange(y_min, y_max, step_xy)
        yaws = np.arange(-math.pi, math.pi, step_yaw)
        xx, yy, yyaw = np.meshgrid(xs, ys, yaws, indexing="ij")
        xx, yy, yyaw = xx.ravel(), yy.ravel(), yyaw.ravel()
        cx = ((xx - self._map_origin_x) / self._map_resolution).astype(int)
        cy = ((yy - self._map_origin_y) / self._map_resolution).astype(int)
        ok = (cx >= 0) & (cx < self._map_width) & (cy >= 0) & (cy < self._map_height)
        keep = np.zeros(len(xx), dtype=bool)
        keep[ok] = self._free_mask[cy[ok], cx[ok]]
        return xx[keep], yy[keep], yyaw[keep]

    def _score_coarse(self, xs, ys, yaws, ranges, angles):
        """查方向距离表，比对期望距离 vs 实际距离。"""
        n = len(xs)
        if n == 0:
            return np.array([], dtype=np.float32)

        batch = 2000
        all_scores = np.empty(n, dtype=np.float32)
        for s in range(0, n, batch):
            e = min(s + batch, n)
            all_scores[s:e] = self._score_coarse_chunk(
                xs[s:e], ys[s:e], yaws[s:e], ranges, angles
            )
        return all_scores

    def _score_coarse_chunk(self, xs, ys, yaws, ranges, angles):
        """全向量化 + 双向线性插值查表。一次广播完成所有候选×所有射线的评分。"""
        n = len(xs)

        ox = self._map_origin_x
        oy = self._map_origin_y
        res = self._map_resolution
        w, h = self._map_width, self._map_height
        tbls = self._range_tables

        # 每个候选在 8 个方向的期望距离（格）
        cx_r = ((xs - ox) / res).astype(int)
        cy_r = ((ys - oy) / res).astype(int)
        ok_r = (cx_r >= 0) & (cx_r < w) & (cy_r >= 0) & (cy_r < h)
        cell_ranges = np.full((n, _N_DIRS), 999.0, dtype=np.float32)
        for d in range(_N_DIRS):
            cell_ranges[ok_r, d] = tbls[d][cy_r[ok_r], cx_r[ok_r]]

        # 所有射线方向 → 分方向索引 + 插值系数
        yaws_b = yaws[:, None]
        a_b = np.asarray(angles, dtype=np.float32)[None, :]
        world_angles = yaws_b + a_b                                   # (N, M)
        dir_frac = np.fmod(world_angles, 2 * math.pi) / _DIR_ANGLE    # (N, M), 方向浮点索引
        dir_frac = np.where(dir_frac < 0, dir_frac + _N_DIRS, dir_frac)
        dir_lo = np.floor(dir_frac).astype(int) % _N_DIRS             # (N, M)
        dir_hi = (dir_lo + 1) % _N_DIRS
        t = dir_frac - np.floor(dir_frac)                             # (N, M), 插值权重

        # 向量化查表：取两个相邻方向的期望距离
        idx = np.arange(n)[:, None]
        r_lo = cell_ranges[idx, dir_lo]   # (N, M)
        r_hi = cell_ranges[idx, dir_hi]   # (N, M)
        r_exp_cells = (1 - t) * r_lo + t * r_hi                       # 线性插值

        # 比对实际距离
        r_b = np.asarray(ranges, dtype=np.float32)[None, :]
        diff = np.abs(r_b - r_exp_cells * res)
        diff = np.minimum(diff, self._range_max_diff)

        return diff.mean(axis=1)

    # ============== 精搜索：真实射线投射 ==============

    def _score_fine(self, xs, ys, yaws, ranges, angles):
        """对少量候选做真实射线投射，精确评分。"""
        n = len(xs)
        if n == 0:
            return np.array([], dtype=np.float32)

        batch = 1000
        all_scores = np.empty(n, dtype=np.float32)
        for s in range(0, n, batch):
            e = min(s + batch, n)
            all_scores[s:e] = self._score_fine_chunk(
                xs[s:e], ys[s:e], yaws[s:e], ranges, angles
            )
        return all_scores

    def _score_fine_chunk(self, xs, ys, yaws, ranges, angles):
        """逐候选做 DDA 射线投射。"""
        n = len(xs)
        scores = np.zeros(n, dtype=np.float32)

        for j in range(n):
            x, y, yaw = xs[j], ys[j], yaws[j]
            total_diff = 0.0
            count = 0
            for r, a in zip(ranges, angles):
                angle = yaw + a
                dx, dy = math.cos(angle), math.sin(angle)
                exp_r = self._raycast(x, y, dx, dy)
                if exp_r is not None:
                    diff = abs(r - exp_r)
                    total_diff += min(diff, self._range_max_diff)
                    count += 1
            scores[j] = total_diff / max(count, 1)
        return scores

    def _raycast(self, x0, y0, dx, dy):
        """DDA 射线投射，返回到最近 occupied 格的距离（米），未命中返回 None。"""
        res = self._map_resolution
        ox, oy = self._map_origin_x, self._map_origin_y
        occ = self._occupied_mask
        w, h = self._map_width, self._map_height
        max_dist = self._laser_max_range

        # 当前格
        cx = int((x0 - ox) / res)
        cy = int((y0 - oy) / res)
        if cx < 0 or cx >= w or cy < 0 or cy >= h:
            return None

        # DDA 步进方向和步长
        step_x = 1 if dx >= 0 else -1
        step_y = 1 if dy >= 0 else -1
        t_delta_x = abs(res / dx) if abs(dx) > 1e-9 else float("inf")
        t_delta_y = abs(res / dy) if abs(dy) > 1e-9 else float("inf")

        # 到下一个格边界的距离
        if dx >= 0:
            t_max_x = ((cx + 1) * res + ox - x0) / dx
        else:
            t_max_x = (cx * res + ox - x0) / dx
        if dy >= 0:
            t_max_y = ((cy + 1) * res + oy - y0) / dy
        else:
            t_max_y = (cy * res + oy - y0) / dy

        dist = 0.0
        # 最多走 max_dist / res 步
        max_steps = int(max_dist / res) + 1
        for _ in range(max_steps):
            if occ[cy, cx]:
                return dist
            if t_max_x < t_max_y:
                dist = t_max_x
                t_max_x += t_delta_x
                cx += step_x
            else:
                dist = t_max_y
                t_max_y += t_delta_y
                cy += step_y
            if cx < 0 or cx >= w or cy < 0 or cy >= h:
                return None
            if dist > max_dist:
                return None
        return None

    # ============== 发布结果 ==============

    def _publish_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = _quat_from_yaw(yaw)
        msg.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.03,
        ]
        self._pose_pub.publish(msg)
        self.get_logger().info("已发布 /initialpose 给 AMCL。")


def main(args=None):
    rclpy.init(args=args)
    node = LidarGlobalLocalize()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
