#!/usr/bin/env python3
"""One-shot global 2D LiDAR localization using coarse search plus ICP."""

import heapq
import json
import math
import os
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class IcpGlobalLocalizer(Node):
    """Find global coarse hypotheses and refine them with point-to-point ICP."""

    def __init__(self) -> None:
        super().__init__("icp_global_localizer")
        self.declare_parameter("map_yaml_path", "")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("pose_topic", "/icp/initial_pose")
        self.declare_parameter("result_topic", "/initial_localization/result")
        self.declare_parameter("lidar_max_range_m", 8.0)
        self.declare_parameter("lidar_offset_x_m", 0.08)
        self.declare_parameter("coarse_xy_step_m", 0.50)
        self.declare_parameter("coarse_yaw_step_deg", 15.0)
        self.declare_parameter("coarse_scan_points", 72)
        self.declare_parameter("top_hypotheses", 30)
        self.declare_parameter("max_iterations", 60)
        self.declare_parameter("max_correspondence_m", 0.60)
        self.declare_parameter("min_correspondences", 30)
        self.declare_parameter("accept_rmse_m", 0.25)
        self.declare_parameter("accept_inlier_ratio", 0.60)

        map_yaml = str(self.get_parameter("map_yaml_path").value)
        if not map_yaml:
            raise RuntimeError("map_yaml_path is required")
        self._max_range = float(self.get_parameter("lidar_max_range_m").value)
        self._lidar_x = float(self.get_parameter("lidar_offset_x_m").value)
        self._xy_step = float(self.get_parameter("coarse_xy_step_m").value)
        self._yaw_step = math.radians(
            float(self.get_parameter("coarse_yaw_step_deg").value)
        )
        self._coarse_scan_count = max(
            12, int(self.get_parameter("coarse_scan_points").value)
        )
        self._top_count = max(1, int(self.get_parameter("top_hypotheses").value))
        self._max_iterations = max(1, int(self.get_parameter("max_iterations").value))
        self._max_correspondence = float(
            self.get_parameter("max_correspondence_m").value
        )
        self._min_correspondences = max(
            3, int(self.get_parameter("min_correspondences").value)
        )
        self._accept_rmse = float(self.get_parameter("accept_rmse_m").value)
        self._accept_inlier = float(
            self.get_parameter("accept_inlier_ratio").value
        )

        self._load_map(map_yaml)
        self._finished = False
        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            str(self.get_parameter("pose_topic").value),
            10,
        )
        self._result_pub = self.create_publisher(
            String, str(self.get_parameter("result_topic").value), 10
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._scan_callback,
            10,
        )
        self.get_logger().info(
            f"全局 ICP 就绪: map_points={len(self._map_points)}, "
            f"coarse_positions={len(self._candidate_positions)}, top={self._top_count}"
        )

    def _load_map(self, yaml_path: str) -> None:
        with open(yaml_path, "r", encoding="utf-8") as stream:
            meta = yaml.safe_load(stream)
        image_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), meta["image"])
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise FileNotFoundError(f"无法读取地图图片: {image_path}")

        self._resolution = float(meta["resolution"])
        self._origin_x = float(meta["origin"][0])
        self._origin_y = float(meta["origin"][1])
        self._height, self._width = image.shape
        normalized = image.astype(np.float32) / 255.0
        occupancy = normalized if meta.get("negate", 0) else 1.0 - normalized
        self._occupied = occupancy >= float(meta.get("occupied_thresh", 0.65))
        self._free = occupancy <= float(meta.get("free_thresh", 0.196))
        self._known = self._occupied | self._free

        rows, cols = np.where(self._occupied)
        map_x = self._origin_x + (cols.astype(np.float64) + 0.5) * self._resolution
        map_y = self._origin_y + (
            self._height - rows.astype(np.float64) - 0.5
        ) * self._resolution
        self._map_points = np.column_stack((map_x, map_y))
        self._map_tree = cKDTree(self._map_points)
        distance_input = (~self._occupied).astype(np.uint8)
        self._distance_m = (
            cv2.distanceTransform(distance_input, cv2.DIST_L2, 5) * self._resolution
        )

        stride = max(1, int(round(self._xy_step / self._resolution)))
        grid_rows, grid_cols = np.indices(image.shape)
        candidate_mask = (
            self._free & (grid_rows % stride == 0) & (grid_cols % stride == 0)
        )
        candidate_rows, candidate_cols = np.where(candidate_mask)
        candidate_x = self._origin_x + (
            candidate_cols.astype(np.float64) + 0.5
        ) * self._resolution
        candidate_y = self._origin_y + (
            self._height - candidate_rows.astype(np.float64) - 0.5
        ) * self._resolution
        self._candidate_positions = np.column_stack((candidate_x, candidate_y))
        if not len(self._candidate_positions) or not len(self._map_points):
            raise RuntimeError("地图没有可用的自由栅格或障碍物点")

    def _scan_callback(self, scan: LaserScan) -> None:
        if self._finished:
            return
        self._finished = True
        started = time.perf_counter()
        points = self._scan_points(scan)
        if len(points) < self._min_correspondences:
            self._publish_result("failed", started, reason="too_few_scan_points")
            return
        try:
            hypotheses = self._coarse_search(points)
            best = self._best_icp(points, hypotheses)
        except Exception as exc:
            self._publish_result("failed", started, reason=f"exception:{exc}")
            return
        if best is None:
            self._publish_result("failed", started, reason="icp_no_solution")
            return

        accepted = (
            best["rmse_m"] <= self._accept_rmse
            and best["inlier_ratio"] >= self._accept_inlier
        )
        status = "matched" if accepted else "failed"
        reason = "" if accepted else "quality_gate"
        self._publish_pose(scan, best)
        self._publish_result(status, started, reason=reason, estimate=best)

    def _scan_points(self, scan: LaserScan) -> np.ndarray:
        indices = np.arange(len(scan.ranges), dtype=np.float64)
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        valid = (
            np.isfinite(ranges)
            & (ranges >= max(0.05, float(scan.range_min)))
            & (ranges <= min(self._max_range, float(scan.range_max)))
        )
        angles = float(scan.angle_min) + indices[valid] * float(scan.angle_increment)
        selected = ranges[valid]
        return np.column_stack(
            (selected * np.cos(angles) + self._lidar_x, selected * np.sin(angles))
        )

    def _coarse_search(self, points: np.ndarray) -> list[tuple[float, float, float, float]]:
        step = max(1, len(points) // self._coarse_scan_count)
        coarse = points[::step][:self._coarse_scan_count]
        heap: list[tuple[float, float, float, float]] = []
        yaw_values = np.arange(-math.pi, math.pi, self._yaw_step)
        penalty = max(1.0, self._max_correspondence * 3.0)

        for yaw in yaw_values:
            cosine, sine = math.cos(yaw), math.sin(yaw)
            rotated = np.column_stack(
                (
                    cosine * coarse[:, 0] - sine * coarse[:, 1],
                    sine * coarse[:, 0] + cosine * coarse[:, 1],
                )
            )
            for start in range(0, len(self._candidate_positions), 256):
                positions = self._candidate_positions[start:start + 256]
                world = positions[:, None, :] + rotated[None, :, :]
                cols = np.rint(
                    (world[..., 0] - self._origin_x) / self._resolution - 0.5
                ).astype(np.int32)
                rows = np.rint(
                    self._height - 0.5
                    - (world[..., 1] - self._origin_y) / self._resolution
                ).astype(np.int32)
                in_bounds = (
                    (rows >= 0) & (rows < self._height)
                    & (cols >= 0) & (cols < self._width)
                )
                safe_rows = np.clip(rows, 0, self._height - 1)
                safe_cols = np.clip(cols, 0, self._width - 1)
                valid = in_bounds & self._known[safe_rows, safe_cols]
                distances = np.minimum(
                    self._distance_m[safe_rows, safe_cols], penalty
                )
                distances[~valid] = penalty
                scores = np.mean(distances, axis=1)
                for index, score in enumerate(scores):
                    x, y = positions[index]
                    entry = (-float(score), float(x), float(y), float(yaw))
                    if len(heap) < self._top_count:
                        heapq.heappush(heap, entry)
                    elif score < -heap[0][0]:
                        heapq.heapreplace(heap, entry)
        return [(-item[0], item[1], item[2], item[3]) for item in heap]

    def _best_icp(
        self, points: np.ndarray, hypotheses: list[tuple[float, float, float, float]]
    ) -> Optional[dict]:
        best: Optional[dict] = None
        for coarse_score, x, y, yaw in hypotheses:
            candidate = self._refine_icp(points, x, y, yaw)
            if candidate is None:
                continue
            candidate["coarse_score_m"] = coarse_score
            key = (candidate["rmse_m"], -candidate["inlier_ratio"])
            if best is None or key < (best["rmse_m"], -best["inlier_ratio"]):
                best = candidate
        return best

    def _refine_icp(
        self, points: np.ndarray, x: float, y: float, yaw: float
    ) -> Optional[dict]:
        iterations = 0
        for iterations in range(1, self._max_iterations + 1):
            cosine, sine = math.cos(yaw), math.sin(yaw)
            transformed = np.column_stack(
                (
                    cosine * points[:, 0] - sine * points[:, 1] + x,
                    sine * points[:, 0] + cosine * points[:, 1] + y,
                )
            )
            distances, indices = self._map_tree.query(transformed, k=1)
            inliers = distances <= self._max_correspondence
            if int(np.count_nonzero(inliers)) < self._min_correspondences:
                return None
            source = points[inliers]
            target = self._map_points[indices[inliers]]
            source_center = np.mean(source, axis=0)
            target_center = np.mean(target, axis=0)
            covariance = (source - source_center).T @ (target - target_center)
            u_matrix, _, vt_matrix = np.linalg.svd(covariance)
            rotation = vt_matrix.T @ u_matrix.T
            if np.linalg.det(rotation) < 0.0:
                vt_matrix[-1, :] *= -1.0
                rotation = vt_matrix.T @ u_matrix.T
            new_yaw = math.atan2(rotation[1, 0], rotation[0, 0])
            translation = target_center - rotation @ source_center
            new_x, new_y = float(translation[0]), float(translation[1])
            if math.hypot(new_x - x, new_y - y) < 1e-4 and abs(
                _wrap(new_yaw - yaw)
            ) < 1e-4:
                x, y, yaw = new_x, new_y, new_yaw
                break
            x, y, yaw = new_x, new_y, new_yaw

        cosine, sine = math.cos(yaw), math.sin(yaw)
        transformed = np.column_stack(
            (
                cosine * points[:, 0] - sine * points[:, 1] + x,
                sine * points[:, 0] + cosine * points[:, 1] + y,
            )
        )
        distances, _ = self._map_tree.query(transformed, k=1)
        inliers = distances <= self._max_correspondence
        count = int(np.count_nonzero(inliers))
        if count < self._min_correspondences:
            return None
        return {
            "x": x,
            "y": y,
            "yaw_deg": math.degrees(_wrap(yaw)),
            "rmse_m": float(math.sqrt(np.mean(np.square(distances[inliers])))),
            "inlier_ratio": float(count / len(points)),
            "iterations": iterations,
        }

    def _publish_pose(self, scan: LaserScan, estimate: dict) -> None:
        message = PoseWithCovarianceStamped()
        message.header = scan.header
        message.header.frame_id = "map"
        message.pose.pose.position.x = estimate["x"]
        message.pose.pose.position.y = estimate["y"]
        yaw = math.radians(estimate["yaw_deg"])
        message.pose.pose.orientation.z = math.sin(yaw * 0.5)
        message.pose.pose.orientation.w = math.cos(yaw * 0.5)
        variance = estimate["rmse_m"] ** 2
        message.pose.covariance[0] = variance
        message.pose.covariance[7] = variance
        message.pose.covariance[35] = variance
        self._pose_pub.publish(message)

    def _publish_result(
        self,
        status: str,
        started: float,
        reason: str = "",
        estimate: Optional[dict] = None,
    ) -> None:
        result = {
            "method": "icp",
            "status": status,
            "reason": reason,
            "elapsed_ms": (time.perf_counter() - started) * 1000.0,
        }
        if estimate is not None:
            result.update(estimate)
        message = String()
        message.data = json.dumps(result, ensure_ascii=False)
        self._result_pub.publish(message)
        print("[INITIAL_LOCALIZATION_RESULT] " + message.data, flush=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IcpGlobalLocalizer()
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
