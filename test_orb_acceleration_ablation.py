#!/usr/bin/env python3
"""Run the three-stage ORB global-localization ablation in Gazebo.

The production solver is imported unchanged.  In worker processes this script
temporarily replaces only its candidate-source and distance-map dependencies:

1. baseline: all free cells + per-pixel ray casting;
2. filter: minimum-range filtering + per-pixel ray casting;
3. full: minimum-range filtering + distance-transform jump scanning.

Each trial starts Gazebo once, captures one LaserScan, and reuses that exact
scan for all groups.  NumPy's RNG is reset before every group so the filter and
full groups sample the same candidates.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import os
import random
import re
import shlex
import signal
import statistics
import subprocess
import sys
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Iterator

import cv2
import numpy as np
import yaml


PROJECT_ROOT = Path(__file__).resolve().parent
ROS_SETUP = Path("/opt/ros/humble/setup.bash")
PROJECT_SETUP = PROJECT_ROOT / "install" / "setup.bash"
CLEANUP_SCRIPT = PROJECT_ROOT / "cleanup.sh"
DEFAULT_MAP_YAML = PROJECT_ROOT / "src" / "nav_slam" / "map" / "dashgo_slam_map.yaml"
DEFAULT_OUTPUT_ROOT = PROJECT_ROOT / "test_results"
RESULT_MARKER = "[ORB_ABLATION_RESULT]"
GROUPS = ("baseline", "filter", "full")
GROUP_LABELS = {
    "baseline": "原始方法",
    "filter": "仅最小距离筛选",
    "full": "筛选+跳跃扫描",
}
PLOT_LABELS = {
    "baseline": "Baseline",
    "filter": "Filter only",
    "full": "Filter + jump scan",
}
CSV_FIELDS = [
    "run_id", "seed", "solver_seed", "execution_order", "group", "group_label",
    "status", "reason", "returned_result", "accuracy_success", "timed_out",
    "spawn_x", "spawn_y", "spawn_yaw_deg", "estimate_x", "estimate_y",
    "estimate_yaw_deg", "position_error_m", "yaw_error_deg", "f1_score",
    "total_time_ms", "distance_transform_time_ms", "scan_dt_time_ms",
    "filter_dt_time_ms", "filter_time_ms", "sim_scan_time_ms", "matching_time_ms",
    "sim_scan_mean_per_candidate_ms", "matching_mean_per_candidate_ms",
    "scoring_time_ms", "initial_candidate_count", "selected_candidate_count",
    "planned_iterations", "tested_candidates", "matching_calls", "used_threshold_px",
    "filter_trace", "max_iterations", "max_time_budget_ms", "stop_f1",
]


def wrap_yaw_deg(first: float, second: float) -> float:
    """Return the absolute wrapped difference in degrees."""
    difference = abs(first - second) % 360.0
    return min(difference, 360.0 - difference)


def percentile(values: list[float], percent: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * percent / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def metric_stats(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "mean": None, "median": None, "p95": None}
    return {
        "count": len(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "p95": percentile(values, 95.0),
    }


def load_map(map_yaml: Path) -> tuple[np.ndarray, dict[str, Any], Path]:
    with map_yaml.open("r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    image_path = (map_yaml.parent / str(metadata["image"])).resolve()
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f"无法读取地图图片: {image_path}")
    return image, metadata, image_path


def pick_random_free_pose(map_yaml: Path, seed: int) -> tuple[float, float, float]:
    """Use the same one-metre-clearance distribution as the baseline tests."""
    image, metadata, _ = load_map(map_yaml)
    resolution = float(metadata["resolution"])
    origin_x = float(metadata["origin"][0])
    origin_y = float(metadata["origin"][1])
    normalized = image.astype(np.float32) / 255.0
    occupancy = normalized if metadata.get("negate", 0) else 1.0 - normalized
    free = occupancy < float(metadata.get("free_thresh", 0.196))
    occupied = occupancy > float(metadata.get("occupied_thresh", 0.65))
    valid_free = free & ~occupied

    safe_radius_m = 1.0
    border = int(safe_radius_m / resolution) + 1
    padded = np.pad(valid_free.astype(np.uint8), border, constant_values=0)
    clearance = cv2.distanceTransform(padded, cv2.DIST_L2, 5)[
        border:-border, border:-border
    ]
    rows, cols = np.where(clearance >= safe_radius_m / resolution)
    if not len(rows):
        raise RuntimeError("地图中没有满足1 m安全半径的出生位置")

    generator = random.Random(seed)
    index = generator.randrange(len(rows))
    row, col = int(rows[index]), int(cols[index])
    yaw = generator.uniform(-math.pi, math.pi)
    height = image.shape[0]
    x = origin_x + (col + 0.5) * resolution
    y = origin_y + (height - row - 0.5) * resolution
    return x, y, yaw


def rotated_group_order(run_id: int, selected: tuple[str, ...]) -> tuple[str, ...]:
    """Rotate execution order to distribute cache and thermal effects."""
    if len(selected) < 2:
        return selected
    offset = (run_id - 1) % len(selected)
    return selected[offset:] + selected[:offset]


def _all_free_candidates(
    map_image: np.ndarray,
) -> tuple[np.ndarray, tuple[np.ndarray, np.ndarray], int, int]:
    _, binary = cv2.threshold(map_image, 150, 255, cv2.THRESH_BINARY)
    pixels = np.where(binary == 255)
    return cv2.cvtColor(binary, cv2.COLOR_GRAY2RGB), pixels, len(pixels[0]), -1


@contextmanager
def instrumented_solver(group: str, metrics: dict[str, Any]) -> Iterator[Any]:
    """Temporarily configure and instrument the unchanged production solver."""
    krf_path = PROJECT_ROOT / "src" / "kidnapped_robot_finder"
    if str(krf_path) not in sys.path:
        sys.path.insert(0, str(krf_path))
    from global_localizer import kidnap_solver  # pylint: disable=import-outside-toplevel

    original_get_candidates = kidnap_solver._get_candidates
    original_compute_dt = kidnap_solver.s_sim.compute_dt_map
    original_filter_dt = kidnap_solver.dt.get_distance_transform
    original_get_scan = kidnap_solver.pf.get_scan_image
    original_matching = kidnap_solver.fm.do_matching
    original_scoring = kidnap_solver.get_f1_score

    metrics.update({
        "scan_dt_time_ms": 0.0,
        "filter_dt_time_ms": 0.0,
        "filter_time_ms": 0.0,
        "sim_scan_time_ms": 0.0,
        "matching_time_ms": 0.0,
        "scoring_time_ms": 0.0,
        "tested_candidates": 0,
        "matching_calls": 0,
        "filter_trace": [],
    })

    def timed_scan_dt(map_image: np.ndarray):
        if group != "full":
            return None
        started = time.perf_counter()
        result = original_compute_dt(map_image)
        metrics["scan_dt_time_ms"] += (time.perf_counter() - started) * 1000.0
        return result

    def timed_filter_dt(*args, **kwargs):
        started = time.perf_counter()
        result = original_filter_dt(*args, **kwargs)
        metrics["filter_dt_time_ms"] += (time.perf_counter() - started) * 1000.0
        threshold_px = int(kwargs.get("threshold_px", 3))
        count = int(np.count_nonzero(result[:, :, 0] == 255))
        metrics["filter_trace"].append({"threshold_px": threshold_px, "count": count})
        return result

    def timed_candidates(map_image, min_distance, map_resolution, min_required=500):
        started = time.perf_counter()
        if group == "baseline":
            result = _all_free_candidates(map_image)
        else:
            result = original_get_candidates(
                map_image, min_distance, map_resolution, min_required=min_required
            )
        metrics["filter_time_ms"] += (time.perf_counter() - started) * 1000.0
        metrics["selected_candidate_count"] = int(result[2])
        metrics["used_threshold_px"] = int(result[3])
        return result

    def timed_scan(*args, **kwargs):
        started = time.perf_counter()
        result = original_get_scan(*args, **kwargs)
        metrics["sim_scan_time_ms"] += (time.perf_counter() - started) * 1000.0
        metrics["tested_candidates"] += 1
        return result

    def timed_matching(*args, **kwargs):
        started = time.perf_counter()
        result = original_matching(*args, **kwargs)
        metrics["matching_time_ms"] += (time.perf_counter() - started) * 1000.0
        metrics["matching_calls"] += 1
        return result

    def timed_scoring(*args, **kwargs):
        started = time.perf_counter()
        result = original_scoring(*args, **kwargs)
        metrics["scoring_time_ms"] += (time.perf_counter() - started) * 1000.0
        return result

    kidnap_solver.s_sim.compute_dt_map = timed_scan_dt
    kidnap_solver.dt.get_distance_transform = timed_filter_dt
    kidnap_solver._get_candidates = timed_candidates
    kidnap_solver.pf.get_scan_image = timed_scan
    kidnap_solver.fm.do_matching = timed_matching
    kidnap_solver.get_f1_score = timed_scoring
    try:
        yield kidnap_solver
    finally:
        kidnap_solver.s_sim.compute_dt_map = original_compute_dt
        kidnap_solver.dt.get_distance_transform = original_filter_dt
        kidnap_solver._get_candidates = original_get_candidates
        kidnap_solver.pf.get_scan_image = original_get_scan
        kidnap_solver.fm.do_matching = original_matching
        kidnap_solver.get_f1_score = original_scoring


def solve_group(
    group: str,
    scan_image: np.ndarray,
    min_distance: float,
    map_image: np.ndarray,
    metadata: dict[str, Any],
    args: argparse.Namespace,
) -> dict[str, Any]:
    """Run one group on a previously captured scan."""
    metrics: dict[str, Any] = {}
    resolution = float(metadata.get("resolution", args.map_resolution))
    origin = tuple(float(value) for value in metadata.get("origin", [0.0, 0.0])[:2])
    pose_offset = tuple(
        float(value) for value in metadata.get("localization_pose_offset", [0.0, 0.0])
    )
    free_count = int(np.count_nonzero(map_image > 150))
    # Importing the solver also imports matplotlib.  Do this before starting the
    # timer so the first group in a trial does not pay a one-off import penalty.
    krf_path = PROJECT_ROOT / "src" / "kidnapped_robot_finder"
    if str(krf_path) not in sys.path:
        sys.path.insert(0, str(krf_path))
    from global_localizer import kidnap_solver as _preloaded_solver  # noqa: F401
    np.random.seed(args.solver_seed)
    started = time.perf_counter()
    try:
        with instrumented_solver(group, metrics) as solver:
            result = solver.solve_kidnap(
                scan_image.copy(),
                map_image,
                min_distance,
                map_origin=origin,
                map_resolution=resolution,
                max_iterations=args.max_iterations,
                max_time_budget_ms=args.max_time_ms,
                stop_search_threshold=args.stop_f1,
                lidar_range=args.lidar_range,
                map_pose_offset=pose_offset,
                show_plot=False,
            )
        reason = ""
    except Exception as exc:  # preserve the other groups when one group fails
        result = None
        reason = f"{type(exc).__name__}: {exc}"
    total_ms = (time.perf_counter() - started) * 1000.0

    selected_count = int(metrics.get("selected_candidate_count", 0))
    from global_localizer.kidnap_solver import _compute_adaptive_iterations
    planned_iterations = _compute_adaptive_iterations(
        selected_count, args.max_iterations, args.max_time_ms
    ) if selected_count else 0
    timed_out = result is None and total_ms >= 0.98 * args.max_time_ms
    row: dict[str, Any] = {
        "group": group,
        "group_label": GROUP_LABELS[group],
        "status": "error" if reason else ("timeout" if timed_out else (
            "no_candidate" if result is None else "matched")),
        "reason": reason,
        "returned_result": result is not None,
        "timed_out": timed_out,
        "total_time_ms": total_ms,
        "distance_transform_time_ms": (
            metrics.get("scan_dt_time_ms", 0.0) + metrics.get("filter_dt_time_ms", 0.0)
        ),
        "initial_candidate_count": free_count,
        "planned_iterations": planned_iterations,
        "max_iterations": args.max_iterations,
        "max_time_budget_ms": args.max_time_ms,
        "stop_f1": args.stop_f1,
        **metrics,
    }
    tested_candidates = int(row.get("tested_candidates", 0))
    matching_calls = int(row.get("matching_calls", 0))
    row["sim_scan_mean_per_candidate_ms"] = (
        float(row["sim_scan_time_ms"]) / tested_candidates if tested_candidates else None
    )
    row["matching_mean_per_candidate_ms"] = (
        float(row["matching_time_ms"]) / matching_calls if matching_calls else None
    )
    row["filter_trace"] = json.dumps(
        row.get("filter_trace", []), ensure_ascii=False, separators=(",", ":")
    )
    if result is not None:
        estimate_x, estimate_y, estimate_yaw, f1 = result
        estimate_yaw = math.atan2(
            math.sin(float(estimate_yaw) + math.pi),
            math.cos(float(estimate_yaw) + math.pi),
        )
        row.update(
            estimate_x=float(estimate_x),
            estimate_y=float(estimate_y),
            estimate_yaw_deg=math.degrees(estimate_yaw),
            f1_score=float(f1),
        )
    return row


def render_scan(message: Any, lidar_range: float, resolution: float) -> tuple[np.ndarray, float]:
    size = int(2.0 * lidar_range / resolution)
    offset = int(lidar_range / resolution)
    image = np.zeros((size, size), dtype=np.uint8)
    minimum = math.inf
    for index, range_value in enumerate(message.ranges):
        if not (0.0 < range_value < lidar_range):
            continue
        angle = message.angle_min + index * message.angle_increment
        px = int(range_value * math.cos(angle) / resolution + offset)
        py = int(range_value * math.sin(angle) / resolution + offset)
        if 0 <= px < size and 0 <= py < size:
            minimum = min(minimum, float(range_value))
            cv2.circle(image, (px, py), 1, 255, -1)
    if not math.isfinite(minimum):
        raise RuntimeError("LaserScan没有有效量程")
    return image, minimum


def run_worker(args: argparse.Namespace) -> int:
    """Capture one Gazebo scan and run all requested groups in one process."""
    import rclpy  # pylint: disable=import-outside-toplevel
    from rclpy.node import Node  # pylint: disable=import-outside-toplevel
    from sensor_msgs.msg import LaserScan  # pylint: disable=import-outside-toplevel

    map_image, metadata, _ = load_map(Path(args.map_yaml))
    resolution = float(metadata.get("resolution", args.map_resolution))

    class ScanCapture(Node):
        def __init__(self) -> None:
            super().__init__("orb_ablation_scan_capture")
            self.count = 0
            self.scan = None
            self.create_subscription(LaserScan, args.scan_topic, self.callback, 10)

        def callback(self, message) -> None:
            self.count += 1
            if self.count >= args.capture_scan_number and self.scan is None:
                self.scan = message

    rclpy.init()
    node = ScanCapture()
    deadline = time.monotonic() + args.scan_timeout_sec
    try:
        while node.scan is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.scan is None:
            print(f"{RESULT_MARKER} " + json.dumps({
                "status": "scan_timeout", "reason": "等待LaserScan超时"
            }, ensure_ascii=False), flush=True)
            return 2
        scan_image, min_distance = render_scan(node.scan, args.lidar_range, resolution)
        for group in args.group_order:
            row = solve_group(group, scan_image, min_distance, map_image, metadata, args)
            print(f"{RESULT_MARKER} {json.dumps(row, ensure_ascii=False)}", flush=True)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def terminate_process_group(process: subprocess.Popen) -> None:
    try:
        process_group = os.getpgid(process.pid)
    except ProcessLookupError:
        return
    for signal_number, timeout in (
        (signal.SIGINT, 8.0),
        (signal.SIGTERM, 5.0),
        (signal.SIGKILL, 3.0),
    ):
        try:
            os.killpg(process_group, signal_number)
            process.wait(timeout=timeout)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            continue


def cleanup(fast: bool) -> None:
    environment = os.environ.copy()
    if fast:
        environment["FAST_CLEANUP"] = "1"
    subprocess.run(
        ["bash", str(CLEANUP_SCRIPT), "--kill"],
        cwd=PROJECT_ROOT,
        env=environment,
        timeout=30 if fast else 90,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def sourced_command(command: list[str]) -> list[str]:
    shell = (
        f"source {shlex.quote(str(ROS_SETUP))} && "
        f"source {shlex.quote(str(PROJECT_SETUP))} && "
        f"{shlex.join(command)}"
    )
    return ["bash", "-lc", shell]


def parse_worker_results(output: str) -> list[dict[str, Any]]:
    results = []
    for line in output.splitlines():
        if RESULT_MARKER not in line:
            continue
        payload = line.split(RESULT_MARKER, 1)[1].strip()
        try:
            result = json.loads(payload)
        except json.JSONDecodeError:
            continue
        if result.get("group") in GROUPS:
            results.append(result)
    return results


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def summarize(rows: list[dict[str, Any]], max_time_ms: int) -> dict[str, Any]:
    summary: dict[str, Any] = {
        "success_definition": "position_error<=0.50m and yaw_error<=5deg",
        "timeout_accounting": "solver timeout retains measured time; missing worker result uses cap",
        "groups": {},
    }
    for group in GROUPS:
        group_rows = [row for row in rows if row.get("group") == group]
        successful = [row for row in group_rows if row.get("accuracy_success")]
        elapsed = [float(row.get("total_time_ms", max_time_ms)) for row in group_rows]
        group_summary = {
            "label": GROUP_LABELS[group],
            "runs": len(group_rows),
            "returned_count": sum(bool(row.get("returned_result")) for row in group_rows),
            "success_count": len(successful),
            "timeout_count": sum(bool(row.get("timed_out")) for row in group_rows),
            "total_time_ms": metric_stats(elapsed),
            "position_error_m_successful": metric_stats([
                float(row["position_error_m"]) for row in successful
            ]),
            "yaw_error_deg_successful": metric_stats([
                float(row["yaw_error_deg"]) for row in successful
            ]),
        }
        group_summary["success_rate_pct"] = (
            100.0 * len(successful) / len(group_rows) if group_rows else 0.0
        )
        for field in (
            "initial_candidate_count", "selected_candidate_count", "planned_iterations",
            "tested_candidates", "distance_transform_time_ms", "filter_time_ms",
            "sim_scan_time_ms", "sim_scan_mean_per_candidate_ms", "matching_time_ms",
            "matching_mean_per_candidate_ms",
        ):
            group_summary[field] = metric_stats([
                float(row[field]) for row in group_rows if row.get(field) is not None
            ])
        summary["groups"][group] = group_summary

    def median_metric(group: str, field: str) -> float | None:
        values = [
            float(row[field]) for row in rows
            if row.get("group") == group and row.get(field) is not None
        ]
        return statistics.median(values) if values else None

    baseline_total = median_metric("baseline", "total_time_ms")
    filter_total = median_metric("filter", "total_time_ms")
    full_total = median_metric("full", "total_time_ms")
    filter_scan = median_metric("filter", "sim_scan_time_ms")
    full_scan = median_metric("full", "sim_scan_time_ms")
    filter_scan_per_candidate = median_metric("filter", "sim_scan_mean_per_candidate_ms")
    full_scan_per_candidate = median_metric("full", "sim_scan_mean_per_candidate_ms")
    summary["speedups"] = {
        "filter_total_median_ratio": (
            baseline_total / filter_total if baseline_total and filter_total else None
        ),
        "scan_median_ratio": (
            filter_scan / full_scan if filter_scan and full_scan else None
        ),
        "scan_per_candidate_median_ratio": (
            filter_scan_per_candidate / full_scan_per_candidate
            if filter_scan_per_candidate and full_scan_per_candidate else None
        ),
        "overall_total_median_ratio": (
            baseline_total / full_total if baseline_total and full_total else None
        ),
    }
    return summary


def write_group_summary_csv(path: Path, summary: dict[str, Any]) -> None:
    """Write the paper-facing one-row-per-group statistical table."""
    fields = [
        "group", "group_label", "runs", "success_count", "success_rate_pct",
        "timeout_count", "position_error_mean_m", "position_error_median_m",
        "yaw_error_mean_deg", "yaw_error_median_deg", "total_time_mean_ms",
        "total_time_median_ms", "total_time_p95_ms", "initial_candidates_mean",
        "selected_candidates_mean", "tested_candidates_mean", "filter_time_mean_ms",
        "distance_transform_time_mean_ms", "sim_scan_time_mean_ms",
        "sim_scan_mean_per_candidate_ms", "matching_time_mean_ms",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for group in GROUPS:
            item = summary["groups"][group]
            writer.writerow({
                "group": group,
                "group_label": item["label"],
                "runs": item["runs"],
                "success_count": item["success_count"],
                "success_rate_pct": item["success_rate_pct"],
                "timeout_count": item["timeout_count"],
                "position_error_mean_m": item["position_error_m_successful"]["mean"],
                "position_error_median_m": item["position_error_m_successful"]["median"],
                "yaw_error_mean_deg": item["yaw_error_deg_successful"]["mean"],
                "yaw_error_median_deg": item["yaw_error_deg_successful"]["median"],
                "total_time_mean_ms": item["total_time_ms"]["mean"],
                "total_time_median_ms": item["total_time_ms"]["median"],
                "total_time_p95_ms": item["total_time_ms"]["p95"],
                "initial_candidates_mean": item["initial_candidate_count"]["mean"],
                "selected_candidates_mean": item["selected_candidate_count"]["mean"],
                "tested_candidates_mean": item["tested_candidates"]["mean"],
                "filter_time_mean_ms": item["filter_time_ms"]["mean"],
                "distance_transform_time_mean_ms": item[
                    "distance_transform_time_ms"]["mean"],
                "sim_scan_time_mean_ms": item["sim_scan_time_ms"]["mean"],
                "sim_scan_mean_per_candidate_ms": item[
                    "sim_scan_mean_per_candidate_ms"]["mean"],
                "matching_time_mean_ms": item["matching_time_ms"]["mean"],
            })


def write_timing_boxplot(path: Path, rows: list[dict[str, Any]]) -> bool:
    """Create the test plan's total-time boxplot when matplotlib is available."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    data = [
        [float(row["total_time_ms"]) for row in rows
         if row.get("group") == group and row.get("total_time_ms") is not None]
        for group in GROUPS
    ]
    if any(not values for values in data):
        return False
    figure, axis = plt.subplots(figsize=(7.2, 4.5))
    axis.boxplot(data, labels=[PLOT_LABELS[group] for group in GROUPS], showmeans=True)
    axis.set_ylabel("Global relocalization time (ms)")
    axis.set_title("ORB global relocalization ablation")
    axis.grid(axis="y", alpha=0.25)
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)
    return True


def run_trial(
    args: argparse.Namespace,
    run_id: int,
    output_dir: Path,
    selected_groups: tuple[str, ...],
) -> list[dict[str, Any]]:
    seed = args.seed_base + run_id - 1
    solver_seed = args.solver_seed_base + run_id - 1
    spawn_x, spawn_y, spawn_yaw = pick_random_free_pose(Path(args.map_yaml), seed)
    order = rotated_group_order(run_id, selected_groups)
    log_prefix = output_dir / f"run_{run_id:04d}_seed_{seed}"
    environment = os.environ.copy()
    environment.setdefault("ROS_LOG_DIR", "/tmp/auto_nav2_orb_ablation_roslogs")
    Path(environment["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)

    gazebo_command = sourced_command([
        "ros2", "launch", "gazebo_modele", "gazebo.launch.py",
        f"world_name:={args.world_name}", "start_moving_obstacle:=false",
        "use_sim_time:=true", "start_gazebo_gui:=false",
        f"spawn_x:={spawn_x:.6f}", f"spawn_y:={spawn_y:.6f}",
        "spawn_z:=0.03", f"spawn_yaw:={spawn_yaw:.6f}",
    ])
    worker_command = sourced_command([
        sys.executable, str(Path(__file__).resolve()), "--worker",
        f"--map-yaml={args.map_yaml}", f"--scan-topic={args.scan_topic}",
        f"--group-order={','.join(order)}", f"--solver-seed={solver_seed}",
        f"--max-iterations={args.max_iterations}", f"--max-time-ms={args.max_time_ms}",
        f"--stop-f1={args.stop_f1}", f"--lidar-range={args.lidar_range}",
        f"--map-resolution={args.map_resolution}",
        f"--capture-scan-number={args.capture_scan_number}",
        f"--scan-timeout-sec={args.scan_timeout_sec}",
    ])

    cleanup(args.fast_cleanup)
    with (log_prefix.with_suffix(".gazebo.log")).open("w", encoding="utf-8") as gazebo_log:
        gazebo = subprocess.Popen(
            gazebo_command,
            cwd=PROJECT_ROOT,
            env=environment,
            stdout=gazebo_log,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid,
        )
        worker = subprocess.Popen(
            worker_command,
            cwd=PROJECT_ROOT,
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid,
        )
        try:
            worker_output, _ = worker.communicate(timeout=args.trial_timeout_sec)
        except subprocess.TimeoutExpired:
            terminate_process_group(worker)
            worker_output = ""
        finally:
            terminate_process_group(gazebo)
            cleanup(args.fast_cleanup)

    (log_prefix.with_suffix(".worker.log")).write_text(worker_output, encoding="utf-8")
    parsed = parse_worker_results(worker_output)
    by_group = {row["group"]: row for row in parsed}
    results = []
    for group in selected_groups:
        row = by_group.get(group, {
            "group": group,
            "group_label": GROUP_LABELS[group],
            "status": "worker_timeout_or_exit",
            "reason": f"worker return code {worker.returncode}",
            "returned_result": False,
            "accuracy_success": False,
            "timed_out": True,
            "total_time_ms": float(args.max_time_ms),
        })
        row.update(
            run_id=run_id,
            seed=seed,
            solver_seed=solver_seed,
            execution_order=",".join(order),
            spawn_x=spawn_x,
            spawn_y=spawn_y,
            spawn_yaw_deg=math.degrees(spawn_yaw),
        )
        if row.get("returned_result"):
            row["position_error_m"] = math.hypot(
                float(row["estimate_x"]) - spawn_x,
                float(row["estimate_y"]) - spawn_y,
            )
            row["yaw_error_deg"] = wrap_yaw_deg(
                float(row["estimate_yaw_deg"]), math.degrees(spawn_yaw)
            )
            row["accuracy_success"] = bool(
                row["position_error_m"] <= args.success_xy_m
                and row["yaw_error_deg"] <= args.success_yaw_deg
            )
        results.append(row)
    return results


def parse_group_list(value: str) -> tuple[str, ...]:
    groups = tuple(item.strip() for item in value.split(",") if item.strip())
    invalid = [item for item in groups if item not in GROUPS]
    if not groups or invalid or len(set(groups)) != len(groups):
        raise argparse.ArgumentTypeError(
            f"组别必须是不重复的 {','.join(GROUPS)}，收到: {value}"
        )
    return groups


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ORB全局重定位三阶段加速消融测试")
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--runs", type=int, default=100)
    parser.add_argument("--groups", type=parse_group_list, default=GROUPS)
    parser.add_argument("--group-order", type=parse_group_list, default=GROUPS,
                        help=argparse.SUPPRESS)
    parser.add_argument("--map-yaml", default=str(DEFAULT_MAP_YAML))
    parser.add_argument("--world-name", default="small_house.world")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT))
    parser.add_argument("--seed-base", type=int, default=10001)
    parser.add_argument("--solver-seed-base", type=int, default=20001)
    parser.add_argument("--solver-seed", type=int, default=20001,
                        help=argparse.SUPPRESS)
    parser.add_argument("--max-iterations", type=int, default=250)
    parser.add_argument("--max-time-ms", type=int, default=5000)
    parser.add_argument("--stop-f1", type=float, default=50.0)
    parser.add_argument("--lidar-range", type=float, default=8.0)
    parser.add_argument("--map-resolution", type=float, default=0.05)
    parser.add_argument("--success-xy-m", type=float, default=0.50)
    parser.add_argument("--success-yaw-deg", type=float, default=5.0)
    parser.add_argument("--capture-scan-number", type=int, default=3)
    parser.add_argument("--scan-timeout-sec", type=float, default=60.0)
    parser.add_argument("--trial-timeout-sec", type=float, default=90.0)
    parser.add_argument("--fast-cleanup", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser


def validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    args.map_yaml = str(Path(args.map_yaml).resolve())
    args.output_root = str(Path(args.output_root).resolve())
    if args.runs < 1:
        parser.error("--runs必须大于0")
    if args.max_iterations < 1 or args.max_time_ms < 1:
        parser.error("迭代次数和时间预算必须大于0")
    if not Path(args.map_yaml).is_file():
        parser.error(f"地图不存在: {args.map_yaml}")
    if not args.worker and (not ROS_SETUP.is_file() or not PROJECT_SETUP.is_file()):
        parser.error("ROS 2或项目install/setup.bash不存在")


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    validate_args(parser, args)
    if args.worker:
        return run_worker(args)

    print(f"三组配置: {', '.join(f'{name}={GROUP_LABELS[name]}' for name in args.groups)}")
    print(f"运行次数: {args.runs}; seeds: {args.seed_base}..{args.seed_base + args.runs - 1}")
    print(f"地图: {args.map_yaml}; world: {args.world_name}")
    print(f"统一求解器随机种子基数: {args.solver_seed_base}")
    if args.dry_run:
        for run_id in range(1, min(args.runs, 6) + 1):
            print(f"run {run_id}: order={','.join(rotated_group_order(run_id, args.groups))}")
        return 0

    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path(args.output_root) / f"orb_acceleration_ablation_{timestamp}"
    output_dir.mkdir(parents=True, exist_ok=False)
    rows: list[dict[str, Any]] = []
    try:
        for run_id in range(1, args.runs + 1):
            print(f"\n[trial {run_id}/{args.runs}] seed={args.seed_base + run_id - 1}")
            trial_rows = run_trial(args, run_id, output_dir, args.groups)
            rows.extend(trial_rows)
            write_csv(output_dir / "all_trials.csv", rows)
            for row in trial_rows:
                print(
                    f"  {row['group']}: {row['status']}, "
                    f"success={row.get('accuracy_success', False)}, "
                    f"time={float(row.get('total_time_ms', 0.0)):.1f}ms"
                )
    except KeyboardInterrupt:
        print("\n用户中断，保存已完成结果。")

    summary = summarize(rows, args.max_time_ms)
    (output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    write_group_summary_csv(output_dir / "group_summary.csv", summary)
    write_timing_boxplot(output_dir / "total_time_boxplot.png", rows)
    metadata = vars(args).copy()
    metadata["groups"] = list(args.groups)
    metadata["created_at"] = dt.datetime.now().isoformat(timespec="seconds")
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(f"\n详细结果: {output_dir / 'all_trials.csv'}")
    print(f"统计表: {output_dir / 'group_summary.csv'}")
    print(f"汇总结果: {output_dir / 'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
