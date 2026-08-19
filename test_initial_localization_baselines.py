#!/usr/bin/env python3
"""Batch-test ORB, AMCL, or global ICP initial localization in Gazebo."""

import argparse
import csv
import datetime
import json
import math
import os
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent
CLEANUP_SCRIPT = PROJECT_ROOT / "cleanup.sh"
RESULTS_DIR = PROJECT_ROOT / "test_results"
DEFAULT_MAP_YAML = PROJECT_ROOT / "src" / "nav_slam" / "map" / "dashgo_slam_map.yaml"
ROS2_SETUP = "/opt/ros/humble/setup.bash"
PROJECT_SETUP = str(PROJECT_ROOT / "install" / "setup.bash")
LAUNCH_PACKAGE = "rtabmap_localization_bringup"
LAUNCH_FILES = {
    "amcl": "gazebo_amcl_initial_localization.launch.py",
    "icp": "gazebo_icp_initial_localization.launch.py",
    "orb": "gazebo_orb_initial_localization.launch.py",
}

RE_SPAWN = re.compile(
    r"\[random_spawn\].*?world=\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)°"
)
RE_RESULT = re.compile(r"\[INITIAL_LOCALIZATION_RESULT\]\s+(\{.*\})")
RE_ORB_START = re.compile(r"ORB 匹配中")
RE_ORB_RESULT = re.compile(
    r"定位完成:\s*x=([-\d.]+)\s+y=([-\d.]+)\s+"
    r"yaw=([-\d.]+)°\s+F1=([-\d.]+)"
)


def wrap_yaw_diff_deg(first: float, second: float) -> float:
    difference = abs(first - second) % 360.0
    return min(difference, 360.0 - difference)


class OutputMonitor(threading.Thread):
    def __init__(self, pipe, verbose: bool) -> None:
        super().__init__(daemon=True)
        self._pipe = pipe
        self._verbose = verbose
        self.lines: list[str] = []
        self.spawn: tuple[float, float, float] | None = None
        self.result: dict | None = None
        self.orb_started_at: float | None = None
        self.spawn_event = threading.Event()
        self.result_event = threading.Event()

    def run(self) -> None:
        for line in iter(self._pipe.readline, ""):
            self.lines.append(line)
            if self._verbose:
                sys.stderr.write(line)
            if self.spawn is None:
                match = RE_SPAWN.search(line)
                if match:
                    self.spawn = tuple(float(match.group(i)) for i in range(1, 4))
                    self.spawn_event.set()
            if self.result is None:
                match = RE_RESULT.search(line)
                if match:
                    try:
                        self.result = json.loads(match.group(1))
                    except json.JSONDecodeError:
                        continue
                    self.result_event.set()
                    continue
                if RE_ORB_START.search(line):
                    self.orb_started_at = time.monotonic()
                match = RE_ORB_RESULT.search(line)
                if match:
                    elapsed_ms = None
                    if self.orb_started_at is not None:
                        elapsed_ms = (
                            time.monotonic() - self.orb_started_at
                        ) * 1000.0
                    self.result = {
                        "status": "matched",
                        "x": float(match.group(1)),
                        "y": float(match.group(2)),
                        "yaw_deg": float(match.group(3)),
                        "f1_score": float(match.group(4)),
                        "elapsed_ms": elapsed_ms,
                    }
                    self.result_event.set()


class TrialRunner:
    def __init__(self, args, run_id: int) -> None:
        self.args = args
        self.run_id = run_id
        self.seed = args.seed_base + run_id - 1

    def _cleanup(self) -> None:
        command = ["bash", str(CLEANUP_SCRIPT), "--kill"]
        environment = os.environ.copy()
        if self.args.fast_cleanup:
            environment["FAST_CLEANUP"] = "1"
        subprocess.run(
            command,
            timeout=30 if self.args.fast_cleanup else 90,
            env=environment,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )

    def _launch(self) -> tuple[subprocess.Popen, OutputMonitor]:
        command = [
            "ros2", "launch", LAUNCH_PACKAGE, LAUNCH_FILES[self.args.method],
            f"static_map_yaml:={self.args.map_yaml}",
            f"world_name:={self.args.world_name}",
            f"spawn_seed:={self.seed}",
            "start_gazebo_gui:=false",
        ]
        if self.args.method == "amcl":
            command.append(f"localization_timeout_sec:={max(5.0, self.args.timeout - 5.0)}")
        shell_command = (
            f"source {shlex.quote(ROS2_SETUP)} && "
            f"source {shlex.quote(PROJECT_SETUP)} && "
            + shlex.join(command)
        )
        environment = os.environ.copy()
        environment.setdefault("ROS_LOG_DIR", "/tmp/auto_nav2_baseline_roslogs")
        os.makedirs(environment["ROS_LOG_DIR"], exist_ok=True)
        process = subprocess.Popen(
            ["bash", "-c", shell_command],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid,
            env=environment,
        )
        monitor = OutputMonitor(process.stdout, self.args.verbose)  # type: ignore[arg-type]
        monitor.start()
        return process, monitor

    @staticmethod
    def _shutdown(process: subprocess.Popen) -> None:
        try:
            group = os.getpgid(process.pid)
        except ProcessLookupError:
            return
        for signal_number, timeout in (
            (signal.SIGINT, 8.0),
            (signal.SIGTERM, 5.0),
            (signal.SIGKILL, 3.0),
        ):
            try:
                os.killpg(group, signal_number)
                process.wait(timeout=timeout)
                return
            except ProcessLookupError:
                return
            except subprocess.TimeoutExpired:
                continue

    def _debug_log(self, monitor: OutputMonitor, status: str) -> None:
        safe_status = re.sub(r"[^A-Za-z0-9_-]+", "_", status)[:50]
        path = Path(self.args.output_dir) / (
            f"{self.args.method}_debug_run_{self.run_id:04d}_{safe_status}.log"
        )
        path.write_text("".join(monitor.lines), encoding="utf-8")

    def run(self) -> dict:
        trial = {
            "run_id": self.run_id,
            "seed": self.seed,
            "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
            "method": self.args.method,
            "status": "UNKNOWN",
            "method_success": False,
            "accuracy_success": False,
        }
        self._cleanup()
        process, monitor = self._launch()
        try:
            if not monitor.spawn_event.wait(timeout=90.0):
                trial["status"] = "SPAWN_TIMEOUT"
                return trial
            spawn_x, spawn_y, spawn_yaw = monitor.spawn  # type: ignore[misc]
            trial.update(
                spawn_x=spawn_x, spawn_y=spawn_y, spawn_yaw_deg=spawn_yaw
            )

            deadline = time.monotonic() + self.args.timeout
            while time.monotonic() < deadline:
                if monitor.result_event.wait(timeout=0.2):
                    break
                if process.poll() is not None:
                    trial["status"] = f"PROCESS_EXIT_{process.returncode}"
                    return trial
            if monitor.result is None:
                trial["status"] = "LOCALIZATION_TIMEOUT"
                return trial

            result = monitor.result
            estimate_x = result.get("x")
            estimate_y = result.get("y")
            estimate_yaw = result.get("yaw_deg")
            ground_truth_x = result.get("gt_x")
            ground_truth_y = result.get("gt_y")
            ground_truth_yaw = result.get("gt_yaw_deg")
            ground_truth_source = result.get("ground_truth_source")
            if ground_truth_x is None or ground_truth_y is None:
                if self.args.method in {"icp", "orb"}:
                    ground_truth_x, ground_truth_y = spawn_x, spawn_y
                    ground_truth_yaw = spawn_yaw
                    ground_truth_source = "spawn_pose_static_robot"
            trial.update(
                status=result.get("status", "UNKNOWN"),
                reason=result.get("reason", ""),
                estimate_x=estimate_x,
                estimate_y=estimate_y,
                estimate_yaw_deg=estimate_yaw,
                ground_truth_x=ground_truth_x,
                ground_truth_y=ground_truth_y,
                ground_truth_yaw_deg=ground_truth_yaw,
                ground_truth_age_ms=result.get("gt_age_ms"),
                ground_truth_source=ground_truth_source,
                elapsed_ms=result.get("elapsed_ms"),
                cov_x=result.get("cov_x"),
                cov_y=result.get("cov_y"),
                cov_yaw=result.get("cov_yaw"),
                orb_f1_score=result.get("f1_score"),
                icp_rmse_m=result.get("rmse_m"),
                icp_inlier_ratio=result.get("inlier_ratio"),
                icp_coarse_score_m=result.get("coarse_score_m"),
                icp_iterations=result.get("iterations"),
            )
            trial["method_success"] = result.get("status") in {"converged", "matched"}
            if (
                estimate_x is not None
                and estimate_y is not None
                and ground_truth_x is not None
                and ground_truth_y is not None
            ):
                trial["error_xy_m"] = math.hypot(
                    float(estimate_x) - float(ground_truth_x),
                    float(estimate_y) - float(ground_truth_y),
                )
            if estimate_yaw is not None and ground_truth_yaw is not None:
                trial["error_yaw_deg"] = wrap_yaw_diff_deg(
                    float(estimate_yaw), float(ground_truth_yaw)
                )
            trial["accuracy_success"] = bool(
                trial["method_success"]
                and trial.get("error_xy_m", math.inf) <= self.args.success_xy_m
                and trial.get("error_yaw_deg", math.inf) <= self.args.success_yaw_deg
            )
            return trial
        finally:
            self._shutdown(process)
            self._cleanup()
            if not trial.get("accuracy_success", False):
                self._debug_log(monitor, str(trial["status"]))


CSV_FIELDS = [
    "run_id", "seed", "timestamp", "method", "status", "reason",
    "method_success", "accuracy_success",
    "spawn_x", "spawn_y", "spawn_yaw_deg",
    "ground_truth_x", "ground_truth_y", "ground_truth_yaw_deg",
    "ground_truth_age_ms", "ground_truth_source",
    "estimate_x", "estimate_y", "estimate_yaw_deg",
    "error_xy_m", "error_yaw_deg", "elapsed_ms",
    "cov_x", "cov_y", "cov_yaw",
    "orb_f1_score",
    "icp_rmse_m", "icp_inlier_ratio", "icp_coarse_score_m", "icp_iterations",
]


def write_results(results: list[dict], args, timestamp: str) -> Path:
    path = Path(args.output_dir) / f"{args.method}_results_{timestamp}.csv"
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(results)
    return path


def percentile(values: list[float], percent: float) -> float:
    """Return a linearly interpolated percentile of a non-empty list."""
    ordered = sorted(values)
    position = (len(ordered) - 1) * percent / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def metric_stats(values: list[float]) -> dict[str, float | int]:
    if not values:
        return {"count": 0}
    ordered = sorted(values)
    mean = sum(values) / len(values)
    return {
        "count": len(values),
        "mean": mean,
        "std": math.sqrt(sum((value - mean) ** 2 for value in values) / len(values)),
        "min": ordered[0],
        "p50": percentile(ordered, 50.0),
        "p90": percentile(ordered, 90.0),
        "p95": percentile(ordered, 95.0),
        "max": ordered[-1],
    }


def write_summary(results: list[dict], args, timestamp: str) -> Path:
    summary = {
        "method": args.method,
        "runs": len(results),
        "method_success_count": sum(bool(item["method_success"]) for item in results),
        "accuracy_success_count": sum(bool(item["accuracy_success"]) for item in results),
        "method_success_definition": {
            "amcl": "amcl_converged",
            "icp": "icp_quality_gate_accepted",
            "orb": "orb_match_returned",
        }[args.method],
        "accuracy_success_definition": "method_success_and_xy_yaw_thresholds",
        "ground_truth_source_counts": {
            source: sum(
                item.get("ground_truth_source") == source for item in results
            )
            for source in sorted({
                str(item.get("ground_truth_source"))
                for item in results if item.get("ground_truth_source")
            })
        },
        "success_xy_threshold_m": args.success_xy_m,
        "success_yaw_threshold_deg": args.success_yaw_deg,
        "error_xy_m": metric_stats([
            float(item["error_xy_m"]) for item in results
            if item.get("error_xy_m") is not None
        ]),
        "error_yaw_deg": metric_stats([
            float(item["error_yaw_deg"]) for item in results
            if item.get("error_yaw_deg") is not None
        ]),
        "elapsed_ms": metric_stats([
            float(item["elapsed_ms"]) for item in results
            if item.get("elapsed_ms") is not None
        ]),
        "orb_f1_score": metric_stats([
            float(item["orb_f1_score"]) for item in results
            if item.get("orb_f1_score") is not None
        ]),
    }
    summary["method_success_rate_pct"] = (
        100.0 * summary["method_success_count"] / len(results) if results else 0.0
    )
    summary["accuracy_success_rate_pct"] = (
        100.0 * summary["accuracy_success_count"] / len(results) if results else 0.0
    )
    path = Path(args.output_dir) / f"{args.method}_summary_{timestamp}.json"
    path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    return path


def main() -> None:
    parser = argparse.ArgumentParser(
        description="ORB / AMCL / 全局 ICP 初始定位重复测试",
    )
    parser.add_argument("--method", choices=sorted(LAUNCH_FILES), required=True)
    parser.add_argument("--runs", "-n", type=int, default=100)
    parser.add_argument("--timeout", "-t", type=float, default=120.0)
    parser.add_argument("--map-yaml", default=str(DEFAULT_MAP_YAML))
    parser.add_argument("--world-name", default="small_house.world")
    parser.add_argument("--output-dir", "-o", default=str(RESULTS_DIR))
    parser.add_argument("--seed-base", type=int, default=10001)
    parser.add_argument("--success-xy-m", type=float, default=0.50)
    parser.add_argument("--success-yaw-deg", type=float, default=5.0)
    parser.add_argument("--fast-cleanup", action="store_true")
    parser.add_argument("--verbose", "-v", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    args.map_yaml = os.path.abspath(args.map_yaml)
    args.output_dir = os.path.abspath(args.output_dir)
    if args.runs < 1:
        parser.error("--runs 必须大于 0")
    if not os.path.isfile(args.map_yaml):
        parser.error(f"地图不存在: {args.map_yaml}")

    if args.dry_run:
        print(f"Method: {args.method}")
        print(f"Runs: {args.runs}")
        print(f"Seeds: {args.seed_base}..{args.seed_base + args.runs - 1}")
        print(f"Map: {args.map_yaml}")
        print(f"World: {args.world_name}")
        print(f"Launch: {LAUNCH_PACKAGE} {LAUNCH_FILES[args.method]}")
        print(
            f"Accuracy success: xy <= {args.success_xy_m} m and "
            f"yaw <= {args.success_yaw_deg} deg"
        )
        return

    if not os.path.isfile(ROS2_SETUP) or not os.path.isfile(PROJECT_SETUP):
        parser.error("ROS 2 或项目 install/setup.bash 不存在，请先完成构建")
    os.makedirs(args.output_dir, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    results: list[dict] = []
    started = time.time()

    try:
        for run_id in range(1, args.runs + 1):
            print(
                f"\n[{args.method.upper()}] run {run_id}/{args.runs} "
                f"seed={args.seed_base + run_id - 1}"
            )
            result = TrialRunner(args, run_id).run()
            results.append(result)
            write_results(results, args, timestamp)
            print(
                f"status={result['status']} method_success={result['method_success']} "
                f"accuracy_success={result['accuracy_success']} "
                f"xy={result.get('error_xy_m', 'N/A')} "
                f"yaw={result.get('error_yaw_deg', 'N/A')}"
            )
    except KeyboardInterrupt:
        print("\n用户中断，保存已有结果。")

    results_path = write_results(results, args, timestamp)
    summary_path = write_summary(results, args, timestamp)
    elapsed = time.time() - started
    print(f"\n完成 {len(results)} 组，耗时 {elapsed / 60.0:.1f} min")
    print(f"详细结果: {results_path}")
    print(f"汇总结果: {summary_path}")


if __name__ == "__main__":
    main()
