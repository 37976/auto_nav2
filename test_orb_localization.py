#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_orb_localization.py — ORB 全局定位精度反复测试工具。

每次随机 spawn 机器人到地图空闲区域，ORB 通过激光雷达匹配估计位姿，
与真实 spawn 位置对比得出定位误差，导出汇总表格。

用法:
    python test_orb_localization.py --runs 100 --duration 15
    python test_orb_localization.py --runs 5 --duration 5 --verbose
    python test_orb_localization.py --dry-run
"""

import argparse
import csv
import datetime
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path


# ---------------------------------------------------------------------------
# 路径常量
# ---------------------------------------------------------------------------
PROJECT_ROOT = Path(__file__).resolve().parent
CLEANUP_SCRIPT = PROJECT_ROOT / "cleanup.sh"
POSE_CSV_PATH = Path.home() / "project" / "位姿对比" / "pose_comparison.csv"
RESULTS_DIR = PROJECT_ROOT / "test_results"

ROS2_SETUP = "/opt/ros/humble/setup.bash"
PROJECT_SETUP = str(PROJECT_ROOT / "install" / "setup.bash")

LAUNCH_PACKAGE = "rtabmap_localization_bringup"
LAUNCH_FILE = "gazebo_nav_xfeat_odometry.launch.py"

# ---------------------------------------------------------------------------
# 正则模式
# ---------------------------------------------------------------------------
# [random_spawn] 地图 400×720 @ 0.050m/格, 空闲候选 12345 个, 选中 px=(123,456) clearance=0.62m → world=(-1.60, -4.20, 34.9°)
RE_SPAWN = re.compile(
    r"\[random_spawn\].*?world=\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)°"
)

# 定位完成: x=-1.58 y=-4.15 yaw=33.2° F1=82.5
RE_ORB_OK = re.compile(
    r"定位完成:\s*x=([-\d.]+)\s+y=([-\d.]+)\s+yaw=([-\d.]+)°\s+F1=([-\d.]+)"
)

# 定位失败
RE_ORB_FAIL = re.compile(r"定位失败")

# 定位失败详细信息
RE_ORB_ERROR = re.compile(r"定位失败[：:](.+)")
RE_ORB_ERROR2 = re.compile(r"ERROR:\s*(.+)")  # kidnap_solver 里的 ERROR


# ---------------------------------------------------------------------------
# 角度工具
# ---------------------------------------------------------------------------
def wrap_yaw_diff_deg(a_deg: float, b_deg: float) -> float:
    """返回 [0, 180] 范围内的角度差（度）。"""
    d = abs(a_deg - b_deg) % 360.0
    return min(d, 360.0 - d)


# ---------------------------------------------------------------------------
# StdoutMonitor — 后台线程解析 launch stdout
# ---------------------------------------------------------------------------
class StdoutMonitor(threading.Thread):
    """在后台线程中读取 launch 进程的 stdout，匹配关键事件。"""

    def __init__(self, pipe, verbose: bool = False):
        super().__init__(daemon=True)
        self._pipe = pipe
        self._verbose = verbose
        self.lines: list[str] = []

        # 解析结果
        self.spawn_x: float | None = None
        self.spawn_y: float | None = None
        self.spawn_yaw_deg: float | None = None

        self.orb_x: float | None = None
        self.orb_y: float | None = None
        self.orb_yaw_deg: float | None = None
        self.orb_f1: float | None = None

        self.orb_failed = False
        self.orb_fail_reason: str = ""

        # 同步事件
        self.spawn_event = threading.Event()
        self.orb_event = threading.Event()

    def run(self):
        for line in iter(self._pipe.readline, ""):
            self.lines.append(line)

            # Verbose 模式实时打印
            if self._verbose:
                sys.stderr.write(line)

            # 匹配 spawn
            if not self.spawn_event.is_set():
                m = RE_SPAWN.search(line)
                if m:
                    self.spawn_x = float(m.group(1))
                    self.spawn_y = float(m.group(2))
                    self.spawn_yaw_deg = float(m.group(3))
                    self.spawn_event.set()

            # 匹配 ORB 结果
            if not self.orb_event.is_set():
                m = RE_ORB_OK.search(line)
                if m:
                    self.orb_x = float(m.group(1))
                    self.orb_y = float(m.group(2))
                    self.orb_yaw_deg = float(m.group(3))
                    self.orb_f1 = float(m.group(4))
                    self.orb_event.set()
                    continue

                if RE_ORB_FAIL.search(line):
                    self.orb_failed = True
                    # 尝试提取失败原因
                    fm = RE_ORB_ERROR.search(line)
                    if fm:
                        self.orb_fail_reason = fm.group(1).strip()
                    else:
                        self.orb_fail_reason = line.strip()
                    self.orb_event.set()
                    continue

                # kidnap_solver 内部的错误
                em = RE_ORB_ERROR2.search(line)
                if em and "未找到" in em.group(1):
                    self.orb_failed = True
                    self.orb_fail_reason = em.group(1).strip()
                    self.orb_event.set()


# ---------------------------------------------------------------------------
# RunManager — 单次测试运行的生命周期管理
# ---------------------------------------------------------------------------
class RunManager:
    """编排一次测试运行：清理 → 启动 → 等待 → 采集 → 停止 → 读结果。"""

    def __init__(self, run_id: int, duration: float, timeout: float,
                 verbose: bool = False, fast_cleanup: bool = False,
                 debug_dir: str | None = None):
        self._run_id = run_id
        self._duration = duration
        self._timeout = timeout
        self._verbose = verbose
        self._fast_cleanup = fast_cleanup
        self._debug_dir = debug_dir
        self._last_monitor: StdoutMonitor | None = None

    # ---- cleanup -----------------------------------------------------------
    def _run_cleanup(self, kill: bool = True):
        """执行 cleanup.sh。kill=True 时终止残留进程并等待端口释放。"""
        cmd = ["bash", str(CLEANUP_SCRIPT)]
        if kill:
            cmd.append("--kill")

        if self._fast_cleanup and kill:
            # 快速清理：只杀进程 + 清理共享内存，跳过 65 秒端口等待
            env = os.environ.copy()
            env["FAST_CLEANUP"] = "1"
            subprocess.run(cmd, timeout=30, env=env,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            subprocess.run(cmd, timeout=90,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # ---- launch ------------------------------------------------------------
    def _launch(self) -> tuple[subprocess.Popen, StdoutMonitor]:
        """启动 ros2 launch，返回 (进程, 监控器)。"""
        cmd = [
            "ros2", "launch", LAUNCH_PACKAGE, LAUNCH_FILE,
            "start_nav_rviz:=false",
            "start_web_ui:=false",
        ]

        # 构造环境：source ROS2 + 项目
        # ros2 需要 setup 后的环境，直接用 bash -c "source ... && ..."
        setup_cmd = (
            f'source "{ROS2_SETUP}" && '
            f'source "{PROJECT_SETUP}" && '
            + " ".join(cmd)
        )

        proc = subprocess.Popen(
            ["bash", "-c", setup_cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid,  # 创建独立进程组，便于清理
        )

        monitor = StdoutMonitor(proc.stdout, verbose=self._verbose)  # type: ignore[arg-type]
        monitor.start()

        return proc, monitor

    # ---- shutdown ----------------------------------------------------------
    def _shutdown(self, proc: subprocess.Popen):
        """优雅关闭进程组：SIGINT → SIGTERM → SIGKILL。"""
        try:
            pgid = os.getpgid(proc.pid)
        except ProcessLookupError:
            return

        for sig, wait_s, name in [
            (signal.SIGINT, 8.0, "SIGINT"),
            (signal.SIGTERM, 5.0, "SIGTERM"),
            (signal.SIGKILL, 3.0, "SIGKILL"),
        ]:
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                return
            try:
                proc.wait(timeout=wait_s)
                return
            except subprocess.TimeoutExpired:
                if self._verbose:
                    print(f"  [{name}] 等待 {wait_s}s 超时，升级...")

    # ---- tracking CSV ------------------------------------------------------
    def _read_tracking_csv(self) -> dict:
        """读取 pose_logger 输出的 CSV，返回跟踪误差统计。"""
        path = POSE_CSV_PATH
        if not path.exists():
            return {
                "tracking_mean_xy_m": None,
                "tracking_max_xy_m": None,
                "tracking_mean_yaw_deg": None,
                "tracking_max_yaw_deg": None,
                "tracking_samples": 0,
            }

        try:
            with open(path, "r", encoding="utf-8") as f:
                reader = csv.DictReader(f)
                errors_xy = []
                errors_yaw = []
                for row in reader:
                    try:
                        errors_xy.append(float(row["error_xy_m"]))
                        errors_yaw.append(float(row["error_yaw_deg"]))
                    except (KeyError, ValueError):
                        continue

            if not errors_xy:
                return {
                    "tracking_mean_xy_m": None,
                    "tracking_max_xy_m": None,
                    "tracking_mean_yaw_deg": None,
                    "tracking_max_yaw_deg": None,
                    "tracking_samples": 0,
                }

            n = len(errors_xy)
            return {
                "tracking_mean_xy_m": sum(errors_xy) / n,
                "tracking_max_xy_m": max(errors_xy),
                "tracking_mean_yaw_deg": sum(errors_yaw) / n,
                "tracking_max_yaw_deg": max(errors_yaw),
                "tracking_samples": n,
            }
        except Exception as e:
            return {
                "tracking_mean_xy_m": None,
                "tracking_max_xy_m": None,
                "tracking_mean_yaw_deg": None,
                "tracking_max_yaw_deg": None,
                "tracking_samples": 0,
                "tracking_error": str(e),
            }

    # ---- debug -------------------------------------------------------------
    def _dump_debug(self, monitor: StdoutMonitor, status: str):
        """保存失败运行的 stdout 到调试文件。"""
        if self._debug_dir is None:
            return
        os.makedirs(self._debug_dir, exist_ok=True)
        debug_path = os.path.join(
            self._debug_dir,
            f"debug_run_{self._run_id:04d}_{status.replace(' ', '_')}.log"
        )
        try:
            with open(debug_path, "w", encoding="utf-8") as f:
                f.writelines(monitor.lines)
        except Exception:
            pass

    # ---- 主流程 ------------------------------------------------------------
    def run(self) -> dict:
        """执行一次完整测试运行，返回结果字典。"""
        result = {
            "run_id": self._run_id,
            "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
            "spawn_x": None,
            "spawn_y": None,
            "spawn_yaw_deg": None,
            "orb_x": None,
            "orb_y": None,
            "orb_yaw_deg": None,
            "orb_f1": None,
            "init_error_xy_m": None,
            "init_error_yaw_deg": None,
            "tracking_mean_xy_m": None,
            "tracking_max_xy_m": None,
            "tracking_mean_yaw_deg": None,
            "tracking_max_yaw_deg": None,
            "tracking_samples": 0,
            "orb_success": False,
            "status": "UNKNOWN",
        }

        # ---- Step 1: 清理 ----
        self._run_cleanup(kill=True)

        # ---- Step 2: 删除上次的 CSV，避免读到旧数据 ----
        if POSE_CSV_PATH.exists():
            POSE_CSV_PATH.unlink()

        # ---- Step 3: 启动 ----
        proc, monitor = self._launch()
        self._last_monitor = monitor

        # ---- Step 4: 等待 spawn ----
        if not monitor.spawn_event.wait(timeout=90.0):
            result["status"] = "SPAWN_TIMEOUT"
            self._shutdown(proc)
            self._run_cleanup(kill=True)
            self._dump_debug(monitor, result["status"])
            return result

        result["spawn_x"] = monitor.spawn_x
        result["spawn_y"] = monitor.spawn_y
        result["spawn_yaw_deg"] = monitor.spawn_yaw_deg

        # ---- Step 5: 等待 ORB ----
        if not monitor.orb_event.wait(timeout=self._timeout):
            result["status"] = "ORB_TIMEOUT"
            self._shutdown(proc)
            self._run_cleanup(kill=True)
            self._dump_debug(monitor, result["status"])
            return result

        if monitor.orb_failed:
            result["orb_success"] = False
            result["status"] = f"ORB_FAILED: {monitor.orb_fail_reason}"
            # 即使失败也记录 spawn 信息
            self._shutdown(proc)
            self._run_cleanup(kill=True)
            self._dump_debug(monitor, result["status"])
            return result

        # ORB 成功
        result["orb_success"] = True
        result["orb_x"] = monitor.orb_x
        result["orb_y"] = monitor.orb_y
        result["orb_yaw_deg"] = monitor.orb_yaw_deg
        result["orb_f1"] = monitor.orb_f1

        # 计算初始定位误差
        if all(v is not None for v in [monitor.spawn_x, monitor.spawn_y,
                                        monitor.orb_x, monitor.orb_y]):
            dx = monitor.orb_x - monitor.spawn_x
            dy = monitor.orb_y - monitor.spawn_y
            result["init_error_xy_m"] = math.hypot(dx, dy)

        if all(v is not None for v in [monitor.spawn_yaw_deg, monitor.orb_yaw_deg]):
            result["init_error_yaw_deg"] = wrap_yaw_diff_deg(
                monitor.spawn_yaw_deg, monitor.orb_yaw_deg)

        result["status"] = "OK"

        # ---- Step 6: 等待采集时长 ----
        time.sleep(self._duration)

        # ---- Step 7: 停止 ----
        self._shutdown(proc)

        # ---- Step 8: 读取追踪 CSV ----
        tracking = self._read_tracking_csv()
        result.update(tracking)

        # ---- Step 9: 再次清理 ----
        self._run_cleanup(kill=True)

        return result


# ---------------------------------------------------------------------------
# 输出格式化
# ---------------------------------------------------------------------------
def _fmt(v, fmt_spec=".3f"):
    """安全格式化数值。"""
    if v is None:
        return "N/A"
    return f"{v:{fmt_spec}}"


def print_run_header(run_id: int, total: int, elapsed: float, eta: float):
    """打印单次运行头部信息。"""
    print(f"\n{'='*60}")
    print(f"  Run {run_id}/{total}"
          f"  |  已用时: {elapsed:.0f}s  |  ETA: {eta:.0f}s")
    print(f"{'='*60}")


def print_run_result(result: dict, cumulative_stats: dict):
    """打印单次运行结果 + 累计统计。"""
    if result["orb_success"]:
        print(f"  [spawn]   ({_fmt(result['spawn_x'])}, {_fmt(result['spawn_y'])})"
              f" @ {_fmt(result['spawn_yaw_deg'], '.1f')}°")
        print(f"  [ORB]     ({_fmt(result['orb_x'])}, {_fmt(result['orb_y'])})"
              f" @ {_fmt(result['orb_yaw_deg'], '.1f')}°  F1={_fmt(result['orb_f1'], '.1f')}")
        print(f"  [ERROR]   xy={_fmt(result['init_error_xy_m'])}m  "
              f"yaw={_fmt(result['init_error_yaw_deg'], '.1f')}°")
    else:
        print(f"  [spawn]   ({_fmt(result['spawn_x'])}, {_fmt(result['spawn_y'])})"
              f" @ {_fmt(result['spawn_yaw_deg'], '.1f')}°")
        print(f"  [FAIL]    {result['status']}")

    if result.get("tracking_samples", 0) > 0:
        print(f"  [TRACK]   {result['tracking_samples']} samples  "
              f"mean_xy={_fmt(result['tracking_mean_xy_m'])}m  "
              f"max_xy={_fmt(result['tracking_max_xy_m'])}m")

    # 累计统计
    s = cumulative_stats
    print(f"  {'─'*56}")
    print(f"  Cumulative: {s['done']}/{s['total']}  "
          f"success={s['success_count']}  "
          f"rate={s['success_rate']:.0f}%")
    if s["n_ok"] > 0:
        print(f"  Init Error: mean_xy={_fmt(s['mean_xy'])}m  "
              f"max_xy={_fmt(s['max_xy'])}m  "
              f"mean_yaw={_fmt(s['mean_yaw'], '.1f')}°  "
              f"F1_mean={_fmt(s['mean_f1'], '.1f')}")


def print_final_summary(results: list[dict], output_dir: str, ts: str):
    """打印最终汇总。"""
    ok = [r for r in results if r["orb_success"]]
    n_total = len(results)
    n_ok = len(ok)

    print(f"\n{'='*60}")
    print(f"  测试完成")
    print(f"{'='*60}")
    print(f"  总次数:    {n_total}")
    print(f"  成功:      {n_ok}")
    print(f"  失败:      {n_total - n_ok}")
    print(f"  成功率:    {100 * n_ok / n_total:.1f}%" if n_total > 0 else "")

    if n_ok > 0:
        xy_errs = [r["init_error_xy_m"] for r in ok if r["init_error_xy_m"] is not None]
        yaw_errs = [r["init_error_yaw_deg"] for r in ok if r["init_error_yaw_deg"] is not None]
        f1s = [r["orb_f1"] for r in ok if r["orb_f1"] is not None]

        def stats(vals):
            if not vals:
                return {}
            srt = sorted(vals)
            return {
                "mean": sum(vals) / len(vals),
                "std": (sum((v - sum(vals)/len(vals))**2 for v in vals) / len(vals)) ** 0.5 if len(vals) > 1 else 0.0,
                "min": min(vals),
                "max": max(vals),
                "p50": srt[len(srt) // 2],
                "p95": srt[min(int(len(srt) * 0.95), len(srt) - 1)],
            }

        def print_stat(name, vals, unit, fmt_spec=".3f"):
            s = stats(vals)
            if not s:
                return
            print(f"\n  {name} ({len(vals)} samples):")
            print(f"    mean={s['mean']:{fmt_spec}}{unit}  "
                  f"std={s['std']:{fmt_spec}}{unit}  "
                  f"min={s['min']:{fmt_spec}}{unit}  "
                  f"max={s['max']:{fmt_spec}}{unit}")
            print(f"    p50={s['p50']:{fmt_spec}}{unit}  "
                  f"p95={s['p95']:{fmt_spec}}{unit}")

        print_stat("XY Error", xy_errs, "m")
        print_stat("Yaw Error", yaw_errs, "°", ".1f")
        print_stat("F1 Score", f1s, "", ".1f")

    print(f"\n  结果已保存到: {output_dir}/")


# ---------------------------------------------------------------------------
# CSV 输出
# ---------------------------------------------------------------------------
RESULTS_HEADER = [
    "run_id", "timestamp", "status", "orb_success",
    "spawn_x", "spawn_y", "spawn_yaw_deg",
    "orb_x", "orb_y", "orb_yaw_deg", "orb_f1",
    "init_error_xy_m", "init_error_yaw_deg",
    "tracking_mean_xy_m", "tracking_max_xy_m",
    "tracking_mean_yaw_deg", "tracking_max_yaw_deg",
    "tracking_samples",
]


def write_results_csv(results: list[dict], output_dir: str, ts: str):
    """写入每次运行的详细结果 CSV。"""
    path = os.path.join(output_dir, f"orb_results_{ts}.csv")
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=RESULTS_HEADER, extrasaction="ignore")
        writer.writeheader()
        for r in results:
            writer.writerow(r)
    return path


def write_summary_csv(results: list[dict], output_dir: str, ts: str):
    """写入汇总统计 CSV。"""
    ok = [r for r in results if r["orb_success"]]
    n_total = len(results)
    n_ok = len(ok)

    def stats(vals):
        if not vals:
            return {}
        srt = sorted(vals)
        n = len(vals)
        mean = sum(vals) / n
        return {
            "count": n,
            "mean": mean,
            "std": (sum((v - mean) ** 2 for v in vals) / n) ** 0.5 if n > 1 else 0.0,
            "min": min(vals),
            "max": max(vals),
            "p50": srt[n // 2],
            "p95": srt[min(int(n * 0.95), n - 1)],
        }

    fields = {
        "init_error_xy_m": [r["init_error_xy_m"] for r in ok if r["init_error_xy_m"] is not None],
        "init_error_yaw_deg": [r["init_error_yaw_deg"] for r in ok if r["init_error_yaw_deg"] is not None],
        "orb_f1": [r["orb_f1"] for r in ok if r["orb_f1"] is not None],
        "tracking_mean_xy_m": [r["tracking_mean_xy_m"] for r in ok if r["tracking_mean_xy_m"] is not None],
        "tracking_max_xy_m": [r["tracking_max_xy_m"] for r in ok if r["tracking_max_xy_m"] is not None],
    }

    rows = []
    # 总体信息行
    rows.append({
        "stat": "count_total",
        "init_error_xy_m": n_total,
        "init_error_yaw_deg": n_total,
        "orb_f1": n_total,
        "tracking_mean_xy_m": n_total,
        "tracking_max_xy_m": n_total,
    })
    rows.append({
        "stat": "count_success",
        "init_error_xy_m": n_ok,
        "init_error_yaw_deg": n_ok,
        "orb_f1": n_ok,
        "tracking_mean_xy_m": n_ok,
        "tracking_max_xy_m": n_ok,
    })
    rate = 100.0 * n_ok / n_total if n_total > 0 else 0.0
    rows.append({
        "stat": "success_rate_pct",
        "init_error_xy_m": f"{rate:.1f}",
        "init_error_yaw_deg": f"{rate:.1f}",
        "orb_f1": f"{rate:.1f}",
        "tracking_mean_xy_m": f"{rate:.1f}",
        "tracking_max_xy_m": f"{rate:.1f}",
    })

    for stat_name in ["mean", "std", "min", "max", "p50", "p95"]:
        row = {"stat": stat_name}
        for field, vals in fields.items():
            s = stats(vals)
            if stat_name in s:
                val = s[stat_name]
                if isinstance(val, float):
                    row[field] = f"{val:.4f}"
                else:
                    row[field] = val
            else:
                row[field] = "N/A"
        rows.append(row)

    path = os.path.join(output_dir, f"orb_summary_{ts}.csv")
    header = ["stat"] + list(fields.keys())
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=header, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    return path


# ---------------------------------------------------------------------------
# 累计统计辅助
# ---------------------------------------------------------------------------
def compute_cumulative(results: list[dict], total: int) -> dict:
    """从已有结果计算累计统计。"""
    ok = [r for r in results if r["orb_success"]]
    n_done = len(results)
    n_ok = len(ok)

    stats = {
        "done": n_done,
        "total": total,
        "success_count": n_ok,
        "success_rate": 100.0 * n_ok / n_done if n_done > 0 else 0.0,
        "n_ok": n_ok,
        "mean_xy": None,
        "max_xy": None,
        "mean_yaw": None,
        "mean_f1": None,
    }

    if n_ok > 0:
        xy_errs = [r["init_error_xy_m"] for r in ok if r["init_error_xy_m"] is not None]
        yaw_errs = [r["init_error_yaw_deg"] for r in ok if r["init_error_yaw_deg"] is not None]
        f1s = [r["orb_f1"] for r in ok if r["orb_f1"] is not None]
        if xy_errs:
            stats["mean_xy"] = sum(xy_errs) / len(xy_errs)
            stats["max_xy"] = max(xy_errs)
        if yaw_errs:
            stats["mean_yaw"] = sum(yaw_errs) / len(yaw_errs)
        if f1s:
            stats["mean_f1"] = sum(f1s) / len(f1s)

    return stats


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="ORB 全局定位精度反复测试工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python test_orb_localization.py --runs 100 --duration 15
  python test_orb_localization.py --runs 5 --duration 5 --verbose
  python test_orb_localization.py --dry-run
        """,
    )
    parser.add_argument("--runs", "-n", type=int, default=100,
                        help="测试次数 (default: 100)")
    parser.add_argument("--duration", "-d", type=float, default=15.0,
                        help="ORB 完成后采集数据时长，秒 (default: 15)")
    parser.add_argument("--timeout", "-t", type=float, default=180.0,
                        help="等待 ORB 最长超时，秒 (default: 180)")
    parser.add_argument("--output-dir", "-o", type=str, default=str(RESULTS_DIR),
                        help=f"输出目录 (default: {RESULTS_DIR})")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="打印所有 launch stdout")
    parser.add_argument("--fast-cleanup", action="store_true",
                        help="加速清理，跳过 65s 端口等待")
    parser.add_argument("--dry-run", "--plan", action="store_true",
                        help="打印测试计划并退出")
    args = parser.parse_args()

    # Dry-run
    if args.dry_run:
        print("=== ORB Localization Test Plan ===")
        print(f"  Runs:           {args.runs}")
        print(f"  Duration/run:   {args.duration}s")
        print(f"  ORB timeout:    {args.timeout}s")
        print(f"  Output dir:     {args.output_dir}")
        print(f"  Fast cleanup:   {args.fast_cleanup}")
        print(f"  Verbose:        {args.verbose}")
        est = args.runs * (20 + args.duration + 10)  # rough estimate per run
        print(f"  Est. total:     ~{est:.0f}s ({est/60:.0f} min)")
        print(f"  Launch:         {LAUNCH_PACKAGE} {LAUNCH_FILE}")
        print(f"  Pose CSV:       {POSE_CSV_PATH}")
        return

    # 创建输出目录
    os.makedirs(args.output_dir, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    # 确认环境
    if not os.path.exists(ROS2_SETUP):
        print(f"[ERROR] ROS2 setup not found: {ROS2_SETUP}")
        sys.exit(1)
    if not os.path.exists(PROJECT_SETUP):
        print(f"[ERROR] Project setup not found: {PROJECT_SETUP}")
        sys.exit(1)

    print(f"=== ORB Localization Test ===")
    print(f"  Runs: {args.runs}  |  Duration: {args.duration}s  |  Timeout: {args.timeout}s")
    print(f"  Output: {args.output_dir}/orb_*_{ts}.csv")
    print()

    results: list[dict] = []
    start_time = time.time()

    try:
        for run_id in range(1, args.runs + 1):
            # ETA
            elapsed = time.time() - start_time
            done = run_id - 1
            if done > 0:
                eta = (elapsed / done) * (args.runs - done)
            else:
                eta = 0

            print_run_header(run_id, args.runs, elapsed, eta)

            manager = RunManager(
                run_id=run_id,
                duration=args.duration,
                timeout=args.timeout,
                verbose=args.verbose,
                fast_cleanup=args.fast_cleanup,
                debug_dir=args.output_dir,
            )
            result = manager.run()
            results.append(result)

            cumulative = compute_cumulative(results, args.runs)
            print_run_result(result, cumulative)

            # 增量保存（防止中途崩溃丢数据）
            write_results_csv(results, args.output_dir, ts)

            # 失败时提示调试日志位置
            if not result["orb_success"]:
                print(f"  [DEBUG]   详见: {args.output_dir}/debug_run_{run_id:04d}_*.log")

    except KeyboardInterrupt:
        print("\n\n[INTERRUPTED] 用户中断，保存已有结果...")
    except Exception as e:
        print(f"\n[FATAL] {e}")
        import traceback
        traceback.print_exc()

    # 最终输出
    elapsed_total = time.time() - start_time
    print(f"\n总耗时: {elapsed_total:.0f}s ({elapsed_total/60:.1f} min)")

    if results:
        print_final_summary(results, args.output_dir, ts)

        # 最终保存
        path_r = write_results_csv(results, args.output_dir, ts)
        path_s = write_summary_csv(results, args.output_dir, ts)
        print(f"  详细结果: {path_r}")
        print(f"  汇总统计: {path_s}")
    else:
        print("  无有效结果。")


if __name__ == "__main__":
    main()
