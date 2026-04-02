#!/usr/bin/env python3

import os
import re
import subprocess
import time
import urllib.error
import urllib.request

from dashgo_web_control.qr_utils import make_qr_matrix


def list_global_ipv4_addresses():
    try:
        result = subprocess.run(
            ["ip", "-4", "-o", "addr", "show", "up", "scope", "global"],
            check=False,
            capture_output=True,
            text=True,
        )
    except Exception:
        result = None

    candidates = []
    if result and result.stdout:
        pattern = re.compile(r"^\d+:\s+([^ ]+)\s+inet\s+(\d+\.\d+\.\d+\.\d+)/")
        for line in result.stdout.splitlines():
            match = pattern.match(line.strip())
            if not match:
                continue
            if_name, ip_addr = match.groups()
            if ip_addr.startswith("127."):
                continue
            candidates.append((if_name, ip_addr))
    return candidates


def detect_best_host(host, require_wifi=False):
    if host and host not in ("0.0.0.0", "::", ""):
        return host

    env_host = os.environ.get("DASHGO_WEB_UI_IP")
    if env_host:
        return env_host

    candidates = []
    for if_name, ip_addr in list_global_ipv4_addresses():
        score = interface_priority(if_name)
        if require_wifi and score != 0:
            continue
        candidates.append((score, if_name, ip_addr))

    if candidates:
        candidates.sort(key=lambda item: (item[0], item[1]))
        return candidates[0][2]

    return "127.0.0.1"


def interface_priority(if_name):
    if if_name.startswith(("wl", "wlan", "ap")):
        return 0
    if if_name.startswith(("en", "eth", "usb")):
        return 1
    return 2


def build_web_url(host, port):
    resolved_host = detect_best_host(host)
    return f"http://{resolved_host}:{port}"


def wait_for_best_host(host, require_wifi=False, timeout_sec=20.0):
    if host and host not in ("0.0.0.0", "::", ""):
        return host

    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        resolved_host = detect_best_host(host, require_wifi=require_wifi)
        if resolved_host != "127.0.0.1":
            return resolved_host
        time.sleep(0.3)

    return detect_best_host(host, require_wifi=require_wifi)


def wait_for_http_ready(host, port, timeout_sec=20.0):
    deadline = time.monotonic() + timeout_sec
    last_error = None

    while time.monotonic() < deadline:
        try:
            with urllib.request.urlopen(
                f"http://{host}:{port}/api/status",
                timeout=1.5,
            ) as response:
                if 200 <= response.status < 300:
                    return
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            last_error = exc
            time.sleep(0.3)

    if last_error:
        raise RuntimeError(f"Web UI is not ready yet: {last_error}")


def wait_for_web_url(host, port, hotspot_enabled=False, timeout_sec=20.0):
    resolved_host = wait_for_best_host(
        host,
        require_wifi=hotspot_enabled,
        timeout_sec=timeout_sec,
    )
    wait_for_http_ready(resolved_host, port, timeout_sec=timeout_sec)

    if hotspot_enabled:
        time.sleep(1.0)

    return f"http://{resolved_host}:{port}"


def make_matrix(url):
    return make_qr_matrix(url)


def render_terminal_qr(matrix):
    quiet_zone = 4
    scale_x = 4
    scale_y = 2
    full_width = len(matrix[0]) + quiet_zone * 2
    white_row = " " * (full_width * scale_x)
    lines = []

    for _ in range(quiet_zone * scale_y):
        lines.append(white_row)

    for row in matrix:
        line = [" " * (quiet_zone * scale_x)]
        for module in row:
            line.append(("█" * scale_x) if module else (" " * scale_x))
        line.append(" " * (quiet_zone * scale_x))
        rendered = "".join(line)
        for _ in range(scale_y):
            lines.append(rendered)

    for _ in range(quiet_zone * scale_y):
        lines.append(white_row)

    return "\n".join(lines)


def print_web_qr(host, port, hotspot_enabled=False):
    try:
        url = wait_for_web_url(host, port, hotspot_enabled=hotspot_enabled)
    except RuntimeError as exc:
        print(f"Web UI readiness check timed out, falling back to current address guess: {exc}")
        url = build_web_url(host, port)
    matrix = make_matrix(url)
    print("")
    print("=" * 60)
    print(f"Dashgo 手机网页地址: {url}")
    print("手机连上同一个热点后，直接扫描下面二维码即可打开：")
    print(render_terminal_qr(matrix))
    print("=" * 60)
    print("")


def main():
    host = os.environ.get("DASHGO_WEB_UI_HOST", "0.0.0.0")
    port = os.environ.get("DASHGO_WEB_UI_PORT", "8080")
    hotspot_enabled = os.environ.get("DASHGO_HOTSPOT_ENABLED", "").lower() in {"1", "true", "yes"}
    print_web_qr(host, port, hotspot_enabled=hotspot_enabled)


if __name__ == "__main__":
    main()
