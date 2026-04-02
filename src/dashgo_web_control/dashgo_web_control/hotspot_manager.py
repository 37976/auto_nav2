#!/usr/bin/env python3

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def run_command(args, check=True):
    result = subprocess.run(args, check=False, capture_output=True, text=True)
    if check and result.returncode != 0:
        stderr = result.stderr.strip() or result.stdout.strip() or "unknown error"
        raise RuntimeError(f"{' '.join(args)} failed: {stderr}")
    return result


def list_wifi_devices():
    result = run_command(
        ["nmcli", "-t", "-f", "DEVICE,TYPE,STATE", "device", "status"],
    )
    devices = []
    for line in result.stdout.splitlines():
        parts = line.strip().split(":")
        if len(parts) < 3:
            continue
        device, device_type, state = parts[0], parts[1], parts[2]
        if device_type != "wifi":
            continue
        devices.append((device, state))
    return devices


def set_wifi_radio_on():
    run_command(["nmcli", "radio", "wifi", "on"])


def wait_for_wifi_device(ifname, timeout_sec=8.0):
    deadline = time.monotonic() + timeout_sec
    last_state = "unknown"
    while time.monotonic() < deadline:
        for device, state in list_wifi_devices():
            if device != ifname:
                continue
            last_state = state
            if state not in {"unavailable", "unmanaged"}:
                return
        time.sleep(0.4)
    raise RuntimeError(
        f"Wi-Fi interface {ifname} is still {last_state} after enabling radio."
    )


def choose_wifi_device(preferred):
    devices = list_wifi_devices()
    if preferred:
        for device, _state in devices:
            if device == preferred:
                return device
        raise RuntimeError(f"Requested hotspot interface {preferred} was not found.")

    for device, state in devices:
        if state not in {"unavailable", "unmanaged"}:
            return device

    if devices:
        return devices[0][0]

    raise RuntimeError("No Wi-Fi interface was found for hotspot mode.")


def connection_exists(name):
    result = run_command(["nmcli", "-t", "-f", "NAME", "connection", "show"])
    return name in {line.strip() for line in result.stdout.splitlines() if line.strip()}


def ensure_hotspot_connection(connection_name, ifname, ssid, password):
    if len(password) < 8:
        raise RuntimeError("Hotspot password must be at least 8 characters.")

    if not connection_exists(connection_name):
        run_command(
            [
                "nmcli",
                "connection",
                "add",
                "type",
                "wifi",
                "ifname",
                ifname,
                "con-name",
                connection_name,
                "ssid",
                ssid,
            ]
        )

    run_command(
        [
            "nmcli",
            "connection",
            "modify",
            connection_name,
            "connection.interface-name",
            ifname,
            "connection.autoconnect",
            "yes",
            "802-11-wireless.mode",
            "ap",
            "802-11-wireless.band",
            "bg",
            "802-11-wireless.ssid",
            ssid,
            "ipv4.method",
            "shared",
            "ipv6.method",
            "ignore",
            "wifi-sec.key-mgmt",
            "wpa-psk",
            "wifi-sec.psk",
            password,
        ]
    )

    run_command(["nmcli", "connection", "up", connection_name])
def get_ipv4_address(ifname):
    result = run_command(
        ["ip", "-4", "-o", "addr", "show", "dev", ifname],
        check=False,
    )
    for line in result.stdout.splitlines():
        parts = line.split()
        if "inet" not in parts:
            continue
        inet_index = parts.index("inet")
        if inet_index + 1 >= len(parts):
            continue
        return parts[inet_index + 1].split("/")[0]
    return ""


def main():
    parser = argparse.ArgumentParser(description="Start a fixed Dashgo hotspot via NetworkManager.")
    parser.add_argument("--connection-name", default="dashgo-hotspot")
    parser.add_argument("--ssid", default="Dashgo-Robot")
    parser.add_argument("--password", default="dashgo12345")
    parser.add_argument("--ifname", default="")
    args = parser.parse_args()

    try:
        set_wifi_radio_on()
        ifname = choose_wifi_device(args.ifname)
        wait_for_wifi_device(ifname)
        ensure_hotspot_connection(args.connection_name, ifname, args.ssid, args.password)
        address = get_ipv4_address(ifname)
        print("")
        print("=" * 60)
        print("Dashgo hotspot ready")
        print(f"interface: {ifname}")
        print(f"ssid:      {args.ssid}")
        print(f"password:  {args.password}")
        if address:
            print(f"ip:        {address}")
        print("=" * 60)
        print("")
    except Exception as exc:
        print(f"Failed to start Dashgo hotspot: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
