#!/usr/bin/env python3

import os
import sys
import tkinter as tk
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dashgo_web_control.qr_utils import build_wifi_qr_text, make_qr_matrix
from dashgo_web_control.show_web_qr import build_web_url, make_matrix, print_web_qr, wait_for_web_url


def draw_qr(canvas, matrix, scale=7, quiet_zone=4):
    canvas_size = (len(matrix) + quiet_zone * 2) * scale
    canvas.configure(width=canvas_size, height=canvas_size)
    for y, row in enumerate(matrix):
        for x, module in enumerate(row):
            if not module:
                continue
            left = (x + quiet_zone) * scale
            top = (y + quiet_zone) * scale
            canvas.create_rectangle(
                left,
                top,
                left + scale,
                top + scale,
                fill="black",
                outline="black",
            )


def make_qr_block(parent, title, subtitle, matrix):
    block = tk.Frame(parent, bg="white", highlightthickness=1, highlightbackground="#dce7f2")
    block.pack(side="left", padx=10, pady=6, fill="both", expand=True)

    title_label = tk.Label(
        block,
        text=title,
        bg="white",
        fg="#10263f",
        font=("DejaVu Sans", 13, "bold"),
        pady=10,
    )
    title_label.pack()

    subtitle_label = tk.Label(
        block,
        text=subtitle,
        bg="white",
        fg="#0f76d6",
        font=("DejaVu Sans Mono", 11),
        justify="center",
        padx=10,
        pady=2,
    )
    subtitle_label.pack()

    canvas = tk.Canvas(
        block,
        bg="white",
        highlightthickness=0,
        bd=0,
    )
    canvas.pack(padx=12, pady=12)
    draw_qr(canvas, matrix)


def show_popup(host, port, hotspot_enabled=False, hotspot_ssid="", hotspot_password=""):
    try:
        url = wait_for_web_url(host, port, hotspot_enabled=hotspot_enabled)
    except RuntimeError as exc:
        print(f"Web UI readiness check timed out, falling back to current address guess: {exc}")
        url = build_web_url(host, port)
    web_matrix = make_matrix(url)

    root = tk.Tk()
    root.title("Dashgo 手机网页二维码")
    root.configure(bg="white")
    root.attributes("-topmost", True)
    root.resizable(False, False)

    title = tk.Label(
        root,
        text="手机连接机器人网络后，扫描二维码打开网页",
        bg="white",
        fg="#10263f",
        font=("DejaVu Sans", 14, "bold"),
        padx=18,
        pady=12,
    )
    title.pack()

    url_label = tk.Label(
        root,
        text=url,
        bg="white",
        fg="#0f76d6",
        font=("DejaVu Sans Mono", 13),
        padx=18,
        pady=4,
    )
    url_label.pack()

    qr_row = tk.Frame(root, bg="white")
    qr_row.pack(padx=12, pady=10, fill="both", expand=True)

    if hotspot_enabled and hotspot_ssid and hotspot_password:
        wifi_text = build_wifi_qr_text(hotspot_ssid, hotspot_password)
        wifi_subtitle = f"Wi-Fi: {hotspot_ssid}\nPassword: {hotspot_password}"
        make_qr_block(qr_row, "1. 连接热点", wifi_subtitle, make_qr_matrix(wifi_text))

    make_qr_block(qr_row, "2. 打开网页", url, web_matrix)

    close_button = tk.Button(
        root,
        text="关闭",
        command=root.destroy,
        bg="#0f76d6",
        fg="white",
        activebackground="#0c66b9",
        activeforeground="white",
        relief="flat",
        padx=18,
        pady=8,
        font=("DejaVu Sans", 12, "bold"),
    )
    close_button.pack(pady=(0, 18))

    root.update_idletasks()
    width = root.winfo_width()
    height = root.winfo_height()
    x = max((root.winfo_screenwidth() - width) // 2, 0)
    y = max((root.winfo_screenheight() - height) // 5, 0)
    root.geometry(f"{width}x{height}+{x}+{y}")
    root.mainloop()


def main():
    host = os.environ.get("DASHGO_WEB_UI_HOST", "0.0.0.0")
    port = os.environ.get("DASHGO_WEB_UI_PORT", "8080")
    hotspot_enabled = os.environ.get("DASHGO_HOTSPOT_ENABLED", "").lower() in {"1", "true", "yes"}
    hotspot_ssid = os.environ.get("DASHGO_HOTSPOT_SSID", "")
    hotspot_password = os.environ.get("DASHGO_HOTSPOT_PASSWORD", "")

    if not os.environ.get("DISPLAY"):
        print_web_qr(host, port, hotspot_enabled=hotspot_enabled)
        return

    try:
        show_popup(host, port, hotspot_enabled, hotspot_ssid, hotspot_password)
    except tk.TclError as exc:
        print(f"QR popup failed, falling back to terminal output: {exc}")
        print_web_qr(host, port, hotspot_enabled=hotspot_enabled)


if __name__ == "__main__":
    main()
