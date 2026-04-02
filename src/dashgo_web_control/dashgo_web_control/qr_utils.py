#!/usr/bin/env python3

from dashgo_web_control._vendor_qrcodegen import QrCode


def make_qr_matrix(text):
    qr = QrCode.encode_text(text, QrCode.Ecc.MEDIUM)
    size = qr.get_size()
    return [[qr.get_module(x, y) for x in range(size)] for y in range(size)]


def escape_wifi_field(value):
    return (
        value.replace("\\", "\\\\")
        .replace(";", r"\;")
        .replace(",", r"\,")
        .replace(":", r"\:")
        .replace('"', r"\"")
    )


def build_wifi_qr_text(ssid, password, security="WPA"):
    escaped_ssid = escape_wifi_field(ssid)
    escaped_password = escape_wifi_field(password)
    return f"WIFI:T:{security};S:{escaped_ssid};P:{escaped_password};;"
