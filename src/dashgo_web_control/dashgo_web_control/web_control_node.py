#!/usr/bin/env python3

import json
import math
import mimetypes
import os
import struct
import threading
import time
import zlib
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class WebControlNode(Node):
    def __init__(self):
        super().__init__("dashgo_web_control")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("map_topic", "/combined_grid")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("path_topic", "/path")
        self.declare_parameter("mode_topic", "/control_mode")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("scan_topic", "/scan_filtered")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("linear_limit", 0.25)
        self.declare_parameter("angular_limit", 1.20)
        self.declare_parameter("cmd_vel_timeout", 0.6)
        self.declare_parameter("map_publish_hz", 1.0)
        self.declare_parameter("camera_publish_hz", 2.0)
        self.declare_parameter("camera_max_width", 240)
        self.declare_parameter("robot_radius", 0.20)
        self.declare_parameter("odom_online_timeout", 1.5)
        self.declare_parameter("scan_online_timeout", 1.5)
        self.declare_parameter("camera_online_timeout", 2.0)

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.mode_topic = str(self.get_parameter("mode_topic").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.linear_limit = float(self.get_parameter("linear_limit").value)
        self.angular_limit = float(self.get_parameter("angular_limit").value)
        self.cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout").value)
        self.map_publish_interval = 1.0 / max(float(self.get_parameter("map_publish_hz").value), 0.2)
        self.camera_publish_interval = 1.0 / max(float(self.get_parameter("camera_publish_hz").value), 0.2)
        self.camera_max_width = max(64, int(self.get_parameter("camera_max_width").value))
        self.robot_radius = float(self.get_parameter("robot_radius").value)
        self.odom_online_timeout = float(self.get_parameter("odom_online_timeout").value)
        self.scan_online_timeout = float(self.get_parameter("scan_online_timeout").value)
        self.camera_online_timeout = float(self.get_parameter("camera_online_timeout").value)

        self.web_dir = os.path.join(get_package_share_directory("dashgo_web_control"), "web")
        self.state_lock = threading.Lock()
        self.map_version = 0
        self.path_version = 0
        self.goal_version = 0
        self.map_snapshot = None
        self.path_snapshot = []
        self.odom_snapshot = None
        self.goal_snapshot = None
        self.control_mode = "nav"
        self.last_cmd_time = 0.0
        self.last_cmd_active = False
        self.last_map_export_time = 0.0
        self.last_camera_export_time = 0.0
        self.last_odom_time = 0.0
        self.last_scan_time = 0.0
        self.camera_frame = None
        self.camera_meta = None
        self.scan_meta = None

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.mode_pub = self.create_publisher(String, self.mode_topic, 10)

        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.create_subscription(Path, self.path_topic, self.path_callback, 10)
        self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data)

        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)

        self.http_server = self.build_http_server()
        self.http_thread = threading.Thread(target=self.http_server.serve_forever, daemon=True)
        self.http_thread.start()

        self.get_logger().info(
            f"Dashgo web control ready at http://{self.host}:{self.port}"
        )

    def destroy_node(self):
        try:
            self.http_server.shutdown()
            self.http_server.server_close()
        except Exception:
            pass
        super().destroy_node()

    def map_callback(self, msg):
        now = time.monotonic()
        if now - self.last_map_export_time < self.map_publish_interval:
            return

        snapshot = {
            "frame_id": msg.header.frame_id,
            "stamp_sec": int(msg.header.stamp.sec),
            "stamp_nanosec": int(msg.header.stamp.nanosec),
            "width": int(msg.info.width),
            "height": int(msg.info.height),
            "resolution": float(msg.info.resolution),
            "origin": {
                "x": float(msg.info.origin.position.x),
                "y": float(msg.info.origin.position.y),
            },
            "data": list(msg.data),
        }

        with self.state_lock:
            self.map_snapshot = snapshot
            self.map_version += 1
            self.last_map_export_time = now

    def odom_callback(self, msg):
        pose = msg.pose.pose
        yaw = quaternion_to_yaw(pose.orientation)
        odom = {
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": float(yaw),
            "linear_x": float(msg.twist.twist.linear.x),
            "angular_z": float(msg.twist.twist.angular.z),
        }
        with self.state_lock:
            self.odom_snapshot = odom
            self.last_odom_time = time.monotonic()

    def path_callback(self, msg):
        sampled = []
        for index, pose_stamped in enumerate(msg.poses):
            if index % 3 != 0 and index != len(msg.poses) - 1:
                continue
            sampled.append(
                {
                    "x": float(pose_stamped.pose.position.x),
                    "y": float(pose_stamped.pose.position.y),
                }
            )
        with self.state_lock:
            self.path_snapshot = sampled
            self.path_version += 1

    def image_callback(self, msg):
        now = time.monotonic()
        if now - self.last_camera_export_time < self.camera_publish_interval:
            return

        try:
            frame_bytes, width, height = self.convert_image_to_png(msg)
        except ValueError as exc:
            self.get_logger().debug(f"Skip image frame: {exc}")
            return

        with self.state_lock:
            self.camera_frame = frame_bytes
            self.camera_meta = {
                "width": width,
                "height": height,
                "encoding": msg.encoding,
            }
            self.last_camera_export_time = now

    def scan_callback(self, msg):
        now = time.monotonic()
        valid_ranges = []
        front_values = []
        points = []
        total_count = len(msg.ranges)
        step = max(1, total_count // 540)

        for index, distance in enumerate(msg.ranges):
            if not (math.isfinite(distance) and msg.range_min <= distance <= msg.range_max):
                continue
            distance = float(distance)
            angle = msg.angle_min + index * msg.angle_increment
            valid_ranges.append(distance)
            if abs(angle) <= math.radians(12.0):
                front_values.append(distance)
            if index % step == 0:
                points.append([
                    distance * math.cos(angle),
                    distance * math.sin(angle),
                ])

        scan_meta = {
            "count": int(total_count),
            "valid_count": int(len(valid_ranges)),
            "nearest_range": min(valid_ranges) if valid_ranges else None,
            "front_range": min(front_values) if front_values else None,
            "range_min": float(msg.range_min),
            "range_max": float(msg.range_max),
            "points": points,
        }
        with self.state_lock:
            self.last_scan_time = now
            self.scan_meta = scan_meta

    def watchdog_callback(self):
        if not self.last_cmd_active:
            return

        if time.monotonic() - self.last_cmd_time <= self.cmd_vel_timeout:
            return

        self.publish_cmd_vel(0.0, 0.0)
        self.last_cmd_active = False

    def publish_cmd_vel(self, linear, angular):
        msg = Twist()
        msg.linear.x = max(-self.linear_limit, min(self.linear_limit, float(linear)))
        msg.angular.z = max(-self.angular_limit, min(self.angular_limit, float(angular)))
        self.cmd_pub.publish(msg)

    def is_manual_mode(self):
        with self.state_lock:
            return self.control_mode == "manual"

    def is_paused_mode(self):
        with self.state_lock:
            return self.control_mode == "pause"

    def stop_robot(self):
        self.publish_cmd_vel(0.0, 0.0)
        self.last_cmd_active = False
        self.last_cmd_time = 0.0

    def publish_goal(self, x, y, yaw):
        self.set_control_mode("nav")
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = math.sin(float(yaw) / 2.0)
        msg.pose.orientation.w = math.cos(float(yaw) / 2.0)
        self.goal_pub.publish(msg)

        with self.state_lock:
            self.goal_snapshot = {"x": float(x), "y": float(y), "yaw": float(yaw)}
            self.goal_version += 1

    def set_control_mode(self, mode):
        clean_mode = mode.strip().lower()
        if clean_mode not in ("manual", "nav", "pause"):
            raise ValueError(f"Unsupported control mode: {mode}")

        msg = String()
        msg.data = clean_mode
        self.mode_pub.publish(msg)

        if clean_mode in ("manual", "pause"):
            self.stop_robot()

        with self.state_lock:
            self.control_mode = clean_mode

    def build_status_payload(self):
        with self.state_lock:
            now = time.monotonic()
            base_online = (now - self.last_odom_time) <= self.odom_online_timeout
            radar_online = (now - self.last_scan_time) <= self.scan_online_timeout
            camera_online = (
                self.camera_frame is not None and
                (now - self.last_camera_export_time) <= self.camera_online_timeout
            )
            return {
                "ok": True,
                "map_version": self.map_version,
                "path_version": self.path_version,
                "goal_version": self.goal_version,
                "has_map": self.map_snapshot is not None,
                "has_odom": self.odom_snapshot is not None,
                "odom": self.odom_snapshot,
                "goal": self.goal_snapshot,
                "control_mode": self.control_mode,
                "has_camera": self.camera_frame is not None,
                "camera": self.camera_meta,
                "devices": {
                    "base": base_online,
                    "radar": radar_online,
                    "camera": camera_online,
                },
                "scan": self.scan_meta,
                "robot_radius": self.robot_radius,
                "limits": {
                    "linear": self.linear_limit,
                    "angular": self.angular_limit,
                },
                "topics": {
                    "cmd_vel": self.cmd_vel_topic,
                    "goal": self.goal_topic,
                    "map": self.map_topic,
                    "odom": self.odom_topic,
                    "path": self.path_topic,
                    "mode": self.mode_topic,
                    "image": self.image_topic,
                    "scan": self.scan_topic,
                },
                "base_frame": self.base_frame,
            }

    def build_map_payload(self):
        with self.state_lock:
            if self.map_snapshot is None:
                return {"ok": False, "message": "map not ready"}
            return {
                "ok": True,
                "version": self.map_version,
                "map": self.map_snapshot,
            }

    def build_path_payload(self):
        with self.state_lock:
            return {
                "ok": True,
                "version": self.path_version,
                "path": self.path_snapshot,
            }

    def build_camera_meta_payload(self):
        with self.state_lock:
            return {
                "ok": self.camera_frame is not None,
                "camera": self.camera_meta,
            }

    def build_camera_frame_payload(self):
        with self.state_lock:
            return self.camera_frame

    def convert_image_to_png(self, msg):
        if msg.encoding not in ("rgb8", "bgr8", "rgba8", "bgra8", "mono8"):
            raise ValueError(f"Unsupported encoding: {msg.encoding}")

        channels = {
            "rgb8": 3,
            "bgr8": 3,
            "rgba8": 4,
            "bgra8": 4,
            "mono8": 1,
        }[msg.encoding]

        if msg.width <= 0 or msg.height <= 0:
            raise ValueError("Empty image")

        step_x = max(1, math.ceil(msg.width / self.camera_max_width))
        out_width = max(1, msg.width // step_x)
        out_height = max(1, msg.height // step_x)
        out_rgb = bytearray(out_width * out_height * 3)
        data = msg.data

        for out_y, src_y in enumerate(range(0, msg.height, step_x)):
            if out_y >= out_height:
                break
            row_base = src_y * msg.step
            for out_x, src_x in enumerate(range(0, msg.width, step_x)):
                if out_x >= out_width:
                    break
                src_index = row_base + src_x * channels
                dst_index = (out_y * out_width + out_x) * 3

                if msg.encoding == "rgb8":
                    r = data[src_index]
                    g = data[src_index + 1]
                    b = data[src_index + 2]
                elif msg.encoding == "bgr8":
                    b = data[src_index]
                    g = data[src_index + 1]
                    r = data[src_index + 2]
                elif msg.encoding == "rgba8":
                    r = data[src_index]
                    g = data[src_index + 1]
                    b = data[src_index + 2]
                elif msg.encoding == "bgra8":
                    b = data[src_index]
                    g = data[src_index + 1]
                    r = data[src_index + 2]
                else:
                    gray = data[src_index]
                    r = gray
                    g = gray
                    b = gray

                out_rgb[dst_index] = r
                out_rgb[dst_index + 1] = g
                out_rgb[dst_index + 2] = b

        return self.rgb_to_png_bytes(out_rgb, out_width, out_height), out_width, out_height

    def png_chunk(self, chunk_type, data):
        crc = zlib.crc32(chunk_type)
        crc = zlib.crc32(data, crc) & 0xFFFFFFFF
        return (
            struct.pack(">I", len(data)) +
            chunk_type +
            data +
            struct.pack(">I", crc)
        )

    def rgb_to_png_bytes(self, rgb_bytes, width, height):
        raw = bytearray()
        row_stride = width * 3
        for row in range(height):
            raw.append(0)
            row_start = row * row_stride
            raw.extend(rgb_bytes[row_start:row_start + row_stride])

        compressed = zlib.compress(bytes(raw), level=6)
        header = b"\x89PNG\r\n\x1a\n"
        ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
        return b"".join([
            header,
            self.png_chunk(b"IHDR", ihdr),
            self.png_chunk(b"IDAT", compressed),
            self.png_chunk(b"IEND", b""),
        ])

    def read_static_file(self, rel_path):
        requested = rel_path.lstrip("/") or "index.html"
        if requested == "":
            requested = "index.html"
        requested = os.path.normpath(requested)
        abs_path = os.path.realpath(os.path.join(self.web_dir, requested))
        if not abs_path.startswith(os.path.realpath(self.web_dir) + os.sep):
            if abs_path != os.path.realpath(os.path.join(self.web_dir, "index.html")):
                return None, None
        if not abs_path.startswith(os.path.realpath(self.web_dir)):
            return None, None
        if not os.path.isfile(abs_path):
            return None, None
        content_type, _ = mimetypes.guess_type(abs_path)
        if content_type is None:
            content_type = "application/octet-stream"
        with open(abs_path, "rb") as f:
            return f.read(), content_type

    def build_http_server(self):
        node = self

        class RequestHandler(BaseHTTPRequestHandler):
            def log_message(self, format_string, *args):
                node.get_logger().debug(format_string % args)

            def send_json(self, payload, status=HTTPStatus.OK):
                data = json.dumps(payload).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def read_json_body(self):
                content_length = int(self.headers.get("Content-Length", "0"))
                if content_length <= 0:
                    return {}
                raw = self.rfile.read(content_length)
                if not raw:
                    return {}
                return json.loads(raw.decode("utf-8"))

            def do_GET(self):
                parsed = urlparse(self.path)
                if parsed.path == "/api/status":
                    self.send_json(node.build_status_payload())
                    return
                if parsed.path == "/api/map":
                    self.send_json(node.build_map_payload())
                    return
                if parsed.path == "/api/path":
                    self.send_json(node.build_path_payload())
                    return
                if parsed.path == "/api/camera/meta":
                    self.send_json(node.build_camera_meta_payload())
                    return
                if parsed.path in ("/api/camera/frame.bmp", "/api/camera/frame.png"):
                    frame = node.build_camera_frame_payload()
                    if frame is None:
                        self.send_json(
                            {"ok": False, "message": "camera not ready"},
                            status=HTTPStatus.NOT_FOUND,
                        )
                        return
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", "image/png")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(frame)))
                    self.end_headers()
                    self.wfile.write(frame)
                    return

                rel_path = parsed.path
                if rel_path == "/":
                    rel_path = "/index.html"
                content, content_type = node.read_static_file(rel_path)
                if content is None:
                    self.send_json(
                        {"ok": False, "message": f"Not found: {parsed.path}"},
                        status=HTTPStatus.NOT_FOUND,
                    )
                    return

                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", content_type)
                self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
                self.send_header("Pragma", "no-cache")
                self.send_header("Expires", "0")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)

            def do_POST(self):
                parsed = urlparse(self.path)
                try:
                    payload = self.read_json_body()
                except Exception as exc:
                    self.send_json(
                        {"ok": False, "message": f"Invalid JSON: {exc}"},
                        status=HTTPStatus.BAD_REQUEST,
                    )
                    return

                if parsed.path == "/api/cmd_vel":
                    if not node.is_manual_mode():
                        self.send_json(
                            {"ok": False, "message": "manual mode required"},
                            status=HTTPStatus.CONFLICT,
                        )
                        return
                    linear = float(payload.get("linear", 0.0))
                    angular = float(payload.get("angular", 0.0))
                    node.publish_cmd_vel(linear, angular)
                    node.last_cmd_time = time.monotonic()
                    node.last_cmd_active = abs(linear) > 1e-5 or abs(angular) > 1e-5
                    self.send_json({"ok": True})
                    return

                if parsed.path == "/api/stop":
                    next_mode = "nav" if node.is_paused_mode() else "pause"
                    node.set_control_mode(next_mode)
                    node.stop_robot()
                    self.send_json({"ok": True, "mode": next_mode})
                    return

                if parsed.path == "/api/goal":
                    x = float(payload["x"])
                    y = float(payload["y"])
                    yaw = float(payload.get("yaw", 0.0))
                    node.publish_goal(x, y, yaw)
                    self.send_json({"ok": True})
                    return

                if parsed.path == "/api/mode":
                    mode = str(payload.get("mode", "nav"))
                    node.set_control_mode(mode)
                    self.send_json({"ok": True, "mode": mode})
                    return

                self.send_json(
                    {"ok": False, "message": f"Unsupported path: {parsed.path}"},
                    status=HTTPStatus.NOT_FOUND,
                )

        return ThreadingHTTPServer((self.host, self.port), RequestHandler)


def main(args=None):
    rclpy.init(args=args)
    node = WebControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
