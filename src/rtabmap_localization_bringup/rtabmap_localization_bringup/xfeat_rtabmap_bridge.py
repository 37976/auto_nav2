#!/usr/bin/env python3

import math
import os
import sys
from typing import Optional

import cv2
import message_filters
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from rtabmap_msgs.msg import KeyPoint, Point2f, Point3f, RGBDImage
from rtabmap_python.compression import compress
from sensor_msgs.msg import CameraInfo, Image


def _resolve_xfeat_repo_dir(configured_dir: str) -> str:
    candidates = []
    if configured_dir:
        candidates.append(configured_dir)

    workspace_candidate = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "XFeat")
    )
    candidates.append(workspace_candidate)
    candidates.append("/home/xu/project/XFeat")

    for candidate in candidates:
        if os.path.isfile(os.path.join(candidate, "modules", "xfeat.py")):
            return candidate

    raise FileNotFoundError(
        "Cannot locate XFeat repo. Set parameter 'xfeat_repo_dir' to the directory containing "
        "'modules/xfeat.py'."
    )


def _image_to_rgb(image: np.ndarray) -> np.ndarray:
    if image.ndim == 2:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    if image.ndim != 3:
        raise ValueError(f"Unsupported image shape: {image.shape}")
    if image.shape[2] == 3:
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    if image.shape[2] == 4:
        return cv2.cvtColor(image, cv2.COLOR_BGRA2RGB)
    raise ValueError(f"Unsupported channel count: {image.shape[2]}")


def _torch_image_from_cv(image: np.ndarray) -> torch.Tensor:
    rgb = _image_to_rgb(image)
    tensor = torch.from_numpy(np.ascontiguousarray(rgb)).to(dtype=torch.float32) / 255.0
    return tensor.permute(2, 0, 1).unsqueeze(0)


class XFeatRtabmapBridge(Node):
    def __init__(self) -> None:
        super().__init__("xfeat_rtabmap_bridge")

        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("output_rgbd_topic", "/xfeat/rgbd_image")
        self.declare_parameter("xfeat_repo_dir", "")
        self.declare_parameter("xfeat_weights_path", "")
        self.declare_parameter("top_k", 400)
        self.declare_parameter("detection_threshold", 0.05)
        self.declare_parameter("min_score", 0.0)
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("depth_min_m", 0.15)
        self.declare_parameter("depth_max_m", 5.0)
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("output_rate_hz", 1.0)

        self._rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
        self._depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        self._camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self._output_rgbd_topic = self.get_parameter("output_rgbd_topic").get_parameter_value().string_value
        self._top_k = int(self.get_parameter("top_k").value)
        self._detection_threshold = float(self.get_parameter("detection_threshold").value)
        self._min_score = float(self.get_parameter("min_score").value)
        self._depth_scale = float(self.get_parameter("depth_scale").value)
        self._depth_min_m = float(self.get_parameter("depth_min_m").value)
        self._depth_max_m = float(self.get_parameter("depth_max_m").value)
        self._output_period_sec = 1.0 / max(0.01, float(self.get_parameter("output_rate_hz").value))
        self._last_publish_time_sec: Optional[float] = None
        self._published_frames = 0
        self._last_camera_info: Optional[CameraInfo] = None
        self._bridge = CvBridge()

        configured_repo_dir = self.get_parameter("xfeat_repo_dir").get_parameter_value().string_value
        self._xfeat_repo_dir = _resolve_xfeat_repo_dir(configured_repo_dir)
        if self._xfeat_repo_dir not in sys.path:
            sys.path.insert(0, self._xfeat_repo_dir)

        from modules.xfeat import XFeat  # pylint: disable=import-outside-toplevel

        configured_weights_path = self.get_parameter("xfeat_weights_path").get_parameter_value().string_value
        weights_path = configured_weights_path or os.path.join(self._xfeat_repo_dir, "weights", "xfeat.pt")
        self._xfeat = XFeat(
            weights=weights_path,
            top_k=self._top_k,
            detection_threshold=self._detection_threshold,
        )

        self._publisher = self.create_publisher(RGBDImage, self._output_rgbd_topic, 10)
        self._rgb_sub = message_filters.Subscriber(self, Image, self._rgb_topic)
        self._depth_sub = message_filters.Subscriber(self, Image, self._depth_topic)
        self._camera_info_sub = self.create_subscription(
            CameraInfo, self._camera_info_topic, self._camera_info_callback, 10
        )
        sync_queue_size = int(self.get_parameter("sync_queue_size").value)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._rgb_sub, self._depth_sub],
            queue_size=sync_queue_size,
            slop=0.1,
        )
        self._sync.registerCallback(self._rgbd_callback)

        self.get_logger().info(f"XFeat repo: {self._xfeat_repo_dir}")
        self.get_logger().info(f"XFeat weights: {weights_path}")
        self.get_logger().info(f"Output RGBDImage topic: {self._output_rgbd_topic}")

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self._last_camera_info = msg

    def _rgbd_callback(self, rgb_msg: Image, depth_msg: Image) -> None:
        if self._last_camera_info is None:
            self.get_logger().warn(
                "Waiting for camera_info before publishing XFeat RGBDImage.",
                throttle_duration_sec=2.0,
            )
            return

        current_stamp_sec = float(rgb_msg.header.stamp.sec) + float(rgb_msg.header.stamp.nanosec) * 1e-9
        if (
            self._last_publish_time_sec is not None
            and current_stamp_sec - self._last_publish_time_sec < self._output_period_sec
        ):
            return

        try:
            rgb_image = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth_image = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
            return

        if rgb_image is None or depth_image is None or rgb_image.size == 0 or depth_image.size == 0:
            return

        try:
            features = self._xfeat.detectAndCompute(
                _torch_image_from_cv(rgb_image),
                top_k=self._top_k,
                detection_threshold=self._detection_threshold,
            )[0]
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"XFeat inference failed: {exc}")
            return

        keypoints = features["keypoints"].detach().cpu().numpy().astype(np.float32, copy=False)
        scores = features["scores"].detach().cpu().numpy().astype(np.float32, copy=False)
        descriptors = features["descriptors"].detach().cpu().numpy().astype(np.float32, copy=False)

        if keypoints.shape[0] == 0 or descriptors.shape[0] == 0:
            self.get_logger().warn(
                "XFeat produced no valid features for current frame.",
                throttle_duration_sec=2.0,
            )
            return

        if self._min_score > 0.0:
            valid_scores = scores >= self._min_score
            keypoints = keypoints[valid_scores]
            scores = scores[valid_scores]
            descriptors = descriptors[valid_scores]

        if keypoints.shape[0] == 0 or descriptors.shape[0] == 0:
            self.get_logger().warn(
                "XFeat features were filtered out by min_score.",
                throttle_duration_sec=2.0,
            )
            return

        filtered_indices = []
        filtered_points = []
        fx = float(self._last_camera_info.k[0])
        fy = float(self._last_camera_info.k[4])
        cx = float(self._last_camera_info.k[2])
        cy = float(self._last_camera_info.k[5])

        for index, (x, y) in enumerate(keypoints):
            u = int(round(float(x)))
            v = int(round(float(y)))
            depth_m = self._read_depth_meters(depth_image, depth_msg.encoding, u, v)
            if depth_m is None:
                continue

            point = Point3f()
            point.x = float((u - cx) * depth_m / fx)
            point.y = float((v - cy) * depth_m / fy)
            point.z = float(depth_m)
            filtered_indices.append(index)
            filtered_points.append(point)

        if not filtered_indices:
            self.get_logger().warn(
                "XFeat found features, but none had valid aligned depth.",
                throttle_duration_sec=2.0,
            )
            return

        keypoints = keypoints[filtered_indices]
        scores = scores[filtered_indices]
        descriptors = np.ascontiguousarray(descriptors[filtered_indices], dtype=np.float32)

        msg = RGBDImage()
        msg.header = rgb_msg.header
        msg.rgb_camera_info = self._last_camera_info
        msg.depth_camera_info = self._last_camera_info
        msg.rgb = rgb_msg
        msg.depth = depth_msg
        msg.key_points = [
            self._to_keypoint_msg(x, y, score, idx)
            for idx, ((x, y), score) in enumerate(zip(keypoints, scores))
        ]
        msg.points = filtered_points
        msg.descriptors = list(compress(descriptors))

        self._publisher.publish(msg)
        self._last_publish_time_sec = current_stamp_sec
        self._published_frames += 1
        self.get_logger().info(
            f"Published XFeat RGBDImage frame #{self._published_frames} with {len(filtered_indices)} valid features.",
            throttle_duration_sec=2.0,
        )

    def _read_depth_meters(self, depth_image: np.ndarray, encoding: str, u: int, v: int) -> Optional[float]:
        if u < 0 or v < 0 or v >= depth_image.shape[0] or u >= depth_image.shape[1]:
            return None

        if encoding == "16UC1":
            raw_depth = float(depth_image[v, u]) * self._depth_scale
        elif encoding == "32FC1":
            raw_depth = float(depth_image[v, u])
        else:
            self.get_logger().error(f"Unsupported depth encoding: {encoding}")
            return None

        if not math.isfinite(raw_depth) or raw_depth < self._depth_min_m or raw_depth > self._depth_max_m:
            return None
        return raw_depth

    @staticmethod
    def _to_keypoint_msg(x: float, y: float, score: float, class_id: int) -> KeyPoint:
        point = Point2f()
        point.x = float(x)
        point.y = float(y)

        keypoint = KeyPoint()
        keypoint.pt = point
        keypoint.size = 1.0
        keypoint.angle = -1.0
        keypoint.response = float(score)
        keypoint.octave = 0
        keypoint.class_id = int(class_id)
        return keypoint


def main(args=None) -> None:
    rclpy.init(args=args)
    node = XFeatRtabmapBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
