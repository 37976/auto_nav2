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
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


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


def _frame_to_tensor(frame_bgr: np.ndarray) -> torch.Tensor:
    rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    tensor = torch.from_numpy(np.ascontiguousarray(rgb)).to(dtype=torch.float32) / 255.0
    return tensor.permute(2, 0, 1).unsqueeze(0)


def _pose_matrix_from_rt(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float32)
    transform[:3, :3] = rotation.astype(np.float32)
    transform[:3, 3] = translation.reshape(3).astype(np.float32)
    return transform


def _invert_pose(transform: np.ndarray) -> np.ndarray:
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inv = np.eye(4, dtype=np.float32)
    inv[:3, :3] = rotation.T
    inv[:3, 3] = -rotation.T @ translation
    return inv


def _rotation_matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rotation[2, 1] - rotation[1, 2]) / s
        qy = (rotation[0, 2] - rotation[2, 0]) / s
        qz = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        qw = (rotation[2, 1] - rotation[1, 2]) / s
        qx = 0.25 * s
        qy = (rotation[0, 1] + rotation[1, 0]) / s
        qz = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        qw = (rotation[0, 2] - rotation[2, 0]) / s
        qx = (rotation[0, 1] + rotation[1, 0]) / s
        qy = 0.25 * s
        qz = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        qw = (rotation[1, 0] - rotation[0, 1]) / s
        qx = (rotation[0, 2] + rotation[2, 0]) / s
        qy = (rotation[1, 2] + rotation[2, 1]) / s
        qz = 0.25 * s
    quat = np.array([qx, qy, qz, qw], dtype=np.float64)
    norm = np.linalg.norm(quat)
    if norm > 0.0:
        quat /= norm
    return quat


def _transform_to_matrix(translation: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = quaternion
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    rotation = np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float32,
    )
    transform = np.eye(4, dtype=np.float32)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation.astype(np.float32)
    return transform


class XFeatRgbdOdometry(Node):
    def __init__(self) -> None:
        super().__init__("xfeat_rgbd_odometry")

        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("xfeat_repo_dir", "")
        self.declare_parameter("xfeat_weights_path", "")
        self.declare_parameter("top_k", 256)
        self.declare_parameter("detection_threshold", 0.05)
        self.declare_parameter("min_score", 0.08)
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("min_depth_m", 0.2)
        self.declare_parameter("max_depth_m", 3.0)
        self.declare_parameter("match_min_cossim", 0.82)
        self.declare_parameter("min_pnp_points", 12)
        self.declare_parameter("min_inliers", 10)
        self.declare_parameter("pnp_reproj_error", 4.0)
        self.declare_parameter("pnp_iterations", 200)
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("odom_topic", "/xfeat/odom")
        self.declare_parameter("odom_frame", "xfeat_odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("publish_tf", True)

        self._rgb_topic = str(self.get_parameter("rgb_topic").value)
        self._depth_topic = str(self.get_parameter("depth_topic").value)
        self._camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self._top_k = int(self.get_parameter("top_k").value)
        self._detection_threshold = float(self.get_parameter("detection_threshold").value)
        self._min_score = float(self.get_parameter("min_score").value)
        self._depth_scale = float(self.get_parameter("depth_scale").value)
        self._min_depth_m = float(self.get_parameter("min_depth_m").value)
        self._max_depth_m = float(self.get_parameter("max_depth_m").value)
        self._match_min_cossim = float(self.get_parameter("match_min_cossim").value)
        self._min_pnp_points = int(self.get_parameter("min_pnp_points").value)
        self._min_inliers = int(self.get_parameter("min_inliers").value)
        self._pnp_reproj_error = float(self.get_parameter("pnp_reproj_error").value)
        self._pnp_iterations = int(self.get_parameter("pnp_iterations").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._camera_frame = str(self.get_parameter("camera_frame").value)
        self._publish_tf = bool(self.get_parameter("publish_tf").value)

        self._bridge = CvBridge()
        self._camera_info: Optional[CameraInfo] = None
        self._prev_obs = None
        self._pose_world_cam = np.eye(4, dtype=np.float32)
        self._last_stamp_sec: Optional[float] = None
        self._last_translation = np.zeros((3,), dtype=np.float64)
        self._last_logged_translation = np.zeros((3,), dtype=np.float64)
        self._base_to_camera_tf: Optional[np.ndarray] = None

        configured_repo_dir = str(self.get_parameter("xfeat_repo_dir").value)
        self._xfeat_repo_dir = _resolve_xfeat_repo_dir(configured_repo_dir)
        if self._xfeat_repo_dir not in sys.path:
            sys.path.insert(0, self._xfeat_repo_dir)

        from modules.xfeat import XFeat  # pylint: disable=import-outside-toplevel

        configured_weights_path = str(self.get_parameter("xfeat_weights_path").value)
        weights_path = configured_weights_path or os.path.join(self._xfeat_repo_dir, "weights", "xfeat.pt")
        self._xfeat = XFeat(
            weights=weights_path,
            top_k=self._top_k,
            detection_threshold=self._detection_threshold,
        )

        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_tf else None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

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
        self.get_logger().info("Running original XFeat RGB-D odometry logic: feature extraction, matching, depth back-projection and PnP.")
        self.get_logger().info(
            f"Publishing odom as {self._odom_frame}->{self._base_frame}, estimated from {self._camera_frame}"
        )

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _rgbd_callback(self, rgb_msg: Image, depth_msg: Image) -> None:
        if self._camera_info is None:
            return

        try:
            color_image = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth_image = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
            return

        try:
            with torch.inference_mode():
                output = self._xfeat.detectAndCompute(
                    _frame_to_tensor(color_image),
                    top_k=self._top_k,
                    detection_threshold=self._detection_threshold,
                )[0]
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"XFeat inference failed: {exc}")
            return

        observation = self._make_observation(color_image, depth_image, output)
        if self._prev_obs is not None:
            pose_result = self._estimate_relative_pose(self._prev_obs, observation)
            if pose_result is not None:
                self._pose_world_cam = self._pose_world_cam @ _invert_pose(pose_result["transform_curr_prev"])
                self._publish_odometry(rgb_msg)
                self._log_motion_status(pose_result)
                self.get_logger().info(
                    "Pose OK | matches %d | PnP %d | inliers %d" % (
                        pose_result["num_matches"],
                        pose_result["num_pnp_points"],
                        pose_result["num_inliers"],
                    ),
                    throttle_duration_sec=2.0,
                )
            else:
                self.get_logger().warn(
                    "Pose unavailable | insufficient stable 3D-2D correspondences",
                    throttle_duration_sec=2.0,
                )
        else:
            self._publish_odometry(rgb_msg)

        self._prev_obs = observation

    def _make_observation(self, color_image: np.ndarray, depth_image: np.ndarray, output: dict) -> dict:
        keypoints = output["keypoints"].detach().cpu().numpy().astype(np.float32, copy=False)
        scores = output["scores"].detach().cpu().numpy().astype(np.float32, copy=False)
        descriptors = output["descriptors"]

        if self._min_score > 0.0 and keypoints.shape[0] > 0:
            keep = scores >= self._min_score
            keypoints = keypoints[keep]
            scores = scores[keep]
            keep_torch = torch.from_numpy(keep).to(descriptors.device)
            descriptors = descriptors[keep_torch]

        return {
            "color_image": color_image,
            "depth_image": depth_image,
            "keypoints": keypoints,
            "scores": scores,
            "descriptors": descriptors,
        }

    def _estimate_relative_pose(self, prev_obs: dict, curr_obs: dict) -> Optional[dict]:
        if (
            len(prev_obs["keypoints"]) == 0
            or len(curr_obs["keypoints"]) == 0
            or prev_obs["descriptors"].numel() == 0
            or curr_obs["descriptors"].numel() == 0
        ):
            return None

        idx_prev, idx_curr = self._xfeat.match(
            prev_obs["descriptors"],
            curr_obs["descriptors"],
            min_cossim=self._match_min_cossim,
        )
        if len(idx_prev) < self._min_pnp_points:
            return None

        prev_kpts = prev_obs["keypoints"][idx_prev.cpu().numpy()]
        curr_kpts = curr_obs["keypoints"][idx_curr.cpu().numpy()]
        camera_matrix = self._camera_matrix_from_info(self._camera_info)

        prev_points_3d, valid_depth = self._backproject_keypoints(prev_kpts, prev_obs["depth_image"], camera_matrix)
        object_points = prev_points_3d[valid_depth].astype(np.float32)
        image_points = curr_kpts[valid_depth].astype(np.float32)
        if len(object_points) < self._min_pnp_points:
            return None

        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            object_points,
            image_points,
            camera_matrix,
            None,
            iterationsCount=self._pnp_iterations,
            reprojectionError=self._pnp_reproj_error,
            confidence=0.999,
            flags=cv2.SOLVEPNP_EPNP,
        )
        if not ok or inliers is None or len(inliers) < self._min_inliers:
            return None

        inlier_idx = inliers.reshape(-1)
        ok, rvec, tvec = cv2.solvePnP(
            object_points[inlier_idx],
            image_points[inlier_idx],
            camera_matrix,
            None,
            rvec=rvec,
            tvec=tvec,
            useExtrinsicGuess=True,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            return None

        rotation, _ = cv2.Rodrigues(rvec)
        return {
            "transform_curr_prev": _pose_matrix_from_rt(rotation, tvec),
            "num_matches": int(len(idx_prev)),
            "num_pnp_points": int(len(object_points)),
            "num_inliers": int(len(inlier_idx)),
        }

    def _backproject_keypoints(
        self,
        keypoints: np.ndarray,
        depth_image: np.ndarray,
        camera_matrix: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        if len(keypoints) == 0:
            return np.zeros((0, 3), dtype=np.float32), np.zeros((0,), dtype=bool)

        xs = np.rint(keypoints[:, 0]).astype(np.int32)
        ys = np.rint(keypoints[:, 1]).astype(np.int32)
        valid = (
            (xs >= 0)
            & (ys >= 0)
            & (xs < depth_image.shape[1])
            & (ys < depth_image.shape[0])
        )

        depths_m = np.zeros((len(keypoints),), dtype=np.float32)
        if depth_image.dtype == np.uint16:
            depths_m[valid] = depth_image[ys[valid], xs[valid]].astype(np.float32) * self._depth_scale
        else:
            depths_m[valid] = depth_image[ys[valid], xs[valid]].astype(np.float32)
        valid &= (depths_m >= self._min_depth_m) & (depths_m <= self._max_depth_m)

        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]

        points_3d = np.zeros((len(keypoints), 3), dtype=np.float32)
        points_3d[:, 0] = (keypoints[:, 0] - cx) * depths_m / fx
        points_3d[:, 1] = (keypoints[:, 1] - cy) * depths_m / fy
        points_3d[:, 2] = depths_m
        return points_3d, valid

    @staticmethod
    def _camera_matrix_from_info(camera_info: CameraInfo) -> np.ndarray:
        return np.array(
            [
                [camera_info.k[0], 0.0, camera_info.k[2]],
                [0.0, camera_info.k[4], camera_info.k[5]],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )

    def _publish_odometry(self, rgb_msg: Image) -> None:
        stamp_sec = float(rgb_msg.header.stamp.sec) + float(rgb_msg.header.stamp.nanosec) * 1e-9
        dt = 0.0 if self._last_stamp_sec is None else max(stamp_sec - self._last_stamp_sec, 1e-6)
        pose_world_base = self._world_base_pose()
        position = pose_world_base[:3, 3].astype(np.float64)
        linear_velocity = np.zeros((3,), dtype=np.float64)
        if dt > 0.0 and self._last_stamp_sec is not None:
            linear_velocity = (position - self._last_translation) / dt
        self._last_translation = position.copy()
        self._last_stamp_sec = stamp_sec

        odom = Odometry()
        odom.header = rgb_msg.header
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = float(position[0])
        odom.pose.pose.position.y = float(position[1])
        odom.pose.pose.position.z = float(position[2])
        quat = _rotation_matrix_to_quaternion(pose_world_base[:3, :3])
        odom.pose.pose.orientation.x = float(quat[0])
        odom.pose.pose.orientation.y = float(quat[1])
        odom.pose.pose.orientation.z = float(quat[2])
        odom.pose.pose.orientation.w = float(quat[3])
        odom.twist.twist.linear.x = float(linear_velocity[0])
        odom.twist.twist.linear.y = float(linear_velocity[1])
        odom.twist.twist.linear.z = float(linear_velocity[2])
        self._odom_pub.publish(odom)

        if self._tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header = odom.header
            tf_msg.child_frame_id = self._base_frame
            tf_msg.transform.translation.x = odom.pose.pose.position.x
            tf_msg.transform.translation.y = odom.pose.pose.position.y
            tf_msg.transform.translation.z = odom.pose.pose.position.z
            tf_msg.transform.rotation = odom.pose.pose.orientation
            self._tf_broadcaster.sendTransform(tf_msg)

    def _log_motion_status(self, pose_result: dict) -> None:
        position = self._world_base_pose()[:3, 3].astype(np.float64)
        delta = position - self._last_logged_translation
        planar_step = float(np.linalg.norm(delta[[0, 2]]))
        total_dist = float(np.linalg.norm(position[[0, 2]]))
        moving = planar_step > 0.005
        self._last_logged_translation = position.copy()

        self.get_logger().info(
            "Motion %s | pos xyz=(%.3f, %.3f, %.3f) m | step_xz=%.3f m | total_xz=%.3f m | matches=%d inliers=%d"
            % (
                "MOVING" if moving else "STABLE",
                position[0],
                position[1],
                position[2],
                planar_step,
                total_dist,
                pose_result["num_matches"],
                pose_result["num_inliers"],
            ),
            throttle_duration_sec=0.5,
        )

    def _world_base_pose(self) -> np.ndarray:
        base_to_camera = self._lookup_base_to_camera()
        if base_to_camera is None:
            return self._pose_world_cam
        camera_to_base = _invert_pose(base_to_camera)
        return self._pose_world_cam @ camera_to_base

    def _lookup_base_to_camera(self) -> Optional[np.ndarray]:
        if self._base_frame == self._camera_frame:
            if self._base_to_camera_tf is None:
                self._base_to_camera_tf = np.eye(4, dtype=np.float32)
            return self._base_to_camera_tf

        if self._base_to_camera_tf is not None:
            return self._base_to_camera_tf

        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._camera_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Waiting for static TF {self._base_frame}->{self._camera_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None

        translation = np.array(
            [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z],
            dtype=np.float32,
        )
        quaternion = np.array(
            [tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w],
            dtype=np.float32,
        )
        self._base_to_camera_tf = _transform_to_matrix(translation, quaternion)
        return self._base_to_camera_tf


def main(args=None) -> None:
    rclpy.init(args=args)
    node = XFeatRgbdOdometry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
