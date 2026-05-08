"""
    "XFeat: Accelerated Features for Lightweight Image Matching, CVPR 2024."
    https://www.verlab.dcc.ufmg.br/descriptors/xfeat_cvpr24/

    Intel RealSense D435 RGB-D demo with XFeat keypoints.
"""

import argparse
from time import sleep, time

import cv2
import numpy as np
import torch

try:
    import pyrealsense2 as rs
except ImportError as exc:
    raise RuntimeError(
        "pyrealsense2 is required for the RGB-D demo. "
        "Install it in the Python environment you plan to run."
    ) from exc

from modules.xfeat import XFeat


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run XFeat on Intel RealSense RGB-D stream and visualize depth at keypoints."
    )
    parser.add_argument("--serial", type=str, default=None, help="Optional RealSense serial number.")
    parser.add_argument("--width", type=int, default=640, help="Color/depth stream width.")
    parser.add_argument("--height", type=int, default=480, help="Color/depth stream height.")
    parser.add_argument("--fps", type=int, default=30, help="RealSense stream FPS.")
    parser.add_argument("--max_kpts", type=int, default=256, help="Maximum XFeat keypoints.")
    parser.add_argument(
        "--detection_threshold",
        type=float,
        default=0.05,
        help="Minimum score threshold used by XFeat before top-k filtering.",
    )
    parser.add_argument(
        "--min_score",
        type=float,
        default=0.08,
        help="Optional score filter applied after extraction.",
    )
    parser.add_argument("--radius", type=int, default=2, help="Circle radius for keypoints.")
    parser.add_argument(
        "--max_depth_m",
        type=float,
        default=3.0,
        help="Maximum depth in meters shown in the depth colormap.",
    )
    parser.add_argument(
        "--label_top_k",
        type=int,
        default=25,
        help="How many strongest keypoints should have depth text labels.",
    )
    parser.add_argument(
        "--match_min_cossim",
        type=float,
        default=0.82,
        help="Minimum cosine similarity for XFeat mutual nearest-neighbor matching.",
    )
    parser.add_argument(
        "--min_pnp_points",
        type=int,
        default=12,
        help="Minimum 3D-2D correspondences required before running PnP.",
    )
    parser.add_argument(
        "--min_inliers",
        type=int,
        default=10,
        help="Minimum inlier count required to accept a pose estimate.",
    )
    parser.add_argument(
        "--pnp_reproj_error",
        type=float,
        default=4.0,
        help="RANSAC reprojection threshold in pixels for solvePnPRansac.",
    )
    parser.add_argument(
        "--pnp_iterations",
        type=int,
        default=200,
        help="Maximum RANSAC iterations for solvePnPRansac.",
    )
    parser.add_argument(
        "--min_depth_m",
        type=float,
        default=0.2,
        help="Minimum valid depth used for odometry correspondences.",
    )
    parser.add_argument(
        "--traj_scale",
        type=float,
        default=90.0,
        help="Pixels per meter for the top-down trajectory panel.",
    )
    return parser.parse_args()


def frame_to_tensor(frame_bgr):
    rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    tensor = torch.from_numpy(rgb).float() / 255.0
    return tensor.permute(2, 0, 1)[None]


def put_label(frame, text, org, color=(0, 255, 0), scale=0.6):
    cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def colorize_depth(depth_image, max_depth_m):
    max_depth_mm = max(int(max_depth_m * 1000.0), 1)
    depth_clipped = np.clip(depth_image, 0, max_depth_mm)
    depth_8u = cv2.convertScaleAbs(depth_clipped, alpha=255.0 / max_depth_mm)
    return cv2.applyColorMap(depth_8u, cv2.COLORMAP_JET)


def sample_depths(depth_frame, keypoints):
    distances = np.zeros((len(keypoints),), dtype=np.float32)
    for i, (x, y) in enumerate(keypoints.astype(np.int32)):
        distances[i] = depth_frame.get_distance(int(x), int(y))
    return distances


def draw_keypoints_with_depth(color_frame, depth_vis, keypoints, scores, depths_m, radius, label_top_k):
    color_canvas = color_frame.copy()
    depth_canvas = depth_vis.copy()

    for i, ((x, y), score, depth_m) in enumerate(zip(keypoints.astype(np.int32), scores, depths_m)):
        valid_depth = depth_m > 0
        color = (0, int(255 * min(max(score, 0.0), 1.0)), 255 if valid_depth else 80)
        cv2.circle(color_canvas, (x, y), radius, color, -1, lineType=cv2.LINE_AA)
        cv2.circle(depth_canvas, (x, y), radius, color, -1, lineType=cv2.LINE_AA)

        if i < label_top_k and valid_depth:
            text = f"{depth_m:.2f}m"
            text_org = (x + 4, y - 4)
            put_label(color_canvas, text, text_org, color=(255, 255, 255), scale=0.45)

    return color_canvas, depth_canvas


def build_pipeline(args):
    pipeline = rs.pipeline()
    config = rs.config()
    if args.serial:
        config.enable_device(args.serial)

    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)

    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)
    device = profile.get_device()
    depth_sensor = device.first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    return pipeline, align, depth_scale, device


def get_camera_matrix(color_frame):
    intr = color_frame.profile.as_video_stream_profile().intrinsics
    camera_matrix = np.array(
        [[intr.fx, 0.0, intr.ppx], [0.0, intr.fy, intr.ppy], [0.0, 0.0, 1.0]],
        dtype=np.float32,
    )
    return camera_matrix, intr


def backproject_keypoints(keypoints, depth_image, camera_matrix, depth_scale, min_depth_m, max_depth_m):
    if len(keypoints) == 0:
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0,), dtype=bool), np.zeros((0,), dtype=np.float32)

    xs = np.rint(keypoints[:, 0]).astype(np.int32)
    ys = np.rint(keypoints[:, 1]).astype(np.int32)

    valid = (
        (xs >= 0)
        & (ys >= 0)
        & (xs < depth_image.shape[1])
        & (ys < depth_image.shape[0])
    )

    depths_m = np.zeros((len(keypoints),), dtype=np.float32)
    depths_m[valid] = depth_image[ys[valid], xs[valid]].astype(np.float32) * depth_scale
    valid &= (depths_m >= min_depth_m) & (depths_m <= max_depth_m)

    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    points_3d = np.zeros((len(keypoints), 3), dtype=np.float32)
    points_3d[:, 0] = (keypoints[:, 0] - cx) * depths_m / fx
    points_3d[:, 1] = (keypoints[:, 1] - cy) * depths_m / fy
    points_3d[:, 2] = depths_m

    return points_3d, valid, depths_m


def pose_matrix_from_rt(rotation, translation):
    transform = np.eye(4, dtype=np.float32)
    transform[:3, :3] = rotation.astype(np.float32)
    transform[:3, 3] = translation.reshape(3).astype(np.float32)
    return transform


def invert_pose(transform):
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inv = np.eye(4, dtype=np.float32)
    inv[:3, :3] = rotation.T
    inv[:3, 3] = -rotation.T @ translation
    return inv


def estimate_relative_pose(prev_obs, curr_obs, xfeat, camera_matrix, depth_scale, args):
    if (
        len(prev_obs["keypoints"]) == 0
        or len(curr_obs["keypoints"]) == 0
        or prev_obs["descriptors"].numel() == 0
        or curr_obs["descriptors"].numel() == 0
    ):
        return None

    idx_prev, idx_curr = xfeat.match(
        prev_obs["descriptors"],
        curr_obs["descriptors"],
        min_cossim=args.match_min_cossim,
    )

    if len(idx_prev) < args.min_pnp_points:
        return None

    prev_kpts = prev_obs["keypoints"][idx_prev.cpu().numpy()]
    curr_kpts = curr_obs["keypoints"][idx_curr.cpu().numpy()]

    prev_points_3d, valid_depth, _ = backproject_keypoints(
        prev_kpts,
        prev_obs["depth_image"],
        camera_matrix,
        depth_scale,
        args.min_depth_m,
        args.max_depth_m,
    )

    object_points = prev_points_3d[valid_depth].astype(np.float32)
    image_points = curr_kpts[valid_depth].astype(np.float32)

    if len(object_points) < args.min_pnp_points:
        return None

    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        object_points,
        image_points,
        camera_matrix,
        None,
        iterationsCount=args.pnp_iterations,
        reprojectionError=args.pnp_reproj_error,
        confidence=0.999,
        flags=cv2.SOLVEPNP_EPNP,
    )
    if not ok or inliers is None or len(inliers) < args.min_inliers:
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
    transform_curr_prev = pose_matrix_from_rt(rotation, tvec)

    return {
        "idx_prev": idx_prev.cpu().numpy(),
        "idx_curr": idx_curr.cpu().numpy(),
        "object_points": object_points,
        "image_points": image_points,
        "inlier_idx": inlier_idx,
        "transform_curr_prev": transform_curr_prev,
        "num_matches": int(len(idx_prev)),
        "num_pnp_points": int(len(object_points)),
        "num_inliers": int(len(inlier_idx)),
    }


def make_observation(color_image, depth_image, output):
    return {
        "color_image": color_image,
        "depth_image": depth_image,
        "keypoints": output["keypoints"].cpu().numpy(),
        "scores": output["scores"].cpu().numpy(),
        "descriptors": output["descriptors"],
    }


def draw_pose_matches(prev_obs, curr_color_vis, pose_result):
    if pose_result is None or pose_result["num_inliers"] <= 0:
        return curr_color_vis

    canvas = curr_color_vis.copy()
    curr_points = pose_result["image_points"][pose_result["inlier_idx"]]
    for x, y in np.rint(curr_points).astype(np.int32):
        cv2.circle(canvas, (x, y), 3, (255, 200, 0), 1, lineType=cv2.LINE_AA)
    return canvas


def draw_trajectory_panel(trajectory_xyz, pose_world_cam, traj_scale, panel_size=480):
    panel = np.full((panel_size, panel_size, 3), 18, dtype=np.uint8)
    center = np.array([panel_size // 2, panel_size // 2], dtype=np.int32)

    cv2.line(panel, (0, center[1]), (panel_size, center[1]), (50, 50, 50), 1, cv2.LINE_AA)
    cv2.line(panel, (center[0], 0), (center[0], panel_size), (50, 50, 50), 1, cv2.LINE_AA)

    def project(point_xyz):
        x = int(round(center[0] + point_xyz[0] * traj_scale))
        z = int(round(center[1] - point_xyz[2] * traj_scale))
        return x, z

    if len(trajectory_xyz) >= 2:
        for p0, p1 in zip(trajectory_xyz[:-1], trajectory_xyz[1:]):
            cv2.line(panel, project(p0), project(p1), (0, 220, 255), 2, cv2.LINE_AA)

    current_pos = pose_world_cam[:3, 3]
    current_px = project(current_pos)
    cv2.circle(panel, current_px, 5, (0, 255, 0), -1, lineType=cv2.LINE_AA)

    forward_world = pose_world_cam[:3, :3] @ np.array([0.0, 0.0, 1.0], dtype=np.float32)
    heading_tip = (
        int(round(current_px[0] + forward_world[0] * 25.0)),
        int(round(current_px[1] - forward_world[2] * 25.0)),
    )
    cv2.arrowedLine(panel, current_px, heading_tip, (255, 255, 255), 2, cv2.LINE_AA, tipLength=0.25)

    put_label(panel, "Top-down trajectory (x-z)", (12, 28))
    put_label(panel, f"Scale: {traj_scale:.0f}px/m", (12, 56))
    put_label(panel, "Origin = startup pose", (12, 84))

    return panel


def wait_for_aligned_frames(pipeline, align, timeout_ms, warmup_frames):
    aligned_frames = None
    for _ in range(max(warmup_frames, 1)):
        frames = pipeline.wait_for_frames(timeout_ms=timeout_ms)
        aligned_frames = align.process(frames)

    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    if not depth_frame or not color_frame:
        raise RuntimeError("Aligned RealSense frames are incomplete.")

    return depth_frame, color_frame


def start_pipeline_with_retry(args, retries=3, timeout_ms=15000, warmup_frames=15):
    last_error = None

    for attempt in range(1, retries + 1):
        pipeline = None
        try:
            pipeline, align, depth_scale, device = build_pipeline(args)
            depth_frame, color_frame = wait_for_aligned_frames(
                pipeline, align, timeout_ms=timeout_ms, warmup_frames=warmup_frames
            )
            return pipeline, align, depth_scale, device, depth_frame, color_frame
        except Exception as exc:
            last_error = exc
            if pipeline is not None:
                try:
                    device = pipeline.get_active_profile().get_device()
                    device.hardware_reset()
                except Exception:
                    pass
                try:
                    pipeline.stop()
                except Exception:
                    pass

            if attempt < retries:
                print(
                    f"RealSense start attempt {attempt}/{retries} failed: {exc}. "
                    "Retrying after device reset...",
                    flush=True,
                )
                sleep(4)

    raise RuntimeError(
        f"Failed to start RealSense RGB-D stream after {retries} attempts: {last_error}"
    ) from last_error


def main():
    args = parse_args()

    try:
        window_name = "RealSense XFeat RGB-D - Press q to quit"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    except cv2.error as exc:
        raise RuntimeError(
            "OpenCV in the current Python environment does not have GUI support, "
            "so it cannot open a display window."
        ) from exc

    pipeline, align, depth_scale, device, depth_frame, color_frame = start_pipeline_with_retry(args)
    xfeat = XFeat(top_k=args.max_kpts, detection_threshold=args.detection_threshold)
    fps_history = []
    pose_world_cam = np.eye(4, dtype=np.float32)
    trajectory_xyz = [pose_world_cam[:3, 3].copy()]
    prev_obs = None
    pose_status = "Waiting for first frame pair"
    last_pose_result = None

    print(
        f"RealSense started: {device.get_info(rs.camera_info.name)} "
        f"(serial {device.get_info(rs.camera_info.serial_number)}), "
        f"depth scale {depth_scale:.6f} m/unit",
        flush=True,
    )

    try:
        while True:
            try:
                depth_frame, color_frame = wait_for_aligned_frames(
                    pipeline, align, timeout_ms=15000, warmup_frames=1
                )
            except Exception as exc:
                print(f"Frame grab timeout: {exc}. Reinitializing RealSense stream...", flush=True)
                try:
                    pipeline.stop()
                except Exception:
                    pass
                pipeline, align, depth_scale, device, depth_frame, color_frame = start_pipeline_with_retry(args)
                prev_obs = None
                pose_status = "Stream restarted; waiting for frame pair"
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            camera_matrix, _ = get_camera_matrix(color_frame)

            t0 = time()
            tensor = frame_to_tensor(color_image)
            with torch.inference_mode():
                output = xfeat.detectAndCompute(
                    tensor,
                    top_k=args.max_kpts,
                    detection_threshold=args.detection_threshold,
                )[0]

            observation = make_observation(color_image, depth_image, output)
            keypoints = observation["keypoints"]
            scores = observation["scores"]

            if args.min_score > 0:
                keep = scores >= args.min_score
                keypoints = keypoints[keep]
                scores = scores[keep]
                keep_torch = torch.from_numpy(keep).to(observation["descriptors"].device)
                observation["keypoints"] = keypoints
                observation["scores"] = scores
                observation["descriptors"] = observation["descriptors"][keep_torch]

            depths_m = sample_depths(depth_frame, keypoints) if len(keypoints) else np.zeros((0,), dtype=np.float32)
            valid_depth = depths_m > 0

            depth_vis = colorize_depth(depth_image, args.max_depth_m)
            color_vis, depth_vis = draw_keypoints_with_depth(
                color_image,
                depth_vis,
                keypoints,
                scores,
                depths_m,
                args.radius,
                args.label_top_k,
            )

            if prev_obs is not None:
                pose_result = estimate_relative_pose(
                    prev_obs,
                    observation,
                    xfeat,
                    camera_matrix,
                    depth_scale,
                    args,
                )
                if pose_result is not None:
                    pose_world_cam = pose_world_cam @ invert_pose(pose_result["transform_curr_prev"])
                    trajectory_xyz.append(pose_world_cam[:3, 3].copy())
                    pose_status = (
                        f"Pose OK | matches {pose_result['num_matches']} | "
                        f"PnP {pose_result['num_pnp_points']} | inliers {pose_result['num_inliers']}"
                    )
                    last_pose_result = pose_result
                else:
                    pose_status = "Pose unavailable | insufficient stable 3D-2D correspondences"
                    last_pose_result = None
            else:
                pose_status = "Initializing odometry from first frame"
                last_pose_result = None

            color_vis = draw_pose_matches(prev_obs, color_vis, last_pose_result)
            traj_vis = draw_trajectory_panel(trajectory_xyz, pose_world_cam, args.traj_scale, panel_size=args.height)

            elapsed = max(time() - t0, 1e-6)
            fps_history.append(1.0 / elapsed)
            if len(fps_history) > 30:
                fps_history.pop(0)
            avg_fps = sum(fps_history) / len(fps_history)

            mean_depth = float(depths_m[valid_depth].mean()) if valid_depth.any() else 0.0
            position = pose_world_cam[:3, 3]
            forward_world = pose_world_cam[:3, :3] @ np.array([0.0, 0.0, 1.0], dtype=np.float32)
            yaw_deg = float(np.degrees(np.arctan2(forward_world[0], forward_world[2])))

            put_label(color_vis, f"XFeat keypoints: {len(keypoints)}", (12, 28))
            put_label(color_vis, f"Valid depth: {int(valid_depth.sum())}", (12, 56))
            put_label(color_vis, f"Mean depth: {mean_depth:.2f}m", (12, 84))
            put_label(color_vis, f"FPS: {avg_fps:.1f}", (12, 112))
            put_label(color_vis, pose_status, (12, 140), color=(0, 220, 255), scale=0.5)
            put_label(
                color_vis,
                f"Pos xyz [m]: {position[0]:+.2f} {position[1]:+.2f} {position[2]:+.2f}",
                (12, 168),
                color=(255, 255, 255),
                scale=0.5,
            )
            put_label(
                color_vis,
                f"Heading yaw [deg]: {yaw_deg:+.1f}",
                (12, 196),
                color=(255, 255, 255),
                scale=0.5,
            )

            put_label(depth_vis, "Aligned depth colormap", (12, 28))
            put_label(depth_vis, f"Max depth view: {args.max_depth_m:.1f}m", (12, 56))
            put_label(depth_vis, "Press q to quit", (12, 84))

            canvas = np.hstack([color_vis, depth_vis, traj_vis])
            cv2.imshow(window_name, canvas)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            prev_obs = observation
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
