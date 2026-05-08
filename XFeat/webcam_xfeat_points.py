"""
    "XFeat: Accelerated Features for Lightweight Image Matching, CVPR 2024."
    https://www.verlab.dcc.ufmg.br/descriptors/xfeat_cvpr24/

    Simple webcam demo for real-time XFeat keypoint visualization.
"""

import argparse
from time import time

import cv2
import numpy as np
import torch

from modules.xfeat import XFeat


def parse_args():
    parser = argparse.ArgumentParser(
        description="Open the webcam and visualize XFeat keypoints in real time."
    )
    parser.add_argument("--cam", type=int, default=0, help="Webcam device index.")
    parser.add_argument("--width", type=int, default=640, help="Capture width.")
    parser.add_argument("--height", type=int, default=480, help="Capture height.")
    parser.add_argument("--fps", type=int, default=30, help="Requested camera FPS.")
    parser.add_argument("--max_kpts", type=int, default=512, help="Maximum keypoints to draw.")
    parser.add_argument(
        "--detection_threshold",
        type=float,
        default=0.05,
        help="Minimum score threshold used by XFeat before top-k filtering.",
    )
    parser.add_argument(
        "--min_score",
        type=float,
        default=0.0,
        help="Optional score filter applied after extraction for visualization.",
    )
    parser.add_argument(
        "--radius",
        type=int,
        default=2,
        help="Circle radius used to draw each keypoint.",
    )
    return parser.parse_args()


def frame_to_tensor(frame_bgr):
    rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    tensor = torch.from_numpy(rgb).float() / 255.0
    return tensor.permute(2, 0, 1)[None]


def ensure_frame_size(frame, width, height):
    if frame.shape[1] == width and frame.shape[0] == height:
        return frame
    return cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)


def is_single_channel_frame(frame):
    return frame.ndim == 2 or (frame.ndim == 3 and frame.shape[2] == 1)


def draw_keypoints(frame, keypoints, scores, radius):
    canvas = frame.copy()

    if len(keypoints) == 0:
        return canvas

    scores = np.clip(scores, 0.0, 1.0)
    for (x, y), score in zip(keypoints.astype(np.int32), scores):
        color = (0, int(255 * score), 255 - int(180 * score))
        cv2.circle(canvas, (x, y), radius, color, -1, lineType=cv2.LINE_AA)

    return canvas


def put_label(frame, text, org):
    cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 1, cv2.LINE_AA)


def main():
    args = parse_args()

    cap = cv2.VideoCapture(args.cam)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {args.cam}")

    window_name = "XFeat Webcam Keypoints - Press q to quit"
    try:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, args.width, args.height)
    except cv2.error as exc:
        cap.release()
        raise RuntimeError(
            "OpenCV in the current Python environment does not have GUI support, "
            "so it cannot open a display window. "
            "Use a GUI-enabled OpenCV build such as `opencv-python`, "
            "or run this script with a system Python that already supports GTK."
        ) from exc

    ok, frame = cap.read()
    if not ok or frame is None:
        cap.release()
        raise RuntimeError("Failed to read initial frame from camera.")

    if is_single_channel_frame(frame):
        cap.release()
        raise RuntimeError(
            f"Camera {args.cam} is outputting a grayscale/infrared stream instead of RGB color. "
            "For the Intel RealSense D435 on this machine, use `--cam 8` for the color stream."
        )

    frame = ensure_frame_size(frame, args.width, args.height)
    preview = frame.copy()
    put_label(preview, "Loading XFeat model...", (12, 28))
    put_label(preview, "Preview window is ready", (12, 56))
    cv2.imshow(window_name, preview)
    cv2.waitKey(1)

    print(
        f"Camera {args.cam} started, showing {args.width}x{args.height} preview. "
        "Press q in the OpenCV window to quit."
    )

    xfeat = XFeat(top_k=args.max_kpts, detection_threshold=args.detection_threshold)

    fps_history = []

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                raise RuntimeError("Failed to read frame from camera.")

            frame = ensure_frame_size(frame, args.width, args.height)

            t0 = time()
            tensor = frame_to_tensor(frame)
            with torch.inference_mode():
                output = xfeat.detectAndCompute(
                    tensor,
                    top_k=args.max_kpts,
                    detection_threshold=args.detection_threshold,
                )[0]

            keypoints = output["keypoints"].cpu().numpy()
            scores = output["scores"].cpu().numpy()

            if args.min_score > 0:
                keep = scores >= args.min_score
                keypoints = keypoints[keep]
                scores = scores[keep]

            vis = draw_keypoints(frame, keypoints, scores, args.radius)

            elapsed = max(time() - t0, 1e-6)
            fps_history.append(1.0 / elapsed)
            if len(fps_history) > 30:
                fps_history.pop(0)
            avg_fps = sum(fps_history) / len(fps_history)

            put_label(vis, f"XFeat keypoints: {len(keypoints)}", (12, 28))
            put_label(vis, f"FPS: {avg_fps:.1f}", (12, 56))
            put_label(vis, "Press q to quit", (12, 84))

            cv2.imshow(window_name, vis)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
