import time
import cv2
import torch
import sys

from modules.xfeat import XFeat


def load_image(path, size=(320, 240)):
    img = cv2.imread(path)
    if img is None:
        raise FileNotFoundError(path)

    img = cv2.resize(img, size)
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    tensor = torch.from_numpy(rgb).float() / 255.0
    tensor = tensor.permute(2, 0, 1)[None]
    return tensor


def main():
    if len(sys.argv) < 2:
        print("用法: python bench_xfeat_extract.py /path/to/image.png")
        return

    torch.set_num_threads(4)

    img = load_image(sys.argv[1])
    xfeat = XFeat(top_k=512)

    with torch.no_grad():
        for _ in range(20):
            _ = xfeat.detectAndCompute(img, top_k=512)

    n = 100
    t0 = time.time()

    with torch.no_grad():
        for _ in range(n):
            out = xfeat.detectAndCompute(img, top_k=512)[0]

    t1 = time.time()

    avg_ms = (t1 - t0) * 1000 / n
    fps = 1000.0 / avg_ms

    print("keypoints:", out["keypoints"].shape)
    print("descriptors:", out["descriptors"].shape)
    print(f"avg: {avg_ms:.2f} ms")
    print(f"fps: {fps:.2f}")


if __name__ == "__main__":
    main()
