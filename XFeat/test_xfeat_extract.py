import sys
import cv2
import torch
import numpy as np

from modules.xfeat import XFeat


def main():
    if len(sys.argv) < 2:
        print("用法: python test_xfeat_extract.py /path/to/image.png")
        return

    img_path = sys.argv[1]

    img = cv2.imread(img_path)
    if img is None:
        raise FileNotFoundError(img_path)

    img = cv2.resize(img, (320, 240))

    # BGR -> RGB, HWC -> BCHW, 0~1
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    tensor = torch.from_numpy(rgb).float() / 255.0
    tensor = tensor.permute(2, 0, 1)[None]

    xfeat = XFeat(top_k=512)

    with torch.no_grad():
        out = xfeat.detectAndCompute(tensor, top_k=512)[0]

    kpts = out["keypoints"].cpu().numpy()
    desc = out["descriptors"].cpu().numpy()
    scores = out["scores"].cpu().numpy()

    print("keypoints:", kpts.shape)
    print("descriptors:", desc.shape)
    print("scores:", scores.shape)

    vis = img.copy()
    for x, y in kpts.astype(int):
        cv2.circle(vis, (x, y), 2, (0, 0, 255), -1)

    cv2.imwrite("xfeat_keypoints_debug.png", vis)
    print("saved: xfeat_keypoints_debug.png")


if __name__ == "__main__":
    main()
