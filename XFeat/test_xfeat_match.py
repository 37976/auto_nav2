import sys
import cv2
import torch
import numpy as np

from modules.xfeat import XFeat


def load_image(path, size=(320, 240)):
    img = cv2.imread(path)
    if img is None:
        raise FileNotFoundError(path)

    img = cv2.resize(img, size)
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    tensor = torch.from_numpy(rgb).float() / 255.0
    tensor = tensor.permute(2, 0, 1)[None]

    return img, tensor


def main():
    if len(sys.argv) < 3:
        print("用法: python test_xfeat_match.py img1.png img2.png")
        return

    img1_path = sys.argv[1]
    img2_path = sys.argv[2]

    img1, t1 = load_image(img1_path)
    img2, t2 = load_image(img2_path)

    xfeat = XFeat(top_k=512)

    with torch.no_grad():
        out1 = xfeat.detectAndCompute(t1, top_k=512)[0]
        out2 = xfeat.detectAndCompute(t2, top_k=512)[0]

    kpts1 = out1["keypoints"].cpu().numpy().astype(np.float32)
    kpts2 = out2["keypoints"].cpu().numpy().astype(np.float32)

    desc1 = out1["descriptors"].cpu().numpy().astype(np.float32)
    desc2 = out2["descriptors"].cpu().numpy().astype(np.float32)

    # BFMatcher，XFeat descriptor 是 float descriptor
    matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
    matches = matcher.match(desc1, desc2)
    matches = sorted(matches, key=lambda x: x.distance)

    pts1 = np.float32([kpts1[m.queryIdx] for m in matches])
    pts2 = np.float32([kpts2[m.trainIdx] for m in matches])

    inliers = 0
    mask = None

    if len(matches) >= 8:
        F, mask = cv2.findFundamentalMat(
            pts1,
            pts2,
            cv2.FM_RANSAC,
            1.5,
            0.99,
        )
        if mask is not None:
            inliers = int(mask.sum())

    print("kpts1:", kpts1.shape)
    print("kpts2:", kpts2.shape)
    print("matches:", len(matches))
    print("ransac inliers:", inliers)

    draw_matches = matches[:100]
    if mask is not None:
        inlier_matches = [m for m, keep in zip(matches, mask.ravel()) if keep]
        draw_matches = inlier_matches[:100]

    kp1_cv = [cv2.KeyPoint(float(x), float(y), 1) for x, y in kpts1]
    kp2_cv = [cv2.KeyPoint(float(x), float(y), 1) for x, y in kpts2]

    vis = cv2.drawMatches(
        img1,
        kp1_cv,
        img2,
        kp2_cv,
        draw_matches,
        None,
        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
    )

    cv2.imwrite("xfeat_matches_debug.png", vis)
    print("saved: xfeat_matches_debug.png")


if __name__ == "__main__":
    main()
