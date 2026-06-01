import cv2
import numpy as np
import math


def bresenham_ray_cast(image, x0, y0, angle):
    x0, y0 = int(x0), int(y0)
    angle_rad = math.radians(angle)
    dx = math.cos(angle_rad)
    dy = math.sin(angle_rad)

    x, y = x0, y0
    while 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
        if image[int(y), int(x)] < 100:
            return int(x), int(y)
        x += dx
        y += dy
    return -1, -1


def _dt_ray_cast(dt_map, x0, y0, dx, dy, max_range_px):
    """距离变换跳跃步进: 每次跳跃到最近障碍物的距离, O(log range)."""
    h, w = dt_map.shape
    x, y = float(x0), float(y0)
    for _ in range(50):  # 安全上限, 实际通常 5-15 步
        px, py = int(x), int(y)
        if px < 0 or px >= w or py < 0 or py >= h:
            return -1, -1
        dist = dt_map[py, px]
        if dist <= 1.5:
            return px, py
        step = max(1.0, dist - 1.0)
        x += dx * step
        y += dy * step
        if math.hypot(x - x0, y - y0) > max_range_px:
            return -1, -1
    return -1, -1


def get_lidar_points(bw_image, x0, y0, add_noise=True, dt_map=None):
    lidar_range = 8  # m
    map_resolution = 0.05  # m/pixel
    mean = 0
    std_dev = 0.01  # m
    max_range_px = lidar_range / map_resolution

    angle_resolution = 1.0  # Degrees.
    angle_range = np.arange(0, 360, angle_resolution)
    lidar_points = []

    if dt_map is not None:
        # 快速路径: 距离变换跳跃步进
        angles_rad = np.radians(angle_range)
        dx_arr = np.cos(angles_rad)
        dy_arr = np.sin(angles_rad)
        for i in range(len(angle_range)):
            point = _dt_ray_cast(dt_map, x0, y0, float(dx_arr[i]), float(dy_arr[i]), max_range_px)
            distance = math.hypot(point[0] - x0, point[1] - y0) if point[0] != -1 else float('inf')
            if distance < max_range_px and point[0] != -1 and point[1] != -1:
                if add_noise:
                    noise_x = np.random.normal(mean, std_dev / map_resolution)
                    noise_y = np.random.normal(mean, std_dev)
                    lidar_points.append([int(point[0] - x0 + noise_x),
                                         int(point[1] - y0 + noise_y)])
                else:
                    lidar_points.append([point[0] - x0, point[1] - y0])
        return lidar_points

    # 回退: 逐像素 Bresenham
    for i in range(angle_range.shape[0]):
        noise_x = np.random.normal(mean, std_dev / map_resolution)
        noise_y = np.random.normal(mean, std_dev)

        point = bresenham_ray_cast(bw_image, x0, y0, angle_range[i])

        distance = math.sqrt((point[0] - x0)**2 + (point[1] - y0)**2)
        if distance < max_range_px and point[0] != -1 and point[1] != -1:
            if add_noise:
                noisy_x = point[0] + noise_x
                noisy_y = point[1] + noise_y
                lidar_points.append([int(noisy_x - x0), int(noisy_y - y0)])
            else:
                point = [point[0] - x0, point[1] - y0]
                lidar_points.append(point)

    return lidar_points


def compute_dt_map(map_image):
    """预计算地图的原始距离变换 (float32 像素距离)."""
    _, binary = cv2.threshold(map_image, 150, 255, cv2.THRESH_BINARY)
    return cv2.distanceTransform(binary, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
