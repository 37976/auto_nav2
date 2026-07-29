import cv2
import numpy as np
from global_localizer import distance_transform as dt
from global_localizer import pose_finder as pf
import matplotlib.pyplot as plt
from global_localizer import feature_matching as fm
from global_localizer import scanner_simulator as s_sim
import math
import time

def calculate_white_pixel_ratio(orig_image, target_image):
    # Ensure the images have the same dimensions
    if orig_image.shape != target_image.shape:
        raise ValueError("Input images must have the same dimensions for white pixel ratio calculation.")
    
    kernel = np.ones((3, 3), np.uint8)
    dilated_orig_image = cv2.dilate(orig_image, kernel, iterations=2)
    dilated_target_image = cv2.dilate(target_image, kernel, iterations=2)

    white_pixels_reference = np.count_nonzero(dilated_orig_image)
    masked_img = cv2.bitwise_and(dilated_orig_image, dilated_target_image)
    white_pixels_target = np.count_nonzero(masked_img)

    pixel_ratio = white_pixels_target / white_pixels_reference

    return pixel_ratio



def get_score_2(reference_img, tf_img):
    kernel = np.ones((3, 3), np.uint8)
    dilated_reference_img = cv2.dilate(reference_img, kernel, iterations=0)
    dilated_tf_img = cv2.dilate(tf_img, kernel, iterations=0)

    mask = cv2.bitwise_and(dilated_reference_img, dilated_tf_img)
    mask2 = cv2.bitwise_and(reference_img, cv2.bitwise_not(tf_img))


    ratio = np.count_nonzero(mask) / np.count_nonzero(mask2)
    percentage = ratio * 100
    return percentage

def get_f1_score(reference_img, tf_img):

    # Adding 0.01 to avoid Division by zero
    TP = np.count_nonzero(cv2.bitwise_and(reference_img, tf_img)) + 0.01
    FP = np.count_nonzero(cv2.bitwise_and(tf_img, cv2.bitwise_not(reference_img))) + 0.01
    FN = np.count_nonzero(cv2.bitwise_and(reference_img, cv2.bitwise_not(tf_img))) + 0.01

    if TP == 0:
        return 0
    
    precision = TP / (TP + FP)
    recall = TP / (TP + FN)


    f1_score = 2 * (precision * recall) / (precision + recall)

    return f1_score * 100


def _compute_adaptive_iterations(num_candidates, user_max_iterations, time_budget_ms):
    """根据候选区域像素数自适应计算迭代次数，保持与小地图相同的采样密度."""
    BASE_ITERS = 30
    BASE_CANDIDATE_PX = 5000       # 10m 地图典型候选像素数
    MS_PER_ITER = 50               # 每次迭代约 30-80ms (大地图 np.where 开销大)
    suggested = max(BASE_ITERS, int(num_candidates * BASE_ITERS / BASE_CANDIDATE_PX))
    max_by_time = max(BASE_ITERS, time_budget_ms // MS_PER_ITER)
    return min(suggested, user_max_iterations, max_by_time)


def _get_candidates(map_image, min_distance, map_resolution, min_required=500):
    """渐进放宽 distance_transform 的 threshold_px，确保候选数 >= min_required."""
    for threshold_px in [3, 6, 12, 24]:
        candidate_area = dt.get_distance_transform(
            map_image, min_distance,
            map_resolution=map_resolution,
            threshold_px=threshold_px)
        red_pixels = np.where(candidate_area[:, :, 0] == 255)
        num = len(red_pixels[0])
        if num >= min_required:
            return candidate_area, red_pixels, num, threshold_px

    # 回退：全部自由空间
    _, binary = cv2.threshold(map_image, 150, 255, cv2.THRESH_BINARY)
    red_pixels = np.where(binary == 255)
    num = len(red_pixels[0])
    candidate_area = cv2.cvtColor(binary, cv2.COLOR_GRAY2RGB)
    return candidate_area, red_pixels, num, -1


def _map_pixel_to_world(pixel_uv, map_height_px, map_resolution, map_origin,
                        map_pose_offset=(0.0, 0.0)):
    """Convert an image pixel-center coordinate into the map world frame."""
    u, v = np.asarray(pixel_uv, dtype=float)
    origin_x, origin_y = np.asarray(map_origin, dtype=float)
    offset_x, offset_y = np.asarray(map_pose_offset, dtype=float)
    return np.array([
        origin_x + (u + 0.5) * map_resolution + offset_x,
        origin_y + (map_height_px - v - 0.5) * map_resolution + offset_y,
    ])


def solve_kidnap(orig_scan_img, map_image, min_distance, map_origin = None,
                 map_resolution = 0.05, max_iterations = 250,
                 max_time_budget_ms = 5000,
                 stop_search_threshold = 50, lidar_range = 8.0,
                 map_pose_offset = (0.0, 0.0),
                 show_plot = False):
    orig_scan_img = cv2.flip(orig_scan_img, 1)

    robot_coord = [orig_scan_img.shape[1]//2, orig_scan_img.shape[0]//2]
    distance =  min_distance

    st_time = time.perf_counter()

    # === 预计算距离变换 (加速模拟扫描) ===
    dt_map = s_sim.compute_dt_map(map_image)

    # === 自适应候选区域 + 迭代次数 ===
    candidate_area, red_pixels, num_red_pixels, used_threshold = _get_candidates(
        map_image, distance, map_resolution, min_required=500)

    with open("/tmp/kidnap_debug.txt", "a") as f:
        f.write(f"min_dist={distance:.2f}m({distance/map_resolution:.1f}px) "
                f"red={num_red_pixels} threshold_px={used_threshold}\n")
    n_iterations = _compute_adaptive_iterations(num_red_pixels, max_iterations, max_time_budget_ms)
    print(f"[DEBUG] ORB 搜索: min_distance={distance:.2f}m, candidates={num_red_pixels}, "
          f"iterations={n_iterations}/{max_iterations}, budget={max_time_budget_ms}ms", flush=True)

    candidate_area_to_show = candidate_area.copy()

    random_coords = []
    removal_radius = max(3, int(0.5 / map_resolution))  # ~0.5m = 机器人半径
    for _ in range(n_iterations):

        if num_red_pixels > 0:
            index = np.random.randint(num_red_pixels)
            x = red_pixels[0][index]
            y = red_pixels[1][index]
            random_coords.append((x, y))
            cv2.circle(candidate_area_to_show, (y, x), 3, (0, 255, 0), -1)

            cv2.circle(candidate_area, (y, x), removal_radius, (0, 255, 0), -1)
            red_pixels = np.where(candidate_area[:,:,0] == 255)
            num_red_pixels = len(red_pixels[0])

    threshold_accuracy = stop_search_threshold # % Needs fine tuning further ---- !
    max_accuracy = 0
    best_coord = None
    best_scan = None
    best_tf_img = None
    best_overlay = None
    best_tf_robot = None
    best_scan_center = None
    best_theta_degrees = None

    all_candidates = []

    iters = 0

    sim_scan_time, matching_time, scoring_time = 0, 0, 0
    # Use the random coordinates for further processing
    for coord in random_coords:
        elapsed_ms = (time.perf_counter() - st_time) * 1000.0
        if elapsed_ms >= max_time_budget_ms:
            print(f"[DEBUG] 达到时间预算 {max_time_budget_ms}ms，停止搜索", flush=True)
            break
        iters += 1
        x, y = coord
        # Do something with the coordinates
        s = time.perf_counter()
        scan_image = pf.get_scan_image(map_image.copy(), [y, x], map_resolution = map_resolution, max_range = 8, dt_map=dt_map)
        e = time.perf_counter()
        time_taken = (e - s) * 1000
        sim_scan_time += time_taken

        s = time.perf_counter()
        tf_orig_scan, overlay_img, tf_robot_pose, theta_degrees = fm.do_matching(orig_scan_img, scan_image, robot_pose = robot_coord)
        e = time.perf_counter()
        time_taken = (e - s) * 1000
        matching_time += time_taken
        if tf_orig_scan is None:
            continue

        # 几何一致性检查: 定位结果必须在 map 图像范围内
        est_robot_px = np.array([y, x]) + (np.array(tf_robot_pose) - np.array(robot_coord))
        if (est_robot_px[0] < 0 or est_robot_px[0] >= map_image.shape[1] or
            est_robot_px[1] < 0 or est_robot_px[1] >= map_image.shape[0]):
            continue

        s = time.perf_counter()
        percentage = get_f1_score(scan_image, tf_orig_scan)
        e = time.perf_counter()
        time_taken = (e - s) * 1000
        scoring_time += time_taken

        all_candidates.append([percentage, y, x])
        if percentage > max_accuracy:
            max_accuracy = percentage
            best_coord = coord
            best_scan = scan_image
            best_tf_img = tf_orig_scan
            best_overlay = overlay_img
            best_tf_robot = tf_robot_pose
            best_scan_center = [y, x]
            best_theta_degrees = theta_degrees
            if max_accuracy >= threshold_accuracy:
                break


    end_time = time.perf_counter()
    if best_coord is None:
        print("ERROR: 未找到任何有效候选位姿")
        return None
    x, y = best_coord
    total_time = (end_time - st_time) * 1000

    # print("Sim scan time: ", sim_scan_time, " Percentage: ", (sim_scan_time / total_time) * 100)
    # print("Matching time: ", matching_time, " Percentage: ", (matching_time / total_time) * 100)
    # print("Scoring time: ", scoring_time, " Percentage: ", (scoring_time / total_time) * 100)

    '''
    Time taken: ms 869.0479400002005
    Sim scan time:  694.8894949991882      Percentage:  79.95985756539827    (It can be improved!!!!)
    Matching time:  159.97490800145897     Percentage:  18.408064807267372
    Scoring time:  1.8264749996887986      Percentage:  0.2101696483727212
    '''


    cv2.circle(best_overlay, (int(best_tf_robot[0]), int(best_tf_robot[1])), 4, (255, 0, 0), -1)

    sorted_candidates = sorted(all_candidates, key = lambda x:x[0])


    robot_on_map = np.array(best_scan_center) + (np.array(best_tf_robot) - np.array(robot_coord))
    vector_length = 20 #px


    # 转世界坐标 (m)
    if map_origin is None:
        map_origin = np.array([0.0, 0.0])
    else:
        map_origin = np.array(map_origin, dtype=float)

    robot_in_map_meters = _map_pixel_to_world(
        robot_on_map, map_image.shape[0], map_resolution, map_origin,
        map_pose_offset)
    robot_angle_in_map = math.radians(-best_theta_degrees)

    print(f"[DEBUG] ORB 完成: f1={max_accuracy:.1f}, tested={iters}/{len(random_coords)}, "
          f"elapsed={total_time:.0f}ms, map=({robot_in_map_meters[0]:.2f},"
          f"{robot_in_map_meters[1]:.2f},{math.degrees(robot_angle_in_map):.1f}°)")

    if show_plot:
        fig, axs = plt.subplots(1, 6, figsize=(12, 4))
        axs[0].imshow(orig_scan_img, cmap='gray')
        axs[0].set_title('Laser Scan Image')
        axs[0].axis('off')
        axs[1].imshow(best_scan, cmap='gray')
        axs[1].set_title('Best Match simulated-scan')
        axs[1].axis('off')
        axs[2].imshow(best_tf_img, cmap='gray')
        axs[2].set_title('transformed laser-scan')
        axs[2].axis('off')
        axs[3].imshow(best_overlay, cmap='gray')
        axs[3].set_title('Real/Sim scan overlay')
        axs[3].axis('off')
        axs[4].imshow(candidate_area_to_show, cmap='gray')
        axs[4].set_title('Candidate Area(Red), samples(Green)')
        axs[4].axis('off')
        map_image_bgr = cv2.cvtColor(map_image, cv2.COLOR_GRAY2BGR)
        cv2.circle(map_image_bgr, (int(robot_on_map[0]), int(robot_on_map[1])), 10, (0, 0, 255), -1)
        end_x = int(robot_on_map[0] + vector_length * math.cos(math.radians(best_theta_degrees)))
        end_y = int(robot_on_map[1] + vector_length * math.sin(math.radians(best_theta_degrees)))
        cv2.line(map_image_bgr, (int(robot_on_map[0]), int(robot_on_map[1])), (end_x, end_y), (0, 0, 255), 2)
        axs[5].imshow(map_image_bgr, cmap='gray')
        axs[5].set_title('Robot Position Estimate')
        axs[5].axis('off')
        plt.tight_layout()
        plt.show()

    return (robot_in_map_meters[0], robot_in_map_meters[1], robot_angle_in_map, max_accuracy)
