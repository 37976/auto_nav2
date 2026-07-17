#!/usr/bin/env python3
"""
Map saver utility: convert OccupancyGrid to PGM (P5 binary) + YAML files.
Used by slam_controller to persist SLAM maps.
"""
import os
import yaml
import numpy as np
import cv2


def save_occupancy_grid(grid_msg, filename, directory):
    """
    Save a nav_msgs/OccupancyGrid message as PGM + YAML files.

    Args:
        grid_msg: nav_msgs/OccupancyGrid message
        filename: base filename without extension (e.g. "my_map")
        directory: directory to save files in

    Returns:
        (pgm_path, yaml_path) tuple of full file paths

    Raises:
        ValueError if grid_msg has no data
        IOError on write failure
    """
    if grid_msg is None or not grid_msg.data:
        raise ValueError("OccupancyGrid message is empty")

    width = grid_msg.info.width
    height = grid_msg.info.height
    resolution = grid_msg.info.resolution
    origin = grid_msg.info.origin

    os.makedirs(directory, exist_ok=True)

    # Convert occupancy data to PGM image (P5 = grayscale binary)
    # Occupancy: 0=free(white), 100=occupied(black), -1=unknown(gray)
    pgm_data = np.full((height, width), 205, dtype=np.uint8)  # default: unknown gray

    for i, cell in enumerate(grid_msg.data):
        y = i // width
        x = i % width
        img_y = height - 1 - y  # flip Y for image coords
        if cell == 0 or (cell < 0):  # free or unknown
            if cell < 0:
                pgm_data[img_y, x] = 205  # unknown: gray
            else:
                pgm_data[img_y, x] = 254  # free: near-white
        elif cell >= 100:
            pgm_data[img_y, x] = 0  # occupied: black
        elif cell > 0:
            # Intermediate occupancy: gradient
            pgm_data[img_y, x] = int(254 - (cell / 100.0) * 254)

    # Write PGM file
    pgm_path = os.path.join(directory, filename + ".pgm")
    cv2.imwrite(pgm_path, pgm_data)

    # Write YAML file
    yaml_path = os.path.join(directory, filename + ".yaml")
    yaml_data = {
        "image": filename + ".pgm",
        "resolution": float(resolution),
        "origin": [
            float(origin.position.x),
            float(origin.position.y),
            0.0,
        ],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    with open(yaml_path, "w") as f:
        yaml.dump(yaml_data, f, default_flow_style=False)

    return pgm_path, yaml_path
