# sp_orb_slam_localization

This package vendors the trimmed SuperPoint ORB-SLAM3 localization core and builds it inside the ROS 2 workspace.

## What it does

- subscribes to a monocular image topic
- runs `ORB_SLAM3::System::TrackMonocular`
- publishes:
  - `sp_orb_slam/pose`
  - `sp_orb_slam/odom`
- optionally publishes a TF from `map_frame` to `child_frame`

## Important limitations

- This package no longer depends on the external SuperPointSLAM3 repository at runtime.
- You still need `libtorch` available when building the package.
- Pure monocular visual SLAM is not a drop-in replacement for Nav2 localization.
  For better navigation accuracy, fuse the visual odometry/pose output with your
  wheel odometry and IMU instead of replacing them directly.

## Build

```bash
cd /home/xu/project/auto_nav2
colcon build --packages-select sp_orb_slam_localization \
  --cmake-args \
    -DSP_ORB_SLAM_TORCH_PREFIX=/path/to/libtorch \
    -DSP_ORB_SLAM_EXTRA_RPATH=/path/to/libtorch/lib
```

## Run

```bash
ros2 launch sp_orb_slam_localization sp_orb_slam_localization.launch.py \
  settings_file:=/absolute/path/to/your_camera.yaml \
  image_topic:=/camera/image_raw
```

For an Intel RealSense D435 color stream, you can start with the packaged example:

```bash
ros2 launch sp_orb_slam_localization sp_orb_slam_localization.launch.py \
  settings_file:=/home/xu/project/auto_nav2/src/sp_orb_slam_localization/config/d435_color.yaml \
  image_topic:=/camera/color/image_raw
```

Then replace the `Camera1.*` values in
`config/d435_color.yaml` with the exact numbers from:

```bash
ros2 topic echo /camera/color/camera_info --once
```

The packaged `ORBvoc.txt` and `superpoint.ts` are used by default, so you do not need to pass those paths unless you want to override them.

If you want to override the packaged SuperPoint model:

```bash
ros2 launch sp_orb_slam_localization sp_orb_slam_localization.launch.py \
  settings_file:=/absolute/path/to/your_camera.yaml \
  superpoint_model_file:=/absolute/path/to/superpoint.ts
```

## Nav2 integration suggestion

- Keep your wheel odometry and IMU.
- Feed `sp_orb_slam/odom` or `sp_orb_slam/pose` into `robot_localization` as an extra source.
- Let EKF or UKF produce the final fused pose for Nav2, instead of replacing `/odom` directly with monocular SLAM output.
