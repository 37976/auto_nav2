# superpoint_rtabmap_bridge

Bridge package for scheme A:

- subscribe RGB image, aligned depth and camera info
- run SuperPoint locally
- back-project valid depth for each keypoint
- publish `rtabmap_msgs/RGBDImage` with:
  - raw RGB / depth
  - local keypoints
  - local 3D points
  - compressed descriptors

The intended RTAB-Map side uses:

- `subscribe_rgbd:=true`
- `rgbd_sync:=false`
- `rgbd_topic:=/superpoint/rgbd_image`

This first version keeps RTAB-Map's own graph optimization and loop validation,
while replacing the feature extraction stage with SuperPoint.

## Launch

Bridge only:

```bash
ros2 launch superpoint_rtabmap_bridge superpoint_rtabmap_bridge.launch.py
```

Bridge + RTAB-Map:

```bash
ros2 launch superpoint_rtabmap_bridge superpoint_rtabmap_rgbd.launch.py \
  localization:=false \
  database_path:=$HOME/.ros/rtabmap_superpoint.db
```

Limit the bridge output to 1 frame per second:

```bash
ros2 launch superpoint_rtabmap_bridge superpoint_rtabmap_rgbd.launch.py \
  output_rate_hz:=1.0 \
  localization:=false \
  database_path:=$HOME/.ros/rtabmap_superpoint.db
```

Camera-only mode is the default for `superpoint_rtabmap_rgbd.launch.py`:

- `visual_odometry:=true`
- `subscribe_scan:=false`
- `frame_id:=camera_link`

When wheel odometry and lidar are available later, switch to:

```bash
ros2 launch superpoint_rtabmap_bridge superpoint_rtabmap_rgbd.launch.py \
  visual_odometry:=false \
  odom_topic:=/odom \
  subscribe_scan:=true \
  scan_topic:=/scan \
  frame_id:=base_footprint
```

Typical real-robot localization chain:

- wheel odometry on `/odom`
- lidar scan on `/scan`
- D435 RGB on `/camera/camera/color/image_raw`
- D435 aligned depth on `/camera/camera/aligned_depth_to_color/image_raw`
- camera info on `/camera/camera/color/camera_info`
