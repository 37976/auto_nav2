# AUTO_NAV2_VORONOI

基于 ROS 2 Humble 的 Gazebo 仿真导航工程，核心特性：

- **ORB 激光全局定位**：LiDAR 扫描图与地图 ORB 特征匹配，一次性算出机器人在地图上的精确位姿
- **XFeat 视觉融合里程计**：RGB-D 相机 + 轮式编码器增量融合，持续提供高精度里程计
- **Voronoi 全局路径规划**：基于 GVD 骨架搜索 + B-spline 平滑
- **Pure Pursuit 路径跟踪**：自适应前视距离 + 卡滞恢复
- **栅格地图融合**：静态 PGM + 实时动态障碍物
- **随机初始位姿**：每次启动随机 spawn 到地图安全空闲区域
- **自适应地图尺度**：定位采样数根据地图大小自动缩放，支持 100 m² ~ 2700 m² 地图

## 环境准备

```bash
cd ~/project/auto_nav2
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 快速启动

```bash
ros2 launch rtabmap_localization_bringup gazebo_nav_xfeat_odometry.launch.py
```

每次启动会：

1. 读取地图随机选安全空闲位姿
2. Gazebo 用随机位姿 spawn 机器人
3. LiDAR 采集 3 帧后，ORB 匹配算出机器人在地图上的精确 (x, y, yaw)
4. `lidar_global_localize` 锁定 `map→odom` 静态 TF
5. Voronoi 规划 + Pure Pursuit 跟踪接管后续导航
6. XFeat 融合里程计持续修正轮式编码器漂移

### 仅 Gazebo + 传感器（无导航）

```bash
ros2 launch rtabmap_localization_bringup gazebo_sensors_only.launch.py
```

### 网页控制

```text
http://<本机IP>:8080
```

---

## 定位架构

```
启动阶段:
  PGM 地图 → 随机出生点 → Gazebo spawn → 等 3 帧雷达
                                              ↓
                                   ORB 特征匹配 (320×320 扫描图 vs 全局地图)
                                              ↓
                                    算出 (x, y, yaw) 精确位姿
                                              ↓
                                   锁定 map→odom 静态 TF
                                              ↓
持续导航:
  /odom (轮式) + /xfeat/delta_odom (视觉) → odom_fusion_node
                                              ↓
                                      /localized_odom (融合里程计, odom 系)
                                              ↓
                            odom_to_map_relay (查 map→odom TF 变换到 map 系)
                                              ↓
                                       /odom_in_map (map 系)
                                              ↓
                               voronoi_node (路径规划) + start_nav (跟踪)
```

**核心思想**：ORB 匹配提供绝对位置（类似 GPS 定位），XFeat 融合里程计提供连续局部运动（类似计步器）。ORB 只做一次初始定位，后续由融合里程计维持位置。

TF 树：

```
map ──(lidar_global_localize 静态)──→ odom ──(odom_tf_bridge)──→ base_footprint ──(robot_state_publisher)──→ 各传感器
```

### 关键节点

| 节点 | 包 | 职责 |
|------|-----|------|
| `lidar_global_localize` | `nav_slam` | ORB 全局定位 → 锁定 `map→odom` 静态 TF |
| `static_map_server` | `nav_slam` | 发布 PGM 地图到 `/map` |
| `map_once_relay` | `nav_slam` | `/map` 转发一次到 `/map_for_amcl`（transient_local） |
| `odom_tf_bridge` | `gazebo_modele` | 监听 `/localized_odom`，发 `odom→base_footprint` TF |
| `odom_to_map_relay` | `nav_slam` | 将 odom 系里程计变换到 map 系 → `/odom_in_map` |
| `xfeat_rgbd_odometry` | `rtabmap_localization_bringup` | XFeat 视觉里程计 → `/xfeat/delta_odom` |
| `odom_fusion_node` | `rtabmap_localization_bringup` | 轮式 + XFeat 增量融合 → `/localized_odom` |
| `map_pub` | `nav_slam` | 静态地图 + 动态障碍 → `/combined_grid` |
| `laser_scan_to_points` | `nav_slam` | 激光扫描转 map 系点云 |
| `voronoi` | `nav2_voronoi_planner` | Voronoi 路径规划（订阅 `/odom_in_map`） |
| `start_nav` | `nav_slam` | Pure Pursuit 跟踪 → `/cmd_vel`（订阅 `/odom_in_map`） |

### 里程计数据流

| 话题 | 坐标系 | 用途 |
|------|--------|------|
| `/odom` | odom | Gazebo 地面真值（底盘累积） |
| `/xfeat/delta_odom` | odom | XFeat 视觉增量修正 |
| `/localized_odom` | odom | 融合里程计（轮式 + 视觉） |
| `/odom_in_map` | map | map 系里程计 → 给 voronoi 和 start_nav |

### 地图话题分离

| 话题 | 来源 | 用途 |
|------|------|------|
| `/map` | `static_map_server` | 供 `map_once_relay` 和 `lidar_global_localize` 参考 |
| `/combined_grid` | `map_pub` | Voronoi 规划 + RViz 显示（静态地图 + 动态障碍物） |

### XFeat 增量融合状态

| 状态 | 含义 |
|------|------|
| `base_only` | 此步未使用 XFeat（超时/无观测） |
| `fused` | 此步使用了 XFeat 增量修正 |
| `rejected` | XFeat 与底盘增量差异过大，丢弃 |

调试 CSV：`/home/xu/xfeat_pose/sim_odom_fusion_debug.csv`

---

## ORB 全局定位原理

`lidar_global_localize` 使用 `kidnapped_robot_finder` 包的 ORB 特征匹配算法：

1. 将 360 点 LiDAR 扫描渲染为 320×320 灰度图（8m 范围, 0.05m/px）
2. 在地图上用距离变换筛选候选区域（离障碍物 `min_distance ± 0.15m` 的环带）
3. 随机采样候选位置，对每个位置模拟 LiDAR 扫描
4. ORB 特征提取 + BFMatcher 匹配真实扫描 vs 模拟扫描
5. RANSAC 估计仿射变换 + F1 打分，选最高分结果
6. 地图越大自动增加采样数（自适应 30 ~ 250 次, 上限 7s）

ORB 定位结果直接锁定 `map→odom` 静态 TF（含 x, y, yaw），不再经过 AMCL 粒子滤波。

---

## 常用启动参数

```bash
ros2 launch rtabmap_localization_bringup gazebo_nav_xfeat_odometry.launch.py \
  world_name:=gpt.world \
  start_nav_rviz:=true \
  start_web_ui:=false \
  start_moving_obstacle:=false
```

XFeat 参数：

```bash
ros2 launch rtabmap_localization_bringup gazebo_nav_xfeat_odometry.launch.py \
  top_k:=768 \
  detection_threshold:=0.05 \
  match_min_cossim:=0.65 \
  min_pnp_points:=6 \
  pnp_reproj_error:=8.0
```

融合参数：

```bash
ros2 launch rtabmap_localization_bringup gazebo_nav_xfeat_odometry.launch.py \
  correction_gain_xy:=0.15 \
  correction_gain_yaw:=0.10 \
  max_delta_translation_diff_m:=0.20 \
  max_delta_yaw_diff_deg:=20.0
```

ORB 定位参数（可选）：

```bash
ros2 launch rtabmap_localization_bringup gazebo_nav_xfeat_odometry.launch.py \
  map_yaml_path:=/path/to/map.yaml
```

---

## 其他启动方式

### Gazebo 单独

```bash
ros2 launch gazebo_modele gazebo.launch.py
```

### 导航单独

```bash
ros2 launch nav_slam 2dpoints.launch.py
ros2 launch nav_slam 2dpoints.launch.py start_web_ui:=false
ros2 launch nav_slam 2dpoints.launch.py start_nav_rviz:=false
```

### RTAB-Map（Gazebo）

```bash
ros2 launch rtabmap_localization_bringup gazebo_nav_rtabmap_localization.launch.py
ros2 launch rtabmap_localization_bringup gazebo_nav_superpoint_rtabmap_localization.launch.py
ros2 launch rtabmap_localization_bringup gazebo_nav_xfeat_rtabmap_localization.launch.py
```

### RTAB-Map（真实 D435）

```bash
ros2 launch rtabmap_localization_bringup real_d435_nav_rtabmap_localization.launch.py
ros2 launch rtabmap_localization_bringup real_d435_only_xfeat_rtabmap.launch.py
```

---

## 挑战场景

```bash
ros2 launch gazebo_modele gazebo_challenge_nav_web.launch.py
```

或传参：

```bash
ros2 launch gazebo_modele gazebo_nav_web.launch.py \
  world_name:=challenge_maze.world \
  static_map_yaml:=/home/xu/project/auto_nav2/src/nav_slam/map/challenge_maze.yaml
```

---

## 雷达配置

| 参数 | 值 |
|------|-----|
| 采样点数 | 360（1° 分辨率） |
| 角度范围 | -180° ~ 180° |
| 距离范围 | 0.1m ~ 50m |
| 更新率 | 10Hz |

---

## 自动评估

```bash
ros2 launch nav_eval auto_challenge_eval.launch.py
```

参数：

```bash
ros2 launch nav_eval auto_challenge_eval.launch.py \
  random_goal_count:=100 \
  random_seed:=42 \
  goal_timeout_sec:=180.0 \
  start_nav_rviz:=true
```

输出：`~/auto_nav2_eval/challenge_maze_auto`

---

## 网页功能

地图显示、路径显示、目标点确认、手动摇杆、暂停/继续、雷达点云

---

## 关键文件

| 文件 | 说明 |
|------|------|
| `src/rtabmap_localization_bringup/launch/gazebo_nav_xfeat_odometry.launch.py` | 主启动文件 |
| `src/rtabmap_localization_bringup/launch/gazebo_sensors_only.launch.py` | 仅 Gazebo + 传感器 |
| `src/gazebo_modele/launch/gazebo.launch.py` | Gazebo 启动 |
| `src/nav_slam/launch/2dpoints.launch.py` | 导航核心启动 |
| `src/nav_slam/nav_slam/lidar_global_localize.py` | ORB 全局定位 → `map→odom` TF |
| `src/nav_slam/nav_slam/odom_to_map_relay.py` | odom→map 坐标转发 |
| `src/nav_slam/nav_slam/static_map_server.py` | 静态地图 → `/map` |
| `src/nav_slam/nav_slam/start_nav.py` | Pure Pursuit 路径跟踪 |
| `src/nav_slam/nav_slam/map_pub.py` | 障碍物栅格 → `/combined_grid` |
| `src/kidnapped_robot_finder/global_localizer/kidnap_solver.py` | ORB 核心求解（自适应迭代） |
| `src/kidnapped_robot_finder/global_localizer/feature_matching.py` | ORB 特征 + RANSAC 匹配 |
| `src/rtabmap_localization_bringup/rtabmap_localization_bringup/xfeat_rgbd_odometry.py` | XFeat 视觉里程计 |
| `src/rtabmap_localization_bringup/rtabmap_localization_bringup/odom_fusion_node.py` | 里程计增量融合 |
| `src/gazebo_modele/gazebo_modele/odom_tf_bridge.py` | odom→TF 桥接 |
| `src/gazebo_modele/urdf/model.urdf` | 机器人模型 |
| `src/nav2_voronoi_planner/src/voronoi_node.cpp` | Voronoi 规划调度 |
| `src/nav_eval/launch/auto_challenge_eval.launch.py` | 自动评估启动 |

---

## 参考仓库

- <https://github.com/Ming2zun/Pure-tracking-slam-automatic-navigation-system>
- <https://github.com/dxs1224/voronoi_planner_ros2>
- <https://github.com/Hongtai-Yuan/Voronoi_Planner_ROS2>
