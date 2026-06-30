# AUTO_NAV2_VORONOI

基于 ROS 2 Humble 的 Gazebo 仿真导航工程，核心特性：

- **ORB 激光全局定位**：LiDAR 扫描图与地图 ORB 特征匹配，一次性算出机器人在地图上的精确位姿
- **ORB 持续定位修正**：导航过程中周期性地将激光扫描与局部地图做 ORB 匹配，持续纠正里程计漂移
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
6. `orb_map_matcher` 持续用激光与地图做 ORB 匹配，周期性纠正里程计漂移

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
启动阶段 (一次性全局定位):
  PGM 地图 → 随机出生点 → Gazebo spawn → 等 3 帧雷达
                                              ↓
                                   ORB 特征匹配 (320×320 扫描图 vs 全局地图)
                                              ↓
                                    算出 (x, y, yaw) 精确位姿
                                              ↓
                                   锁定 map→odom 静态 TF
                                              ↓
持续导航 (周期性局部修正):
  /odom (轮式) + /orb/delta_odom (ORB 局部修正) → odom_fusion_node
                                              ↓
                                      /localized_odom (融合里程计, odom 系)
                                              ↓
                            odom_to_map_relay (查 map→odom TF 变换到 map 系)
                                              ↓
                                       /odom_in_map (map 系)
                                              ↓
                               voronoi_node (路径规划) + start_nav (跟踪)
```

**核心思想**：
- **启动阶段**：ORB 全局定位搜索全图，提供初始绝对位姿（类似 GPS 冷启动）
- **持续导航**：`orb_map_matcher` 每 2 秒用当前激光扫描与**当前估计位姿附近的局部地图**（±9m）做 ORB 匹配，搜索范围小、速度快，持续纠正里程计累积漂移（类似 GPS 持续修正）

TF 树：

```
map ──(lidar_global_localize 静态)──→ odom ──(odom_tf_bridge)──→ base_footprint ──(robot_state_publisher)──→ 各传感器
```

### 关键节点

| 节点 | 包 | 职责 |
|------|-----|------|
| `lidar_global_localize` | `nav_slam` | ORB 全局定位 → 锁定 `map→odom` 静态 TF |
| `orb_map_matcher` | `nav_slam` | 持续 ORB 扫描-局部地图匹配 → `/orb/delta_odom` |
| `static_map_server` | `nav_slam` | 发布 PGM 地图到 `/map` |
| `map_once_relay` | `nav_slam` | `/map` 转发一次到 `/map_for_amcl`（transient_local） |
| `odom_tf_bridge` | `gazebo_modele` | 监听 `/localized_odom`，发 `odom→base_footprint` TF |
| `odom_to_map_relay` | `nav_slam` | 将 odom 系里程计变换到 map 系 → `/odom_in_map` |
| `odom_fusion_node` | `rtabmap_localization_bringup` | 轮式 + ORB 增量融合 → `/localized_odom` |
| `map_pub` | `nav_slam` | 静态地图 + 动态障碍 → `/combined_grid` |
| `laser_scan_to_points` | `nav_slam` | 激光扫描转 map 系点云 |
| `voronoi` | `nav2_voronoi_planner` | Voronoi 路径规划（订阅 `/odom_in_map`） |
| `start_nav` | `nav_slam` | Pure Pursuit 跟踪 → `/cmd_vel`（订阅 `/odom_in_map`） |

### 里程计数据流

| 话题 | 坐标系 | 用途 |
|------|--------|------|
| `/odom` | odom | Gazebo 地面真值（底盘累积） |
| `/orb/delta_odom` | base_footprint | ORB 扫描-地图匹配增量修正 |
| `/localized_odom` | odom | 融合里程计（轮式 + ORB 修正） |
| `/odom_in_map` | map | map 系里程计 → 给 voronoi 和 start_nav |

### 地图话题分离

| 话题 | 来源 | 用途 |
|------|------|------|
| `/map` | `static_map_server` | 供 `map_once_relay` 和 `lidar_global_localize` 参考 |
| `/combined_grid` | `map_pub` | Voronoi 规划 + RViz 显示（静态地图 + 动态障碍物） |

### ORB 增量融合状态

| 状态 | 含义 |
|------|------|
| `base_only` | 此步未使用 ORB 修正（超时/无匹配） |
| `fused` | 此步使用了 ORB 增量修正 |
| `rejected` | ORB 修正与底盘增量差异过大，丢弃 |

调试 CSV：`/home/xu/xfeat_pose/sim_odom_fusion_debug.csv`

---

## ORB 定位原理

系统使用两阶段 ORB 定位：

### 1. 一次性全局定位 (`lidar_global_localize`)

启动时运行一次，搜索全图：

1. 将 360 点 LiDAR 扫描渲染为 320×320 灰度图（8m 范围, 0.05m/px）
2. 在**全局地图**上用距离变换筛选候选区域（离障碍物 `min_distance ± 0.15m` 的环带）
3. 随机采样候选位置，对每个位置模拟 LiDAR 扫描
4. ORB 特征提取 + BFMatcher 匹配真实扫描 vs 模拟扫描
5. RANSAC 估计仿射变换 + F1 打分，选最高分结果
6. 地图越大自动增加采样数（自适应 30 ~ 250 次, 上限 7s）

ORB 定位结果直接锁定 `map→odom` 静态 TF（含 x, y, yaw），不再经过 AMCL 粒子滤波。

### 2. 持续局部匹配 (`orb_map_matcher`)

导航过程中周期性运行，仅搜索**当前位姿附近局部区域**（±9m）：

1. 渲染当前激光扫描为 320×320 灰度图（与全局定位相同管线）
2. 根据当前估计位姿，从静态地图中**裁取局部子图**（~18m × 18m）
3. 在局部子图内运行 ORB 匹配（迭代上限 50 次，最低 F1 30%）
4. 将匹配得到的绝对位姿与当前估计位姿做差，经 P 控制器（增益 0.3）转为局部修正增量
5. 发布到 `/orb/delta_odom`，由 `odom_fusion_node` 融合

| | 全局定位 | 持续匹配 |
|---|---|---|
| 触发 | 启动时一次 | 每 2 秒 |
| 搜索范围 | 全图 | 当前位姿 ±9m |
| 迭代次数 | 30~250（自适应） | 固定 50 |
| 耗时 | ~0.9~7s | ~0.5~1s |
| 输出 | 静态 map→odom TF | Odometry delta |

---

## 常用启动参数

```bash
ros2 launch rtabmap_localization_bringup gazebo_nav_xfeat_odometry.launch.py \
  world_name:=gpt.world \
  start_nav_rviz:=true \
  start_web_ui:=false \
  start_moving_obstacle:=false
```

ORB 持续定位参数：

```bash
ros2 launch rtabmap_localization_bringup gazebo_nav_xfeat_odometry.launch.py \
  orb_match_period_sec:=2.0 \
  orb_max_iterations:=50 \
  orb_min_f1_score:=30.0 \
  orb_gain_xy:=0.3 \
  orb_gain_yaw:=0.3
```

ORB 全局定位参数（可选）：

```bash
ros2 launch rtabmap_localization_bringup gazebo_nav_xfeat_odometry.launch.py \
  static_map_yaml:=/path/to/map.yaml
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

## 定位精度测试

`test_orb_localization.py` 反复随机 spawn 机器人到地图空闲区域，对比 ORB 定位结果与真实 spawn 位姿，统计定位误差。

```bash
# 100 次测试，每次 ORB 成功后采集 15 秒跟踪数据
python3 test_orb_localization.py -n 100 -d 15

# 快速验证（5 次）
python3 test_orb_localization.py -n 5 -d 5

# 查看测试计划但不执行
python3 test_orb_localization.py --dry-run
```

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `-n, --runs` | 测试次数 | 100 |
| `-d, --duration` | 每次 ORB 完成后采集跟踪数据时长（秒） | 15 |
| `-t, --timeout` | 等待 ORB 超时（秒） | 180 |
| `-v, --verbose` | 打印所有 launch stdout | off |
| `--fast-cleanup` | 加速清理，跳过端口等待 | off |

输出：`test_results/orb_results_<时间戳>.csv`（逐次详细） + `orb_summary_<时间戳>.csv`（统计汇总）

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
| `src/nav_slam/nav_slam/orb_map_matcher.py` | 持续 ORB 扫描-地图匹配 → `/orb/delta_odom` |
| `src/nav_slam/nav_slam/odom_to_map_relay.py` | odom→map 坐标转发 |
| `src/nav_slam/nav_slam/static_map_server.py` | 静态地图 → `/map` |
| `src/nav_slam/nav_slam/start_nav.py` | Pure Pursuit 路径跟踪 |
| `src/nav_slam/nav_slam/map_pub.py` | 障碍物栅格 → `/combined_grid` |
| `src/kidnapped_robot_finder/global_localizer/kidnap_solver.py` | ORB 核心求解（自适应迭代） |
| `src/kidnapped_robot_finder/global_localizer/feature_matching.py` | ORB 特征 + RANSAC 匹配 |
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
