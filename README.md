# AUTO_NAV2_VORONOI

基于 ROS 2 Humble 的 Gazebo 仿真导航工程，核心特性：

- **双定位分工**：AMCL 一次性全局定位 + XFeat 视觉融合持续里程计
- **Voronoi 全局路径规划**：基于 GVD 骨架搜索 + B-spline 平滑
- **Pure Pursuit 路径跟踪**：自适应前视距离 + 卡滞恢复
- **栅格地图融合**：静态 PGM + 实时动态障碍物
- **随机初始位姿**：每次启动随机 spawn 到地图安全空闲区域
- **自动导航评估**：批量随机目标点 + 中文评估报告

## 环境准备

```bash
cd ~/project/auto_nav2
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 快速启动

### 主方案（XFeat 视觉融合 + AMCL 初始定位）

```bash
ros2 launch rtabmap_localization_bringup gazebo_nav_xfeat_odometry.launch.py
```

每次启动会：
1. 读取地图随机选安全空闲位姿
2. Gazebo 用随机位姿 spawn 机器人
3. AMCL 用同一位姿初始化，快速收敛
4. `amcl_init_bridge` 锁定 `map→odom` 静态 TF
5. XFeat 融合里程计接管后续导航

### 纯底盘里程计方案（无视觉，无 AMCL）

```bash
ros2 launch gazebo_modele gazebo_nav_web.launch.py
```

### 网页控制

```text
http://<本机IP>:8080
```

---

## 定位架构

> **2D 雷达 + AMCL = GPS（一次性全局定位）**
> **XFeat 视觉融合 = 高精度计步器（持续局部里程计）**

两者分工明确，互不冲突：

```mermaid
flowchart TD
    subgraph 启动阶段
        A[读 PGM 地图] --> B[随机选空闲位姿]
        B --> C[Gazebo spawn 机器人]
        B --> D[AMCL 初始化粒子]
    end

    subgraph 全局定位 - AMCL
        D --> E[AMCL 匹配 /scan + /map]
        E --> F{协方差收敛?}
        F -- 是 --> G[amcl_init_bridge 锁定 map→odom TF]
    end

    subgraph 持续导航 - XFeat
        H[RGB-D 相机] --> I[XFeat 提特征 + PnP]
        I --> J[输出 /xfeat/delta_odom 局部增量]
        K[/odom 轮式] --> L[odom_fusion_node 增量融合]
        J --> L
        L --> M[/localized_odom 融合里程计]
        M --> N[Pure Pursuit 路径跟踪]
    end
```

TF 树：

```
map ──(amcl_init_bridge 静态)──→ odom ──(odom_tf_bridge)──→ base_footprint
```

### 关键节点

| 节点                  | 包                            | 职责                               |
| --------------------- | ----------------------------- | ---------------------------------- |
| `static_map_server`   | `nav_slam`                    | 发布 PGM 地图到 `/map`，仅供 AMCL   |
| `odom_tf_bridge`      | `gazebo_modele`               | 监听 `/localized_odom`，发 TF       |
| `xfeat_rgbd_odometry` | `rtabmap_localization_bringup` | XFeat 视觉里程计 → `/xfeat/delta_odom` |
| `odom_fusion_node`    | `rtabmap_localization_bringup` | 轮式 + XFeat 增量融合 → `/localized_odom` |
| `amcl`                | `nav2_amcl`                   | 蒙特卡洛全局定位（不发 TF）          |
| `amcl_init_bridge`    | `nav_slam`                    | AMCL 收敛后锁定 `map→odom` 静态 TF   |
| `map_pub`             | `nav_slam`                    | 静态地图 + 动态障碍 → `/combined_grid` |
| `voronoi`             | `nav2_voronoi_planner`        | Voronoi 路径规划                    |
| `start_nav`           | `nav_slam`                    | Pure Pursuit 跟踪 → `/cmd_vel`      |

### 地图话题分离

| 话题             | 来源                 | 用途               | 内容                 |
| ---------------- | -------------------- | ------------------ | -------------------- |
| `/map`           | `static_map_server`  | AMCL 全局定位      | 纯静态 PGM           |
| `/combined_grid` | `map_pub`            | Voronoi 规划 + RViz | 静态地图 + 动态障碍物 |

### XFeat 增量融合状态

| 状态         | 含义                              |
| ------------ | --------------------------------- |
| `base_only`  | 此步未使用 XFeat（超时/无观测）    |
| `fused`      | 此步使用了 XFeat 增量修正          |
| `rejected`   | XFeat 与底盘增量差异过大，丢弃     |

调试 CSV：`/home/xu/xfeat_pose/sim_odom_fusion_debug.csv`

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

| 参数       | 值                |
| ---------- | ----------------- |
| 采样点数   | 360（1° 分辨率）   |
| 角度范围   | -180° ~ 180°      |
| 距离范围   | 0.1m ~ 50m        |
| 更新率     | 10Hz              |

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
| `src/rtabmap_localization_bringup/launch/gazebo_nav_xfeat_odometry.launch.py` | 主启动（双定位方案） |
| `src/gazebo_modele/launch/gazebo.launch.py` | Gazebo 启动 |
| `src/nav_slam/launch/2dpoints.launch.py` | 导航核心启动 |
| `src/nav_slam/config/amcl_params.yaml` | AMCL 参数 |
| `src/nav_slam/nav_slam/amcl_init_bridge.py` | AMCL 收敛→TF 锁定 |
| `src/nav_slam/nav_slam/static_map_server.py` | 静态地图 → `/map` |
| `src/nav_slam/nav_slam/start_nav.py` | Pure Pursuit 路径跟踪 |
| `src/nav_slam/nav_slam/map_pub.py` | 障碍物栅格 → `/combined_grid` |
| `src/rtabmap_localization_bringup/rtabmap_localization_bringup/xfeat_rgbd_odometry.py` | XFeat 视觉里程计 |
| `src/rtabmap_localization_bringup/rtabmap_localization_bringup/odom_fusion_node.py` | 里程计增量融合 |
| `src/gazebo_modele/gazebo_modele/odom_tf_bridge.py` | odom→TF 桥接 |
| `src/gazebo_modele/urdf/model.urdf` | 机器人模型 |
| `src/nav2_voronoi_planner/src/voronoi_node.cpp` | Voronoi 规划调度 |
| `src/nav_eval/launch/auto_challenge_eval.launch.py` | 自动评估启动 |
| `docs/voronoi_route_improvement_record.md` | Voronoi 路径优化记录 |
| `对比.md` | XFeat/RTAB-Map 链路对比 + 双定位架构文档 |

---

## 参考仓库

- <https://github.com/Ming2zun/Pure-tracking-slam-automatic-navigation-system>
- <https://github.com/dxs1224/voronoi_planner_ros2>
- <https://github.com/Hongtai-Yuan/Voronoi_Planner_ROS2>
