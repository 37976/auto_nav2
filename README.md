# AUTO_NAV2_VORONOI

基于 ROS 2 Humble 的 Gazebo 仿真导航工程，当前包含：

- Gazebo 差速机器人仿真
- Voronoi 全局路径规划
- 栅格地图融合与路径跟踪控制
- RViz 与网页端可视化/控制
- 自动导航评估包 `nav_eval`

当前工程已经针对 `challenge_maze.world` 做过一轮稳定性收敛，适合继续做参数调优、算法评估和学习型局部控制实验。

## 环境准备

```bash
cd ~/project/auto_nav2
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 快速启动

默认场景是 `gpt.world`，会同时启动 Gazebo、导航、RViz 和网页控制：

```bash
ros2 launch gazebo_modele gazebo_nav_web.launch.py
```

网页默认地址：

```text
http://<本机IP>:8080
```

如果需要机器人自己开热点，并在弹窗里显示“连接 Wi-Fi / 打开网页”二维码：

```bash
ros2 launch gazebo_modele gazebo_nav_web.launch.py start_hotspot:=true
```

## 挑战场景

挑战场景使用更复杂的迷宫地图：

- world: `src/gazebo_modele/world/challenge_maze.world`
- map: `src/nav_slam/map/challenge_maze.yaml`

直接启动挑战场景：

```bash
ros2 launch gazebo_modele gazebo_challenge_nav_web.launch.py
```

或者使用总启动传参：

```bash
ros2 launch gazebo_modele gazebo_nav_web.launch.py \
  world_name:=challenge_maze.world \
  static_map_yaml:=/home/xu/project/auto_nav2/src/nav_slam/map/challenge_maze.yaml
```

## 常用启动方式

只启动 Gazebo：

```bash
ros2 launch gazebo_modele gazebo.launch.py
```

只启动导航：

```bash
ros2 launch nav_slam 2dpoints.launch.py
```

说明：

- 这个启动现在会自动补起 `robot_state_publisher` 和 `joint_state_publisher`
- 所以即使不经过 Gazebo，总能在 RViz 里看到完整底盘和轮子连接

导航但不打开网页：

```bash
ros2 launch nav_slam 2dpoints.launch.py start_web_ui:=false
```

导航但不打开 RViz：

```bash
ros2 launch nav_slam 2dpoints.launch.py start_nav_rviz:=false
```

默认场景但不开热点：

```bash
ros2 launch gazebo_modele gazebo_nav_web.launch.py start_hotspot:=false
```

## 当前导航链路

这个工程现在不是 AMCL/SLAM 全局定位方案，而是仿真里程计导航链路：

1. Gazebo `diff_drive` 插件发布 `/odom`
2. `odom_map_tf.py` 发布 `map -> odom` 和 `odom -> base_footprint`
3. `map_pub.py` 结合静态地图与点云生成 `/combined_grid`
4. `voronoi_node` 基于 `/combined_grid` 规划 `/path`
5. `start_nav.py` 对 `/path` 做 Pure Pursuit 风格跟踪并发布 `/cmd_vel`

当前默认配置里：

- 规划地图默认会叠加静态地图与动态障碍层
- `voronoi_node` 不再发布 `/cmd_vel`
- `/cmd_vel` 由 `start_nav.py` 单独负责
- 轮子 TF 由 Gazebo `diff_drive` 直接发布，避免 RViz 轮子 frame 报红
- 如果不启 Gazebo，只跑 `2dpoints.launch.py`，则由 `joint_state_publisher` 提供静态轮子 joint state，保证 RViz 模型完整

## 当前稳定性修复

相对最初版本，当前工程已经做了这些关键修复：

- 机器人底盘 URDF 重做为更稳定的圆形差速底盘
- 关闭了仿真动态障碍的默认启动
- 规划地图支持通过 launch 参数开关动态障碍层，当前默认开启
- `start_nav.py` 增加了路径可见性检查，减少隔墙追点
- `start_nav.py` 增加了进度 watchdog 和恢复逻辑
- 新路径到来时会按当前位姿选择路径锚点，而不是一律从 `idx=0` 起步
- `voronoi_node` 不再与控制器争抢 `/cmd_vel`

## 自动评估

工程内新增了 `nav_eval` 包，可以自动启动挑战场景、自动随机发目标点并输出中文评估表。

一键评估：

```bash
ros2 launch nav_eval auto_challenge_eval.launch.py
```

这个 launch 默认会：

- 启动 `gazebo_nav_web.launch.py`
- 使用 `challenge_maze.world`
- 打开动态障碍层
- 关闭网页 UI
- 自动随机选择目标点
- 记录 `/odom`、`/cmd_vel`、`/path`、`/combined_grid` 等指标

默认输出目录：

```text
~/auto_nav2_eval/challenge_maze_auto
```

可直接调的常用参数：

```bash
ros2 launch nav_eval auto_challenge_eval.launch.py \
  random_goal_count:=100 \
  random_seed:=42 \
  goal_timeout_sec:=180.0 \
  start_nav_rviz:=true
```

如果想只开评估节点，不自动发目标：

```bash
ros2 launch nav_eval eval.launch.py output_dir:=~/auto_nav2_eval/manual
```

## 评估表字段

评估会输出中文 CSV 和 Markdown，主要字段包括：

- `状态`：成功 / 超时
- `耗时_秒`
- `实际路程_米`
- `最终目标距离_米`
- `最小障碍净距_米`
- `重规划次数`
- `速度样本数`
- `停滞时间_秒`
- `是否超时`
- `备注`

说明：

- 当前评估逻辑里，超时后不会自动切下一个目标，而是继续等待当前目标完成
- 所以有些样本会显示 `状态=超时`，但备注是“超时后进入目标容差范围”

## 当前挑战场景基线

基于 `challenge_maze.world` 的一组 100 次自动评估结果：

- 直接成功率：`94%`
- 最终到达率：`100%`
- 碰撞数：`0`
- 近碰数：`0`
- 平均耗时：约 `118s`
- 成功样本平均耗时：约 `113s`
- 平均最小障碍净距：约 `0.50m`

说明：

- 这组 100 次基线主要用于验证“长轮次运行稳定性”和“第 N 次后是否僵死”
- 如果重新打开动态障碍层，成功率、超时率和最小障碍净距可能会发生变化，建议重新跑一组评估表作为新基线

这说明当前版本已经能连续完成长轮次评估，主要剩余问题不再是“第 N 次直接卡死”，而更偏向：

- 长路径任务偶尔超过 `180s` 超时阈值
- 某些路段仍然会贴障碍走，安全余量还能继续优化

## 网页能力

当前网页支持：

- 地图显示、路径显示、目标点确认
- 手动摇杆控制 `/cmd_vel`
- 顶部按钮暂停/继续导航
- 地图拖拽、滚轮缩放、双指缩放
- 雷达点云显示
- 热点二维码和网页二维码弹窗

说明：

- 当前仿真工程没有接入真实相机话题，所以网页里不显示相机区块
- 仿真雷达网页显示来自 `/points_raw`

## 主要文件

- `src/gazebo_modele/launch/gazebo_nav_web.launch.py`：默认总启动
- `src/gazebo_modele/launch/gazebo_challenge_nav_web.launch.py`：挑战场景总启动
- `src/gazebo_modele/launch/gazebo.launch.py`：Gazebo 启动
- `src/gazebo_modele/urdf/model.urdf`：机器人模型
- `src/nav_slam/launch/2dpoints.launch.py`：导航主启动
- `src/nav_slam/nav_slam/start_nav.py`：路径跟踪控制
- `src/nav2_voronoi_planner/src/voronoi_node.cpp`：Voronoi 规划调度节点
- `src/nav_eval/launch/auto_challenge_eval.launch.py`：一键自动评估
- `src/nav_eval/nav_eval/nav_eval_node.py`：评估统计节点

## 参考仓库

- https://github.com/Ming2zun/Pure-tracking-slam-automatic-navigation-system
- https://github.com/dxs1224/voronoi_planner_ros2
- https://github.com/Hongtai-Yuan/Voronoi_Planner_ROS2
