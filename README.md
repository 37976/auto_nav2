# AUTO_NAV2_VORONOI

基于 ROS 2 Humble 的 Gazebo 仿真导航工程，包含 Voronoi 全局规划、动态/静态栅格融合、手机网页控制，以及固定热点二维码接入。

## 环境准备

```bash
cd ~/project/auto_nav2
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 默认启动

默认场景是 `gpt.world`，会同时启动：Gazebo、Voronoi 导航、RViz、网页控制面板。

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

更复杂的测试场景已经准备好，包含更长走廊、更多门洞、绕行区和更密的静态障碍，适合压导航算法。

```bash
ros2 launch gazebo_modele gazebo_challenge_nav_web.launch.py
```

对应文件：
- world: `src/gazebo_modele/world/challenge_maze.world`
- map: `src/nav_slam/map/challenge_maze.yaml`

如果你想手动切 world 和地图，也可以直接用总启动传参数：

```bash
ros2 launch gazebo_modele gazebo_nav_web.launch.py \
  world_name:=challenge_maze.world \
  static_map_yaml:=/home/xu/project/auto_nav2/src/nav_slam/map/challenge_maze.yaml
```

## 常用变体

只启动 Gazebo：

```bash
ros2 launch gazebo_modele gazebo.launch.py
```

只启动导航和网页：

```bash
ros2 launch nav_slam 2dpoints.launch.py
```

启动导航但不打开网页：

```bash
ros2 launch nav_slam 2dpoints.launch.py start_web_ui:=false
```

启动导航但不打开 RViz：

```bash
ros2 launch nav_slam 2dpoints.launch.py start_nav_rviz:=false
```

启动默认场景但不开热点：

```bash
ros2 launch gazebo_modele gazebo_nav_web.launch.py start_hotspot:=false
```

## 网页能力

当前仿真网页支持：

- 导航页查看地图、路径、目标点
- 手动页摇杆控制 `/cmd_vel`
- 摇杆带轻微前后左右吸附
- 顶部按钮可暂停/继续当前导航
- 地图支持拖拽、滚轮缩放、双指缩放
- 点选目标后再确认，避免误发导航
- 手动页显示雷达实时点画面
- 支持热点二维码和网页二维码弹窗

说明：

- 这个仿真工程没有接入真实相机话题，所以网页里不显示相机区块。
- 仿真雷达网页显示来自 3D 点云话题 `/points_raw`，不是 2D `/scan`。
- 当前导航地图会融合静态地图和动态障碍，并对动态残留做较快清除。

## 主要启动文件

- `src/gazebo_modele/launch/gazebo_nav_web.launch.py`：默认总启动
- `src/gazebo_modele/launch/gazebo_challenge_nav_web.launch.py`：挑战场景总启动
- `src/gazebo_modele/launch/gazebo.launch.py`：Gazebo 启动，可切换 `world_name`
- `src/nav_slam/launch/2dpoints.launch.py`：导航 + 网页启动

## 参考仓库

- https://github.com/Ming2zun/Pure-tracking-slam-automatic-navigation-system
- https://github.com/dxs1224/voronoi_planner_ros2
- https://github.com/Hongtai-Yuan/Voronoi_Planner_ROS2
