# AUTO_NAV2_VORONOI

基于 ROS 2 Humble 的 Gazebo 仿真导航工程，包含 Voronoi 全局规划、动态/静态栅格融合，以及手机网页控制。

## 快速启动

```bash
cd ~/project/auto_nav2
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch gazebo_modele gazebo_nav_web.launch.py
```

如需机器人自己开热点并在弹窗里显示“连接 Wi-Fi”二维码：

```bash
ros2 launch gazebo_modele gazebo_nav_web.launch.py start_hotspot:=true
```

这条命令会同时启动：

- Gazebo 仿真环境
- Voronoi 导航链路
- RViz
- 网页控制面板

网页默认地址：

```text
http://<本机IP>:8080
```

## 常用命令

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

## 当前网页能力

- 导航页查看地图、路径、目标点
- 手动页摇杆控制 `/cmd_vel`
- 网页可切换 `导航 / 手动` 模式
- 地图支持拖拽、滚轮缩放、双指缩放
- 点选目标后再确认，避免误发导航

说明：当前 Gazebo 工程里没有接入机器人相机图像话题，所以网页相机区域会显示等待相机，这属于预期行为。

## 参考仓库

- https://github.com/Ming2zun/Pure-tracking-slam-automatic-navigation-system
- https://github.com/dxs1224/voronoi_planner_ros2
- https://github.com/Hongtai-Yuan/Voronoi_Planner_ROS2


说明：仿真网页的雷达在线状态来自 `/scan`。
