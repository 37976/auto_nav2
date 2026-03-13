# Pure-tracking-slam-automatic-navigation-system
simulate，ros2，gazebo，navigation，slam



<!--
 * @作者: boxing
 * @b号: 喵了个水蓝蓝
 * @描述: README
-->
## 注意当前分支代码为humble版本

# 基于ROS2实现的差速机器人，slam（建图定位），导航控制（纯追踪）,实现用voronoi全局规划路径

## 安装运行

```
git clone https://github.com/Ming2zun/Pure-tracking-slam-automatic-navigation-system.git
```

## 运行测试
###  启动差速仿真
```
ros2 launch gazebo_modele gazebo.launch.py
```

###  启动导航
```
ros2 launch nav_slam 2dpoints.launch.py
```

