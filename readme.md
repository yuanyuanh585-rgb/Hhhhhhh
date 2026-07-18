# Origincar 仿真、建图与导航工作区

这是一个 ROS 2 Humble 工作区，用于 Origincar 阿克曼小车的建图、定位和 Nav2 导航；实车入口使用 `real_` 前缀，仿真入口使用 `sim_` 前缀，均保留在 `origincar_bringup/launch/` 下。

## 快速开始

```bash
cd /home/xjl/digua
colcon build --packages-select origincar_msg origincar_base lslidar_msgs lslidar_driver origincar_mapping origincar_navigation origincar_strategy origincar_bringup
source install/setup.bash
```

启动实车底盘和传感器：

```bash
ros2 launch origincar_bringup real_robot.launch.py
```

实车建图：

```bash
ros2 launch origincar_bringup real_mapping.launch.py
```

实车导航：

```bash
ros2 launch origincar_bringup real_navigation.launch.py
```

只启动仿真：

```bash
ros2 launch origincar_bringup sim.launch.py
```

启动仿真和在线建图：

```bash
ros2 launch origincar_bringup sim_mapping.launch.py
```

启动仿真、地图服务器、纯激光全局定位和 Nav2：

```bash
ros2 launch origincar_bringup sim_navigation.launch.py
```

导航默认不打开 RViz。查看地图、小车和发送目标点：

```bash
ros2 launch origincar_bringup real_view_navigation.launch.py
```

## 文档

- 统一使用说明：`doc/usage.md`
- 建图操作指南：`doc/mapping_guide.md`
- 导航操作指南：`doc/navigation_usage.md`
- 实车部署操作手册：`doc/小车实车部署操作手册.md`
- 建图与导航计划：`doc/navigation_plan.md`

导航参数位于 `src/origincar_navigation/config/nav2_params.yaml`。策略控制器位于 `origincar_strategy`。地图和路线点按场景成套保存：

```text
maps/sim/map.yaml + maps/sim/global_routes.yaml
maps/real/map.yaml + maps/real/global_routes.yaml
```

临时更换地图和路线文件时使用：

```bash
ros2 launch origincar_bringup real_navigation.launch.py map:=/path/to/map.yaml routes_file:=/path/to/global_routes.yaml
```
