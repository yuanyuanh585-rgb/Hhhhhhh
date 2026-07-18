# Origincar 导航使用说明

本文档说明如何在已经完成建图的基础上，使用 Nav2 控制小车进行定位和导航。真实小车部署优先阅读 `小车实车部署操作手册.md`。

## 1. 前置条件

导航前需要先满足：

- Gazebo 仿真可以正常启动，小车能发布 `/scan`、`/odom`、`/tf`。
- 已经完成建图，并保存地图和路线点到同一个场景目录：

```text
/home/xjl/digua/maps/sim/map.yaml
/home/xjl/digua/maps/sim/map.pgm
/home/xjl/digua/maps/sim/global_routes.yaml
```

- 已安装 Nav2 依赖：

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-map-server ros-humble-nav2-amcl
```

## 2. 启动仿真

终端 1：

```bash
ros2 launch origincar_bringup sim_navigation.launch.py
```

这个统一入口会依次启动：

- Gazebo 仿真和小车
- `origincar_mapping` 中的 `map_server` 和激光定位
- Nav2 导航服务
- `origincar_strategy` 路线策略控制器

注意：导航启动不会自动打开 RViz。如果要同时查看地图和小车，另开终端启动：

```bash
ros2 launch origincar_bringup sim_view_navigation.launch.py
```

`sim_view_navigation.launch.py` 默认只打开 RViz 和路线 marker，不发布 `/map`，也不重复发布机器人 TF。`/map`、`/robot_description`、`map -> odom_combined` 都由 `origincar_bringup sim_navigation.launch.py` 提供。

检查基础话题：

```bash
ros2 topic list | grep -E "scan|odom|tf"
```

至少应看到：

```text
/scan
/odom
/tf
/tf_static
```

## 3. 单独启动导航

如果你已经单独启动了仿真和地图服务器，也可以只启动 Nav2 导航服务：

```bash
ros2 launch origincar_navigation navigation.launch.py
```

此 launch 不负责发布 `/map`，也不启动 AMCL。地图和定位由 `origincar_mapping` 包中的 `localization.launch.py` 发布：

```bash
ros2 launch origincar_mapping localization.launch.py
```

默认地图路径：

- 地图：`/home/xjl/digua/maps/sim/map.yaml`
- 路线：`/home/xjl/digua/maps/sim/global_routes.yaml`
- 参数：`origincar_navigation/config/nav2_params.yaml`

如果想临时指定其他地图，地图和路线文件需要一起覆盖：

```bash
ros2 launch origincar_bringup real_navigation.launch.py map:=/home/xjl/digua/maps/your_map.yaml routes_file:=/home/xjl/digua/maps/your_routes.yaml
```

如果是分模块调试，需要把同一个 `map:=...` 传给定位入口：

```bash
ros2 launch origincar_mapping localization.launch.py map:=/home/xjl/digua/maps/your_map.yaml
```

## 4. 检查 Nav2 是否启动成功

查看节点：

```bash
ros2 node list
```

常见 Nav2 节点包括：

```text
/amcl
/bt_navigator
/controller_server
/planner_server
/map_server
/behavior_server
```

查看 lifecycle 状态：

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
```

正常应为：

```text
active [3]
```

## 5. 在 RViz 中设置初始位姿

Nav2 使用 AMCL 定位，需要先告诉它小车大概在哪里。

操作步骤：

1. RViz 左上角确认 `Fixed Frame` 是 `map`。
2. 点击工具栏的 `2D Pose Estimate`。
3. 在地图上小车实际位置点击并拖动，箭头方向表示车头方向。
4. 观察小车模型是否和地图中的位置对齐。

设置后可以检查：

```bash
ros2 topic echo /amcl_pose --once
```

如果没有输出，说明 AMCL 还没有完成定位。

## 6. 在 RViz 中发送目标点

操作步骤：

1. 点击工具栏的 `Nav2 Goal` 或 `2D Goal Pose`。
2. 在地图中选择目标位置。
3. 拖动箭头指定目标朝向。
4. 观察小车是否开始移动。

导航过程中可检查：

```bash
ros2 topic echo /cmd_vel --once
ros2 topic echo /plan --once
```

如果 `/plan` 有路径，说明全局规划器工作正常。

如果 `/cmd_vel` 有输出，说明控制器正在尝试控制小车。

## 7. 使用命令行发送目标点

也可以不通过 RViz，直接发送导航目标：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {z: 0.0, w: 1.0}
    }
  }
}"
```

其中：

- `position.x`、`position.y` 是地图坐标。
- `orientation` 是目标朝向四元数。

## 8. 当前默认导航配置

当前配置文件：

```text
/home/xjl/digua/src/origincar_navigation/config/nav2_params.yaml
```

默认插件：

- 全局规划器：`nav2_navfn_planner/NavfnPlanner`
- 局部控制器：`nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`
- 路径平滑器：`nav2_smoother::SimpleSmoother`
- 定位：`AMCL`
- 控制输出：`/cmd_vel`
- 机器人基坐标：`base_footprint`

参数编辑要点：

- 地图路径由 launch 参数 `map:=...` 传入，不建议把换地图作为 `nav2_params.yaml` 的常规编辑项。
- `params_file:=...` 用于替换整套 AMCL/Nav2 参数。
- `nav2_params.yaml` 中的 `planner_server`、`controller_server`、`smoother_server`、`behavior_server`、`waypoint_follower` 和 `velocity_smoother` 分别对应 Nav2 官方导航 launch 启动的生命周期节点。

TF 链应为：

```text
map -> odom -> base_footprint -> base_link -> laser_link
```

## 9. 常见问题

### 启动时报 `nav2_bringup` 找不到

说明 Nav2 没安装：

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

### RViz 没有地图

检查地图文件是否存在：

```bash
ls -lh /home/xjl/digua/maps/sim_5m_map.*
```

检查 `/map`：

```bash
ros2 node list | grep map_server
ros2 lifecycle get /map_server
ros2 topic info /map --verbose
ros2 topic echo /map --once --qos-durability transient_local
```

正常情况：

```text
/map_server
active [3]
Publisher count: 1
```

如果 `Publisher count: 0`，说明你现在只是打开了 RViz，或者只启动了 `view_robot.launch.py`，还没有启动地图服务器。先运行：

```bash
ros2 launch origincar_mapping map_server.launch.py
```

如果这条命令提示 `Package 'origincar_mapping' not found`，说明当前终端没有加载工作区环境：

```bash
source /opt/ros/humble/setup.bash
source /home/xjl/digua/install/setup.bash
```

### 无法设置初始位姿

确认 RViz 的 `Fixed Frame` 是 `map`，并且 AMCL 已启动：

```bash
ros2 node list | grep amcl
```

### 小车不动

依次检查：

```bash
ros2 topic echo /cmd_vel --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo map base_footprint
```

如果 `/cmd_vel` 没有输出，说明 Nav2 控制器没有进入控制状态。

如果 `/cmd_vel` 有输出但小车不动，检查 Gazebo 仿真插件是否加载。

### 全局规划失败

可能原因：

- 起点或终点在障碍物内。
- 地图保存质量不好。
- costmap 膨胀半径过大。
- 初始位姿设置不准确。

建议先选择离小车较近、无遮挡的目标点测试。

### 小车转弯不自然

当前仿真小车通过阿克曼运动学插件接收 `/cmd_vel`。Nav2 默认控制器不是严格阿克曼控制器，但适合第一阶段跑通导航链路。后续需要实现自定义 `nav2_core::Controller`，限制曲率和最小转弯半径。

## 10. 推荐启动顺序

推荐使用 3 个终端：

终端 1，启动完整导航后台：

```bash
cd /home/xjl/digua
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch origincar_bringup real_navigation.launch.py
```

终端 2，打开 RViz：

```bash
cd /home/xjl/digua
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch origincar_sim view_robot.launch.py
```

终端 3，检查地图是否真的发布：

```bash
source /opt/ros/humble/setup.bash
source /home/xjl/digua/install/setup.bash
ros2 node list | grep map_server
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 topic info /map --verbose
ros2 topic info /robot_description --verbose
ros2 run tf2_ros tf2_echo map base_footprint
```

看到 `/map_server active [3]`、`/amcl active [3]`，并且 `tf2_echo map base_footprint` 持续输出后，说明 RViz 可以同时显示地图和小车。然后在 RViz 中发送目标点。

完整流程建议按下面顺序执行：

```bash
# 终端 1：启动仿真
ros2 launch origincar_bringup real_navigation.launch.py

# 终端 2：查看地图和小车
ros2 launch origincar_sim view_robot.launch.py

# RViz：设置初始位姿，再发送目标点
```

如果还没有地图，先按照 `doc/mapping_guide.md` 完成建图。
