# Origincar 仿真、建图与导航使用说明

本文档是当前项目的统一启动说明。推荐优先使用 `origincar_bringup` 里的统一入口；只有调试时才单独启动各模块。

## 1. 环境准备

项目根目录：

```bash
cd /home/xjl/digua
```

你的 `~/.bashrc` 已配置自动加载：

```bash
source /opt/ros/humble/setup.bash
source /home/xjl/digua/install/setup.bash
```

如果是已经打开的旧终端，执行一次：

```bash
source ~/.bashrc
```

检查包是否能找到：

```bash
ros2 pkg list | grep origincar
```

应能看到：

```text
origincar_sim
origincar_mapping
origincar_navigation
origincar_bringup
```

## 2. 依赖安装

仿真依赖：

```bash
sudo apt update
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ros2launch ros-humble-ackermann-msgs ros-humble-xacro ros-humble-joint-state-publisher ros-humble-rviz2
```

建图和导航依赖：

```bash
sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-map-server ros-humble-nav2-amcl
```

键盘控制依赖：

```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

## 3. 构建项目

修改代码或 launch 后执行：

```bash
cd /home/xjl/digua
colcon build
source install/setup.bash
```

只构建当前几个包：

```bash
colcon build --packages-select origincar_sim origincar_mapping origincar_navigation origincar_bringup
source install/setup.bash
```

## 4. 推荐启动入口

### 4.1 只启动仿真

```bash
ros2 launch origincar_bringup sim.launch.py
```

启动内容：

- Gazebo 世界
- 5m x 5m 地图场地
- 阿克曼小车
- `/scan`
- `/imu/data`
- `/odom`
- `/tf`

### 4.2 启动仿真 + SLAM 建图

```bash
ros2 launch origincar_bringup mapping.launch.py
```

启动内容：

- Gazebo 仿真
- 小车传感器
- `slam_toolbox`
- `/map`
- `map -> odom`

另开终端键盘控制小车：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

建图完成后保存：

```bash
ros2 run nav2_map_server map_saver_cli -f /home/xjl/digua/maps/sim_5m_map
```

保存后应生成：

```text
/home/xjl/digua/maps/sim_5m_map.yaml
/home/xjl/digua/maps/sim_5m_map.pgm
```

当前仿真导航默认读取的场景数据为：

```text
/home/xjl/digua/maps/sim/map.yaml
/home/xjl/digua/maps/sim/map.pgm
/home/xjl/digua/maps/sim/global_routes.yaml
```

### 4.3 启动仿真 + 定位 + 导航

已有地图后启动：

```bash
ros2 launch origincar_bringup sim_navigation.launch.py
```

启动内容：

- Gazebo 仿真
- `map_server` 发布 `/map`
- 激光定位发布 `map -> odom_combined`
- Nav2 planner/controller/bt navigator
- `origincar_strategy` 路线策略控制器
- 不启动 RViz

注意：这个入口不启动 RViz，并且会自动使用 `/home/xjl/digua/src/origincar_navigation/config/nav2_params.yaml`。

更换地图时不要直接改 `nav2_params.yaml` 里的 `map_server.yaml_filename`。统一导航入口会把 `map:=...` 传给定位 launch，并覆盖地图服务器参数：

```bash
ros2 launch origincar_bringup sim_navigation.launch.py map:=/home/xjl/digua/maps/your_map.yaml routes_file:=/home/xjl/digua/maps/your_routes.yaml
```

更换整套 Nav2/AMCL 参数文件时再使用：

```bash
ros2 launch origincar_bringup navigation.launch.py params_file:=/path/to/nav2_params.yaml
```

## 5. RViz 查看

要在 RViz 中同时看到地图和小车，先启动完整导航后台：

```bash
ros2 launch origincar_bringup navigation.launch.py
```

然后另开终端启动 RViz：

```bash
ros2 launch origincar_sim view_robot.launch.py
```

`view_robot.launch.py` 现在只打开 RViz，不发布 `/map`、`/robot_description`、`/joint_states`、TF 或纹理 Marker。地图和小车外观都来自 `navigation.launch.py` 启动的后台节点。RViz 配置预设为：

- Fixed Frame 设置为 `map`
- 添加 `Map`，topic 为 `/map`，默认打开，用于显示 Nav2/AMCL 使用的占据栅格地图
- 保留 `SmartCarMapTexture`，topic 为 `/smart_car_map_texture`，默认关闭；需要查看 Gazebo 原始彩色场地图时再手动打开
- 添加 `RobotModel`
- 添加 `TF`
- 使用 `2D Pose Estimate` 设置初始位姿
- 使用 `Nav2 Goal` 或 `2D Goal Pose` 发送目标点

同时显示地图和小车必须满足：

```text
/map 有发布者
/robot_description 有发布者
TF 存在 map -> odom -> base_footprint
```

检查命令：

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 topic info /map --verbose
ros2 topic info /robot_description --verbose
ros2 run tf2_ros tf2_echo map base_footprint
```

如果只是想单独检查 URDF 模型，不依赖 Gazebo 和地图，分两个入口启动：

```bash
# 终端 1：发布 robot_description、joint_states 和模型 TF
ros2 launch origincar_sim robot_state.launch.py

# 终端 2：只打开模型 RViz
ros2 launch origincar_sim view_model.launch.py
```

如果确实需要在 RViz 中叠加 Gazebo 使用的原始彩色场地图，单独启动纹理 Marker 发布器。发布器会把图片顺时针旋转 90 度，并转换成彩色栅格 Marker，避免 RViz 的内嵌纹理渲染崩溃：

```bash
ros2 launch origincar_sim texture_marker.launch.py
```

然后在 RViz 左侧手动打开 `SmartCarMapTexture`。正常导航调试时建议只看默认 `/map`，避免彩色纹理和占据栅格互相遮挡。

## 6. 单独调试入口

### 6.1 单独启动仿真

```bash
ros2 launch origincar_sim sim_world.launch.py
```

### 6.2 单独启动 SLAM

```bash
ros2 launch origincar_mapping online_slam.launch.py
```

### 6.3 单独发布静态地图

```bash
ros2 launch origincar_mapping map_server.launch.py
```

指定其他地图：

```bash
ros2 launch origincar_mapping map_server.launch.py map:=/home/xjl/digua/maps/your_map.yaml
```

### 6.4 单独启动地图 + AMCL

```bash
ros2 launch origincar_mapping localization.launch.py
```

指定其他地图：

```bash
ros2 launch origincar_mapping localization.launch.py map:=/home/xjl/digua/maps/your_map.yaml
```

### 6.5 单独启动 Nav2 导航服务

```bash
ros2 launch origincar_navigation navigation.launch.py
```

注意：这个命令不发布 `/map`，也不启动 AMCL。通常需要先运行：

```bash
ros2 launch origincar_mapping localization.launch.py
```

## 7. 控制小车

键盘控制：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

直接发布 `/cmd_vel`：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.4}}" -r 10
```

直接发布阿克曼指令：

```bash
ros2 topic pub /ackermann_cmd ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 0.2, steering_angle: 0.3}}" -r 10
```

建议同一时间只使用一种控制输入。

## 8. 常用检查命令

检查话题：

```bash
ros2 topic list
```

仿真基础话题：

```bash
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic echo /imu/data --once
```

地图：

```bash
ros2 node list | grep map_server
ros2 lifecycle get /map_server
ros2 topic info /map --verbose
ros2 topic echo /map --once --qos-durability transient_local
```

如果 `ros2 topic info /map --verbose` 显示 `Publisher count: 0`，说明当前没有任何节点在发布地图。此时需要启动：

```bash
ros2 launch origincar_mapping map_server.launch.py
```

或者启动包含地图服务器的统一导航入口：

```bash
ros2 launch origincar_bringup navigation.launch.py
```

TF：

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_ros tf2_echo base_footprint laser_link
```

Nav2 lifecycle：

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
```

正常应为：

```text
active [3]
```

## 9. 常见问题

### 没有 `/map`

如果是导航模式，确认启动的是：

```bash
ros2 launch origincar_bringup navigation.launch.py
```

或者单独启动：

```bash
ros2 launch origincar_mapping map_server.launch.py
```

### 没有 `map -> odom`

说明 AMCL 没启动或没有初始位姿。确认：

```bash
ros2 node list | grep amcl
```

然后在 RViz 中使用 `2D Pose Estimate` 设置初始位姿。

### Nav2 启动但小车不动

检查是否有速度输出：

```bash
ros2 topic echo /cmd_vel --once
```

如果没有 `/cmd_vel`，说明导航没有进入控制状态。

如果有 `/cmd_vel` 但车不动，检查 Gazebo 仿真和阿克曼插件：

```bash
ros2 topic echo /odom --once
```

### 地图文件不存在

先完成建图并保存：

```bash
ros2 launch origincar_bringup mapping.launch.py
ros2 run nav2_map_server map_saver_cli -f /home/xjl/digua/maps/sim_5m_map
```

### 想看更详细文档

建图步骤：

```text
/home/xjl/digua/doc/mapping_guide.md
```

导航步骤：

```text
/home/xjl/digua/doc/navigation_usage.md
```

项目规划：

```text
/home/xjl/digua/doc/navigation_plan.md
```
