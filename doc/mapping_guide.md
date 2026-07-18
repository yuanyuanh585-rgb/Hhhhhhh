# Origincar 仿真建图操作指南

本文档说明如何在 Gazebo 仿真环境中使用 `SLAM Toolbox` 建图，并保存为 Nav2 可加载的 `yaml + pgm` 地图。

## 1. 安装依赖

如果还没有安装建图和导航依赖，先执行：

```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-map-server ros-humble-nav2-amcl
```

新开终端后，工作区环境会通过 `~/.bashrc` 自动加载。若当前终端还没加载，执行：

```bash
source ~/.bashrc
```

## 2. 启动仿真

终端 1：

```bash
ros2 launch origincar_bringup mapping.launch.py
```

这个统一入口会启动 Gazebo、机器人和 `slam_toolbox`。确认 Gazebo 中出现 5m x 5m 场地和小车。

## 3. 检查基础话题

终端 2：

```bash
ros2 topic list
```

建图前至少需要看到：

```text
/scan
/odom
/tf
/tf_static
```

检查 TF 是否连通：

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint laser_link
```

如果 TF 没有输出，先不要建图，优先检查仿真是否正常启动。

## 4. 单独启动 SLAM

如果你没有使用上面的统一建图入口，而是先单独启动了仿真，也可以另开终端启动 SLAM：

```bash
ros2 launch origincar_mapping online_slam.launch.py
```

启动后检查 `/map`：

```bash
ros2 topic echo /map --once
```

如果能输出 `nav_msgs/msg/OccupancyGrid`，说明建图节点已经开始工作。

## 5. 打开 RViz 观察地图

可以使用仿真自带 RViz，或者直接启动 RViz 后手动添加：

```bash
rviz2
```

建议显示项：

- `Map`：topic 选择 `/map`
- `LaserScan`：topic 选择 `/scan`
- `TF`
- `RobotModel`

Fixed Frame 设置为 `map`。

## 6. 键盘遥控覆盖场地

终端 4：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

常用按键：

```text
i 前进
, 后退
j 左转
l 右转
k 停止
q/z 增减最大速度
w/x 增减线速度
e/c 增减角速度
```

建图建议：

- 慢速行驶，避免高速急转。
- 沿围挡绕一圈，再覆盖中间区域。
- 雷达看到墙面后，地图边界会逐渐变清晰。
- 如果地图明显扭曲，降低速度并重新建图。

## 7. 保存地图

确认 RViz 中地图完整后，终端 5 执行：

```bash
ros2 run nav2_map_server map_saver_cli -f /home/xjl/digua/maps/sim_5m_map
```

也可以使用项目 launch：

```bash
ros2 launch origincar_mapping save_map.launch.py
```

保存成功后应生成：

```text
/home/xjl/digua/maps/sim_5m_map.yaml
/home/xjl/digua/maps/sim_5m_map.pgm
```

当前仿真导航默认读取：

```text
/home/xjl/digua/maps/sim/map.yaml
/home/xjl/digua/maps/sim/map.pgm
/home/xjl/digua/maps/sim/global_routes.yaml
```

检查：

```bash
ls -lh /home/xjl/digua/maps
cat /home/xjl/digua/maps/sim/map.yaml
```

## 8. 常见问题

### 没有 `/scan`

说明小车没有成功 spawn，或者 Gazebo 雷达插件没有加载。检查 Gazebo 启动日志和：

```bash
ros2 topic list | grep scan
```

### 没有 `/map`

说明 `slam_toolbox` 没有正常启动，或没有收到 `/scan`/TF。检查：

```bash
ros2 node list
ros2 topic echo /scan --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

### 地图漂移或拉扯

通常是速度过快、急转太多、TF 中断或里程计不连续。建议慢速重建，先沿墙走，再补中间区域。

### 保存失败

确认已安装 `nav2_map_server`：

```bash
ros2 pkg prefix nav2_map_server
```

确认目录存在：

```bash
mkdir -p /home/xjl/digua/maps
```

### 地图方向和 Gazebo 视觉方向不一致

Gazebo 地面纹理的视觉方向可能和 ROS map 坐标显示方向不同。导航使用的是 SLAM 生成的 occupancy map，不直接依赖纹理方向。
