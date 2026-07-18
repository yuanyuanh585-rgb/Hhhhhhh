# 局部避障手动测试工具

这个包是临时测试工具包，不侵入 `origincar_navigation` 主控制器代码。测试完成后可以直接删除：

```bash
rm -rf src/origincar_navigation_test_tools
```

## 1. 功能

测试节点会：

- 订阅 Nav2 的 `/plan` 作为全局路径。
- 如果没有 `/plan`，可用默认起点/终点生成一条直线路径。
- 订阅 RViz `Publish Point` 发布的 `/clicked_point`，把每个点当作一个手动障碍物。
- 用手动障碍物生成一张临时 local costmap。
- 调用现有 `LocalAvoidancePlanner` 实时计算局部避障路径。
- 发布参考路径、A* 路径、B 样条优化路径和最终跟踪路径。

## 2. 编译

```bash
cd <workspace>
source /opt/ros/humble/setup.bash
colcon build --packages-select origincar_navigation origincar_navigation_test_tools
source install/setup.bash
```

## 3. 离线一键测试

推荐先用这个方式测试，不需要启动仿真、Nav2 或 TF：

```bash
ros2 launch origincar_navigation_test_tools offline_local_avoidance_test.launch.py
```

这个 launch 会自动：

- 固定起点：`(-2.0, -2.0, yaw=45deg)`。
- 固定终点：`(2.0, 2.0, yaw=45deg)`。
- 启动手动避障测试节点。
- 打开 RViz。
- 发布一张临时测试地图，不需要单独启动 map_server。
- 加载好路径、障碍物、costmap 栅格、起点终点显示。

如果要改起点终点：

```bash
ros2 launch origincar_navigation_test_tools offline_local_avoidance_test.launch.py \
  start_x:=-2.0 start_y:=-2.0 \
  goal_x:=2.0 goal_y:=2.0
```

RViz 打开后，直接点击工具栏的 `Publish Point`，在路径上点障碍物即可。每点一次都会实时重新计算避障路径。

如果 RViz 一开始看起来是深色背景，这是正常的离线视图；`TestMap` 显示的是测试节点发布的临时地图，白色/灰色区域为可通行区域，点击障碍物后会出现障碍栅格。

交互工具：

- `2D Pose Estimate`：重新指定离线测试起点。
- `2D Goal Pose`：重新指定局部规划目标点。
- `Publish Point`：手动放置障碍物。

播放和暂停：

```bash
ros2 topic pub --once /local_avoidance_manual_test/start std_msgs/msg/Empty "{}"
ros2 topic pub --once /local_avoidance_manual_test/pause std_msgs/msg/Empty "{}"
```

重置到默认起点：

```bash
ros2 topic pub --once /local_avoidance_manual_test/reset_start std_msgs/msg/Empty "{}"
```

## 4. 手动节点启动

```bash
ros2 launch origincar_navigation_test_tools local_avoidance_manual_test.launch.py
```

如果你已经启动了仿真和 Nav2，节点会优先使用：

- TF 中的 `map -> base_footprint` 作为车辆当前位置；
- Nav2 发布的 `/plan` 作为全局路径。

如果你只想离线测试，不启动 Nav2，也可以关闭 TF 位姿：

```bash
ros2 launch origincar_navigation_test_tools local_avoidance_manual_test.launch.py use_tf_pose:=false
```

默认离线起点和终点可通过参数改：

```bash
ros2 launch origincar_navigation_test_tools local_avoidance_manual_test.launch.py \
  use_tf_pose:=false \
  default_start_x:=-2.0 default_start_y:=-2.0 \
  default_goal_x:=2.0 default_goal_y:=2.0
```

## 5. RViz 操作

在 RViz 中：

1. Fixed Frame 设为 `map`。
2. 添加 Path 显示：
   - `/local_avoidance_manual_test/reference_path`
   - `/local_avoidance_manual_test/astar_path`
   - `/local_avoidance_manual_test/optimized_path`
   - `/local_avoidance_manual_test/tracking_path`
3. 添加 MarkerArray：
   - `/local_avoidance_manual_test/manual_obstacle_markers`
4. 添加 Marker：
   - `/local_avoidance_manual_test/test_costmap_obstacles`
5. 使用工具栏 `Publish Point`，在路径上点击放置障碍物。

每点击一次，节点会立即重新计算路径。

使用 `offline_local_avoidance_test.launch.py` 时，上面这些显示项已经自动配置好。

## 6. 话题说明

```text
/local_avoidance_manual_test/reference_path
```

输入参考路径。启动 Nav2 时通常来自 `/plan`；离线测试时是默认起点到终点的直线路径。

```text
/local_avoidance_manual_test/astar_path
```

局部规划器输出的 A* 路径。无障碍时通常显示贴合全局路径的局部参考段；有障碍时显示绕障路径。

```text
/local_avoidance_manual_test/optimized_path
```

B 样条/NLopt 优化后的候选路径。它表示“平滑器认为可以用的路径”。

```text
/local_avoidance_manual_test/tracking_path
```

最终应该交给纯追踪跟踪的路径。优化成功时，它通常等于 `optimized_path`；如果优化失败、路径太短，它会回退为 A* 路径或贴合全局路径的参考段。

为了离线测试观察方便，如果 `optimized_path` 原本为空，测试节点会把 `tracking_path` 也发布到 `optimized_path` 话题。真实控制器内部不会把这两个概念混在一起。

```text
/clicked_point
```

RViz `Publish Point` 的默认输出。测试节点把这个点当作手动障碍物中心。

```text
/local_avoidance_manual_test/clear_obstacles
```

清空手动障碍物：

```bash
ros2 topic pub --once /local_avoidance_manual_test/clear_obstacles std_msgs/msg/Empty "{}"
```

```text
/initialpose
```

RViz `2D Pose Estimate` 输出。离线测试节点用它更新起点。

```text
/goal_pose
```

RViz `2D Goal Pose` 输出。离线测试节点用它更新局部规划目标点。

## 7. 常用参数

```text
map_width / map_height
```

临时 costmap 尺寸，默认 `6.0m x 6.0m`。

```text
map_resolution
```

临时 costmap 分辨率，默认 `0.05m`。

```text
obstacle_radius
```

手动障碍物实体半径，默认 `0.12m`。

```text
obstacle_inflation_radius
```

手动障碍物膨胀半径，默认 `0.18m`。

```text
astar_global_path_bias
```

A* 贴近参考路径的权重，默认测试节点设为 `0.12`。如果 A* 绕障后偏离全局路径太远，可以增大。

```text
astar_previous_path_bias
```

A* 贴近上一条可通行局部路径的权重，默认测试节点设为 `0.25`。它不会阻止 A* 每次向前搜索，只会在左右绕障代价接近时优先保持上一轮绕障侧；如果仍然左右横跳，可以逐步增大到 `0.4` 或 `0.6` 测试。

```text
stable_path_reuse_enabled
stable_path_max_robot_distance
stable_path_min_remaining_length
```

稳定路径复用参数。默认开启复用：每次更新仍会从当前起点向前重新规划，并实时更新 A* 与优化路径；上一条成功路径只会被裁剪保存为备用，当本次规划失败且旧路径仍可通行时才沿用。

## 8. 删除方式

测试完成后删除整个包：

```bash
rm -rf src/origincar_navigation_test_tools
```

然后重新编译工作区即可。
