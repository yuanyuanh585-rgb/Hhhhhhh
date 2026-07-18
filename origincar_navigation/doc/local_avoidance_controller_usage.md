# Origincar 阿克曼纯追踪与片段替换避障使用说明

本文档说明当前 `origincar_navigation/OrigincarAckermannPurePursuitController` 的实现状态、参数和调试方式。旧版滚动局部避障算法已经从源码中移除；当前只保留路径片段替换式局部避障。

## 1. 当前能力

- 纯追踪跟踪 Nav2 下发的全局路径。
- 保留阿克曼倒车逻辑，倒车时输出负线速度。
- 输出阿克曼兼容的 `/cmd_vel`：

```text
cmd_vel.angular.z = v * tan(delta) / wheelbase
```

- 维护一条当前跟踪路径 `active_route`，初始等于全局路径。
- 在 `local_costmap` 上扫描 `active_route` 前方障碍，只替换被挡住的局部片段。
- 受阻片段用前进-only Hybrid A* 搜索符合阿克曼运动学的绕障路径，再做轻量平滑后替换回 `active_route`。
- 全局规划器只在新目标到来时执行；局部障碍不会触发全局重规划。

## 2. 代码结构

主要文件：

```text
src/origincar_navigation/include/origincar_navigation/origincar_ackermann_pure_pursuit_controller.hpp
src/origincar_navigation/src/origincar_ackermann_pure_pursuit_controller.cpp
src/origincar_navigation/include/origincar_navigation/local_avoidance/segment_bypass_planner.hpp
src/origincar_navigation/src/segment_bypass_planner.cpp
src/origincar_navigation/config/nav2_params.yaml
```

控制器插件名称：

```text
origincar_navigation/OrigincarAckermannPurePursuitController
```

## 3. 控制流程

```text
Nav2 全局路径
  |
转换到 local_costmap world frame
  |
初始化或维护 active_route
  |
按 segment_replan_frequency 低频扫描前方障碍
  |
无障碍：继续跟踪 active_route
  |
有障碍：截取受阻片段并扩展前后过渡区
  |
前进-only Hybrid A* 搜索 bypass_path
  |
轻量平滑生成 smoothed_bypass
  |
替换 active_route 中对应片段
  |
纯追踪跟踪 active_route
  |
输出 cmd_vel
```

如果 Hybrid A* 或平滑失败，控制器不会破坏当前 `active_route`，会继续跟踪上一条可用路径，等待下一次片段重规划。

## 4. 全局重规划策略

自定义行为树：

```text
behavior_trees/navigate_to_pose_fixed_route_once.xml
```

当前策略：

- 新目标到来：调用一次 `ComputePathToPose`。
- 同一个目标执行中：不因局部障碍重新调用全局规划器。
- 固定路线行为树不调用 Nav2 `SmoothPath` 二次平滑全局路径。
- 局部障碍完全由控制器内部 `segment_bypass` 处理。

## 5. 参数说明

参数位于 `config/nav2_params.yaml` 的 `controller_server.ros__parameters.FollowPath` 下。

常用控制参数：

```yaml
wheelbase: 0.189
max_steering_angle: 0.65
min_lookahead_distance: 0.25
max_lookahead_distance: 0.50
lookahead_ratio: 0.35
max_linear_velocity: 0.70
max_reverse_velocity: 0.30
reverse_lookahead_ratio: 0.40
goal_tolerance: 0.08
yaw_goal_tolerance: 1.05
```

片段替换避障参数：

```yaml
avoidance_enabled: true
astar_allow_unknown: false
astar_lethal_cost: 253
astar_cost_weight: 0.02
astar_grid_downsample_factor: 1
hybrid_astar_yaw_bins: 72
hybrid_astar_steering_samples: 5
hybrid_astar_step_distance: 0.12
hybrid_astar_goal_tolerance: 0.12
hybrid_astar_yaw_tolerance: 0.70
hybrid_astar_max_iterations: 20000
hybrid_astar_steering_cost_weight: 0.05
hybrid_astar_goal_yaw_weight: 0.05
hybrid_astar_max_path_length_ratio: 3.0
hybrid_astar_search_lateral_margin: 0.80

segment_replan_frequency: 3.0
segment_scan_distance: 2.0
segment_pre_margin: 0.30
segment_post_margin: 0.50
segment_pre_point_margin: 3
segment_post_point_margin: 5
segment_min_blocked_length: 0.05
segment_min_replace_length: 0.40
segment_prune_search_distance: 1.0
segment_smoothing_enabled: true
segment_smoothing_collision_check: true
segment_sample_interval: 0.05
segment_route_sample_interval: 0.05
```

调参建议：

- 发现障碍太晚：增大 `segment_scan_distance` 或 `segment_replan_frequency`。
- 绕障过于贴近障碍：检查 local costmap 的 `robot_radius` 和 `inflation_radius`，必要时降低 `astar_lethal_cost`。
- 计算量过大：把 `astar_grid_downsample_factor` 调到 `2`、降低 `hybrid_astar_yaw_bins`，或降低 `segment_replan_frequency`。
- 绕障路径转弯仍然太急：减小 `hybrid_astar_step_distance`，或增大 `hybrid_astar_steering_cost_weight`。
- 绕障路径绕出大圆：减小 `hybrid_astar_max_path_length_ratio` 或 `hybrid_astar_search_lateral_margin`，也可以降低 `hybrid_astar_goal_yaw_weight`。
- U 形弯最近点跳到另一侧路径臂：减小 `segment_prune_search_distance`。
- 替换片段太短、连接处不顺：减小 `segment_route_sample_interval`，或增大 `segment_pre_point_margin` / `segment_post_point_margin`。
- 绕障段不够平滑：适当增大 `segment_pre_margin`、`segment_post_margin`，或增大 `segment_sample_interval`。

## 6. RViz 调试话题

建议在 RViz 中查看：

```text
/FollowPath/local_plan
/FollowPath/target_pose
/FollowPath/goal_reached
/FollowPath/astar_path
/FollowPath/optimized_local_path
```

含义：

- `local_plan`：当前纯追踪实际跟踪的 `active_route`。无障碍时应贴合全局路径；有障碍时只有受阻片段附近不同。
- `astar_path`：当前受阻片段的 Hybrid A* 绕障路径。话题名保持不变，便于沿用 RViz 配置。
- `optimized_local_path`：当前绕障片段平滑后的路径，不是整条全局路径的平滑结果。
- `target_pose`：当前纯追踪前视点。
- `goal_reached`：控制器内部目标到达状态。

## 7. 编译与运行

```bash
cd /home/xjl/digua
source /opt/ros/humble/setup.bash
colcon build --packages-select origincar_navigation
source install/setup.bash
ros2 launch origincar_navigation navigation.launch.py
```

## 8. 验证建议

无障碍验证：

- `local_plan` 应基本贴合全局路径。
- U 形弯等曲线区域不应整体向弯道内侧偏移。
- 小车应正常跟踪并到达目标。

静态障碍验证：

- 障碍出现在 `local_costmap` 后，`astar_path` 应以前进可行的阿克曼轨迹绕开障碍。
- `optimized_local_path` 应只显示平滑后的绕障片段。
- `local_plan` 应只在障碍附近被替换。
- `planner_server` 不应因为局部障碍重新输出新的全局路径。

倒车验证：

- 给一段目标点位于车体后方的路径。
- `cmd_vel.linear.x` 应为负。
- 车体应按倒车路径转向，而不是强行原地掉头。
