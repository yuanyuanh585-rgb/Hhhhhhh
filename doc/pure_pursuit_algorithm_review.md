# 纯追踪算法流程审查与对比

本文审查当前 `digua` 系统中实际启用的纯追踪控制器，并与 `/home/xjl/du-zhong/src/pure_pursuit_planner` 以及 Nav2 Humble 自带的 `nav2_regulated_pure_pursuit_controller` 对比。

## 1. 审查范围与源码依据

当前系统源码：

- `src/origincar_navigation/src/origincar_ackermann_pure_pursuit_controller.cpp`
- `src/origincar_navigation/include/origincar_navigation/origincar_ackermann_pure_pursuit_controller.hpp`
- `src/origincar_navigation/src/segment_bypass_planner.cpp`
- `src/origincar_navigation/include/origincar_navigation/local_avoidance/segment_bypass_planner.hpp`
- `src/origincar_navigation/config/nav2_params.yaml`
- `src/origincar_navigation/behavior_trees/navigate_to_pose_fixed_route_once.xml`

`du-zhong` 参考实现：

- `/home/xjl/du-zhong/src/pure_pursuit_planner/src/pure_pursuit_planner.cpp`
- `/home/xjl/du-zhong/src/pure_pursuit_planner/src/pure_pursuit_planner_back.cpp`
- `/home/xjl/du-zhong/src/pure_pursuit_planner/include/pure_pursuit_planner/*.h`
- `/home/xjl/du-zhong/src/pure_pursuit_planner/cfg/PurePursuit.cfg`

Nav2 参考：

- 本机安装包：`ros-humble-nav2-regulated-pure-pursuit-controller`，版本 `1.1.20-1jammy.20260425.081712`
- 本机头文件：`/opt/ros/humble/include/nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp`
- 上游 Humble 源码：`https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp`
- 上游 README：`https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_regulated_pure_pursuit_controller/README.md`

## 2. 当前系统总体结论

当前系统启用的是自定义 Nav2 控制器插件：

```yaml
controller_plugins: ["FollowPath"]
FollowPath:
  plugin: origincar_navigation/OrigincarAckermannPurePursuitController
```

它不是 Nav2 原生 `RegulatedPurePursuitController` 的简单参数化使用，而是从 ROS1 `du-zhong` 纯追踪迁移并重写成 Nav2 `nav2_core::Controller` 插件，额外加入：

- 阿克曼前轮转角模型：内部先算前轮等效转角 `delta`，再转换为 `/cmd_vel.angular.z`。
- 前进/倒车跟踪：用短前视点是否位于车体后方判断倒车。
- 终点姿态校正：接近终点时把目标 yaw 误差融合进转向角，并降低速度。
- 当前跟踪路径 `active_route_`：在全局路径外维护一条可被局部片段替换的实际跟踪路径。
- 片段替换式局部避障：只替换前方受阻路径片段，不触发全局重规划。
- 局部绕障失败保护：失败时不破坏旧 `active_route_`，等待下一次片段重规划。

所以当前系统的“纯追踪算法流程”应理解为：

```text
Nav2 FollowPath action
  -> OrigincarAckermannPurePursuitController::setPlan(global_path)
  -> 20 Hz computeVelocityCommands()
      -> 坐标变换与到达判断
      -> 全局路径转换到 local costmap frame
      -> SegmentBypassPlanner 低频维护 active_route_
      -> 在 tracking_plan 上做纯追踪
      -> 倒车判断
      -> 自适应前视距离
      -> 前视点选择或终点虚拟延长
      -> 阿克曼转向角 delta
      -> 转弯/终点限速
      -> 输出 /cmd_vel
```

## 3. 当前系统控制流程详解

### 3.1 生命周期与插件接入

`OrigincarAckermannPurePursuitController` 实现 `nav2_core::Controller`，由 Nav2 `controller_server` 生命周期节点加载。

`configure()` 做以下事情：

1. 保存生命周期节点、TF buffer、costmap ROS 对象。
2. 从 `costmap_ros_` 获取 `robot_base_frame_` 和 `global_frame_`。
3. 调用 `loadParameters()` 读取 `FollowPath.*` 参数。
4. 创建调试话题：
   - `FollowPath/local_plan`
   - `FollowPath/astar_path`
   - `FollowPath/optimized_local_path`
   - `FollowPath/target_pose`
   - `FollowPath/goal_reached`
5. 创建 `SegmentBypassPlanner`。

`activate()` / `deactivate()` 只切换 lifecycle publisher 状态。

`cleanup()` 会清空所有路径缓存和局部避障状态，包括 `global_plan_`、`active_tracking_path_`、`last_astar_path_`、`last_optimized_path_`、`active_route_`、`active_route_valid_`。

### 3.2 setPlan 行为

Nav2 每次给控制器新路径时调用：

```cpp
void setPlan(const nav_msgs::msg::Path & path)
```

当前实现会：

- 保存 `global_plan_ = path`
- 清空 `active_tracking_path_`
- 清空上一条 A* 和优化路径
- 清空 `active_route_`
- 重置局部重规划节流时间
- 关闭 `local_avoidance_active_`
- 重置 `last_goal_reached_`

重点：当前控制器不在 `setPlan()` 里裁剪全局路径。全局路径仍作为原始参考，实际跟踪路径在控制循环中由 `active_route_` 维护。

### 3.3 computeVelocityCommands 主流程

`computeVelocityCommands()` 输入是 Nav2 提供的：

- 当前机器人位姿 `pose`
- 当前速度 `velocity`
- goal checker 指针，但当前实现没有使用该指针

主流程如下：

1. 初始化返回消息：

   ```cpp
   cmd_vel.header.stamp = node_->now();
   cmd_vel.header.frame_id = robot_base_frame_;
   ```

2. 空路径保护：

   如果 `global_plan_.poses.empty()`，抛出 `nav2_core::PlannerException`。

3. 机器人位姿 frame 处理：

   - 如果 `pose.header.frame_id` 为空，补成 `global_plan_.header.frame_id`。
   - 如果机器人位姿不在 `global_frame_`，调用 `transformPose()` 转到 `global_frame_`。

4. 内部到达判断：

   `isGoalReached(robot_pose)` 成功时：

   - `current_velocity_ = 0`
   - `reverse_mode_ = false`
   - 发布 `goal_reached`
   - 返回零速度

5. 路径坐标转换：

   调用 `transformedPlan(global_frame_)`，把全局路径每个点转换到 local costmap 的 world frame。这里不是转换到 base frame，因为后续 A* 的 `worldToMap()` 必须用 costmap frame 下的 world 坐标。

6. 更新当前速度：

   ```cpp
   current_velocity_ = velocity.linear.x;
   ```

   注意当前系统用 Nav2 传入的速度，不像 `du-zhong` 的 `pure_pursuit_planner_back.cpp` 那样订阅 `/odom` 后自己保存 `current_velocity_`。

7. 更新局部片段绕障路径：

   ```cpp
   updateSegmentBypassPath(robot_pose, plan);
   ```

8. 选择实际跟踪路径：

   默认 `tracking_plan = plan`。如果局部绕障维护出了 `active_tracking_path_`，则改用 `active_tracking_path_.poses`。

9. 倒车判断：

   ```cpp
   reverse_mode_ = shouldReverse(tracking_plan, robot_pose);
   ```

10. 前视距离：

   ```cpp
   lookahead_distance = adaptiveLookaheadDistance(current_velocity_);
   ```

11. 前视点：

   ```cpp
   target_pose = getLookaheadPoint(tracking_plan, robot_pose, lookahead_distance, true);
   ```

12. 阿克曼转角：

   ```cpp
   delta = steeringAngle(target_pose, robot_base_frame_);
   ```

13. 终点姿态融合：

   如果非倒车，且距 `tracking_plan.back()` 小于 `pose_adjust_distance_`，把终点 yaw 误差按权重融合到 `delta`：

   ```text
   angle_error = shortest_angular_distance(robot_yaw, goal_yaw)
   orientation_delta = orientation_kp * angle_error
   correction_weight = clamp(1 - dist_to_goal / pose_adjust_distance, 0.30, 1.0)
   delta = delta * (1 - correction_weight) + orientation_delta * correction_weight
   ```

14. 转角限幅：

   ```cpp
   delta = clamp(delta, -max_steering_angle_, max_steering_angle_);
   ```

15. 速度计算：

   ```cpp
   commanded_linear_speed = targetLinearVelocity(robot_pose, delta);
   ```

16. 输出 `/cmd_vel`：

   前进：

   ```text
   linear.x = commanded_linear_speed
   angular.z = commanded_linear_speed * tan(delta) / wheelbase
   ```

   倒车：

   ```text
   linear.x = -commanded_linear_speed
   angular.z = -commanded_linear_speed * tan(delta) / wheelbase
   ```

   这里的 `angular.z` 不是差速车意义的“直接 yaw rate”，而是给当前仿真/底盘阿克曼插件反解前轮转角的中间量。代码注释说明底盘插件使用：

   ```text
   steering = atan(wheelbase * angular.z / linear.x)
   ```

18. 发布调试话题。

### 3.4 坐标变换策略

当前 `transformPose()` 有一个重要处理：变换前把输入 pose 的 stamp 置 0。

```cpp
pose.header.stamp.sec = 0;
pose.header.stamp.nanosec = 0;
```

含义：使用最新可用 TF。这样更适合静态全局路径长期跟踪，避免路径点带旧时间戳导致 `extrapolation into the past`。

这与 Nav2 原生 RPP 不同。Nav2 RPP 通常使用机器人当前时间和局部 costmap/base frame 变换近端路径。

### 3.5 前视点选择

当前实现：

1. `closestPoseIndex()` 在 `tracking_plan` 上线性搜索离机器人最近的路径点。
2. 从最近点向后找第一个距离机器人大于等于 `lookahead_distance` 的路径点。
3. 如果找不到：
   - 默认取路径末端。
   - 若 `allow_goal_extension=true`，沿路径末端 yaw 创建虚拟延长点。
   - 倒车时 yaw 加 `pi`，再归一化。
   - 延长距离为：

     ```text
     extend_distance = lookahead_distance - dist_to_goal + 0.10
     ```

这个逻辑与 `du-zhong/src/pure_pursuit_planner.cpp` 的普通版非常接近；当前系统把单位从“度制转角”改成了“弧度制转角”，并整理到 Nav2 插件接口中。

### 3.6 自适应前视距离

当前公式：

```text
Ld = min_lookahead_distance + lookahead_ratio * abs(current_speed)
if reverse_mode:
  Ld *= reverse_lookahead_ratio
Ld = clamp(Ld, min_lookahead_distance * 0.5, max_lookahead_distance)
```

配置值：

```yaml
min_lookahead_distance: 0.25
max_lookahead_distance: 0.50
lookahead_ratio: 0.35
reverse_lookahead_ratio: 0.40
```

因此：

- 静止前进时 `Ld = 0.30m`
- `0.70m/s` 前进时理论 `0.30 + 0.50 * 0.70 = 0.65m`，限幅到 `0.60m`
- 倒车时先乘 `0.40`，但再被下限 `min_lookahead_distance * 0.5 = 0.15m` 保护

### 3.7 倒车判断

当前倒车判断：

```text
check_pose = getLookaheadPoint(plan, robot_pose, reverse_check_distance, false)
check_pose -> base frame
if check_pose.x < 0:
  reverse_mode = true
else:
  reverse_mode = false
```

配置：

```yaml
reverse_check_distance: 0.20
max_reverse_velocity: 0.30
```

特点：

- 每个控制周期重新判断。
- 没有状态保持或 hysteresis。
- 判断依据是实际跟踪路径 `tracking_plan`，所以局部绕障替换后的路径也会影响倒车判定。
- TF 失败时返回 `false`，即默认前进。

### 3.8 转向角公式

前视点先变换到 `robot_base_frame_`。

```text
x = target_in_robot.x
y = target_in_robot.y
distance_sq = x^2 + y^2
alpha = atan2(y, x)
delta = atan(2 * wheelbase * sin(alpha) / sqrt(distance_sq))
```

其中：

- `delta` 是前轮等效转向角，单位弧度。
- `wheelbase = 0.189m`
- `max_steering_angle = 0.65rad`

由于 `sqrt(distance_sq)` 就是前视点距离，该公式等价于常见的阿克曼纯追踪：

```text
delta = atan(2L sin(alpha) / Ld)
```

### 3.9 速度策略

`targetLinearVelocity()` 先按模式选择速度上限：

```text
max_velocity = reverse_mode ? max_reverse_velocity : max_linear_velocity
target_velocity = max_velocity
```

#### 3.9.1 转弯减速

如果 `abs(delta) > decel_angle_threshold`：

```text
angle_ratio = clamp(abs(delta) / max_steering_angle, 0, 1)
turn_factor = max(1 - angle_ratio^2 * decel_ratio, min_turn_speed_factor)
target_velocity = min(max_velocity * turn_factor, max_turn_linear_velocity)
```

配置：

```yaml
decel_angle_threshold: 0.35
decel_ratio: 0.80
min_turn_speed_factor: 0.20
max_turn_linear_velocity: 0.30
```

#### 3.9.2 接近终点减速

只在非倒车模式启用。以 `global_plan_.poses.back()` 为目标点：

```text
if dist_to_goal < goal_decel_distance:
  ratio = clamp(dist_to_goal / goal_decel_distance, 0, 1)
  target_velocity = min_approach_linear_velocity
                  + (max_velocity - min_approach_linear_velocity) * ratio
```

如果终点 yaw 误差大于 `yaw_goal_tolerance`，继续乘一个角度因子：

```text
angle_factor = max(0.3, 1 - (angle_error - yaw_goal_tolerance) / (pi - yaw_goal_tolerance))
target_velocity *= angle_factor
```

配置：

```yaml
goal_decel_distance: 0.30
min_approach_linear_velocity: 0.05
yaw_goal_tolerance: 1.05
```

注意：终点姿态融合用的是 `tracking_plan.back()`，终点减速用的是 `global_plan_.back()`。无局部绕障时两者一致；有局部绕障且 `active_route_` 已被裁剪/替换时，二者理论上仍应终点一致，但审查时需要注意这种双来源。

### 3.10 到达判断

当前内部到达判断：

```text
distance(robot, global_plan.back) <= goal_tolerance
and
abs(shortest_angular_distance(robot_yaw, goal_yaw)) <= yaw_goal_tolerance
```

配置与 Nav2 `SimpleGoalChecker` 对齐：

```yaml
xy_goal_tolerance: 0.08
yaw_goal_tolerance: 1.05
goal_tolerance: 0.08
```

虽然 Nav2 已经有 `general_goal_checker`，当前控制器仍额外做内部判断，目的是接近目标时立即输出零速度。

## 4. 当前系统片段替换式局部避障

该部分不是传统纯追踪算法的一部分，但在当前系统中决定了纯追踪实际跟踪的 `tracking_plan`，因此必须纳入审查。

### 4.1 行为树层面的设计

`navigate_to_pose_fixed_route_once.xml` 明确禁止因为局部障碍重新全局规划：

- 新 goal 或 goal 更新时才计算全局路径。
- `FollowPath` 直接接收固定全局路径。
- 不运行 Nav2 `SmoothPath`，避免 U 型路线被整体拉偏。
- 局部障碍由控制器内部处理。

这意味着当前系统的障碍响应主要靠 `SegmentBypassPlanner`，不是靠 Nav2 原生控制器碰撞预测，也不是靠频繁 `ComputePathToPose`。

### 4.2 active_route_ 的生命周期

在控制器中：

- `global_plan_`：Nav2 下发的原始全局路径。
- `active_route_`：控制器持续维护的当前跟踪路径。
- `active_tracking_path_`：本周期交给纯追踪的路径；通常等于 `active_route_`。

初始化：

```text
if active_route_ invalid:
  active_route_ = posesToPath(transformed_global_plan, global_frame)
```

更新频率：

```yaml
segment_replan_frequency: 3.0
```

控制频率是 `20Hz`，但片段绕障只按 `3Hz` 执行。非重规划周期直接复用 `active_route_`。

### 4.3 updateRoute 流程

`SegmentBypassPlanner::updateRoute()`：

1. 检查 `enabled` 和路径长度。
2. `pruneRouteFromRobot()`：
   - 找 `active_route` 中离机器人最近点。
   - 新路径头部插入当前机器人位姿作为锚点。
   - 从最近点往后复制路径点。
   - 跳过与上一个点距离小于 `0.02m` 的点。
   - 重新计算路径 yaw。
3. `findFirstBlockedSegment()`：
   - 只扫描前方 `scan_distance`。
   - 逐段按 `max(costmap_resolution * 0.5, 0.01)` 插值采样。
   - `cost != NO_INFORMATION && cost >= astar_lethal_cost` 认为 blocked。
   - 连续 blocked 长度大于 `min_blocked_length` 才成立。
4. 没有 blocked segment：
   - 返回 `success=true`
   - `active_route = pruned_route`
5. 有 blocked segment：
   - 用 `segment_pre_margin` 往前扩展替换起点。
   - 用 `segment_post_margin` 往后扩展替换终点。
   - 替换段长度必须大于 `segment_min_replace_length`。
6. 起终点映射到 costmap cell，并找附近 free cell。
7. 在 local costmap 上对该局部片段跑 A*。
8. 如果启用 smoothing 且 A* 路径点数足够：
   - 做轻量三点平滑，固定首尾连接锚点。
   - 可选碰撞检查。
9. `replaceRouteSegment()` 把替换段拼回 `active_route`。
10. 重算所有路径点 yaw。

配置：

```yaml
avoidance_enabled: true
astar_allow_unknown: false
astar_lethal_cost: 253
astar_cost_weight: 0.02
astar_grid_downsample_factor: 1
segment_scan_distance: 2.0
segment_pre_margin: 0.30
segment_post_margin: 0.50
segment_min_blocked_length: 0.05
segment_min_replace_length: 0.40
segment_smoothing_enabled: true
segment_smoothing_collision_check: true
segment_sample_interval: 0.05
```

### 4.4 A* 细节

A* 使用 8 邻域：

- 直线代价 `1.0`
- 对角代价 `1.4142`
- heuristic 是欧氏距离
- cell 额外代价：

```text
tentative_g = previous_g + move_cost + astar_cost_weight * normalized_cost
normalized_cost = costmap_cost / LETHAL_OBSTACLE
```

如果启用 downsample，每个 downsample cell 覆盖的原始 costmap cell 必须全部可通行。

输出路径处理：

- A* cell path 转 world points。
- 起点/终点强制使用原始 start/goal cell 的 world 坐标。
- 按 `sample_interval` 重采样。
- 转成 `nav_msgs::Path` 并更新 yaw。

### 4.5 smoothing 细节

当前 smoothing 是轻量三点平滑：

```text
next[i] = 0.25 * prev + 0.50 * current + 0.25 * next
```

迭代 4 次。为了保护与原 `active_route` 的连接：

- 若替换段前有点，临时插入前一个原路径点作为锚点。
- 若替换段后有点，临时追加后一个原路径点作为锚点。
- 平滑后移除这些临时锚点。

如果 `segment_smoothing_collision_check=true`，平滑路径必须通过 `isPathCollisionFree()`，否则回退使用 A* 原始路径。

## 5. 与 du-zhong pure_pursuit_planner 对比

### 5.1 框架差异

| 项目 | 当前系统 | du-zhong |
|---|---|---|
| ROS 版本/接口 | ROS2 Nav2 `nav2_core::Controller` | ROS1 `nav_core::BaseLocalPlanner` |
| 控制器命名 | `origincar_navigation/OrigincarAckermannPurePursuitController` | `pure_pursuit_planner/PurePursuitPlanner` |
| 当前位姿来源 | Nav2 传入 pose，必要时 TF 转 frame | `costmap_ros_->getRobotPose()` |
| 当前速度来源 | Nav2 传入 `velocity.linear.x` | 普通版自己记录上一指令；back 版订阅 `/odom` |
| 动态参数 | Nav2 参数，当前无运行时 callback | ROS1 dynamic_reconfigure |
| 局部避障 | 有 `SegmentBypassPlanner` | 无内置局部避障 |
| 调试 local_plan | 发布完整实际跟踪路径 | 只发布机器人位姿到前视点的两点 path |

### 5.2 主控制逻辑继承关系

当前系统明显继承了 `du-zhong/src/pure_pursuit_planner.cpp` 普通版的核心流程：

```text
检查目标
-> 判断倒车
-> 自适应前视距离
-> 找前视点
-> 计算阿克曼转角
-> 计算线速度
-> 倒车时 linear.x 取负，angular.z 取反
```

当前系统改动：

- 把 `calculateSteering()` 返回值从“角度制”改成“弧度制”。
- 把 `max_angular_` 概念改成 `max_steering_angle_`。
- 把位姿/路径 TF 处理集中到 ROS2 `transformPose()`。
- 把 `global_plan_` 转换成 costmap frame 后，支持局部片段替换。
- 增加固定路线下的局部片段绕障。
- 到达判断容差与 Nav2 goal checker 对齐。

### 5.3 前视距离

两者公式基本一致：

```text
Ld = min_lookahead_distance + lookahead_ratio * abs(speed)
倒车时 Ld *= reverse_lookahead_ratio
```

差异：

- `du-zhong` 只做 `min(distance, max_lookahead_distance)`，没有显式下限 clamp。
- 当前系统做 `clamp(distance, min_lookahead_distance * 0.5, max_lookahead_distance)`，倒车时不会短到过小。
- 当前系统默认最大前视 `0.60m`，`du-zhong` 默认/配置上限常见为 `2.0m`。

### 5.4 前视点与虚拟延长点

当前系统和 `du-zhong` 普通版都支持终点虚拟延长：

- 最近点后找第一个距离超过 `Ld` 的点。
- 找不到时用终点。
- 需要延长时沿终点 yaw 增加 `lookahead_dist - dist_to_goal + 0.1`。
- 倒车时 yaw 翻转 `pi`。

差异：

- `du-zhong` 普通版的 `getLookaheadPoint()` 固定在 `global_plan_` 上操作。
- 当前系统在 `tracking_plan` 上操作，所以可能是原始全局路径，也可能是局部绕障后的 `active_route_`。
- `du-zhong back` 版没有虚拟延长，找不到满足前视距离的点时直接返回终点。

### 5.5 倒车判断

`du-zhong` 普通版：

```text
取 0.2m 前视点
转 base frame
x < 0 -> 倒车
```

当前系统：

```text
取 reverse_check_distance 前视点，默认 0.20m
转 robot_base_frame_
x < 0 -> 倒车
```

两者几乎一致。

`du-zhong back` 版更复杂：

- 支持 `auto_reverse_enable`。
- 倒车判断最短间隔 `0.1s`。
- 运动中根据当前 odom 线速度符号保持当前模式。
- 静止且未初始化时，检查前 3 个路径点；若至少 2 个点在 `reverse_commit_distance` 内且与车头夹角大于 90 度，则进入倒车。

当前系统没有这些 hysteresis，因此在路径点或 TF 抖动时，理论上比 `back` 版更可能出现前进/倒车模式来回切换。实际是否发生取决于路径连续性和 `reverse_check_distance` 附近点的稳定性。

### 5.6 转向角单位和限幅

`du-zhong`：

```text
angular_z = atan(2 * CARL * sin(alpha) / sqrt(L_sq)) * 57.3
```

该变量命名为 `angular_z`，但实际是“前轮转角，单位度”。随后：

```text
cmd_vel.angular.z = linear_x * tan(angular_z * deg_to_rad) / CARL
```

当前系统：

```text
delta = atan(2 * wheelbase * sin(alpha) / sqrt(distance_sq))
cmd_vel.angular.z = linear_speed * tan(delta) / wheelbase
```

当前系统在单位表达上更清楚：`delta` 全程弧度，并用 `max_steering_angle` 限幅。

### 5.7 终点姿态校正

`du-zhong` 普通版：

- `calculateSteering()` 内部接近终点 `0.25m` 时，把目标 yaw 误差乘 `kp=1.5` 后转成度，与纯追踪角度加权融合。
- `acalculateVel()` 接近终点 `0.3m` 时线性降速，角度误差超过 `0.2rad` 继续降速。

当前系统：

- 保留同样思想，但参数化：
  - `pose_adjust_distance`
  - `orientation_kp`
  - `goal_decel_distance`
  - `min_approach_linear_velocity`
  - `yaw_goal_tolerance`
- 全程弧度。
- 转向融合放在 `computeVelocityCommands()` 中，速度降速放在 `targetLinearVelocity()` 中。

`du-zhong back` 版：

- 接近目标 `0.4m` 时简单将转角乘 `1.5`。
- 距离减速代码中 `decel_distance = 0.0`，导致该分支正常情况下不会触发，还存在若触发会除零的风险。

### 5.8 速度控制

共同点：

- 前进/倒车分别有速度上限。
- 大转角减速。
- 速度输出转换为阿克曼底盘需要的 `linear.x` 与 `angular.z`。

差异：

- `du-zhong` 普通版有“速度差过大时限制加速”：`target_linear_vel - current_vel > decel_velocity_threshold` 时只增加一个阈值。
- 当前系统没有显式加速度限制，直接根据当前转角/距离输出目标速度。Nav2 controller server 或底盘层如果没有限加速度，速度变化可能比 `du-zhong` 普通版更快。
- 当前系统在局部绕障失败时保留旧路径继续跟踪。
- 当前系统保留 `setSpeedLimit()` 接口以满足 Nav2 控制器基类，但实现为空，不使用 Nav2 speed filter 动态限速。

### 5.9 到达判断

`du-zhong` 普通版：

- 位置距离 `< goal_tolerance`。
- 朝向误差阈值为 `2.0rad`，较宽。

`du-zhong back` 版：

- 位置距离 `< goal_tolerance`。
- 朝向误差阈值为 `0.2rad`。

当前系统：

- 位置阈值 `0.08m`。
- 朝向阈值 `0.25rad`。
- 与 Nav2 `SimpleGoalChecker` 参数一致。

## 6. 与 Nav2 Regulated Pure Pursuit 对比

### 6.1 总体定位

Nav2 原生 `RegulatedPurePursuitController` 是通用移动底盘控制器，核心输出是：

```text
linear_vel = regulated desired linear velocity
angular_vel = linear_vel * curvature
```

它把前视点变换到 robot base frame，直接基于曲率：

```text
curvature = 2 * y / (x^2 + y^2)
angular_vel = linear_vel * curvature
```

当前系统是阿克曼定制控制器，核心输出先计算前轮转角：

```text
delta = atan(2L sin(alpha) / Ld)
angular.z = linear * tan(delta) / wheelbase
```

从几何上，这两种在理想 bicycle model 下相关，但 Nav2 RPP 不显式限制前轮转角，也不按本项目底盘插件的反解协议组织符号。

### 6.2 路径处理

Nav2 RPP：

- `setPlan()` 保存 `global_plan_`。
- 每个周期 `transformGlobalPlan(pose)`：
  - 把机器人 pose 变换到 plan frame。
  - 找路径最近点。
  - 只转换 local costmap 范围内的近端路径。
  - 把近端路径转换到 robot base frame。
  - 从 `global_plan_` 中擦除已经经过的点，实现路径 pruning。

当前系统：

- `setPlan()` 保存全局路径，不裁剪。
- 每个周期把整条 `global_plan_` 转到 `global_frame_`。
- `active_route_` 由 `SegmentBypassPlanner` 裁剪并维护。
- 纯追踪前视点搜索在 costmap frame 的 `tracking_plan` 上进行，转向计算时再把目标点转到 base frame。

差异影响：

- Nav2 RPP 的 transformed plan 已经在 base frame，前视点距离计算以机器人原点为基准。
- 当前系统前视点距离计算在 costmap frame 中用机器人 pose 做欧氏距离，几何等价，但多一次 target 到 base frame 的 TF。
- Nav2 RPP 会修改 `global_plan_` 删除已过点；当前系统保留 `global_plan_`，另用 `active_route_` 裁剪，便于固定全局路线和局部片段替换。

### 6.3 前视距离

Nav2 RPP：

```text
if use_velocity_scaled_lookahead_dist:
  Ld = clamp(abs(speed.linear.x) * lookahead_time,
             min_lookahead_dist,
             max_lookahead_dist)
else:
  Ld = lookahead_dist
```

当前系统：

```text
Ld = min_lookahead_distance + lookahead_ratio * abs(speed)
if reverse:
  Ld *= reverse_lookahead_ratio
Ld = clamp(Ld, min_lookahead_distance * 0.5, max_lookahead_distance)
```

当前系统始终是速度相关前视；Nav2 RPP 可切换固定前视或速度缩放前视。

### 6.4 前视点插值

Nav2 RPP：

- 找第一个距离超过 `Ld` 的点。
- 如果 `use_interpolation=true` 且不是第一个点，会计算“路径线段与以机器人为圆心、半径 `Ld` 的圆”的精确交点。
- 这样可以减少稀疏路径导致的控制跳变。

当前系统：

- 直接取第一个距离大于等于 `Ld` 的离散路径点。
- 局部 A* / 平滑路径会按 `segment_sample_interval=0.05m` 重采样，因此局部绕障段比较密。
- 全局路径是否足够密取决于全局规划器输出。

审查点：如果全局路径点间距较大，当前系统前视点可能跳变；Nav2 RPP 的插值更平滑。

### 6.5 倒车逻辑

Nav2 RPP：

- 参数 `allow_reversing` 默认 false。
- 只有 `use_rotate_to_heading=false` 时才允许 reversing，两者不能同时 true。
- 如果允许倒车，会用 `findVelocitySignChange()` 查路径 cusp；若 cusp 比前视距离近，则前视距离截断到 cusp。
- 输出方向由 carrot pose 的 x 符号决定：

  ```text
  sign = carrot_pose.x >= 0 ? 1 : -1
  linear_vel = sign * constrained_speed
  ```

当前系统：

- 始终支持自动倒车，无 `allow_reversing` 开关。
- 不查 cusp。
- 用短前视点 `reverse_check_distance` 的 x 符号决定 `reverse_mode_`。
- 倒车时使用独立速度上限 `max_reverse_velocity`。

差异影响：

- Nav2 RPP 更适合含前进/倒车 cusp 的全局路径，因为会避免前视点跨过 cusp。
- 当前系统更接近 `du-zhong` 逻辑，适合简单判断“短前视点在车后则倒车”，但没有 cusp 截断，复杂倒车路径需要重点验证。

### 6.6 旋转到路径/目标朝向

Nav2 RPP：

- 支持 `use_rotate_to_heading`。
- 起步时如果 carrot 方向与车体差异大，可原地旋转到路径方向。
- 接近 goal 时可旋转到 goal heading。
- 旋转角速度受 `rotate_to_heading_angular_vel` 和 `max_angular_accel` 限制。

当前系统：

- 阿克曼小车不能原地旋转，因此没有 rotate in place。
- 通过接近终点时融合 `orientation_kp * yaw_error` 到前轮转角来校正朝向。

这点是底盘模型差异导致的设计差异，不是遗漏。

### 6.7 速度 regulation

Nav2 RPP 的 regulation 更完整：

- 曲率限速：

  ```text
  radius = abs(1 / curvature)
  if radius < regulated_linear_scaling_min_radius:
    velocity scales down
  ```

- 障碍物接近限速：
  - 读取机器人当前位置 costmap cost。
  - 结合 inflation layer 参数估算离障碍距离。
  - 小于 `cost_scaling_dist` 时按 `cost_scaling_gain` 降速。

- 接近终点限速：
  - 计算 transformed path 剩余长度。
  - 小于 `approach_velocity_scaling_dist` 时降速到不低于 `min_approach_linear_velocity`。

当前系统：

- 转角限速：按 `delta / max_steering_angle` 二次降速。
- 终点距离限速：按到 `global_plan_.back()` 的欧氏距离线性降速。
- 终点朝向误差限速。
- 局部绕障失败时保留旧路径继续跟踪。
- 没有基于当前 pose cost 的连续障碍接近限速。
- 没有基于 time-to-collision 的预测碰撞异常。

### 6.8 碰撞检测与避障机制

Nav2 RPP：

- 不重写路径。
- 对当前速度指令做前向仿真。
- 在 `max_allowed_time_to_collision_up_to_carrot` 时间内，且不超过 carrot 距离，沿弧线检查 footprint collision。
- 发现 collision ahead 时抛出异常，交给 Nav2 行为树恢复/重规划。
- 发布 `lookahead_collision_arc` 可视化。

当前系统：

- 不对最终 `cmd_vel` 做 footprint time-to-collision 预测。
- 会扫描 `active_route_` 前方路径段是否被 local costmap lethal cell 阻断。
- 若阻断，局部 A* 搜一段绕障路径并替换回 `active_route_`。
- 若失败，不抛异常，继续旧路径等待下一次片段重规划。

本质差异：

```text
Nav2 RPP: 速度命令碰撞预测，失败即报错/恢复
当前系统: 路径片段局部改写，失败时保守继续
```

当前系统更符合“固定全局路线 + 局部绕障”的项目目标，但其安全性依赖：

- local costmap 障碍检测准确
- 替换段 A* 成功率
- smoothing 后碰撞检查
- fallback 速度足够低
- 下游底盘/仿真对障碍的额外保护

### 6.9 参数能力对比

| 能力 | 当前系统 | Nav2 RPP |
|---|---|---|
| 固定前视距离 | 否 | 是 |
| 速度缩放前视距离 | 是，固定公式 | 是，可开关 |
| 前视点插值 | 否 | 是 |
| 阿克曼前轮转角限幅 | 是 | 否，输出通用 twist |
| 自动倒车 | 是 | 需 `allow_reversing=true` |
| cusp 检测 | 否 | 是 |
| 原地旋转到路径 | 否 | 是 |
| 曲率限速 | 以转角近似 | 是 |
| cost 接近限速 | 否 | 是 |
| time-to-collision | 否 | 是 |
| 局部片段 A* 绕障 | 是 | 否 |
| 保留固定全局路线 | 是 | 默认会 prune global plan |
| 动态参数回调 | 否 | 是 |
| Nav2 speed filter | 接口保留但不生效 | 是 |

## 7. 当前实现值得重点审查的点

### 7.1 倒车模式缺少状态保持

当前每个控制周期用 `reverse_check_distance` 处点的 `x < 0` 判断倒车。若路径点在车体前后边界附近抖动，可能导致模式切换频繁。

`du-zhong back` 版用 0.1s 检查节流、运动中保持当前速度符号、静止时多点投票，稳定性更强。

建议审查：

- 倒车路径起点附近是否存在前后反复切换。
- 局部绕障替换后 `active_route_` 的首段 yaw 是否可能导致误判。
- 是否需要引入 hysteresis 或最小保持时间。

### 7.2 没有前视点插值

当前取离散点，路径稀疏时 `target_pose` 可能跳变。局部绕障段有 `0.05m` 重采样，但全局路径密度需确认。

建议审查：

- `OrigincarGlobalPlanner` 输出点间距。
- RViz 中 `FollowPath/target_pose` 是否平滑移动。
- 是否需要借鉴 Nav2 RPP 的 circle-segment intersection 插值。

### 7.3 没有命令级碰撞预测

当前避障检查对象是路径段，不是最终速度指令在未来若干秒内扫过的 footprint。

建议审查：

- 阿克曼转弯弧线是否可能扫到障碍，而路径中心线没有被判 blocked。
- 局部 A* 路径是否考虑了机器人 footprint 或仅依赖 costmap inflation。
- 是否需要补充类似 Nav2 RPP 的 `lookahead_collision_arc` 检查。

### 7.4 绕障失败后继续旧路径的安全性

片段绕障失败时：

```text
active_tracking_path_ = active_route_
```

如果 `active_route_` 前方仍被障碍阻断，车辆仍会按正常纯追踪速度规划继续旧路径。

建议审查：

- 是否应在障碍距离过近时输出 0，而不是继续低速。
- 是否需要记录连续失败次数并触发 Nav2 recovery。

### 7.5 速度缺少加速度限制

`du-zhong` 普通版有 `decel_velocity_threshold` 限制目标速度向上跳变。当前系统没有等价逻辑。

建议审查：

- 下游底盘插件或控制链是否有限加速度。
- 从转弯低速恢复直线高速时是否有突变。
- 是否需要在控制器内部加入 acceleration clamp。

### 7.6 global_plan_ 与 active_route_ 双终点来源

当前：

- 前视点和终点姿态融合使用 `tracking_plan`。
- 终点减速和到达判断使用 `global_plan_.back()`。

通常两者终点一致，但如果未来 `active_route_` 被更复杂逻辑截断或替换，可能出现不一致。

建议审查：

- 局部替换是否永远保留原最终目标。
- 有无特殊行为树/规划器会发短路径或中间目标。

### 7.7 local costmap frame 命名

变量名 `global_frame_` 实际来自 `costmap_ros_->getGlobalFrameID()`。在 local costmap 中它通常可能是 `odom` 或 `map`，不是必然全局地图 frame。

当前注释已说明它是 local costmap 的 world frame，但审查配置时要确认：

- local costmap global frame 与路径 frame 的 TF 是否稳定。
- `transformPose()` 使用 stamp 0 是否符合所有路径来源。

## 8. 当前系统与两个参考实现的核心差异摘要

当前系统相对 `du-zhong`：

- 更 ROS2/Nav2 化：插件生命周期、`TwistStamped`、Nav2 参数、speed limit。
- 更清晰的阿克曼单位：内部转角用弧度。
- 增加固定全局路线下的局部片段绕障。
- 到达判断更严格且与 Nav2 goal checker 对齐。
- 少了 `du-zhong` 普通版的速度跳变限制。
- 少了 `du-zhong back` 版的倒车状态保持。

当前系统相对 Nav2 RPP：

- 更面向阿克曼底盘：显式 wheelbase、steering angle、倒车符号。
- 更面向固定路线：不让局部障碍触发全局重规划，而是替换局部片段。
- 少了 Nav2 RPP 的前视点插值、cusp 检测、cost regulation、time-to-collision。
- 不支持原地旋转，这对阿克曼是合理选择。
- 调试话题更贴合本项目绕障：`astar_path`、`optimized_local_path`、`local_plan`。

## 9. 建议的审查/测试清单

1. 无障碍直线：
   - `local_plan` 应等于全局路径或其裁剪后的 `active_route`。
   - `target_pose` 应沿路径平滑前移。
   - 速度应接近 `max_linear_velocity`。

2. 无障碍急弯：
   - `delta` 到达 `decel_angle_threshold` 后应降速。
   - `cmd_vel.angular.z` 应与前轮转向方向一致。

3. 终点姿态：
   - 距终点 `0.30m` 内开始线性降速。
   - 距终点 `0.25m` 内终点 yaw 误差应影响转角。
   - 到达时必须同时满足 `0.08m` 与 `0.25rad`。

4. 倒车路径：
   - `reverse_check_distance=0.20m` 前视点在车后时进入倒车。
   - 倒车速度不超过 `0.30m/s`。
   - 倒车时前轮方向与路径收敛方向一致。
   - 检查是否发生前进/倒车抖动。

5. 局部障碍：
   - 障碍进入 `segment_scan_distance=2.0m` 内后，应发布 `astar_path`。
   - smoothing 成功时应发布 `optimized_local_path`。
   - `local_plan` 只在受阻片段附近偏离原路线。

6. 绕障失败：
   - 控制器不应清空 `active_route_`。
   - 控制器会继续跟踪旧路径，需确认障碍距离足够安全。
   - 连续失败时车辆是否仍安全，需要实车/仿真确认。

7. 路径稀疏：
   - 检查 `target_pose` 是否跳变。
   - 必要时补前视点插值或提高全局路径采样密度。

8. 近障转弯：
   - 重点检查车体 footprint 是否可能扫到障碍。
   - 若中心线安全但车体会碰撞，需要补 footprint/time-to-collision 检查或增大 inflation。

## 10. 一句话结论

当前系统是一个“阿克曼定制 + ROS1 纯追踪逻辑迁移 + Nav2 插件化 + 固定路线局部片段绕障”的控制器；它比 `du-zhong` 更工程化、比 Nav2 RPP 更贴合本项目固定路线与阿克曼底盘，但在倒车状态稳定、前视点插值、命令级碰撞预测和速度加速度约束方面，需要作为下一轮审查重点。
