# 自定义阿克曼纯追踪控制器说明

本文档说明 `origincar_navigation/OrigincarAckermannPurePursuitController` 的工作方式、参数和使用方法。

## 功能范围

当前阶段只实现纯追踪控制器，用于跟踪 Nav2 全局路径。A* + 3 阶 B 样条局部避障暂未接入控制代码，后续会在获得参考实现后融合。

控制器实现内容：

- 实现 Nav2 `nav2_core::Controller` 插件接口。
- 接收 `planner_server` 下发的全局路径。
- 根据当前速度计算动态前视距离。
- 搜索全局路径上的前视点。
- 将前视点转换到 `base_footprint` 坐标系。
- 计算阿克曼转向角 `delta`。
- 输出 `/cmd_vel` 中的 `linear.x` 和 `angular.z`。
- 保留前进/倒车判断、倒车跟踪、转弯减速、终点减速和终点姿态校正。

## 控制流程

每次 `controller_server` 调用 `computeVelocityCommands()` 时，控制器执行以下流程：

1. 检查全局路径是否为空。
2. 根据当前位置判断是否满足目标位置和角度容差。
3. 将全局路径转换到当前机器人位姿所在坐标系。
4. 用短前视点判断是否需要倒车。
5. 计算动态前视距离：

   ```text
   lookahead = min_lookahead_distance + lookahead_ratio * abs(current_speed)
   ```

   倒车时再乘以 `reverse_lookahead_ratio`。

6. 从最近路径点向前搜索满足前视距离的点。
7. 将前视点转换到 `base_footprint`。
8. 计算转向角：

   ```text
   delta = atan(2 * wheelbase * sin(alpha) / lookahead_distance)
   ```

9. 根据转向角和距离终点的距离计算目标线速度。
10. 输出 `/cmd_vel`。

## `/cmd_vel.angular.z` 与阿克曼转向角

当前仿真底盘插件订阅 `/cmd_vel` 后，会用下面关系从角速度反解转向角：

```text
steering_angle = atan(wheelbase * angular.z / linear.x)
```

因此控制器按阿克曼模型输出：

```text
angular.z = v * tan(delta) / wheelbase
```

这样底盘插件反解后得到的转向角约等于 `delta`。当前仿真默认轴距为 `0.189m`，最大转角为 `0.65rad`，控制器参数需要与底盘参数保持一致。

注意：在本系统中，`/cmd_vel.angular.z` 更像是“给底盘插件反解转向角的中间量”，不是直接控制真实差速底盘的目标角速度。

因为 `angular.z` 是转向角反解中间量，速度平滑器不能按普通差速车设置过小的角速度上限。例如 `v=0.7m/s`、`delta=0.65rad`、`wheelbase=0.189m` 时：

```text
angular.z = 0.7 * tan(0.65) / 0.189 ~= 2.83rad/s
```

如果 `velocity_smoother.max_velocity[2]` 只有 `1.0rad/s`，底盘插件反解出的前轮转角会被压到约 `0.26rad`，车辆会表现为“只轻微打方向，然后一直向前走”。当前配置已将 `wz` 上限调整为 `3.0rad/s`。

## 倒车逻辑

倒车逻辑参考 ROS1 `pure_pursuit_planner` 的实现思路：

- 使用 `reverse_check_distance` 对应的短前视点进行判断。
- 如果该点转换到 `base_footprint` 后 `x < 0`，认为目标在车体后方，进入倒车模式。
- 倒车时 `linear.x` 输出负值。
- 倒车速度上限使用 `max_reverse_velocity`。
- 倒车前视距离乘以 `reverse_lookahead_ratio`。
- 倒车时保留参考实现的角速度方向约定，使底盘反解出的转向角符合倒车跟踪需要。

## 参数说明

参数位于 `src/origincar_navigation/config/nav2_params.yaml` 的 `controller_server.ros__parameters.FollowPath` 下。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `plugin` | `origincar_navigation/OrigincarAckermannPurePursuitController` | 控制器插件类型 |
| `wheelbase` | `0.189` | 前后轴距，需与仿真/实车一致 |
| `max_steering_angle` | `0.65` | 最大转向角，单位 rad |
| `min_lookahead_distance` | `0.25` | 最小前视距离 |
| `max_lookahead_distance` | `0.50` | 最大前视距离 |
| `lookahead_ratio` | `0.35` | 速度到前视距离的比例 |
| `max_linear_velocity` | `0.70` | 最大前进速度 |
| `max_reverse_velocity` | `0.30` | 最大倒车速度绝对值 |
| `reverse_lookahead_ratio` | `0.40` | 倒车前视距离缩放比例 |
| `reverse_check_distance` | `0.20` | 判断倒车的短前视距离 |
| `goal_tolerance` | `0.08` | 目标位置容差 |
| `yaw_goal_tolerance` | `1.05` | 目标朝向容差，约 60 度 |
| `decel_angle_threshold` | `0.35` | 触发转弯减速的转角阈值 |
| `decel_ratio` | `0.80` | 转弯减速比例 |
| `min_turn_speed_factor` | `0.20` | 急转弯时最小速度比例 |
| `max_turn_linear_velocity` | `0.30` | 转弯时线速度上限 |
| `goal_decel_distance` | `0.30` | 接近终点开始减速的距离 |
| `min_approach_linear_velocity` | `0.05` | 接近终点的最低速度 |
| `pose_adjust_distance` | `0.25` | 接近终点时增强朝向校正的距离 |
| `orientation_kp` | `1.50` | 终点朝向校正比例 |
| `transform_tolerance` | `0.20` | TF 查询容差 |

## 编译

在工作空间根目录执行：

```bash
colcon build --packages-select origincar_navigation
source install/setup.bash
```

## 启动

按项目原有方式启动仿真、定位和导航。`nav2_params.yaml` 已将 `FollowPath` 切换为自定义控制器，启动 `controller_server` 时会自动加载。

如果启动失败，优先检查：

- 是否已经重新 `source install/setup.bash`。
- `controller_server` 日志中是否出现插件加载失败。
- `plugins/origincar_nav_plugins.xml` 是否安装到了 `install/origincar_navigation/share/origincar_navigation/plugins/`。

## RViz 调试话题

控制器发布以下调试话题：

- `/controller_server/FollowPath/local_plan`：当前控制器使用的路径。
- `/controller_server/FollowPath/target_pose`：当前纯追踪前视点。
- `/controller_server/FollowPath/goal_reached`：控制器内部目标到达状态。

在 RViz 中添加 `Path` 和 `Pose` 显示即可检查前视点是否落在路径前方、倒车时目标点是否位于车体后方。
