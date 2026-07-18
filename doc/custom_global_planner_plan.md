# 自定义三路线 Nav2 全局规划器实现计划

## 目标

在 `origincar_navigation` 包内实现一个 Nav2 全局规划器插件：

- 从独立配置文件 `config/global_routes.yaml` 读取三条预设路径。
- 每条路径由若干点位组成，每个点位包含 `x、y、orientation quaternion`，也兼容 `yaw_deg` 和 `yaw`。
- 规划请求中 `goal.pose.position.z = 1/2/3` 时选择对应路径。
- `goal.pose.position.z = 0` 或非法值时回退到 Nav2 自带 `NavfnPlanner`，兼容 RViz 普通导航目标。
- 参考 `base_graph_global_planner` 的 `SmoothHermiteGenerator` 思路，对预设路径点进行平滑 Hermite 拟合。

## 配置方案

插件注册名：

```yaml
origincar_navigation/OrigincarGlobalPlanner
```

规划器参数位于：

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: origincar_navigation/OrigincarGlobalPlanner
```

`nav2_params.yaml` 只保留规划器参数和路线文件路径：

```yaml
GridBased:
  plugin: origincar_navigation/OrigincarGlobalPlanner
  route_ids: ["1", "2", "3"]
  routes_file: ""
```

`GridBased` 这个实例名要保留，因为 Nav2 默认行为树会请求 `planner_id=GridBased`。`routes_file` 为空时，插件默认读取安装目录下的 `config/global_routes.yaml`；需要临时切换时可填写绝对路径。

路线文件格式如下：

```yaml
routes:
  route_1:
    points:
      - x: -1.8
        y: -1.4
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      - x: -0.6
        y: -1.2
        yaw_deg: 30.0
```

推荐使用 `orientation` 四元数格式，因为 RViz/ROS 话题会直接输出这四个值，不需要自己计算弧度。

## 从 RViz 获取点位

1. 在 RViz 用 `2D Pose Estimate` 点一个位姿，然后执行：

```bash
ros2 topic echo /initialpose --once
```

2. 或者用 `2D Goal` 点一个位姿，然后执行：

```bash
ros2 topic echo /goal_pose --once
```

3. 把输出里的 `pose.position.x`、`pose.position.y`、`pose.orientation.x/y/z/w` 复制到 `global_routes.yaml`。

如果只想手写方向，也可以用角度：

```yaml
- x: 1.0
  y: 0.5
  yaw_deg: 90.0
```

## 实现步骤

1. 新增 `OrigincarGlobalPlanner`，实现 `nav2_core::GlobalPlanner` 的生命周期接口和 `createPlan()`。
2. 在 `createPlan()` 中解析 `goal.pose.position.z`：
   - `1/2/3`：选择对应配置路线。
   - 其他值：调用内部 `NavfnPlanner` 回退规划。
3. 将当前车位姿 `start` 作为路径第一个点，后面拼接独立配置文件中的点位。
4. 使用 ROS2 版平滑 Hermite 拟合生成 `nav_msgs::msg::Path`。
5. 从生成路径后向前查找距离当前车位姿小于 `trim_distance` 的点，裁掉已走过路径。
6. 更新 `CMakeLists.txt`、`package.xml`、`plugins/origincar_nav_plugins.xml` 和 `nav2_params.yaml` 完成编译、安装和插件加载。

## 验证项

- `colcon build --packages-select origincar_navigation` 可以通过。
- Nav2 `planner_server` 可以加载 `origincar_navigation/OrigincarGlobalPlanner`。
- RViz 普通目标 `z=0` 时仍可使用 Navfn 规划。
- 手动发送 `goal.pose.position.z=1/2/3` 时分别生成三条配置路线。
- 路径 frame 使用全局代价地图 frame，当前默认是 `map`。
