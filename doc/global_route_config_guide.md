# 全局规划器路径配置说明

本文说明如何配置自定义全局规划器使用的三条固定路线、如何从 RViz 获取点位、如何预览路线，以及如何在导航时选择路线。

## 1. 配置文件位置

路线配置文件默认位于：

```bash
/home/xjl/digua/src/origincar_navigation/config/global_routes.yaml
```

如果 `nav2_params.yaml` 中保持：

```yaml
GridBased:
  routes_file: ""
```

导航启动时会读取安装目录中的：

```bash
/home/xjl/digua/install/origincar_navigation/share/origincar_navigation/config/global_routes.yaml
```

如果你希望调试时直接修改源码目录里的配置并重启 Nav2 后生效，可以把 `routes_file` 改成绝对路径：

```yaml
GridBased:
  routes_file: "/home/xjl/digua/src/origincar_navigation/config/global_routes.yaml"
```

注意：全局规划器在启动时读取路线文件。修改路线文件后，需要重启 Nav2 才能让导航使用新路线。

## 2. 路线文件格式

`global_routes.yaml` 的基本结构如下：

```yaml
routes:
  route_1:
    points:
      - x: -1.8
        y: -1.4
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      - x: -0.6
        y: -1.2
        orientation: {x: 0.0, y: 0.0, z: 0.0998334, w: 0.9950042}

  route_2:
    points:
      - x: -1.5
        y: 0.8
        orientation: {x: 0.0, y: 0.0, z: -0.0499792, w: 0.9987503}

  route_3:
    points:
      - x: -1.2
        y: -0.4
        orientation: {x: 0.0, y: 0.0, z: 0.5646425, w: 0.8253356}
```

字段含义：

- `route_1 / route_2 / route_3`：三条可选路线。
- `points`：该路线的点位列表，按从前到后的顺序填写。
- `x / y`：点在 `map` 坐标系下的位置。
- `orientation`：点位方向，使用四元数 `{x, y, z, w}`。

每条路线至少建议配置 2 个点。点越多，拟合出来的路径越接近你指定的路线形状。

## 3. 从 RViz 获取点位

推荐使用本包提供的格式化工具，它会把 RViz 点选结果直接转换成可粘贴到 `global_routes.yaml` 的格式。

监听 `2D Pose Estimate`：

```bash
source /opt/ros/humble/setup.bash
source /home/xjl/digua/install/setup.bash
ros2 run origincar_navigation route_point_formatter.py --ros-args -p topic:=/initialpose
```

然后在 RViz 中点击 `2D Pose Estimate`，终端会输出：

```yaml
# from /initialpose
- x: -1.230000
  y: 0.450000
  orientation: {x: 0.000000, y: 0.000000, z: 0.382683, w: 0.923880}
  # yaw_deg: 45.000000
```

直接复制前三行到 `route_1.points`、`route_2.points` 或 `route_3.points` 下面即可。`yaw_deg` 是注释，只用于人工核对方向。

监听 `2D Goal`：

```bash
ros2 run origincar_navigation route_point_formatter.py --ros-args -p topic:=/goal_pose
```

如果只想接收一次后自动退出：

```bash
ros2 run origincar_navigation route_point_formatter.py --ros-args -p topic:=/initialpose -p once:=true
```

也可以直接使用 ROS 原始 echo 输出，但复制会麻烦一些。

方法一：使用 `2D Pose Estimate`

```bash
ros2 topic echo /initialpose --once
```

然后在 RViz 中点击 `2D Pose Estimate`，在地图上按住并拖动方向。终端会输出类似：

```yaml
pose:
  pose:
    position:
      x: -1.23
      y: 0.45
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.382683
      w: 0.923880
```

把它整理到路线文件中：

```yaml
- x: -1.23
  y: 0.45
  orientation: {x: 0.0, y: 0.0, z: 0.382683, w: 0.923880}
```

方法二：使用 `2D Goal`

```bash
ros2 topic echo /goal_pose --once
```

然后在 RViz 中点击 `2D Goal`。同样复制输出中的 `position.x`、`position.y` 和 `orientation.x/y/z/w`。

如果 `/goal_pose` 没有数据，说明当前 RViz 工具或 Nav2 面板没有发布该话题；使用 `/initialpose` 更稳定。

## 4. 使用角度格式

如果你只想手写方向，也可以不用四元数，写角度：

```yaml
- x: 1.0
  y: 0.5
  yaw_deg: 90.0
```

`yaw_deg` 单位是度：

- `0`：朝地图 x 正方向。
- `90`：朝地图 y 正方向。
- `180` 或 `-180`：朝地图 x 负方向。
- `-90`：朝地图 y 负方向。

也支持弧度字段：

```yaml
- x: 1.0
  y: 0.5
  yaw: 1.5708
```

推荐优先使用 `orientation` 或 `yaw_deg`，少手算，少出错。

## 5. 预览路线点

已经提供了单独的路线点可视化节点。启动：

```bash
source /opt/ros/humble/setup.bash
source /home/xjl/digua/install/setup.bash
ros2 launch origincar_navigation route_markers.launch.py routes_file:=/home/xjl/digua/src/origincar_navigation/config/global_routes.yaml
```

RViz 设置：

1. `Fixed Frame` 设置为 `map`。
2. Add -> `MarkerArray`。
3. Topic 选择：

```text
/global_route_markers
```

显示内容：

- 小球：点位位置。
- 箭头：点位方向。
- 线条：点位连接顺序。
- 文字：点位编号，例如 `route_1_1`、`route_1_2`。

这个可视化节点会周期性重新读取 `routes_file`，所以你修改源码目录中的 `global_routes.yaml` 后，RViz 中会自动刷新。

## 6. 编译与安装

修改配置文件后，如果导航读取的是安装目录配置，需要重新编译安装：

```bash
source /opt/ros/humble/setup.bash
cd /home/xjl/digua
colcon build --packages-select origincar_navigation
source install/setup.bash
```

如果 `routes_file` 已经指向源码目录：

```yaml
routes_file: "/home/xjl/digua/src/origincar_navigation/config/global_routes.yaml"
```

则修改路线文件后不需要重新编译，但需要重启 Nav2，因为全局规划器只在启动时加载路线配置。

## 7. 导航时选择路线

自定义规划器注册在 `GridBased` 下，兼容 Nav2 默认行为树的 `planner_id="GridBased"`。

发送目标时，通过 `goal.pose.position.z` 选择路线：

- `z = 1.0`：使用 `route_1`
- `z = 2.0`：使用 `route_2`
- `z = 3.0`：使用 `route_3`
- `z = 0.0` 或非法值：回退到 Navfn 普通规划

示例：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 0.0, y: 0.0, z: 1.0},
      orientation: {w: 1.0}
    }
  }
}"
```

这里 `z: 1.0` 表示选择 `route_1`。对于固定路线模式，目标的 `x/y` 主要用于满足 Nav2 action 格式；真正的全局路径来自 `global_routes.yaml`。

## 8. 路径拟合相关参数

路径拟合参数在 `nav2_params.yaml` 中：

```yaml
GridBased:
  tension: 1.5
  min_interpolation_steps: 10
  path_resolution_factor: 10.0
  trim_distance: 0.15
```

含义：

- `tension`：Hermite 曲线张力。越大越强调点位方向，曲线可能更“绕”；越小越贴近点间连线。
- `min_interpolation_steps`：每段最少插值点数量。
- `path_resolution_factor`：按距离增加插值点的倍率。
- `trim_distance`：从当前车位置裁剪已走路径的距离阈值，单位米。

一般先只改点位，不改这些参数。路线形状调顺后，再微调 `tension`。

## 9. 常见问题

### RViz 中看不到路线点

检查：

```bash
ros2 topic list | grep global_route_markers
```

如果没有话题，说明可视化节点没启动。

如果有话题但 RViz 不显示：

- `Fixed Frame` 是否是 `map`。
- MarkerArray 的 Topic 是否是 `/global_route_markers`。
- 路线文件是否 YAML 格式错误。

### 导航时没有读取新路线

原因通常是导航读取的是安装目录配置，而你改的是源码目录配置。

解决方式二选一：

1. 修改后重新编译：

```bash
colcon build --packages-select origincar_navigation
source install/setup.bash
```

2. 在 `nav2_params.yaml` 中把 `routes_file` 指向源码目录绝对路径，然后重启 Nav2。

### 发送目标后一直等待 action server

说明 Nav2 还没有启动完成，或者当前终端和 Nav2 终端的 ROS 环境不一致。

检查：

```bash
ros2 action list
ros2 node list
echo $ROS_DOMAIN_ID
```

需要看到：

```text
/navigate_to_pose
```

并且启动日志中应出现：

```text
Managed nodes are active
```
