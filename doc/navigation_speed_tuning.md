# 小车导航提速说明

小车速度主要由 Nav2 控制器和速度平滑器限制。当前参数偏保守，适合小地图低速调试。

## 1. 主要速度参数

配置文件：

```bash
/home/xjl/digua/src/origincar_navigation/config/nav2_params.yaml
```

### 控制器期望速度

位置：

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      desired_linear_vel: 0.18
```

含义：

- `desired_linear_vel` 是路径跟踪控制器期望的前进速度，单位 m/s。
- 当前是 `0.18`，比较慢。
- 可以先试：

```yaml
desired_linear_vel: 0.30
```

如果路径比较顺、车不抖，再试：

```yaml
desired_linear_vel: 0.40
```

### 速度平滑器上限

位置：

```yaml
velocity_smoother:
  ros__parameters:
    max_velocity: [0.25, 0.0, 1.0]
    max_accel: [0.4, 0.0, 1.5]
    max_decel: [-0.4, 0.0, -1.5]
```

含义：

- `max_velocity[0]` 是线速度上限。
- 如果 `desired_linear_vel` 调到 `0.30`，但 `max_velocity[0]` 仍是 `0.25`，实际速度仍会被裁剪到 `0.25`。
- `max_accel[0]` 是加速度上限，越大起步越快。
- `max_decel[0]` 是减速度上限，负值绝对值越大刹车越快。

建议第一档提速：

```yaml
FollowPath:
  desired_linear_vel: 0.30

velocity_smoother:
  max_velocity: [0.35, 0.0, 1.2]
  max_accel: [0.6, 0.0, 2.0]
  max_decel: [-0.6, 0.0, -2.0]
```

建议第二档提速：

```yaml
FollowPath:
  desired_linear_vel: 0.40

velocity_smoother:
  max_velocity: [0.45, 0.0, 1.5]
  max_accel: [0.8, 0.0, 2.5]
  max_decel: [-0.8, 0.0, -2.5]
```

## 2. 前视距离参数

速度提高后，前视距离也要适当增加，否则车会追得太近，转弯容易抖。

当前：

```yaml
lookahead_dist: 0.35
min_lookahead_dist: 0.15
max_lookahead_dist: 0.6
lookahead_time: 1.5
```

第一档提速建议：

```yaml
lookahead_dist: 0.45
min_lookahead_dist: 0.20
max_lookahead_dist: 0.8
lookahead_time: 1.5
```

第二档提速建议：

```yaml
lookahead_dist: 0.55
min_lookahead_dist: 0.25
max_lookahead_dist: 1.0
lookahead_time: 1.5
```

## 3. 修改后如何生效

修改 `nav2_params.yaml` 后：

```bash
source /opt/ros/humble/setup.bash
cd /home/xjl/digua
colcon build --packages-select origincar_navigation
source install/setup.bash
```

然后重启 Nav2。

如果 launch 使用的是源码参数文件路径，则不需要重新编译，但仍需要重启 Nav2。

## 4. 怎么确认速度是否被限制

查看控制输出：

```bash
ros2 topic echo /cmd_vel
```

如果 `/cmd_vel.linear.x` 一直不超过 `0.25`，说明速度平滑器或参数上限还在限制。

查看平滑后速度：

```bash
ros2 topic echo /cmd_vel_smoothed
```

如果系统中没有 `/cmd_vel_smoothed`，以实际 `ros2 topic list` 为准。

## 5. 调参建议

建议按顺序调：

1. 先把 `desired_linear_vel` 从 `0.18` 调到 `0.30`。
2. 同步把 `max_velocity[0]` 调到至少 `0.35`。
3. 如果起步太慢，再增大 `max_accel[0]`。
4. 如果转弯抖，增大 `lookahead_dist` 和 `max_lookahead_dist`。
5. 如果接近终点冲过头，减小速度或增大减速度绝对值。

不要只改 `desired_linear_vel`。如果速度平滑器上限不改，车还是会慢。
