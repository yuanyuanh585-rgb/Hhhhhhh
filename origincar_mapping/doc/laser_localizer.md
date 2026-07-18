# 纯激光全局定位说明

本文档说明 `origincar_mapping` 包中的 `laser_global_localizer` 节点。该节点用于替换 AMCL，基于静态栅格地图、2D 激光和 IMU 方向先验估计机器人在 `map` 坐标系中的位姿，并发布 Nav2 需要的 `map -> odom`。

## 算法概览

输入：

- `/map`：`nav_msgs/OccupancyGrid`，由 `nav2_map_server` 发布。
- `/scan`：`sensor_msgs/LaserScan`，由仿真雷达或实际雷达驱动发布。
- `/imu/data`：`sensor_msgs/Imu`，用于提供 yaw 方向先验。
- TF：需要已有 `odom -> base_footprint` 和 `base_footprint -> laser_link`。

输出：

- `/laser_pose`：`geometry_msgs/PoseWithCovarianceStamped`，定位结果。
- TF `map -> odom`：供 Nav2 形成 `map -> odom -> base_footprint` 的完整定位链路。

核心流程：

1. 地图到达后，节点把占据栅格转换成障碍距离场。
2. 距离场进一步预计算成 `distance_scores_`，运行时评分只做查表，避免每个候选位姿都计算指数函数。
3. 激光到达后，节点按 `scan_step` 降采样，并把激光端点转换到 `base_footprint` 坐标系。
4. 在地图自由空间内做粗搜索：遍历降采样后的自由栅格位置，并在 yaw 候选中寻找最高匹配分数。
5. 围绕粗搜索最佳结果做局部细搜索。
6. 用最佳 `map -> base_footprint` 和当前 `odom -> base_footprint` 计算 `map -> odom`。
7. 定位匹配按 `localization_update_frequency` 更新；TF 按 `tf_publish_frequency` 独立重发最近一次结果，避免 Nav2 查询 TF 时落后。

## IMU 方向先验

场地地图接近正方形时，纯激光全局匹配可能出现 90 度或 180 度对称误匹配。节点默认启用 IMU yaw 先验：

- `use_imu_yaw_prior: true`
- `imu_topic: /imu/data`
- `imu_yaw_search_window_deg: 30.0`
- `imu_timeout: 1.0`

启用后，粗搜索不再扫描完整 360 度，而只在 IMU yaw 附近 `±imu_yaw_search_window_deg` 内搜索。这样既降低计算量，也能避免正方形地图中的方向歧义。

如果 IMU 数据超时或未收到，节点会自动退回完整 yaw 搜索，保证仍能定位。

## 主要参数

参数位于 `origincar_navigation/config/nav2_params.yaml` 的 `laser_localizer` 段。

- `scan_step`：激光降采样步长。当前默认 `10`，720 点激光约使用 72 条射线。
- `coarse_xy_step`：粗搜索位置分辨率，默认 `0.18m`。
- `fine_xy_step`：细搜索位置分辨率，默认 `0.06m`。
- `coarse_yaw_step_deg`：粗搜索 yaw 步长，默认 `5deg`。
- `fine_yaw_step_deg`：细搜索 yaw 步长，默认 `1deg`。
- `localization_update_frequency`：定位匹配频率，默认 `10Hz`。
- `tf_publish_frequency`：`map -> odom` 重发频率，默认 `20Hz`。
- `score_sigma`：距离场评分宽度，值越大越宽容，默认 `0.08m`。
- `min_score`：最低接受分数，默认 `0.0`。

## 计算量说明

当前默认参数主要为实时性服务：

- 地图约 `169 x 169`，自由栅格约 `27684`。
- `coarse_xy_step=0.18`，地图分辨率 `0.03`，粗搜索约每 6 格取一个候选位置。
- `scan_step=10`，720 点激光约取 72 条射线。
- 启用 IMU 先验后，yaw 候选只在方向窗口内搜索，默认约 13 个候选。

粗搜索规模约为：

```text
候选位置数 * yaw 候选数 * 激光点数
约 780 * 13 * 72 ~= 73 万次查表评分/帧
```

相比完整 360 度搜索和更密集激光点，计算量明显降低。若需要更低负载，可增大 `scan_step`、`coarse_xy_step`、`fine_xy_step`，或降低 `localization_update_frequency`。

## 使用方法

构建：

```bash
cd <workspace>
colcon build --packages-select origincar_mapping origincar_navigation origincar_bringup
source install/setup.bash
```

启动完整导航：

```bash
ros2 launch origincar_bringup navigation.launch.py
```

单独启动定位：

```bash
ros2 launch origincar_mapping localization.launch.py
```

查看定位输出：

```bash
ros2 topic echo /laser_pose
ros2 run tf2_ros tf2_echo map odom
```

检查 IMU：

```bash
ros2 topic echo /imu/data
```

若实际机器人没有 IMU，或 IMU yaw 不可信，可在参数中关闭：

```yaml
use_imu_yaw_prior: false
```

## 调参建议

定位不稳定或跳变：

- 增大 `imu_yaw_search_window_deg`，例如 `45.0`。
- 降低 `min_score` 或保持默认 `0.0` 先观察。
- 检查 `/scan` 的 `frame_id` 是否能通过 TF 转到 `base_footprint`。
- 检查 IMU yaw 是否和机器人实际朝向一致。

定位计算太重：

- 增大 `scan_step`，例如 `12` 或 `15`。
- 增大 `coarse_xy_step`，例如 `0.24`。
- 降低 `localization_update_frequency`，例如 `5.0`。

定位精度不够：

- 减小 `fine_xy_step`，例如 `0.03`。
- 减小 `scan_step`，例如 `6`。
- 减小 `coarse_xy_step`，例如 `0.12`。
