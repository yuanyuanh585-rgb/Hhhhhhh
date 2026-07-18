# 路径片段替换式局部避障说明

本文档说明当前唯一保留的局部避障算法：路径片段替换式避障。旧版滚动局部路径算法已经从源码和配置中移除。

## 1. 核心概念

```text
active_route
```

当前跟踪路径。新目标到来时，它初始化为 Nav2 全局规划路径。固定路线行为树不会再额外执行 Nav2 `SmoothPath`，因此无障碍时 `active_route` 应与控制器收到的全局路径一致。小车运行过程中，控制器会持续维护它：删除已经走过的部分，并在发现局部障碍时只替换受阻片段。

```text
blocked_segment
```

当前跟踪路径上被 local costmap 障碍挡住的片段。算法只处理机器人前方 `segment_scan_distance` 范围内最近的一个受阻片段。

```text
bypass_path
```

使用前进-only Hybrid A* 在 local costmap 上搜索得到的绕障路径。搜索状态包含位置和车头朝向，动作只模拟阿克曼小车前进转向，不包含倒车。

```text
smoothed_bypass
```

对 `bypass_path` 做轻量样条/平滑后的绕障片段。它会替换回 `active_route`，成为后续纯追踪实际跟踪的路径。该平滑只作用于绕障片段，不作用于完整全局路径。

## 2. 工作流程

```text
Nav2 全局路径
  |
初始化 active_route
  |
裁剪 active_route 已走过部分
  |
在 active_route 前方扫描障碍
  |
无受阻片段：继续跟踪 active_route
  |
发现 blocked_segment
  |
前后扩展得到替换区间
  |
前进-only Hybrid A* 生成 bypass_path
  |
平滑生成 smoothed_bypass
  |
替换 active_route 中对应片段
  |
纯追踪跟踪新的 active_route
```

注意：障碍消失后，算法不会自动把已经替换过的路径恢复成原始全局路径。这样可以避免路径来回切换。发送新目标时，`active_route` 会重新初始化。

固定路线行为树中已经移除 Nav2 `simple_smoother` 的全局平滑节点。这样可以避免 U 形弯等场景中，完整全局路径被二次平滑后向弯道内侧偏移。当前只允许 `smoothed_bypass` 对局部绕障片段做平滑。

## 3. 平滑过渡

替换路径时不能只把搜索结果硬塞进原路径，否则连接处容易出现急转。

当前处理方式：

- 替换片段前后各扩展一段距离，默认前方 `0.30m`、后方 `0.50m`。
- 平滑时加入原路径前后锚点，让绕障段方向更接近原路径。
- 固定替换区间起点和终点，不移动连接点。
- 替换完成后统一重算路径点 yaw，保证纯追踪前视点方向连续。
- 如果平滑路径碰撞检查失败，则使用原始 Hybrid A* `bypass_path`。
- 如果 Hybrid A* 失败，则不修改 `active_route`，控制器继续跟踪旧路径并限速。

## 4. 参数说明

```yaml
segment_replan_frequency: 3.0
```

片段检测、Hybrid A* 和平滑的运行频率。纯追踪控制仍然高频运行；局部避障建议 `3Hz` 起步。障碍变化快时可提高到 `5Hz`。

```yaml
segment_scan_distance: 2.0
```

只扫描机器人前方多少米的 `active_route`。过短会发现障碍太晚，过长会增加计算量。

```yaml
segment_pre_margin: 0.30
segment_post_margin: 0.50
segment_pre_point_margin: 3
segment_post_point_margin: 5
```

受阻片段前后扩展距离，用于给 Hybrid A* 和平滑留过渡空间。点数扩展会在距离扩展之后再向前后多取几个加密后的路径点，避免障碍只占住少量点时替换片段太短。

```yaml
segment_min_blocked_length: 0.05
segment_min_replace_length: 0.40
segment_prune_search_distance: 1.0
```

过滤太小的障碍命中和太短的替换片段，避免路径被噪声频繁修改。`segment_prune_search_distance` 限制裁剪路径时只在当前跟踪路径前方一段距离内找最近点，避免 U 形弯道中最近点跳到另一侧路径臂。

```yaml
segment_smoothing_enabled: true
segment_smoothing_collision_check: true
segment_sample_interval: 0.05
segment_route_sample_interval: 0.05
```

是否启用绕障路径平滑、平滑后是否做碰撞复查，以及平滑后路径采样间隔。`segment_route_sample_interval` 用于在检测障碍前加密当前跟踪路径，只改变路径点密度，不改变路径几何形状。

## 5. Hybrid A* 参数

Hybrid A* 使用以下基础代价地图参数：

```yaml
astar_allow_unknown: false
astar_lethal_cost: 253
astar_cost_weight: 0.02
astar_grid_downsample_factor: 1
```

其中 `astar_grid_downsample_factor` 可用于降低搜索计算量。默认 `1` 表示使用原始 costmap 分辨率；设为 `2` 时，搜索位置节点数大约减少到四分之一，但仍以原始 costmap 膨胀层判断安全性。

```yaml
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
```

`hybrid_astar_yaw_bins` 控制车头朝向离散精度，越大越细但计算量越高。`hybrid_astar_steering_samples` 控制每个节点展开多少个前进转向动作。当前不展开倒车动作，也没有换挡惩罚。

终点朝向不再作为硬约束，只作为软代价参与排序；否则局部片段很短时，车辆可能为了把车头完全对齐终点 yaw 而前进绕出大半圆。`hybrid_astar_goal_yaw_weight` 控制这个软代价强度，`hybrid_astar_yaw_tolerance` 是软代价死区。

`hybrid_astar_max_path_length_ratio` 限制绕障路径最大长度，`hybrid_astar_search_lateral_margin` 限制搜索只能在起点到终点附近的局部走廊中进行，用来防止局部绕障搜索绕出很大的圆。

## 6. RViz 调试话题

```text
FollowPath/local_plan
```

当前纯追踪实际跟踪的 `active_route`。

无障碍时，它应与控制器收到的全局路径重合；有障碍时，只有被替换的绕障片段会不同。

```text
FollowPath/astar_path
```

当前检测到障碍时，Hybrid A* 生成的 `bypass_path`。话题名保持 `astar_path`，便于沿用已有 RViz 配置。

```text
FollowPath/optimized_local_path
```

平滑后的 `smoothed_bypass`。

```text
FollowPath/target_pose
```

纯追踪当前前视点。

## 7. 使用方式

默认参数已经启用片段替换避障：

修改参数后重新编译并启动导航：

```bash
cd /home/xjl/digua
source install/setup.bash
ros2 launch origincar_navigation navigation.launch.py
```
