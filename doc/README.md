# Origincar 文档索引

本目录保存 Origincar 仿真、建图、定位和导航的说明文档。当前推荐优先阅读 `usage.md`，它覆盖统一启动入口和常用排查命令。

## 文档列表

- `usage.md`：统一使用说明，包含构建、仿真、建图、导航、RViz 和常见问题。
- `mapping_guide.md`：在线 SLAM 建图和保存地图的操作步骤。
- `navigation_usage.md`：Nav2 定位导航、设置初始位姿、发送目标点和排查流程。
- `小车实车部署操作手册.md`：真实小车部署、建图、导航和排查步骤。
- `navigation_plan.md`：建图与导航阶段规划，以及后续自定义规划器/控制器计划。
- `plan.md`：早期仿真搭建计划，保留作为历史记录。

## 当前工程概览

- `origincar_sim`：Gazebo 世界、阿克曼小车 URDF、传感器、运动学插件和 RViz 配置。
- `origincar_mapping`：SLAM Toolbox、地图服务器、AMCL 定位入口和地图保存入口。
- `origincar_navigation`：Nav2 参数、自定义规划/控制插件、路线可视化和导航 RViz 配置。
- `origincar_strategy`：全局路线策略控制器，负责按任务状态机调度 Nav2。
- `origincar_bringup`：统一启动入口。

实车 TF 主链路为：

```text
map -> odom_combined -> base_footprint -> base_link -> laser_link
```

导航参数文件为 `src/origincar_navigation/config/nav2_params.yaml`；策略参数文件为 `src/origincar_strategy/config/strategy_params.yaml`。地图和路线点按场景成套保存到 `maps/sim/` 与 `maps/real/`，实车默认使用系统时间，仿真入口会切换到 Gazebo `/clock`。
