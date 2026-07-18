# Maps

本目录保留旧仿真地图副本，用于包内示例和兼容默认值。当前 bringup 默认使用工作区根目录下的场景地图：

```text
/home/xjl/digua/maps/sim/map.yaml
/home/xjl/digua/maps/sim/global_routes.yaml
```

重新生成实车地图时，优先使用：

```bash
ros2 launch origincar_bringup real_save_map.launch.py
```
