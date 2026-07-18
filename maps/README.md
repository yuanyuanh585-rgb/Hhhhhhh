# Maps

本目录按场景保存地图和路线点。路线点与地图坐标强绑定，应成套维护。

```text
sim/
  map.yaml
  map.pgm
  global_routes.yaml

real/
  map.yaml
  map.pgm
  global_routes.yaml
```

实车保存地图默认写入：

```bash
ros2 launch origincar_bringup real_save_map.launch.py
```

保存实车地图后，需要按新地图重新采集 `maps/real/global_routes.yaml`。
