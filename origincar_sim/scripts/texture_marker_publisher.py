#!/usr/bin/env python3

# 这个节点用于把 Gazebo 地面贴图以 RViz Marker 的形式发布出来。
# Gazebo 世界里的地图纹理只能在 Gazebo 中直接看到，RViz 默认只能看到
# /map、TF、LaserScan、RobotModel 等 ROS 数据。为了在 RViz 中也能对照原始地面图片，
# 这里把 PNG 采样成彩色小方块，发布为 visualization_msgs/Marker.CUBE_LIST。
#
# 为什么不用 Marker 的 embedded texture：
# RViz Humble 在某些图形后端下同时显示 Map 和“内嵌纹理 Marker”时，可能触发
# GLSL sampler 冲突并导致 RViz 崩溃。CUBE_LIST 不走纹理采样器，稳定性更好。
import ast
from pathlib import Path

import rclpy
from PIL import Image
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker


class TextureMarkerPublisher(Node):
    def __init__(self):
        # 节点名会出现在 ros2 node list 中，便于确认 RViz 纹理发布器是否启动。
        super().__init__('smart_car_map_texture_publisher')

        # 参数都由 launch 文件传入，保留默认值是为了节点单独运行时也有清晰的行为。
        # image_path: 要嵌入 Marker 的 PNG 文件路径。
        # frame_id: Marker 所属坐标系，通常是 map，这样它和地图/导航坐标一致。
        # topic: Marker 发布话题，RViz 的 Marker Display 会订阅它。
        # map_yaml_path: 可选的 Nav2 地图 YAML；提供后会自动按真实 origin/resolution 对齐。
        # width/height: 彩色栅格在 RViz 中占用的实际尺寸，单位是米。
        # z: 栅格离地高度，略高于 0 可避免和地图/网格重叠闪烁。
        # alpha: 透明度，便于同时观察地图、机器人和激光。
        # cells_x/cells_y: 采样后的栅格数量。<=0 时优先跟随 map.yaml 对应的栅格尺寸。
        # rotate_clockwise_90: 是否把原图顺时针旋转 90 度后再采样，保留旧参数兼容。
        # rotate_counterclockwise_90: 是否把发布到 Marker 话题的点坐标逆时针旋转 90 度。
        self.declare_parameter('image_path', '')
        self.declare_parameter('map_yaml_path', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic', '/smart_car_map_texture')
        self.declare_parameter('width', 5.0)
        self.declare_parameter('height', 5.0)
        self.declare_parameter('z', -0.02)
        self.declare_parameter('alpha', 0.55)
        self.declare_parameter('cells_x', 0)
        self.declare_parameter('cells_y', 0)
        self.declare_parameter('rotate_clockwise_90', True)
        self.declare_parameter('rotate_counterclockwise_90', True)

        self.image_path = Path(self.get_parameter('image_path').value)
        self.map_yaml_path = Path(self.get_parameter('map_yaml_path').value)
        self.frame_id = self.get_parameter('frame_id').value
        topic = self.get_parameter('topic').value
        self.z = float(self.get_parameter('z').value)
        self.alpha = float(self.get_parameter('alpha').value)
        requested_cells_x = int(self.get_parameter('cells_x').value)
        requested_cells_y = int(self.get_parameter('cells_y').value)
        self.rotate_clockwise_90 = bool(self.get_parameter('rotate_clockwise_90').value)
        self.rotate_counterclockwise_90 = bool(
            self.get_parameter('rotate_counterclockwise_90').value
        )
        self.map_geometry = self._resolve_map_geometry()
        if self.map_geometry is not None:
            self.width = self.map_geometry['width']
            self.height = self.map_geometry['height']
            self.center_x = self.map_geometry['center_x']
            self.center_y = self.map_geometry['center_y']
        else:
            self.width = float(self.get_parameter('width').value)
            self.height = float(self.get_parameter('height').value)
            self.center_x = 0.0
            self.center_y = 0.0
        self.cells_x = self._resolve_sampling_cells(
            requested_cells_x,
            self.map_geometry['cells_x'] if self.map_geometry is not None else None,
        )
        self.cells_y = self._resolve_sampling_cells(
            requested_cells_y,
            self.map_geometry['cells_y'] if self.map_geometry is not None else None,
        )

        # TRANSIENT_LOCAL 相当于 ROS 1 的 latched topic：RViz 后启动时也能立刻收到
        # 最近一次 Marker，不必等待下一次发布。
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(Marker, topic, qos)

        # Marker 内容本身不变，只需要定期刷新时间戳并重发。这样 RViz 重新订阅或
        # Fixed Frame 切换时也更容易保持显示。
        self.marker = self._make_marker()
        self.timer = self.create_timer(1.0, self.publish_marker)
        self.publish_marker()

    def _resolve_sampling_cells(self, requested_cells, map_cells):
        if requested_cells > 0:
            return requested_cells
        if map_cells is not None:
            return max(1, int(map_cells))
        return 169

    def _resolve_map_geometry(self):
        map_yaml_path = self._find_map_yaml_path()
        if map_yaml_path is None:
            self.get_logger().info(
                'No map.yaml found for texture marker; using width/height centered at map origin.'
            )
            return None

        metadata = self._load_map_yaml(map_yaml_path)
        self.map_image_path = metadata['image_path']
        map_image = Image.open(metadata['image_path'])
        cells_x, cells_y = map_image.size
        width = cells_x * metadata['resolution']
        height = cells_y * metadata['resolution']
        origin_x, origin_y = metadata['origin'][:2]

        self.get_logger().info(
            'Using map geometry from %s: origin=(%.3f, %.3f), resolution=%.3f, size=%dx%d'
            % (
                str(map_yaml_path),
                origin_x,
                origin_y,
                metadata['resolution'],
                cells_x,
                cells_y,
            )
        )
        return {
            'width': width,
            'height': height,
            'cells_x': cells_x,
            'cells_y': cells_y,
            'center_x': origin_x + width * 0.5,
            'center_y': origin_y + height * 0.5,
        }

    def _find_map_yaml_path(self):
        if str(self.map_yaml_path):
            if not self.map_yaml_path.exists():
                raise FileNotFoundError(f'Map YAML not found: {self.map_yaml_path}')
            return self.map_yaml_path

        candidate = self.image_path.with_name('map.yaml')
        if candidate.exists():
            return candidate
        return None

    def _load_map_yaml(self, map_yaml_path):
        values = {}
        for raw_line in map_yaml_path.read_text(encoding='utf-8').splitlines():
            line = raw_line.split('#', 1)[0].strip()
            if not line or ':' not in line:
                continue
            key, value = line.split(':', 1)
            values[key.strip()] = value.strip()

        try:
            image_name = values['image']
            resolution = float(values['resolution'])
            origin = ast.literal_eval(values['origin'])
        except (KeyError, SyntaxError, ValueError) as exc:
            raise ValueError(f'Failed to parse map YAML: {map_yaml_path}') from exc

        image_path = (map_yaml_path.parent / image_name).resolve()
        if not image_path.exists():
            raise FileNotFoundError(f'Map image referenced by YAML not found: {image_path}')

        if not isinstance(origin, (list, tuple)) or len(origin) < 2:
            raise ValueError(f'Invalid origin in map YAML: {map_yaml_path}')

        return {
            'image_path': image_path,
            'resolution': resolution,
            'origin': origin,
        }

    def _make_marker(self):
        # 启动时检查图片是否存在，路径错误时直接报错，避免 RViz 中悄悄什么都不显示。
        if not self.image_path.exists():
            raise FileNotFoundError(f'Texture image not found: {self.image_path}')

        if (
            hasattr(self, 'map_image_path')
            and self.image_path.resolve() == self.map_image_path.resolve()
        ):
            raise ValueError(
                f'image_path points to the occupancy map {self.image_path}. '
                'Use a real texture image such as smart_car_map.png for image_path, '
                'and pass the occupancy map YAML with map_yaml_path.'
            )

        # 读取图片后先旋转，再缩放到目标栅格大小。ROTATE_270 等价于顺时针 90 度。
        # Pillow 新旧版本的旋转常量位置不同，这里用旧版也支持的 Image.ROTATE_270。
        image = Image.open(self.image_path)
        if image.mode == 'L' or self.image_path.suffix.lower() in ('.pgm', '.pbm'):
            self.get_logger().warn(
                'Texture image is grayscale. If RViz shows gray stripes, set image_path '
                'to the colored smart_car_map.png and keep map_yaml_path pointing to map.yaml.'
            )
        image = image.convert('RGB')
        if self.rotate_clockwise_90:
            image = image.transpose(Image.ROTATE_270)
        image = image.resize((self.cells_x, self.cells_y), Image.BILINEAR)

        half_w = self.width * 0.5
        half_h = self.height * 0.5
        cell_w = self.width / self.cells_x
        cell_h = self.height / self.cells_y

        # CUBE_LIST 中每个 point 是一个小方块中心，scale 是每个小方块的尺寸。
        # 使用很薄的 z 尺寸，把它当作贴在 XY 平面上的彩色栅格。
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = 'smart_car_map_texture'
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = cell_h if self.rotate_counterclockwise_90 else cell_w
        marker.scale.y = cell_w if self.rotate_counterclockwise_90 else cell_h
        marker.scale.z = 0.002
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        # frame_locked=true 表示 RViz 每帧按 TF 重新计算 Marker 位置，适合固定在 map 上。
        marker.frame_locked = True

        marker.points = []
        marker.colors = []
        pixels = image.load()
        for row in range(self.cells_y):
            # 图片 row=0 是顶部；映射到 map 平面时，需要从地图中心往上半区开始铺。
            rel_y = half_h - (row + 0.5) * cell_h
            for col in range(self.cells_x):
                rel_x = -half_w + (col + 0.5) * cell_w
                if self.rotate_counterclockwise_90:
                    x = self.center_x - rel_y
                    y_marker = self.center_y + rel_x
                else:
                    x = self.center_x + rel_x
                    y_marker = self.center_y + rel_y
                r, g, b = pixels[col, row]
                marker.points.append(Point(x=x, y=y_marker, z=self.z))
                marker.colors.append(ColorRGBA(
                    r=r / 255.0,
                    g=g / 255.0,
                    b=b / 255.0,
                    a=self.alpha,
                ))
        return marker

    def publish_marker(self):
        # 更新时间戳后发布。Marker 几何和纹理不变，所以不用每次重新构造。
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.marker)


def main():
    # 标准 rclpy 节点生命周期：初始化 -> 创建节点 -> spin -> 销毁节点 -> shutdown。
    rclpy.init()
    node = TextureMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
