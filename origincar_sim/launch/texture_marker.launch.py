#!/usr/bin/env python3

# Gazebo 地面纹理 Marker 发布入口。
# 这个文件只启动 texture_marker_publisher.py，把 smart_car_map.png 采样成彩色栅格 Marker
# 发布到 /smart_car_map_texture。是否在 RViz 中显示，由 RViz 配置或用户手动开关决定。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 默认使用 Gazebo 场地模型里的 PNG 贴图。
    pkg_share = get_package_share_directory('origincar_sim')
    default_texture_path = os.path.join(
        pkg_share,
        'models',
        '5m_map_arena',
        'materials',
        'textures',
        'smart_car_map.png'
    )
    default_map_yaml_path = '/root/digua/maps/real/map.yaml'

    image_path = LaunchConfiguration('image_path')
    map_yaml_path = LaunchConfiguration('map_yaml_path')
    frame_id = LaunchConfiguration('frame_id')
    topic = LaunchConfiguration('topic')
    width = ParameterValue(LaunchConfiguration('width'), value_type=float)
    height = ParameterValue(LaunchConfiguration('height'), value_type=float)
    z = ParameterValue(LaunchConfiguration('z'), value_type=float)
    alpha = ParameterValue(LaunchConfiguration('alpha'), value_type=float)
    cells_x = ParameterValue(LaunchConfiguration('cells_x'), value_type=int)
    cells_y = ParameterValue(LaunchConfiguration('cells_y'), value_type=int)
    rotate_clockwise_90 = ParameterValue(
        LaunchConfiguration('rotate_clockwise_90'),
        value_type=bool
    )
    rotate_counterclockwise_90 = ParameterValue(
        LaunchConfiguration('rotate_counterclockwise_90'),
        value_type=bool
    )

    return LaunchDescription([
        DeclareLaunchArgument('image_path', default_value=default_texture_path),
        DeclareLaunchArgument('map_yaml_path', default_value=default_map_yaml_path),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('topic', default_value='/smart_car_map_texture'),
        DeclareLaunchArgument('width', default_value='5.0'),
        DeclareLaunchArgument('height', default_value='5.0'),
        DeclareLaunchArgument('z', default_value='-0.02'),
        DeclareLaunchArgument('alpha', default_value='0.55'),
        DeclareLaunchArgument('cells_x', default_value='0'),
        DeclareLaunchArgument('cells_y', default_value='0'),
        DeclareLaunchArgument('rotate_clockwise_90', default_value='true'),
        DeclareLaunchArgument('rotate_counterclockwise_90', default_value='true'),
        # 只发布纹理 Marker，不启动 RViz。
        Node(
            package='origincar_sim',
            executable='texture_marker_publisher.py',
            name='smart_car_map_texture_publisher',
            parameters=[{
                'image_path': image_path,
                'map_yaml_path': map_yaml_path,
                'frame_id': frame_id,
                'topic': topic,
                'width': width,
                'height': height,
                'z': z,
                'alpha': alpha,
                'cells_x': cells_x,
                'cells_y': cells_y,
                'rotate_clockwise_90': rotate_clockwise_90,
                'rotate_counterclockwise_90': rotate_counterclockwise_90,
            }],
            output='screen',
        ),
    ])
