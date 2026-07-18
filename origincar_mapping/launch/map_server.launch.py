#!/usr/bin/env python3

# 单独启动地图服务器的入口。
# 适合只想在 RViz 中查看静态地图，或调试地图 yaml/pgm 是否能被 Nav2 正常读取时使用。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = get_package_share_directory('origincar_navigation')
    default_map = os.path.join(nav_pkg, 'maps', 'sim_5m_map.yaml')

    # 地图文件和仿真时间都做成 launch 参数，便于命令行覆盖。
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # Nav2 map_server 接收的是地图 yaml，而不是 pgm 图片本身。
        DeclareLaunchArgument(
            'map',
            default_value=default_map
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # map_server 是 lifecycle node，启动后需要被 configure/activate 才会发布 /map。
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_file,
            }],
        ),
        # 这里使用官方 lifecycle_manager 自动激活 map_server。
        # localization.launch.py 则使用自定义脚本同时管理 map_server 和 amcl。
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server'],
            }],
        ),
    ])
