#!/usr/bin/env python3

# 打开导航视角 RViz。
# 这个 launch 不启动任何导航节点，只加载 rviz_nav.rviz，便于查看地图、机器人、
# 激光、全局路径和激光定位位姿，并通过工具栏发送 Goal。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # RViz 配置文件随 origincar_navigation 包一起安装。
    nav_pkg = get_package_share_directory('origincar_navigation')
    rviz_config = os.path.join(nav_pkg, 'config', 'rviz_nav.rviz')

    return LaunchDescription([
        # -d 参数指定 RViz 配置，避免每次手动添加显示项。
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_navigation',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
