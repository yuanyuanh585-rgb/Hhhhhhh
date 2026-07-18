#!/usr/bin/env python3

# RViz 仿真/地图视图入口。
# 这个 launch 文件只负责启动 RViz，不发布 /robot_description、/joint_states、
# /smart_car_map_texture，也不做任何 Gazebo 或 Nav2 控制任务。
# 需要哪些后台数据，由其他 launch 明确启动：
# - sim_world.launch.py / bringup navigation.launch.py：提供仿真、TF、/robot_description；
# - robot_state.launch.py：单独查看模型时提供机器人状态；
# - texture_marker.launch.py：需要叠加 Gazebo 原始地面纹理时提供 Marker。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 本入口固定加载仿真地图视图配置。这个配置使用 map 作为 Fixed Frame，
    # 适合连接正在运行的 Gazebo/SLAM/Nav2 系统观察地图、小车、TF 和导航状态。
    pkg_share = get_package_share_directory('origincar_sim')
    rviz_config = os.path.join(pkg_share, 'rviz', 'origincar_sim.rviz')

    return LaunchDescription([
        # 只启动 RViz 进程。所有话题和 TF 都必须由外部系统提供。
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_origincar_sim',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
