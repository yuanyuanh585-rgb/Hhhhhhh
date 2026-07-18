#!/usr/bin/env python3

# 定位启动入口。
# 本文件启动 map_server 和纯激光预测定位节点，用已有地图完成机器人定位。
# 注意：map_server 是 lifecycle node，所以还会启动一个
# localization_lifecycle_bringup.py 脚本来配置并激活它。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = get_package_share_directory('origincar_navigation')
    mapping_pkg = get_package_share_directory('origincar_mapping')
    default_map = os.path.join(nav_pkg, 'maps', 'sim_5m_map.yaml')

    # 可由命令行覆盖的启动参数：
    # map: 地图 yaml 文件；use_sim_time: 是否使用 Gazebo /clock；
    # params_file: 激光预测定位参数文件。
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    # map_server 读取 yaml + pgm 地图，并发布 /map。
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file,
        }],
    )

    # 纯激光预测定位使用 /scan、/map 和 TF 估计 map -> odom 变换。
    laser_predictive_localizer = Node(
        package='origincar_mapping',
        executable='laser_predictive_localizer',
        name='laser_predictive_localizer',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # 纯激光全局定位节点当前不用；如需切回，取消下方注释并在返回列表中替换预测节点。
    # laser_global_localizer = Node(
    #     package='origincar_mapping',
    #     executable='laser_global_localizer',
    #     name='laser_localizer',
    #     output='screen',
    #     parameters=[
    #         os.path.join(nav_pkg, 'config', 'nav2_params.yaml'),
    #         {'use_sim_time': use_sim_time},
    #     ],
    # )

    # 自定义生命周期启动脚本：等待 map_server 服务出现，执行 configure 和 activate。
    lifecycle_bringup = Node(
        package='origincar_mapping',
        executable='localization_lifecycle_bringup.py',
        name='localization_lifecycle_bringup',
        output='screen',
    )

    return LaunchDescription([
        # 默认地图是仿真 5m 场地地图，也可以通过 map:=... 替换为自己保存的地图。
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(mapping_pkg, 'config', 'laser_predictive_localizer.yaml')
        ),
        map_server,
        laser_predictive_localizer,
        # 给 map_server 留出创建 lifecycle 服务的时间。
        TimerAction(period=3.0, actions=[lifecycle_bringup]),
    ])
