#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_pkg = get_package_share_directory('origincar_bringup')
    sim_pkg = get_package_share_directory('origincar_sim')
    mapping_pkg = get_package_share_directory('origincar_mapping')
    nav_pkg = get_package_share_directory('origincar_navigation')
    strategy_pkg = get_package_share_directory('origincar_strategy')

    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(bringup_pkg))))

    map_file = LaunchConfiguration('map')
    routes_file = LaunchConfiguration('routes_file')
    params_file = LaunchConfiguration('params_file')
    strategy_params_file = LaunchConfiguration('strategy_params_file')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg, 'launch', 'sim_world.launch.py')
        )
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mapping_pkg, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true',
            'params_file': params_file,
        }.items()
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': params_file,
            'routes_file': routes_file,
        }.items()
    )
    strategy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(strategy_pkg, 'launch', 'route_strategy.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': strategy_params_file,
            'routes_file': routes_file,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(workspace_root, 'maps', 'sim', 'map.yaml')),
        DeclareLaunchArgument(
            'routes_file',
            default_value=os.path.join(workspace_root, 'maps', 'sim', 'global_routes.yaml')
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
        ),
        DeclareLaunchArgument(
            'strategy_params_file',
            default_value=os.path.join(strategy_pkg, 'config', 'strategy_params.yaml')
        ),
        sim,
        TimerAction(period=4.0, actions=[localization]),
        TimerAction(period=10.0, actions=[navigation]),
        TimerAction(period=11.0, actions=[strategy]),
    ])