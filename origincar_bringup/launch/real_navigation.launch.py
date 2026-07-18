#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('origincar_bringup')
    mapping_pkg = get_package_share_directory('origincar_mapping')
    nav_pkg = get_package_share_directory('origincar_navigation')
    strategy_pkg = get_package_share_directory('origincar_strategy')

    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(bringup_pkg))))

    map_file = LaunchConfiguration('map')
    routes_file = LaunchConfiguration('routes_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    localizer_params_file = LaunchConfiguration('localizer_params_file')
    strategy_params_file = LaunchConfiguration('strategy_params_file')
    use_rviz = LaunchConfiguration('use_rviz')

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'real_robot.launch.py')
        )
    )
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
    laser_localizer = Node(
        package='origincar_mapping',
        executable='laser_predictive_localizer',
        name='laser_predictive_localizer',
        output='screen',
        parameters=[localizer_params_file, {'use_sim_time': use_sim_time}],
    )
    lifecycle_bringup = Node(
        package='origincar_mapping',
        executable='localization_lifecycle_bringup.py',
        name='localization_lifecycle_bringup',
        output='screen',
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'routes_file': routes_file,
        }.items()
    )
    strategy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(strategy_pkg, 'launch', 'route_strategy.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': strategy_params_file,
            'routes_file': routes_file,
        }.items()
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'view_navigation.launch.py')
        ),
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(workspace_root, 'maps', 'real', 'map.yaml')),
        DeclareLaunchArgument(
            'routes_file',
            default_value=os.path.join(workspace_root, 'maps', 'real', 'global_routes.yaml')
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
        ),
        DeclareLaunchArgument(
            'localizer_params_file',
            default_value=os.path.join(mapping_pkg, 'config', 'laser_predictive_localizer.yaml')
        ),
        DeclareLaunchArgument(
            'strategy_params_file',
            default_value=os.path.join(strategy_pkg, 'config', 'strategy_params.yaml')
        ),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        robot,
        TimerAction(period=4.0, actions=[
            map_server,
            laser_localizer,
            TimerAction(period=3.0, actions=[lifecycle_bringup]),
        ]),
        TimerAction(period=10.0, actions=[navigation]),
        TimerAction(period=11.0, actions=[strategy]),
        TimerAction(period=11.0, actions=[rviz]),
    ])
