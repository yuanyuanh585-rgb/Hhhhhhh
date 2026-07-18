#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    strategy_pkg = get_package_share_directory('origincar_strategy')
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(strategy_pkg))))

    params_file = LaunchConfiguration('params_file')
    routes_file = LaunchConfiguration('routes_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    configured_params = ReplaceString(
        source_file=params_file,
        replacements={'<global_routes_file>': routes_file}
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(strategy_pkg, 'config', 'strategy_params.yaml')
        ),
        DeclareLaunchArgument(
            'routes_file',
            default_value=os.path.join(workspace_root, 'maps', 'real', 'global_routes.yaml')
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='origincar_strategy',
            executable='route_strategy_controller',
            name='route_strategy_controller',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
        ),
    ])