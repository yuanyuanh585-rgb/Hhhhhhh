#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_pkg = get_package_share_directory('origincar_bringup')
    nav_pkg = get_package_share_directory('origincar_navigation')
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(bringup_pkg))))
    routes_file = LaunchConfiguration('routes_file')
    use_route_markers = LaunchConfiguration('use_route_markers')

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'view_navigation.launch.py')
        )
    )
    route_markers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'route_markers.launch.py')
        ),
        launch_arguments={'routes_file': routes_file}.items(),
        condition=IfCondition(use_route_markers)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'routes_file',
            default_value=os.path.join(workspace_root, 'maps', 'sim', 'global_routes.yaml')
        ),
        DeclareLaunchArgument('use_route_markers', default_value='true'),
        rviz,
        route_markers,
    ])