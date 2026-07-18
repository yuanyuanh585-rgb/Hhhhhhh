#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    map_name = LaunchConfiguration('map_name')
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
        get_package_share_directory('origincar_bringup')
    ))))

    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value=os.path.join(workspace_root, 'maps', 'real', 'map')),
        ExecuteProcess(
            cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_name],
            output='screen',
        ),
    ])