#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg = get_package_share_directory('origincar_navigation')
    default_routes_file = os.path.join(nav_pkg, 'config', 'global_routes.yaml')

    routes_file = LaunchConfiguration('routes_file')
    frame_id = LaunchConfiguration('frame_id')
    marker_topic = LaunchConfiguration('marker_topic')
    publish_period_ms = LaunchConfiguration('publish_period_ms')

    return LaunchDescription([
        DeclareLaunchArgument('routes_file', default_value=default_routes_file),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('marker_topic', default_value='global_route_markers'),
        DeclareLaunchArgument('publish_period_ms', default_value='1000'),
        Node(
            package='origincar_navigation',
            executable='route_markers_publisher',
            name='route_markers_publisher',
            output='screen',
            parameters=[{
                'routes_file': routes_file,
                'frame_id': frame_id,
                'marker_topic': marker_topic,
                'publish_period_ms': publish_period_ms,
            }],
        ),
    ])
