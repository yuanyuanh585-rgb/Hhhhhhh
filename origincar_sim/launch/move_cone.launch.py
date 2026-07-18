#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='origincar_5m_map_world'),
        DeclareLaunchArgument('name', default_value='traffic_cone'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        Node(
            package='origincar_sim',
            executable='move_cone.py',
            arguments=[
                '--world', LaunchConfiguration('world'),
                '--name', LaunchConfiguration('name'),
                '--x', LaunchConfiguration('x'),
                '--y', LaunchConfiguration('y'),
                '--z', LaunchConfiguration('z'),
                '--yaw', LaunchConfiguration('yaw'),
            ],
            output='screen',
        ),
    ])
