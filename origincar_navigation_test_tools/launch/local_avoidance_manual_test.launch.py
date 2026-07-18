#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('robot_frame', default_value='base_footprint'),
        DeclareLaunchArgument('use_tf_pose', default_value='true'),
        DeclareLaunchArgument('map_width', default_value='6.0'),
        DeclareLaunchArgument('map_height', default_value='6.0'),
        DeclareLaunchArgument('map_resolution', default_value='0.05'),
        DeclareLaunchArgument('obstacle_radius', default_value='0.12'),
        DeclareLaunchArgument('obstacle_inflation_radius', default_value='0.18'),
        DeclareLaunchArgument('publish_frequency', default_value='5.0'),
        DeclareLaunchArgument('current_speed', default_value='0.30'),
        Node(
            package='origincar_navigation_test_tools',
            executable='local_avoidance_manual_test_node',
            name='local_avoidance_manual_test',
            output='screen',
            parameters=[{
                'frame_id': LaunchConfiguration('frame_id'),
                'robot_frame': LaunchConfiguration('robot_frame'),
                'use_tf_pose': LaunchConfiguration('use_tf_pose'),
                'map_width': LaunchConfiguration('map_width'),
                'map_height': LaunchConfiguration('map_height'),
                'map_resolution': LaunchConfiguration('map_resolution'),
                'obstacle_radius': LaunchConfiguration('obstacle_radius'),
                'obstacle_inflation_radius': LaunchConfiguration('obstacle_inflation_radius'),
                'publish_frequency': LaunchConfiguration('publish_frequency'),
                'current_speed': LaunchConfiguration('current_speed'),
            }],
        ),
    ])
