#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('origincar_navigation_test_tools')
    rviz_config = os.path.join(pkg, 'rviz', 'offline_local_avoidance_test.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('start_x', default_value='-2.0'),
        DeclareLaunchArgument('start_y', default_value='-2.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.785398'),
        DeclareLaunchArgument('goal_x', default_value='2.0'),
        DeclareLaunchArgument('goal_y', default_value='2.0'),
        DeclareLaunchArgument('goal_yaw', default_value='0.785398'),
        DeclareLaunchArgument('map_width', default_value='6.0'),
        DeclareLaunchArgument('map_height', default_value='6.0'),
        DeclareLaunchArgument('map_resolution', default_value='0.05'),
        DeclareLaunchArgument('current_speed', default_value='0.30'),
        Node(
            package='origincar_navigation_test_tools',
            executable='local_avoidance_manual_test_node',
            name='local_avoidance_manual_test',
            output='screen',
            parameters=[{
                'frame_id': LaunchConfiguration('frame_id'),
                'use_tf_pose': False,
                'use_nav2_plan': False,
                'default_start_x': LaunchConfiguration('start_x'),
                'default_start_y': LaunchConfiguration('start_y'),
                'default_start_yaw': LaunchConfiguration('start_yaw'),
                'default_goal_x': LaunchConfiguration('goal_x'),
                'default_goal_y': LaunchConfiguration('goal_y'),
                'default_goal_yaw': LaunchConfiguration('goal_yaw'),
                'map_width': LaunchConfiguration('map_width'),
                'map_height': LaunchConfiguration('map_height'),
                'map_resolution': LaunchConfiguration('map_resolution'),
                'current_speed': LaunchConfiguration('current_speed'),
                'local_target_time': 4.0,
                'min_local_target_distance': 1.4,
                'max_local_target_distance': 2.5,
                'astar_allow_unknown': True,
                'astar_global_path_bias': 0.18,
                'astar_previous_path_bias': 0.25,
                'astar_grid_downsample_factor': 1,
                'bspline_enabled': True,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_local_avoidance_test',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
