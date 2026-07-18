#!/usr/bin/env python3

# Dynamically insert a traffic cone into a running Gazebo Fortress world.
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _first_available_package(*names):
    for name in names:
        try:
            return name, get_package_share_directory(name)
        except PackageNotFoundError:
            continue
    raise PackageNotFoundError('No Gazebo create package found')


def generate_launch_description():
    pkg_share = get_package_share_directory('origincar_sim')
    sim_pkg, _ = _first_available_package('ros_gz_sim', 'ros_ign_gazebo')
    model_dir = os.path.join(pkg_share, 'models')
    model_path = os.path.join(model_dir, 'traffic_cone', 'model.sdf')

    name = LaunchConfiguration('name')
    x_pose = LaunchConfiguration('x')
    y_pose = LaunchConfiguration('y')
    z_pose = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    spawn_cone = Node(
        package=sim_pkg,
        executable='create',
        arguments=[
            '-name', name,
            '-file', model_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw,
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='traffic_cone'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[model_dir, ':', os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')],
        ),
        spawn_cone,
    ])
