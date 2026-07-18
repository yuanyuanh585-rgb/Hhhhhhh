#!/usr/bin/env python3

# Gazebo Fortress 仿真主启动文件。
# 负责设置 Gazebo Sim 资源/插件路径、启动 world、发布 robot_description、
# 插入机器人实体，并把 /clock、/scan、/imu/data 桥接到 ROS 2。
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _first_available_package(*names):
    for name in names:
        try:
            return name, get_package_share_directory(name)
        except PackageNotFoundError:
            continue
    joined = ', '.join(names)
    raise PackageNotFoundError(f'None of these Gazebo ROS packages were found: {joined}')


def generate_launch_description():
    pkg_share = get_package_share_directory('origincar_sim')
    sim_pkg, _ = _first_available_package('ros_gz_sim', 'ros_ign_gazebo')
    bridge_pkg, _ = _first_available_package('ros_gz_bridge', 'ros_ign_bridge')

    world_path = os.path.join(pkg_share, 'worlds', '5m_map_world.world')
    model_path = os.path.join(pkg_share, 'urdf', 'origincar_ackermann.urdf.xacro')
    generated_model_path = '/tmp/origincar_ackermann_fortress.urdf'
    model_dir = os.path.join(pkg_share, 'models')
    plugin_dir = os.path.join(os.path.dirname(os.path.dirname(pkg_share)), 'lib')

    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x')
    y_pose = LaunchConfiguration('y')
    z_pose = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    robot_description_command = Command(['xacro ', model_path])
    robot_description = ParameterValue(robot_description_command, value_type=str)

    gazebo = ExecuteProcess(
        cmd=[
            'ign', 'gazebo',
            '--render-engine-gui', 'ogre2',
            '--render-engine-server', 'ogre2',
            '-r', world,
        ],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    generate_robot_urdf = ExecuteProcess(
        cmd=['xacro', model_path, '-o', generated_model_path],
        output='screen'
    )

    spawn_robot = Node(
        package=sim_pkg,
        executable='create',
        arguments=[
            '-name', 'origincar_ackermann',
            '-file', generated_model_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw,
        ],
        output='screen'
    )

    parameter_bridge = Node(
        package=bridge_pkg,
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world_path),
        DeclareLaunchArgument('x', default_value='-2.25'),
        DeclareLaunchArgument('y', default_value='-2.25'),
        DeclareLaunchArgument('z', default_value='0.04'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[model_dir, ':', os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')]
        ),
        SetEnvironmentVariable(
            name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
            value=[plugin_dir, ':', os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')]
        ),
        gazebo,
        generate_robot_urdf,
        robot_state_publisher,
        parameter_bridge,
        TimerAction(period=2.0, actions=[spawn_robot]),
    ])
