#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_ekf = LaunchConfiguration('enable_ekf')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf')
    base_share = FindPackageShare('origincar_base')
    imu_calib_share = FindPackageShare('imu_calib')
    lidar_share = FindPackageShare('lslidar_driver')
    ekf_config = PathJoinSubstitution([base_share, 'config', 'ekf.yaml'])
    imu_calib_config = PathJoinSubstitution([imu_calib_share, 'imu_calibration.yaml'])
    lidar_config = PathJoinSubstitution([lidar_share, 'params', 'lidar_uart_ros2', 'lsn10.yaml'])

    base_serial = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([base_share, 'launch', 'base_serial.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'publish_odom_tf': publish_odom_tf,
        }.items()
    )

    lidar = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        parameters=[lidar_config, {'use_sim_time': use_sim_time}],
    )

    base_to_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_link',
        arguments=['0.41', '0.12', '0', '0', '0', '0', 'base_footprint', 'base_link'],
    )
    base_to_gyro = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_gyro',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'gyro_link'],
    )
    base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        arguments=['0', '0', '0.18', '0', '0', '0', 'base_footprint', 'laser_link'],
    )

    imu_calibration = Node(
        package='imu_calib',
        executable='apply_calib_node',
        name='imu_apply_calib',
        output='screen',
        parameters=[{
            'calib_file': imu_calib_config,
            'calibrate_gyros': False,
            'queue_size': 10,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('raw', 'imu/data_raw'),
            ('corrected', 'imu/data_calibrated'),
        ],
    )

    imu_filter = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[{
            'gain_acc': 0.01,
            'gain_mag': 0.01,
            'bias_alpha': 0.01,
            'do_bias_estimation': True,
            'do_adaptive_gain': True,
            'use_mag': False,
            'fixed_frame': 'base_footprint',
            'publish_tf': False,
            'reverse_tf': False,
            'constant_dt': 0.0,
            'publish_debug_topics': False,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('imu/data_raw', 'imu/data_calibrated'),
            ('/imu/data_raw', 'imu/data_calibrated'),
        ],
    )

    ekf = Node(
        condition=IfCondition(enable_ekf),
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odom_combined')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_ekf', default_value='true'),
        DeclareLaunchArgument('publish_odom_tf', default_value='false'),
        base_serial,
        base_to_link,
        base_to_gyro,
        base_to_laser,
        lidar,
        imu_calibration,
        imu_filter,
        ekf,
    ])
