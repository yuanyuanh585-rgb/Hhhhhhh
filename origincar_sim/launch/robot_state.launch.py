#!/usr/bin/env python3

# 机器人描述和关节状态发布入口。
# 这个文件专门负责把 origincar_ackermann.urdf.xacro 展开为 robot_description，
# 并启动 robot_state_publisher / joint_state_publisher。
# 它不启动 RViz，也不启动 Gazebo；常用于单独查看模型时给 RViz 提供 TF 和关节状态。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # xacro 模型路径。
    pkg_share = get_package_share_directory('origincar_sim')
    model_path = os.path.join(pkg_share, 'urdf', 'origincar_ackermann.urdf.xacro')

    # 单独模型查看通常不需要仿真时间；如果要接入 Gazebo，也可以通过 use_sim_time:=true 覆盖。
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'),
        value_type=bool
    )

    # robot_state_publisher 需要 robot_description 字符串。
    robot_description = ParameterValue(
        Command(['xacro ', model_path]),
        value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock when this robot state publisher is attached to a running simulation.'
        ),
        # 发布固定关节 TF，并根据 /joint_states 发布非固定关节 TF。
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),
        # 为前轮转向关节和车轮连续关节提供默认关节值。
        # 这样在没有 Gazebo 的情况下，RViz RobotModel 也能看到完整 TF 树。
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),
    ])
