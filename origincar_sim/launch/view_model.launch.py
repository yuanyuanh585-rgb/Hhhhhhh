#!/usr/bin/env python3

# RViz 纯模型视图入口。
# 这个 launch 文件也只负责启动 RViz，不发布机器人描述或关节状态。
# 若要单独查看 URDF/xacro 模型，请先启动 robot_state.launch.py，再启动本文件。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 模型视图配置使用 base_link 作为 Fixed Frame，适合检查车体、轮子、
    # 雷达、IMU 等 URDF 链路外观。
    pkg_share = get_package_share_directory('origincar_sim')
    rviz_config = os.path.join(pkg_share, 'rviz', 'origincar_model.rviz')

    return LaunchDescription([
        # 只启动 RViz。/robot_description 和 /joint_states 由 robot_state.launch.py 提供。
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_origincar_model',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
