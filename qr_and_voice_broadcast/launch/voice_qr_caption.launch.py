#!/usr/bin/env python3
"""一键启动：二维码解码 + 语音播报 + 图生文"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    baudrate = LaunchConfiguration("baudrate")
    volume = LaunchConfiguration("volume")

    return LaunchDescription([
        # ============ 语音参数 ============
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyS1",
                              description="VTX316 语音模块串口"),
        DeclareLaunchArgument("baudrate", default_value="115200",
                              description="串口波特率"),
        DeclareLaunchArgument("volume", default_value="10",
                              description="语音音量 (0-10)"),

        # ============ 二维码解码节点 ============
        Node(
            package="qr_and_voice_broadcast",
            executable="qr_decoder",
            name="qr_decoder",
            output="screen",
        ),

        # ============ 语音播报节点（含图生文触发）============
        Node(
            package="qr_and_voice_broadcast",
            executable="speak_result.py",
            name="speak_result",
            output="screen",
            parameters=[{
                "serial_port": serial_port,
                "baudrate": baudrate,
                "volume": volume,
            }],
        ),

        # ============ 图生文推理节点 ============
        Node(
            package="qr_and_voice_broadcast",
            executable="image_to_string.py",
            name="image_to_string",
            output="screen",
        ),
    ])
