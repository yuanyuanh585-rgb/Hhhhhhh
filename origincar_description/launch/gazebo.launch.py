#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    description_dir = get_package_share_directory("origincar_description")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    default_model_path = os.path.join(description_dir, "urdf", "origincar.urdf")
    gazebo_launch_path = os.path.join(gazebo_ros_dir, "launch", "gazebo.launch.py")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=default_model_path,
        description="URDF or xacro model file to load into robot_description.",
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "origincar",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0.05",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            model_arg,
            LogInfo(msg="Starting Gazebo Classic and spawning origincar from robot_description."),
            robot_state_publisher,
            joint_state_publisher,
            gazebo,
            spawn_entity,
        ]
    )
