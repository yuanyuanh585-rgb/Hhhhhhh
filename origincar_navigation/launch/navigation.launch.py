#!/usr/bin/env python3

# 单独启动本项目裁剪后的 Nav2 导航栈。
# 它假设定位和 TF 已经存在，例如：
# - map_server 已经发布 /map；
# - 激光定位节点已经发布 map -> odom；
# - 仿真插件已经发布 odom -> base_footprint。
#
# 为什么不能继续 include nav2_bringup/navigation_launch.py：
# - Humble 官方 launch 会固定拉起 smoother_server、behavior_server、
#   waypoint_follower、velocity_smoother 等整套节点；
# - 当前项目只需要“单目标只规划一次 + 自定义控制器纯跟踪”；
# - 因此要真正减少节点数量，就必须在这里手写最小启动集合。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    nav_pkg = get_package_share_directory('origincar_navigation')

    default_params = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    default_keepout_mask = os.path.join(nav_pkg, 'config', 'local_keepout_mask.yaml')
    default_bt_xml = os.path.join(
        nav_pkg,
        'behavior_trees',
        'navigate_to_pose_fixed_route_once.xml'
    )
    placeholder_through_poses_bt_xml = os.path.join(
        nav_pkg,
        'behavior_trees',
        'navigate_through_poses_placeholder.xml'
    )

    # 启动参数引用。实车默认 false；仿真入口会显式传 true。
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    routes_file = LaunchConfiguration('routes_file')
    controller_id = LaunchConfiguration('controller_id')
    configured_nav_to_pose_bt_xml = ReplaceString(
        source_file=default_bt_xml,
        replacements={'ORIGINCAR_CONTROLLER_ID': controller_id}
    )
    configured_through_poses_bt_xml = ReplaceString(
        source_file=placeholder_through_poses_bt_xml,
        replacements={'ORIGINCAR_CONTROLLER_ID': controller_id}
    )
    configured_params = ReplaceString(
        source_file=params_file,
        replacements={
            '<origincar_nav_to_pose_bt_xml>': configured_nav_to_pose_bt_xml,
            '<origincar_nav_through_poses_bt_xml>': configured_through_poses_bt_xml,
            '<global_routes_file>': routes_file,
            '<local_keepout_mask_file>': default_keepout_mask,
        }
    )
    # 参数文件仍然由本包维护；这里只额外统一覆写 use_sim_time，
    # 保证仿真和实车共享同一份 YAML 时不会把所有节点写死成 false。
    rewritten_params = ParameterFile(
        RewrittenYaml(
            source_file=configured_params,
            param_rewrites={'use_sim_time': use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # planner_server 只负责把目标转换成一次全局固定路径，不参与周期性重规划。
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[rewritten_params],
    )

    # controller_server 只加载本项目自定义控制器和最小 GoalChecker/ProgressChecker。
    # 这里直接输出 /cmd_vel，不再经过 velocity_smoother 的 cmd_vel_nav 链路。
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[rewritten_params],
    )

    # bt_navigator 继续承接 NavigateToPose action，但行为树已裁剪为
    # “新目标时规划一次，然后一直跟踪路径”的最小闭环。
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[rewritten_params],
    )

    # keepout filter 需要一个 mask map_server 和一个 filter info server。
    keepout_filter_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='keepout_filter_mask_server',
        output='screen',
        parameters=[rewritten_params],
    )

    keepout_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='keepout_filter_info_server',
        output='screen',
        parameters=[rewritten_params],
    )

    # lifecycle_manager 这里只管理导航核心节点和 keepout filter 所需的两个辅助节点。
    # smoother / behavior / waypoint / velocity_smoother 已被设计层面裁掉，
    # 因此不能再放进 node_names，否则会等待不存在的节点进入生命周期。
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'keepout_filter_mask_server',
                'keepout_filter_info_server',
                'planner_server',
                'controller_server',
                'bt_navigator',
            ],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # 默认参数见 config/nav2_params.yaml，也可用 params_file:=... 覆盖。
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument(
            'routes_file',
            default_value=os.path.join(nav_pkg, 'config', 'global_routes.yaml')
        ),
        # 控制器 ID 必须来自 nav2_params.yaml 的 controller_plugins。
        # PurePursuit 使用旧纯追踪控制器，MyPlannerFollowPath 使用新控制器。
        # DeclareLaunchArgument('controller_id', default_value='MyPlannerFollowPath'),
        DeclareLaunchArgument('controller_id', default_value='PurePursuit'),
        planner_server,
        controller_server,
        bt_navigator,
        keepout_filter_mask_server,
        keepout_filter_info_server,
        lifecycle_manager,
    ])
