from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf')

    robot_parameters = [
        {
            'usart_port_name': '/dev/ttyACM0',
            'serial_baud_rate': 115200,
            'robot_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'cmd_vel': 'cmd_vel',
            'product_number': 0,
            'use_sim_time': use_sim_time,
            'publish_odom_tf': publish_odom_tf,
        }
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.'
        ),
        DeclareLaunchArgument(
            'publish_odom_tf',
            default_value='true',
            description='Whether origincar_base publishes odom TF.'
        ),

        launch_ros.actions.Node(
            package='origincar_base',
            executable='origincar_base_node',
            parameters=robot_parameters + [{'akm_cmd_vel': 'none'}],
            remappings=[('/cmd_vel', 'cmd_vel')],
        ),
    ])
