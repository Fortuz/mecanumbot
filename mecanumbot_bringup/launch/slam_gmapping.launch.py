from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    description_prefix = get_package_share_directory('mecanumbot_description')

    cartographer_config_dir = LaunchConfiguration(
        'cartographer_config_dir',
        default=os.path.join(description_prefix, 'param')
    )
    configuration_basename = LaunchConfiguration(
        'configuration_basename',
        default='mecanumbot_lds.lua'
    )

    rviz_config = os.path.join(
        get_package_share_directory('mecanumbot_description'),
        'rviz',
        'gmapping.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('cartographer_config_dir', default_value=cartographer_config_dir),
        DeclareLaunchArgument('configuration_basename', default_value=configuration_basename),

        # CARTOGRAPHER NODE
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            namespace='mecanumbot',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
            ],
            remappings=[
                ('scan', '/mecanumbot/scan'),
                ('imu', '/mecanumbot/imu'),
                ('odom', '/mecanumbot/odom'),
                # NOTE: Global /tf and /tf_static are intentionally NOT remapped here
            ]
        ),

        # RVIZ2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            remappings=[
                ('scan', '/mecanumbot/scan'),
                ('imu', '/mecanumbot/imu'),
                ('odom', '/mecanumbot/odom'),
                # NOTE: Global tf and tf_static NOT remapped
            ],
            arguments=['-d', rviz_config]
        )
    ])