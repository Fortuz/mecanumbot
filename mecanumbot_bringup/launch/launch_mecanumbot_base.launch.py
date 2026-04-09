from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')


    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='mecanumbot',
        description='Robot namespace'
    )

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock'
    )

    # YAML config
    yaml_file = os.path.join(
        get_package_share_directory('mecanumbot_core'),
        'param',
        'mecanumbot_core_parameters.yaml'
    )

    # State publisher from another package
    state_publisher_path = os.path.join(
        get_package_share_directory('mecanumbot_bringup'),
        'launch',
        'mecanumbot_state_publisher.launch.py'
    )

    return LaunchDescription([
        declare_namespace,
        declare_sim_time,

        # mecanumbot_core IO node
        Node(
            package='mecanumbot_core',
            executable='mecanumbot_io_node',
            name='mecanumbot_io_node',
            namespace= namespace,
            parameters=[yaml_file, {'use_sim_time': use_sim_time}],
            remappings=[('/mecanumbot/cmd_vel','/cmd_vel'),('/mecanumbot/cmd_accessory_pos','/cmd_accessory_pos') ],
            output='screen'
        ),
        Node(
            package='mecanumbot_core',
            executable='mecanumbot_battery_alert',
            name='mecanumbot_battery_alert',
            namespace= namespace,
            parameters=[yaml_file, {'use_sim_time': use_sim_time}],
            remappings=[('/mecanumbot/cmd_vel','/cmd_vel'),('/mecanumbot/cmd_accessory_pos','/cmd_accessory_pos') ],
            output='screen'
        ),
        # mecanumbot_core Sensor Processing node
        Node(
            package='mecanumbot_core',
            executable='mecanumbot_sensorproc_node',
            name='mecanumbot_sensorproc_node',
            namespace= namespace,
            parameters=[yaml_file, {'use_sim_time': use_sim_time, 'namespace': namespace}],
            #remappings=[('/tf', '/mecanumbot/tf'),('/tf_static', '/mecanumbot/tf_static')],
            output='screen'
        ),

        # LD08 Lidar driver node
        Node(
            package='ld08_driver',
            executable='ld08_driver',
            name='ld08_driver_node',
            namespace= namespace,
            parameters=[
                {'port': '/dev/ttyUSB0'},
                {'frame_id': [namespace, '/base_scan']},
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),

        # State publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(state_publisher_path),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'namespace': namespace
            }.items()
        )
    ])