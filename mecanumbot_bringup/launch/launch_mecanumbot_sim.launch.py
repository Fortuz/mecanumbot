from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    model_path = LaunchConfiguration('model_path')
    subject_source = LaunchConfiguration('subject_source')
    scenario_path = LaunchConfiguration('scenario_path')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='mecanumbot',
        description='Robot namespace',
    )

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock',
    )

    declare_model_path = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(
            get_package_share_directory('mecanumbot_description'),
            'mujoco',
            'mecanumbot.xml',
        ),
        description='Path to the MuJoCo MJCF model',
    )

    declare_scenario_path = DeclareLaunchArgument(
        'scenario_path',
        default_value=os.path.join(
            get_package_share_directory('mecanumbot_bringup'),
            'config',
            'sim_scenarios',
            'single_human_follow.yaml',
        ),
        description='Path to the simulation scenario YAML file',
    )

    declare_subject_source = DeclareLaunchArgument(
        'subject_source',
        default_value='none',
        description='How subject_pose should be sourced: none or oracle',
    )

    yaml_file = os.path.join(
        get_package_share_directory('mecanumbot_core'),
        'param',
        'mecanumbot_core_parameters.yaml',
    )

    urdf_path = os.path.join(
        get_package_share_directory('mecanumbot_description'),
        'urdf',
        'mecanumbot.urdf',
    )
    robot_description = Command([
        'xacro ',
        urdf_path,
        ' namespace:=',
        namespace,
    ])

    return LaunchDescription([
        declare_namespace,
        declare_sim_time,
        declare_model_path,
        declare_scenario_path,
        declare_subject_source,
        Node(
            package='mecanumbot_core',
            executable='mecanumbot_sim_io_node',
            name='mecanumbot_sim_io_node',
            namespace=namespace,
            parameters=[yaml_file, {'use_sim_time': False, 'model_path': model_path, 'scenario_path': scenario_path, 'publish_clock': True, 'show_viewer': True}],
            remappings=[('/mecanumbot/cmd_vel', '/cmd_vel'), ('/mecanumbot/cmd_accessory_pos', '/cmd_accessory_pos')],
            output='screen',
        ),
        Node(
            package='mecanumbot_core',
            executable='mecanumbot_sensorproc_node',
            name='mecanumbot_sensorproc_node',
            namespace=namespace,
            parameters=[yaml_file, {
                'use_sim_time': use_sim_time,
                'namespace': namespace,
                'use_state_stamp_for_dt': True,
                'require_state_stamp': True,
            }],
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': use_sim_time},
            ],
            remappings=[('/joint_states', '/mecanumbot/joint_states')],
        ),
        Node(
            package='mecanumbot_core',
            executable='mecanumbot_sim_oracle_subject_node',
            name='mecanumbot_sim_oracle_subject_node',
            namespace=namespace,
            output='screen',
            condition=IfCondition(PythonExpression(["'", subject_source, "' == 'oracle'"])),
        ),
    ])
