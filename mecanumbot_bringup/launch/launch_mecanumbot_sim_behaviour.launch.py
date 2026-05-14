import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    condition = LaunchConfiguration('condition')
    scenario_path = LaunchConfiguration('scenario_path')
    use_rviz = LaunchConfiguration('use_rviz')
    yaml_path = LaunchConfiguration('yaml_path')

    bringup_share = get_package_share_directory('mecanumbot_bringup')
    behaviour_share = get_package_share_directory('mecanumbot_leading_behaviour')

    truth_twin_launch = os.path.join(
        bringup_share,
        'launch',
        'launch_mecanumbot_truth_twin.launch.py',
    )
    default_scenario = os.path.join(
        bringup_share,
        'config',
        'sim_scenarios',
        'single_human_follow.yaml',
    )
    default_yaml = os.path.join(
        behaviour_share,
        'config',
        'sim_behaviour_setting_constants.yaml',
    )

    common_bt_remappings = [
        ('/mecanumbot/cmd_vel', '/cmd_vel'),
        ('/mecanumbot/cmd_accessory_pos', '/cmd_accessory_pos'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='mecanumbot',
            description='Robot namespace',
        ),
        DeclareLaunchArgument(
            'condition',
            default_value='Doglike',
            description='Behaviour condition: Doglike / Control / LED',
        ),
        DeclareLaunchArgument(
            'scenario_path',
            default_value=default_scenario,
            description='Simulation scenario YAML used by the truth twin',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz with the sim behaviour stack',
        ),
        DeclareLaunchArgument(
            'yaml_path',
            default_value=default_yaml,
            description='Sim-specific behaviour constants YAML',
        ),
        SetEnvironmentVariable(name='YAML_PATH', value=yaml_path),
        SetEnvironmentVariable(name='BEHAVIOUR_YAML_PATH', value=yaml_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(truth_twin_launch),
            launch_arguments={
                'namespace': namespace,
                'scenario_path': scenario_path,
                'use_rviz': use_rviz,
                'enable_behavior_evaluation': 'true',
                'enable_visualization_markers': 'true',
            }.items(),
        ),
        Node(
            package='mecanumbot_core',
            executable='mecanumbot_sim_nav_shim_node',
            name='mecanumbot_sim_nav_shim_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'odom_topic': '/mecanumbot/odom',
                'goal_topic': '/goal_pose',
                'cmd_vel_topic': '/cmd_vel',
                'amcl_pose_topic': '/amcl_pose',
                'nav_status_topic': '/navigate_to_pose/_action/status',
            }],
        ),
        Node(
            package='mecanumbot_leading_behaviour',
            executable='doglike_leading_bt_node',
            name='doglike_leading_bt_node',
            namespace=namespace,
            output='screen',
            remappings=common_bt_remappings,
            parameters=[yaml_path, {'use_sim_time': True}],
            condition=IfCondition(PythonExpression(["'", condition, "' == 'Doglike'"])),
        ),
        Node(
            package='mecanumbot_leading_behaviour',
            executable='control_leading_bt_node',
            name='control_leading_bt_node',
            namespace=namespace,
            output='screen',
            remappings=common_bt_remappings,
            parameters=[yaml_path, {'use_sim_time': True}],
            condition=IfCondition(PythonExpression(["'", condition, "' == 'Control'"])),
        ),
        Node(
            package='mecanumbot_leading_behaviour',
            executable='LED_leading_bt_node',
            name='LED_leading_bt_node',
            namespace=namespace,
            output='screen',
            remappings=common_bt_remappings,
            parameters=[yaml_path, {'use_sim_time': True}],
            condition=IfCondition(PythonExpression(["'", condition, "' == 'LED'"])),
        ),
    ])
