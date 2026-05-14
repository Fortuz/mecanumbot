import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    scenario_path = LaunchConfiguration('scenario_path')
    use_rviz = LaunchConfiguration('use_rviz')
    enable_behavior_evaluation = LaunchConfiguration('enable_behavior_evaluation')
    enable_visualization_markers = LaunchConfiguration('enable_visualization_markers')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='mecanumbot',
        description='Robot namespace',
    )

    declare_scenario_path = DeclareLaunchArgument(
        'scenario_path',
        default_value=os.path.join(
            get_package_share_directory('mecanumbot_bringup'),
            'config',
            'sim_scenarios',
            'two_humans_wall_discrimination.yaml',
        ),
        description='Path to the simulation scenario YAML file',
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz together with the truth twin',
    )

    declare_enable_behavior_evaluation = DeclareLaunchArgument(
        'enable_behavior_evaluation',
        default_value='true',
        description='Publish /sim/behavior_evaluation using oracle subject tracking',
    )

    declare_enable_visualization_markers = DeclareLaunchArgument(
        'enable_visualization_markers',
        default_value='true',
        description='Publish clean RViz truth markers without detector overlays',
    )

    bringup_share = get_package_share_directory('mecanumbot_bringup')
    rviz_launch = os.path.join(bringup_share, 'launch', 'rviz2.launch.py')
    sim_launch = os.path.join(bringup_share, 'launch', 'launch_mecanumbot_sim.launch.py')

    return LaunchDescription([
        declare_namespace,
        declare_scenario_path,
        declare_use_rviz,
        declare_enable_behavior_evaluation,
        declare_enable_visualization_markers,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': 'true',
                'subject_source': 'oracle',
                'scenario_path': scenario_path,
            }.items(),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_identity',
            arguments=['0', '0', '0', '0', '0', '0', 'map', [namespace, '/odom']],
            output='screen',
        ),
        Node(
            namespace=namespace,
            package='mecanumbot_core',
            executable='mecanumbot_sim_detection_evaluator_node',
            name='mecanumbot_sim_truth_target_evaluator_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'subject_pose_topic': 'subject_pose',
                'actors_topic': '/sim/actors',
                'evaluation_topic': '/sim/detection_evaluation',
                'require_raw_detection': False,
            }],
        ),
        Node(
            namespace=namespace,
            package='mecanumbot_core',
            executable='mecanumbot_sim_behavior_evaluator_node',
            name='mecanumbot_sim_behavior_evaluator_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'actors_topic': '/sim/actors',
                'detection_evaluation_topic': '/sim/detection_evaluation',
                'odom_topic': 'odom',
                'cmd_vel_topic': '/cmd_vel',
                'behavior_evaluation_topic': '/sim/behavior_evaluation',
            }],
            condition=IfCondition(enable_behavior_evaluation),
        ),
        Node(
            namespace=namespace,
            package='mecanumbot_core',
            executable='mecanumbot_sim_visualization_node',
            name='mecanumbot_sim_visualization_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'actors_topic': '/sim/actors',
                'subject_pose_topic': 'subject_pose',
                'detection_evaluation_topic': '/sim/detection_evaluation',
                'behavior_evaluation_topic': '/sim/behavior_evaluation',
                'actor_markers_topic': '/sim/actor_markers',
                'evaluation_markers_topic': '/sim/evaluation_markers',
                'show_detection_markers': False,
            }],
            condition=IfCondition(enable_visualization_markers),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch),
            condition=IfCondition(use_rviz),
        ),
    ])
