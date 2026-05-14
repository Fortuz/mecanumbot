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
    enable_evaluation = LaunchConfiguration('enable_evaluation')
    enable_behavior_evaluation = LaunchConfiguration('enable_behavior_evaluation')
    enable_visualization_markers = LaunchConfiguration('enable_visualization_markers')
    enable_detector_debug_markers = LaunchConfiguration('enable_detector_debug_markers')

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
            'moving_human_patrol.yaml',
        ),
        description='Path to the simulation scenario YAML file',
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz together with the perception stack',
    )

    declare_enable_evaluation = DeclareLaunchArgument(
        'enable_evaluation',
        default_value='true',
        description='Publish /sim/detection_evaluation by comparing detector output with scenario ground truth',
    )

    declare_enable_behavior_evaluation = DeclareLaunchArgument(
        'enable_behavior_evaluation',
        default_value='true',
        description='Publish /sim/behavior_evaluation by checking command safety against perception and ground truth',
    )

    declare_enable_visualization_markers = DeclareLaunchArgument(
        'enable_visualization_markers',
        default_value='true',
        description='Publish RViz marker labels for sim actors, detections, and evaluator status',
    )

    declare_enable_detector_debug_markers = DeclareLaunchArgument(
        'enable_detector_debug_markers',
        default_value='true',
        description='Publish detector-to-truth association markers for scan/detector alignment debugging',
    )

    bringup_share = get_package_share_directory('mecanumbot_bringup')
    detector_yaml = os.path.join(
        get_package_share_directory('mecanumbot_sensorprocess_smart'),
        'param',
        'lidar_peopledetect_config.yaml',
    )
    rviz_launch = os.path.join(bringup_share, 'launch', 'rviz2.launch.py')
    sim_launch = os.path.join(bringup_share, 'launch', 'launch_mecanumbot_sim.launch.py')

    return LaunchDescription([
        declare_namespace,
        declare_scenario_path,
        declare_use_rviz,
        declare_enable_evaluation,
        declare_enable_behavior_evaluation,
        declare_enable_visualization_markers,
        declare_enable_detector_debug_markers,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': 'true',
                'subject_source': 'none',
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
            package='mecanumbot_sensorprocess_smart',
            executable='mecanumbot_lidar_detect_people',
            name='mecanumbot_lidar_detect_people',
            output='screen',
            parameters=[detector_yaml, {
                'use_sim_time': True,
                'leading_mode': True,
                'robot_profile': 'sim',
                'tracker_mode': 'simple',
                'enable_map_filter': False,
                'use_detection_header_frame': True,
                'reuse_last_subject_pose': True,
            }],
        ),
        Node(
            namespace=namespace,
            package='mecanumbot_core',
            executable='mecanumbot_sim_detection_evaluator_node',
            name='mecanumbot_sim_detection_evaluator_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'subject_pose_topic': 'subject_pose',
                'detections_topic': 'dr_spaam/dets',
                'actors_topic': '/sim/actors',
                'evaluation_topic': '/sim/detection_evaluation',
            }],
            condition=IfCondition(enable_evaluation),
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
                'detections_topic': 'dr_spaam/dets',
                'subject_pose_topic': 'subject_pose',
                'detection_evaluation_topic': '/sim/detection_evaluation',
                'behavior_evaluation_topic': '/sim/behavior_evaluation',
                'actor_markers_topic': '/sim/actor_markers',
                'detection_markers_topic': '/sim/detection_markers',
                'evaluation_markers_topic': '/sim/evaluation_markers',
            }],
            condition=IfCondition(enable_visualization_markers),
        ),
        Node(
            namespace=namespace,
            package='mecanumbot_core',
            executable='mecanumbot_sim_detector_debug_node',
            name='mecanumbot_sim_detector_debug_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'actors_topic': '/sim/actors',
                'detections_topic': 'dr_spaam/dets',
                'debug_markers_topic': '/sim/detector_debug_markers',
                'map_frame': 'map',
            }],
            condition=IfCondition(enable_detector_debug_markers),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch),
            condition=IfCondition(use_rviz),
        ),
    ])
