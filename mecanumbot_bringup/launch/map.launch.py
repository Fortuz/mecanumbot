import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    namespace = LaunchConfiguration('namespace', default='')
    use_respawn = LaunchConfiguration('use_respawn', default='false')
    log_level = LaunchConfiguration('log_level', default='debug')
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    mecanumbot_bringup_pkg_share = get_package_share_directory('mecanumbot_bringup')
  
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(
            mecanumbot_bringup_pkg_share,
            'param',
            'nav2_params.yaml')],
    )
    recovery_server_node = Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recovery_server',
            output='screen',
            parameters=[os.path.join(
                mecanumbot_bringup_pkg_share,
                'param',
                'nav2_params.yaml')],
        )
    controller_server_node = Node(
    package='nav2_controller',
    executable='controller_server',
    name='controller_server',
    output='screen',
    respawn=use_respawn,
    respawn_delay=2.0,
    parameters=[os.path.join(
        mecanumbot_bringup_pkg_share,
        'param',
        'nav2_params.yaml')],
    arguments=['--ros-args', '--log-level', log_level],
    remappings=remappings,
)
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[os.path.join(
            mecanumbot_bringup_pkg_share,
            'param',
            'nav2_params.yaml')],
    )
    planner_server_node = Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[os.path.join(
                            mecanumbot_bringup_pkg_share,
                            'param',
                            'nav2_params.yaml')],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings)
    navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[os.path.join(
            mecanumbot_bringup_pkg_share,
            'param',
            'nav2_params.yaml')],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings)
    
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl', 'recovery_server', 'planner_server', 'controller_server', 'bt_navigator']
        }]
)
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value=namespace,
            description='Namespace for nodes'),
        PushRosNamespace(namespace),
        map_server_node,
        navigator_node,
        recovery_server_node,
        planner_server_node,
        controller_server_node,
        amcl_node,
        lifecycle_manager_node,
    ])
