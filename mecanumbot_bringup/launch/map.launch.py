import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    map_name = 'AI_room_lounge'
    mecanumbot_description_pkg_share = get_package_share_directory('mecanumbot_description')
    param_file = os.path.join(mecanumbot_description_pkg_share, 'param', 'mecanumbot_custom_nav2.yaml')
    map_file = os.path.join(mecanumbot_description_pkg_share,'maps',map_name, f"{map_name}.yaml")

    # Declare a namespace argument
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='mecanumbot',
        description='Namespace for the robot'
    )
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        declare_namespace,

        # Push namespace so all nodes inside will be namespaced
        PushRosNamespace(namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('nav2_bringup'), 
                    'launch', 'bringup_launch.py'
                )
            ]),
            launch_arguments={
                "map": map_file,
                "params_file": param_file,
                "use_sim_time": "false",  # or true if in simulation
            }.items()
        ),
    ])
