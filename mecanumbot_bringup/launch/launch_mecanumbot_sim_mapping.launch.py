import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='mecanumbot',
        description='Robot namespace',
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz together with the sim and SLAM toolbox',
    )

    bringup_share = get_package_share_directory('mecanumbot_bringup')

    sim_launch = os.path.join(bringup_share, 'launch', 'launch_mecanumbot_sim.launch.py')
    mapping_launch = os.path.join(bringup_share, 'launch', 'mapping.launch.py')
    rviz_launch = os.path.join(bringup_share, 'launch', 'rviz2.launch.py')

    return LaunchDescription([
        declare_namespace,
        declare_use_rviz,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch),
            launch_arguments={
                'use_sim_time': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch),
            condition=IfCondition(use_rviz),
        ),
    ])
