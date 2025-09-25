import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')


    nav2_bringup_pkg_share = get_package_share_directory('nav2_bringup')

    nav2_bringup_launch_file = os.path.join(
                                            nav2_bringup_pkg_share,
                                            'launch',
                                            'navigation_launch.py'
                                            )
    nav2_bringup_launch = IncludeLaunchDescription(
                                            PythonLaunchDescriptionSource(nav2_bringup_launch_file),
                                            launch_arguments={
                                                'use_sim_time': use_sim_time,
                                                'params_file': os.path.join(
                                                    get_package_share_directory('mecanumbot_bringup'),
                                                    'param',
                                                    'nav2_params.yaml')
                                                }.items()
                                            )
    return LaunchDescription([
        nav2_bringup_launch
    ])
