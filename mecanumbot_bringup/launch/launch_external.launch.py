import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, SetRemap
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    map_name = 'AI_dept'
    mecanumbot_description_pkg_share = get_package_share_directory('mecanumbot_description')
    rviz_config_dir = os.path.join(get_package_share_directory('mecanumbot_description'),'rviz','model.rviz')
    param_file = os.path.join(mecanumbot_description_pkg_share, 'param', 'mecanumbot_custom_nav2.yaml')
    map_file = os.path.join(mecanumbot_description_pkg_share, 'maps', map_name, f"{map_name}.yaml")
    yaml_file = os.path.join(get_package_share_directory('mecanumbot_sensorprocess_smart'),'config','lidar_peopledetect_config.yaml')
    return LaunchDescription([
        GroupAction([

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('nav2_bringup'),
                        'launch',
                        'bringup_launch.py'
                    )
                ),
                launch_arguments={
                    "map": map_file,
                    "params_file": param_file,
                    "use_sim_time": "false",
                }.items()
            ),
        ]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'),

        Node(
            namespace="mecanumbot",
            package="mecanumbot_sensorprocess_smart",
            executable="mecanumbot_lidar_detect_people",
            name="mecanumbot_lidar_detect_people",  # must match YAML top-level key
            output="screen",
            parameters=[yaml_file]
)
    ])
