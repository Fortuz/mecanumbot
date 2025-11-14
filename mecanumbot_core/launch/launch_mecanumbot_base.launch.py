from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

yaml_file = os.path.join(
            get_package_share_directory('mecanumbot_core'),  # <-- correct package name
            'config',
            'Opencr_handling_params.yaml')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecanumbot_core',  # <-- correct package name
            executable='mecanumbot_io_node',
            name='mecanumbot_io_node',
            parameters=[yaml_file],
            output='screen'
        )
    ])