from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os  

mecanumbot_bringup_pkg_share = get_package_share_directory('mecanumbot_bringup')
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            parameters=[os.path.join(mecanumbot_bringup_pkg_share, 'param', 'picamera_config.yaml')],
            output='screen'
        )
    ])