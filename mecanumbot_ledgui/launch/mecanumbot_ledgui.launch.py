from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecanumbot_ledgui',   # ROS csomag neve
            executable='mecanumbot_ledgui',# a console_scripts-ben definiált név
            output='screen'
        )
    ])
