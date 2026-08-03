from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            LogInfo(msg="Launching rosbag stepper and metric evaluation nodes."),
            Node(
                package="mecanumbot_monitor",
                executable="rosbag_stepper",
                output="screen",
            ),
            Node(
                package="mecanumbot_monitor",
                executable="metric_eval",
                output="screen",
            ),
        ]
    )
