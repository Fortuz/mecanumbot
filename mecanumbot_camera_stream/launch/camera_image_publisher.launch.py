from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = PathJoinSubstitution([
        FindPackageShare('mecanumbot_camera_stream'),
        'config',
        'camera_stream.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params_file),
        DeclareLaunchArgument('camera_backend', default_value='auto'),
        DeclareLaunchArgument('device', default_value='/dev/video0'),
        DeclareLaunchArgument('topic_name', default_value='/camera/image_raw'),
        DeclareLaunchArgument('width', default_value='1280'),
        DeclareLaunchArgument('height', default_value='720'),
        DeclareLaunchArgument('fps', default_value='30.0'),
        Node(
            package='mecanumbot_camera_stream',
            executable='camera_image_publisher_node',
            name='camera_image_publisher_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'camera_backend': LaunchConfiguration('camera_backend'),
                    'device': LaunchConfiguration('device'),
                    'topic_name': LaunchConfiguration('topic_name'),
                    'width': LaunchConfiguration('width'),
                    'height': LaunchConfiguration('height'),
                    'fps': LaunchConfiguration('fps'),
                },
            ],
        ),
    ])
