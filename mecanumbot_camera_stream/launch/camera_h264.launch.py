from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = PathJoinSubstitution([
        FindPackageShare('mecanumbot_camera_stream'),
        'config',
        'camera_h264.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to H.264 camera config file'
        ),
        DeclareLaunchArgument(
            'device',
            default_value='/dev/video0',
            description='Camera device path'
        ),
        DeclareLaunchArgument(
            'topic_name',
            default_value='/camera/image_raw/h264',
            description='Output topic name'
        ),
        DeclareLaunchArgument(
            'width',
            default_value='640',
            description='Image width'
        ),
        DeclareLaunchArgument(
            'height',
            default_value='480',
            description='Image height'
        ),
        DeclareLaunchArgument(
            'fps',
            default_value='15.0',
            description='Frames per second'
        ),
        DeclareLaunchArgument(
            'bitrate',
            default_value='2000000',
            description='H.264 bitrate in bits per second (2Mbps default)'
        ),
        DeclareLaunchArgument(
            'hardware_encoder',
            default_value='auto',
            description='Hardware encoder: auto, nvenc, vaapi, omx, or x264'
        ),
        Node(
            package='mecanumbot_camera_stream',
            executable='h264_camera_publisher_node',
            name='h264_camera_publisher_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'device': LaunchConfiguration('device'),
                    'topic_name': LaunchConfiguration('topic_name'),
                    'width': LaunchConfiguration('width'),
                    'height': LaunchConfiguration('height'),
                    'fps': LaunchConfiguration('fps'),
                    'bitrate': LaunchConfiguration('bitrate'),
                    'hardware_encoder': LaunchConfiguration('hardware_encoder'),
                },
            ],
        ),
    ])
