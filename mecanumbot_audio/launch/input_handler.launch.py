from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='auto',
            description='ALSA capture device or auto',
        ),
        DeclareLaunchArgument(
            'topic_name',
            default_value='audio_input',
            description='Output audio topic',
        ),
        DeclareLaunchArgument(
            'sample_rate',
            default_value='16000',
            description='Audio sample rate in Hz',
        ),
        DeclareLaunchArgument(
            'channels',
            default_value='1',
            description='Number of audio channels',
        ),
        DeclareLaunchArgument(
            'chunk_size',
            default_value='1024',
            description='Frames per published chunk',
        ),
        Node(
            package='mecanumbot_audio',
            executable='input_handler',
            name='audio_input_handler',
            output='screen',
            parameters=[{
                'device': LaunchConfiguration('device'),
                'topic_name': LaunchConfiguration('topic_name'),
                'sample_rate': LaunchConfiguration('sample_rate'),
                'channels': LaunchConfiguration('channels'),
                'chunk_size': LaunchConfiguration('chunk_size'),
            }],
        ),
    ])
