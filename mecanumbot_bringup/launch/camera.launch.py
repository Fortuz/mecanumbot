from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    """
    Optimized camera launch - uses JPEG compression for 118x bandwidth reduction.
    
    This replaces the old v4l2_camera with the optimized compressed camera.
    Publishes to: /camera/image_raw/compressed
    Bandwidth: ~220 KB/s (vs 26 MB/s with old camera)
    """
    
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mecanumbot_camera_stream'),
                'launch',
                'camera_compressed.launch.py'
            ])
        ])
    )
    
    return LaunchDescription([
        camera_launch
    ])
