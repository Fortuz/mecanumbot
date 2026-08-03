from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params_file = PathJoinSubstitution(
        [
            FindPackageShare("mecanumbot_camera_stream"),
            "config",
            "camera_compressed.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Path to compressed camera config file",
            ),
            DeclareLaunchArgument(
                "camera_backend",
                default_value="csi",
                description="Camera backend: auto, usb, or csi",
            ),
            DeclareLaunchArgument(
                "device", default_value="/dev/video0", description="Camera device path"
            ),
            DeclareLaunchArgument(
                "topic_name",
                default_value="/camera/image_raw/compressed",
                description="Output topic name",
            ),
            DeclareLaunchArgument(
                "width", default_value="640", description="Image width"
            ),
            DeclareLaunchArgument(
                "height", default_value="480", description="Image height"
            ),
            DeclareLaunchArgument(
                "fps", default_value="15.0", description="Frames per second"
            ),
            DeclareLaunchArgument(
                "jpeg_quality",
                default_value="80",
                description="JPEG quality (0-100, higher is better)",
            ),
            Node(
                package="mecanumbot_camera_stream",
                executable="compressed_camera_publisher_node",
                name="compressed_camera_publisher_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                ],
            ),
        ]
    )
