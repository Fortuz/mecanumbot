from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Profile mapping: profile name -> config file
    # Profiles: low_bandwidth, medium, high, compressed, compressed_hq, h264, h264_hq

    default_params_file = PathJoinSubstitution(
        [
            FindPackageShare("mecanumbot_camera_stream"),
            "config",
            "camera_medium.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="medium",
                description="Camera profile: low_bandwidth, medium, high (raw images)",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Path to camera config file (overrides profile)",
            ),
            DeclareLaunchArgument(
                "camera_backend",
                default_value="auto",
                description="Camera backend: auto, usb, or csi",
            ),
            DeclareLaunchArgument(
                "device", default_value="/dev/video0", description="Camera device path"
            ),
            DeclareLaunchArgument(
                "topic_name",
                default_value="/camera/image_raw",
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
            Node(
                package="mecanumbot_camera_stream",
                executable="camera_image_publisher_node",
                name="camera_image_publisher_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        "camera_backend": LaunchConfiguration("camera_backend"),
                        "device": LaunchConfiguration("device"),
                        "topic_name": LaunchConfiguration("topic_name"),
                        "width": LaunchConfiguration("width"),
                        "height": LaunchConfiguration("height"),
                        "fps": LaunchConfiguration("fps"),
                    },
                ],
            ),
        ]
    )
