"""
Start the compressed camera publisher: JPEG frames on one topic.

The declared arguments are **applied**, as parameter overrides on top of the
packaged config file. They used to be declared and then dropped on the floor,
which meant `width:=1280` did nothing and the node quietly kept the config's
640x480 -- and a consumer that had been told the frame was 1280 wide computed
every bearing off it and was wrong by the ratio between the two.

The geometry matters to more than bandwidth. Anything that turns a pixel into an
angle -- `mecanumbot_onboard_cam_detect_people`, `mecanumbot_locate_detections`
-- has to be configured with the same frame size this node captures at, which is
why `mecanumbot_bringup/launch/perception.launch.py` passes one width and height
to all three rather than letting each keep its own copy.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the launch description for the compressed camera publisher."""
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
                    # Overrides, so the arguments above actually reach the node.
                    # `publish_fps` is deliberately tied to `fps`: the config
                    # lets them differ (capture faster than you publish), but a
                    # caller who asks for a frame rate means the rate frames
                    # come out at.
                    {
                        "camera_backend": LaunchConfiguration("camera_backend"),
                        "device": LaunchConfiguration("device"),
                        "topic_name": LaunchConfiguration("topic_name"),
                        "width": ParameterValue(
                            LaunchConfiguration("width"), value_type=int
                        ),
                        "height": ParameterValue(
                            LaunchConfiguration("height"), value_type=int
                        ),
                        "fps": ParameterValue(
                            LaunchConfiguration("fps"), value_type=float
                        ),
                        "publish_fps": ParameterValue(
                            LaunchConfiguration("fps"), value_type=float
                        ),
                        "jpeg_quality": ParameterValue(
                            LaunchConfiguration("jpeg_quality"), value_type=int
                        ),
                    },
                ],
            ),
        ]
    )
