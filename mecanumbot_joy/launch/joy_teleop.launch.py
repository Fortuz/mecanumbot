"""
Launch the joy driver and the mecanumbot joystick node together.

Included by ``mecanumbot_bringup/launch/launch_mecanumbot_base.launch.py``;
also usable standalone for bench testing a gamepad.

Note that ``joy_node``'s own deadzone is set to 0.0 on purpose.  The
previous setup applied 0.05 here *and* another 0.05 in the teleop node,
so the two stacked and the effective deadzone was neither value.  It is
now applied exactly once, in the profile, where the GUI can edit it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description."""
    default_profile_dir = os.path.join(
        get_package_share_directory("mecanumbot_description"), "config", "joystick")

    namespace = LaunchConfiguration("namespace")
    joystick_profile = LaunchConfiguration("joystick_profile")
    joystick_config_dir = LaunchConfiguration("joystick_config_dir")
    joy_dev = LaunchConfiguration("joy_dev")

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace", default_value="mecanumbot",
            description="Namespace for both the joy driver and the joystick node"),
        DeclareLaunchArgument(
            "joystick_profile", default_value="auto",
            description="Profile stem to load, or 'auto' to detect from the pad"),
        DeclareLaunchArgument(
            "joystick_config_dir", default_value=default_profile_dir,
            description="Directory holding joystick profile YAML files"),
        DeclareLaunchArgument(
            "joy_dev", default_value="/dev/input/js0",
            description="Joystick device node"),

        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            namespace=namespace,
            output="screen",
            parameters=[{
                "dev": joy_dev,
                # Deadzone is applied once, in the profile. See module docstring.
                "deadzone": 0.0,
                # Republish while idle so the joystick node can tell a
                # centred stick from an unplugged pad.
                "autorepeat_rate": 20.0,
            }],
        ),

        Node(
            package="mecanumbot_joy",
            executable="mecanumbot_joy_node",
            name="mecanumbot_joy_node",
            namespace=namespace,
            output="screen",
            parameters=[{
                "profile": joystick_profile,
                "profile_dir": joystick_config_dir,
            }],
            # Match the remaps every other node in the base launch uses, so
            # commands land on the global topics mecanumbot_io_node reads.
            remappings=[
                ("/mecanumbot/cmd_vel", "/cmd_vel"),
                ("/mecanumbot/cmd_accessory_pos", "/cmd_accessory_pos"),
            ],
        ),
    ])
