"""
Launch the robot-hosted web GUI.

Included by ``mecanumbot_bringup/launch/launch_mecanumbot_base.launch.py``
under the ``use_web`` argument; also usable standalone.

The behaviour config directory is resolved leniently: if
``mecanumbot_leading_behaviour`` is not part of this checkout the argument
comes through empty and the behaviour page simply reports nothing to edit,
rather than the whole GUI failing to launch.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _share(package, *parts):
    """Return a path inside a package's share directory, or '' if absent."""
    try:
        return os.path.join(get_package_share_directory(package), *parts)
    except (PackageNotFoundError, KeyError):
        return ""


def generate_launch_description():
    """Build the launch description."""
    namespace = LaunchConfiguration("namespace")

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="mecanumbot"),
        DeclareLaunchArgument(
            "web_host", default_value="0.0.0.0",
            description="Interface to bind the web server to"),
        DeclareLaunchArgument(
            "web_port", default_value="8080",
            description="Port to serve the GUI on"),
        DeclareLaunchArgument(
            "joystick_config_dir",
            default_value=_share("mecanumbot_description", "config", "joystick"),
            description="Directory holding joystick profile YAML files"),
        DeclareLaunchArgument(
            "behaviour_config_dir",
            default_value=_share("mecanumbot_leading_behaviour", "config"),
            description="Directory holding behaviour constants YAML files"),
        DeclareLaunchArgument(
            "diagnostics_config",
            default_value=_share("mecanumbot_web", "config", "diagnostics_topics.yaml"),
            description="Table of monitored topics and their nominal rates"),
        DeclareLaunchArgument(
            "led_poll_period", default_value="2.0",
            description="Seconds between LED status reads; 0 disables them"),
        DeclareLaunchArgument(
            "odom_topic", default_value="odom",
            description="Odometry source for measured motion; empty disables it"),

        Node(
            package="mecanumbot_web",
            executable="mecanumbot_web_node",
            name="mecanumbot_web_node",
            namespace=namespace,
            output="screen",
            parameters=[{
                "host": LaunchConfiguration("web_host"),
                "port": LaunchConfiguration("web_port"),
                "joystick_config_dir": LaunchConfiguration("joystick_config_dir"),
                "behaviour_config_dir": LaunchConfiguration("behaviour_config_dir"),
                "diagnostics_config": LaunchConfiguration("diagnostics_config"),
                "joy_node": "/mecanumbot/mecanumbot_joy_node",
                "joy_topic": "/mecanumbot/joy",
                # Relative: resolved inside ``namespace``, alongside the
                # LED service node and mecanumbot_sensorproc_node.
                "led_service": "get_led_status",
                # Typed explicitly: an empty odom_topic must stay an
                # empty string rather than being inferred into None.
                "led_poll_period": ParameterValue(
                    LaunchConfiguration("led_poll_period"), value_type=float),
                "odom_topic": ParameterValue(
                    LaunchConfiguration("odom_topic"), value_type=str),
                # Absolute: the base launch remaps cmd_vel out of the
                # namespace so Nav2 and the joy node meet on one topic.
                "cmd_vel_topic": "/cmd_vel",
            }],
        ),
    ])
