"""
Entry point: rclpy executor on the main thread, Flask on a daemon thread.

The threading is the other way round from the GUI this replaces, which
spun ROS in a background thread and ran Flask on main.  That arrangement
suited a Docker container whose whole job was to serve HTTP, but it makes
a poor ROS node: ``ros2 launch`` could not shut it down cleanly because
SIGINT never reached the executor.

Here the executor owns the main thread and Flask is a daemon, so the node
behaves like every other node in the workspace -- launched, signalled and
killed normally -- and the web server simply dies with it.
"""

import os
import threading

import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from .app import WebApp
from .rate_monitor import specs_from_config
from .ros_bridge import LED_POLL_PERIOD, WebNode
from .yaml_io import DEFAULT_BACKUP_ROOT


def _share(package: str, *parts) -> str:
    """Return a path inside a package's share directory, or '' if absent."""
    try:
        return os.path.join(get_package_share_directory(package), *parts)
    except (PackageNotFoundError, KeyError):
        return ""


def _load_monitor_settings(path: str, logger=None):
    """Read the diagnostics topic table, falling back to an empty monitor."""
    if not path or not os.path.isfile(path):
        if logger:
            logger.warn("Diagnostics config '{}' not found; no topics will be "
                        "monitored".format(path))
        return {"specs": [], "window_seconds": 3.0, "stale_after": 2.0}

    try:
        with open(path, "r") as stream:
            document = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        if logger:
            logger.error("Cannot read diagnostics config '{}': {}".format(path, exc))
        return {"specs": [], "window_seconds": 3.0, "stale_after": 2.0}

    return specs_from_config(document)


class _ParamReader(Node):
    """
    Throwaway node used only to read launch parameters before setup.

    The real node needs its monitor configuration at construction time,
    but that configuration is itself a ROS parameter, so it is read here
    first and the node is discarded.
    """

    def __init__(self):
        """Declare and hold every launch parameter."""
        super().__init__("mecanumbot_web_params")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter(
            "joystick_config_dir",
            _share("mecanumbot_description", "config", "joystick"))
        # One per startable behaviour. Each is resolved leniently: a
        # package missing from the checkout leaves its entry empty, and
        # the page shows that behaviour as unavailable instead of the
        # GUI failing to start.
        self.declare_parameter(
            "behaviour_config_dir",
            _share("mecanumbot_leading_behaviour", "config"))
        self.declare_parameter(
            "ostensive_config_dir",
            _share("mecanumbot_ostensive_behaviour", "config"))
        self.declare_parameter(
            "demo_config_dir",
            _share("mecanumbot_demo_behaviours", "config"))
        self.declare_parameter(
            "seek_config_dir",
            _share("mecanumbot_seek", "config"))
        self.declare_parameter(
            "fetch_config_dir",
            _share("mecanumbot_fetch_behaviour", "config"))
        self.declare_parameter(
            "autoslam_config_dir",
            _share("mecanumbot_autoslam", "config"))
        self.declare_parameter(
            "diagnostics_config",
            _share("mecanumbot_web", "config", "diagnostics_topics.yaml"))
        self.declare_parameter("joy_node", "/mecanumbot/mecanumbot_joy_node")
        self.declare_parameter("joy_topic", "/mecanumbot/joy")
        self.declare_parameter("backup_root", DEFAULT_BACKUP_ROOT)

        # Relative names resolve inside this node's namespace, which is
        # the robot's. Empty disables the source; the page then says so
        # rather than showing a permanently dead reading.
        self.declare_parameter("led_service", "get_led_status")
        self.declare_parameter("led_poll_period", LED_POLL_PERIOD)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "odom")

    def values(self) -> dict:
        """Return every parameter as a plain dict."""
        return {
            name: self.get_parameter(name).value
            for name in (
                "host", "port", "joystick_config_dir", "behaviour_config_dir",
                "ostensive_config_dir", "demo_config_dir", "seek_config_dir",
                "fetch_config_dir", "autoslam_config_dir",
                "diagnostics_config", "joy_node", "joy_topic", "backup_root",
                "led_service", "led_poll_period", "cmd_vel_topic", "odom_topic",
            )
        }


def main(args=None):
    """Start the web GUI and its ROS node."""
    rclpy.init(args=args)

    reader = _ParamReader()
    settings = reader.values()
    monitor = _load_monitor_settings(
        settings["diagnostics_config"], reader.get_logger())
    reader.destroy_node()

    node = None
    web = None
    try:
        node = WebNode(
            monitor_settings=monitor,
            joy_node=settings["joy_node"],
            joy_topic=settings["joy_topic"],
            led_service=settings["led_service"],
            led_poll_period=float(settings["led_poll_period"]),
            cmd_vel_topic=settings["cmd_vel_topic"],
            odom_topic=settings["odom_topic"],
        )

        # One entry per catalog key. A package missing from the checkout
        # leaves its directory empty, and the page shows that behaviour as
        # unavailable rather than the GUI failing to start.
        behaviour_dirs = {
            "leading": settings["behaviour_config_dir"],
            "ostensive": settings["ostensive_config_dir"],
            "seek": settings["seek_config_dir"],
            "fetch": settings["fetch_config_dir"],
            "autoslam": settings["autoslam_config_dir"],
            "demo": settings["demo_config_dir"],
        }
        for key, path in sorted(behaviour_dirs.items()):
            if not path:
                node.get_logger().warn(
                    "The '{}' behaviour package is not installed; that "
                    "behaviour cannot be edited or started".format(key))

        web = WebApp(
            node=node,
            joystick_dir=settings["joystick_config_dir"],
            behaviour_dir=settings["behaviour_config_dir"],
            behaviour_dirs=behaviour_dirs,
            backup_root=settings["backup_root"],
        )

        threading.Thread(
            target=web.run,
            kwargs={"host": settings["host"], "port": int(settings["port"])},
            daemon=True,
            name="mecanumbot_web_flask",
        ).start()

        node.get_logger().info(
            "Web GUI on http://{}:{}".format(
                settings["host"], settings["port"]))
        node.get_logger().info(
            "  joystick profiles: {}".format(settings["joystick_config_dir"]))
        node.get_logger().info(
            "  behaviour config:  {}".format(", ".join(
                "{}: {}".format(key, path or "(not installed)")
                for key, path in sorted(behaviour_dirs.items()))))
        node.get_logger().info(
            "  robot state:       LEDs via {} every {}s, movement from {} and {}"
            .format(settings["led_service"], settings["led_poll_period"],
                    settings["cmd_vel_topic"] or "(disabled)",
                    settings["odom_topic"] or "(disabled)"))

        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # A behaviour this node started is a child process, and Ctrl-C
        # reaches only this node -- the child has its own session so that
        # stopping it is a deliberate, ordered act rather than a race.
        # Which means shutting down here is the only thing that stops a
        # tree from outliving the stack it was driving.
        if web is not None:
            web.runner.shutdown()
        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
