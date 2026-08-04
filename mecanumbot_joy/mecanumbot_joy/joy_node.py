"""
The onboard joystick node: a thin rclpy adapter over the pure core.

This node is the single consumer of ``/joy`` on the robot.  It replaces
both of the previous joystick paths -- ``mecanumbot_teleop``'s
``teleop_joystick`` (which hardcoded its button semantics) and
``mecanumbot_gui``'s ``controller_node`` + ``mapping_listener`` pair
(which routed input through custom event messages into a SQLite-backed
action interpreter).

Everything interesting lives in :mod:`~mecanumbot_joy.layout`,
:mod:`~mecanumbot_joy.profile` and :mod:`~mecanumbot_joy.actions`, none of
which import rclpy.  What is left here is exactly the ROS-shaped work:
subscribe, publish, serve, and own the clock.

Runtime control uses only standard interfaces, so that consolidating the
joystick stack could *shrink* ``mecanumbot_msgs`` rather than grow it:

``profile`` parameter
    Set to a profile stem to switch, or ``auto`` to fingerprint the pad.
``~/reload_profile`` (``std_srvs/Trigger``)
    Re-read the active profile from disk after an external edit.
``~/clear_estop`` (``std_srvs/Trigger``)
    Release the e-stop latch.
``~/active_profile`` (``std_msgs/String``, transient local)
``~/estop`` (``std_msgs/Bool``, transient local)
    Status, latched for late joiners.
"""

import os

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from mecanumbot_msgs.msg import AccessMotorCmd
from mecanumbot_msgs.srv import SetLedStatus

from .actions import ActionExecutor, unbound_actions
from .layout import describe_controls, fingerprint, resolve_controls
from .profile import LED_CORNERS, ProfileError, load_profile_dir, select_profile

#: Where profiles ship, unless the ``profile_dir`` parameter overrides it.
DEFAULT_PROFILE_PACKAGE = "mecanumbot_description"
DEFAULT_PROFILE_SUBDIR = os.path.join("config", "joystick")

#: Used when ``profile: auto`` finds no fingerprint match.
FALLBACK_PROFILE = "generic"


def _latched_qos(depth: int = 1) -> QoSProfile:
    """Return a transient-local QoS so late subscribers get current status."""
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _default_profile_dir() -> str:
    """Return the packaged profile directory, or '' if unavailable."""
    try:
        share = get_package_share_directory(DEFAULT_PROFILE_PACKAGE)
    except (PackageNotFoundError, KeyError):
        return ""
    return os.path.join(share, DEFAULT_PROFILE_SUBDIR)


class MecanumbotJoyNode(Node):
    """Translate ``/joy`` into robot commands using a YAML profile."""

    def __init__(self):
        """Load profiles, wire up the graph, and start the publish timer."""
        super().__init__("mecanumbot_joy_node")

        self.declare_parameter(
            "profile_dir",
            _default_profile_dir(),
            ParameterDescriptor(description="Directory holding joystick profile YAML files"),
        )
        self.declare_parameter(
            "profile",
            "auto",
            ParameterDescriptor(
                description="Profile stem to load, or 'auto' to detect from the pad"),
        )

        self._profile_dir = self.get_parameter("profile_dir").value
        self._profiles = {}
        self._load_errors = {}
        self._active_stem = None
        self._auto = True
        self._detected = False

        self._executor_core = None
        self._timer = None
        self._zero_ticks = 0
        self._joy_last = None
        self._joy_lost = False
        self._accessory_last_sent = None

        callback_group = ReentrantCallbackGroup()

        qos = QoSProfile(depth=10)
        self._pub_vel = self.create_publisher(Twist, "cmd_vel", qos)
        self._pub_accessory = self.create_publisher(AccessMotorCmd, "cmd_accessory_pos", qos)
        self._pub_profile = self.create_publisher(String, "~/active_profile", _latched_qos())
        self._pub_estop = self.create_publisher(Bool, "~/estop", _latched_qos())

        self._led_client = self.create_client(
            SetLedStatus, "/mecanumbot/set_led_status", callback_group=callback_group)

        self.create_service(
            Trigger, "~/reload_profile", self._on_reload, callback_group=callback_group)
        self.create_service(
            Trigger, "~/clear_estop", self._on_clear_estop, callback_group=callback_group)

        self._reload_profiles()
        self._apply_requested_profile(self.get_parameter("profile").value, initial=True)

        self.create_subscription(Joy, "joy", self._on_joy, qos)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._publish_estop()
        self.get_logger().info("mecanumbot_joy_node ready; waiting for /joy")

    # ── profile management ───────────────────────────────────────────────

    def _reload_profiles(self) -> None:
        """Re-read every profile in ``profile_dir``, logging any failures."""
        if not self._profile_dir or not os.path.isdir(self._profile_dir):
            self.get_logger().error(
                "Profile directory '{}' does not exist".format(self._profile_dir))
            self._profiles, self._load_errors = {}, {}
            return

        self._profiles, self._load_errors = load_profile_dir(self._profile_dir)
        for stem, message in self._load_errors.items():
            self.get_logger().error("Profile '{}' is invalid: {}".format(stem, message))
        if self._profiles:
            self.get_logger().info(
                "Loaded {} joystick profile(s) from {}: {}".format(
                    len(self._profiles), self._profile_dir, ", ".join(sorted(self._profiles))))

    def _apply_requested_profile(self, requested, initial: bool = False) -> None:
        """Honour a ``profile`` parameter value, resolving 'auto' lazily."""
        self._auto = (requested or "auto") == "auto"
        self._detected = False

        if self._auto:
            # Until the first frame arrives we cannot fingerprint, so run on
            # the fallback rather than leaving the robot with no bindings.
            stem = FALLBACK_PROFILE if FALLBACK_PROFILE in self._profiles else None
            if stem is None and self._profiles:
                stem = sorted(self._profiles)[0]
        else:
            stem = requested

        if stem is None or stem not in self._profiles:
            if initial:
                raise ProfileError(
                    "No usable joystick profile in '{}'".format(self._profile_dir))
            return

        self._activate(stem, reason="startup" if initial else "parameter")

    def _activate(self, stem: str, reason: str) -> None:
        """Make ``stem`` the live profile and resize the publish timer."""
        profile = self._profiles[stem]

        if self._executor_core is None:
            self._executor_core = ActionExecutor(profile)
        else:
            self._executor_core.set_profile(profile)

        self._active_stem = stem
        self._rebuild_timer(profile.limits["publish_hz"])

        self.get_logger().info(
            "Joystick profile '{}' active ({})".format(stem, reason))
        for line in describe_controls(profile):
            self.get_logger().info(line)

        missing = unbound_actions(profile)
        if missing:
            level = self.get_logger().warn if "estop" in missing else self.get_logger().info
            level("Profile '{}' leaves unbound: {}".format(stem, ", ".join(missing)))

        message = String()
        message.data = stem
        self._pub_profile.publish(message)

    def _rebuild_timer(self, publish_hz: float) -> None:
        """Recreate the publish timer at the active profile's rate."""
        if self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)
        self._timer = self.create_timer(1.0 / max(1.0, publish_hz), self._on_tick)

    def _on_set_parameters(self, parameters):
        """Validate and apply a runtime ``profile`` change."""
        for parameter in parameters:
            if parameter.name != "profile":
                continue
            requested = parameter.value
            if requested != "auto" and requested not in self._profiles:
                return SetParametersResult(
                    successful=False,
                    reason="Unknown profile '{}'. Available: auto, {}".format(
                        requested, ", ".join(sorted(self._profiles))),
                )
            self._apply_requested_profile(requested)
        return SetParametersResult(successful=True)

    # ── services ─────────────────────────────────────────────────────────

    def _on_reload(self, request, response):
        """
        Re-read profiles from disk, keeping the old one on failure.

        A reload must never leave the robot with no bindings, so the new
        document is parsed and validated before anything is swapped in.
        """
        del request

        previous = self._profiles
        self._reload_profiles()

        if self._load_errors and self._active_stem in self._load_errors:
            self._profiles = previous
            response.success = False
            response.message = "Reload rejected, keeping '{}': {}".format(
                self._active_stem, self._load_errors[self._active_stem])
            self.get_logger().error(response.message)
            return response

        if not self._profiles:
            self._profiles = previous
            response.success = False
            response.message = "Reload found no valid profiles; keeping the previous set"
            self.get_logger().error(response.message)
            return response

        stem = self._active_stem if self._active_stem in self._profiles else None
        if stem is None:
            stem = select_profile(self._profiles, 0, 0, FALLBACK_PROFILE)
        self._activate(stem, reason="reload")

        response.success = True
        response.message = "Reloaded '{}' from {}".format(stem, self._profile_dir)
        if self._load_errors:
            response.message += " (invalid: {})".format(", ".join(sorted(self._load_errors)))
        return response

    def _on_clear_estop(self, request, response):
        """Release the e-stop latch."""
        del request
        cleared = self._executor_core.clear_estop()
        self._publish_estop()
        response.success = True
        response.message = "E-stop cleared" if cleared else "E-stop was not engaged"
        self.get_logger().info(response.message)
        return response

    # ── input ────────────────────────────────────────────────────────────

    def _on_joy(self, message: Joy) -> None:
        """Handle one controller frame."""
        if self._auto and not self._detected:
            self._detect(message)

        if self._joy_lost:
            self.get_logger().info("Controller input resumed")
            self._joy_lost = False

        self._joy_last = self.get_clock().now()
        self._executor_core.on_frame(
            resolve_controls(message.axes, message.buttons, self._executor_core.profile))

    def _detect(self, message: Joy) -> None:
        """Fingerprint the pad and switch to the matching profile."""
        self._detected = True
        shape = fingerprint(message.axes, message.buttons)
        stem = select_profile(
            self._profiles, shape["axes"], shape["buttons"], FALLBACK_PROFILE)

        if stem is None:
            self.get_logger().error("No joystick profile available for auto-selection")
            return

        self.get_logger().info(
            "Controller reports {} axes, {} buttons".format(shape["axes"], shape["buttons"]))
        if stem != self._active_stem:
            self._activate(stem, reason="auto-detected")
        else:
            self.get_logger().info("Joystick profile '{}' active (auto-detected)".format(stem))

    # ── output ───────────────────────────────────────────────────────────

    def _on_tick(self) -> None:
        """Advance the executor and publish whatever changed."""
        self._check_joy_timeout()

        intents = self._executor_core.tick()
        limits = self._executor_core.profile.limits

        if not intents.twist_is_zero:
            self._publish_twist(intents.twist)
            self._zero_ticks = 0
        elif self._zero_ticks < int(limits["zero_hold_ticks"]):
            # Keep sending zeros briefly after a stop so the drive train
            # cannot miss the one message that ends the motion, then go
            # quiet instead of republishing forever.
            self._publish_twist((0.0, 0.0, 0.0))
            self._zero_ticks += 1

        self._maybe_publish_accessory(intents, limits)

        if intents.led is not None:
            self._send_led(intents.led)

        if intents.estop_changed:
            self._publish_estop()
            self.get_logger().warn(
                "E-stop ENGAGED" if intents.estop else "E-stop released")

    def _check_joy_timeout(self) -> None:
        """Stop the wheels if the controller has gone silent."""
        if self._joy_last is None or self._joy_lost:
            return
        timeout = self._executor_core.profile.limits["joy_timeout"]
        elapsed = (self.get_clock().now() - self._joy_last).nanoseconds / 1e9
        if elapsed > timeout:
            self._joy_lost = True
            self._executor_core.on_joy_lost()
            self._zero_ticks = 0
            self.get_logger().warn(
                "No /joy for {:.1f}s; stopping".format(elapsed))

    def _publish_twist(self, velocity) -> None:
        """Publish one velocity command."""
        message = Twist()
        message.linear.x = float(velocity[0])
        message.linear.y = float(velocity[1])
        message.angular.z = float(velocity[2])
        self._pub_vel.publish(message)

    def _maybe_publish_accessory(self, intents, limits) -> None:
        """
        Publish accessory positions on change, plus a slow keepalive.

        These are positions rather than velocities, so republishing them
        at the drive rate is pure noise -- but a periodic refresh still
        matters so a board that missed the change eventually catches up.
        """
        now = self.get_clock().now()
        due = False
        if self._accessory_last_sent is not None:
            keepalive_hz = max(0.0, limits["accessory_keepalive_hz"])
            if keepalive_hz > 0.0:
                elapsed = (now - self._accessory_last_sent).nanoseconds / 1e9
                due = elapsed >= 1.0 / keepalive_hz

        if not (intents.accessory_changed or due):
            return

        message = AccessMotorCmd()
        message.n_pos = float(intents.accessory[0])
        message.gl_pos = float(intents.accessory[1])
        message.gr_pos = float(intents.accessory[2])
        self._pub_accessory.publish(message)
        self._accessory_last_sent = now

    def _send_led(self, preset) -> None:
        """Fire a SetLedStatus request without blocking the timer."""
        if not self._led_client.service_is_ready():
            self.get_logger().warn("set_led_status unavailable; ignoring LED preset")
            return

        request = SetLedStatus.Request()
        for corner in LED_CORNERS:
            entry = preset.get(corner, {})
            setattr(request, "{}_mode".format(corner), int(entry.get("mode", 0)))
            setattr(request, "{}_color".format(corner), int(entry.get("color", 0)))
        self._led_client.call_async(request)

    def _publish_estop(self) -> None:
        """Publish the current e-stop latch state."""
        message = Bool()
        message.data = self._executor_core.estop
        self._pub_estop.publish(message)


def main(args=None):
    """Spin the joystick node."""
    rclpy.init(args=args)
    node = None
    try:
        node = MecanumbotJoyNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except ProfileError as exc:
        print("mecanumbot_joy_node: {}".format(exc))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
