"""
The ROS side of the web GUI: rate monitoring, robot state, joy control.

Runs as a normal rclpy node under a ``MultiThreadedExecutor``.  Flask
serves from a daemon thread and reaches in here through the accessors
below, all of which are lock-guarded.

Three details are load bearing and easy to get wrong:

**Service calls come from Flask threads.**  ``call_async`` returns a
future that only completes when the executor spins, so a Flask thread has
to block on an event rather than on the future.  The clients therefore
sit in a :class:`ReentrantCallbackGroup`; under the default mutually
exclusive group the executor thread would be busy inside the request and
the future would never complete.  The ``if future.done()`` re-check after
registering the callback closes the race where completion beats
registration.

**Monitor subscriptions are raw.**  ``create_subscription(..., raw=True)``
hands the callback undeserialised bytes, so subscribing to a 100 Hz
``OpenCRState`` costs essentially nothing on the Orin -- which is what
makes it acceptable to run this permanently alongside the robot stack
rather than only on demand.

**The LED poll never blocks.**  ``get_led_status`` is a serial round trip
with a sleep inside it, and it is called from an executor timer rather
than from a request, so it uses ``call_async`` with a done callback.
Blocking there the way the Flask-thread helper does would stall a
callback group that the rest of this node shares.
"""

import threading
import time
from functools import partial
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import Twist
from mecanumbot_msgs.srv import GetLedStatus
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from . import led_enums, motion
from .rate_monitor import RateMonitor, TopicSpec

#: How often the graph is rescanned for topics that have appeared.
DISCOVERY_PERIOD = 2.0

#: Default blocking timeout for a service call made from a Flask thread.
SERVICE_TIMEOUT = 8.0

#: How often the LED controller is asked what it is showing.  Every poll
#: is a serial exchange on the same link the behaviour trees use to set
#: the LEDs, so this is deliberately far slower than the page refresh --
#: the page reads a cache.
LED_POLL_PERIOD = 2.0

#: A cached LED reading older than this is reported as stale.
LED_STALE_AFTER = 8.0

#: Seconds of ``cmd_vel`` silence after which nothing is commanding the
#: robot.  Short, because the topic ticks continuously while driving.
COMMAND_STALE_AFTER = 0.5

#: Seconds of odometry silence after which measured motion is unknown.
ODOM_STALE_AFTER = 2.0

#: Minimum gap between odometry deserialisations.  See _on_odom.
ODOM_DECIMATE = 0.2


def _monitor_qos() -> QoSProfile:
    """
    Build the QoS profile used for rate-monitoring subscriptions.

    Best effort on purpose.  ``scan`` and the compressed camera topics are
    published best effort, and a *reliable* subscription simply never
    matches a best-effort publisher -- the page would report a perfectly
    healthy topic as DEAD.  A best-effort subscriber matches both kinds.
    """
    return QoSProfile(
        depth=5,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )


class WebNode(Node):
    """Measures topic rates and robot state, proxies joystick control."""

    def __init__(
        self,
        monitor_settings=None,
        joy_node: str = "",
        joy_topic: str = "",
        led_service: str = "",
        led_poll_period: float = LED_POLL_PERIOD,
        cmd_vel_topic: Optional[str] = None,
        odom_topic: Optional[str] = None,
    ):
        """Create the node, its monitor and its clients to the robot."""
        super().__init__("mecanumbot_web_node")

        settings = monitor_settings or {}
        self._monitor = RateMonitor(
            specs=settings.get("specs") or (),
            window_seconds=settings.get("window_seconds", 3.0),
            stale_after=settings.get("stale_after", 2.0),
        )
        self._rate_lock = threading.Lock()
        # Not ``_subscriptions``: rclpy's Node keeps its own list under that name.
        self._monitor_subs: Dict[str, object] = {}

        self._joy_node = joy_node or "/mecanumbot/mecanumbot_joy_node"
        self._joy_topic = joy_topic or "/mecanumbot/joy"

        self._joy_lock = threading.Lock()
        self._latest_joy: Optional[dict] = None
        self._active_profile: Optional[str] = None
        self._estop: Optional[bool] = None

        # Relative names on purpose: this node is launched into the
        # ``mecanumbot`` namespace and so are the LED service and odom,
        # so they resolve alongside it without repeating the namespace.
        # ``cmd_vel`` is absolute because the base launch remaps it out
        # of the namespace for Nav2's benefit.
        self._led_service = led_service or "get_led_status"
        self._led_poll_period = float(led_poll_period)
        self._cmd_vel_topic = "/cmd_vel" if cmd_vel_topic is None else cmd_vel_topic
        self._odom_topic = "odom" if odom_topic is None else odom_topic

        self._led_lock = threading.Lock()
        self._led: Optional[dict] = None
        self._led_stamp: Optional[float] = None
        self._led_error: Optional[str] = None
        self._led_pending = False

        self._motion_lock = threading.Lock()
        self._commanded: Optional[tuple] = None
        self._measured: Optional[tuple] = None
        self._odom_decoded_at = 0.0

        group = ReentrantCallbackGroup()
        self._group = group

        self._reload_client = self.create_client(
            Trigger, "{}/reload_profile".format(self._joy_node), callback_group=group)
        self._clear_estop_client = self.create_client(
            Trigger, "{}/clear_estop".format(self._joy_node), callback_group=group)
        self._set_params_client = self.create_client(
            SetParameters, "{}/set_parameters".format(self._joy_node),
            callback_group=group)

        self.create_subscription(
            Joy, self._joy_topic, self._on_joy, _monitor_qos(), callback_group=group)
        self.create_subscription(
            String, "{}/active_profile".format(self._joy_node),
            self._on_active_profile, self._latched_qos(), callback_group=group)
        self.create_subscription(
            Bool, "{}/estop".format(self._joy_node),
            self._on_estop, self._latched_qos(), callback_group=group)

        self._led_client = self.create_client(
            GetLedStatus, self._led_service, callback_group=group)
        if self._led_poll_period > 0.0:
            self.create_timer(
                self._led_poll_period, self._poll_led, callback_group=group)

        if self._cmd_vel_topic:
            self.create_subscription(
                Twist, self._cmd_vel_topic, self._on_cmd_vel, _monitor_qos(),
                callback_group=group)
        if self._odom_topic:
            self.create_subscription(
                Odometry, self._odom_topic, self._on_odom, _monitor_qos(),
                raw=True, callback_group=group)

        self.create_timer(DISCOVERY_PERIOD, self._discover, callback_group=group)
        self._discover()

        self.get_logger().info(
            "Monitoring {} topics; joy node at {}".format(
                len(self._monitor.topics), self._joy_node))

    @staticmethod
    def _latched_qos() -> QoSProfile:
        """Transient-local QoS matching the joy node's status publishers."""
        from rclpy.qos import DurabilityPolicy
        return QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

    # ── rate monitoring ──────────────────────────────────────────────────

    def _discover(self) -> None:
        """
        Subscribe to configured topics that have appeared, refresh counts.

        Re-run periodically rather than once at startup: this node is
        launched alongside the rest of the stack, so most of what it
        monitors does not exist yet when it comes up.
        """
        try:
            available = dict(self.get_topic_names_and_types())
        except Exception as exc:  # pragma: no cover - rmw-dependent
            self.get_logger().warn("Topic discovery failed: {}".format(exc))
            return

        for topic in self._monitor.topics:
            try:
                count = self.count_publishers(topic)
            except Exception:  # pragma: no cover - rmw-dependent
                count = 0
            subscribers = self._external_subscriber_count(topic)
            with self._rate_lock:
                self._monitor.set_publisher_count(topic, count)
                self._monitor.set_subscriber_count(topic, subscribers)

            if topic in self._monitor_subs or topic not in available:
                continue

            types = available.get(topic) or []
            if not types:
                continue

            try:
                message_class = get_message(types[0])
            except (ImportError, AttributeError, ValueError) as exc:
                self.get_logger().warn(
                    "Cannot monitor {} ({}): {}".format(topic, types[0], exc))
                # Record it so the failure is not retried every 2 seconds.
                self._monitor_subs[topic] = None
                continue

            self._monitor_subs[topic] = self.create_subscription(
                message_class,
                topic,
                partial(self._on_monitored, topic),
                _monitor_qos(),
                raw=True,
                callback_group=self._group,
            )
            self.get_logger().info("Monitoring {} [{}]".format(topic, types[0]))

    def _external_subscriber_count(self, topic: str) -> int:
        """
        Count subscribers on a topic, excluding this node's own.

        Load bearing for the command topics, whose liveness is judged by
        who is listening: this node subscribes to everything it monitors,
        so a raw ``count_subscribers`` would never be zero and ``/cmd_vel``
        could never read DEAD, however far down the io_node was.  The
        endpoint list carries node names, so our own entries are dropped
        by name rather than by counting our subscriptions -- which would
        have to resolve relative topic names to match.
        """
        try:
            endpoints = self.get_subscriptions_info_by_topic(topic)
        except Exception:  # pragma: no cover - rmw-dependent
            return 0

        name, namespace = self.get_name(), self.get_namespace()
        return sum(
            1 for endpoint in endpoints
            if not (endpoint.node_name == name
                    and endpoint.node_namespace == namespace)
        )

    def _on_monitored(self, topic: str, message) -> None:
        """Record a raw message receipt. The payload is never deserialised."""
        del message
        now = time.monotonic()
        with self._rate_lock:
            self._monitor.record(topic, now)

    def diagnostics(self) -> dict:
        """Return rates, LED state and movement, for the diagnostics API."""
        now = time.monotonic()
        with self._rate_lock:
            readings = self._monitor.read(now)
            summary = self._monitor.summary(readings)
        # Both of these take their own locks; taking them outside the
        # rate lock keeps the two independent and unorderable.
        return {
            "topics": [reading.as_dict() for reading in readings],
            "summary": summary,
            "window_seconds": self._monitor.window_seconds,
            "stale_after": self._monitor.stale_after,
            "led": self.led_state(),
            "motion": self.motion_state(),
        }

    def reset_rates(self) -> None:
        """Clear every measurement window."""
        with self._rate_lock:
            self._monitor.reset()

    def monitor_specs(self) -> List[dict]:
        """Return the configured nominal table, for rendering before data."""
        with self._rate_lock:
            specs = [self._monitor.spec(topic) for topic in self._monitor.topics]
        return [
            {
                "topic": spec.topic,
                "nominal_hz": spec.nominal_hz,
                "tol": spec.tol,
                "label": spec.label,
                "note": spec.note,
                "idle_when_silent": spec.idle_when_silent,
                "presence": spec.presence,
            }
            for spec in specs if spec is not None
        ]

    # ── behaviour trees on the graph ─────────────────────────────────────

    def behaviour_nodes(self, names) -> List[str]:
        """
        Return which of ``names`` are on the graph, fully qualified.

        The behaviour page starts trees itself, but a tree started from a
        terminal is just as real and would fight the one the page starts
        for ``/goal_pose`` and ``/cmd_vel``.  This is how the page knows
        about it: the graph is the only common ground between a run this
        node spawned and a run somebody else did.
        """
        wanted = set(names or ())
        if not wanted:
            return []
        try:
            found = self.get_node_names_and_namespaces()
        except Exception:  # pragma: no cover - rmw-dependent
            return []
        return [
            "{}/{}".format(namespace.rstrip("/"), name)
            for name, namespace in found if name in wanted
        ]

    # ── LED state ────────────────────────────────────────────────────────

    def _poll_led(self) -> None:
        """
        Ask the LED controller what it is currently showing.

        There is no LED state topic anywhere in the workspace -- the
        Arduino Nano is only ever asked -- so this is the only way to
        show it, and it costs a serial exchange each time.  Hence the
        cache, the slow period, and the ``_led_pending`` guard: a reply
        that never arrives must not queue a second request behind it.
        """
        if self._led_pending:
            return
        if not self._led_client.service_is_ready():
            with self._led_lock:
                self._led_error = "The LED service is not running"
            return

        self._led_pending = True
        future = self._led_client.call_async(GetLedStatus.Request())
        future.add_done_callback(self._on_led_response)

    def _on_led_response(self, future) -> None:
        """Cache one LED reading, or the reason there is not one."""
        self._led_pending = False
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - transport failure
            with self._led_lock:
                self._led_error = "LED read failed: {}".format(exc)
            return

        corners = {
            corner: {
                "mode": int(getattr(response, "{}_mode".format(corner))),
                "color": int(getattr(response, "{}_color".format(corner))),
            }
            for corner in led_enums.LED_CORNERS
        }
        with self._led_lock:
            # The LED node answers a bad checksum or a silent Arduino
            # with -1 in every field. That is a failed read, not a state
            # the strips can be in, so it must not overwrite the last
            # good reading.
            if all(values["mode"] < 0 for values in corners.values()):
                self._led_error = "The LED controller did not answer over serial"
                return
            self._led = corners
            self._led_stamp = time.monotonic()
            self._led_error = None

    def led_state(self) -> dict:
        """Return the cached LED reading, described for display."""
        with self._led_lock:
            corners = ({key: dict(value) for key, value in self._led.items()}
                       if self._led else None)
            stamp = self._led_stamp
            error = self._led_error

        state = {
            "service": self._led_service,
            "polling": self._led_poll_period > 0.0,
            "corners": None,
            "summary": None,
            "age_s": None,
            "stale": True,
            "error": error,
        }
        if corners and stamp is not None:
            age = time.monotonic() - stamp
            state.update({
                "corners": {corner: led_enums.describe_corner(values)
                            for corner, values in corners.items()},
                "summary": led_enums.describe_corners(corners),
                "age_s": round(age, 1),
                "stale": age > LED_STALE_AFTER,
            })
        return state

    # ── movement state ───────────────────────────────────────────────────

    def _on_cmd_vel(self, message: Twist) -> None:
        """Cache the latest commanded body velocity."""
        with self._motion_lock:
            self._commanded = (
                message.linear.x, message.linear.y, message.angular.z,
                time.monotonic())

    def _on_odom(self, data) -> None:
        """
        Decimate odometry before paying to deserialise it.

        ``odom`` runs at 50 Hz and the page reads once a second, so all
        but a handful of those messages would be decoded only to be
        overwritten.  The subscription is therefore raw and the bytes are
        decoded at most every ``ODOM_DECIMATE`` seconds -- the same
        reasoning that makes the rate-monitor subscriptions raw, taken
        one step further because here the payload is actually wanted.
        """
        now = time.monotonic()
        if now - self._odom_decoded_at < ODOM_DECIMATE:
            return
        self._odom_decoded_at = now

        try:
            message = deserialize_message(data, Odometry)
        except Exception:  # pragma: no cover - malformed payload
            return

        # REP-105: the twist in Odometry is already in the child frame,
        # so it is the body velocity with no transform needed.
        twist = message.twist.twist
        with self._motion_lock:
            self._measured = (
                twist.linear.x, twist.linear.y, twist.angular.z, now)

    def _motion_slot(self, sample, topic, stale_after, stale_state, now):
        """Describe one velocity source, flagging staleness and absence."""
        if not topic:
            slot = motion.idle(motion.UNKNOWN)
            slot.update({"topic": "", "available": False, "age_s": None,
                         "stale": True, "disabled": True})
            return slot
        if sample is None:
            slot = motion.idle(motion.UNKNOWN)
            slot.update({"topic": topic, "available": False, "age_s": None,
                         "stale": True, "disabled": False})
            return slot

        vx, vy, wz, stamp = sample
        age = now - stamp
        stale = age > stale_after
        # A stale sample says nothing about *now*, so the numbers are
        # kept for reference but the state falls back rather than
        # claiming the robot is still doing whatever it last did.
        slot = motion.describe_motion(vx, vy, wz)
        if stale:
            slot["state"] = stale_state
            slot["moving"] = False
        slot.update({"topic": topic, "available": True,
                     "age_s": round(age, 2), "stale": stale, "disabled": False})
        return slot

    def motion_state(self) -> dict:
        """
        Return what the robot is being told to do and what it is doing.

        Both, not one: a healthy ``cmd_vel`` carrying a forward command
        while odometry reads zero is exactly the failure this is worth
        showing, and either alone would hide it.
        """
        now = time.monotonic()
        with self._motion_lock:
            commanded = self._commanded
            measured = self._measured

        commanded_slot = self._motion_slot(
            commanded, self._cmd_vel_topic, COMMAND_STALE_AFTER,
            motion.NO_COMMAND, now)
        measured_slot = self._motion_slot(
            measured, self._odom_topic, ODOM_STALE_AFTER, motion.UNKNOWN, now)

        if measured_slot["available"] and not measured_slot["stale"]:
            state, source = measured_slot["state"], "measured"
        elif commanded_slot["available"] and not commanded_slot["stale"]:
            state, source = commanded_slot["state"], "commanded"
        else:
            state, source = motion.UNKNOWN, "none"

        return {
            "state": state,
            "source": source,
            "commanded": commanded_slot,
            "measured": measured_slot,
        }

    # ── joystick node state ──────────────────────────────────────────────

    def _on_joy(self, message: Joy) -> None:
        """Cache the latest controller frame for the live readout."""
        with self._joy_lock:
            self._latest_joy = {
                "axes": [round(float(value), 3) for value in message.axes],
                "buttons": [int(value) for value in message.buttons],
                "received_monotonic": time.monotonic(),
            }

    def _on_active_profile(self, message: String) -> None:
        """Cache the profile the joy node reports as active."""
        with self._joy_lock:
            self._active_profile = message.data

    def _on_estop(self, message: Bool) -> None:
        """Cache the joy node's e-stop latch state."""
        with self._joy_lock:
            self._estop = bool(message.data)

    def joy_state(self) -> dict:
        """Return the cached controller frame and joy-node status."""
        with self._joy_lock:
            latest = dict(self._latest_joy) if self._latest_joy else None
            active = self._active_profile
            estop = self._estop

        state = {
            "connected": False,
            "axes": [],
            "buttons": [],
            "age_s": None,
            "active_profile": active,
            "estop": estop,
            "joy_topic": self._joy_topic,
            "joy_node": self._joy_node,
        }
        if latest:
            age = time.monotonic() - latest.pop("received_monotonic")
            state.update({
                "connected": age < 1.0,
                "axes": latest["axes"],
                "buttons": latest["buttons"],
                "age_s": round(age, 2),
            })
        return state

    # ── service proxying ─────────────────────────────────────────────────

    def _call(self, client, request, timeout: float = SERVICE_TIMEOUT):
        """
        Call a service from a Flask thread and wait for the result.

        See the module docstring for why this cannot simply spin.
        """
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=1.0):
                return None

        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        # Guard the race where the call completed before the callback was
        # registered, which would otherwise wait out the whole timeout.
        if future.done():
            done.set()

        if not done.wait(timeout):
            return None
        try:
            return future.result()
        except Exception as exc:  # pragma: no cover - transport failure
            self.get_logger().warn("Service call failed: {}".format(exc))
            return None

    def reload_profile(self):
        """Ask the joy node to re-read its profile. Returns (ok, message)."""
        result = self._call(self._reload_client, Trigger.Request())
        if result is None:
            return False, "The joystick node did not respond (is it running?)"
        return bool(result.success), result.message

    def clear_estop(self):
        """Ask the joy node to release the e-stop latch."""
        result = self._call(self._clear_estop_client, Trigger.Request())
        if result is None:
            return False, "The joystick node did not respond (is it running?)"
        return bool(result.success), result.message

    def select_profile(self, name: str):
        """Set the joy node's ``profile`` parameter. Returns (ok, message)."""
        request = SetParameters.Request()
        parameter = Parameter()
        parameter.name = "profile"
        parameter.value = ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=str(name))
        request.parameters = [parameter]

        result = self._call(self._set_params_client, request)
        if result is None:
            return False, "The joystick node did not respond (is it running?)"
        if not result.results:
            return False, "The joystick node returned no result"

        outcome = result.results[0]
        if outcome.successful:
            return True, "Profile set to '{}'".format(name)
        return False, outcome.reason or "The joystick node rejected the profile"


def create_node(monitor_settings=None, **settings):
    """Initialise rclpy and build a :class:`WebNode`."""
    if not rclpy.ok():
        rclpy.init()
    return WebNode(monitor_settings=monitor_settings, **settings)
