"""
The ROS side of the web GUI: rate monitoring and joy-node control.

Runs as a normal rclpy node under a ``MultiThreadedExecutor``.  Flask
serves from a daemon thread and reaches in here through the accessors
below, all of which are lock-guarded.

Two details are load bearing and easy to get wrong:

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
"""

import threading
import time
from functools import partial
from typing import Dict, List, Optional

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .rate_monitor import RateMonitor, TopicSpec

#: How often the graph is rescanned for topics that have appeared.
DISCOVERY_PERIOD = 2.0

#: Default blocking timeout for a service call made from a Flask thread.
SERVICE_TIMEOUT = 8.0


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
    """Measures topic rates and proxies control of the joystick node."""

    def __init__(self, monitor_settings=None, joy_node: str = "", joy_topic: str = ""):
        """Create the node, its monitor and its clients to the joy node."""
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
            with self._rate_lock:
                self._monitor.set_publisher_count(topic, count)

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

    def _on_monitored(self, topic: str, message) -> None:
        """Record a raw message receipt. The payload is never deserialised."""
        del message
        now = time.monotonic()
        with self._rate_lock:
            self._monitor.record(topic, now)

    def diagnostics(self) -> dict:
        """Return the current rate readings, for the diagnostics API."""
        now = time.monotonic()
        with self._rate_lock:
            readings = self._monitor.read(now)
            summary = self._monitor.summary(readings)
        return {
            "topics": [reading.as_dict() for reading in readings],
            "summary": summary,
            "window_seconds": self._monitor.window_seconds,
            "stale_after": self._monitor.stale_after,
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
            }
            for spec in specs if spec is not None
        ]

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


def create_node(monitor_settings=None, joy_node: str = "", joy_topic: str = ""):
    """Initialise rclpy and build a :class:`WebNode`."""
    if not rclpy.ok():
        rclpy.init()
    return WebNode(
        monitor_settings=monitor_settings, joy_node=joy_node, joy_topic=joy_topic)
