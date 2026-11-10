"""
Browser teleop: limits, the dead-man lease and LED requests, without rclpy.

The teleop page drives the robot from a browser, which is the least
reliable control link in the workspace: Wi-Fi drops, a tab is backgrounded,
a laptop lid is shut mid-drive.  The rules that keep that safe are
arithmetic, so they live here, where they can be tested without a graph,
for the same reason :mod:`.motion` does.

**Velocity is a lease, not a setting.**  The page re-sends the command
several times a second while the operator holds a control, and each send
renews a short lease.  The node publishes the leased command at a fixed
rate; when the lease runs out it publishes zeros for a few ticks and then
goes *silent*.  Silent rather than zero, because Nav2, the joystick node
and the behaviour trees all publish ``/cmd_vel`` too, and a page left open
on a laptop must not hold the robot still against them.  The few zeros are
there because ``mecanumbot_io_node`` keeps obeying the last command for
``cmd_vel_timeout`` (0.5 s) -- the zeros stop it now rather than then.

**Accessories and LEDs are positions, not streams.**  The io node holds the
last accessory position it was sent and the LED controller holds its last
state, so both are sent once per change.

The limits are ``mecanumbot_joy``'s, so the page cannot drive the robot
faster, or the neck and grabbers further, than the joystick can.
"""

import math

from . import led_enums

try:  # The joystick package owns these; fall back if it is not on the path.
    from mecanumbot_joy.profile import DEFAULT_GRIPPER, DEFAULT_LIMITS, DEFAULT_NECK
except ImportError:  # pragma: no cover - depends on the checkout
    DEFAULT_LIMITS = {"max_lin_vel": 0.234, "max_ang_vel": 1.092}
    DEFAULT_NECK = {"min": 2.0, "max": 8.6, "default": 8.6, "step": 0.06}
    DEFAULT_GRIPPER = {"min": 1.6, "front": 5.12, "max": 8.54, "default": 5.12}

#: Seconds a velocity command stays in force without being renewed.  The
#: page renews it every 100 ms, so this rides out a few lost requests
#: while still stopping well inside a second of the link going away.
LEASE_SECONDS = 0.4

#: How often the node publishes a leased command (Hz).
PUBLISH_HZ = 20.0

#: Zero frames published after a lease ends, before going silent.
ZERO_TICKS = 5


class TeleopError(ValueError):
    """Raised when a teleop request cannot be turned into a command."""


def _number(payload, key, default=None):
    """Return ``payload[key]`` as a finite float, or raise TeleopError."""
    value = payload.get(key, default) if isinstance(payload, dict) else None
    if value is None:
        raise TeleopError("'{}' is required".format(key))
    if isinstance(value, bool):
        raise TeleopError("'{}' must be a number".format(key))
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise TeleopError("'{}' must be a number".format(key)) from None
    if not math.isfinite(number):
        raise TeleopError("'{}' must be finite".format(key))
    return number


def _clamp(value, low, high):
    """Clamp ``value`` into ``[low, high]``."""
    return max(low, min(high, value))


def default_limits():
    """Return the limits the page enforces, in the shape it renders from."""
    return {
        "max_lin_vel": float(DEFAULT_LIMITS["max_lin_vel"]),
        "max_ang_vel": float(DEFAULT_LIMITS["max_ang_vel"]),
        "neck": {key: float(DEFAULT_NECK[key]) for key in ("min", "max", "default")},
        "gripper": {key: float(DEFAULT_GRIPPER[key])
                    for key in ("min", "front", "max", "default")},
        "lease_seconds": LEASE_SECONDS,
    }


def parse_velocity(payload, limits=None):
    """
    Return a clamped ``(vx, vy, wz)`` from a request body.

    Clamped per axis, the same way the joystick limits each axis, rather
    than scaling the planar vector: a diagonal at full stick is then the
    same diagonal from either control surface.
    """
    limits = limits or default_limits()
    lin = limits["max_lin_vel"]
    ang = limits["max_ang_vel"]
    return (
        _clamp(_number(payload, "vx", 0.0), -lin, lin),
        _clamp(_number(payload, "vy", 0.0), -lin, lin),
        _clamp(_number(payload, "wz", 0.0), -ang, ang),
    )


def parse_accessory(payload, limits=None):
    """
    Return a clamped ``(n_pos, gl_pos, gr_pos)`` from a request body.

    Units are ``AccessMotorCmd``'s, which the io node multiplies by 100
    into Dynamixel ticks.  Every field is required: the message carries all
    three, so a missing one would silently send that servo to zero.
    """
    limits = limits or default_limits()
    neck, grip = limits["neck"], limits["gripper"]
    return (
        _clamp(_number(payload, "n_pos"), neck["min"], neck["max"]),
        _clamp(_number(payload, "gl_pos"), grip["min"], grip["max"]),
        _clamp(_number(payload, "gr_pos"), grip["min"], grip["max"]),
    )


def gripper_presets(limits=None):
    """
    Return the named grabber positions the joystick uses, as ``(gl, gr)``.

    The two grabbers mirror each other, so "open" moves them in opposite
    directions from ``front``.  Taken from ``mecanumbot_joy.actions``'
    ``gripper_open`` / ``gripper_close`` so both surfaces mean the same.
    """
    grip = (limits or default_limits())["gripper"]
    front, low, high = grip["front"], grip["min"], grip["max"]
    return {
        "open": ((high + front) / 2.0, (low + front) / 2.0),
        "front": (front, front),
        "close": ((low + front) / 2.0, (high + front) / 2.0),
    }


def parse_led(payload):
    """
    Return a ``{corner: {'mode', 'color'}}`` request from a request body.

    Accepts either ``corners`` (per corner) or a single ``mode`` / ``color``
    pair applied to all four.  Numbers outside the documented vocabulary
    are refused: the LED firmware has no defined behaviour for them.
    """
    if not isinstance(payload, dict):
        raise TeleopError("Expected a JSON object")

    if "corners" in payload:
        given = payload["corners"]
        if not isinstance(given, dict):
            raise TeleopError("'corners' must be an object")
        missing = [corner for corner in led_enums.LED_CORNERS if corner not in given]
        if missing:
            raise TeleopError("Missing corner(s): {}".format(", ".join(missing)))
        corners = {corner: given[corner] for corner in led_enums.LED_CORNERS}
    else:
        corners = {corner: payload for corner in led_enums.LED_CORNERS}

    request = {}
    for corner, values in corners.items():
        mode = _number(values, "mode")
        color = _number(values, "color")
        if mode != int(mode) or int(mode) not in led_enums.MODE_NAMES:
            raise TeleopError("{}: unknown LED mode {}".format(corner, values.get("mode")))
        if color != int(color) or int(color) not in led_enums.COLOR_NAMES:
            raise TeleopError("{}: unknown LED colour {}".format(corner, values.get("color")))
        request[corner] = {"mode": int(mode), "color": int(color)}
    return request


class VelocityLease:
    """
    Hold the browser's velocity command and decide what to publish each tick.

    Not thread-safe by itself; the node guards it with a lock, because
    Flask threads renew it while an executor timer ticks it.
    """

    def __init__(self, lease_seconds=LEASE_SECONDS, zero_ticks=ZERO_TICKS):
        """Start with no command, publishing nothing."""
        self.lease_seconds = float(lease_seconds)
        self.zero_ticks = int(zero_ticks)
        self._command = None
        self._renewed_at = None
        self._zeros_left = 0

    def renew(self, command, now):
        """Set the command and restart the lease."""
        self._command = tuple(float(value) for value in command)
        self._renewed_at = float(now)
        self._zeros_left = self.zero_ticks

    def stop(self):
        """End the lease now; the next ticks publish the zero tail."""
        if self._command is not None:
            self._command = None
            self._renewed_at = None
            self._zeros_left = self.zero_ticks

    def active(self, now):
        """Report whether a command is currently in force."""
        return (self._command is not None
                and now - self._renewed_at <= self.lease_seconds)

    def tick(self, now):
        """
        Return the ``(vx, vy, wz)`` to publish now, or None for silence.

        A lapsed lease is treated exactly like an explicit stop.
        """
        if self._command is not None and not self.active(now):
            self.stop()
        if self._command is not None:
            return self._command
        if self._zeros_left > 0:
            self._zeros_left -= 1
            return (0.0, 0.0, 0.0)
        return None

    def state(self, now):
        """Return the lease for the page: what is held and for how long."""
        active = self.active(now)
        return {
            "active": active,
            "command": list(self._command) if active else None,
            "age_s": round(now - self._renewed_at, 2) if active else None,
            "lease_seconds": self.lease_seconds,
        }
