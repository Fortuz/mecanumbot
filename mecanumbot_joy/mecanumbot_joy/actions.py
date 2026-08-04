"""
The fixed action vocabulary and the state machine that executes it.

Pure Python -- no ``rclpy``.  Feed it resolved :class:`ControlState`
frames and clock ticks, get back :class:`Intents` describing what the
robot should do.  The ROS node above is a thin adapter that turns those
intents into ``Twist`` / ``AccessMotorCmd`` / ``SetLedStatus``.

Two entry points, deliberately split by rate:

``on_frame(state)``
    Called for every ``/joy`` message.  Discrete actions live here so a
    quick button tap between publish ticks can never be dropped.

``tick(dt)``
    Called on the publish timer.  Continuous behaviour lives here --
    velocity ramping and hold-to-move neck control -- so their rate is a
    property of the configured ``publish_hz`` rather than of however fast
    the pad happens to report.
"""

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from .layout import ControlState, clamp
from .profile import AXIS_ACTIONS, Profile


@dataclass
class Intents:
    """What the robot should do, as produced by one :meth:`ActionExecutor.tick`."""

    twist: Tuple[float, float, float]
    accessory: Tuple[float, float, float]
    accessory_changed: bool = False
    led: Optional[Dict] = None
    estop: bool = False
    estop_changed: bool = False

    @property
    def twist_is_zero(self) -> bool:
        """Report whether every velocity component is zero."""
        return all(component == 0.0 for component in self.twist)


def make_simple_profile(output: float, target: float, slop: float) -> float:
    """
    Step ``output`` towards ``target`` by at most ``slop``.

    Carried over from the previous teleop node, where it was defined but
    never called -- the ramping block was commented out, which silently
    made its ``smoothing.*`` parameters dead.  A non-positive ``slop``
    means "no ramp", so a profile can still opt out.
    """
    if slop <= 0.0:
        return target
    if target > output:
        return min(target, output + slop)
    if target < output:
        return max(target, output - slop)
    return target


class ActionExecutor:
    """Applies a :class:`Profile`'s bindings to a stream of joy frames."""

    def __init__(self, profile: Profile):
        """Start executing ``profile``, with everything at its neutral pose."""
        self._profile = profile
        self._latest: Optional[ControlState] = None
        self._prev_buttons: Dict[str, int] = {}

        self._velocity = [0.0, 0.0, 0.0]
        self._estop = False
        self._estop_changed = False
        self._pending_led: Optional[Dict] = None

        self._neck = float(profile.neck["default"])
        self._gl = float(profile.gripper["default"])
        self._gr = float(profile.gripper["default"])
        self._accessory_dirty = True

    # ── profile lifecycle ────────────────────────────────────────────────

    @property
    def profile(self) -> Profile:
        """Return the profile currently being executed."""
        return self._profile

    def set_profile(self, profile: Profile) -> None:
        """
        Swap in a new profile, keeping the physical pose and e-stop latch.

        Accessory positions are preserved rather than reset: the neck and
        grippers are physically where they are, and snapping them to a new
        profile's defaults on a reload would be a surprise movement.
        """
        self._profile = profile
        self._prev_buttons = {}
        self._latest = None
        self._velocity = [0.0, 0.0, 0.0]
        self._neck = clamp(self._neck, profile.neck["min"], profile.neck["max"])
        self._gl = clamp(self._gl, profile.gripper["min"], profile.gripper["max"])
        self._gr = clamp(self._gr, profile.gripper["min"], profile.gripper["max"])
        self._accessory_dirty = True

    # ── e-stop ───────────────────────────────────────────────────────────

    @property
    def estop(self) -> bool:
        """Report whether the e-stop latch is engaged."""
        return self._estop

    def clear_estop(self) -> bool:
        """Release the e-stop latch. Returns True if it had been engaged."""
        if not self._estop:
            return False
        self._estop = False
        self._estop_changed = True
        return True

    def _toggle_estop(self) -> None:
        """Flip the e-stop latch and stop the wheels immediately if engaging."""
        self._estop = not self._estop
        self._estop_changed = True
        if self._estop:
            self._velocity = [0.0, 0.0, 0.0]

    # ── input ────────────────────────────────────────────────────────────

    def on_frame(self, state: ControlState) -> None:
        """Consume one joy frame: run edge detection and discrete actions."""
        self._latest = state

        for action, spec in self._profile.bindings.items():
            if spec.get("control") != "button":
                continue
            name = spec.get("button")
            if name is None:
                continue

            trigger = spec.get("trigger", "press")
            now = state.button(name)
            was = self._prev_buttons.get(name, 0)

            # An e-stop bound to `hold` would toggle at the joy rate, which
            # is unusable, so it is always edge-triggered.
            edge_only = trigger == "press" or action == "estop"
            fired = (now == 1 and was == 0) if edge_only else (now == 1)

            # Hold-to-ramp neck control is continuous and belongs in tick().
            if action in ("neck_up", "neck_down") and trigger == "hold":
                fired = False

            if fired:
                self._apply_button_action(action, spec)

        for name in self._profile.buttons:
            self._prev_buttons[name] = state.button(name)
        for name in state.buttons:
            self._prev_buttons[name] = state.buttons[name]

    def on_joy_lost(self) -> None:
        """
        Handle the controller going silent: stop, but hold the pose.

        The previous node had no equivalent and would hold the last
        commanded velocity indefinitely if the pad was unplugged mid-drive.
        """
        self._latest = None
        self._velocity = [0.0, 0.0, 0.0]

    # ── discrete actions ─────────────────────────────────────────────────

    def _apply_button_action(self, action: str, spec: Dict) -> None:
        """Execute one discrete action from the fixed vocabulary."""
        neck = self._profile.neck
        gripper = self._profile.gripper

        if action == "neck_up":
            self._set_neck(self._neck + neck["step"])
        elif action == "neck_down":
            self._set_neck(self._neck - neck["step"])
        elif action == "neck_preset":
            self._set_neck(float(spec.get("value", neck["default"])))
        elif action == "gripper_open":
            # The two grippers mirror each other: opening means the left
            # travels towards max and the right towards min.  Midpoints
            # preserved from the previous teleop node.
            self._set_gripper(
                (gripper["max"] + gripper["front"]) / 2.0,
                (gripper["min"] + gripper["front"]) / 2.0,
            )
        elif action == "gripper_close":
            self._set_gripper(
                (gripper["min"] + gripper["front"]) / 2.0,
                (gripper["max"] + gripper["front"]) / 2.0,
            )
        elif action == "estop":
            self._toggle_estop()
        elif action == "led_preset":
            preset = self._profile.led_presets.get(spec.get("preset"))
            if preset is not None:
                self._pending_led = preset

    def _set_neck(self, value: float) -> None:
        """Move the neck target, clamped to the profile's range."""
        clamped = clamp(value, self._profile.neck["min"], self._profile.neck["max"])
        if clamped != self._neck:
            self._neck = clamped
            self._accessory_dirty = True

    def _set_gripper(self, left: float, right: float) -> None:
        """Move both gripper targets, clamped to the profile's range."""
        low = self._profile.gripper["min"]
        high = self._profile.gripper["max"]
        left = clamp(left, low, high)
        right = clamp(right, low, high)
        if left != self._gl or right != self._gr:
            self._gl = left
            self._gr = right
            self._accessory_dirty = True

    # ── continuous actions ───────────────────────────────────────────────

    def _axis_value(self, action: str) -> float:
        """Read one bound axis, with deadzone and inversion applied."""
        spec = self._profile.bindings.get(action)
        if not spec or self._latest is None:
            return 0.0

        value = self._latest.axis(spec.get("axis"))
        if abs(value) < self._profile.limits["deadzone"]:
            return 0.0
        if spec.get("invert"):
            value = -value
        return value

    def _ramp_neck(self) -> None:
        """Apply hold-to-move neck bindings for one tick."""
        if self._latest is None:
            return
        for action, delta in (("neck_up", 1.0), ("neck_down", -1.0)):
            spec = self._profile.bindings.get(action)
            if not spec or spec.get("trigger", "press") != "hold":
                continue
            if self._latest.button(spec.get("button")) == 1:
                self._set_neck(self._neck + delta * self._profile.neck["step"])

    def tick(self, dt: float = 0.0) -> Intents:
        """
        Advance continuous state by one publish tick and return intents.

        The returned tuples are freshly built every call.  The previous
        node aliased its output list to its target list, so clamping the
        output silently clamped the target too and the limit became
        sticky; there is a regression test for exactly that.
        """
        del dt  # ramp steps are per-tick by definition; see profile limits

        self._ramp_neck()

        limits = self._profile.limits
        if self._estop:
            targets = (0.0, 0.0, 0.0)
        else:
            targets = (
                self._axis_value("drive") * limits["max_lin_vel"],
                self._axis_value("strafe") * limits["max_lin_vel"],
                self._axis_value("turn") * limits["max_ang_vel"],
            )

        steps = (limits["lin_step"], limits["lin_step"], limits["ang_step"])
        bounds = (limits["max_lin_vel"], limits["max_lin_vel"], limits["max_ang_vel"])
        for index in range(3):
            value = make_simple_profile(self._velocity[index], targets[index], steps[index])
            self._velocity[index] = clamp(value, -bounds[index], bounds[index])

        intents = Intents(
            twist=(self._velocity[0], self._velocity[1], self._velocity[2]),
            accessory=(self._neck, self._gl, self._gr),
            accessory_changed=self._accessory_dirty,
            led=self._pending_led,
            estop=self._estop,
            estop_changed=self._estop_changed,
        )

        self._accessory_dirty = False
        self._pending_led = None
        self._estop_changed = False
        return intents


def unbound_actions(profile: Profile) -> Tuple[str, ...]:
    """
    Return vocabulary entries this profile leaves unbound.

    Not an error -- a profile may legitimately omit the LED preset, say --
    but worth logging at startup, since a silently unbound ``estop`` is
    something the operator should know about before driving.
    """
    from .profile import VOCABULARY

    return tuple(action for action in VOCABULARY if action not in profile.bindings)


def axis_action_names() -> Tuple[str, ...]:
    """Return the continuous action names, for callers building UI."""
    return AXIS_ACTIONS
