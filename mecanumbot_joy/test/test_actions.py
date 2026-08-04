"""
Tests for the action executor.

Feeds synthetic joy frames through the executor and checks what comes
out.  No ROS graph and no gamepad, which is the point of keeping the
executor free of rclpy.
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mecanumbot_joy.actions import (  # noqa: E402
    ActionExecutor,
    make_simple_profile,
    unbound_actions,
)
from mecanumbot_joy.layout import resolve_controls  # noqa: E402
from mecanumbot_joy.profile import load_profile  # noqa: E402

PROFILE_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "mecanumbot_description", "config", "joystick")

#: Index of each xbox360 button, matching the shipped profile.
A, B, X, Y, LB, RB, BACK, START, GUIDE, LS, RS = range(11)

#: A neutral xbox360 frame: sticks centred, triggers at rest (+1).
NEUTRAL_AXES = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]


@pytest.fixture
def profile():
    """Return the shipped xbox360 profile."""
    return load_profile(os.path.join(PROFILE_DIR, "xbox360.yaml"))


@pytest.fixture
def executor(profile):
    """Build a fresh executor on the xbox360 profile."""
    return ActionExecutor(profile)


def frame(profile, axes=None, pressed=()):
    """Build a resolved control state from axis values and pressed buttons."""
    values = list(axes if axes is not None else NEUTRAL_AXES)
    buttons = [0] * 11
    for index in pressed:
        buttons[index] = 1
    return resolve_controls(values, buttons, profile)


def settle(executor, ticks=200):
    """Tick until the ramp has converged, returning the last intents."""
    intents = None
    for _ in range(ticks):
        intents = executor.tick()
    return intents


# ── ramping ──────────────────────────────────────────────────────────────

def test_make_simple_profile_steps_towards_target():
    """The ramp approaches without overshooting."""
    assert make_simple_profile(0.0, 1.0, 0.25) == 0.25
    assert make_simple_profile(0.9, 1.0, 0.25) == 1.0
    assert make_simple_profile(1.0, 0.0, 0.25) == 0.75
    assert make_simple_profile(0.1, 0.0, 0.25) == 0.0


def test_zero_step_disables_ramping():
    """A profile can opt out of ramping entirely."""
    assert make_simple_profile(0.0, 1.0, 0.0) == 1.0
    assert make_simple_profile(0.0, 1.0, -1.0) == 1.0


def test_ramp_reaches_full_speed_in_the_expected_number_of_ticks(profile, executor):
    """Time to full speed follows from max_lin_vel and lin_step."""
    executor.on_frame(frame(profile, [0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]))

    target = profile.limits["max_lin_vel"]
    step = profile.limits["lin_step"]
    expected = math.ceil(target / step)

    for _ in range(expected):
        intents = executor.tick()
    assert intents.twist[0] == pytest.approx(target)


def test_ramp_never_overshoots(profile, executor):
    """No tick may exceed the configured maximum."""
    executor.on_frame(frame(profile, [0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]))
    limit = profile.limits["max_lin_vel"]
    for _ in range(200):
        assert executor.tick().twist[0] <= limit + 1e-9


# ── the aliasing regression ──────────────────────────────────────────────

def test_returned_twist_is_not_aliased_to_internal_state(profile, executor):
    """
    Mutating the returned intents must not reach the executor.

    The previous teleop node did ``self.control_vel = self.target_vel``,
    aliasing the two lists, so clamping the output silently clamped the
    target as well and the limit became sticky.  This is the regression
    test for that bug.
    """
    executor.on_frame(frame(profile, [0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]))
    intents = settle(executor)
    before = intents.twist

    mutated = list(intents.twist)
    mutated[0] = 999.0
    assert executor.tick().twist == pytest.approx(before)


def test_each_tick_returns_a_fresh_tuple(profile, executor):
    """Consecutive results must be independent objects."""
    executor.on_frame(frame(profile, [0.0, 0.5, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]))
    first = executor.tick().twist
    second = executor.tick().twist
    assert first is not second


# ── deadzone and axis mapping ────────────────────────────────────────────

def test_deadzone_zeroes_small_input(profile, executor):
    """Stick drift below the deadzone must not creep the robot."""
    small = profile.limits["deadzone"] / 2.0
    executor.on_frame(frame(profile, [small, small, 1.0, small, 0.0, 1.0, 0.0, 0.0]))
    assert settle(executor).twist == (0.0, 0.0, 0.0)


def test_axes_map_to_the_expected_velocity_components(profile, executor):
    """Left stick drives and strafes, right stick turns."""
    executor.on_frame(frame(profile, [1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 0.0, 0.0]))
    intents = settle(executor)
    assert intents.twist[0] == pytest.approx(profile.limits["max_lin_vel"])
    assert intents.twist[1] == pytest.approx(profile.limits["max_lin_vel"])
    assert intents.twist[2] == pytest.approx(profile.limits["max_ang_vel"])


def test_invert_flips_an_axis(profile):
    """The invert flag is honoured."""
    profile.bindings["drive"]["invert"] = True
    executor = ActionExecutor(profile)
    executor.on_frame(frame(profile, [0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]))
    assert settle(executor).twist[0] == pytest.approx(-profile.limits["max_lin_vel"])


# ── e-stop ───────────────────────────────────────────────────────────────

def test_estop_latches_and_blocks_the_axes(profile, executor):
    """Once engaged, stick input is ignored until it is cleared."""
    executor.on_frame(frame(profile, [0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]))
    settle(executor)

    executor.on_frame(frame(profile, [0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                            pressed=[BACK]))
    intents = executor.tick()
    assert intents.estop is True
    assert intents.estop_changed is True
    assert intents.twist == (0.0, 0.0, 0.0)

    executor.on_frame(frame(profile, [0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]))
    assert settle(executor).twist == (0.0, 0.0, 0.0)


def test_estop_is_edge_triggered_not_level(profile, executor):
    """Holding the button must not toggle repeatedly at the joy rate."""
    held = frame(profile, pressed=[BACK])
    executor.on_frame(held)
    assert executor.estop is True
    for _ in range(10):
        executor.on_frame(held)
    assert executor.estop is True


def test_estop_toggles_on_a_second_press(profile, executor):
    """Releasing and pressing again clears the latch."""
    executor.on_frame(frame(profile, pressed=[BACK]))
    executor.on_frame(frame(profile))
    executor.on_frame(frame(profile, pressed=[BACK]))
    assert executor.estop is False


def test_clear_estop_releases_the_latch(profile, executor):
    """The web page's clear button has something to call."""
    executor.on_frame(frame(profile, pressed=[BACK]))
    assert executor.clear_estop() is True
    assert executor.estop is False
    assert executor.clear_estop() is False


def test_estop_changed_is_consumed_by_one_tick(profile, executor):
    """The node publishes the change once, not forever."""
    executor.on_frame(frame(profile, pressed=[BACK]))
    assert executor.tick().estop_changed is True
    assert executor.tick().estop_changed is False


# ── accessories ──────────────────────────────────────────────────────────

def test_neck_holds_ramp_and_clamp(profile, executor):
    """Holding neck_down walks the neck to its lower limit and stops."""
    held = frame(profile, pressed=[A])
    for _ in range(500):
        executor.on_frame(held)
        intents = executor.tick()
    assert intents.accessory[0] == pytest.approx(profile.neck["min"])


def test_neck_up_clamps_at_the_maximum(profile, executor):
    """The neck cannot be driven past its travel."""
    held = frame(profile, pressed=[Y])
    for _ in range(500):
        executor.on_frame(held)
        intents = executor.tick()
    assert intents.accessory[0] == pytest.approx(profile.neck["max"])


def test_neck_preset_jumps(profile, executor):
    """The preset restores the old jump-to-look-ahead behaviour."""
    for _ in range(20):
        executor.on_frame(frame(profile, pressed=[A]))
        executor.tick()
    executor.on_frame(frame(profile, pressed=[LS]))
    assert executor.tick().accessory[0] == pytest.approx(8.6)


def test_gripper_open_and_close_use_the_documented_midpoints(profile, executor):
    """The values match what the previous teleop node commanded."""
    gripper = profile.gripper
    open_left = (gripper["max"] + gripper["front"]) / 2.0
    open_right = (gripper["min"] + gripper["front"]) / 2.0

    executor.on_frame(frame(profile, pressed=[X]))
    intents = executor.tick()
    assert intents.accessory[1] == pytest.approx(open_left)
    assert intents.accessory[2] == pytest.approx(open_right)

    executor.on_frame(frame(profile))
    executor.on_frame(frame(profile, pressed=[B]))
    intents = executor.tick()
    assert intents.accessory[1] == pytest.approx(open_right)
    assert intents.accessory[2] == pytest.approx(open_left)


def test_accessory_changed_only_when_it_changes(profile, executor):
    """Positions are republished on change, not at the drive rate."""
    executor.tick()                      # consume the initial pose
    assert executor.tick().accessory_changed is False

    executor.on_frame(frame(profile, pressed=[X]))
    assert executor.tick().accessory_changed is True
    assert executor.tick().accessory_changed is False


def test_gripper_positions_stay_inside_travel(profile, executor):
    """Repeated commands cannot walk the grippers out of range."""
    for _ in range(50):
        executor.on_frame(frame(profile, pressed=[X]))
        executor.on_frame(frame(profile))
        intents = executor.tick()
    assert profile.gripper["min"] <= intents.accessory[1] <= profile.gripper["max"]
    assert profile.gripper["min"] <= intents.accessory[2] <= profile.gripper["max"]


# ── LED ──────────────────────────────────────────────────────────────────

def test_led_preset_is_emitted_once(profile, executor):
    """A press queues one LED command, consumed by the next tick."""
    executor.on_frame(frame(profile, pressed=[START]))
    intents = executor.tick()
    assert intents.led is not None
    assert set(intents.led) == {"fl", "fr", "bl", "br"}
    assert executor.tick().led is None


# ── controller loss ──────────────────────────────────────────────────────

def test_joy_loss_stops_the_wheels_but_keeps_the_pose(profile, executor):
    """An unplugged pad must not leave the robot driving."""
    executor.on_frame(frame(profile, [0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                            pressed=[X]))
    intents = settle(executor)
    assert intents.twist[0] > 0.0
    pose = intents.accessory

    executor.on_joy_lost()
    intents = executor.tick()
    assert intents.twist == (0.0, 0.0, 0.0)
    assert intents.accessory == pose


# ── profile swapping ─────────────────────────────────────────────────────

def test_set_profile_preserves_the_estop_latch(profile, executor):
    """Reloading a profile must not silently release an engaged e-stop."""
    executor.on_frame(frame(profile, pressed=[BACK]))
    assert executor.estop is True
    executor.set_profile(load_profile(os.path.join(PROFILE_DIR, "generic.yaml")))
    assert executor.estop is True


def test_set_profile_stops_the_wheels(profile, executor):
    """A swap is not a moment to keep driving through."""
    executor.on_frame(frame(profile, [0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]))
    settle(executor)
    executor.set_profile(profile)
    assert executor.tick().twist == (0.0, 0.0, 0.0)


def test_unbound_actions_are_reported(profile):
    """Startup logging can warn about anything left unbound."""
    assert unbound_actions(profile) == ()
    del profile.bindings["estop"]
    assert "estop" in unbound_actions(profile)
