"""
Tests for controller layout normalisation.

Pure-Python: no ROS graph, no gamepad.  Several cases are ported from
``mecanumbot_gui/tests/test_controller_node.py``, which tested the same
behaviour when it was hardcoded in a node instead of driven by YAML.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mecanumbot_joy.layout import (  # noqa: E402
    DPAD_NAMES,
    ControlState,
    clamp,
    fingerprint,
    normalise_axis,
    read_dpad,
    resolve_controls,
    safe_axis,
    safe_button,
)
from mecanumbot_joy.profile import load_profile_dir, select_profile  # noqa: E402

PROFILE_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "mecanumbot_description", "config", "joystick")


# ── safe indexing ────────────────────────────────────────────────────────

def test_safe_axis_out_of_range_reads_neutral():
    """A profile written for a bigger pad must not crash on a smaller one."""
    assert safe_axis([0.5, -0.5], 0) == 0.5
    assert safe_axis([0.5, -0.5], 7) == 0.0
    assert safe_axis([], 0) == 0.0
    assert safe_axis([0.5], None) == 0.0


def test_safe_button_out_of_range_reads_unpressed():
    """Missing buttons read as released rather than raising."""
    assert safe_button([0, 1], 1) == 1
    assert safe_button([0, 1], 9) == 0
    assert safe_button([], 0) == 0


def test_clamp_bounds():
    """Keep values inside the interval."""
    assert clamp(5.0, 0.0, 1.0) == 1.0
    assert clamp(-5.0, 0.0, 1.0) == 0.0
    assert clamp(0.5, 0.0, 1.0) == 0.5


# ── axis normalisation ───────────────────────────────────────────────────

def test_stick_axis_passes_through_clamped():
    """A spec without a range is a stick and keeps its signed value."""
    assert normalise_axis(0.7, {"index": 0}) == pytest.approx(0.7)
    assert normalise_axis(-0.7, {"index": 0}) == pytest.approx(-0.7)
    assert normalise_axis(3.0, {"index": 0}) == 1.0
    assert normalise_axis(-3.0, {"index": 0}) == -1.0


def test_xpad_trigger_convention():
    """Xbox 360 via xpad rests at +1 and reads -1 fully pressed."""
    spec = {"index": 2, "range": [1.0, -1.0]}
    assert normalise_axis(1.0, spec) == 0.0
    assert normalise_axis(0.0, spec) == pytest.approx(0.5)
    assert normalise_axis(-1.0, spec) == 1.0


def test_generic_trigger_convention():
    """Most other pads rest at -1 and read +1 fully pressed."""
    spec = {"index": 2, "range": [-1.0, 1.0]}
    assert normalise_axis(-1.0, spec) == 0.0
    assert normalise_axis(0.0, spec) == pytest.approx(0.5)
    assert normalise_axis(1.0, spec) == 1.0


def test_both_trigger_conventions_agree_at_the_ends():
    """
    The two conventions are the same map, just mirrored.

    This is the whole reason `range` exists: the previous node carried two
    separate hardcoded formulas selected by an `if layout == 'xbox360'`.
    """
    xpad = {"range": [1.0, -1.0]}
    other = {"range": [-1.0, 1.0]}
    assert normalise_axis(1.0, xpad) == normalise_axis(-1.0, other)
    assert normalise_axis(-1.0, xpad) == normalise_axis(1.0, other)


def test_trigger_at_rest_is_positive_zero():
    """Rest must not read as -0.0, which looks wrong in the GUI."""
    value = normalise_axis(1.0, {"range": [1.0, -1.0]})
    assert value == 0.0
    assert str(value) == "0.0"


def test_degenerate_range_is_neutral():
    """A range with equal endpoints cannot divide, and reads neutral."""
    assert normalise_axis(0.5, {"range": [1.0, 1.0]}) == 0.0


# ── D-Pad ────────────────────────────────────────────────────────────────

def test_dpad_from_axes():
    """Positive X is left and positive Y is up, the xpad convention."""
    spec = {"source": "axes", "x_axis": 6, "y_axis": 7}
    axes = [0.0] * 6 + [1.0, 1.0]
    states = read_dpad(axes, [], spec)
    assert states["DPadLeft"] is True
    assert states["DPadUp"] is True
    assert states["DPadRight"] is False
    assert states["DPadDown"] is False

    axes = [0.0] * 6 + [-1.0, -1.0]
    states = read_dpad(axes, [], spec)
    assert states["DPadRight"] is True
    assert states["DPadDown"] is True


def test_dpad_from_buttons():
    """The button convention produces the same names."""
    spec = {"source": "buttons", "up": 11, "down": 12, "left": 13, "right": 14}
    buttons = [0] * 15
    buttons[11] = 1
    buttons[13] = 1
    states = read_dpad([], buttons, spec)
    assert states["DPadUp"] is True
    assert states["DPadLeft"] is True
    assert states["DPadDown"] is False


def test_both_dpad_sources_produce_identical_output():
    """Whichever way the pad reports it, the action layer sees one shape."""
    from_axes = read_dpad(
        [0.0] * 6 + [1.0, 1.0], [], {"source": "axes", "x_axis": 6, "y_axis": 7})
    buttons = [0] * 15
    buttons[11] = 1
    buttons[13] = 1
    from_buttons = read_dpad(
        [], buttons, {"source": "buttons", "up": 11, "down": 12,
                      "left": 13, "right": 14})
    assert from_axes == from_buttons


def test_dpad_below_threshold_is_not_pressed():
    """A partially deflected D-Pad axis does not latch."""
    spec = {"source": "axes", "x_axis": 6, "y_axis": 7}
    states = read_dpad([0.0] * 6 + [0.3, 0.3], [], spec)
    assert not any(states.values())


def test_missing_dpad_spec_is_all_released():
    """A profile without a D-Pad still reports the four names."""
    states = read_dpad([], [], {})
    assert set(states) == set(DPAD_NAMES)
    assert not any(states.values())


# ── fingerprinting ───────────────────────────────────────────────────────

def test_fingerprint_shape():
    """The fingerprint is just the two counts."""
    assert fingerprint([0.0] * 8, [0] * 11) == {"axes": 8, "buttons": 11}


def test_shipped_profiles_are_selected_by_fingerprint():
    """8/11 is xbox360, 8/13 is ps4, anything else falls back to generic."""
    profiles, errors = load_profile_dir(PROFILE_DIR)
    assert not errors, errors
    assert select_profile(profiles, 8, 11) == "xbox360"
    assert select_profile(profiles, 8, 13) == "ps4"
    assert select_profile(profiles, 6, 12) == "generic"
    assert select_profile(profiles, 0, 0) == "generic"


def test_profile_without_match_is_never_auto_selected():
    """generic.yaml carries no fingerprint, so it is fallback-only."""
    profiles, _ = load_profile_dir(PROFILE_DIR)
    assert profiles["generic"].match is None


# ── resolving a whole frame ──────────────────────────────────────────────

def test_resolve_controls_against_shipped_xbox360():
    """A full frame resolves into the names the profile declares."""
    profiles, _ = load_profile_dir(PROFILE_DIR)
    profile = profiles["xbox360"]

    axes = [0.25, 0.5, 1.0, -0.75, 0.0, -1.0, 1.0, 0.0]
    buttons = [0] * 11
    buttons[3] = 1  # Y

    state = resolve_controls(axes, buttons, profile)
    assert state.axis("left_x") == pytest.approx(0.25)
    assert state.axis("left_y") == pytest.approx(0.5)
    assert state.axis("right_x") == pytest.approx(-0.75)
    assert state.axis("lt") == 0.0     # at rest
    assert state.axis("rt") == 1.0     # fully pressed
    assert state.button("Y") == 1
    assert state.button("A") == 0
    assert state.button("DPadLeft") == 1


def test_control_state_defaults_for_unknown_names():
    """Asking for a control the profile lacks is neutral, not an error."""
    state = ControlState()
    assert state.axis("nope") == 0.0
    assert state.button("nope") == 0
