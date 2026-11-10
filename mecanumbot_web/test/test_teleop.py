"""
Tests for the teleop limits, the dead-man lease and LED request parsing.

Pure Python, no ROS graph: the safety of the browser teleop rests on the
lease, so it is tested here rather than through a node.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mecanumbot_web import teleop  # noqa: E402


# ── the lease ────────────────────────────────────────────────────────────

def test_nothing_is_published_before_any_command():
    """An open page with nobody driving is not a /cmd_vel publisher."""
    lease = teleop.VelocityLease()
    assert lease.tick(0.0) is None


def test_a_renewed_command_is_published():
    lease = teleop.VelocityLease(lease_seconds=0.4)
    lease.renew((0.1, 0.0, 0.2), now=0.0)
    assert lease.tick(0.1) == (0.1, 0.0, 0.2)
    lease.renew((0.1, 0.0, 0.2), now=0.3)
    assert lease.tick(0.6) == (0.1, 0.0, 0.2)


def test_a_lapsed_lease_publishes_zeros_then_falls_silent():
    """The link dropping must stop the wheels, and then stop holding them."""
    lease = teleop.VelocityLease(lease_seconds=0.4, zero_ticks=3)
    lease.renew((0.2, 0.1, 0.0), now=0.0)
    assert lease.tick(0.5) == (0.0, 0.0, 0.0)
    assert lease.tick(0.55) == (0.0, 0.0, 0.0)
    assert lease.tick(0.6) == (0.0, 0.0, 0.0)
    assert lease.tick(0.65) is None
    assert lease.active(0.65) is False


def test_stop_ends_the_lease_immediately():
    lease = teleop.VelocityLease(zero_ticks=2)
    lease.renew((0.2, 0.0, 0.0), now=0.0)
    lease.stop()
    assert lease.tick(0.01) == (0.0, 0.0, 0.0)
    assert lease.tick(0.02) == (0.0, 0.0, 0.0)
    assert lease.tick(0.03) is None


def test_stop_with_nothing_held_publishes_nothing():
    """A stop from a page that never drove must not fight Nav2 with zeros."""
    lease = teleop.VelocityLease()
    lease.stop()
    assert lease.tick(0.0) is None


def test_state_reports_the_held_command():
    lease = teleop.VelocityLease(lease_seconds=0.4)
    lease.renew((0.1, 0.0, 0.0), now=1.0)
    assert lease.state(1.1)["active"] is True
    assert lease.state(1.1)["command"] == [0.1, 0.0, 0.0]
    assert lease.state(2.0)["active"] is False


# ── parsing and limits ───────────────────────────────────────────────────

def test_velocity_is_clamped_per_axis():
    limits = teleop.default_limits()
    vx, vy, wz = teleop.parse_velocity({"vx": 5, "vy": -5, "wz": -50})
    assert (vx, vy, wz) == (limits["max_lin_vel"], -limits["max_lin_vel"],
                            -limits["max_ang_vel"])


def test_missing_velocity_axes_are_zero():
    assert teleop.parse_velocity({"vx": 0.1}) == (0.1, 0.0, 0.0)


@pytest.mark.parametrize("bad", ["fast", None, True, float("nan"), float("inf")])
def test_non_numbers_are_refused(bad):
    with pytest.raises(teleop.TeleopError):
        teleop.parse_velocity({"vx": bad} if bad is not None else {"vx": "x"})


def test_accessory_is_clamped_to_the_profile_ranges():
    limits = teleop.default_limits()
    n, gl, gr = teleop.parse_accessory({"n_pos": 0, "gl_pos": 99, "gr_pos": 5.0})
    assert n == limits["neck"]["min"]
    assert gl == limits["gripper"]["max"]
    assert gr == 5.0


def test_accessory_needs_every_field():
    """A missing field would send that servo to zero."""
    with pytest.raises(teleop.TeleopError):
        teleop.parse_accessory({"n_pos": 5.0, "gl_pos": 5.0})


def test_gripper_presets_are_the_joysticks():
    """Open and close are each other's mirror, halfway to the range ends."""
    presets = teleop.gripper_presets()
    grip = teleop.default_limits()["gripper"]
    assert presets["front"] == (grip["front"], grip["front"])
    assert presets["open"] == presets["close"][::-1]
    assert presets["open"][0] == pytest.approx((grip["max"] + grip["front"]) / 2)


def test_led_single_pair_fills_every_corner():
    request = teleop.parse_led({"mode": 4, "color": 2})
    assert request == {c: {"mode": 4, "color": 2} for c in ("fl", "fr", "bl", "br")}


def test_led_per_corner():
    corners = {"fl": {"mode": 4, "color": 1}, "fr": {"mode": 3, "color": 2},
               "bl": {"mode": 5, "color": 3}, "br": {"mode": 6, "color": 0}}
    assert teleop.parse_led({"corners": corners}) == corners


def test_led_refuses_missing_corners_and_unknown_numbers():
    with pytest.raises(teleop.TeleopError):
        teleop.parse_led({"corners": {"fl": {"mode": 4, "color": 1}}})
    with pytest.raises(teleop.TeleopError):
        teleop.parse_led({"mode": 9, "color": 1})
    with pytest.raises(teleop.TeleopError):
        teleop.parse_led({"mode": 4, "color": 1.5})
