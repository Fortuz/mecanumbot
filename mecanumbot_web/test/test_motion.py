"""
Tests for the movement classifier and the LED display helpers.

Both are pure Python, so this runs without a ROS graph -- which is the
point of keeping them out of the node.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mecanumbot_web import led_enums, motion  # noqa: E402


# ── movement ─────────────────────────────────────────────────────────────

def test_a_still_robot_reads_stopped():
    """Zeros are not a description of motion."""
    described = motion.describe_motion(0.0, 0.0, 0.0)
    assert described["state"] == motion.STOPPED
    assert described["moving"] is False
    assert described["components"] == []


def test_sensor_noise_stays_inside_the_deadband():
    """A parked robot's odometry jitter must not read as creeping."""
    described = motion.describe_motion(0.004, -0.003, 0.01)
    assert described["state"] == motion.STOPPED
    assert described["heading_deg"] is None


def test_each_axis_names_its_direction():
    """Sign conventions follow REP-103, as the rest of the workspace does."""
    assert motion.describe_motion(0.3, 0, 0)["state"] == "forward"
    assert motion.describe_motion(-0.3, 0, 0)["state"] == "backward"
    assert motion.describe_motion(0, 0.3, 0)["state"] == "strafing left"
    assert motion.describe_motion(0, -0.3, 0)["state"] == "strafing right"
    assert motion.describe_motion(0, 0, 0.6)["state"] == "rotating left"
    assert motion.describe_motion(0, 0, -0.6)["state"] == "rotating right"


def test_a_diagonal_carries_both_components():
    """The reason this is not a single-word answer on a mecanum base."""
    described = motion.describe_motion(0.2, 0.2, 0.0)
    assert described["components"] == ["forward", "strafing left"]
    assert described["state"] == "forward + strafing left"
    assert described["heading_deg"] == 45.0


def test_translating_while_turning_reports_all_three():
    """Nav2 arcs look like this, and both halves matter."""
    described = motion.describe_motion(0.25, 0.0, 0.4)
    assert described["state"] == "forward + rotating left"
    assert described["translating"] is True
    assert described["rotating"] is True


def test_a_pure_spin_has_no_direction_of_travel():
    """The page's arrow must not point somewhere arbitrary."""
    described = motion.describe_motion(0.0, 0.0, 0.8)
    assert described["heading_deg"] is None
    assert described["translating"] is False


def test_heading_is_positive_to_the_left():
    """Strafing left is +90 degrees, which is what rotates the arrow."""
    assert motion.describe_motion(0.0, 0.3, 0.0)["heading_deg"] == 90.0
    assert motion.describe_motion(0.0, -0.3, 0.0)["heading_deg"] == -90.0


def test_speed_is_the_planar_magnitude():
    """Strafe counts toward speed; on a differential base it could not."""
    assert motion.describe_motion(0.3, 0.4, 0.0)["speed"] == 0.5


def test_deadbands_are_caller_settable():
    """Simulation and hardware do not have the same noise floor."""
    assert motion.describe_motion(0.05, 0, 0,
                                  linear_deadband=0.1)["state"] == motion.STOPPED
    assert motion.describe_motion(0.05, 0, 0,
                                  linear_deadband=0.01)["state"] == "forward"


def test_idle_keeps_the_shape_but_relabels_it():
    """A stale slot must render with the same fields as a live one."""
    live = motion.describe_motion(0.1, 0.0, 0.0)
    stale = motion.idle(motion.NO_COMMAND)
    assert set(stale) == set(live)
    assert stale["state"] == motion.NO_COMMAND
    assert stale["moving"] is False


# ── LED display ──────────────────────────────────────────────────────────

def test_every_colour_has_a_swatch():
    """A colour the page cannot draw would render as an invisible corner."""
    assert set(led_enums.COLOR_HEX) == set(led_enums.COLOR_NAMES)


def test_a_corner_is_described_by_name_and_colour():
    """What the panel renders per corner."""
    described = led_enums.describe_corner({"mode": 4, "color": 5})
    assert described["mode_name"] == "solid"
    assert described["color_name"] == "cyan"
    assert described["hex"] == led_enums.COLOR_HEX[5]
    assert described["known"] is True


def test_a_failed_serial_read_is_described_not_dropped():
    """The LED node answers -1 in every field; the operator should see that."""
    described = led_enums.describe_corner({"mode": -1, "color": -1})
    assert described["known"] is False
    assert described["hex"] is None
    assert "-1" in described["mode_name"]


def test_a_malformed_corner_does_not_raise():
    """Nothing about a bad reading should take the page down."""
    described = led_enums.describe_corner({"mode": "x"})
    assert described["known"] is False
    assert described["color_name"] == "unknown"
