"""
Tests for the fixed scan grid, which need no ROS.

What slam_toolbox needs from it is one thing above all: the same number of
readings, at the same angles, whatever the driver sent. The rest is not losing
or moving what the lidar saw on the way.
"""

import math

import pytest

from mecanumbot_core.scan_grid import TWO_PI, grid_geometry, regrid


def a_revolution(count, angle_min=0.01, wall_at=None, wall_range=1.0, far=5.0):
    """Return one LD08-like revolution: ``count`` readings over just under a turn."""
    increment = (TWO_PI - 0.03) / (count - 1)
    ranges = []
    for i in range(count):
        angle = angle_min + i * increment
        near = wall_at is not None and abs(angle - wall_at) < increment / 2
        ranges.append(wall_range if near else far)
    return angle_min, increment, ranges


@pytest.mark.parametrize("count", [201, 202, 203, 204, 205, 206])
def test_every_revolution_comes_out_the_same_length(count):
    angle_min, increment, ranges = a_revolution(count)
    out, intensities = regrid(angle_min, increment, ranges, 0.1, 8.0, 200)
    assert len(out) == 200
    assert len(intensities) == 200


@pytest.mark.parametrize("count", [201, 204, 206])
def test_a_full_revolution_leaves_no_sector_empty(count):
    """1.8 degree sectors against ~1.76 degree spacing: nothing falls between."""
    angle_min, increment, ranges = a_revolution(count)
    out, _ = regrid(angle_min, increment, ranges, 0.1, 8.0, 200)
    assert all(math.isfinite(r) for r in out)


def test_a_wall_stays_at_its_angle():
    wall = math.radians(91.0)  # mid-sector: 90 deg is exactly a sector boundary
    angle_min, increment, ranges = a_revolution(204, wall_at=wall)
    out, _ = regrid(angle_min, increment, ranges, 0.1, 8.0, 200)
    _, _, width = grid_geometry(200)
    assert out.count(1.0) == 1
    # The reading nearest the wall is at most half a lidar step away from it.
    assert abs(out.index(1.0) - wall / width) <= 1.0


def test_a_sector_keeps_its_nearest_return():
    """A wall and the room behind it in one sector is a wall."""
    out, _ = regrid(0.0, 0.001, [3.0, 0.8, 2.0], 0.1, 8.0, 4)
    assert out[0] == pytest.approx(0.8)


def test_readings_that_are_not_returns_are_ignored():
    ranges = [0.0, float("nan"), float("inf"), 0.05, 12.0]
    out, _ = regrid(0.0, 0.001, ranges, 0.1, 8.0, 4)
    assert out[0] == math.inf


def test_a_negative_angle_wraps_to_the_end_of_the_turn():
    out, _ = regrid(-0.1, 0.0, [1.5], 0.1, 8.0, 200)
    assert out[-4] == pytest.approx(1.5)
    assert out.count(1.5) == 1


def test_an_angle_at_two_pi_does_not_index_past_the_grid():
    out, _ = regrid(TWO_PI - 1e-15, 0.0, [2.0], 0.1, 8.0, 200)
    assert len(out) == 200
    assert out.count(2.0) == 1


def test_intensities_follow_the_reading_that_won():
    out, intensities = regrid(0.0, 0.001, [3.0, 0.8], 0.1, 8.0, 4, intensities=[10, 99])
    assert out[0] == pytest.approx(0.8)
    assert intensities[0] == pytest.approx(99.0)


def test_the_grid_geometry_describes_the_ranges():
    angle_min, angle_max, increment = grid_geometry(200)
    assert angle_min == 0.0
    assert increment == pytest.approx(TWO_PI / 200)
    # Karto's count: round((max - min) / increment) + 1 -- the same every scan.
    assert round((angle_max - angle_min) / increment) + 1 == 200


def test_zero_bins_is_refused():
    with pytest.raises(ValueError):
        regrid(0.0, 0.01, [1.0], 0.1, 8.0, 0)
