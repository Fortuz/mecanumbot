"""
Tests for the held-object decision, which need no ROS.

The claim under test: the grabbers hold something only when the VL53L0X sees
something close. No target, a missing sensor and a corrupted reading all mean
"empty", so a broken sensor can never confirm a grasp.
"""

import math

import pytest

from mecanumbot_core.held_object import (
    DMS_NO_TARGET,
    DMS_NOT_READY,
    HeldObjectDetector,
    reading_means_held,
)

MAX_CM = 6.0


@pytest.mark.parametrize("dms", [0.5, 3.0, 5.9, 6.0])
def test_a_close_reading_is_held(dms):
    assert reading_means_held(dms, MAX_CM)


@pytest.mark.parametrize(
    "dms", [6.1, 14.5, 120.0, DMS_NO_TARGET, DMS_NOT_READY, 0.0, math.nan]
)
def test_far_missing_and_sentinel_readings_are_not_held(dms):
    assert not reading_means_held(dms, MAX_CM)


def test_nothing_is_held_at_startup():
    assert not HeldObjectDetector(MAX_CM).held


def test_open_space_never_reads_as_held():
    # An empty gripper facing open space reads -1 continuously.
    det = HeldObjectDetector(MAX_CM, window=11)
    for _ in range(50):
        det.update(DMS_NO_TARGET)
    assert not det.held


def test_a_missing_sensor_never_reads_as_held_and_is_reported():
    det = HeldObjectDetector(MAX_CM, window=11)
    for _ in range(50):
        det.update(DMS_NOT_READY)
    assert not det.held
    assert not det.sensor_ready
    det.update(DMS_NO_TARGET)
    assert det.sensor_ready


def test_held_needs_a_majority_of_the_window():
    det = HeldObjectDetector(MAX_CM, window=11)
    for _ in range(5):
        det.update(4.0)
    assert not det.held  # 5 of 11
    det.update(4.0)
    assert det.held  # 6 of 11


def test_a_single_dropout_does_not_release_the_object():
    det = HeldObjectDetector(MAX_CM, window=11)
    for _ in range(11):
        det.update(4.0)
    det.update(DMS_NO_TARGET)
    det.update(80.0)
    assert det.held


def test_releasing_clears_after_the_majority_turns():
    det = HeldObjectDetector(MAX_CM, window=11)
    for _ in range(11):
        det.update(4.0)
    for _ in range(6):
        det.update(80.0)
    assert not det.held


@pytest.mark.parametrize(
    "kwargs", [{"max_distance_cm": 0.0}, {"max_distance_cm": 5.0, "window": 0}]
)
def test_bad_configuration_is_refused(kwargs):
    with pytest.raises(ValueError):
        HeldObjectDetector(**kwargs)
