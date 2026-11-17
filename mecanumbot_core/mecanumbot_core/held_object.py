"""
Decide from the gripper distance sensor whether an object is held.

The sensor is a VL53L0X time-of-flight ranger looking forward between the
grabber shafts. The OpenCR firmware sends its reading as ``dms`` in
``OpenCRState``:

* a distance in cm (> 0) when the sensor sees something in range;
* ``-1`` when it has no valid reading: nothing within range (about 1.2 m in
  the default mode), an I2C error, or no fresh sample;
* ``-2`` while the sensor is not initialised (not found on the bus).

An object is held when the sensor sees something *close*: a valid reading at
or below ``max_distance_cm``. Anything else -- a far reading, no target, a
missing sensor -- is "not held". A broken sensor therefore makes the robot
believe its grabbers are empty, never full, so a grasp is retried rather than
falsely confirmed.

Single readings are noisy, so the decision is a majority vote over the last
``window`` readings.
"""

from collections import deque

DMS_NO_TARGET = -1.0
DMS_NOT_READY = -2.0


def reading_is_valid(dms):
    """Return True when ``dms`` is a real distance rather than a sentinel."""
    # NaN compares False, so a corrupted float is not a distance either.
    return dms > 0.0


def reading_is_not_ready(dms):
    """Return True when ``dms`` says the sensor is not initialised."""
    # Halfway between the two sentinels, so float rounding cannot matter.
    return dms <= -1.5


def reading_means_held(dms, max_distance_cm):
    """Return True when one reading puts an object inside the grabbers."""
    return reading_is_valid(dms) and dms <= max_distance_cm


class HeldObjectDetector:
    """Majority vote over the last ``window`` gripper distance readings."""

    def __init__(self, max_distance_cm, window=11):
        """Start with an empty-gripper history, so nothing is held at startup."""
        if max_distance_cm <= 0.0:
            raise ValueError(f"max_distance_cm must be > 0, got {max_distance_cm}")
        if window < 1:
            raise ValueError(f"window must be >= 1, got {window}")
        self.max_distance_cm = float(max_distance_cm)
        self._votes = deque([False] * int(window), maxlen=int(window))
        self.sensor_ready = True

    def update(self, dms):
        """Add one reading and return whether an object is held now."""
        self._votes.append(reading_means_held(dms, self.max_distance_cm))
        self.sensor_ready = not reading_is_not_ready(dms)
        return self.held

    @property
    def held(self):
        """Return True when more than half of the recent readings say held."""
        return sum(self._votes) > len(self._votes) / 2
