"""
The hardware faults the robot reports, and when each one is up.

No ROS here, so the timing can be tested without a robot. The node that watches
the hardware (`mecanumbot_battery_alert`) feeds these the readings, shows on the
LEDs whatever `choose_alarm` picks, and publishes `hardware_alerts` for the web
GUI's alert lines.

The lights are for the battery alone. They are also what the LED condition of
the leading experiment signals with, so a fault the operator can read on the
page has no business taking them over mid-trial.
"""

# SetLedStatus vocabulary, from mecanumbot_led.
BLACK, RED = 0, 3
SOLID, FAST_BLINK = 4, 5

# What each alarm looks like on all four LED groups, as (colour, mode).
BATTERY_LOW = (RED, FAST_BLINK)
LEDS_OFF = (BLACK, SOLID)

# How an alert line is graded: the page draws WARN yellow and ERROR red.
WARN, ERROR = "warn", "error"

# An OpenCR reading below this is not a flat battery, it is no battery: the
# board is running on USB from the Jetson with the 12 V rail absent.
NO_POWER_BELOW = 1.0


def dxl_bus_silent(errors):
    """
    Say whether one OpenCR frame reports the Dynamixel bus as unreadable.

    A negative `err_*` is the board's "this wheel did not answer" sentinel (see
    `mecanumbot_IO_node`); any one of the four is enough, since the frame's
    wheel velocity, position and current are then stale for that wheel.
    """
    return min(errors) < 0


class Debounce:
    """
    A fault that is raised after `raise_after` s bad and cleared after `clear_after` s good.

    Both edges are timed, so a bus that answers every other frame neither
    strobes the lights nor hides behind the good frames in between.
    """

    def __init__(self, raise_after=1.0, clear_after=1.0):
        self.raise_after = float(raise_after)
        self.clear_after = float(clear_after)
        self.active = False
        self._since = None  # when the readings last started disagreeing with `active`

    def update(self, bad, now):
        """Fold in one reading taken at `now` (seconds); return whether the fault is up."""
        if bool(bad) == self.active:
            self._since = None
            return self.active
        if self._since is None:
            self._since = now
        hold = self.clear_after if self.active else self.raise_after
        if now - self._since >= hold:
            self.active = not self.active
            self._since = None
        return self.active


def choose_alarm(battery_low):
    """Return the (colour, mode) to show, or None when no alarm is up."""
    return BATTERY_LOW if battery_low else None


def active_alerts(bus_silent, opencr_silent_for, low_batteries):
    """
    Return the faults that are up, as (severity, key, message), worst first.

    `opencr_silent_for` is the seconds of `opencr_state` silence if the board
    counts as silent, else None. `low_batteries` maps a battery's name to its
    voltage. A silent board says nothing about the bus, whose last state is only
    the last thing the board said, so the bus line gives way to the board's.
    """
    alerts = []
    if opencr_silent_for is not None:
        alerts.append((
            ERROR, "opencr_silent",
            f"OpenCR not responding: no data for {opencr_silent_for:.0f} s. "
            "Check its USB cable and that the board is on.",
        ))
    for battery, voltage in sorted(low_batteries.items()):
        message = f"Battery low: {battery} at {voltage:.1f} V."
        if battery == "opencr" and voltage < NO_POWER_BELOW:
            message = (
                f"No main power: the OpenCR reads {voltage:.1f} V, so it is "
                "probably running on USB alone. Check the battery and the 12 V rail."
            )
        alerts.append((ERROR, f"battery_{battery}", message))
    if bus_silent and opencr_silent_for is None:
        alerts.append((
            WARN, "motors_silent",
            "Motors not responding: the OpenCR cannot read the Dynamixel bus, so "
            "wheel feedback is stale. Check DXL power and cabling.",
        ))
    return alerts
