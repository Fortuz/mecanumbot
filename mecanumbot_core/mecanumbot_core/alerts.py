"""
The hardware alarms the LEDs show, and when each one is up.

No ROS here, so the timing can be tested without a robot. The node that owns
the LEDs (`mecanumbot_battery_alert`) feeds these the readings and shows
whichever alarm `choose_alarm` picks.
"""

# SetLedStatus vocabulary, from mecanumbot_led.
BLACK, RED, YELLOW = 0, 3, 7
SOLID, FAST_BLINK = 4, 5

# What each alarm looks like on all four LED groups, as (colour, mode). Yellow
# is used by none of the leading experiment's scripts, so a bus fault cannot be
# mistaken for a trial signal, and it is not the battery's red.
BATTERY_LOW = (RED, FAST_BLINK)
DXL_BUS_SILENT = (YELLOW, FAST_BLINK)
LEDS_OFF = (BLACK, SOLID)


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


def choose_alarm(battery_low, bus_silent):
    """
    Return the (colour, mode) to show, or None when no alarm is up.

    The battery wins: a flat battery is a common reason for the Dynamixel bus
    to go silent, so it is the cause to show.
    """
    if battery_low:
        return BATTERY_LOW
    if bus_silent:
        return DXL_BUS_SILENT
    return None
