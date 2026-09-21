"""Unit tests for when a hardware alarm is shown, and which one."""

from mecanumbot_core.alerts import (
    BATTERY_LOW,
    DXL_BUS_SILENT,
    Debounce,
    choose_alarm,
    dxl_bus_silent,
)

FRAME = 0.01  # the IO node publishes opencr_state at 100 Hz


def feed(latch, bad, seconds, start=0.0):
    """Feed `seconds` of identical frames; return the time after the last one."""
    t = start
    for _ in range(int(round(seconds / FRAME))):
        latch.update(bad, t)
        t += FRAME
    return t


class TestBusSentinel:
    def test_one_silent_wheel_is_a_silent_bus(self):
        assert dxl_bus_silent((0, 0, -1, 0))

    def test_a_hardware_error_code_is_not(self):
        # Bit 0 is the Input Voltage Error: the wheel answered.
        assert not dxl_bus_silent((1, 0, 0, 0))
        assert not dxl_bus_silent((0, 0, 0, 0))


class TestDebounce:
    def test_a_brief_dropout_raises_nothing(self):
        latch = Debounce(1.0, 1.0)
        feed(latch, True, 0.5)
        assert not latch.active

    def test_a_second_of_silence_raises_it(self):
        latch = Debounce(1.0, 1.0)
        feed(latch, True, 1.05)
        assert latch.active

    def test_it_stays_up_through_a_brief_recovery(self):
        latch = Debounce(1.0, 1.0)
        t = feed(latch, True, 1.5)
        feed(latch, False, 0.5, start=t)
        assert latch.active

    def test_a_second_of_answers_clears_it(self):
        latch = Debounce(1.0, 1.0)
        t = feed(latch, True, 1.5)
        feed(latch, False, 1.05, start=t)
        assert not latch.active

    def test_a_flapping_bus_does_not_strobe(self):
        # Every other frame answers: neither edge is ever held long enough.
        latch = Debounce(1.0, 1.0)
        t, states = 0.0, set()
        for i in range(500):
            states.add(latch.update(i % 2 == 0, t))
            t += FRAME
        assert states == {False}


class TestChoice:
    def test_nothing_up_shows_nothing(self):
        assert choose_alarm(False, False) is None

    def test_each_alarm_has_its_own_look(self):
        assert choose_alarm(True, False) == BATTERY_LOW
        assert choose_alarm(False, True) == DXL_BUS_SILENT
        assert BATTERY_LOW != DXL_BUS_SILENT

    def test_the_battery_wins(self):
        assert choose_alarm(True, True) == BATTERY_LOW
