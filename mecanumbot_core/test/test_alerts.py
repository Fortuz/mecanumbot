"""Unit tests for when a hardware alarm is shown, and which one."""

from mecanumbot_core.alerts import (
    BATTERY_LOW,
    ERROR,
    WARN,
    Debounce,
    active_alerts,
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
        assert choose_alarm(False) is None

    def test_a_low_battery_blinks_red(self):
        assert choose_alarm(True) == BATTERY_LOW


class TestAlertLines:
    def test_a_healthy_robot_has_none(self):
        assert active_alerts(False, None, {}) == []

    def test_silent_motors_are_a_warning(self):
        (severity, key, message), = active_alerts(True, None, {})
        assert (severity, key) == (WARN, "motors_silent")
        assert "Motors not responding" in message

    def test_a_silent_opencr_is_an_error_that_says_for_how_long(self):
        (severity, key, message), = active_alerts(False, 12.4, {})
        assert (severity, key) == (ERROR, "opencr_silent")
        assert "12 s" in message

    def test_a_silent_opencr_hides_its_last_word_about_the_bus(self):
        keys = [alert[1] for alert in active_alerts(True, 5.0, {})]
        assert keys == ["opencr_silent"]

    def test_each_low_battery_gets_a_line(self):
        alerts = active_alerts(False, None, {"opencr": 9.41, "orin": 9.2})
        assert [alert[1] for alert in alerts] == ["battery_opencr", "battery_orin"]
        assert all(alert[0] == ERROR for alert in alerts)
        assert "9.4 V" in alerts[0][2]

    def test_an_opencr_on_usb_alone_is_not_called_a_low_battery(self):
        (_severity, _key, message), = active_alerts(False, None, {"opencr": 0.2})
        assert message.startswith("No main power")

    def test_errors_come_before_warnings(self):
        alerts = active_alerts(True, None, {"opencr": 9.0})
        assert [alert[0] for alert in alerts] == [ERROR, WARN]
