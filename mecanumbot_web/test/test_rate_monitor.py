"""
Tests for sliding-window rate measurement and health classification.

Synthetic timestamps throughout, so there is no clock, no ROS graph and
no waiting.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mecanumbot_web.rate_monitor import (  # noqa: E402
    STATUS_DEAD,
    STATUS_FAST,
    STATUS_OK,
    STATUS_SLOW,
    RateMonitor,
    RateWindow,
    TopicSpec,
    classify,
    specs_from_config,
)


def feed(window, count, period, start=100.0):
    """Push ``count`` evenly spaced stamps and return the last one."""
    now = start
    for _ in range(count):
        window.record(now)
        now += period
    return now - period


# ── measurement ──────────────────────────────────────────────────────────

def test_measures_a_steady_rate():
    """100 stamps 10 ms apart is 100 Hz."""
    window = RateWindow(window_seconds=3.0)
    last = feed(window, 100, 0.01)
    hz, age, samples = window.measure(last)
    assert hz == pytest.approx(100.0, abs=1.0)
    assert age == pytest.approx(0.0)
    assert samples == 100


def test_measures_a_slow_rate():
    """10 stamps 100 ms apart is 10 Hz."""
    window = RateWindow(window_seconds=3.0)
    last = feed(window, 10, 0.1)
    hz, _age, _samples = window.measure(last)
    assert hz == pytest.approx(10.0, abs=0.5)


def test_a_single_sample_cannot_give_a_rate():
    """One stamp is not an interval."""
    window = RateWindow()
    window.record(1.0)
    hz, age, samples = window.measure(1.0)
    assert hz == 0.0
    assert samples == 1
    assert age == 0.0


def test_empty_window_reports_no_age():
    """Nothing received yet is distinguishable from received long ago."""
    hz, age, samples = RateWindow().measure(5.0)
    assert (hz, age, samples) == (0.0, None, 0)


def test_rate_decays_when_the_publisher_stops():
    """
    Ageing samples out is what makes a stalled topic's number fall.

    Capping the deque alone would leave the last healthy rate frozen on
    screen indefinitely, which is precisely the wrong thing to show.
    """
    window = RateWindow(window_seconds=1.0)
    last = feed(window, 100, 0.01)
    assert window.measure(last)[0] > 50.0
    assert window.measure(last + 2.0)[0] == 0.0


def test_age_grows_after_the_last_message():
    """Age is measured from the newest sample, not the window edge."""
    window = RateWindow(window_seconds=3.0)
    last = feed(window, 5, 0.1)
    assert window.measure(last + 1.5)[1] == pytest.approx(1.5)


def test_reset_clears_samples():
    """Resetting a window forgets everything."""
    window = RateWindow()
    feed(window, 10, 0.01)
    window.reset()
    assert window.measure(200.0)[2] == 0


# ── classification ───────────────────────────────────────────────────────

def test_on_rate_is_ok():
    """Within tolerance is OK."""
    spec = TopicSpec("/t", nominal_hz=100.0, tol=0.30)
    assert classify(spec, 98.4, 0.01, 1) == STATUS_OK


def test_under_rate_is_slow():
    """A node that is running but falling behind reads SLOW."""
    spec = TopicSpec("/t", nominal_hz=100.0, tol=0.30)
    assert classify(spec, 60.0, 0.01, 1) == STATUS_SLOW


def test_over_rate_is_fast():
    """Above tolerance is reported, but only informationally."""
    spec = TopicSpec("/t", nominal_hz=100.0, tol=0.30)
    assert classify(spec, 200.0, 0.01, 1) == STATUS_FAST


def test_no_publisher_is_dead_regardless_of_history():
    """A departed publisher outranks a healthy-looking window."""
    spec = TopicSpec("/t", nominal_hz=100.0)
    assert classify(spec, 100.0, 0.0, 0) == STATUS_DEAD


def test_never_received_is_dead():
    """No age means nothing has arrived at all."""
    spec = TopicSpec("/t", nominal_hz=100.0)
    assert classify(spec, 0.0, None, 1) == STATUS_DEAD


def test_staleness_limit_is_extended_for_slow_topics():
    """
    stale_after is a floor; slow topics get three periods instead.

    A 0.5 Hz topic publishes every 2 s, so judging it against a flat 2 s
    limit would flap between OK and DEAD every cycle. Fast topics keep the
    2 s floor: half a second of silence at 100 Hz is a hiccup, and the
    measured rate already reports it as SLOW.
    """
    slow = TopicSpec("/slow", nominal_hz=0.5)
    assert classify(slow, 0.5, 4.0, 1, stale_after=2.0) == STATUS_OK
    assert classify(slow, 0.5, 8.0, 1, stale_after=2.0) == STATUS_DEAD

    fast = TopicSpec("/fast", nominal_hz=100.0)
    assert classify(fast, 100.0, 0.5, 1, stale_after=2.0) == STATUS_OK
    assert classify(fast, 100.0, 3.0, 1, stale_after=2.0) == STATUS_DEAD


def test_event_driven_topic_is_never_slow():
    """Idle is the normal state for /cmd_vel; crying wolf helps nobody."""
    spec = TopicSpec("/cmd_vel", nominal_hz=0.0)
    assert spec.is_event_driven
    assert classify(spec, 0.0, 0.5, 1) == STATUS_OK


def test_event_driven_topic_still_goes_dead_when_stale():
    """Staleness is the only judgement left for an event-driven topic."""
    spec = TopicSpec("/cmd_vel", nominal_hz=0.0)
    assert classify(spec, 0.0, 5.0, 1, stale_after=2.0) == STATUS_DEAD


# ── the monitor ──────────────────────────────────────────────────────────

def test_monitor_reports_every_configured_topic():
    """Configured but absent topics appear as DEAD, not missing."""
    monitor = RateMonitor([TopicSpec("/a", 50.0), TopicSpec("/b", 0.0)])
    readings = monitor.read(10.0)
    assert [reading.topic for reading in readings] == ["/a", "/b"]
    assert all(reading.status == STATUS_DEAD for reading in readings)


def test_monitor_tracks_a_live_topic():
    """Recording receipts moves a topic to OK."""
    monitor = RateMonitor([TopicSpec("/a", 100.0, tol=0.3)])
    monitor.set_publisher_count("/a", 1)
    now = 100.0
    for _ in range(100):
        monitor.record("/a", now)
        now += 0.01
    reading = monitor.read(now - 0.01)[0]
    assert reading.status == STATUS_OK
    assert reading.measured_hz == pytest.approx(100.0, abs=2.0)


def test_recording_an_unmonitored_topic_is_ignored():
    """Stray callbacks must not create phantom rows."""
    monitor = RateMonitor([TopicSpec("/a", 10.0)])
    monitor.record("/unknown", 1.0)
    assert [reading.topic for reading in monitor.read(1.0)] == ["/a"]


def test_summary_counts_statuses():
    """The header badges add up."""
    monitor = RateMonitor([TopicSpec("/a", 10.0), TopicSpec("/b", 10.0)])
    readings = monitor.read(1.0)
    assert monitor.summary(readings)[STATUS_DEAD] == 2


def test_reading_serialises_for_the_api():
    """The API contract stays stable."""
    monitor = RateMonitor([TopicSpec("/a", 10.0, label="A topic")])
    payload = monitor.read(1.0)[0].as_dict()
    assert set(payload) >= {
        "topic", "nominal_hz", "measured_hz", "age_s",
        "publishers", "status", "label"}


# ── configuration ────────────────────────────────────────────────────────

def test_specs_from_config_parses_the_shipped_table():
    """The shipped diagnostics table loads and covers the core topics."""
    import yaml
    path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config", "diagnostics_topics.yaml")
    with open(path, "r") as handle:
        settings = specs_from_config(yaml.safe_load(handle))

    topics = {spec.topic for spec in settings["specs"]}
    assert "/mecanumbot/opencr_state" in topics
    assert "/mecanumbot/joy" in topics
    assert settings["window_seconds"] > 0.0

    opencr = next(spec for spec in settings["specs"]
                  if spec.topic == "/mecanumbot/opencr_state")
    assert opencr.nominal_hz == 100.0


def test_specs_from_config_skips_bad_entries_rather_than_failing():
    """A typo in the table must not take the whole GUI down."""
    settings = specs_from_config({
        "topics": [
            {"topic": "/good", "nominal_hz": 10.0},
            {"nominal_hz": 5.0},                       # no topic
            "not a mapping",
            {"topic": "/coerced", "nominal_hz": "fast"},
        ],
    })
    topics = {spec.topic: spec for spec in settings["specs"]}
    assert set(topics) == {"/good", "/coerced"}
    assert topics["/coerced"].nominal_hz == 0.0


def test_specs_from_config_falls_back_on_bad_window():
    """A nonsense window is replaced, not propagated."""
    settings = specs_from_config({"topics": [], "window_seconds": -5})
    assert settings["window_seconds"] > 0.0


def test_specs_from_config_handles_an_empty_document():
    """No config is an empty monitor, not a crash."""
    assert specs_from_config(None)["specs"] == []
