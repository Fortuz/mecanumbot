"""
Sliding-window publish-rate measurement and health classification.

Pure Python -- no ``rclpy``, no clock of its own.  Callers pass in a
monotonic timestamp, which is what makes the whole thing testable by
feeding it synthetic stamps.

This answers the question "is each node keeping up?".  Nothing in the
workspace measured it before: the only liveness signal anywhere was a
pair of presence/absence watchdogs in the old GUI's ROS bridge, which
could tell you a topic had gone silent but not that it had quietly
dropped from 100 Hz to 12 Hz.

Timestamps are *receipt* times, not header stamps, deliberately:

* several monitored topics carry no header at all (``has_object`` is a
  bare ``Bool``);
* the subscriptions are raw (undeserialised bytes) so there is no header
  to read even where one exists;
* and the question being asked is whether the pipeline is delivering to a
  subscriber, which is a receipt-side property.
"""

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, Optional

#: Publishing at or near the nominal rate.
STATUS_OK = "OK"
#: Publishing, but materially below the nominal rate.
STATUS_SLOW = "SLOW"
#: Publishing materially above the nominal rate. Informational.
STATUS_FAST = "FAST"
#: Wired up, but silent because it has nothing to say right now.
STATUS_IDLE = "IDLE"
#: No publisher, or nothing received recently enough to measure.
STATUS_DEAD = "DEAD"

#: Liveness is judged by who publishes the topic.  The default: a sensor
#: or state topic is alive because the node producing it is running.
PRESENCE_PUBLISHERS = "publishers"
#: Liveness is judged by who *consumes* the topic.  Right for command
#: topics: ``/cmd_vel`` has no permanent publisher, and the question worth
#: answering is whether the driver is still listening.
PRESENCE_SUBSCRIBERS = "subscribers"

#: Default fractional tolerance around the nominal rate.
DEFAULT_TOLERANCE = 0.30

#: Default seconds of silence before an event-driven topic reads DEAD.
DEFAULT_STALE_AFTER = 2.0

#: Default measurement window.
DEFAULT_WINDOW_SECONDS = 3.0

#: Hard cap on retained samples, so a 100 Hz topic cannot grow unbounded.
MAX_SAMPLES = 512


@dataclass
class TopicSpec:
    """One monitored topic and what is expected of it."""

    topic: str
    nominal_hz: float = 0.0
    tol: float = DEFAULT_TOLERANCE
    label: str = ""
    note: str = ""
    #: Silence is this topic's normal resting state, so it reads IDLE
    #: rather than DEAD once it goes quiet.  ``/cmd_vel`` is silent
    #: whenever nothing is driving and ``people_fusion`` whenever nobody
    #: is in view; neither is a fault.
    idle_when_silent: bool = False
    #: Which endpoint count proves the topic is still wired up.
    presence: str = PRESENCE_PUBLISHERS

    @property
    def is_event_driven(self) -> bool:
        """
        Report whether the topic has no steady rate to judge against.

        ``people_fusion`` fires once per detection batch and ``/cmd_vel``
        only while something is driving; calling either SLOW because it is
        idle would make the page cry wolf.
        """
        return self.nominal_hz <= 0.0

    @property
    def presence_by_subscribers(self) -> bool:
        """Report whether subscribers, not publishers, prove this topic alive."""
        return self.presence == PRESENCE_SUBSCRIBERS

    def presence_count(self, publishers: int, subscribers: int) -> int:
        """Return whichever endpoint count decides this topic's liveness."""
        return subscribers if self.presence_by_subscribers else publishers


@dataclass
class TopicReading:
    """A measured result for one topic."""

    topic: str
    nominal_hz: float
    measured_hz: float
    age_s: Optional[float]
    samples: int
    publishers: int
    status: str
    label: str = ""
    note: str = ""
    subscribers: int = 0
    presence: str = PRESENCE_PUBLISHERS

    def as_dict(self) -> dict:
        """Return a JSON-serialisable form for the web API."""
        return {
            "topic": self.topic,
            "nominal_hz": round(self.nominal_hz, 3),
            "measured_hz": round(self.measured_hz, 2),
            "age_s": None if self.age_s is None else round(self.age_s, 2),
            "samples": self.samples,
            "publishers": self.publishers,
            "subscribers": self.subscribers,
            "presence": self.presence,
            "status": self.status,
            "label": self.label,
            "note": self.note,
        }


class RateWindow:
    """A bounded sliding window of message receipt times for one topic."""

    def __init__(self, window_seconds: float = DEFAULT_WINDOW_SECONDS):
        """Create an empty window spanning ``window_seconds``."""
        self.window_seconds = float(window_seconds)
        self._stamps: Deque[float] = deque(maxlen=MAX_SAMPLES)

    def record(self, now: float) -> None:
        """Record one message received at monotonic time ``now``."""
        self._stamps.append(float(now))

    def reset(self) -> None:
        """Discard all samples."""
        self._stamps.clear()

    def _trim(self, now: float) -> None:
        """
        Drop samples that have fallen out of the window.

        Ageing samples out (rather than only capping the deque) is what
        makes a dead publisher's measured rate *fall* to zero instead of
        freezing at its last healthy value.
        """
        cutoff = now - self.window_seconds
        while self._stamps and self._stamps[0] < cutoff:
            self._stamps.popleft()

    def measure(self, now: float):
        """Return ``(hz, age_s, samples)`` as of monotonic time ``now``."""
        age = None if not self._stamps else now - self._stamps[-1]
        self._trim(now)

        if len(self._stamps) < 2:
            return 0.0, age, len(self._stamps)

        span = self._stamps[-1] - self._stamps[0]
        if span <= 0.0:
            return 0.0, age, len(self._stamps)

        return (len(self._stamps) - 1) / span, age, len(self._stamps)


def classify(
    spec: TopicSpec,
    measured_hz: float,
    age_s: Optional[float],
    publishers: int,
    stale_after: float = DEFAULT_STALE_AFTER,
    subscribers: int = 0,
) -> str:
    """
    Grade one topic's health.

    ``DEAD`` wins over everything: nothing on the graph at the end that
    proves the topic wired up (publishers, or subscribers for a command
    topic), or nothing heard for long enough that the measured rate is
    meaningless.  For a topic with a nominal rate the staleness limit
    scales with that rate -- three missed periods -- so a 100 Hz topic is
    not given the same two-second grace as a 1 Hz one.

    Silence on a topic marked ``idle_when_silent`` is ``IDLE``, not
    ``DEAD``.  Some topics have nothing to say most of the time --
    ``/cmd_vel`` while the robot stands still, ``people_fusion`` while
    nobody is in view -- and grading those red would train the operator
    to ignore the colour that means something is actually broken.
    """
    if spec.presence_count(publishers, subscribers) <= 0:
        return STATUS_DEAD

    silent = STATUS_IDLE if spec.idle_when_silent else STATUS_DEAD

    if age_s is None:
        return silent

    limit = stale_after
    if not spec.is_event_driven:
        limit = max(stale_after, 3.0 / spec.nominal_hz)
    if age_s > limit:
        return silent

    if spec.is_event_driven:
        return STATUS_OK

    tolerance = spec.tol if spec.tol is not None else DEFAULT_TOLERANCE
    if measured_hz < spec.nominal_hz * (1.0 - tolerance):
        return STATUS_SLOW
    if measured_hz > spec.nominal_hz * (1.0 + tolerance):
        return STATUS_FAST
    return STATUS_OK


class RateMonitor:
    """Tracks a fixed set of topics and reports their measured rates."""

    def __init__(
        self,
        specs=None,
        window_seconds: float = DEFAULT_WINDOW_SECONDS,
        stale_after: float = DEFAULT_STALE_AFTER,
    ):
        """Create a monitor for ``specs`` (an iterable of :class:`TopicSpec`)."""
        self.window_seconds = float(window_seconds)
        self.stale_after = float(stale_after)
        self._specs: Dict[str, TopicSpec] = {}
        self._windows: Dict[str, RateWindow] = {}
        self._publishers: Dict[str, int] = {}
        self._subscribers: Dict[str, int] = {}
        for spec in specs or ():
            self.add(spec)

    @property
    def topics(self):
        """Return the monitored topic names, in configured order."""
        return list(self._specs)

    def spec(self, topic: str) -> Optional[TopicSpec]:
        """Return the spec for one topic, if monitored."""
        return self._specs.get(topic)

    def add(self, spec: TopicSpec) -> None:
        """Start monitoring one topic."""
        self._specs[spec.topic] = spec
        self._windows.setdefault(spec.topic, RateWindow(self.window_seconds))
        self._publishers.setdefault(spec.topic, 0)
        self._subscribers.setdefault(spec.topic, 0)

    def record(self, topic: str, now: float) -> None:
        """Record a message receipt. Unmonitored topics are ignored."""
        window = self._windows.get(topic)
        if window is not None:
            window.record(now)

    def set_publisher_count(self, topic: str, count: int) -> None:
        """Update how many publishers the graph reports for a topic."""
        if topic in self._specs:
            self._publishers[topic] = int(count)

    def set_subscriber_count(self, topic: str, count: int) -> None:
        """
        Update how many subscribers the graph reports for a topic.

        This is what proves a command topic is still wired up: nothing
        publishes ``/cmd_vel`` while the robot stands still, so only the
        driver listening on the other end distinguishes idle from gone.
        """
        if topic in self._specs:
            self._subscribers[topic] = int(count)

    def reset(self) -> None:
        """Clear every window, keeping the configured specs."""
        for window in self._windows.values():
            window.reset()

    def read(self, now: float):
        """Return a :class:`TopicReading` per monitored topic."""
        readings = []
        for topic, spec in self._specs.items():
            measured, age, samples = self._windows[topic].measure(now)
            publishers = self._publishers.get(topic, 0)
            subscribers = self._subscribers.get(topic, 0)
            readings.append(TopicReading(
                topic=topic,
                nominal_hz=spec.nominal_hz,
                measured_hz=measured,
                age_s=age,
                samples=samples,
                publishers=publishers,
                subscribers=subscribers,
                presence=spec.presence,
                status=classify(spec, measured, age, publishers,
                                self.stale_after, subscribers),
                label=spec.label,
                note=spec.note,
            ))
        return readings

    def summary(self, readings) -> Dict[str, int]:
        """Count readings per status, for the page's header badges."""
        counts = {STATUS_OK: 0, STATUS_SLOW: 0, STATUS_FAST: 0,
                  STATUS_IDLE: 0, STATUS_DEAD: 0}
        for reading in readings:
            counts[reading.status] = counts.get(reading.status, 0) + 1
        return counts


def specs_from_config(document) -> Dict[str, object]:
    """
    Build monitor settings from a parsed ``diagnostics_topics.yaml``.

    Returns ``{'specs': [...], 'window_seconds': float, 'stale_after': float}``.
    Unparseable entries are skipped rather than fatal -- a typo in the
    diagnostics table must not take the whole web GUI down.
    """
    document = document or {}
    specs = []

    for entry in document.get("topics") or ():
        if not isinstance(entry, dict) or not entry.get("topic"):
            continue
        nominal = entry.get("nominal_hz", 0.0)
        try:
            nominal = float(nominal)
        except (TypeError, ValueError):
            nominal = 0.0
        if math.isnan(nominal) or nominal < 0.0:
            nominal = 0.0
        try:
            tol = float(entry.get("tol", DEFAULT_TOLERANCE))
        except (TypeError, ValueError):
            tol = DEFAULT_TOLERANCE
        presence = str(entry.get("presence", PRESENCE_PUBLISHERS)).strip().lower()
        if presence not in (PRESENCE_PUBLISHERS, PRESENCE_SUBSCRIBERS):
            presence = PRESENCE_PUBLISHERS
        specs.append(TopicSpec(
            topic=str(entry["topic"]),
            nominal_hz=nominal,
            tol=tol,
            label=str(entry.get("label", "")),
            note=str(entry.get("note", "")),
            idle_when_silent=bool(entry.get("idle_when_silent", False)),
            presence=presence,
        ))

    def _positive(key, fallback):
        try:
            value = float(document.get(key, fallback))
        except (TypeError, ValueError):
            return fallback
        return value if value > 0.0 else fallback

    return {
        "specs": specs,
        "window_seconds": _positive("window_seconds", DEFAULT_WINDOW_SECONDS),
        "stale_after": _positive("stale_after", DEFAULT_STALE_AFTER),
    }
