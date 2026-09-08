"""
Deciding when T1 is finished.

T1 is the first pass: the robot explores on its own while the server builds the
Deep3R point cloud. Ending it is a decision with two halves that are easy to
confuse -- *the robot has nowhere left to go* and *the reconstruction has
stopped improving* -- and they do not fall due at the same time. A robot can run
out of frontiers with half the room reconstructed from one bad angle, and a
cloud can stop growing while the map still has a corridor nobody has driven
down.

So five criteria are evaluated separately and reported separately, and the
composite rule combines them. Which one fired is kept and published, because for
a thesis "T1 ended because the reconstruction stopped improving" and "T1 ended
because the battery ran out" are different runs and must not be summarised into
the same word.

    1. FRONTIERS   no reachable frontier worth driving to, held for a while
    2. GAIN        newly-observed cells per metre driven has collapsed
    3. STABLE      the occupancy grid has stopped changing, and no recent loop
                   closure is still settling
    4. CLOUD       the server says the cloud accounts for enough of the map, has
                   few uncertain regions left, and has stopped growing
    5. BUDGET      time, distance or battery ran out

    finished = (FRONTIERS or GAIN) and STABLE and CLOUD, or BUDGET

Why that shape. 1 and 2 are alternatives because each catches what the other
misses: frontier exhaustion is exact but brittle -- one sliver of unknown space
behind a glass door keeps it false for ever -- while the gain rate is robust but
never quite reaches zero. 3 is a conjunct rather than an alternative because a
map still being re-rasterised after a loop closure will produce a fresh crop of
frontiers a second later, and exiting into that is how a run ends with half a
map. 4 is a conjunct because it is the whole reason the robot is driving around
in the first place: the 2D map is the *means*, the cloud is the product, and a
finished 2D map with an unfinished cloud is a failed T1 that looks like a
successful one. 5 overrides everything because an unattended run has to stop.

Every threshold is a constructor argument; nothing here reads a file or a
blackboard, and nothing imports rclpy, so the whole rule is testable at a desk.
"""

import math

# The names a criterion is reported under.
FRONTIERS = "frontiers"
GAIN = "gain"
STABLE = "stable"
CLOUD = "cloud"
BUDGET = "budget"

CRITERIA = (FRONTIERS, GAIN, STABLE, CLOUD, BUDGET)


class Verdict:
    """The state of every criterion at one moment, and what it adds up to."""

    def __init__(self, met, reasons, finished, trigger=None):
        self.met = dict(met)
        self.reasons = dict(reasons)
        self.finished = bool(finished)
        # Which criterion is responsible for it being finished; None while it
        # is not. BUDGET means the run was cut short rather than completed.
        self.trigger = trigger

    @property
    def complete(self):
        """Say whether T1 finished because it was done, not because time ran out."""
        return self.finished and self.trigger != BUDGET

    def summary(self):
        """One line naming each criterion and why it is or is not met."""
        return "; ".join(
            f"{name}={'yes' if self.met[name] else 'no'} ({self.reasons[name]})"
            for name in CRITERIA
        )

    def __repr__(self):
        """Show the verdict and its trigger, for a log line."""
        return f"Verdict(finished={self.finished}, trigger={self.trigger})"


class ExplorationProgress:
    """
    A sliding record of how fast the map is filling in and how far the robot went.

    Kept as a bounded deque of `(time, known_cells, distance_travelled)` samples
    so the rate is measured over a window rather than between two consecutive
    updates, which at 0.5 Hz map updates is far too noisy to threshold.

    The rate is per *metre driven*, not per second. A robot that has stopped --
    waiting for a plan, stuck against a chair, or parked because there is
    genuinely nothing left -- fills in no cells either way, and a per-second rate
    cannot tell those apart from a finished map. Per metre, a stationary robot
    simply contributes no evidence and the window holds its last value.
    """

    def __init__(self, window=45.0):
        self.window = float(window)
        self._samples = []
        self._loop_closure_time = None

    def add(self, now, known_cells, distance_travelled):
        """Record one sample and drop everything older than the window."""
        self._samples.append((float(now), int(known_cells), float(distance_travelled)))
        cutoff = float(now) - self.window
        self._samples = [s for s in self._samples if s[0] >= cutoff]

    def note_loop_closure(self, now):
        """Record that slam_toolbox re-rasterised the map."""
        self._loop_closure_time = float(now)

    def seconds_since_loop_closure(self, now):
        """Seconds since the last loop closure, or None if there has been none."""
        if self._loop_closure_time is None:
            return None
        return float(now) - self._loop_closure_time

    @property
    def span(self):
        """Seconds covered by the samples currently held."""
        if len(self._samples) < 2:
            return 0.0
        return self._samples[-1][0] - self._samples[0][0]

    def cells_per_metre(self):
        """
        Newly-observed cells per metre driven over the window, or None.

        None while there is not enough evidence to say -- too few samples, or
        the robot has barely moved. A caller must read that as "no opinion" and
        not as "zero", which is the whole difference between an explorer that
        waits for a plan and one that decides it has finished while waiting.
        """
        if len(self._samples) < 2:
            return None
        driven = self._samples[-1][2] - self._samples[0][2]
        if driven < 1e-3:
            return None
        return (self._samples[-1][1] - self._samples[0][1]) / driven

    def cells_changed_fraction(self):
        """
        How much of the map's known area was added over the window, or None.

        This is the stability measure: a map still being extended is not stable,
        whether or not the robot is finding frontiers.
        """
        if len(self._samples) < 2:
            return None
        latest = self._samples[-1][1]
        if latest <= 0:
            return None
        return (latest - self._samples[0][1]) / latest


class CloudProgress:
    """
    What the server has said about the reconstruction, and whether it is settling.

    Fed from `mecanumbot_msgs/MapCloudAgreement`. Only the fields that bear on
    stopping are kept -- the comparison itself belongs on the server, where the
    cloud is.
    """

    def __init__(self, window=60.0):
        self.window = float(window)
        self._samples = []
        self.latest = None
        #: When the reconstruction last restarted, or None if it never has.
        #: T1 must not finish just after one -- see `ExitCriteria._cloud`.
        self.last_reset = None

    def add(
        self,
        now,
        grid_coverage,
        cloud_points,
        uncertain_scores,
        uncertain_points=(),
        cloud_map_id=None,
        agreement=None,
    ):
        """
        Record one comparison verdict.

        `cloud_map_id` is CUT3R's reconstruction session, **not** the robot's
        SLAM map -- pass `MapCloudAgreement.cloud_map_id`. This criterion is
        about whether the *cloud* has stopped growing, and it is the cloud
        restarting that makes earlier point counts incomparable. The robot's
        map restarting does not: the cloud carries on regardless.
        """
        if self.latest is not None and cloud_map_id != self.latest["cloud_map_id"]:
            # A new session did not shrink the cloud, it restarted it, so every
            # earlier sample is a count of something else.
            self._samples = []
            self.last_reset = float(now)
        self.latest = {
            "time": float(now),
            "grid_coverage": float(grid_coverage),
            "cloud_points": int(cloud_points),
            "uncertain": [float(s) for s in uncertain_scores],
            # Parallel to `uncertain`, in the map frame. Kept so the explorer can
            # turn a complaint about the reconstruction into somewhere to drive.
            "points": [(float(p[0]), float(p[1])) for p in uncertain_points],
            "cloud_map_id": cloud_map_id,
            # The share of cloud-occupied cells the map also calls occupied.
            # None means the server compared nothing -- which is a different
            # answer from 0.0, and they have different fixes.
            "agreement": None if agreement is None else float(agreement),
        }
        self._samples.append(self.latest)
        cutoff = float(now) - self.window
        self._samples = [s for s in self._samples if s["time"] >= cutoff]

    @property
    def age(self):
        """Timestamp of the newest verdict, or None if none has arrived."""
        return None if self.latest is None else self.latest["time"]

    def growth_fraction(self):
        """Fractional growth in cloud points over the window, or None."""
        if len(self._samples) < 2:
            return None
        first = self._samples[0]["cloud_points"]
        if first <= 0:
            return None
        return (self._samples[-1]["cloud_points"] - first) / first

    def mean_agreement(self):
        """Mean agreement over the window, or None if nothing was compared."""
        values = [s["agreement"] for s in self._samples if s["agreement"] is not None]
        return sum(values) / len(values) if values else None

    def pending_uncertain(self, score_threshold):
        """Count of uncertain regions still scoring above the threshold."""
        if self.latest is None:
            return None
        return sum(1 for s in self.latest["uncertain"] if s >= score_threshold)


class ExitCriteria:
    """
    The five criteria and the rule that combines them.

    `evaluate()` is called once per detection cycle and is a pure function of
    what it is handed -- it keeps no state of its own beyond the two progress
    records the caller feeds, so the same call replays identically off a bag.
    """

    def __init__(
        self,
        # 1. frontiers
        frontier_quiet_time=20.0,
        # 2. information gain
        min_cells_per_metre=25.0,
        # 3. map stability
        max_growth_fraction=0.01,
        loop_closure_settle=15.0,
        # 4. the reconstruction
        min_grid_coverage=0.75,
        max_uncertain_regions=3,
        uncertain_score_threshold=0.5,
        max_cloud_growth=0.02,
        cloud_verdict_timeout=30.0,
        min_agreement=0.15,
        cloud_reset_settle=30.0,
        require_cloud=True,
        # floors, which no criterion above supplies
        min_runtime=60.0,
        # 5. budget
        max_duration=None,
        max_distance=None,
        min_battery_voltage=None,
    ):
        self.min_agreement = float(min_agreement)
        self.cloud_reset_settle = float(cloud_reset_settle)
        self.min_runtime = None if min_runtime is None else float(min_runtime)
        self.frontier_quiet_time = float(frontier_quiet_time)
        self.min_cells_per_metre = float(min_cells_per_metre)
        self.max_growth_fraction = float(max_growth_fraction)
        self.loop_closure_settle = float(loop_closure_settle)
        self.min_grid_coverage = float(min_grid_coverage)
        self.max_uncertain_regions = int(max_uncertain_regions)
        self.uncertain_score_threshold = float(uncertain_score_threshold)
        self.max_cloud_growth = float(max_cloud_growth)
        self.cloud_verdict_timeout = float(cloud_verdict_timeout)
        self.require_cloud = bool(require_cloud)
        self.max_duration = max_duration
        self.max_distance = max_distance
        self.min_battery_voltage = min_battery_voltage

        self._frontiers_quiet_since = None

    def note_frontiers(self, now, count):
        """
        Record how many frontiers the last detection cycle produced.

        The quiet period is tracked here rather than in `evaluate()` because a
        single empty cycle means very little: the RRT is stochastic and finds
        nothing on plenty of cycles where frontiers plainly remain.
        """
        if count > 0:
            self._frontiers_quiet_since = None
        elif self._frontiers_quiet_since is None:
            self._frontiers_quiet_since = float(now)

    def frontier_quiet_for(self, now):
        """Seconds since the last cycle that found a frontier, or 0.0."""
        if self._frontiers_quiet_since is None:
            return 0.0
        return float(now) - self._frontiers_quiet_since

    def evaluate(
        self,
        now,
        progress,
        cloud=None,
        elapsed=None,
        distance=None,
        battery_voltage=None,
    ):
        """Evaluate all five criteria and return the `Verdict`."""
        met = {}
        reasons = {}

        met[FRONTIERS], reasons[FRONTIERS] = self._frontiers(now)
        met[GAIN], reasons[GAIN] = self._gain(progress)
        met[STABLE], reasons[STABLE] = self._stable(now, progress)
        met[CLOUD], reasons[CLOUD] = self._cloud(now, cloud)
        met[BUDGET], reasons[BUDGET] = self._budget(elapsed, distance, battery_voltage)

        # The budget overrides everything, floors included: an unattended run
        # has to be able to stop, and a flat battery mid-reconstruction loses
        # the whole session.
        if met[BUDGET]:
            return Verdict(met, reasons, True, BUDGET)

        # The floor. Every quality criterion above can be satisfied within
        # seconds of start-up by a robot that has not moved: no frontiers found
        # yet, no movement to measure gain over, a map that is trivially not
        # growing. The server has carried minimums for exactly this since it was
        # written; this end had none.
        if self.min_runtime is not None and elapsed is not None:
            if elapsed < self.min_runtime:
                reasons[BUDGET] = (
                    f"{elapsed:.0f} s in, under the {self.min_runtime:.0f} s "
                    "minimum -- too early to call anything finished"
                )
                return Verdict(met, reasons, False, None)

        nowhere_to_go = met[FRONTIERS] or met[GAIN]
        if nowhere_to_go and met[STABLE] and met[CLOUD]:
            return Verdict(met, reasons, True, FRONTIERS if met[FRONTIERS] else GAIN)
        return Verdict(met, reasons, False, None)

    # --- the individual criteria ---------------------------------------------

    def _frontiers(self, now):
        quiet = self.frontier_quiet_for(now)
        if quiet >= self.frontier_quiet_time:
            return True, f"no frontier for {quiet:.0f} s"
        return False, f"quiet for {quiet:.0f}/{self.frontier_quiet_time:.0f} s"

    def _gain(self, progress):
        rate = progress.cells_per_metre()
        if rate is None:
            return False, "not enough movement to measure"
        if rate < self.min_cells_per_metre:
            return True, f"{rate:.0f} cells/m < {self.min_cells_per_metre:.0f}"
        return False, f"{rate:.0f} cells/m"

    def _stable(self, now, progress):
        since_closure = progress.seconds_since_loop_closure(now)
        if since_closure is not None and since_closure < self.loop_closure_settle:
            return False, f"loop closure {since_closure:.0f} s ago, still settling"
        growth = progress.cells_changed_fraction()
        if growth is None:
            return False, "no map growth measured yet"
        if growth <= self.max_growth_fraction:
            return True, f"map grew {growth * 100:.1f}% over the window"
        return False, f"map still growing ({growth * 100:.1f}%)"

    def _cloud(self, now, cloud):
        if not self.require_cloud:
            return True, "not required"
        if cloud is None or cloud.latest is None:
            return False, "no comparison verdict from the server yet"

        age = now - cloud.age
        if age > self.cloud_verdict_timeout:
            return False, f"last verdict {age:.0f} s old"

        # Placement first, because it is the failure that looks like a success.
        # A fully explored grid over a cloud that was never correctly placed is
        # a failed T1 indistinguishable from a good one from the 2D data alone,
        # and it surfaces an hour into T2 as "the detector never sees anything".
        # `None` and `0.0` are reported separately because they have different
        # fixes: nothing was ever compared (no grid, no pose, or compare off)
        # against compared and agreed with nothing (wrong camera height, wrong
        # mount, pose in the wrong frame).
        agreement = cloud.mean_agreement()
        if agreement is None:
            return False, ("the server has compared nothing -- no grid, no pose, "
                           "or compare is off")
        if agreement < self.min_agreement:
            return False, (
                f"cloud/grid agreement is {agreement:.2f}, under "
                f"{self.min_agreement:.2f} -- the reconstruction is not placed "
                "where the map is, so T2 would search a cloud that does not "
                "line up with the room"
            )

        # And not immediately after the reconstruction restarted, for the same
        # reason the map criterion refuses to finish just after a loop closure:
        # the cloud T2 is about to search would have been built from whatever
        # fraction of the drive came after the reset.
        if cloud.last_reset is not None:
            since_reset = now - cloud.last_reset
            if since_reset < self.cloud_reset_settle:
                return False, (
                    f"the reconstruction restarted {since_reset:.0f} s ago; "
                    f"settling for {self.cloud_reset_settle:.0f} s"
                )

        coverage = cloud.latest["grid_coverage"]
        if coverage < self.min_grid_coverage:
            return (
                False,
                f"cloud covers {coverage * 100:.0f}% of the map, "
                f"needs {self.min_grid_coverage * 100:.0f}%",
            )

        pending = cloud.pending_uncertain(self.uncertain_score_threshold)
        if pending > self.max_uncertain_regions:
            return False, f"{pending} uncertain regions left"

        growth = cloud.growth_fraction()
        if growth is None:
            return False, "cloud growth not measurable yet"
        if growth > self.max_cloud_growth:
            return False, f"cloud still growing ({growth * 100:.1f}%)"

        return (
            True,
            f"cloud covers {coverage * 100:.0f}%, agreement {agreement:.2f}, "
            f"{pending} uncertain, growth {growth * 100:.1f}%",
        )

    def _budget(self, elapsed, distance, battery_voltage):
        if self.max_duration is not None and elapsed is not None:
            if elapsed >= self.max_duration:
                return True, f"ran for {elapsed:.0f} s"
        if self.max_distance is not None and distance is not None:
            if distance >= self.max_distance:
                return True, f"drove {distance:.0f} m"
        if self.min_battery_voltage is not None and battery_voltage is not None:
            # A flat battery mid-reconstruction loses the whole session, because
            # the cloud lives on the server against a map_id that will not
            # survive the robot coming back.
            if battery_voltage <= self.min_battery_voltage:
                return True, f"battery at {battery_voltage:.1f} V"
        return False, "within budget"


def travelled(previous, current):
    """Planar distance between two `(x, y)` points, 0.0 if either is missing."""
    if previous is None or current is None:
        return 0.0
    return math.hypot(current[0] - previous[0], current[1] - previous[1])
