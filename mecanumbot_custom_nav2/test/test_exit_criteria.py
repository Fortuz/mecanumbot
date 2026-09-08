"""
Tests for the T1 exit criteria.

This is the rule that decides a scan is finished, so the tests are written
around the ways it can be wrong in a way that looks right: exiting while the map
is still being re-rasterised after a loop closure, exiting because the robot was
standing still rather than because there was nothing left to see, and exiting on
a finished 2D map over an unfinished point cloud -- which is a failed T1 that
looks exactly like a successful one.
"""

import pytest

from mecanumbot_custom_nav2.exit_criteria import (
    BUDGET,
    CLOUD,
    CRITERIA,
    FRONTIERS,
    GAIN,
    STABLE,
    CloudProgress,
    ExitCriteria,
    ExplorationProgress,
    travelled,
)


def finished_progress(now=100.0, window=45.0):
    """Build a progress record satisfying GAIN and STABLE: driving, learning nothing."""
    progress = ExplorationProgress(window=window)
    # 20 m driven over the window, and the known area has not moved.
    progress.add(now - 40.0, 10_000, 0.0)
    progress.add(now, 10_000, 20.0)
    return progress


def finished_cloud(now=100.0, window=60.0):
    """Build a cloud record satisfying CLOUD: covered, settled, few complaints."""
    cloud = CloudProgress(window=window)
    # `agreement` is what the `placed` half of this criterion is judged on, and
    # a verdict without it means the server compared nothing.
    cloud.add(now - 40.0, 0.80, 1_000_000, [0.2], cloud_map_id="a", agreement=0.55)
    cloud.add(now, 0.82, 1_005_000, [0.2], cloud_map_id="a", agreement=0.57)
    return cloud


def quiet(criteria, now=100.0, quiet_for=30.0):
    """Tell the criteria the detector has found nothing for a while."""
    criteria.note_frontiers(now - quiet_for, 0)
    return criteria


class TestExplorationProgress:
    """The sliding record of how fast the map fills in."""

    def test_no_opinion_with_one_sample(self):
        progress = ExplorationProgress()
        progress.add(0.0, 100, 0.0)
        assert progress.cells_per_metre() is None

    def test_no_opinion_while_the_robot_stands_still(self):
        # The distinction the whole criterion turns on: a robot waiting for a
        # plan learns nothing, and that is not evidence the map is finished.
        progress = ExplorationProgress()
        progress.add(0.0, 100, 5.0)
        progress.add(10.0, 100, 5.0)
        assert progress.cells_per_metre() is None

    def test_the_rate_is_per_metre_driven(self):
        progress = ExplorationProgress()
        progress.add(0.0, 1_000, 0.0)
        progress.add(10.0, 1_500, 10.0)
        assert progress.cells_per_metre() == pytest.approx(50.0)

    def test_samples_older_than_the_window_are_dropped(self):
        progress = ExplorationProgress(window=10.0)
        progress.add(0.0, 1_000, 0.0)
        progress.add(50.0, 2_000, 10.0)
        progress.add(55.0, 2_000, 20.0)
        assert progress.span == pytest.approx(5.0)
        assert progress.cells_per_metre() == pytest.approx(0.0)

    def test_growth_fraction_is_of_the_current_known_area(self):
        progress = ExplorationProgress()
        progress.add(0.0, 900, 0.0)
        progress.add(10.0, 1_000, 5.0)
        assert progress.cells_changed_fraction() == pytest.approx(0.1)

    def test_loop_closures_are_remembered(self):
        progress = ExplorationProgress()
        assert progress.seconds_since_loop_closure(10.0) is None
        progress.note_loop_closure(5.0)
        assert progress.seconds_since_loop_closure(10.0) == pytest.approx(5.0)


class TestCloudProgress:
    """What the server has said about the reconstruction."""

    def test_growth_needs_two_samples(self):
        cloud = CloudProgress()
        cloud.add(0.0, 0.5, 1_000, [], cloud_map_id="a", agreement=0.55)
        assert cloud.growth_fraction() is None

    def test_growth_is_fractional(self):
        cloud = CloudProgress()
        cloud.add(0.0, 0.5, 1_000, [], cloud_map_id="a", agreement=0.55)
        cloud.add(10.0, 0.6, 1_100, [], cloud_map_id="a", agreement=0.55)
        assert cloud.growth_fraction() == pytest.approx(0.1)

    def test_a_new_cloud_map_id_voids_the_history(self):
        # CUT3R anchors its world frame on the first frame of a session, so a new
        # one means the cloud restarted -- not that it shrank by 99%.
        cloud = CloudProgress()
        cloud.add(0.0, 0.9, 1_000_000, [], cloud_map_id="a", agreement=0.55)
        cloud.add(10.0, 0.1, 1_000, [], cloud_map_id="b", agreement=0.55)
        assert cloud.growth_fraction() is None

    def test_pending_uncertain_counts_above_the_threshold(self):
        cloud = CloudProgress()
        cloud.add(0.0, 0.9, 1_000, [0.9, 0.6, 0.2], cloud_map_id="a", agreement=0.55)
        assert cloud.pending_uncertain(0.5) == 2
        assert cloud.pending_uncertain(0.95) == 0

    def test_uncertain_points_are_kept_alongside_their_scores(self):
        cloud = CloudProgress()
        cloud.add(0.0, 0.9, 1_000, [0.9], uncertain_points=[(1.0, 2.0)], cloud_map_id="a", agreement=0.55)
        assert cloud.latest["points"] == [(1.0, 2.0)]


class TestIndividualCriteria:
    """Each criterion on its own."""

    def test_frontiers_needs_a_sustained_quiet(self):
        # One empty cycle means nothing: the RRT is stochastic and draws a blank
        # constantly on maps with plenty of frontier left.
        criteria = ExitCriteria(frontier_quiet_time=20.0)
        criteria.note_frontiers(100.0, 0)
        verdict = criteria.evaluate(105.0, ExplorationProgress())
        assert not verdict.met[FRONTIERS]
        verdict = criteria.evaluate(125.0, ExplorationProgress())
        assert verdict.met[FRONTIERS]

    def test_a_detection_resets_the_quiet(self):
        criteria = ExitCriteria(frontier_quiet_time=20.0)
        criteria.note_frontiers(100.0, 0)
        criteria.note_frontiers(115.0, 3)
        assert criteria.frontier_quiet_for(125.0) == 0.0

    def test_gain_is_not_met_without_movement(self):
        criteria = ExitCriteria()
        progress = ExplorationProgress()
        progress.add(0.0, 1_000, 0.0)
        progress.add(10.0, 1_000, 0.0)
        verdict = criteria.evaluate(10.0, progress)
        assert not verdict.met[GAIN]
        assert "movement" in verdict.reasons[GAIN]

    def test_gain_is_met_once_driving_stops_paying(self):
        criteria = ExitCriteria(min_cells_per_metre=25.0)
        progress = ExplorationProgress()
        progress.add(0.0, 10_000, 0.0)
        progress.add(20.0, 10_100, 20.0)  # 5 cells/m
        assert criteria.evaluate(20.0, progress).met[GAIN]

    def test_stable_is_refused_right_after_a_loop_closure(self):
        # slam_toolbox re-rasterises the whole grid on a closure and a fresh
        # crop of frontiers appears a second later. Exiting into that is how a
        # run ends with half a map.
        criteria = ExitCriteria(loop_closure_settle=15.0)
        progress = finished_progress()
        progress.note_loop_closure(95.0)
        verdict = criteria.evaluate(100.0, progress)
        assert not verdict.met[STABLE]
        assert "loop closure" in verdict.reasons[STABLE]

    def test_stable_returns_once_the_closure_has_settled(self):
        criteria = ExitCriteria(loop_closure_settle=15.0)
        progress = finished_progress(now=100.0)
        progress.note_loop_closure(50.0)
        assert criteria.evaluate(100.0, progress).met[STABLE]

    def test_stable_is_refused_while_the_map_still_grows(self):
        criteria = ExitCriteria(max_growth_fraction=0.01)
        progress = ExplorationProgress()
        progress.add(60.0, 9_000, 0.0)
        progress.add(100.0, 10_000, 20.0)
        assert not criteria.evaluate(100.0, progress).met[STABLE]

    def test_cloud_is_refused_with_no_verdict_at_all(self):
        criteria = ExitCriteria()
        verdict = criteria.evaluate(100.0, finished_progress())
        assert not verdict.met[CLOUD]
        assert "server" in verdict.reasons[CLOUD]

    def test_cloud_is_refused_on_a_stale_verdict(self):
        criteria = ExitCriteria(cloud_verdict_timeout=30.0)
        cloud = finished_cloud(now=10.0)
        assert not criteria.evaluate(100.0, finished_progress(), cloud).met[CLOUD]

    def test_cloud_is_refused_on_thin_coverage(self):
        criteria = ExitCriteria(min_grid_coverage=0.75)
        cloud = CloudProgress()
        cloud.add(60.0, 0.30, 1_000_000, [], cloud_map_id="a", agreement=0.55)
        cloud.add(100.0, 0.31, 1_005_000, [], cloud_map_id="a", agreement=0.55)
        verdict = criteria.evaluate(100.0, finished_progress(), cloud)
        assert not verdict.met[CLOUD]
        assert "31%" in verdict.reasons[CLOUD]

    def test_cloud_is_refused_while_it_is_still_growing(self):
        criteria = ExitCriteria(max_cloud_growth=0.02)
        cloud = CloudProgress()
        cloud.add(60.0, 0.90, 1_000_000, [], cloud_map_id="a", agreement=0.55)
        cloud.add(100.0, 0.90, 1_500_000, [], cloud_map_id="a", agreement=0.55)
        assert not criteria.evaluate(100.0, finished_progress(), cloud).met[CLOUD]

    def test_cloud_is_refused_with_too_many_uncertain_regions(self):
        criteria = ExitCriteria(max_uncertain_regions=1)
        cloud = CloudProgress()
        cloud.add(60.0, 0.90, 1_000_000, [0.9, 0.9, 0.9], cloud_map_id="a", agreement=0.55)
        cloud.add(100.0, 0.90, 1_005_000, [0.9, 0.9, 0.9], cloud_map_id="a", agreement=0.55)
        assert not criteria.evaluate(100.0, finished_progress(), cloud).met[CLOUD]

    def test_cloud_can_be_switched_off_for_a_dry_run(self):
        criteria = ExitCriteria(require_cloud=False)
        verdict = criteria.evaluate(100.0, finished_progress())
        assert verdict.met[CLOUD]
        assert verdict.reasons[CLOUD] == "not required"

    def test_budget_limits_are_off_when_unset(self):
        criteria = ExitCriteria()
        verdict = criteria.evaluate(
            100.0, finished_progress(), elapsed=1e9, distance=1e9
        )
        assert not verdict.met[BUDGET]

    @pytest.mark.parametrize(
        "kwargs, call",
        [
            ({"max_duration": 60.0}, {"elapsed": 61.0}),
            ({"max_distance": 50.0}, {"distance": 51.0}),
            ({"min_battery_voltage": 11.0}, {"battery_voltage": 10.9}),
        ],
    )
    def test_each_budget_limit_trips(self, kwargs, call):
        criteria = ExitCriteria(**kwargs)
        assert criteria.evaluate(100.0, finished_progress(), **call).met[BUDGET]


class TestCompositeRule:
    """finished = (FRONTIERS or GAIN) and STABLE and CLOUD, or BUDGET."""

    def test_a_fresh_run_is_not_finished(self):
        criteria = ExitCriteria()
        verdict = criteria.evaluate(0.0, ExplorationProgress())
        assert not verdict.finished
        assert verdict.trigger is None

    def test_everything_met_finishes_the_run(self):
        criteria = quiet(ExitCriteria())
        verdict = criteria.evaluate(
            100.0, finished_progress(), finished_cloud(), elapsed=100.0
        )
        assert verdict.finished
        assert verdict.complete
        assert verdict.trigger == FRONTIERS

    def test_gain_alone_can_stand_in_for_frontiers(self):
        # The corridor behind a glass door that keeps frontier exhaustion false
        # for ever, while the map has plainly stopped filling in.
        criteria = ExitCriteria()
        criteria.note_frontiers(100.0, 5)
        verdict = criteria.evaluate(100.0, finished_progress(), finished_cloud())
        assert verdict.finished
        assert verdict.trigger == GAIN

    def test_an_unfinished_cloud_blocks_a_finished_map(self):
        # The failure this criterion exists for: a complete 2D map over a cloud
        # covering one wall is a failed T1 that looks like a successful one.
        criteria = quiet(ExitCriteria(min_grid_coverage=0.75))
        cloud = CloudProgress()
        cloud.add(60.0, 0.20, 1_000_000, [], cloud_map_id="a", agreement=0.55)
        cloud.add(100.0, 0.20, 1_005_000, [], cloud_map_id="a", agreement=0.55)
        assert not criteria.evaluate(100.0, finished_progress(), cloud).finished

    def test_an_unsettled_map_blocks_everything_else(self):
        criteria = quiet(ExitCriteria(loop_closure_settle=15.0))
        progress = finished_progress()
        progress.note_loop_closure(98.0)
        assert not criteria.evaluate(100.0, progress, finished_cloud()).finished

    def test_budget_overrides_every_other_criterion(self):
        criteria = ExitCriteria(max_duration=60.0)
        verdict = criteria.evaluate(100.0, ExplorationProgress(), elapsed=61.0)
        assert verdict.finished
        assert verdict.trigger == BUDGET

    def test_a_budget_exit_is_not_a_complete_one(self):
        # The distinction a thesis needs: "T1 ended because the reconstruction
        # stopped improving" and "T1 ran out of time" are different runs.
        criteria = ExitCriteria(max_duration=60.0)
        verdict = criteria.evaluate(100.0, ExplorationProgress(), elapsed=61.0)
        assert not verdict.complete

    def test_the_summary_names_every_criterion(self):
        criteria = ExitCriteria()
        summary = criteria.evaluate(0.0, ExplorationProgress()).summary()
        for name in CRITERIA:
            assert name in summary


class TestTravelled:
    """The odometry accumulator."""

    def test_a_missing_endpoint_contributes_nothing(self):
        assert travelled(None, (1.0, 1.0)) == 0.0
        assert travelled((1.0, 1.0), None) == 0.0

    def test_it_is_the_planar_distance(self):
        assert travelled((0.0, 0.0), (3.0, 4.0)) == pytest.approx(5.0)


class TestPlacement:
    """
    The `placed` half of CLOUD -- whether the cloud is where the map is.

    This is the failure that looks exactly like a success. Everything the 2D
    data can see says the run went well -- the grid is complete, the frontiers
    are exhausted, the map has stopped growing -- and the cloud T2 is about to
    search does not line up with the room. It surfaces an hour later as "the
    detector never sees anything", forty minutes after the decision that caused
    it. Nothing in the 2D data can catch it, which is why the criterion is here.
    """

    def test_a_misplaced_cloud_blocks_an_otherwise_finished_run(self):
        criteria = ExitCriteria(min_agreement=0.15)
        cloud = CloudProgress()
        cloud.add(60.0, 0.90, 1_000_000, [], cloud_map_id="a", agreement=0.04)
        cloud.add(100.0, 0.90, 1_005_000, [], cloud_map_id="a", agreement=0.04)
        verdict = quiet(criteria).evaluate(100.0, finished_progress(), cloud)
        assert not verdict.met[CLOUD]
        assert not verdict.finished
        assert "not placed where the map is" in verdict.reasons[CLOUD]

    def test_never_compared_is_a_different_answer_from_agreed_with_nothing(self):
        """They have different fixes, so they must not share a message."""
        criteria = ExitCriteria(min_agreement=0.15)

        never = CloudProgress()
        never.add(60.0, 0.90, 1_000_000, [], cloud_map_id="a")
        never.add(100.0, 0.90, 1_005_000, [], cloud_map_id="a")
        nothing = CloudProgress()
        nothing.add(60.0, 0.90, 1_000_000, [], cloud_map_id="a", agreement=0.0)
        nothing.add(100.0, 0.90, 1_005_000, [], cloud_map_id="a", agreement=0.0)

        never_reason = criteria.evaluate(100.0, finished_progress(), never).reasons[CLOUD]
        nothing_reason = criteria.evaluate(100.0, finished_progress(), nothing).reasons[CLOUD]
        assert "compared nothing" in never_reason
        assert "not placed" in nothing_reason
        assert never_reason != nothing_reason

    def test_a_well_placed_cloud_does_not_block(self):
        verdict = quiet(ExitCriteria()).evaluate(
            100.0, finished_progress(), finished_cloud())
        assert verdict.met[CLOUD]
        assert verdict.finished


class TestResetSettling:
    """
    T1 must not end just after the reconstruction restarted.

    The same argument as `loop_closure_settle` on the map side: a reset means
    the cloud T2 will search was built from whatever fraction of the drive came
    after it, and exiting into that ends the run with a cloud of one corner.
    Neither end had this criterion before.
    """

    def test_a_recent_reset_blocks_the_exit(self):
        criteria = ExitCriteria(cloud_reset_settle=30.0)
        cloud = CloudProgress()
        cloud.add(50.0, 0.90, 1_000_000, [], cloud_map_id="a", agreement=0.55)
        cloud.add(90.0, 0.90, 900_000, [], cloud_map_id="b", agreement=0.55)
        cloud.add(100.0, 0.90, 905_000, [], cloud_map_id="b", agreement=0.55)
        verdict = quiet(criteria).evaluate(100.0, finished_progress(), cloud)
        assert not verdict.met[CLOUD]
        assert "restarted" in verdict.reasons[CLOUD]

    def test_the_exit_is_allowed_once_it_has_settled(self):
        criteria = ExitCriteria(cloud_reset_settle=30.0)
        cloud = CloudProgress(window=300.0)
        cloud.add(10.0, 0.90, 1_000_000, [], cloud_map_id="a", agreement=0.55)
        cloud.add(20.0, 0.90, 900_000, [], cloud_map_id="b", agreement=0.55)
        cloud.add(100.0, 0.90, 905_000, [], cloud_map_id="b", agreement=0.55)
        verdict = quiet(criteria).evaluate(100.0, finished_progress(), cloud)
        assert verdict.met[CLOUD]

    def test_a_run_with_no_reset_is_never_blocked_by_this(self):
        verdict = quiet(ExitCriteria()).evaluate(
            100.0, finished_progress(), finished_cloud())
        assert verdict.met[CLOUD]


class TestTheFloor:
    """
    A run that has not started yet must not be able to finish.

    Every quality criterion can be satisfied within seconds of start-up by a
    robot that has not moved: no frontiers found yet, no movement to measure
    gain over, a map that is trivially not growing. The server carried
    minimums for this; this end had none.
    """

    def test_a_run_that_has_barely_started_cannot_finish(self):
        criteria = ExitCriteria(min_runtime=60.0)
        verdict = quiet(criteria).evaluate(
            100.0, finished_progress(), finished_cloud(), elapsed=5.0)
        assert not verdict.finished
        assert "too early" in verdict.reasons[BUDGET]

    def test_past_the_floor_the_criteria_decide_again(self):
        criteria = ExitCriteria(min_runtime=60.0)
        verdict = quiet(criteria).evaluate(
            100.0, finished_progress(), finished_cloud(), elapsed=120.0)
        assert verdict.finished

    def test_the_budget_still_overrides_the_floor(self):
        """An unattended run has to be able to stop, floor or no floor."""
        criteria = ExitCriteria(min_runtime=600.0, max_duration=100.0)
        verdict = criteria.evaluate(
            100.0, finished_progress(), finished_cloud(), elapsed=150.0)
        assert verdict.finished
        assert verdict.trigger == BUDGET
        assert not verdict.complete
