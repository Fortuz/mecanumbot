"""
Tests for what the robot does with the server's comparison verdict.

The decisions here add lethal cells to a costmap, so the tests are written
around the two ways that goes wrong: marking something the robot could have
driven under, and *not* marking something it will hit. The height bands are
pinned exactly, and so is the rule that a `map_only` region never subtracts
anything -- the reconstruction must not be able to talk the robot into driving
somewhere the lidar says it cannot.
"""

import pytest

from mecanumbot_custom_nav2.agreement import (
    CLOUD_ONLY,
    DISAGREEMENT,
    FREE,
    IGNORE,
    KEEPOUT,
    LETHAL,
    MAP_ONLY,
    REVISIT,
    UNOBSERVED,
    AgreementModel,
    Region,
    stamp_mask,
)


def model(**kwargs):
    """Build a handler model with the packaged envelope, overridden as needed."""
    settings = dict(
        clearance=0.03,
        lidar_height=0.20,
        robot_height=0.45,
        min_keepout_score=0.4,
        min_revisit_score=0.5,
    )
    settings.update(kwargs)
    return AgreementModel(**settings)


def verdict(handler, now=0.0, **region):
    """Push a single-region verdict through the model and return that region."""
    settings = dict(x=1.0, y=1.0, score=0.9, kind=CLOUD_ONLY, height=0.10, radius=0.5)
    settings.update(region)
    handler.update(
        now,
        [(settings["x"], settings["y"])],
        [settings["score"]],
        kinds=[settings["kind"]],
        heights=[settings["height"]],
        radii=[settings["radius"]],
        map_id="a",
    )
    return handler.regions[-1]


class TestHeightBands:
    """Whether structure of a given height is something the robot would hit."""

    @pytest.mark.parametrize(
        "height, blocks",
        [
            (0.005, False),  # a cable -- the robot rolls over it
            (0.03, True),  # exactly at the clearance
            (0.10, True),  # a step: below the lidar plane, too high to cross
            (0.20, True),  # at the lidar plane
            (0.45, True),  # exactly at the robot's height
            (0.75, False),  # a table top -- the robot drives under it
            (2.10, False),  # a doorframe
        ],
    )
    def test_the_band_is_the_robot_envelope(self, height, blocks):
        assert model().blocks(height) is blocks

    def test_an_unknown_height_counts_as_blocking(self):
        # The server said there is structure and could not say how tall.
        # Assuming an overhang is the one reading that drives the robot into it.
        assert model().blocks(None) is True

    def test_the_lidar_blind_band_is_below_the_plane(self):
        handler = model()
        assert handler.invisible_to_lidar(0.10)
        assert not handler.invisible_to_lidar(0.30)  # the lidar should have seen it
        assert not handler.invisible_to_lidar(0.005)  # rolled over, not missed

    def test_an_unknown_height_is_not_claimed_to_be_lidar_blind(self):
        # Blocking is the cautious default; claiming to know *why* the lidar
        # missed it would be inventing information.
        assert model().invisible_to_lidar(None) is False


class TestActions:
    """What each kind of disagreement makes the robot do."""

    def test_cloud_structure_in_the_way_is_a_keepout(self):
        handler = model()
        region = verdict(handler, kind=CLOUD_ONLY, height=0.10, score=0.9)
        assert handler.action_for(region) == KEEPOUT

    def test_cloud_structure_overhead_is_ignored(self):
        handler = model()
        region = verdict(handler, kind=CLOUD_ONLY, height=0.75, score=0.9)
        assert handler.action_for(region) == IGNORE

    def test_a_map_only_region_never_becomes_a_keepout(self):
        # The lidar sees something the camera could not reconstruct -- glass, a
        # thin dark panel. The map already has it, and nothing here subtracts.
        handler = model()
        region = verdict(handler, kind=MAP_ONLY, height=0.10, score=0.99)
        assert handler.action_for(region) == REVISIT

    def test_a_map_only_region_below_the_revisit_score_is_ignored(self):
        handler = model()
        region = verdict(handler, kind=MAP_ONLY, score=0.1)
        assert handler.action_for(region) == IGNORE

    def test_an_unobserved_region_is_only_ever_a_revisit(self):
        handler = model()
        region = verdict(handler, kind=UNOBSERVED, height=0.0, score=0.9)
        assert handler.action_for(region) == REVISIT

    def test_a_low_confidence_blocking_region_is_looked_at_not_blocked(self):
        handler = model(min_keepout_score=0.8, min_revisit_score=0.4)
        region = verdict(handler, kind=CLOUD_ONLY, height=0.10, score=0.5)
        assert handler.action_for(region) == REVISIT

    def test_a_region_below_both_thresholds_is_ignored(self):
        handler = model(min_keepout_score=0.8, min_revisit_score=0.6)
        region = verdict(handler, kind=CLOUD_ONLY, height=0.10, score=0.2)
        assert handler.action_for(region) == IGNORE

    def test_an_unknown_kind_is_treated_as_a_plain_disagreement(self):
        region = Region(0.0, 0.0, 0.9, kind="something_new", height=0.1)
        assert region.kind == DISAGREEMENT
        assert model().action_for(region) == KEEPOUT


class TestAccumulation:
    """A verdict is a snapshot, so regions have to be kept between them."""

    def test_a_region_survives_a_verdict_that_omits_it(self):
        handler = model(forget_after=120.0)
        verdict(handler, now=0.0, x=1.0, y=1.0)
        handler.update(10.0, [], [], map_id="a")
        assert len(handler.regions) == 1

    def test_a_region_is_forgotten_once_the_server_goes_quiet(self):
        handler = model(forget_after=30.0)
        verdict(handler, now=0.0, x=1.0, y=1.0)
        handler.update(100.0, [], [], map_id="a")
        assert handler.regions == []

    def test_nearby_reports_merge_into_one_region(self):
        handler = model(merge_radius=1.0)
        verdict(handler, now=0.0, x=1.0, y=1.0)
        verdict(handler, now=1.0, x=1.4, y=1.0)
        assert len(handler.regions) == 1

    def test_distant_reports_stay_separate(self):
        handler = model(merge_radius=0.5)
        verdict(handler, now=0.0, x=1.0, y=1.0)
        verdict(handler, now=1.0, x=5.0, y=5.0)
        assert len(handler.regions) == 2

    def test_a_re_report_updates_the_height(self):
        handler = model(merge_radius=1.0)
        verdict(handler, now=0.0, x=1.0, y=1.0, height=0.75)
        region = verdict(handler, now=1.0, x=1.0, y=1.0, height=0.10)
        assert region.height == pytest.approx(0.10)
        assert handler.action_for(region) == KEEPOUT

    def test_a_new_map_id_throws_everything_away(self):
        # CUT3R anchors its world frame on the first frame of a map_id, so
        # regions from the old one would be placed at random.
        handler = model()
        verdict(handler, now=0.0, x=1.0, y=1.0)
        handler.update(1.0, [(2.0, 2.0)], [0.9], kinds=[CLOUD_ONLY], map_id="b")
        assert len(handler.regions) == 1
        assert handler.regions[0].point == (2.0, 2.0)


class TestServicing:
    """The robot having been to look."""

    def test_being_near_a_region_marks_it_serviced(self):
        handler = model(serviced_radius=1.0)
        verdict(handler, kind=UNOBSERVED, x=1.0, y=1.0, score=0.9)
        handler.mark_serviced(1.5, 1.0)
        assert handler.regions[0].serviced

    def test_being_far_away_does_not(self):
        handler = model(serviced_radius=1.0)
        verdict(handler, kind=UNOBSERVED, x=1.0, y=1.0, score=0.9)
        handler.mark_serviced(9.0, 9.0)
        assert not handler.regions[0].serviced

    def test_a_serviced_region_drops_off_the_revisit_list(self):
        handler = model()
        verdict(handler, kind=UNOBSERVED, x=1.0, y=1.0, score=0.9)
        assert len(handler.revisits()) == 1
        handler.mark_serviced(1.0, 1.0)
        assert handler.revisits() == []

    def test_a_re_report_does_not_send_the_robot_back(self):
        # Servicing is a fact about the robot, not about the verdict.
        handler = model(merge_radius=1.0)
        verdict(handler, kind=UNOBSERVED, x=1.0, y=1.0, score=0.9)
        handler.mark_serviced(1.0, 1.0)
        verdict(handler, kind=UNOBSERVED, x=1.0, y=1.0, score=0.9)
        assert handler.regions[0].serviced
        assert handler.revisits() == []

    def test_a_keepout_is_never_a_revisit_target(self):
        # Driving up to something the robot has been told it would hit, to have
        # a better look at it, is the one response that cannot be right.
        handler = model()
        verdict(handler, kind=CLOUD_ONLY, height=0.10, score=0.9)
        assert handler.keepouts()
        assert handler.revisits() == []

    def test_revisits_come_back_best_first(self):
        handler = model()
        verdict(handler, kind=UNOBSERVED, x=1.0, y=1.0, score=0.6)
        verdict(handler, kind=UNOBSERVED, x=5.0, y=5.0, score=0.9)
        assert [r.score for r in handler.revisits()] == [0.9, 0.6]


class TestShortArrays:
    """A server that predates the per-region fields still has to work."""

    def test_missing_kinds_heights_and_radii_are_defaulted(self):
        handler = model()
        handler.update(0.0, [(1.0, 1.0)], [0.9], map_id="a")
        region = handler.regions[0]
        assert region.kind == DISAGREEMENT
        # Unknown height, so cautious: it becomes a keepout rather than being
        # waved through as an overhang.
        assert handler.action_for(region) == KEEPOUT

    def test_a_short_array_does_not_raise(self):
        handler = model()
        handler.update(
            0.0,
            [(1.0, 1.0), (2.0, 2.0)],
            [0.9, 0.9],
            kinds=[CLOUD_ONLY],  # only one entry for two points
            heights=[],
            map_id="a",
        )
        assert len(handler.regions) == 2

    def test_a_none_in_an_array_falls_back_to_the_default(self):
        handler = model()
        handler.update(
            0.0, [(1.0, 1.0)], [0.9], kinds=[None], heights=[None], map_id="a"
        )
        assert handler.regions[0].kind == DISAGREEMENT


class TestMask:
    """Rasterising keepouts onto a grid nav2 can read."""

    def test_a_region_stamps_lethal_cells(self):
        data = stamp_mask(
            20, 20, 0.5, 0.0, 0.0, [Region(5.0, 5.0, 0.9, radius=1.0)]
        )
        # (5.0, 5.0) at 0.5 m resolution from origin 0 is cell (10, 10).
        assert data[10 * 20 + 10] == LETHAL

    def test_everything_else_stays_free(self):
        data = stamp_mask(
            20, 20, 0.5, 0.0, 0.0, [Region(5.0, 5.0, 0.9, radius=0.5)]
        )
        assert data[0] == FREE
        assert data.count(LETHAL) < 20 * 20

    def test_no_regions_gives_an_empty_mask(self):
        data = stamp_mask(10, 10, 0.5, 0.0, 0.0, [])
        assert set(data) == {FREE}
        assert len(data) == 100

    def test_a_region_off_the_edge_is_clipped_not_refused(self):
        # The map grows underneath a verdict all the time; a keepout the map has
        # not reached yet is not an error.
        data = stamp_mask(
            10, 10, 0.5, 0.0, 0.0, [Region(-50.0, -50.0, 0.9, radius=1.0)]
        )
        assert set(data) == {FREE}

    def test_the_origin_offsets_the_stamp(self):
        data = stamp_mask(
            20, 20, 0.5, -5.0, -5.0, [Region(0.0, 0.0, 0.9, radius=0.3)]
        )
        assert data[10 * 20 + 10] == LETHAL

    def test_a_bigger_radius_stamps_more_cells(self):
        small = stamp_mask(40, 40, 0.5, 0.0, 0.0, [Region(10.0, 10.0, 0.9, radius=0.5)])
        large = stamp_mask(40, 40, 0.5, 0.0, 0.0, [Region(10.0, 10.0, 0.9, radius=2.0)])
        assert large.count(LETHAL) > small.count(LETHAL)


class TestSummary:
    """The log line."""

    def test_counts_add_up_to_the_regions_held(self):
        handler = model()
        verdict(handler, x=1.0, y=1.0, kind=CLOUD_ONLY, height=0.1, score=0.9)
        verdict(handler, x=5.0, y=5.0, kind=UNOBSERVED, height=0.0, score=0.9)
        actions, kinds = handler.summary()
        assert sum(actions.values()) == len(handler.regions)
        assert sum(kinds.values()) == len(handler.regions)
        assert actions[KEEPOUT] == 1
        assert actions[REVISIT] == 1


def test_regions_survive_the_reconstruction_restarting():
    """
    A cloud reset does not move a map-frame coordinate.

    This reset used to be keyed on CUT3R's session id, back when the message
    called it `map_id`. It is the wrong event: the server re-derives its
    alignment after a reset and keeps publishing in the map frame, so "a table
    top is at (3.2, 1.4)" is still true. Throwing the keepouts away there means
    the robot forgets every obstacle it cannot see every time the model resets.
    """
    model = AgreementModel()
    model.update(0.0, [(3.2, 1.4)], [0.9], kinds=["cloud_only"], heights=[0.74],
                 map_id="slam-map-1")
    assert len(model.regions) == 1

    # Same robot map, whatever the reconstruction did.
    model.update(1.0, [], [], map_id="slam-map-1")
    assert len(model.regions) == 1


def test_regions_are_dropped_when_the_robot_s_map_restarts():
    """The event that really does invalidate a map-frame coordinate."""
    model = AgreementModel()
    model.update(0.0, [(3.2, 1.4)], [0.9], kinds=["cloud_only"], heights=[0.74],
                 map_id="slam-map-1")
    assert len(model.regions) == 1

    model.update(1.0, [], [], map_id="slam-map-2")
    assert model.regions == []


def test_regions_of_different_kinds_are_never_merged():
    """
    A table beside a glass panel must not become one region.

    `cloud_only` and `map_only` are opposite decisions -- make it lethal, leave
    it alone -- and they routinely occur within a merge radius of each other,
    because a table pushed against a glass partition is exactly that. Merging
    them let the newest report pick the kind, so the same two surfaces produced
    a keepout or a collision depending on which verdict arrived last. The
    server clusters within a kind (`robocam/regions.py`); this end used to
    undo it.
    """
    model = AgreementModel(merge_radius=1.0)
    model.update(0.0, [(1.0, 1.0)], [0.9], kinds=["cloud_only"], heights=[0.30],
                 map_id="m")
    model.update(0.1, [(1.1, 1.0)], [0.9], kinds=["map_only"], heights=[float("nan")],
                 map_id="m")

    kinds = sorted(r.kind for r in model.regions)
    assert kinds == ["cloud_only", "map_only"]
    # And the table is still a keepout, not talked out of it by the glass.
    assert len(model.keepouts()) == 1


def test_the_same_kind_still_merges():
    """The merge is not disabled, only confined."""
    model = AgreementModel(merge_radius=1.0)
    model.update(0.0, [(1.0, 1.0)], [0.9], kinds=["cloud_only"], heights=[0.30],
                 map_id="m")
    model.update(0.1, [(1.1, 1.0)], [0.9], kinds=["cloud_only"], heights=[0.32],
                 map_id="m")
    assert len(model.regions) == 1


class TestTheMaskDoesNotChurn:
    """
    An unchanged keepout mask must not be republished.

    The mask is latched and nav2's KeepoutFilter re-inflates the costmap on
    every publish. Republishing an identical one at the verdict rate makes the
    costmap flicker, and the planner replans through a flickering costmap --
    which on the robot reads as driving in short bursts with stops between
    them, and looks nothing like a costmap problem. These tests are on the
    model rather than the node because the node needs a ROS graph; what they
    pin is the property the node's check depends on: that a repeated verdict
    produces a byte-identical set of keepouts.
    """

    def a_table(self, model, now, x=1.0):
        model.update(now, [(x, 1.0)], [0.9], kinds=["cloud_only"],
                     heights=[0.30], radii=[0.4], map_id="m")

    def test_a_repeated_verdict_leaves_the_keepouts_identical(self):
        model = AgreementModel()
        self.a_table(model, 0.0)
        first = [(r.x, r.y, r.height) for r in model.keepouts()]
        self.a_table(model, 1.0)
        self.a_table(model, 2.0)
        assert [(r.x, r.y, r.height) for r in model.keepouts()] == first

    def test_a_new_obstacle_does_change_them(self):
        """The check must not be so eager that a real change is swallowed."""
        model = AgreementModel()
        self.a_table(model, 0.0)
        before = len(model.keepouts())
        self.a_table(model, 1.0, x=5.0)
        assert len(model.keepouts()) == before + 1
