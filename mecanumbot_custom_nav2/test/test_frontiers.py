"""
Tests for clustering, revalidating and scoring the detector's candidates.

The scoring is the only discretionary step in the exploration pipeline, so the
tests pin the behaviour that the constants are meant to produce: the near
frontier wins on a tie, a big enough gain overcomes distance, the goal already
being driven to is not abandoned for an equal one, and the single-cell
boundaries that line every rasterised wall are thrown away.
"""

import numpy as np
import pytest

from mecanumbot_custom_nav2.frontiers import (
    Frontier,
    best,
    cluster,
    revalidate,
    score,
    still_a_frontier,
)
from mecanumbot_custom_nav2.occupancy import UNKNOWN, Grid


def half_open(size=40, resolution=0.5):
    """Left half observed and free, right half never observed. One long frontier."""
    data = np.zeros((size, size), dtype=np.int16)
    data[:, size // 2 :] = UNKNOWN
    return Grid(data, resolution, 0.0, 0.0)


def all_free(size=40, resolution=0.5):
    """Build a fully observed grid, where no frontier survives revalidation."""
    return Grid(np.zeros((size, size), dtype=np.int16), resolution, 0.0, 0.0)


class TestClustering:
    """Turning repeated detections of one frontier into one frontier."""

    def test_nearby_points_become_one(self):
        clustered = cluster([(1.0, 1.0), (1.1, 1.0), (1.0, 1.2)], radius=0.5)
        assert len(clustered) == 1
        assert clustered[0].support == 3

    def test_distant_points_stay_apart(self):
        assert len(cluster([(1.0, 1.0), (5.0, 5.0)], radius=0.5)) == 2

    def test_the_centroid_is_the_running_mean(self):
        clustered = cluster([(0.0, 0.0), (1.0, 0.0)], radius=2.0)
        assert clustered[0].x == pytest.approx(0.5)

    def test_an_empty_batch_clusters_to_nothing(self):
        assert cluster([], radius=0.5) == []

    def test_the_radius_is_inclusive(self):
        assert len(cluster([(0.0, 0.0), (0.5, 0.0)], radius=0.5)) == 1


class TestRevalidation:
    """Dropping the candidates the map filled in while the detector ran."""

    def test_a_real_boundary_survives(self):
        grid = half_open()
        # x = 10.0 is the last free column at 0.5 m resolution over 40 cells.
        assert still_a_frontier(grid, Frontier(9.75, 5.0), radius=0.4)

    def test_a_filled_in_candidate_is_dropped(self):
        assert not still_a_frontier(all_free(), Frontier(5.0, 5.0), radius=0.4)

    def test_revalidate_filters_the_list(self):
        grid = half_open()
        kept = revalidate(
            grid, [Frontier(9.75, 5.0), Frontier(3.0, 3.0)], radius=0.4
        )
        assert len(kept) == 1
        assert kept[0].x == pytest.approx(9.75)


class TestScoring:
    """What a frontier is worth."""

    def test_the_nearer_of_two_equals_wins(self):
        grid = half_open()
        scored = score(
            grid,
            [Frontier(9.75, 5.0), Frontier(9.75, 15.0)],
            robot_xy=(5.0, 5.0),
            min_gain=0,
        )
        assert scored[0].y == pytest.approx(5.0)

    def test_distance_is_paid_for_at_the_cost_weight(self):
        grid = half_open()
        near, far = Frontier(9.75, 5.0), Frontier(9.75, 15.0)
        scored = score(
            grid, [near, far], robot_xy=(9.75, 5.0), min_gain=0, cost_weight=0.4
        )
        gap = scored[0].score - scored[1].score
        # Same normalized gain along a uniform boundary, so the whole difference
        # is the 10 m detour at 0.4 per metre.
        assert gap == pytest.approx(4.0, abs=0.35)

    def test_a_trivial_frontier_is_dropped(self):
        # The single-cell boundaries that line every rasterised wall, which can
        # never be resolved because the thing behind them is a wall.
        grid = half_open()
        assert score(grid, [Frontier(9.75, 5.0)], (5.0, 5.0), min_gain=10_000) == []

    def test_gain_is_normalized_within_the_batch(self):
        grid = half_open()
        scored = score(grid, [Frontier(9.75, 5.0)], (9.75, 5.0), min_gain=0)
        # One frontier is the largest gain in its own batch, so the gain term is
        # exactly gain_weight and the distance term is zero.
        assert scored[0].score == pytest.approx(1.0)

    def test_hysteresis_holds_on_to_the_current_goal(self):
        grid = half_open()
        current = Frontier(9.75, 15.0)
        scored = score(
            grid,
            [Frontier(9.75, 5.0), current],
            robot_xy=(9.75, 5.0),
            min_gain=0,
            cost_weight=0.0,
            current_goal=(9.75, 15.0),
            hysteresis=0.5,
        )
        # With no distance cost the two are equal on merit, and the one already
        # being driven to has to win -- otherwise the explorer re-decides at its
        # tick rate and drives to neither.
        assert scored[0].y == pytest.approx(15.0)

    def test_hysteresis_does_not_hold_on_to_a_much_worse_goal(self):
        grid = half_open()
        scored = score(
            grid,
            [Frontier(9.75, 5.0), Frontier(9.75, 35.0)],
            robot_xy=(9.75, 5.0),
            min_gain=0,
            cost_weight=0.4,
            current_goal=(9.75, 35.0),
            hysteresis=0.25,
        )
        assert scored[0].y == pytest.approx(5.0)

    def test_the_list_comes_back_sorted(self):
        grid = half_open()
        scored = score(
            grid,
            [Frontier(9.75, 35.0), Frontier(9.75, 5.0), Frontier(9.75, 20.0)],
            robot_xy=(9.75, 5.0),
            min_gain=0,
        )
        assert scored == sorted(scored, key=lambda f: f.score, reverse=True)

    def test_an_empty_list_scores_to_nothing(self):
        assert score(half_open(), [], (5.0, 5.0)) == []


class TestPipeline:
    """The whole thing, as the node calls it."""

    def test_best_returns_a_frontier_and_the_full_list(self):
        grid = half_open()
        points = [(9.75, y) for y in (5.0, 5.1, 15.0)]
        chosen, scored = best(grid, points, robot_xy=(5.0, 5.0), min_gain=0)
        assert chosen is not None
        assert chosen in scored
        assert len(scored) == 2  # the first two clustered into one

    def test_nothing_survives_on_a_fully_observed_map(self):
        chosen, scored = best(all_free(), [(5.0, 5.0)], robot_xy=(1.0, 1.0))
        assert chosen is None
        assert scored == []

    def test_no_candidates_means_no_goal(self):
        chosen, scored = best(half_open(), [], robot_xy=(5.0, 5.0))
        assert chosen is None
        assert scored == []
