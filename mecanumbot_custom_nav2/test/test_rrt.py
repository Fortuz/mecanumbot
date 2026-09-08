"""
Tests for the sampling-based frontier detector.

The detector is stochastic, so the tests fix the seed and assert on properties
that hold for any seed -- the tree stays inside free space, a closed room yields
nothing, a room with a gap yields the gap -- rather than on particular points.
The one thing worth pinning exactly is that the tree never grows through a wall,
because a detector that does produces goals nav2 can never reach and the symptom
is a robot that simply stops moving.
"""

import random

import numpy as np
import pytest

from mecanumbot_custom_nav2.occupancy import UNKNOWN, Grid, line_is_clear
from mecanumbot_custom_nav2.rrt import FrontierSearch, RRTDetector


def closed_room(size=20, resolution=0.5):
    """Build a fully observed square room walled all round, so it has no frontiers."""
    data = np.zeros((size, size), dtype=np.int16)
    data[0, :] = data[-1, :] = 100
    data[:, 0] = data[:, -1] = 100
    return Grid(data, resolution, 0.0, 0.0)


def room_with_a_corridor(size=20, resolution=0.5):
    """
    Build the same room with the right-hand third unobserved behind a doorway.

    A wall runs down the middle with a two-cell gap in it, and everything past
    the wall is unknown. The gap is the only frontier, and finding it is the job.
    """
    grid = closed_room(size, resolution)
    middle = size // 2
    grid.data[:, middle] = 100
    grid.data[middle - 1 : middle + 1, middle] = 0  # the doorway
    grid.data[:, middle + 1 :] = UNKNOWN
    return grid


class TestGrowth:
    """One tree, growing."""

    def test_the_tree_starts_as_a_single_node(self):
        tree = RRTDetector((5.0, 5.0))
        assert tree.size == 1
        assert list(tree.edges()) == []

    def test_growing_adds_vertices(self):
        tree = RRTDetector((5.0, 5.0), rng=random.Random(1))
        tree.grow_many(closed_room(), 100)
        assert tree.size > 1

    def test_every_edge_stays_in_free_space(self):
        # The property the whole detector depends on: a branch through a wall
        # is a proposal to drive somewhere no route exists to.
        grid = room_with_a_corridor()
        tree = RRTDetector((3.0, 3.0), step_size=1.0, rng=random.Random(7))
        tree.grow_many(grid, 400)
        for parent, child in tree.edges():
            assert line_is_clear(grid, parent, child), f"{parent} -> {child}"

    def test_steering_never_exceeds_the_step_size(self):
        grid = closed_room()
        tree = RRTDetector((5.0, 5.0), step_size=0.8, rng=random.Random(3))
        tree.grow_many(grid, 200)
        for parent, child in tree.edges():
            assert np.hypot(child[0] - parent[0], child[1] - parent[1]) <= 0.8 + 1e-9

    def test_reset_returns_to_a_single_node(self):
        tree = RRTDetector((5.0, 5.0), rng=random.Random(2))
        tree.grow_many(closed_room(), 100)
        tree.reset((5.0, 5.0))
        assert tree.size == 1


class TestDetection:
    """What the tree reports."""

    def test_a_closed_room_has_no_frontiers(self):
        # Every branch that stops, stops at a wall. Reporting one here is the
        # failure that keeps an explorer driving at a wall for ever.
        grid = closed_room()
        tree = RRTDetector((5.0, 5.0), rng=random.Random(11))
        assert tree.grow_many(grid, 600) == []

    def test_a_doorway_is_found(self):
        grid = room_with_a_corridor()
        tree = RRTDetector((3.0, 3.0), step_size=1.0, rng=random.Random(5))
        found = tree.grow_many(grid, 1500)
        assert found, "the detector never found the doorway"

    def test_every_detection_is_on_the_boundary(self):
        grid = room_with_a_corridor()
        tree = RRTDetector((3.0, 3.0), step_size=1.0, rng=random.Random(13))
        for x, y in tree.grow_many(grid, 1500):
            column, row = grid.to_cell(x, y)
            assert grid.is_free(column, row)
            assert any(
                grid.is_unknown(column + dc, row + dr)
                for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1))
            )

    def test_a_local_tree_restarts_after_a_detection(self):
        grid = room_with_a_corridor()
        tree = RRTDetector(
            (3.0, 3.0), step_size=1.0, reset_on_detection=True, rng=random.Random(5)
        )
        found = []
        for _ in range(1500):
            point = tree.grow(grid)
            if point is not None:
                found.append(point)
                assert tree.size == 1
        assert found

    def test_a_global_tree_keeps_growing_through_detections(self):
        grid = room_with_a_corridor()
        tree = RRTDetector((3.0, 3.0), step_size=1.0, rng=random.Random(5))
        tree.grow_many(grid, 1500)
        assert tree.size > 10


class TestFrontierSearch:
    """The pair of trees the node actually runs."""

    def test_a_step_returns_points_and_builds_both_trees(self):
        grid = room_with_a_corridor()
        search = FrontierSearch(
            step_size=1.0, global_iterations=400, local_iterations=200, seed=4
        )
        found = []
        for _ in range(8):
            found += search.step(grid, (3.0, 3.0))
        assert search.global_tree is not None
        assert search.local_tree is not None
        assert found

    def test_the_local_tree_is_re_rooted_at_the_robot_every_step(self):
        grid = closed_room()
        search = FrontierSearch(step_size=1.0, seed=4)
        search.step(grid, (3.0, 3.0))
        search.step(grid, (7.0, 7.0))
        assert search.local_tree.root == (7.0, 7.0)

    def test_the_global_tree_keeps_its_root(self):
        grid = closed_room()
        search = FrontierSearch(step_size=1.0, seed=4)
        search.step(grid, (3.0, 3.0))
        search.step(grid, (7.0, 7.0))
        assert search.global_tree.root == (3.0, 3.0)

    def test_the_local_tree_samples_only_nearby(self):
        grid = closed_room(size=40)
        search = FrontierSearch(step_size=1.0, local_radius=3.0, seed=4)
        search.step(grid, (10.0, 10.0))
        for x, y in search.local_tree.vertices:
            # Sampling is bounded; a branch may still end one step outside it.
            assert abs(x - 10.0) <= 3.0 + 1.0 + 1e-9
            assert abs(y - 10.0) <= 3.0 + 1.0 + 1e-9

    def test_restart_throws_both_trees_away(self):
        # What happens after a loop closure: the free space a tree was grown
        # through may have moved out from under it.
        grid = closed_room()
        search = FrontierSearch(step_size=1.0, seed=4)
        search.step(grid, (3.0, 3.0))
        search.restart((5.0, 5.0))
        assert search.local_tree is None
        assert search.global_tree.size == 1
        assert search.global_tree.root == (5.0, 5.0)

    def test_a_seed_makes_a_run_reproducible(self):
        grid = room_with_a_corridor()
        runs = []
        for _ in range(2):
            search = FrontierSearch(step_size=1.0, seed=99)
            runs.append(
                [p for _ in range(5) for p in search.step(grid, (3.0, 3.0))]
            )
        assert runs[0] == runs[1]


class TestDegenerateMaps:
    """The maps a detector meets before the robot has driven anywhere."""

    def test_an_entirely_unobserved_map_yields_nothing(self):
        # No free space to grow through at all: the first cycle after start-up,
        # before slam_toolbox has published anything useful.
        grid = Grid(np.full((20, 20), UNKNOWN, dtype=np.int16), 0.5, 0.0, 0.0)
        tree = RRTDetector((5.0, 5.0), rng=random.Random(1))
        assert tree.grow_many(grid, 200) == []
        assert tree.size == 1

    def test_a_robot_outside_the_map_does_not_crash(self):
        grid = closed_room()
        tree = RRTDetector((-50.0, -50.0), rng=random.Random(1))
        assert tree.grow_many(grid, 100) == []

    def test_a_fully_free_map_has_frontiers_only_at_its_edge(self):
        grid = Grid(np.zeros((20, 20), dtype=np.int16), 0.5, 0.0, 0.0)
        tree = RRTDetector((5.0, 5.0), step_size=1.0, rng=random.Random(6))
        found = tree.grow_many(grid, 800)
        for x, y in found:
            column, row = grid.to_cell(x, y)
            assert column in (0, grid.width - 1) or row in (0, grid.height - 1)


@pytest.mark.parametrize("seed", [1, 2, 3, 4, 5])
def test_no_seed_finds_a_frontier_in_a_closed_room(seed):
    """A closed room has no frontiers whatever the sampling happens to do."""
    assert RRTDetector((5.0, 5.0), rng=random.Random(seed)).grow_many(
        closed_room(), 400
    ) == []
