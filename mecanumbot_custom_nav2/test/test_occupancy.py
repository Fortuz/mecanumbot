"""
Tests for the occupancy-grid reader.

Everything the frontier detector believes about the world comes through this
module, and a sign error here produces a plausible-looking exploration that
drives into walls, so the world/cell conversion and the frontier test are
checked against grids drawn out by hand.
"""

import numpy as np
import pytest

from mecanumbot_custom_nav2.occupancy import (
    FREE,
    Grid,
    OCCUPIED,
    UNCERTAIN,
    UNKNOWN,
    bresenham,
    first_obstruction,
    is_frontier_cell,
    line_is_clear,
    nearest_frontier_cell,
)

# A 5x5 room: the left two columns observed and free, the middle column a wall,
# the right two columns never observed.
#
#   row 4 | 0 0 100 -1 -1
#   row 3 | 0 0 100 -1 -1
#   row 2 | 0 0 100 -1 -1
#   row 1 | 0 0 100 -1 -1
#   row 0 | 0 0 100 -1 -1
#           c0 c1 c2  c3 c4
ROOM = np.array([[0, 0, 100, -1, -1]] * 5, dtype=np.int16)


def room(resolution=1.0, origin_x=0.0, origin_y=0.0):
    """Build the hand-drawn room as a Grid."""
    return Grid(ROOM.copy(), resolution, origin_x, origin_y)


def open_room(width=10, height=10, resolution=1.0):
    """Build an entirely unobserved grid, for tests that fill it in themselves."""
    return Grid(np.full((height, width), UNKNOWN, dtype=np.int16), resolution, 0.0, 0.0)


class TestCellConversion:
    """World points to cells and back."""

    def test_origin_is_the_first_cell(self):
        assert room().to_cell(0.0, 0.0) == (0, 0)

    def test_cell_centres_round_trip(self):
        grid = room(resolution=0.5, origin_x=-1.0, origin_y=2.0)
        for column, row in ((0, 0), (3, 4), (2, 1)):
            assert grid.to_cell(*grid.to_world(column, row)) == (column, row)

    def test_origin_offsets_the_whole_grid(self):
        grid = room(resolution=1.0, origin_x=-2.0, origin_y=-2.0)
        assert grid.to_cell(-2.0, -2.0) == (0, 0)
        assert grid.to_cell(0.0, 0.0) == (2, 2)

    def test_a_point_just_below_the_origin_is_outside(self):
        # floor() and not round(): a point 1 cm below the origin belongs to the
        # cell before the grid starts, not to the first one.
        assert room().to_cell(-0.01, -0.01) == (-1, -1)

    def test_contains_rejects_indices_off_every_edge(self):
        grid = room()
        assert grid.contains(0, 0)
        assert grid.contains(4, 4)
        for column, row in ((-1, 0), (0, -1), (5, 0), (0, 5)):
            assert not grid.contains(column, row)

    def test_bounds_span_the_whole_grid(self):
        grid = room(resolution=0.5, origin_x=1.0, origin_y=-1.0)
        assert grid.bounds == pytest.approx((1.0, -1.0, 3.5, 1.5))


class TestClassification:
    """Raw occupancy values into free, occupied and uncertain."""

    def test_the_three_classes(self):
        grid = room()
        assert grid.classify(0, 0) == FREE
        assert grid.classify(2, 0) == OCCUPIED
        assert grid.classify(3, 0) == UNCERTAIN

    def test_outside_the_grid_is_unknown(self):
        # Not merely uncertain: an explorer has to be able to grow towards the
        # space beyond the edge of the current map, because that is where the
        # map grows.
        grid = room()
        assert grid.value(-1, -1) == UNKNOWN
        assert grid.is_unknown(-1, -1)

    def test_thresholds_are_the_callers(self):
        grid = Grid(np.array([[40]], dtype=np.int16), 1.0, 0.0, 0.0)
        assert grid.classify(0, 0) == UNCERTAIN
        grid.free_threshold = 50
        assert grid.classify(0, 0) == FREE

    def test_counts_split_the_grid_three_ways(self):
        free, occupied, unknown = room().counts()
        assert (free, occupied, unknown) == (10, 5, 10)
        assert free + occupied + unknown == 25

    def test_known_cells_excludes_only_the_unobserved(self):
        assert room().known_cells() == 15


class TestInformationGain:
    """Counting unobserved cells around a point."""

    def test_gain_is_zero_in_fully_observed_space(self):
        grid = Grid(np.zeros((10, 10), dtype=np.int16), 1.0, 0.0, 0.0)
        assert grid.unknown_within(5.0, 5.0, 3.0) == 0

    def test_gain_counts_the_unobserved_side(self):
        # Standing in the free half of the room, a radius that reaches past the
        # wall should find the unobserved columns.
        assert room().unknown_within(1.5, 2.5, 3.0) > 0

    def test_gain_is_masked_to_a_disc_not_a_square(self):
        # A square window over-counts the corners, which makes the gain of a
        # frontier depend on which way the nearest wall happens to lie.
        grid = open_room(21, 21)
        gain = grid.unknown_within(10.5, 10.5, 5.0)
        assert gain < 11 * 11  # the square window
        assert gain == pytest.approx(np.pi * 25, rel=0.15)

    def test_gain_near_the_edge_does_not_run_off_the_grid(self):
        grid = open_room(5, 5)
        assert grid.unknown_within(0.5, 0.5, 10.0) <= 25


class TestBresenham:
    """The line walk the collision check is built on."""

    def test_a_line_includes_both_endpoints(self):
        cells = list(bresenham(0, 0, 3, 0))
        assert cells[0] == (0, 0)
        assert cells[-1] == (3, 0)

    def test_a_point_is_a_single_cell(self):
        assert list(bresenham(2, 2, 2, 2)) == [(2, 2)]

    def test_a_diagonal_is_stepped_evenly(self):
        assert list(bresenham(0, 0, 3, 3)) == [(0, 0), (1, 1), (2, 2), (3, 3)]

    def test_it_walks_backwards_too(self):
        assert list(bresenham(3, 0, 0, 0)) == [(3, 0), (2, 0), (1, 0), (0, 0)]


class TestLineChecks:
    """Whether the straight line between two points is drivable."""

    def test_a_line_inside_free_space_is_clear(self):
        assert line_is_clear(room(), (0.5, 0.5), (1.5, 4.5))

    def test_a_line_into_the_wall_is_not(self):
        assert not line_is_clear(room(), (0.5, 2.5), (4.5, 2.5))

    def test_the_obstruction_is_where_the_wall_is(self):
        stop = first_obstruction(room(), (0.5, 2.5), (4.5, 2.5))
        assert stop is not None
        assert stop == pytest.approx((2.5, 2.5))

    def test_unknown_space_stops_a_line_by_default(self):
        # The RRT's edges have to stay in space the robot could drive; a branch
        # through the unobserved is a proposal to go where no route exists.
        grid = room()
        grid.data[:, 2] = UNKNOWN  # take the wall away, leave the unknown
        assert not line_is_clear(grid, (0.5, 2.5), (4.5, 2.5))

    def test_through_unknown_only_stops_at_obstacles(self):
        grid = room()
        grid.data[:, 2] = UNKNOWN
        assert line_is_clear(grid, (0.5, 2.5), (4.5, 2.5), through_unknown=True)

    def test_through_unknown_still_stops_at_a_wall(self):
        assert not line_is_clear(
            room(), (0.5, 2.5), (4.5, 2.5), through_unknown=True
        )


class TestFrontierCells:
    """The free/unknown boundary."""

    def test_a_free_cell_touching_unknown_is_a_frontier(self):
        grid = room()
        grid.data[:, 2] = UNKNOWN  # column 2 unobserved, so column 1 borders it
        assert is_frontier_cell(grid, 1, 2)

    def test_a_free_cell_in_the_middle_of_free_space_is_not(self):
        # Interior of a fully observed grid, so none of the four neighbours is
        # the off-grid unknown the edge cells border.
        grid = Grid(np.zeros((5, 5), dtype=np.int16), 1.0, 0.0, 0.0)
        assert not is_frontier_cell(grid, 2, 2)

    def test_an_unknown_cell_is_not_a_frontier(self):
        # The boundary belongs to the free side: it is somewhere the robot can
        # actually stand.
        grid = room()
        assert not is_frontier_cell(grid, 3, 2)

    def test_a_wall_between_free_and_unknown_is_not_a_frontier(self):
        # This is the case the whole exploration depends on getting right: the
        # room's right-hand columns are unobserved *because* there is a wall,
        # and driving at it forever is what a naive detector does.
        assert not is_frontier_cell(room(), 1, 2)
        assert not is_frontier_cell(room(), 2, 2)

    def test_a_diagonal_touch_is_not_enough(self):
        grid = Grid(np.zeros((3, 3), dtype=np.int16), 1.0, 0.0, 0.0)
        grid.data[0, 0] = UNKNOWN
        assert not is_frontier_cell(grid, 1, 1)
        grid.data[0, 1] = UNKNOWN
        assert is_frontier_cell(grid, 1, 1)

    def test_the_edge_of_the_grid_counts_as_unknown(self):
        grid = Grid(np.zeros((3, 3), dtype=np.int16), 1.0, 0.0, 0.0)
        assert is_frontier_cell(grid, 0, 1)


class TestSnapping:
    """Finding the real boundary near where a branch stopped."""

    def test_a_nearby_frontier_is_found(self):
        grid = room()
        grid.data[:, 2] = UNKNOWN
        found = nearest_frontier_cell(grid, 2.5, 2.5, 2.0)
        assert found == pytest.approx((1.5, 2.5))

    def test_nothing_is_found_where_there_is_no_boundary(self):
        # The candidate the map filled in between the tree growing and the
        # check: the normal fate of most candidates, and not an error.
        grid = Grid(np.zeros((10, 10), dtype=np.int16), 1.0, 0.0, 0.0)
        assert nearest_frontier_cell(grid, 5.0, 5.0, 2.0) is None

    def test_the_search_radius_is_respected(self):
        grid = room()
        grid.data[:, 2] = UNKNOWN
        assert nearest_frontier_cell(grid, 4.5, 2.5, 0.5) is None
        assert nearest_frontier_cell(grid, 4.5, 2.5, 4.0) is not None

    def test_the_closest_boundary_wins(self):
        grid = Grid(np.zeros((11, 11), dtype=np.int16), 1.0, 0.0, 0.0)
        grid.data[:, 0] = UNKNOWN
        grid.data[:, 10] = UNKNOWN
        found = nearest_frontier_cell(grid, 2.5, 5.5, 8.0)
        assert found is not None
        assert found[0] == pytest.approx(1.5)


class TestConstruction:
    """Building a grid from a message-shaped object."""

    def test_from_message_reshapes_row_major(self):
        class Info:
            width, height, resolution = 3, 2, 0.5

            class origin:
                class position:
                    x, y = 1.0, 2.0

        class Message:
            info = Info()
            data = [0, 1, 2, 3, 4, 5]

        grid = Grid.from_message(Message())
        assert grid.width == 3
        assert grid.height == 2
        # index = y * width + x, which is how OccupancyGrid orders its data.
        assert grid.value(2, 1) == 5
        assert grid.value(0, 1) == 3
        assert (grid.origin_x, grid.origin_y) == (1.0, 2.0)

    def test_a_one_dimensional_array_is_refused(self):
        with pytest.raises(ValueError):
            Grid(np.zeros(9, dtype=np.int16), 1.0, 0.0, 0.0)
