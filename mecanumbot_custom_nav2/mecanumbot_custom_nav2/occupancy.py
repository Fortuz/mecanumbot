"""
Reading a `nav_msgs/OccupancyGrid` as an array, without any ROS in the way.

Everything the frontier detector needs to know about the map is here: where a
world point falls in the grid, whether a cell is free, unknown or occupied,
whether the straight line between two points stays in known-free space, and
whether a cell sits on the boundary between free and unknown -- which is what a
frontier *is*.

The grid is held as a numpy array of the raw occupancy values slam_toolbox
publishes: `-1` unknown, `0` certainly free, `100` certainly occupied, and the
values in between a probability in percent. The thresholds that turn those into
three classes are the caller's, because "free enough to drive through" is a
navigation decision and not a property of the map.

None of this imports rclpy, so the whole detector is testable at a desk.
"""

import math

import numpy as np

UNKNOWN = -1

# What a cell is, once the thresholds have been applied.
FREE = 0
OCCUPIED = 1
UNCERTAIN = 2


class Grid:
    """
    An occupancy grid, its pose in the world, and the classification thresholds.

    `data` is row-major with row 0 at the origin, which is how
    `nav_msgs/OccupancyGrid` orders it: index `y * width + x`.
    """

    def __init__(
        self,
        data,
        resolution,
        origin_x,
        origin_y,
        free_threshold=25,
        occupied_threshold=65,
    ):
        self.data = np.asarray(data, dtype=np.int16)
        if self.data.ndim != 2:
            raise ValueError("grid data must be 2-D (height, width)")
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.free_threshold = int(free_threshold)
        self.occupied_threshold = int(occupied_threshold)

    # --- construction ---------------------------------------------------------

    @classmethod
    def from_message(cls, msg, free_threshold=25, occupied_threshold=65):
        """Build a grid from a `nav_msgs/OccupancyGrid`."""
        info = msg.info
        data = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        return cls(
            data,
            info.resolution,
            info.origin.position.x,
            info.origin.position.y,
            free_threshold=free_threshold,
            occupied_threshold=occupied_threshold,
        )

    # --- shape ----------------------------------------------------------------

    @property
    def height(self):
        """Rows in the grid."""
        return int(self.data.shape[0])

    @property
    def width(self):
        """Columns in the grid."""
        return int(self.data.shape[1])

    @property
    def bounds(self):
        """World extent as `(min_x, min_y, max_x, max_y)`."""
        return (
            self.origin_x,
            self.origin_y,
            self.origin_x + self.width * self.resolution,
            self.origin_y + self.height * self.resolution,
        )

    # --- world <-> cell -------------------------------------------------------

    def to_cell(self, x, y):
        """Cell `(column, row)` a world point falls in, whether or not it is inside."""
        return (
            int(math.floor((x - self.origin_x) / self.resolution)),
            int(math.floor((y - self.origin_y) / self.resolution)),
        )

    def to_world(self, column, row):
        """World point at the centre of a cell."""
        return (
            self.origin_x + (column + 0.5) * self.resolution,
            self.origin_y + (row + 0.5) * self.resolution,
        )

    def contains(self, column, row):
        """Say whether a cell index is inside the grid."""
        return 0 <= column < self.width and 0 <= row < self.height

    # --- classification -------------------------------------------------------

    def value(self, column, row):
        """Raw occupancy value of a cell; outside the grid counts as unknown."""
        if not self.contains(column, row):
            return UNKNOWN
        return int(self.data[row, column])

    def classify(self, column, row):
        """Return FREE, OCCUPIED or UNCERTAIN for one cell."""
        value = self.value(column, row)
        if value == UNKNOWN:
            return UNCERTAIN
        if value <= self.free_threshold:
            return FREE
        if value >= self.occupied_threshold:
            return OCCUPIED
        return UNCERTAIN

    def is_free(self, column, row):
        """Say whether a cell is confidently free."""
        return self.classify(column, row) == FREE

    def is_unknown(self, column, row):
        """Say whether a cell has never been observed."""
        return self.value(column, row) == UNKNOWN

    def is_occupied(self, column, row):
        """Say whether a cell is confidently occupied."""
        return self.classify(column, row) == OCCUPIED

    def free_at(self, x, y):
        """Say whether a world point falls in a confidently free cell."""
        return self.is_free(*self.to_cell(x, y))

    def unknown_at(self, x, y):
        """Say whether a world point falls in a cell that has never been observed."""
        return self.is_unknown(*self.to_cell(x, y))

    # --- counts, for coverage and information gain ----------------------------

    def counts(self):
        """Return `(free, occupied, unknown)` cell counts over the whole grid."""
        unknown = int(np.count_nonzero(self.data == UNKNOWN))
        known = self.data[self.data != UNKNOWN]
        free = int(np.count_nonzero(known <= self.free_threshold))
        occupied = int(np.count_nonzero(known >= self.occupied_threshold))
        return free, occupied, unknown

    def known_cells(self):
        """Cells that have been observed at all."""
        return int(np.count_nonzero(self.data != UNKNOWN))

    def unknown_within(self, x, y, radius):
        """
        Count unobserved cells within `radius` metres of a world point.

        This is the information gain of standing there: how much of the map a
        frontier would fill in if the robot went to it and looked.
        """
        cells = int(math.ceil(radius / self.resolution))
        column, row = self.to_cell(x, y)
        c0, c1 = max(0, column - cells), min(self.width, column + cells + 1)
        r0, r1 = max(0, row - cells), min(self.height, row + cells + 1)
        if c0 >= c1 or r0 >= r1:
            return 0
        window = self.data[r0:r1, c0:c1]

        # A square window over-counts the corners by a quarter, and at the scales
        # this runs at (a few hundred cells) masking to a disc is cheap enough to
        # be worth doing properly -- an information gain that depends on which
        # way a wall happens to lie is not a gain.
        columns = np.arange(c0, c1)[None, :] - column
        rows = np.arange(r0, r1)[:, None] - row
        inside = (columns**2 + rows**2) <= cells**2
        return int(np.count_nonzero((window == UNKNOWN) & inside))


def bresenham(x0, y0, x1, y1):
    """Yield every cell on the line between two cells, endpoints included."""
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    x, y = x0, y0
    while True:
        yield x, y
        if x == x1 and y == y1:
            return
        doubled = 2 * error
        if doubled >= dy:
            error += dy
            x += sx
        if doubled <= dx:
            error += dx
            y += sy


def first_obstruction(grid, start, end, through_unknown=False):
    """
    Walk the line between two world points and return where it stops, or None.

    Returns the world point of the first cell that is not drivable, so a caller
    can both test the line and use the place it broke. With `through_unknown`
    the walk only stops at confidently occupied cells, which is how the *global*
    RRT is allowed to grow into unmapped space; the local one is not.
    """
    c0, r0 = grid.to_cell(*start)
    c1, r1 = grid.to_cell(*end)
    for column, row in bresenham(c0, r0, c1, r1):
        cell = grid.classify(column, row)
        if cell == OCCUPIED:
            return grid.to_world(column, row)
        if not through_unknown and cell != FREE:
            return grid.to_world(column, row)
    return None


def line_is_clear(grid, start, end, through_unknown=False):
    """Say whether the straight line between two world points is drivable."""
    return first_obstruction(grid, start, end, through_unknown) is None


def is_frontier_cell(grid, column, row):
    """
    Say whether a cell is free and touches unobserved space.

    That is the definition a frontier detector works to: the boundary between
    what the robot knows is drivable and what it has not looked at yet. Only the
    four edge neighbours count -- a diagonal touch is one corner, and accepting
    those makes a ragged frontier out of the staircase edges every rasterised
    map has.
    """
    if not grid.is_free(column, row):
        return False
    return any(
        grid.is_unknown(column + dc, row + dr)
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1))
    )


def nearest_frontier_cell(grid, x, y, search_radius):
    """
    Find a frontier cell within `search_radius` metres of a world point.

    The RRT lands *near* a frontier rather than exactly on one -- it grows in
    free space and stops when it runs into the unknown -- so a candidate is
    snapped to the real boundary before it is clustered. Returns a world point,
    or None where the candidate was a false alarm (the map filled in between the
    tree growing and the candidate being checked, which happens constantly).
    """
    cells = int(math.ceil(search_radius / grid.resolution))
    column, row = grid.to_cell(x, y)
    best = None
    best_distance = None
    for dr in range(-cells, cells + 1):
        for dc in range(-cells, cells + 1):
            distance = dc * dc + dr * dr
            if distance > cells * cells:
                continue
            if best_distance is not None and distance >= best_distance:
                continue
            if is_frontier_cell(grid, column + dc, row + dr):
                best = (column + dc, row + dr)
                best_distance = distance
    return None if best is None else grid.to_world(*best)
