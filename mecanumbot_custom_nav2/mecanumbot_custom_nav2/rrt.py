"""
Sampling-based frontier detection: an RRT that reports where it runs out of map.

The idea is Umari & Mukhopadhyay's (2017): rather than scanning every cell of
the occupancy grid for the free/unknown boundary, grow a rapidly-exploring
random tree through the *known-free* space and record every point at which a
branch runs into the unknown. Because an RRT's growth is biased towards
unexplored regions by construction, its leaves land on frontiers on their own,
and the cost of a detection step does not grow with the size of the map -- which
is the property that matters on the Orin, where a full-grid scan of a room-sized
map at 2 Hz is real CPU the perception stack wants back.

Two trees, as in the paper, because they answer different questions:

* the **global** tree spans the whole map, is never reset, and finds frontiers
  anywhere -- including the far side of a room the robot has already left;
* the **local** tree is rooted at the robot and is reset every time it finds
  something, so it keeps proposing frontiers *near the robot*, which are the
  ones worth driving to next.

Neither tree is a plan. A detected point is a place worth going; getting there
is nav2's problem, and the straight-line steps here are only how the tree grows.

This module holds no ROS types and no map of its own -- it is given a
`occupancy.Grid` on every step, because the map underneath a detector changes
continuously and a tree grown against a stale one proposes frontiers that have
already been filled in.
"""

import math
import random

from mecanumbot_custom_nav2.occupancy import (
    first_obstruction,
    nearest_frontier_cell,
)


class RRTDetector:
    """
    One rapidly-exploring random tree over the free space of an occupancy grid.

    `grow(grid)` runs a single iteration and returns a frontier point, or None.
    Callers run it many times per detection cycle: an iteration is cheap and
    most of them find nothing, which is the shape of the algorithm rather than a
    sign anything is wrong.

    `reset_on_detection` is what makes a tree local: rooted at the robot and
    started over whenever it succeeds, it never wanders far enough to stop being
    about the robot's own surroundings.
    """

    def __init__(
        self,
        root,
        step_size=1.0,
        snap_radius=0.5,
        reset_on_detection=False,
        through_unknown=True,
        rng=None,
    ):
        self.step_size = float(step_size)
        self.snap_radius = float(snap_radius)
        self.reset_on_detection = bool(reset_on_detection)
        self.through_unknown = bool(through_unknown)
        self.rng = rng or random.Random()
        self.reset(root)

    # --- the tree -------------------------------------------------------------

    def reset(self, root):
        """Start over from a single node at `root`."""
        self.root = (float(root[0]), float(root[1]))
        self.vertices = [self.root]
        # Parent index per vertex; the root's parent is itself. Kept so the tree
        # can be drawn in RViz, which is the only way to see why a detector is
        # not finding anything.
        self.parents = [0]

    @property
    def size(self):
        """Vertices currently in the tree."""
        return len(self.vertices)

    def edges(self):
        """Yield `(parent_point, child_point)` for every edge, for visualization."""
        for index in range(1, len(self.vertices)):
            yield self.vertices[self.parents[index]], self.vertices[index]

    # --- one iteration --------------------------------------------------------

    def grow(self, grid, bounds=None):
        """
        Run one RRT iteration against `grid`; return a frontier point or None.

        The three steps are the standard ones -- sample, steer, collision-check
        -- with the detection folded into the collision check: a branch that
        stops because it reached *unknown* space rather than an obstacle has
        found a frontier, and where it stopped is the candidate.
        """
        sample = self._sample(grid, bounds)
        nearest_index = self._nearest(sample)
        nearest = self.vertices[nearest_index]
        target = self._steer(nearest, sample)

        # An edge is legal only through confidently free space. Growing towards
        # the unknown is the point of the exercise, but the *tree* must stay in
        # space the robot could actually drive, or its leaves are proposals to
        # go somewhere no route exists to.
        obstruction = first_obstruction(grid, nearest, target, through_unknown=False)
        if obstruction is None:
            self._add(target, nearest_index)
            return None

        # The branch stopped. If it stopped at a wall there is nothing here; if
        # it stopped at the edge of what has been observed, this is a frontier
        # -- but only if a real free/unknown boundary is actually nearby, which
        # is what rules out the candidates the map filled in while we grew.
        if grid.is_occupied(*grid.to_cell(*obstruction)):
            return None
        candidate = nearest_frontier_cell(
            grid, obstruction[0], obstruction[1], self.snap_radius
        )
        if candidate is None:
            return None

        if self.reset_on_detection:
            self.reset(self.root)
        return candidate

    def grow_many(self, grid, iterations, bounds=None):
        """Run `iterations` growth steps and return every frontier point found."""
        found = []
        for _ in range(int(iterations)):
            point = self.grow(grid, bounds)
            if point is not None:
                found.append(point)
        return found

    # --- internals ------------------------------------------------------------

    def _sample(self, grid, bounds):
        min_x, min_y, max_x, max_y = bounds if bounds is not None else grid.bounds
        return (self.rng.uniform(min_x, max_x), self.rng.uniform(min_y, max_y))

    def _nearest(self, point):
        # Linear scan. The trees here hold hundreds of vertices, not millions,
        # and a k-d tree that has to be rebuilt every time the local tree resets
        # costs more than it saves at this size.
        best_index = 0
        best = float("inf")
        for index, vertex in enumerate(self.vertices):
            distance = (vertex[0] - point[0]) ** 2 + (vertex[1] - point[1]) ** 2
            if distance < best:
                best = distance
                best_index = index
        return best_index

    def _steer(self, origin, target):
        dx = target[0] - origin[0]
        dy = target[1] - origin[1]
        distance = math.hypot(dx, dy)
        if distance <= self.step_size or distance == 0.0:
            return (target[0], target[1])
        scale = self.step_size / distance
        return (origin[0] + dx * scale, origin[1] + dy * scale)

    def _add(self, point, parent_index):
        self.vertices.append(point)
        self.parents.append(parent_index)


class FrontierSearch:
    """
    The pair of trees, run together, as the node's detection step.

    `step(grid, robot_xy)` grows both and returns the frontier points found this
    cycle. The local tree is re-rooted at the robot on every call, because a
    local tree still rooted where the robot stood a minute ago is a global tree
    with a smaller budget.
    """

    def __init__(
        self,
        step_size=1.0,
        snap_radius=0.5,
        global_iterations=60,
        local_iterations=30,
        local_radius=5.0,
        seed=None,
    ):
        self.step_size = float(step_size)
        self.snap_radius = float(snap_radius)
        self.global_iterations = int(global_iterations)
        self.local_iterations = int(local_iterations)
        self.local_radius = float(local_radius)
        self.rng = random.Random(seed)
        self.global_tree = None
        self.local_tree = None

    def step(self, grid, robot_xy):
        """Grow both trees one cycle and return the frontier points detected."""
        if self.global_tree is None:
            self.global_tree = RRTDetector(
                robot_xy,
                step_size=self.step_size,
                snap_radius=self.snap_radius,
                reset_on_detection=False,
                rng=self.rng,
            )
        self.local_tree = self.local_tree or RRTDetector(
            robot_xy,
            step_size=self.step_size,
            snap_radius=self.snap_radius,
            reset_on_detection=True,
            rng=self.rng,
        )
        self.local_tree.reset(robot_xy)

        found = self.global_tree.grow_many(grid, self.global_iterations)
        found += self.local_tree.grow_many(
            grid, self.local_iterations, bounds=self._local_bounds(grid, robot_xy)
        )
        return found

    def restart(self, robot_xy):
        """
        Throw both trees away and start again from the robot.

        Worth doing after a loop closure: slam_toolbox re-rasterises the whole
        occupancy grid from the pose graph, so the free space a tree was grown
        through may have moved out from under it.
        """
        self.global_tree = None
        self.local_tree = None
        if robot_xy is not None:
            self.global_tree = RRTDetector(
                robot_xy,
                step_size=self.step_size,
                snap_radius=self.snap_radius,
                reset_on_detection=False,
                rng=self.rng,
            )

    def _local_bounds(self, grid, robot_xy):
        min_x, min_y, max_x, max_y = grid.bounds
        x, y = robot_xy
        return (
            max(min_x, x - self.local_radius),
            max(min_y, y - self.local_radius),
            min(max_x, x + self.local_radius),
            min(max_y, y + self.local_radius),
        )
