"""
Turning a scatter of detected points into the one place to drive to next.

The RRT hands over raw candidates, many of them duplicates of each other and
many of them stale by the time they are looked at. Three things happen here, all
of them pure functions of a grid and a list of points:

1. **clustering** -- candidates within one radius of each other are the same
   frontier seen several times, and are replaced by their centroid;
2. **revalidation** -- a candidate is dropped if the cell it names is no longer
   on a free/unknown boundary. The map fills in continuously while the detector
   runs, so this is the normal fate of most candidates rather than an error;
3. **scoring** -- what a frontier is worth, which is the only genuinely
   discretionary step and therefore the one written down most explicitly.

The score is deliberately simple and deliberately not a utility in any
principled sense. It trades information gain against the cost of going to get
it, and it holds on to the goal already being pursued:

    score = gain_weight * gain - cost_weight * distance + hysteresis

`gain` is the count of unobserved cells the robot would fill in from there,
normalized by the largest gain in the batch, so the two terms are commensurable
without anyone having to pick units. `hysteresis` is added only to the frontier
closest to the current goal, and exists because an explorer that re-decides from
scratch at 1 Hz oscillates between two equally good frontiers and drives to
neither -- which looks exactly like a broken planner and is not one.
"""

import math

from mecanumbot_custom_nav2.occupancy import is_frontier_cell


class Frontier:
    """One clustered frontier: where it is, how big it was, what it is worth."""

    def __init__(self, x, y, support=1, gain=0, distance=0.0, score=0.0):
        self.x = float(x)
        self.y = float(y)
        self.support = int(support)
        self.gain = int(gain)
        self.distance = float(distance)
        self.score = float(score)

    @property
    def point(self):
        """Return the frontier as an `(x, y)` tuple."""
        return (self.x, self.y)

    def __repr__(self):
        """Show the position and the score, which is what a log line wants."""
        return (
            f"Frontier(x={self.x:.2f}, y={self.y:.2f}, "
            f"support={self.support}, gain={self.gain}, score={self.score:.2f})"
        )


def cluster(points, radius):
    """
    Merge points within `radius` of each other into their centroids.

    Single-link, greedy and O(n * clusters): the batches are tens of points, and
    a proper mean-shift here would be precision the input does not have.
    """
    clusters = []
    for x, y in points:
        for entry in clusters:
            if math.hypot(entry["x"] - x, entry["y"] - y) <= radius:
                # Running mean, so a frontier detected many times is placed at
                # the middle of its detections rather than at the first one.
                entry["n"] += 1
                entry["x"] += (x - entry["x"]) / entry["n"]
                entry["y"] += (y - entry["y"]) / entry["n"]
                break
        else:
            clusters.append({"x": float(x), "y": float(y), "n": 1})
    return [Frontier(c["x"], c["y"], support=c["n"]) for c in clusters]


def still_a_frontier(grid, frontier, radius):
    """
    Say whether a clustered point still sits on a free/unknown boundary.

    The centroid of a cluster need not land on a frontier cell itself -- it is
    an average of nearby ones -- so the test is whether *any* cell within
    `radius` still qualifies.
    """
    cells = int(math.ceil(radius / grid.resolution))
    column, row = grid.to_cell(frontier.x, frontier.y)
    for dr in range(-cells, cells + 1):
        for dc in range(-cells, cells + 1):
            if dc * dc + dr * dr > cells * cells:
                continue
            if is_frontier_cell(grid, column + dc, row + dr):
                return True
    return False


def revalidate(grid, frontiers, radius):
    """Drop the frontiers the map has filled in since they were detected."""
    return [f for f in frontiers if still_a_frontier(grid, f, radius)]


def score(
    grid,
    frontiers,
    robot_xy,
    gain_radius=1.5,
    gain_weight=1.0,
    cost_weight=0.4,
    min_gain=8,
    current_goal=None,
    hysteresis=0.25,
    hysteresis_radius=1.0,
):
    """
    Fill in each frontier's gain, distance and score, and drop the trivial ones.

    `min_gain` is the count of unobserved cells below which a frontier is not
    worth a drive at all. It is what stops the robot chasing the single-cell
    boundaries that every rasterised map has along its walls -- the ones that
    can never be resolved because the thing behind them is a wall.

    The distance is straight-line, not the length of a plan. A frontier behind a
    wall is therefore under-costed, and the robot occasionally sets off towards
    one and takes the long way round. Asking nav2 for a path to every candidate
    would cost more than the whole detection step and would still be wrong by
    the time the robot got there; the local tree already biases the choice
    towards frontiers that are genuinely nearby.
    """
    kept = []
    for frontier in frontiers:
        frontier.gain = grid.unknown_within(frontier.x, frontier.y, gain_radius)
        if frontier.gain < min_gain:
            continue
        frontier.distance = math.hypot(
            frontier.x - robot_xy[0], frontier.y - robot_xy[1]
        )
        kept.append(frontier)

    if not kept:
        return []

    largest = max(f.gain for f in kept) or 1
    for frontier in kept:
        frontier.score = (
            gain_weight * (frontier.gain / largest) - cost_weight * frontier.distance
        )
        if current_goal is not None and (
            math.hypot(frontier.x - current_goal[0], frontier.y - current_goal[1])
            <= hysteresis_radius
        ):
            frontier.score += hysteresis

    kept.sort(key=lambda f: f.score, reverse=True)
    return kept


def best(
    grid,
    points,
    robot_xy,
    cluster_radius=0.6,
    revalidate_radius=0.4,
    current_goal=None,
    **scoring,
):
    """
    Run the whole pipeline and return `(best_frontier, all_scored_frontiers)`.

    Both are returned because the node publishes all of them as markers -- being
    able to see the frontiers the robot rejected is most of the debugging of an
    explorer that will not move.
    """
    frontiers = revalidate(grid, cluster(points, cluster_radius), revalidate_radius)
    scored = score(grid, frontiers, robot_xy, current_goal=current_goal, **scoring)
    return (scored[0] if scored else None), scored
