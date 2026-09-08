"""
What the robot does with the server's comparison verdict.

`MapCloudAgreement` arrives from the server saying where the 2D occupancy grid
and the Deep3R point cloud disagree. Feeding that into the exit criteria is one
use and `exit_criteria.py` has it; this module is the other, and the more
consequential one: turning the disagreement into something the **navigation
stack** acts on.

The whole reason a disagreement is informative is that the two sources see
different things. **The lidar is one horizontal plane** at a fixed height above
the floor. It cannot see a table top, a low step, a cable tray or an overhang --
and the 2D map it builds says "free" for all of them, because at the lidar's
height they *are* free. The cloud sees the volume. So a `cloud_only` region is
not noise in the reconstruction; it is usually the 2D map being wrong in the one
way it is systematically able to be wrong.

Which is why the decision is about **height**, and why the message carries one:

    ┌───────────────────────────────────────────────────────────────┐
    │  above robot_height          overhang -- drive under it       │  ignore
    ├───────────────────────────────────────────────────────────────┤
    │  robot_height .. lidar_height   the lidar should have seen    │  keepout
    │                                 this; the map is wrong        │  + flag
    ├─── lidar plane ───────────────────────────────────────────────┤
    │  clearance .. lidar_height   too low for the lidar, too high  │  KEEPOUT
    │                              to drive over -- the case this   │
    │                              whole module exists for          │
    ├───────────────────────────────────────────────────────────────┤
    │  below clearance             the robot rolls over it          │  ignore
    └───────────────────────────────────────────────────────────────┘

The other kinds are handled by *not* acting on them, and that is deliberate.
`map_only` means the lidar sees an obstacle the camera could not reconstruct --
glass, a thin dark panel, a specular surface. The lidar is right and the cloud
is missing something, so nothing is subtracted from the map: this module can
only ever make the robot more cautious, never less. `unobserved` is not a
disagreement at all, it is a place to go and look from, and that is the
explorer's business.

Nothing here imports ROS. `AgreementModel` takes plain numbers and returns plain
numbers, so the whole policy is testable at a desk and a run can be replayed off
a bag.
"""

import math

# The kinds the server may report, matching `MapCloudAgreement.uncertain_kinds`.
CLOUD_ONLY = "cloud_only"
MAP_ONLY = "map_only"
UNOBSERVED = "unobserved"
DISAGREEMENT = "disagreement"

KINDS = (CLOUD_ONLY, MAP_ONLY, UNOBSERVED, DISAGREEMENT)

# What the robot does about a region.
IGNORE = "ignore"
KEEPOUT = "keepout"
REVISIT = "revisit"

# Occupancy value a keepout cell is written with. `nav2_costmap_2d::KeepoutFilter`
# reads the mask through `base + data * multiplier`; with base 0 and multiplier 1
# that makes 100 lethal, which is the convention the stock nav2 masks use.
LETHAL = 100
FREE = 0


class Region:
    """One place the two sources disagree about, and what to do there."""

    def __init__(self, x, y, score, kind=DISAGREEMENT, height=None, radius=0.5, seen=0.0):
        self.x = float(x)
        self.y = float(y)
        self.score = float(score)
        self.kind = kind if kind in KINDS else DISAGREEMENT
        # `None` is a real value here and must survive: it means the server saw
        # structure and could not say how tall, which `blocks()` reads as the
        # cautious answer. Coercing it to 0.0 would silently turn every
        # unmeasured obstacle into a floor marking the robot drives over.
        self.height = None if height is None else float(height)
        self.radius = float(radius)
        # When the server last reported it, on the robot's clock.
        self.seen = float(seen)
        # Set once the robot has been to look; a serviced region stops being a
        # goal but does not stop being a keepout.
        self.serviced = False

    @property
    def point(self):
        """Return the region centre as an `(x, y)` tuple."""
        return (self.x, self.y)

    def distance_to(self, x, y):
        """Planar distance from the region centre to a point."""
        return math.hypot(self.x - x, self.y - y)

    def __repr__(self):
        """Show the kind, the place and the height, which is what a log line wants."""
        height = "?" if self.height is None else f"{self.height:.2f}"
        return (
            f"Region({self.kind}, x={self.x:.2f}, y={self.y:.2f}, "
            f"h={height}, score={self.score:.2f})"
        )


class AgreementModel:
    """
    The regions the server has reported, and what the robot should do with them.

    A verdict is a snapshot, not a diff: the server sends the regions it is
    currently unsure about, and a region it stops mentioning has usually been
    resolved rather than forgotten. So regions are *accumulated* with an age,
    and a keepout is dropped only when the server has been silent about it for
    `forget_after` seconds. Dropping one the moment it leaves a verdict would
    make the costmap flicker at whatever rate the comparison runs at, and a
    costmap that flickers is one nav2 replans through.
    """

    def __init__(
        self,
        # --- the robot's vertical envelope [m] -------------------------------
        # Below `clearance` the robot rolls over it; above `robot_height` it
        # drives under it; the lidar plane is where the 2D map's knowledge stops.
        clearance=0.03,
        lidar_height=0.20,
        robot_height=0.45,
        # --- what is worth acting on ------------------------------------------
        min_keepout_score=0.4,
        min_revisit_score=0.5,
        # --- housekeeping ------------------------------------------------------
        merge_radius=0.5,
        forget_after=120.0,
        serviced_radius=1.0,
    ):
        self.clearance = float(clearance)
        self.lidar_height = float(lidar_height)
        self.robot_height = float(robot_height)
        self.min_keepout_score = float(min_keepout_score)
        self.min_revisit_score = float(min_revisit_score)
        self.merge_radius = float(merge_radius)
        self.forget_after = float(forget_after)
        self.serviced_radius = float(serviced_radius)
        self.regions = []
        self.map_id = None

    # --- taking a verdict -----------------------------------------------------

    def update(self, now, points, scores, kinds=(), heights=(), radii=(), map_id=None):
        """
        Fold one comparison verdict into the model.

        The parallel arrays are the message's; any of the three optional ones
        may be short or missing, because a server that predates them still has
        to work. A region with no kind is a plain `disagreement` and a region
        with no height is treated as unknown height, which -- see
        `_action_for` -- is the cautious reading rather than the ignorable one.

        `map_id` is the **robot's** SLAM map. Pass `MapCloudAgreement.map_id`,
        not `cloud_map_id`; see the reset below for why they are not
        interchangeable.
        """
        if map_id is not None and map_id != self.map_id:
            # `map_id` is the ROBOT's SLAM map, not CUT3R's reconstruction
            # session -- the two used to share a name, and this reset was keyed
            # on the wrong one. Regions are map-frame facts: "a table top is at
            # (3.2, 1.4)" survives the reconstruction restarting, because the
            # server re-derives the alignment and keeps publishing in the map
            # frame. What it does not survive is the map frame itself changing.
            # slam_toolbox re-rasterising or restarting is what puts every
            # accumulated keepout at a coordinate that now means somewhere else.
            self.regions = []
            self.map_id = map_id

        for index, (x, y) in enumerate(points):
            region = Region(
                x,
                y,
                _at(scores, index, 0.0),
                kind=_at(kinds, index, DISAGREEMENT),
                height=_at(heights, index, None),
                radius=_at(radii, index, self.merge_radius),
                seen=now,
            )
            self._absorb(region)

        self._forget(now)
        return self.regions

    def _absorb(self, region):
        """Merge a region into an existing one of the same kind nearby, or add it."""
        for existing in self.regions:
            # Only ever within a kind. `cloud_only` and `map_only` are not two
            # flavours of the same thing -- one is a table top the lidar cannot
            # see and the other is glass the camera could not reconstruct, and
            # the robot's responses are opposite: make it lethal, or leave it
            # alone. A table beside a glass panel is one of each within a
            # merge radius of the other, and merging them lets the newest
            # report decide which, so the same pair of surfaces produces a
            # keepout or a collision depending on the order they arrived in.
            # The server clusters within a kind for exactly this reason
            # (`robocam/regions.py`); this end used to undo it.
            if existing.kind != region.kind:
                continue
            if existing.distance_to(region.x, region.y) > self.merge_radius:
                continue
            # The newest report wins on everything except `serviced`: the robot
            # having been to look is a fact about the robot, not about the
            # verdict, and a re-report must not send it back round again.
            serviced = existing.serviced
            existing.__dict__.update(region.__dict__)
            existing.serviced = serviced
            return
        self.regions.append(region)

    def _forget(self, now):
        self.regions = [r for r in self.regions if now - r.seen <= self.forget_after]

    def mark_serviced(self, x, y):
        """Note that the robot has been to look at whatever is near a point."""
        for region in self.regions:
            if region.distance_to(x, y) <= self.serviced_radius:
                region.serviced = True

    # --- what to do about a region -------------------------------------------

    def action_for(self, region):
        """Return IGNORE, KEEPOUT or REVISIT for one region."""
        if region.kind == UNOBSERVED:
            return REVISIT if region.score >= self.min_revisit_score else IGNORE

        if region.kind == MAP_ONLY:
            # The lidar sees something the camera could not reconstruct: glass,
            # a thin panel, a specular surface. The lidar is the one to believe,
            # and the map already has it. Going to look is still worth it -- more
            # frames from a better angle is the only feedback the reconstruction
            # can use -- but nothing is subtracted from the costmap. This module
            # can make the robot more cautious, never less.
            return REVISIT if region.score >= self.min_revisit_score else IGNORE

        if region.score < self.min_keepout_score:
            return REVISIT if region.score >= self.min_revisit_score else IGNORE

        return KEEPOUT if self.blocks(region.height) else IGNORE

    def blocks(self, height):
        """
        Say whether structure of this height is something the robot would hit.

        An unknown height (`None`) counts as blocking. The server said the cloud
        has structure here and could not say how tall; assuming it is an
        overhang would be the one reading that lets the robot drive into it.
        """
        if height is None:
            return True
        return self.clearance <= float(height) <= self.robot_height

    def invisible_to_lidar(self, height):
        """
        Say whether structure of this height sits outside the lidar's plane.

        The case the whole module is about: too low for the lidar to have seen,
        too high to roll over. A `cloud_only` region here is the 2D map being
        wrong in the one way it is systematically able to be wrong, rather than
        the reconstruction being noisy.
        """
        if height is None:
            return False
        return self.clearance <= float(height) < self.lidar_height

    # --- the products ---------------------------------------------------------

    def keepouts(self):
        """Regions that should be marked lethal in the costmap."""
        return [r for r in self.regions if self.action_for(r) == KEEPOUT]

    def revisits(self):
        """
        Regions worth going to look at again, best first, unserviced only.

        A keepout is not a revisit target: the robot has been told there is
        something there it would hit, and driving up to it to have a better look
        is the one response that cannot be right.
        """
        pending = [
            r
            for r in self.regions
            if not r.serviced and self.action_for(r) == REVISIT
        ]
        pending.sort(key=lambda r: r.score, reverse=True)
        return pending

    def summary(self):
        """Return counts by action and by kind, for a log line."""
        actions = {IGNORE: 0, KEEPOUT: 0, REVISIT: 0}
        kinds = {kind: 0 for kind in KINDS}
        for region in self.regions:
            actions[self.action_for(region)] += 1
            kinds[region.kind] += 1
        return actions, kinds


def stamp_mask(width, height, resolution, origin_x, origin_y, regions, value=LETHAL):
    """
    Rasterise regions onto a flat occupancy list of the given grid shape.

    Returned as a plain list in `nav_msgs/OccupancyGrid` order (`y * width + x`)
    so the node only has to attach a header. Regions off the edge of the grid
    are clipped rather than refused: the map grows underneath a verdict all the
    time, and a keepout the map has not reached yet is not an error.
    """
    data = [FREE] * (width * height)
    for region in regions:
        cells = int(math.ceil(region.radius / resolution))
        column = int(math.floor((region.x - origin_x) / resolution))
        row = int(math.floor((region.y - origin_y) / resolution))
        for dr in range(-cells, cells + 1):
            for dc in range(-cells, cells + 1):
                if dc * dc + dr * dr > cells * cells:
                    continue
                c, r = column + dc, row + dr
                if 0 <= c < width and 0 <= r < height:
                    data[r * width + c] = value
    return data


def _at(sequence, index, default):
    """Read a parallel array safely -- a short or missing one gives the default."""
    try:
        value = sequence[index]
    except (IndexError, TypeError):
        return default
    return default if value is None else value
