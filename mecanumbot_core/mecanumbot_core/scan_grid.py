"""
Put every lidar revolution on the same angular grid.

The LD08 driver publishes each revolution with its own geometry: `angle_min`,
`angle_max` and `angle_increment` all move by a few hundredths from one scan to
the next, and the number of readings runs from 201 to 206. Most consumers do
not care. slam_toolbox does, fatally: Karto builds its laser model from the
FIRST scan it receives and drops, in `LaserRangeFinder::Validate`, every later
scan whose reading count differs from that model's. Which scan arrives first
changes from launch to launch, so a T1 pass keeps somewhere between half its
scans and almost none -- measured on the robot on 2026-09-15, about one launch
in seven kept 2%, and slam_toolbox's map was still 0 x 0 after five minutes.
Nav2's global costmap, which takes its size from that map, then falls back to
a 5 x 5 m square at the origin and reports the robot out of bounds.

So slam_toolbox reads a copy of the scan on a fixed grid instead: `bins`
equal sectors over a full turn, starting at angle 0. Each sector holds the
NEAREST valid return that fell inside it -- the conservative choice for a map,
since a sector that saw a wall and a gap behind it should say "wall" -- and a
sector nothing valid fell into is `inf`, the LaserScan convention for "no
return". With `bins` at 200 a sector is 1.8 degrees, wider than the driver's
~1.76 degree spacing, so a full revolution leaves no sector empty.

Pure Python and no ROS, so it can be tested on its own.
"""

import math

TWO_PI = 2.0 * math.pi


def regrid(angle_min, angle_increment, ranges, range_min, range_max, bins,
           intensities=None):
    """
    Return ``(ranges, intensities)`` resampled onto ``bins`` sectors of a turn.

    Sector ``i`` covers ``[i, i + 1) * 2pi / bins``, measured from angle 0 and
    wrapped, so a reading at -0.1 rad lands near the end of the turn. A reading
    is valid when it is finite and inside ``[range_min, range_max]``; 0.0, which
    the LD08 sends for "no return", is not. ``intensities`` follow the reading
    that won their sector, and are all 0.0 when none are given.
    """
    bins = int(bins)
    if bins <= 0:
        raise ValueError(f"bins must be positive, got {bins}")
    width = TWO_PI / bins
    out_ranges = [math.inf] * bins
    out_intensities = [0.0] * bins
    have_intensities = intensities is not None and len(intensities) == len(ranges)

    for index, value in enumerate(ranges):
        value = float(value)
        if not math.isfinite(value) or value < range_min or value > range_max:
            continue
        angle = (angle_min + index * angle_increment) % TWO_PI
        sector = int(angle / width)
        if sector >= bins:  # angle within float error of 2pi
            sector = bins - 1
        if value < out_ranges[sector]:
            out_ranges[sector] = value
            out_intensities[sector] = (
                float(intensities[index]) if have_intensities else 0.0)
    return out_ranges, out_intensities


def grid_geometry(bins):
    """Return ``(angle_min, angle_max, angle_increment)`` for the fixed grid."""
    width = TWO_PI / int(bins)
    return 0.0, (int(bins) - 1) * width, width
