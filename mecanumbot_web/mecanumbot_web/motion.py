"""
Turn a body-frame velocity into a movement state, without rclpy.

The rest of the diagnostics page answers "is every topic ticking?".
That is not the same question as "what is the robot doing", and an
operator standing next to a robot that will not move needs the second
one -- ``/cmd_vel`` can be perfectly healthy at 50 Hz while carrying
zeros, and odometry can be healthy while the wheels are stalled.

Deciding it is arithmetic against thresholds, so it lives here rather
than in the node, for the same reason ``mecanumbot_joy`` keeps
``actions.py`` free of rclpy: it can then be tested without a ROS graph.

Mecanum drive is why this is not a one-liner.  ``vy`` is a real degree of
freedom rather than structurally zero, so "forward" and "strafing left"
hold at the same time during a diagonal, and a single-word answer would
be wrong.  The state is assembled from up to three simultaneous
components instead.

Frames follow REP-103, as the whole workspace does: +x forward, +y left,
+z yaw counter-clockwise.
"""

import math

#: Linear speeds below this read as stopped (m/s).  The drivetrain's own
#: deadband is smaller; this is set by what odometry noise looks like on
#: a stationary robot, so the page does not report a parked robot as
#: creeping.
DEFAULT_LINEAR_DEADBAND = 0.02

#: Yaw rates below this read as not rotating (rad/s), same reasoning.
DEFAULT_ANGULAR_DEADBAND = 0.05

#: State reported when nothing is driving.
STOPPED = "stopped"

#: State reported when the command topic has gone silent.  Distinct from
#: STOPPED: nobody is asking for zero, nobody is asking for anything.
NO_COMMAND = "no command"

#: State reported when there is no data to judge from at all.
UNKNOWN = "unknown"


def _component(value, deadband, positive, negative):
    """Return the name for one signed axis, or '' inside the deadband."""
    if value > deadband:
        return positive
    if value < -deadband:
        return negative
    return ""


def describe_motion(
    vx,
    vy,
    wz,
    linear_deadband=DEFAULT_LINEAR_DEADBAND,
    angular_deadband=DEFAULT_ANGULAR_DEADBAND,
):
    """
    Describe one body-frame velocity triple in words and numbers.

    Return the raw components, the named parts that are active, a
    one-line ``state`` built from them, and the flags the page renders
    from.  ``heading_deg`` is the direction of travel -- 0 straight
    ahead, positive to the left -- or None when the robot is only
    turning, because a pure spin has no direction of travel.
    """
    vx, vy, wz = float(vx), float(vy), float(wz)

    parts = [
        _component(vx, linear_deadband, "forward", "backward"),
        _component(vy, linear_deadband, "strafing left", "strafing right"),
        _component(wz, angular_deadband, "rotating left", "rotating right"),
    ]
    components = [part for part in parts if part]

    speed = math.hypot(vx, vy)
    translating = speed > linear_deadband
    rotating = abs(wz) > angular_deadband

    return {
        "vx": round(vx, 3),
        "vy": round(vy, 3),
        "wz": round(wz, 3),
        "speed": round(speed, 3),
        "heading_deg": round(math.degrees(math.atan2(vy, vx)), 1)
        if translating else None,
        "components": components,
        "state": " + ".join(components) if components else STOPPED,
        "moving": bool(components),
        "translating": translating,
        "rotating": rotating,
    }


def idle(state=STOPPED):
    """Return the shape :func:`describe_motion` returns, with no motion in it."""
    described = describe_motion(0.0, 0.0, 0.0)
    described["state"] = state
    return described
