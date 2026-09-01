"""
Scripted motion and articulation for scenario actors.

Two independent things happen here. `SimActorRuntime.update` dead-reckons the
actor along its patrol waypoints, which is what `/sim/actors` reports. On top of
that, `segment_poses` runs a small forward-kinematic chain over the figure's two
spine segments and four limbs, so an actor can be doing aerobics while standing
still, or while walking.

Angle convention, uniform across every segment: **positive roll swings the
segment's far end toward the actor's left (+y), positive pitch swings it forward
(+x)**. The two torso segments grow along +z from their pivot and the four limbs
hang along -z from theirs, so the sign that achieves this differs per segment;
`segment_quaternion` handles that with a growth sign and nothing else has to
think about it.
"""

import math
from dataclasses import dataclass

from mecanumbot_sim.sim_scenarios import (
    ARM_LENGTH,
    HIP_Z,
    LEG_LATERAL_OFFSET,
    SHOULDER_LATERAL,
    SHOULDER_LOCAL_Z,
    SEGMENT_NAMES,
    SPINE_BASE_Z,
    TORSO_LOWER_LENGTH,
    SimActorConfig,
    SimGaitConfig,
)

TWO_PI = 2.0 * math.pi

#: Arms go overhead rather than out to the side in the bending moves. Clamped
#: short of straight up so the two arms stay visibly apart.
OVERHEAD_SCALE = 1.7
OVERHEAD_LIMIT = 0.92 * math.pi

IDENTITY_QUAT = (1.0, 0.0, 0.0, 0.0)


@dataclass
class SimActorState:
    x: float
    y: float
    z: float
    yaw: float
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    wz: float = 0.0


@dataclass(frozen=True)
class SimSegmentPose:
    """One figure segment's pivot and orientation, in the actor's own frame."""

    name: str
    x: float
    y: float
    z: float
    quat: tuple[float, float, float, float]


# --------------------------------------------------------------- quaternions


def quat_about_x(angle: float) -> tuple[float, float, float, float]:
    return (math.cos(angle / 2.0), math.sin(angle / 2.0), 0.0, 0.0)


def quat_about_y(angle: float) -> tuple[float, float, float, float]:
    return (math.cos(angle / 2.0), 0.0, math.sin(angle / 2.0), 0.0)


def quat_about_z(angle: float) -> tuple[float, float, float, float]:
    return (math.cos(angle / 2.0), 0.0, 0.0, math.sin(angle / 2.0))


def quat_multiply(a, b) -> tuple[float, float, float, float]:
    """Compose two (w, x, y, z) rotations, `a` applied after `b`."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_rotate(q, vector) -> tuple[float, float, float]:
    """Rotate a 3-vector by a (w, x, y, z) quaternion."""
    w, x, y, z = q
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def segment_quaternion(roll: float, pitch: float, growth_sign: float):
    """
    Build a segment rotation from the roll/pitch convention above.

    `growth_sign` is +1 for a segment whose geometry runs along +z from its pivot
    (the torso) and -1 for one that hangs along -z (arms and legs). It is the only
    place the two cases differ.
    """
    return quat_multiply(
        quat_about_x(-growth_sign * roll),
        quat_about_y(growth_sign * pitch),
    )


# ------------------------------------------------------------ exercise moves


def _neutral_posture() -> dict:
    return {name: (0.0, 0.0) for name in SEGMENT_NAMES}


def _march(gait: SimGaitConfig, phase: float) -> dict:
    """Marching leg lifts: legs swing fore-aft, opposite arm leading."""
    posture = _neutral_posture()
    swing = math.sin(phase)
    posture["leg_left"] = (0.0, gait.leg_swing * swing)
    posture["leg_right"] = (0.0, -gait.leg_swing * swing)
    posture["arm_left"] = (0.0, -gait.arm_swing * swing)
    posture["arm_right"] = (0.0, gait.arm_swing * swing)
    # The weight shift shows in the spine, which is the point of having one.
    posture["torso_upper"] = (0.3 * gait.torso_bend * swing, 0.0)
    return posture


def _jacks(gait: SimGaitConfig, phase: float) -> dict:
    """Jumping jacks: arms and legs open together and close together."""
    posture = _neutral_posture()
    openness = 0.5 * (1.0 - math.cos(phase))
    posture["leg_left"] = (gait.leg_spread * openness, 0.0)
    posture["leg_right"] = (-gait.leg_spread * openness, 0.0)
    posture["arm_left"] = (gait.arm_raise * openness, 0.0)
    posture["arm_right"] = (-gait.arm_raise * openness, 0.0)
    return posture


def _step_touch(gait: SimGaitConfig, phase: float) -> dict:
    """Side steps: one leg out at a time, the arm on that side rising with it."""
    posture = _neutral_posture()
    left_out = max(0.0, math.sin(phase))
    right_out = max(0.0, -math.sin(phase))
    posture["leg_left"] = (gait.leg_spread * left_out, 0.0)
    posture["leg_right"] = (-gait.leg_spread * right_out, 0.0)
    posture["arm_left"] = (0.55 * gait.arm_raise * left_out, 0.0)
    posture["arm_right"] = (-0.55 * gait.arm_raise * right_out, 0.0)
    posture["torso_upper"] = (gait.torso_bend * (left_out - right_out), 0.0)
    return posture


def _side_bend(gait: SimGaitConfig, phase: float) -> dict:
    """Arms overhead, bending left and right at both spine joints."""
    posture = _neutral_posture()
    lean = math.sin(phase)
    posture["torso_lower"] = (0.35 * gait.torso_bend * lean, 0.0)
    posture["torso_upper"] = (gait.torso_bend * lean, 0.0)
    overhead = min(OVERHEAD_LIMIT, OVERHEAD_SCALE * gait.arm_raise)
    posture["arm_left"] = (overhead, 0.0)
    posture["arm_right"] = (-overhead, 0.0)
    return posture


EXERCISE_MOVE_FUNCTIONS = {
    "march": _march,
    "jacks": _jacks,
    "step_touch": _step_touch,
    "side_bend": _side_bend,
}


def active_move(gait: SimGaitConfig, phase: float) -> str:
    """
    Which move a gait is performing at this (unwrapped) phase.

    Only `routine` varies: it holds each move for `routine_reps` repetitions
    before moving to the next, and repeats the list forever.
    """
    if not gait.is_active:
        return "none"
    if gait.mode != "routine":
        return gait.mode
    if not gait.routine:
        return "none"

    reps = max(gait.routine_reps, 1e-6)
    index = int((phase / TWO_PI) // reps) % len(gait.routine)
    return gait.routine[index]


def exercise_posture(gait: SimGaitConfig, phase: float) -> dict:
    """Segment name -> (roll, pitch) in radians for this phase."""
    move = active_move(gait, phase)
    move_function = EXERCISE_MOVE_FUNCTIONS.get(move)
    if move_function is None:
        return _neutral_posture()
    return move_function(gait, phase % TWO_PI)


def segment_poses(gait: SimGaitConfig, phase: float) -> tuple[SimSegmentPose, ...]:
    """
    Run the figure's forward kinematics and return every segment's pose.

    Poses are in the actor's own frame — x forward, y left, z up, origin on the
    floor under the actor — so the caller only has to apply the actor's world
    position and yaw. The chain is: the lower torso pivots at the spine base, the
    upper torso pivots at the top of the lower one and bends relative to it, the
    shoulders ride on the upper torso, and the hips stay on the root, which is why
    a deep torso bend does not drag the legs with it.
    """
    posture = exercise_posture(gait, phase)

    lower_quat = segment_quaternion(*posture["torso_lower"], 1.0)
    lower_pivot = (0.0, 0.0, SPINE_BASE_Z)

    upper_offset = quat_rotate(lower_quat, (0.0, 0.0, TORSO_LOWER_LENGTH))
    upper_pivot = tuple(a + b for a, b in zip(lower_pivot, upper_offset))
    upper_quat = quat_multiply(
        lower_quat, segment_quaternion(*posture["torso_upper"], 1.0)
    )

    poses = [
        SimSegmentPose("torso_lower", *lower_pivot, quat=lower_quat),
        SimSegmentPose("torso_upper", *upper_pivot, quat=upper_quat),
    ]

    for side, lateral_sign in (("left", 1.0), ("right", -1.0)):
        shoulder_offset = quat_rotate(
            upper_quat,
            (0.0, lateral_sign * SHOULDER_LATERAL, SHOULDER_LOCAL_Z),
        )
        shoulder = tuple(a + b for a, b in zip(upper_pivot, shoulder_offset))
        arm_quat = quat_multiply(
            upper_quat, segment_quaternion(*posture[f"arm_{side}"], -1.0)
        )
        poses.append(SimSegmentPose(f"arm_{side}", *shoulder, quat=arm_quat))

    for side, lateral_sign in (("left", 1.0), ("right", -1.0)):
        hip = (0.0, lateral_sign * LEG_LATERAL_OFFSET, HIP_Z)
        leg_quat = segment_quaternion(*posture[f"leg_{side}"], -1.0)
        poses.append(SimSegmentPose(f"leg_{side}", *hip, quat=leg_quat))

    return tuple(poses)


def segment_tip(
    pose: SimSegmentPose, length: float, growth_sign: float = -1.0
) -> tuple[float, float, float]:
    """
    Return where a segment's far end lands.

    `growth_sign` matches `segment_quaternion`: -1 for a limb hanging along -z,
    +1 for a torso segment growing along +z.
    """
    direction = quat_rotate(pose.quat, (0.0, 0.0, growth_sign * length))
    return (pose.x + direction[0], pose.y + direction[1], pose.z + direction[2])


# ------------------------------------------------------------------- runtime


class SimActorRuntime:
    def __init__(self, config: SimActorConfig):
        self.config = config
        self.state = SimActorState(
            x=config.x,
            y=config.y,
            z=config.z,
            yaw=config.yaw,
        )
        self._current_waypoint_index = 0
        self._elapsed = 0.0
        self._pause_remaining = 0.0
        # Unwrapped, because a routine needs to count whole repetitions.
        self._gait_phase = config.gait.phase_offset

    @property
    def gait_phase(self) -> float:
        return self._gait_phase

    def active_move(self) -> str:
        return active_move(self.config.gait, self._gait_phase)

    def segment_poses(self) -> tuple[SimSegmentPose, ...]:
        return segment_poses(self.config.gait, self._gait_phase)

    def update(self, dt: float) -> None:
        self._elapsed += dt
        self._update_gait(dt)
        self.state.vx = 0.0
        self.state.vy = 0.0
        self.state.vz = 0.0
        self.state.wz = 0.0

        if self.config.motion_mode != "patrol":
            return
        if self._elapsed < self.config.motion_start_delay:
            return
        if not self.config.motion_waypoints:
            return
        if self._pause_remaining > 0.0:
            self._pause_remaining = max(0.0, self._pause_remaining - dt)
            return
        if self.config.motion_speed <= 0.0:
            return

        waypoint = self.config.motion_waypoints[self._current_waypoint_index]
        dx = waypoint.x - self.state.x
        dy = waypoint.y - self.state.y
        dz = waypoint.z - self.state.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < 1e-3:
            self._arrive(waypoint)
            return

        step = min(self.config.motion_speed * dt, dist)
        scale = step / dist
        new_x = self.state.x + dx * scale
        new_y = self.state.y + dy * scale
        new_z = self.state.z + dz * scale
        new_yaw = (
            math.atan2(dy, dx) if abs(dx) > 1e-6 or abs(dy) > 1e-6 else self.state.yaw
        )

        self.state.vx = (new_x - self.state.x) / dt if dt > 0.0 else 0.0
        self.state.vy = (new_y - self.state.y) / dt if dt > 0.0 else 0.0
        self.state.vz = (new_z - self.state.z) / dt if dt > 0.0 else 0.0
        self.state.wz = self._yaw_rate(self.state.yaw, new_yaw, dt)

        self.state.x = new_x
        self.state.y = new_y
        self.state.z = new_z
        self.state.yaw = new_yaw

        if dist - step < 1e-3:
            self._arrive(waypoint)

    def _update_gait(self, dt: float) -> None:
        gait = self.config.gait
        if not gait.is_active:
            return
        self._gait_phase += TWO_PI * dt / gait.period

    def _arrive(self, waypoint) -> None:
        """Start the waypoint's dwell, then queue the next one."""
        self._pause_remaining = max(self._pause_remaining, waypoint.pause)
        self._advance_waypoint()

    def _advance_waypoint(self) -> None:
        if not self.config.motion_waypoints:
            return
        next_index = self._current_waypoint_index + 1
        if next_index >= len(self.config.motion_waypoints):
            if self.config.motion_loop:
                next_index = 0
            else:
                next_index = len(self.config.motion_waypoints) - 1
        self._current_waypoint_index = next_index

    @staticmethod
    def _yaw_rate(old_yaw: float, new_yaw: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0
        delta = math.atan2(math.sin(new_yaw - old_yaw), math.cos(new_yaw - old_yaw))
        return delta / dt
