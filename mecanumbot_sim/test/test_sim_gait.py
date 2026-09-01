"""
Figure articulation: the quaternion chain and the aerobic moves.

Pure geometry, no ROS and no simulator: run with
`PYTHONPATH=. python3 -m pytest test/test_sim_gait.py` from the package directory.
"""

import math

from mecanumbot_sim.sim_actor_runtime import (
    SimActorRuntime,
    active_move,
    exercise_posture,
    quat_about_z,
    quat_multiply,
    quat_rotate,
    segment_poses,
    segment_quaternion,
    segment_tip,
)
from mecanumbot_sim.sim_scenarios import (
    ARM_LENGTH,
    HIP_Z,
    LEG_LATERAL_OFFSET,
    LEG_LENGTH,
    SEGMENT_NAMES,
    SHOULDER_LATERAL,
    SPINE_BASE_Z,
    TORSO_LOWER_LENGTH,
    TORSO_UPPER_LENGTH,
    SimActorConfig,
    SimGaitConfig,
    SimWaypoint,
)

LIMB = -1.0
TORSO = 1.0


def make_actor(gait: SimGaitConfig, **overrides) -> SimActorConfig:
    fields = dict(
        actor_id=1,
        name="exercising_person",
        kind="human",
        body_name="sim_human_0",
        x=1.0,
        y=0.5,
        z=0.0,
        yaw=0.0,
        is_subject=True,
        visible_to_lidar=True,
        motion_mode="static",
        motion_speed=0.0,
        motion_loop=True,
        motion_start_delay=0.0,
        motion_waypoints=(),
        gait=gait,
    )
    fields.update(overrides)
    return SimActorConfig(**fields)


def poses_by_name(gait: SimGaitConfig, phase: float) -> dict:
    return {pose.name: pose for pose in segment_poses(gait, phase)}


def sample_cycle(gait: SimGaitConfig, samples: int = 64):
    return [
        poses_by_name(gait, 2.0 * math.pi * step / samples) for step in range(samples)
    ]


# ------------------------------------------------------------- the convention


def test_positive_roll_swings_a_hanging_limb_left():
    quat = segment_quaternion(math.radians(30.0), 0.0, LIMB)
    tip = quat_rotate(quat, (0.0, 0.0, -1.0))

    assert tip[1] > 0.0
    assert math.isclose(tip[1], math.sin(math.radians(30.0)), abs_tol=1e-9)


def test_positive_pitch_swings_a_hanging_limb_forward():
    quat = segment_quaternion(0.0, math.radians(30.0), LIMB)
    tip = quat_rotate(quat, (0.0, 0.0, -1.0))

    assert tip[0] > 0.0
    assert math.isclose(tip[0], math.sin(math.radians(30.0)), abs_tol=1e-9)


def test_the_convention_holds_for_an_upward_growing_segment_too():
    """A torso segment leans the same way for the same sign, despite growing +z."""
    roll_quat = segment_quaternion(math.radians(20.0), 0.0, TORSO)
    pitch_quat = segment_quaternion(0.0, math.radians(20.0), TORSO)

    assert quat_rotate(roll_quat, (0.0, 0.0, 1.0))[1] > 0.0
    assert quat_rotate(pitch_quat, (0.0, 0.0, 1.0))[0] > 0.0


# ------------------------------------------------------------------- the rig


def test_the_neutral_figure_matches_the_mjcf_dimensions():
    poses = poses_by_name(SimGaitConfig(), 1.234)

    assert poses["torso_lower"].z == SPINE_BASE_Z
    assert math.isclose(
        segment_tip(poses["torso_lower"], TORSO_LOWER_LENGTH, TORSO)[2],
        SPINE_BASE_Z + TORSO_LOWER_LENGTH,
    )
    assert math.isclose(
        segment_tip(poses["torso_upper"], TORSO_UPPER_LENGTH, TORSO)[2],
        SPINE_BASE_Z + TORSO_LOWER_LENGTH + TORSO_UPPER_LENGTH,
    )
    # The lidar slices the legs between 0.05 and 0.55 m; that is the one part of
    # the figure the detector actually sees, so it must not drift.
    assert poses["leg_left"].z == HIP_Z
    assert poses["leg_left"].y == LEG_LATERAL_OFFSET
    assert math.isclose(
        segment_tip(poses["leg_left"], LEG_LENGTH)[2], HIP_Z - LEG_LENGTH
    )
    assert poses["arm_left"].y == SHOULDER_LATERAL


def test_every_segment_is_posed():
    poses = poses_by_name(SimGaitConfig(mode="jacks", period=1.0), 2.0)

    assert set(poses) == set(SEGMENT_NAMES)


def test_the_shoulders_ride_on_the_bending_spine():
    """A waist bend carries the arms with it, but leaves the legs alone."""
    upright = poses_by_name(SimGaitConfig(), 0.0)
    bent = poses_by_name(SimGaitConfig(mode="side_bend", period=1.0), math.pi / 2.0)

    assert bent["arm_left"].y > upright["arm_left"].y
    assert bent["leg_left"].y == upright["leg_left"].y
    assert bent["leg_left"].z == upright["leg_left"].z


# ------------------------------------------------------------ exercise moves


def test_no_gait_holds_the_neutral_posture():
    posture = exercise_posture(SimGaitConfig(), 1.234)

    assert all(angles == (0.0, 0.0) for angles in posture.values())


def test_a_zero_period_gait_is_inactive():
    gait = SimGaitConfig(mode="march", period=0.0)

    assert gait.is_active is False
    assert exercise_posture(gait, 0.9) == exercise_posture(SimGaitConfig(), 0.9)


def test_march_lifts_the_legs_fore_and_aft_in_antiphase():
    gait = SimGaitConfig(mode="march", period=1.2)
    samples = sample_cycle(gait)

    for poses in samples:
        left = segment_tip(poses["leg_left"], LEG_LENGTH)
        right = segment_tip(poses["leg_right"], LEG_LENGTH)
        # One foot forward is the other foot back, always.
        assert math.isclose(left[0], -right[0], abs_tol=1e-12)
        # A pivoting leg lifts its foot on its own; there is no separate lift term.
        assert left[2] >= HIP_Z - LEG_LENGTH - 1e-12

    travel = [segment_tip(poses["leg_left"], LEG_LENGTH)[0] for poses in samples]
    reach = LEG_LENGTH * math.sin(gait.leg_swing)
    assert math.isclose(max(travel), reach, rel_tol=1e-2)
    assert math.isclose(min(travel), -reach, rel_tol=1e-2)


def test_march_swings_the_arm_opposite_its_own_leg():
    poses = poses_by_name(SimGaitConfig(mode="march", period=1.0), math.pi / 2.0)

    left_foot = segment_tip(poses["leg_left"], LEG_LENGTH)
    left_hand = segment_tip(poses["arm_left"], ARM_LENGTH)
    assert left_foot[0] > 0.0
    assert left_hand[0] < 0.0


def test_jacks_open_and_close_arms_and_legs_together():
    gait = SimGaitConfig(mode="jacks", period=1.4)
    closed = poses_by_name(gait, 0.0)
    open_ = poses_by_name(gait, math.pi)

    closed_feet = segment_tip(closed["leg_left"], LEG_LENGTH)
    open_feet = segment_tip(open_["leg_left"], LEG_LENGTH)
    closed_hand = segment_tip(closed["arm_left"], ARM_LENGTH)
    open_hand = segment_tip(open_["arm_left"], ARM_LENGTH)

    assert math.isclose(closed_feet[1], LEG_LATERAL_OFFSET, abs_tol=1e-9)
    assert open_feet[1] > closed_feet[1]
    assert open_hand[1] > closed_hand[1]
    # Arms go up as they go out, which is what makes it a jack and not a shrug.
    assert open_hand[2] > closed_hand[2]

    for poses in sample_cycle(gait):
        left = segment_tip(poses["leg_left"], LEG_LENGTH)
        right = segment_tip(poses["leg_right"], LEG_LENGTH)
        assert math.isclose(left[1], -right[1], abs_tol=1e-12)


def test_step_touch_moves_one_leg_out_at_a_time():
    gait = SimGaitConfig(mode="step_touch", period=1.0)

    for poses in sample_cycle(gait):
        left_out = segment_tip(poses["leg_left"], LEG_LENGTH)[1] - LEG_LATERAL_OFFSET
        right_out = -segment_tip(poses["leg_right"], LEG_LENGTH)[1] - LEG_LATERAL_OFFSET
        assert left_out >= -1e-9 and right_out >= -1e-9
        assert min(left_out, right_out) < 1e-9


def test_side_bend_bends_both_spine_segments_the_same_way():
    gait = SimGaitConfig(mode="side_bend", period=1.0)
    posture = exercise_posture(gait, math.pi / 2.0)

    lower_roll = posture["torso_lower"][0]
    upper_roll = posture["torso_upper"][0]
    assert lower_roll > 0.0 and upper_roll > 0.0
    # The upper joint carries most of the bend, and they compound.
    assert upper_roll > lower_roll

    poses = poses_by_name(gait, math.pi / 2.0)
    crown = segment_tip(poses["torso_upper"], TORSO_UPPER_LENGTH, TORSO)
    assert crown[1] > poses["torso_lower"].y


def test_side_bend_puts_the_arms_overhead():
    poses = poses_by_name(SimGaitConfig(mode="side_bend", period=1.0), 0.0)

    for side in ("arm_left", "arm_right"):
        hand = segment_tip(poses[side], ARM_LENGTH)
        assert hand[2] > poses[side].z


# ---------------------------------------------------------------- the routine


def test_a_routine_cycles_its_moves_in_order():
    gait = SimGaitConfig(
        mode="routine", period=1.0, routine=("jacks", "march"), routine_reps=2.0
    )
    two_pi = 2.0 * math.pi

    assert active_move(gait, 0.0) == "jacks"
    assert active_move(gait, 1.9 * two_pi) == "jacks"
    assert active_move(gait, 2.1 * two_pi) == "march"
    assert active_move(gait, 3.9 * two_pi) == "march"
    # ...and then round again.
    assert active_move(gait, 4.1 * two_pi) == "jacks"


def test_a_single_move_gait_never_switches():
    gait = SimGaitConfig(mode="jacks", period=1.0)

    assert active_move(gait, 0.0) == "jacks"
    assert active_move(gait, 97.3) == "jacks"


def test_the_runtime_walks_through_a_whole_routine():
    runtime = SimActorRuntime(
        make_actor(
            SimGaitConfig(
                mode="routine",
                period=1.0,
                routine=("jacks", "march", "side_bend"),
                routine_reps=2.0,
            )
        )
    )

    seen = []
    for _ in range(700):
        runtime.update(0.01)
        if not seen or seen[-1] != runtime.active_move():
            seen.append(runtime.active_move())

    assert seen[:3] == ["jacks", "march", "side_bend"]


# ------------------------------------------------------------------- motion


def test_phase_offset_desynchronises_two_actors():
    plain = SimActorRuntime(make_actor(SimGaitConfig(mode="march", period=1.0)))
    offset = SimActorRuntime(
        make_actor(SimGaitConfig(mode="march", period=1.0, phase_offset=math.pi))
    )

    plain_foot = segment_tip(
        poses_by_name(plain.config.gait, plain.gait_phase)["leg_left"], LEG_LENGTH
    )
    offset_foot = segment_tip(
        poses_by_name(offset.config.gait, offset.gait_phase)["leg_left"], LEG_LENGTH
    )
    assert plain_foot[0] != offset_foot[0]


def test_a_gait_never_translates_the_actor():
    runtime = SimActorRuntime(make_actor(SimGaitConfig(mode="jacks", period=1.0)))

    for _ in range(200):
        runtime.update(0.01)

    assert runtime.state.x == 1.0
    assert runtime.state.y == 0.5
    assert runtime.state.vx == 0.0
    assert runtime.state.vy == 0.0


def test_a_walking_actor_can_also_be_exercising():
    runtime = SimActorRuntime(
        make_actor(
            SimGaitConfig(mode="march", period=1.0),
            motion_mode="patrol",
            motion_speed=0.3,
            motion_waypoints=(SimWaypoint(2.0, 0.5),),
        )
    )

    for _ in range(100):
        runtime.update(0.01)

    assert runtime.state.x > 1.0
    assert runtime.gait_phase > 0.0


def test_a_waypoint_pause_holds_the_actor_in_place():
    runtime = SimActorRuntime(
        make_actor(
            SimGaitConfig(mode="jacks", period=1.0),
            motion_mode="patrol",
            motion_speed=0.5,
            motion_loop=True,
            motion_waypoints=(
                SimWaypoint(1.2, 0.5, pause=3.0),
                SimWaypoint(2.0, 0.5),
            ),
        )
    )

    # Reach the first waypoint, then sit on it.
    for _ in range(100):
        runtime.update(0.01)
    arrived_x = runtime.state.x
    assert math.isclose(arrived_x, 1.2, abs_tol=1e-6)

    for _ in range(200):
        runtime.update(0.01)
    assert math.isclose(runtime.state.x, arrived_x, abs_tol=1e-9)
    # ...but the workout carries on through the dwell.
    assert runtime.gait_phase > 0.0

    for _ in range(300):
        runtime.update(0.01)
    assert runtime.state.x > arrived_x


def test_world_placement_composes_yaw_with_the_segment_pose():
    """What the backend does to put an actor-frame segment into the world."""
    poses = poses_by_name(SimGaitConfig(mode="jacks", period=1.0), math.pi)
    yaw_quat = quat_about_z(math.pi / 2.0)
    arm = poses["arm_left"]

    offset = quat_rotate(yaw_quat, (arm.x, arm.y, arm.z))
    world_quat = quat_multiply(yaw_quat, arm.quat)

    # Facing +y, the actor's left shoulder points to -x.
    assert math.isclose(offset[0], -arm.y, abs_tol=1e-9)
    assert math.isclose(offset[2], arm.z, abs_tol=1e-9)
    assert math.isclose(sum(value * value for value in world_quat), 1.0, abs_tol=1e-9)
