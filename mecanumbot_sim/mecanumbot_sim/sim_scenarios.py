import math
from dataclasses import dataclass, field
from pathlib import Path

import yaml

SUPPORTED_ACTOR_BODIES = (
    "sim_human_0",
    "sim_human_1",
    "sim_wall_panel_0",
    "sim_wall_panel_1",
)

# Props are scenery rather than tracked actors: mats are kinematic floor markings,
# cubes are free bodies the robot can push and grip. They are pooled the same way
# actors are, because on MuJoCo the bodies have to exist in the MJCF before a
# scenario names them.
SUPPORTED_PROP_BODIES = (
    "sim_mat_0",
    "sim_mat_1",
    "sim_cube_0",
    "sim_cube_1",
    "sim_cube_2",
    "sim_cube_3",
)

#: Aerobic moves an exercising actor can perform, one repetition per gait period.
EXERCISE_MOVES = (
    "march",
    "jacks",
    "step_touch",
    "side_bend",
)

#: `routine` cycles through several moves; it is not itself a move, so it cannot
#: appear inside a routine list.
SUPPORTED_GAIT_MODES = ("none",) + EXERCISE_MOVES + ("routine",)

# ---------------------------------------------------------------------------
# Figure dimensions, in metres, all mirrored by the `sim_human_*` bodies in
# `mujoco/mecanumbot.xml`. The forward kinematics in `sim_actor_runtime.py`
# chains these, so a number changed here has to be changed there too.
#
# A segment's geometry is defined about its own pivot at the body origin: the two
# torso segments grow along +z, the four limbs hang along -z.
# ---------------------------------------------------------------------------

#: Lateral offset of each leg from the actor's centre line.
LEG_LATERAL_OFFSET = 0.11
#: Height of the hip joints the legs pivot about.
HIP_Z = 0.55
LEG_LENGTH = 0.50

#: Height of the spine base, where the lower torso segment pivots.
SPINE_BASE_Z = 0.45
TORSO_LOWER_LENGTH = 0.40
TORSO_UPPER_LENGTH = 0.30

#: Shoulder position in the upper torso segment's own frame.
SHOULDER_LATERAL = 0.17
SHOULDER_LOCAL_Z = 0.24
ARM_LENGTH = 0.42

#: Segment body-name suffixes, in the order `segment_poses` returns them.
SEGMENT_NAMES = (
    "torso_lower",
    "torso_upper",
    "arm_left",
    "arm_right",
    "leg_left",
    "leg_right",
)

#: A standard yoga mat, used when a mat prop gives no explicit size.
YOGA_MAT_LENGTH = 1.73
YOGA_MAT_WIDTH = 0.61


@dataclass(frozen=True)
class SimGaitConfig:
    """
    An aerobic routine layered on top of whatever the actor's motion mode does.

    A gait only articulates the segments; it never moves the actor itself, so an
    exercising person is `motion.mode: static` plus a gait, and one who works out
    and then walks away is `motion.mode: patrol` plus a gait.

    Amplitudes are angles in radians. The scenario spells them in degrees with the
    `_deg` suffix this workspace uses elsewhere, and the loader converts.
    """

    mode: str = "none"
    period: float = 1.5
    #: Fore-aft leg lift, the marching/knee-raise amplitude.
    leg_swing: float = math.radians(32.0)
    #: Lateral leg opening, the jumping-jack amplitude.
    leg_spread: float = math.radians(28.0)
    #: Fore-aft arm swing.
    arm_swing: float = math.radians(55.0)
    #: Lateral arm raise. Doubled and clamped for the overhead moves.
    arm_raise: float = math.radians(95.0)
    #: Bend at each spine joint.
    torso_bend: float = math.radians(18.0)
    phase_offset: float = 0.0
    #: Moves `routine` cycles through, and how many repetitions of each.
    routine: tuple[str, ...] = ("jacks", "march", "side_bend")
    routine_reps: float = 4.0

    @property
    def is_active(self) -> bool:
        return self.mode != "none" and self.period > 0.0


@dataclass(frozen=True)
class SimWaypoint:
    """One leg of a patrol, plus how long the actor waits on arriving."""

    x: float
    y: float
    z: float = 0.0
    pause: float = 0.0


@dataclass(frozen=True)
class SimActorConfig:
    actor_id: int
    name: str
    kind: str
    body_name: str
    x: float
    y: float
    z: float
    yaw: float
    is_subject: bool
    visible_to_lidar: bool
    motion_mode: str
    motion_speed: float
    motion_loop: bool
    motion_start_delay: float
    motion_waypoints: tuple[SimWaypoint, ...]
    gait: SimGaitConfig = field(default_factory=SimGaitConfig)


@dataclass(frozen=True)
class SimPropConfig:
    """
    A mat or a cube placed in the arena.

    `size` is normalised to a tuple: one entry for a cube edge or a square mat,
    two for a rectangular mat's (length, width). `None` means the MJCF default.
    """

    prop_id: int
    name: str
    kind: str
    body_name: str
    x: float
    y: float
    z: float
    yaw: float
    size: tuple[float, ...] | None = None
    mass: float | None = None

    def mat_extents(self) -> tuple[float, float]:
        """Return the mat's (length, width), defaulting to a yoga mat."""
        if self.size is None:
            return YOGA_MAT_LENGTH, YOGA_MAT_WIDTH
        if len(self.size) == 1:
            return self.size[0], self.size[0]
        return self.size[0], self.size[1]


@dataclass(frozen=True)
class SimScenarioConfig:
    name: str
    actors: tuple[SimActorConfig, ...]
    props: tuple[SimPropConfig, ...] = ()

    @property
    def subject_actor(self) -> SimActorConfig | None:
        for actor in self.actors:
            if actor.is_subject:
                return actor
        return None


def _load_gait(payload: dict, scenario_path: Path) -> SimGaitConfig:
    mode = str(payload.get("mode", "none"))
    if mode not in SUPPORTED_GAIT_MODES:
        raise ValueError(f'Unsupported gait mode "{mode}" in {scenario_path}')

    routine = tuple(str(move) for move in payload.get("routine", ()))
    for move in routine:
        if move not in EXERCISE_MOVES:
            raise ValueError(
                f'Unsupported routine move "{move}" in {scenario_path}; '
                f"a routine is a sequence of {list(EXERCISE_MOVES)}"
            )
    if mode == "routine" and not routine:
        routine = SimGaitConfig().routine

    defaults = SimGaitConfig()

    def angle(key: str, fallback: float) -> float:
        if f"{key}_deg" in payload:
            return math.radians(float(payload[f"{key}_deg"]))
        return fallback

    return SimGaitConfig(
        mode=mode,
        period=float(payload.get("period", defaults.period)),
        leg_swing=angle("leg_swing", defaults.leg_swing),
        leg_spread=angle("leg_spread", defaults.leg_spread),
        arm_swing=angle("arm_swing", defaults.arm_swing),
        arm_raise=angle("arm_raise", defaults.arm_raise),
        torso_bend=angle("torso_bend", defaults.torso_bend),
        phase_offset=angle("phase_offset", defaults.phase_offset),
        routine=routine or defaults.routine,
        routine_reps=float(payload.get("routine_reps", defaults.routine_reps)),
    )


def _load_size(payload: dict) -> tuple[float, ...] | None:
    size = payload.get("size")
    if size is None:
        return None
    if isinstance(size, (list, tuple)):
        return tuple(float(value) for value in size)
    return (float(size),)


def _load_prop(payload: dict, index: int, scenario_path: Path) -> SimPropConfig:
    body_name = str(payload["body_name"])
    if body_name not in SUPPORTED_PROP_BODIES:
        raise ValueError(
            f'Unsupported scenario prop body "{body_name}" in {scenario_path}'
        )

    pose_payload = payload.get("pose", {})
    mass = payload.get("mass")
    return SimPropConfig(
        prop_id=int(payload.get("id", index)),
        name=str(payload.get("name", body_name)),
        kind=str(payload.get("kind", body_name.rsplit("_", 1)[0].removeprefix("sim_"))),
        body_name=body_name,
        x=float(pose_payload.get("x", 0.0)),
        y=float(pose_payload.get("y", 0.0)),
        z=float(pose_payload.get("z", 0.0)),
        yaw=float(pose_payload.get("yaw", 0.0)),
        size=_load_size(payload),
        mass=None if mass is None else float(mass),
    )


def load_sim_scenario(path: str) -> SimScenarioConfig:
    scenario_path = Path(path)
    with scenario_path.open("r", encoding="utf-8") as stream:
        payload = yaml.safe_load(stream) or {}

    scenario_payload = payload.get("scenario", payload)
    actor_payloads = scenario_payload.get("actors", [])
    actors = []
    for index, actor_payload in enumerate(actor_payloads):
        body_name = str(actor_payload["body_name"])
        if body_name not in SUPPORTED_ACTOR_BODIES:
            raise ValueError(
                f'Unsupported scenario body "{body_name}" in {scenario_path}'
            )

        pose_payload = actor_payload.get("pose", {})
        motion_payload = actor_payload.get("motion", {})
        waypoints = tuple(
            SimWaypoint(
                x=float(waypoint.get("x", 0.0)),
                y=float(waypoint.get("y", 0.0)),
                z=float(waypoint.get("z", 0.0)),
                pause=float(waypoint.get("pause", 0.0)),
            )
            for waypoint in motion_payload.get("waypoints", [])
        )
        actors.append(
            SimActorConfig(
                actor_id=int(actor_payload.get("id", index)),
                name=str(actor_payload.get("name", body_name)),
                kind=str(actor_payload.get("kind", "unknown")),
                body_name=body_name,
                x=float(pose_payload.get("x", 0.0)),
                y=float(pose_payload.get("y", 0.0)),
                z=float(pose_payload.get("z", 0.0)),
                yaw=float(pose_payload.get("yaw", 0.0)),
                is_subject=bool(actor_payload.get("is_subject", False)),
                visible_to_lidar=bool(actor_payload.get("visible_to_lidar", True)),
                motion_mode=str(motion_payload.get("mode", "static")),
                motion_speed=float(motion_payload.get("speed", 0.0)),
                motion_loop=bool(motion_payload.get("loop", True)),
                motion_start_delay=float(motion_payload.get("start_delay", 0.0)),
                motion_waypoints=waypoints,
                gait=_load_gait(actor_payload.get("gait", {}), scenario_path),
            )
        )

    props = tuple(
        _load_prop(prop_payload, index, scenario_path)
        for index, prop_payload in enumerate(scenario_payload.get("props", []))
    )

    used_prop_bodies = [prop.body_name for prop in props]
    if len(set(used_prop_bodies)) != len(used_prop_bodies):
        raise ValueError(f"Scenario {scenario_path} reuses a prop body")

    return SimScenarioConfig(
        name=str(scenario_payload.get("name", scenario_path.stem)),
        actors=tuple(actors),
        props=props,
    )
