from dataclasses import dataclass
from pathlib import Path

import yaml


SUPPORTED_ACTOR_BODIES = (
    'sim_human_0',
    'sim_human_1',
    'sim_wall_panel_0',
    'sim_wall_panel_1',
)


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
    motion_waypoints: tuple[tuple[float, float, float], ...]


@dataclass(frozen=True)
class SimScenarioConfig:
    name: str
    actors: tuple[SimActorConfig, ...]

    @property
    def subject_actor(self) -> SimActorConfig | None:
        for actor in self.actors:
            if actor.is_subject:
                return actor
        return None


def load_sim_scenario(path: str) -> SimScenarioConfig:
    scenario_path = Path(path)
    with scenario_path.open('r', encoding='utf-8') as stream:
        payload = yaml.safe_load(stream) or {}

    scenario_payload = payload.get('scenario', payload)
    actor_payloads = scenario_payload.get('actors', [])
    actors = []
    for index, actor_payload in enumerate(actor_payloads):
        body_name = str(actor_payload['body_name'])
        if body_name not in SUPPORTED_ACTOR_BODIES:
            raise ValueError(f'Unsupported scenario body "{body_name}" in {scenario_path}')

        pose_payload = actor_payload.get('pose', {})
        motion_payload = actor_payload.get('motion', {})
        waypoint_payloads = motion_payload.get('waypoints', [])
        waypoints = tuple(
            (
                float(waypoint.get('x', 0.0)),
                float(waypoint.get('y', 0.0)),
                float(waypoint.get('z', 0.0)),
            )
            for waypoint in waypoint_payloads
        )
        actors.append(
            SimActorConfig(
                actor_id=int(actor_payload.get('id', index)),
                name=str(actor_payload.get('name', body_name)),
                kind=str(actor_payload.get('kind', 'unknown')),
                body_name=body_name,
                x=float(pose_payload.get('x', 0.0)),
                y=float(pose_payload.get('y', 0.0)),
                z=float(pose_payload.get('z', 0.0)),
                yaw=float(pose_payload.get('yaw', 0.0)),
                is_subject=bool(actor_payload.get('is_subject', False)),
                visible_to_lidar=bool(actor_payload.get('visible_to_lidar', True)),
                motion_mode=str(motion_payload.get('mode', 'static')),
                motion_speed=float(motion_payload.get('speed', 0.0)),
                motion_loop=bool(motion_payload.get('loop', True)),
                motion_start_delay=float(motion_payload.get('start_delay', 0.0)),
                motion_waypoints=waypoints,
            )
        )

    return SimScenarioConfig(
        name=str(scenario_payload.get('name', scenario_path.stem)),
        actors=tuple(actors),
    )
