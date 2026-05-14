import math
from dataclasses import dataclass

from mecanumbot_core.sim_scenarios import SimActorConfig


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

    def update(self, dt: float) -> None:
        self._elapsed += dt
        self.state.vx = 0.0
        self.state.vy = 0.0
        self.state.vz = 0.0
        self.state.wz = 0.0

        if self.config.motion_mode != 'patrol':
            return
        if self._elapsed < self.config.motion_start_delay:
            return
        if not self.config.motion_waypoints:
            return
        if self.config.motion_speed <= 0.0:
            return

        target_x, target_y, target_z = self.config.motion_waypoints[self._current_waypoint_index]
        dx = target_x - self.state.x
        dy = target_y - self.state.y
        dz = target_z - self.state.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < 1e-3:
            self._advance_waypoint()
            return

        step = min(self.config.motion_speed * dt, dist)
        scale = step / dist
        new_x = self.state.x + dx * scale
        new_y = self.state.y + dy * scale
        new_z = self.state.z + dz * scale
        new_yaw = math.atan2(dy, dx) if abs(dx) > 1e-6 or abs(dy) > 1e-6 else self.state.yaw

        self.state.vx = (new_x - self.state.x) / dt if dt > 0.0 else 0.0
        self.state.vy = (new_y - self.state.y) / dt if dt > 0.0 else 0.0
        self.state.vz = (new_z - self.state.z) / dt if dt > 0.0 else 0.0
        self.state.wz = self._yaw_rate(self.state.yaw, new_yaw, dt)

        self.state.x = new_x
        self.state.y = new_y
        self.state.z = new_z
        self.state.yaw = new_yaw

        if dist - step < 1e-3:
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
