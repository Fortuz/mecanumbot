"""
Mecanum kinematics and OpenCR unit conversions shared by the simulation backends.

Both the MuJoCo and the Gazebo Sim backend have to present the same `opencr_state`
interface as the real `mecanumbot_io_node`, so every tick/angle conversion lives here
rather than in a backend. Changing a constant in this file changes both backends at
once, which is the point: a scenario must behave the same whichever simulator ran it.
"""

import math

# Accessory servo scaling, matching the OpenCR firmware.
TICK_TO_RAD = 0.005061
MIDPOINT_COMPENSATE_CONSTANT = 2.618

# The OpenCR firmware saturates wheel velocity commands at +/- 300 ticks.
WHEEL_TICK_LIMIT = 300

# Wheel-order key used by every dict this module returns.
WHEEL_KEYS = ("bl", "br", "fl", "fr")

# Left-hand wheels are mounted mirrored, so their joint axis runs opposite to the
# direction the OpenCR reports as positive. Both simulation models follow this.
WHEEL_JOINT_SIGN = {"bl": -1.0, "br": 1.0, "fl": -1.0, "fr": 1.0}


class MecanumKinematics:
    """Convert between body twists, wheel angular velocities and OpenCR ticks."""

    def __init__(
        self,
        wheel_radius: float,
        wheel_separation_x: float,
        wheel_separation_y: float,
        vel_tick: float,
    ):
        self.wheel_radius = wheel_radius
        self.wheel_separation_x = wheel_separation_x
        self.wheel_separation_y = wheel_separation_y
        self.vel_tick = vel_tick

        # Metres per second produced by one velocity tick.
        self.scale = (vel_tick / 60.0) * 2.0 * math.pi * wheel_radius
        # Effective lever arm turning a yaw rate into a wheel surface speed.
        self.wheel_dist_scale = (wheel_separation_x + wheel_separation_y) / 2.0
        # Radians per second produced by one velocity tick.
        self.rad_s_per_tick = (vel_tick / 60.0) * 2.0 * math.pi

    def body_twist_to_wheel_ticks(self, vx: float, vy: float, wz: float) -> dict:
        """Mecanum inverse kinematics, in the OpenCR tick units the board expects."""
        yaw_term = wz * self.wheel_dist_scale
        raw = {
            "bl": (vx + vy - yaw_term) / self.scale,
            "br": (vx - vy + yaw_term) / self.scale,
            "fl": (vx - vy - yaw_term) / self.scale,
            "fr": (vx + vy + yaw_term) / self.scale,
        }
        return {
            key: int(max(min(value, WHEEL_TICK_LIMIT), -WHEEL_TICK_LIMIT))
            for key, value in raw.items()
        }

    def wheel_ticks_to_body_twist(self, ticks: dict) -> tuple:
        """Forward kinematics: per-wheel ticks back to a (vx, vy, wz) body twist."""
        bl = ticks["bl"] * self.scale
        br = ticks["br"] * self.scale
        fl = ticks["fl"] * self.scale
        fr = ticks["fr"] * self.scale
        vx = (bl + br + fl + fr) / 4.0
        vy = (bl - br - fl + fr) / 4.0
        wz = (-bl + br - fl + fr) / (4.0 * self.wheel_dist_scale)
        return vx, vy, wz

    def ticks_to_wheel_rad_s(self, tick_value: float) -> float:
        return tick_value * self.rad_s_per_tick

    def wheel_rad_s_to_ticks(self, angular_velocity: float) -> int:
        if abs(self.rad_s_per_tick) < 1e-9:
            return 0
        return int(round(angular_velocity / self.rad_s_per_tick))


def accessory_ticks_to_angle(ticks: float) -> float:
    """Neck/grabber servo ticks to joint angle in radians."""
    return MIDPOINT_COMPENSATE_CONSTANT - (ticks * TICK_TO_RAD)


def joint_angle_to_ticks(angle: float) -> int:
    """Joint angle in radians back to neck/grabber servo ticks."""
    return int(round((MIDPOINT_COMPENSATE_CONSTANT - angle) / TICK_TO_RAD))


def accessory_cmd_to_ticks(position: float) -> int:
    """
    `AccessMotorCmd` field value to servo ticks.

    The behaviour trees and teleop publish `n_pos` in the 2.0...8.6 range documented
    in `mecanumbot_leading_behaviour`, which the real firmware scales by 100.
    """
    return int(position * 100.0)
