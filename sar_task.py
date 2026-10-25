"""Privileged task definition: simulator ground truth, reward, success, and episode limits.

Everything here may use ground truth (the target pose from the versioned world). Nothing here may be
passed into the policy observation: sar_observation.py has no target input, and tests enforce that it
never imports this module.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

WORLD_SDF = Path(__file__).resolve().with_name("sim") / "worlds" / "sar_walls.sdf"


@dataclass(frozen=True)
class Target:
    """Target centre in PX4 local NED (origin = Gazebo world origin, measured offset ~5 cm)."""
    x_ned: float
    y_ned: float
    z_enu: float
    radius: float


def load_target(world_sdf: Path = WORLD_SDF, name: str = "target") -> Target:
    """Read the target sphere from the world SDF and convert Gazebo ENU to PX4 NED (x=y_enu, y=x_enu)."""
    world = ET.parse(world_sdf).getroot().find("world")
    models = [m for m in world.findall("model") if m.get("name") == name]
    if len(models) != 1:
        raise ValueError(f"{world_sdf}: expected exactly one model named {name!r}")
    x, y, z = (float(v) for v in models[0].findtext("pose").split()[:3])
    radii = {float(r.text) for r in models[0].iter("radius")}
    if len(radii) != 1:
        raise ValueError(f"{world_sdf}: target collision/visual radii disagree: {sorted(radii)}")
    return Target(x_ned=y, y_ned=x, z_enu=z, radius=radii.pop())


@dataclass(frozen=True)
class TaskConfig:
    success_radius: float = 0.75     # m, horizontal distance from the target centre
    step_cost: float = 0.1           # per step
    progress_weight: float = 1.0     # per metre of horizontal distance reduction
    collision_penalty: float = 100.0
    success_bonus: float = 100.0
    max_steps: int = 600             # 60 s of simulation time at 0.1 s per step


@dataclass(frozen=True)
class StepOutcome:
    reward: float
    terminated: bool
    truncated: bool
    reason: str | None               # "success", "collision", "timeout", or None


RESET_OPTIONS = ("start_yaw_offset",)


def start_yaw_offset(options) -> float:
    """Validated start-heading offset (rad, NED: positive = clockwise) from reset(options), 0.0 if absent.

    Shared by the live env and the stand-in so both interpret it identically. Unknown option keys, a
    non-finite value or |offset| > pi raise ValueError (fail closed: no episode starts on a bad request).
    """
    if not options:
        return 0.0
    unknown = set(options) - set(RESET_OPTIONS)
    if unknown:
        raise ValueError(f"unknown reset options {sorted(unknown)}; supported: {list(RESET_OPTIONS)}")
    value = options.get("start_yaw_offset", 0.0)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) \
            or abs(value) > math.pi:
        raise ValueError(f"start_yaw_offset must be a finite angle in [-pi, pi] rad, got {value!r}")
    return float(value)


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def horizontal_distance(x_ned: float, y_ned: float, target: Target) -> float:
    return math.hypot(target.x_ned - x_ned, target.y_ned - y_ned)


def step_outcome(previous_distance: float, distance: float, collision: bool, steps: int,
                 config: TaskConfig = TaskConfig()) -> StepOutcome:
    """Reward and episode end for one step. Collision takes precedence over success."""
    values = (previous_distance, distance)
    if not all(math.isfinite(v) for v in values):
        raise ValueError(f"non-finite distance in reward: {values}")
    reward = -config.step_cost + config.progress_weight * (previous_distance - distance)
    reached = distance <= config.success_radius
    if collision:
        return StepOutcome(reward - config.collision_penalty, True, False, "collision")
    if reached:
        return StepOutcome(reward + config.success_bonus, True, False, "success")
    if steps >= config.max_steps:
        return StepOutcome(reward, False, True, "timeout")
    return StepOutcome(reward, False, False, None)
