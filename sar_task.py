"""Privileged task definition: simulator ground truth, reward, success, and episode limits.

Everything here may use ground truth (the target pose from the versioned world). Nothing here may be
passed into the policy observation: sar_observation.py has no target input, and tests enforce that it
never imports this module.
"""

from __future__ import annotations

import hashlib
import math
import os
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

SIM_WORLDS = Path(__file__).resolve().with_name("sim") / "worlds"
WORLD_SDF = SIM_WORLDS / "sar_walls.sdf"          # the default world
# Stage C: optional world variants, sim/worlds/variants/<name>/sar_walls.sdf, each the default world with only the
# target pose changed (same world name, so every topic is unchanged). Selected for every process of a run by one
# environment variable; unset means the default world.
WORLD_VARIANTS = SIM_WORLDS / "variants"
WORLD_VARIANT_ENV = "SAR_WORLD_VARIANT"
_VARIANT_NAME = re.compile(r"[a-z0-9][a-z0-9_]{0,39}")


def world_variant_sdf(variant: str) -> Path:
    """The world file of a named variant; ValueError for a malformed or unknown name."""
    if not isinstance(variant, str) or not _VARIANT_NAME.fullmatch(variant):
        raise ValueError(f"malformed world variant name {variant!r}")
    path = WORLD_VARIANTS / variant / WORLD_SDF.name
    if not path.is_file():
        known = sorted(p.name for p in WORLD_VARIANTS.glob("*") if (p / WORLD_SDF.name).is_file())
        raise ValueError(f"unknown world variant {variant!r}; known: {known}")
    return path


def selected_world_variant(environ=None) -> str | None:
    """The variant named by SAR_WORLD_VARIANT, None if unset. Set but empty or invalid is an error."""
    value = (os.environ if environ is None else environ).get(WORLD_VARIANT_ENV)
    if value is None:
        return None
    world_variant_sdf(value)                       # validate
    return value


def selected_world_sdf(environ=None) -> Path:
    variant = selected_world_variant(environ)
    return WORLD_SDF if variant is None else world_variant_sdf(variant)


def file_sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


@dataclass(frozen=True)
class Target:
    """Target centre in PX4 local NED (origin = Gazebo world origin, measured offset ~5 cm)."""
    x_ned: float
    y_ned: float
    z_enu: float
    radius: float


def load_target(world_sdf: Path | None = None, name: str = "target") -> Target:
    """Read the target sphere from the world SDF and convert Gazebo ENU to PX4 NED (x=y_enu, y=x_enu).

    Without an explicit file this reads the selected world (SAR_WORLD_VARIANT, resolved at call time)."""
    if world_sdf is None:
        world_sdf = selected_world_sdf()
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
