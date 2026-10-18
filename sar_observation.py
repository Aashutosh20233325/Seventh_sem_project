"""Policy observation contract ``sar_obs_v1`` (65 float32 values) and its builder.

The builder's inputs are realistic onboard data only: LiDAR sectors, the previous action, PX4 EKF
estimates, and sensor-derived target perception. It has no target-position input and does not import
the privileged task module (sar_task), so ground truth cannot reach the policy through this code path.

Frames: NED for PX4 estimates (heading clockwise from north). Body FLU for the policy (+x forward,
+y left). A body bearing b (+left) corresponds to the NED direction heading - b.

The target "estimate" is the last range-valid detection projected into the local NED frame with the
EKF pose at that step, then re-projected into the body frame on every later step (dead reckoning). It
expires ``memory_expiry_s`` of simulation time after the last range-valid detection.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

VERSION = "sar_obs_v1"
LIDAR_SECTORS = 49
LIDAR_RANGE_MAX = 30.0       # m, lidar_2d_v2 <range><max>
VELOCITY_SCALE = 1.5         # m/s
YAW_RATE_SCALE = 1.0         # rad/s
HEIGHT_SCALE = 3.0           # m
TARGET_RANGE_SCALE = 10.0    # m

# (name, size, low, high, description)
FIELDS = (
    ("lidar", LIDAR_SECTORS, 0.0, 1.0, "minimum range per sector / 30 m, 49 equal sectors over +-135 deg"),
    ("prev_speed", 1, 0.0, 1.0, "previous action[0] (forward speed command, 1 = 1 m/s)"),
    ("prev_turn", 1, -1.0, 1.0, "previous action[1] (NED yaw-rate command, 1 = +0.5 rad/s = clockwise/right)"),
    ("vel_forward", 1, -1.0, 1.0, "EKF horizontal velocity along body x / 1.5 m/s, clipped"),
    ("vel_left", 1, -1.0, 1.0, "EKF horizontal velocity along body y (+left) / 1.5 m/s, clipped"),
    ("yaw_rate", 1, -1.0, 1.0, "body yaw rate (+left/CCW) / 1 rad/s, clipped"),
    ("height", 1, 0.0, 1.0, "height above EKF origin (-z NED) / 3 m, clipped"),
    ("det_now", 1, 0.0, 1.0, "1 if a fresh perception message reports the target, else 0"),
    ("det_bearing_sin", 1, -1.0, 1.0, "sin(measured bearing), 0 when det_now = 0"),
    ("det_bearing_cos", 1, -1.0, 1.0, "cos(measured bearing), 0 when det_now = 0"),
    ("det_range_valid", 1, 0.0, 1.0, "1 if the fresh detection includes a valid depth range"),
    ("det_range", 1, 0.0, 1.0, "measured horizontal range / 10 m, clipped; 0 when invalid"),
    ("est_valid", 1, 0.0, 1.0, "1 if a dead-reckoned target estimate exists and has not expired"),
    ("est_bearing_sin", 1, -1.0, 1.0, "sin(estimate bearing in current body frame), 0 when invalid"),
    ("est_bearing_cos", 1, -1.0, 1.0, "cos(estimate bearing in current body frame), 0 when invalid"),
    ("est_range", 1, 0.0, 1.0, "estimate horizontal range / 10 m, clipped; 0 when invalid"),
    ("est_age", 1, 0.0, 1.0, "time since last range-valid detection / expiry; 1 when none or expired"),
)
SLICES = {}
_start = 0
for _name, _size, *_ in FIELDS:
    SLICES[_name] = slice(_start, _start + _size)
    _start += _size
DIM = _start
LOW = np.concatenate([np.full(s, lo, np.float32) for _, s, lo, _, _ in FIELDS])
HIGH = np.concatenate([np.full(s, hi, np.float32) for _, s, _, hi, _ in FIELDS])


@dataclass(frozen=True)
class EgoState:
    """PX4 EKF estimates in local NED. x and y are used only for target dead reckoning, never output."""
    x: float
    y: float
    z: float
    vx: float
    vy: float
    heading: float          # rad, NED (clockwise from north)
    yaw_rate: float         # rad/s, body FLU (+CCW/left)


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def lidar_sectors_from_raw(raw_sectors_m):
    raw = np.asarray(raw_sectors_m, np.float64)
    if raw.shape != (LIDAR_SECTORS,) or not np.all(np.isfinite(raw)):
        raise ValueError(f"LiDAR sectors must be {LIDAR_SECTORS} finite ranges")
    return np.clip(raw / LIDAR_RANGE_MAX, 0.0, 1.0)


class ObservationBuilder:
    def __init__(self, memory_expiry_s: float = 8.0, perception_max_age_s: float = 1.0):
        self.memory_expiry_s = memory_expiry_s
        self.perception_max_age_s = perception_max_age_s
        self.reset()

    def reset(self):
        self.estimate = None          # (x_ned, y_ned) of the target from sensor + EKF
        self.estimate_time = None     # simulation seconds of the last range-valid detection
        self.last_used_stamp = -math.inf

    def _fresh(self, perception, age_s):
        return (
            perception is not None
            and age_s is not None
            and math.isfinite(age_s)
            and 0.0 <= age_s <= self.perception_max_age_s
        )

    def build(self, lidar_m, prev_action, ego: EgoState, perception, perception_age_s, now_s):
        """Return the sar_obs_v1 vector.

        ``perception`` is a decoded target_perception message (dict) or None. ``perception_age_s`` is the
        host-receipt age of that message. ``now_s`` is the current simulation time in seconds.
        """
        values = (ego.x, ego.y, ego.z, ego.vx, ego.vy, ego.heading, ego.yaw_rate, now_s)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("non-finite ego state or time in observation")
        prev = np.asarray(prev_action, np.float64)
        if prev.shape != (2,) or not np.all(np.isfinite(prev)):
            raise ValueError("previous action must be 2 finite values")
        obs = np.zeros(DIM, np.float32)
        obs[SLICES["lidar"]] = lidar_sectors_from_raw(lidar_m)
        obs[SLICES["prev_speed"]] = np.clip(prev[0], 0.0, 1.0)
        obs[SLICES["prev_turn"]] = np.clip(prev[1], -1.0, 1.0)
        c, s = math.cos(ego.heading), math.sin(ego.heading)
        v_forward = ego.vx * c + ego.vy * s
        v_left = ego.vx * s - ego.vy * c
        obs[SLICES["vel_forward"]] = np.clip(v_forward / VELOCITY_SCALE, -1, 1)
        obs[SLICES["vel_left"]] = np.clip(v_left / VELOCITY_SCALE, -1, 1)
        obs[SLICES["yaw_rate"]] = np.clip(ego.yaw_rate / YAW_RATE_SCALE, -1, 1)
        obs[SLICES["height"]] = np.clip(-ego.z / HEIGHT_SCALE, 0, 1)

        if self._fresh(perception, perception_age_s) and perception["detected"]:
            b = perception["bearing"]
            obs[SLICES["det_now"]] = 1.0
            obs[SLICES["det_bearing_sin"]] = math.sin(b)
            obs[SLICES["det_bearing_cos"]] = math.cos(b)
            if perception["range_valid"]:
                r = perception["horizontal_range"]
                obs[SLICES["det_range_valid"]] = 1.0
                obs[SLICES["det_range"]] = min(r / TARGET_RANGE_SCALE, 1.0)
                if perception["image_stamp"] > self.last_used_stamp:
                    world = ego.heading - b
                    self.estimate = (ego.x + r * math.cos(world), ego.y + r * math.sin(world))
                    self.estimate_time = now_s
                    self.last_used_stamp = perception["image_stamp"]

        if self.estimate is not None and now_s - self.estimate_time <= self.memory_expiry_s:
            dx, dy = self.estimate[0] - ego.x, self.estimate[1] - ego.y
            bearing = wrap(ego.heading - math.atan2(dy, dx))
            obs[SLICES["est_valid"]] = 1.0
            obs[SLICES["est_bearing_sin"]] = math.sin(bearing)
            obs[SLICES["est_bearing_cos"]] = math.cos(bearing)
            obs[SLICES["est_range"]] = min(math.hypot(dx, dy) / TARGET_RANGE_SCALE, 1.0)
            obs[SLICES["est_age"]] = max(0.0, now_s - self.estimate_time) / self.memory_expiry_s
        else:
            obs[SLICES["est_age"]] = 1.0
        return obs


def describe():
    """Markdown table of the contract (used by docs/OBSERVATION_CONTRACT.md)."""
    rows = [f"| index | name | low | high | meaning |", "|---|---|---|---|---|"]
    for name, size, lo, hi, doc in FIELDS:
        sl = SLICES[name]
        idx = f"{sl.start}" if size == 1 else f"{sl.start}-{sl.stop - 1}"
        rows.append(f"| {idx} | {name} | {lo:g} | {hi:g} | {doc} |")
    return "\n".join(rows)
