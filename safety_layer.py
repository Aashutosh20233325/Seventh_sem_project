"""LiDAR action safety filter for any sar_obs_v1 policy (e.g. PPO). Evaluation-only so far; not used in flight.

It applies the baseline controller's existing obstacle rule to a policy's action, using only the observation's
LiDAR sectors, and adds conservative handling for bad sensor data. Rules, first match wins:

1. ``lidar_invalid``: the LiDAR slice is missing, the wrong length, non-finite, or outside [0, 1]. Command a
   hover (speed 0, turn 0). Missing data is never treated as clear space, and the filter does not turn
   blindly either.
2. ``lidar_stale``: ``lidar_age_s`` is None (unknown) or greater than ``max_age_s``. Hover as in rule 1.
3. ``emergency``: any sector is closer than ``emergency_clearance`` (0.5 m, twice the 0.25 m collision
   threshold). Speed 0, and turn toward the clearer side.
4. ``stop``: the front clearance (minimum over |angle| <= 35 deg) is below ``stop_clearance``. Speed 0, and
   turn toward the clearer side. This is the baseline rule.
5. ``slow``: the front clearance is below ``slow_clearance``. Cap speed at
   0.8 * (front - stop) / (slow - stop), keeping the policy's turn. This is the baseline rule.
6. Otherwise the policy's action passes unchanged.

"Toward the clearer side" is the baseline's choice: turn = -0.8 (left, CCW) if the mean left-half range is
larger, else +0.8 (right). The filter only reduces speed and replaces turn; it never adds forward speed.
Stopping and turning is not path planning. A filtered policy can avoid a collision and still be stuck behind
an obstacle.
"""

from __future__ import annotations

import math

import numpy as np

import sar_observation as so
from baseline_controller import FRONT, SECTOR_ANGLES, BaselineController

_BASELINE = BaselineController()
STOP_CLEARANCE = _BASELINE.stop_clearance          # 1.2 m, shared with the baseline
SLOW_CLEARANCE = _BASELINE.slow_clearance          # 3.0 m, shared with the baseline
EMERGENCY_CLEARANCE = 0.5                          # m, any direction
TURN_AWAY = 0.8                                    # |turn| command when stopping, as in the baseline
LIDAR_MAX_AGE_S = 0.5                              # five scans at the 10 Hz LiDAR rate


class LidarSafetyFilter:
    def __init__(self, stop_clearance=STOP_CLEARANCE, slow_clearance=SLOW_CLEARANCE,
                 emergency_clearance=EMERGENCY_CLEARANCE, max_age_s=LIDAR_MAX_AGE_S):
        if not 0 < emergency_clearance < stop_clearance < slow_clearance:
            raise ValueError("need 0 < emergency < stop < slow clearance")
        self.stop_clearance = stop_clearance
        self.slow_clearance = slow_clearance
        self.emergency_clearance = emergency_clearance
        self.max_age_s = max_age_s

    def filter(self, obs, action, lidar_age_s):
        """Return (safe_action, info). info: intervened, reason, front_clear_m, min_clear_m, original action."""
        action = np.asarray(action, np.float32)
        speed, turn = float(np.clip(action[0], 0.0, 1.0)), float(np.clip(action[1], -1.0, 1.0))
        info = {"original": [float(action[0]), float(action[1])], "reason": None,
                "front_clear_m": None, "min_clear_m": None}

        obs = np.asarray(obs, np.float32)
        lidar = obs[so.SLICES["lidar"]] if obs.ndim == 1 and obs.shape[0] == so.DIM else None
        if lidar is None or lidar.shape[0] != so.LIDAR_SECTORS or not np.all(np.isfinite(lidar)) \
                or np.any(lidar < 0.0) or np.any(lidar > 1.0):
            return self._out(0.0, 0.0, "lidar_invalid", info)
        if lidar_age_s is None or not math.isfinite(lidar_age_s) or lidar_age_s > self.max_age_s:
            return self._out(0.0, 0.0, "lidar_stale", info)

        ranges = lidar * so.LIDAR_RANGE_MAX
        front = float(np.min(ranges[FRONT]))
        info["front_clear_m"], info["min_clear_m"] = front, float(np.min(ranges))
        away = -TURN_AWAY if np.mean(ranges[SECTOR_ANGLES > 0]) > np.mean(ranges[SECTOR_ANGLES < 0]) else TURN_AWAY
        if info["min_clear_m"] < self.emergency_clearance:
            return self._out(0.0, away, "emergency", info)
        if front < self.stop_clearance:
            return self._out(0.0, away, "stop", info)
        if front < self.slow_clearance:
            cap = 0.8 * (front - self.stop_clearance) / (self.slow_clearance - self.stop_clearance)
            if speed > cap:
                return self._out(cap, turn, "slow", info)
        return self._out(speed, turn, None, info)

    @staticmethod
    def _out(speed, turn, reason, info):
        info["reason"] = reason
        safe = np.array([speed, turn], np.float32)
        info["intervened"] = reason is not None or not np.allclose(safe, info["original"])
        return safe, info
