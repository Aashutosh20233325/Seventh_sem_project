"""Non-learning target-reaching baseline that uses only the sar_obs_v1 observation.

It uses the same observation and action interface as PPO: no privileged information and no simulator
access.

Sign conventions:
- Observation bearings are body FLU, +left.
- Action turn is NED yaw rate, +clockwise (right). To turn toward a bearing b, command turn proportional
  to -b.
- LiDAR sector i covers scan angles from -135 deg (i = 0, right) to +135 deg (i = 48, left).

Behaviour:
1. Target known (fresh range-valid detection, else unexpired estimate, else bearing-only detection):
   turn toward it; if roughly aligned, fly forward with speed decreasing with range.
2. Target unknown: search by rotating right (the start heading leaves the target about 45 deg right),
   then move forward briefly to change viewpoint and rotate again. It never follows expired data.
3. Obstacles: forward speed is capped by frontal LiDAR clearance; below the stop distance it turns
   toward the clearer side instead of advancing.
"""

from __future__ import annotations

import math

import numpy as np

import sar_observation as so

SECTOR_ANGLES = np.radians(-135.0 + (np.arange(so.LIDAR_SECTORS) + 0.5) * 270.0 / so.LIDAR_SECTORS)
FRONT = np.abs(SECTOR_ANGLES) <= math.radians(35.0)


class BaselineController:
    def __init__(self, turn_gain=2.0, align_angle=math.radians(25.0), stop_clearance=1.2,
                 slow_clearance=3.0, search_turn=0.6, search_rotation_steps=110, search_advance_steps=25):
        self.turn_gain = turn_gain
        self.align_angle = align_angle
        self.stop_clearance = stop_clearance
        self.slow_clearance = slow_clearance
        self.search_turn = search_turn
        self.search_rotation_steps = search_rotation_steps
        self.search_advance_steps = search_advance_steps
        self.reset()

    def reset(self):
        self.search_step = 0

    @staticmethod
    def _angle(obs, sin_name, cos_name):
        return math.atan2(float(obs[so.SLICES[sin_name]][0]), float(obs[so.SLICES[cos_name]][0]))

    def target(self, obs):
        """(bearing rad +left, range m or None, source) from the observation only, or None."""
        f = lambda name: float(obs[so.SLICES[name]][0])  # noqa: E731
        if f("det_now") and f("det_range_valid"):
            return self._angle(obs, "det_bearing_sin", "det_bearing_cos"), f("det_range") * so.TARGET_RANGE_SCALE, "det"
        if f("est_valid"):
            return self._angle(obs, "est_bearing_sin", "est_bearing_cos"), f("est_range") * so.TARGET_RANGE_SCALE, "est"
        if f("det_now"):
            return self._angle(obs, "det_bearing_sin", "det_bearing_cos"), None, "bearing_only"
        return None

    def act(self, obs):
        obs = np.asarray(obs, np.float32)
        lidar_m = obs[so.SLICES["lidar"]] * so.LIDAR_RANGE_MAX
        front_clear = float(np.min(lidar_m[FRONT]))
        target = self.target(obs)

        if target is None:
            speed, turn = self._search()
        else:
            self.search_step = 0
            bearing, rng, _ = target
            turn = float(np.clip(-self.turn_gain * bearing, -1.0, 1.0))
            if abs(bearing) > self.align_angle:
                speed = 0.05
            elif rng is None:
                speed = 0.3
            else:
                speed = float(np.clip(0.15 + 0.2 * (rng - 0.5), 0.15, 0.8))

        # Obstacle avoidance overrides forward motion.
        if front_clear < self.stop_clearance:
            left = float(np.mean(lidar_m[SECTOR_ANGLES > 0]))
            right = float(np.mean(lidar_m[SECTOR_ANGLES < 0]))
            speed = 0.0
            turn = -0.8 if left > right else 0.8           # turn toward the clearer side
        elif front_clear < self.slow_clearance:
            speed = min(speed, 0.8 * (front_clear - self.stop_clearance) / (self.slow_clearance - self.stop_clearance))
        return np.array([max(speed, 0.0), turn], np.float32)

    def _search(self):
        cycle = self.search_rotation_steps + self.search_advance_steps
        phase = self.search_step % cycle
        self.search_step += 1
        if phase < self.search_rotation_steps:
            return 0.0, self.search_turn                    # rotate right in place
        return 0.4, 0.0                                     # change viewpoint
