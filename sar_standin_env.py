"""Offline kinematic stand-in for PX4UAVEnv (no ROS, PX4, or Gazebo).

It implements the same Gymnasium interface, the same ``sar_obs_v1`` observation (the real
sar_observation.ObservationBuilder), and the same reward/termination rules (sar_task) as the SITL
environment. That lets controllers and PPO be developed and tested without flights.

Simulated, from the versioned world sim/worlds/sar_walls.sdf:
- Walls: axis-aligned boxes, all taller than the flight altitude.
- Target: sphere on the floor, below the horizontal LiDAR plane at hover, so the LiDAR never sees it.
- Vehicle: altitude-held, first-order-lag horizontal velocity and yaw rate.
- LiDAR: 1080 rays over +-135 deg (CCW positive), 0.1-30 m, binned like observation_publisher.
- Camera perception: a geometric model of the 30-deg-down OAK-D Lite (RGB FOV for detection, depth range
  limit, measured-scale noise, dropout, fixed rate and latency). Its output is a target_perception-style
  dict.

This is an idealised model, not a substitute for SITL validation.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import gymnasium as gym
import numpy as np
from gymnasium import spaces

import sar_observation as so
import sar_task
import target_perception as tp

DT = 0.1                      # s per step (PX4UAVEnv.STEP_DURATION)
SUBSTEPS = 5
VEL_TAU = 0.45                # s, measured-scale lag of PX4 velocity response
YAW_TAU = 0.2
MAX_SPEED = 1.0               # m/s for action[0] = 1
MAX_YAW_RATE = 0.5            # rad/s for action[1] = 1, positive = clockwise (NED)
HOLD_Z = -1.8
ROOT_HEIGHT = 1.615           # m above floor at hover (measured)
LIDAR_N, LIDAR_FOV, LIDAR_MIN, LIDAR_MAX = 1080, math.radians(270.0), 0.1, 30.0

# Camera model (target_perception mount; IMX214 RGB FOV for detection).
CAM_HEIGHT = ROOT_HEIGHT + tp.CAMERA_IN_BODY[2]
CAM_HALF_HFOV = 0.5 * 1.204
CAM_HALF_VFOV = math.atan(540 / 1397.2235870361328)
CAM_PITCH = tp.MOUNT_PITCH


def load_walls(world_sdf=sar_task.WORLD_SDF):
    """Wall boxes as NED rectangles (xmin, xmax, ymin, ymax)."""
    walls = []
    world = ET.parse(world_sdf).getroot().find("world")
    for model in world.findall("model"):
        box = model.find(".//collision/geometry/box/size")
        if box is None:
            continue
        x, y = (float(v) for v in model.findtext("pose").split()[:2])
        sx, sy = (float(v) for v in box.text.split()[:2])
        # ENU (x east, y north) -> NED (x north = y_enu, y east = x_enu)
        walls.append((y - sy / 2, y + sy / 2, x - sx / 2, x + sx / 2))
    return walls


def ray_box_distances(px, py, angles, box):
    """Distance along NED-angle rays from (px, py) to an axis-aligned box (inf for no hit)."""
    dx, dy = np.cos(angles), np.sin(angles)
    xmin, xmax, ymin, ymax = box
    with np.errstate(divide="ignore", invalid="ignore"):
        tx1, tx2 = (xmin - px) / dx, (xmax - px) / dx
        ty1, ty2 = (ymin - py) / dy, (ymax - py) / dy
    tmin = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
    tmax = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
    inside = xmin <= px <= xmax and ymin <= py <= ymax
    if inside:
        return np.zeros_like(angles)
    hit = (tmax >= tmin) & (tmax > 0) & np.isfinite(tmin)
    return np.where(hit, np.where(tmin > 0, tmin, 0.0), np.inf)


def bin_sectors(ranges, angle_min, angle_max, angle_increment, range_min, range_max, sectors=49):
    """Same binning as observation_publisher.ObservationPublisher.get_49_lidar."""
    width = (angle_max - angle_min) / sectors
    out = []
    for s in range(sectors):
        start = angle_min + s * width
        i0 = max(0, int((start - angle_min) / angle_increment))
        i1 = min(len(ranges), int((start + width - angle_min) / angle_increment) + 1)
        vals = [max(range_min, min(r if math.isfinite(r) else range_max, range_max)) for r in ranges[i0:i1]]
        out.append(min(vals) if vals else range_max)
    return out


class SarStandInEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, randomize_start=False, perception_noise=True, detection_prob=0.95,
                 perception_period_steps=2, perception_latency_steps=1, max_steps=None, seed=None):
        super().__init__()
        self.action_space = spaces.Box(np.array([0.0, -1.0], np.float32), np.array([1.0, 1.0], np.float32))
        self.observation_space = spaces.Box(so.LOW, so.HIGH, dtype=np.float32)
        self.task_config = sar_task.TaskConfig()
        if max_steps is not None:
            self.task_config = sar_task.TaskConfig(max_steps=max_steps)
        self.target = sar_task.load_target()
        self.walls = load_walls()
        self.randomize_start = randomize_start
        self.perception_noise = perception_noise
        self.detection_prob = detection_prob
        self.perception_period = perception_period_steps
        self.perception_latency = perception_latency_steps
        self.builder = so.ObservationBuilder()
        self.lidar_angles = np.linspace(-LIDAR_FOV / 2, LIDAR_FOV / 2, LIDAR_N)
        self._initial_seed = seed

    # ---------- simulation ----------
    def _lidar(self):
        # LaserScan angle is CCW-positive (left); NED angle = heading - scan angle.
        world = self.heading - self.lidar_angles
        d = np.full(LIDAR_N, np.inf)
        for box in self.walls:
            d = np.minimum(d, ray_box_distances(self.x, self.y, world, box))
        ranges = np.where(np.isfinite(d), np.clip(d, LIDAR_MIN, LIDAR_MAX), np.inf)
        inc = LIDAR_FOV / (LIDAR_N - 1)
        return np.array(bin_sectors(list(ranges), -LIDAR_FOV / 2, LIDAR_FOV / 2, inc, LIDAR_MIN, LIDAR_MAX))

    def _true_relative(self):
        dx, dy = self.target.x_ned - self.x, self.target.y_ned - self.y
        bearing = so.wrap(self.heading - math.atan2(dy, dx))       # body FLU, +left
        return bearing, math.hypot(dx, dy)

    def _camera_sees(self, bearing, rng):
        """Geometric visibility of any part of the sphere in the RGB frame (flat floor, no occlusion)."""
        horiz = max(rng - tp.CAMERA_IN_BODY[0] * math.cos(bearing), 0.05)
        height = CAM_HEIGHT - self.target.z_enu
        slant = math.hypot(horiz, height)
        ang_r = math.asin(min(1.0, self.target.radius / slant))     # angular radius seen from the camera
        if abs(bearing) > CAM_HALF_HFOV + ang_r:
            return False
        depression = math.atan2(height, horiz)
        return abs(depression - CAM_PITCH) <= CAM_HALF_VFOV + ang_r

    def _perceive(self):
        bearing, rng = self._true_relative()
        stamp = self.steps * DT
        if not self._camera_sees(bearing, rng) or self.np_random.random() > self.detection_prob:
            return {"detected": False, "range_valid": False, "bearing": math.nan,
                    "horizontal_range": math.nan, "image_stamp": stamp}
        if self.perception_noise:
            bearing += self.np_random.normal(0.0, math.radians(0.3))
            rng *= 1.0 + self.np_random.normal(0.0, 0.01)
        return {"detected": True, "range_valid": rng <= tp.DEPTH_MAX, "bearing": bearing,
                "horizontal_range": rng if rng <= tp.DEPTH_MAX else math.nan, "image_stamp": stamp}

    def _collided(self, sectors):
        return float(np.min(sectors)) < 0.25

    # ---------- gym API ----------
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed if seed is not None else self._initial_seed)
        self._initial_seed = None
        self.x, self.y, self.heading = 0.0, 0.0, math.pi / 2
        if self.randomize_start:
            self.x += self.np_random.uniform(-1.0, 1.0)
            self.y += self.np_random.uniform(-1.0, 1.0)
            self.heading = so.wrap(self.heading + self.np_random.uniform(-math.pi, math.pi))
        self.vx = self.vy = 0.0
        self.yaw_rate_ned = 0.0
        self.steps = 0
        self.previous_action = np.zeros(2, np.float32)
        self.pending = []                     # (deliver_step, perception) for latency
        self.latest = None
        self.latest_step = None
        self.builder.reset()
        self._schedule_perception()
        sectors = self._lidar()
        return self._observe(sectors), {"privileged": self._privileged()}

    def _schedule_perception(self):
        if self.steps % self.perception_period == 0:
            self.pending.append((self.steps + self.perception_latency, self._perceive()))
        while self.pending and self.pending[0][0] <= self.steps:
            _, self.latest = self.pending.pop(0)
            self.latest_step = self.steps

    def _observe(self, sectors):
        ego = so.EgoState(self.x, self.y, HOLD_Z, self.vx, self.vy, self.heading, -self.yaw_rate_ned)
        age = None if self.latest is None else (self.steps - self.latest_step) * DT
        return self.builder.build(sectors, self.previous_action, ego, self.latest, age, self.steps * DT)

    def _privileged(self):
        return {"distance_to_target": sar_task.horizontal_distance(self.x, self.y, self.target),
                "target_ned": (self.target.x_ned, self.target.y_ned), "x": self.x, "y": self.y, "z": HOLD_Z}

    def step(self, action):
        action = np.clip(np.asarray(action, np.float32), self.action_space.low, self.action_space.high)
        old_distance = sar_task.horizontal_distance(self.x, self.y, self.target)
        h = DT / SUBSTEPS
        for _ in range(SUBSTEPS):
            target_rate = float(action[1]) * MAX_YAW_RATE
            self.yaw_rate_ned += (target_rate - self.yaw_rate_ned) * h / YAW_TAU
            self.heading = so.wrap(self.heading + self.yaw_rate_ned * h)
            speed = float(action[0]) * MAX_SPEED
            tvx, tvy = speed * math.cos(self.heading), speed * math.sin(self.heading)
            self.vx += (tvx - self.vx) * h / VEL_TAU
            self.vy += (tvy - self.vy) * h / VEL_TAU
            self.x += self.vx * h
            self.y += self.vy * h
        self.steps += 1
        self._schedule_perception()
        sectors = self._lidar()
        distance = sar_task.horizontal_distance(self.x, self.y, self.target)
        collision = self._collided(sectors)
        outcome = sar_task.step_outcome(old_distance, distance, collision, self.steps, self.task_config)
        self.previous_action = action.copy()
        obs = self._observe(sectors)
        info = {"reason": outcome.reason, "collision": collision,
                "reached_target": outcome.reason == "success",
                "target_detected": bool(obs[so.SLICES["det_now"]][0]),
                "target_estimate_valid": bool(obs[so.SLICES["est_valid"]][0]),
                "step": self.steps, "privileged": self._privileged()}
        return obs, float(outcome.reward), outcome.terminated, outcome.truncated, info
