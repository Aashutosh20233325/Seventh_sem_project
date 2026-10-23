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

import dataclasses
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


# Randomized-target training (offline only). Targets are sampled in the open corner bounded by the north
# (x ~ 4.5-5.5) and east (y ~ 4.5-5.5) walls, never near a wall, the start, or a held-out position.
TARGET_REGION = (-6.0, 3.5, -6.0, 3.5)            # NED x_min, x_max, y_min, y_max
TARGET_WALL_CLEARANCE = 1.0                       # m from any wall box
TARGET_START_RANGE = (2.5, 6.0)                   # m from the nominal start (0, 0); starts vary by <= 1.4 m
# Held out of training: the world's nominal target plus five evaluation positions.
HELD_OUT_TARGETS = ((-3.0, 3.0), (-3.0, -2.5), (2.0, -3.0), (-1.0, 3.0), (-5.5, 0.5), (1.5, 2.5))
HELD_OUT_RADIUS = 0.75                            # m exclusion disc around each held-out position


def box_distance(x, y, box):
    xmin, xmax, ymin, ymax = box
    return math.hypot(max(xmin - x, 0.0, x - xmax), max(ymin - y, 0.0, y - ymax))


def valid_random_target(x, y, walls, held_out=HELD_OUT_TARGETS):
    x_min, x_max, y_min, y_max = TARGET_REGION
    return (x_min <= x <= x_max and y_min <= y <= y_max
            and TARGET_START_RANGE[0] <= math.hypot(x, y) <= TARGET_START_RANGE[1]
            and all(box_distance(x, y, w) >= TARGET_WALL_CLEARANCE for w in walls)
            and all(math.hypot(x - hx, y - hy) > HELD_OUT_RADIUS for hx, hy in held_out))


# Randomized interior obstacles (offline training/evaluation only; opt-in). Thin, tall, axis-aligned walls
# placed independently of the target (so the layout carries no cue about where the target is), subject to:
OBSTACLE_COUNT = (4, 7)                 # inclusive range per episode
OBSTACLE_LENGTH = (2.0, 4.5)            # m
OBSTACLE_THICKNESS = 0.3                # m
OBSTACLE_CENTRE_REGION = (-6.5, 3.5, -6.5, 3.5)   # NED x_min, x_max, y_min, y_max of obstacle centres
OBSTACLE_START_CLEARANCE = 0.8          # m from the whole start square [-1, 1] x [-1, 1]
OBSTACLE_TARGET_CLEARANCE = 1.0         # m from the target centre (same as fixed walls)
OBSTACLE_GAP = 1.2                      # m minimum gap to any other wall: keeps passages open, no enclosures
START_SQUARE = (-1.0, 1.0, -1.0, 1.0)


def box_box_distance(a, b):
    return math.hypot(max(a[0] - b[1], b[0] - a[1], 0.0), max(a[2] - b[3], b[2] - a[3], 0.0))


def segment_hits_box(x0, y0, x1, y1, box):
    """True if the segment (x0, y0)-(x1, y1) touches the closed box (xmin, xmax, ymin, ymax). Slab method."""
    t0, t1 = 0.0, 1.0
    for p0, d, lo, hi in ((x0, x1 - x0, box[0], box[1]), (y0, y1 - y0, box[2], box[3])):
        if abs(d) < 1e-12:
            if p0 < lo or p0 > hi:
                return False
            continue
        ta, tb = (lo - p0) / d, (hi - p0) / d
        if ta > tb:
            ta, tb = tb, ta
        t0, t1 = max(t0, ta), min(t1, tb)
        if t0 > t1:
            return False
    return True


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
                 perception_period_steps=2, perception_latency_steps=1, max_steps=None, seed=None,
                 task_config=None, randomize_target=False, camera_occlusion=False, randomize_obstacles=False):
        super().__init__()
        self.action_space = spaces.Box(np.array([0.0, -1.0], np.float32), np.array([1.0, 1.0], np.float32))
        self.observation_space = spaces.Box(so.LOW, so.HIGH, dtype=np.float32)
        # task_config lets offline training vary reward weights; evaluation uses the default task.
        self.task_config = task_config if task_config is not None else sar_task.TaskConfig()
        if max_steps is not None:
            self.task_config = dataclasses.replace(self.task_config, max_steps=max_steps)
        self.start_range = None      # curriculum: (r_min, r_max, max_heading_error); None = default start
        self.nominal_target = sar_task.load_target()
        self.target = self.nominal_target
        self.base_walls = load_walls()
        self.walls = list(self.base_walls)
        # Opt-in: new interior obstacles each reset (see OBSTACLE_*). When off, reset never touches self.walls.
        self.randomize_obstacles = randomize_obstacles
        self.obstacles = []
        self.sampled_layouts = []
        # Offline training only: a new collision-free target each reset (ground truth for reward, the camera
        # model, and evaluation; it never enters the observation). sampled_targets records every draw.
        self.randomize_target = randomize_target
        self.sampled_targets = []
        # Opt-in: walls block the camera's view of the target (all walls are modelled as taller than the camera,
        # true for the world's 15 m walls, so the test is 2D). Default off keeps earlier results reproducible.
        self.camera_occlusion = camera_occlusion
        self.occluded_frames = 0       # diagnostics only: in-FOV frames suppressed by occlusion this episode
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

    def _camera_xy(self):
        offset = float(tp.CAMERA_IN_BODY[0])
        return self.x + offset * math.cos(self.heading), self.y + offset * math.sin(self.heading)

    def _occluded(self):
        """True if every sightline from the camera to the target (centre and both horizontal edges, offset by
        the target radius perpendicular to the line of sight) crosses a wall. A partly visible target is
        detectable."""
        cx, cy = self._camera_xy()
        tx, ty = self.target.x_ned, self.target.y_ned
        d = math.hypot(tx - cx, ty - cy)
        if d < 1e-9:
            return False
        px, py = -(ty - cy) / d * self.target.radius, (tx - cx) / d * self.target.radius
        for ex, ey in ((tx, ty), (tx + px, ty + py), (tx - px, ty - py)):
            if not any(segment_hits_box(cx, cy, ex, ey, w) for w in self.walls):
                return False
        return True

    def _perceive(self):
        bearing, rng = self._true_relative()
        stamp = self.steps * DT
        visible = self._camera_sees(bearing, rng)
        if visible and self.camera_occlusion and self._occluded():
            visible = False
            self.occluded_frames += 1
        if not visible or self.np_random.random() > self.detection_prob:
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
        if self.randomize_target:
            self._sample_target()
        if self.randomize_obstacles:
            self._sample_obstacles()
        self.x, self.y, self.heading = 0.0, 0.0, math.pi / 2
        if self.start_range is not None:
            self._sample_curriculum_start()
        elif self.randomize_start:
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
        self.occluded_frames = 0
        self._schedule_perception()
        sectors = self._lidar()
        return self._observe(sectors), {"privileged": self._privileged()}

    def _sample_target(self):
        x_min, x_max, y_min, y_max = TARGET_REGION
        for _ in range(1000):
            x, y = self.np_random.uniform(x_min, x_max), self.np_random.uniform(y_min, y_max)
            if valid_random_target(x, y, self.walls):
                self.target = dataclasses.replace(self.nominal_target, x_ned=float(x), y_ned=float(y))
                self.sampled_targets.append((self.target.x_ned, self.target.y_ned))
                return
        raise RuntimeError("no valid random target found")

    def _valid_obstacle(self, box, placed):
        t = self.target
        return (box_box_distance(box, START_SQUARE) >= OBSTACLE_START_CLEARANCE
                and box_distance(t.x_ned, t.y_ned, box) >= OBSTACLE_TARGET_CLEARANCE
                and all(box_box_distance(box, w) >= OBSTACLE_GAP for w in self.base_walls + placed))

    def _sample_obstacles(self):
        """Independent of the target except for target clearance. Separated convex obstacles cannot enclose
        any region, so the target stays reachable."""
        n = int(self.np_random.integers(OBSTACLE_COUNT[0], OBSTACLE_COUNT[1] + 1))
        x0, x1, y0, y1 = OBSTACLE_CENTRE_REGION
        placed = []
        for _ in range(n):
            for _ in range(500):
                cx, cy = self.np_random.uniform(x0, x1), self.np_random.uniform(y0, y1)
                half_len = 0.5 * self.np_random.uniform(*OBSTACLE_LENGTH)
                half_t = 0.5 * OBSTACLE_THICKNESS
                if self.np_random.random() < 0.5:                      # long along NED x
                    box = (cx - half_len, cx + half_len, cy - half_t, cy + half_t)
                else:                                                 # long along NED y
                    box = (cx - half_t, cx + half_t, cy - half_len, cy + half_len)
                box = tuple(float(v) for v in box)
                if self._valid_obstacle(box, placed):
                    placed.append(box)
                    break
            else:
                raise RuntimeError("could not place an obstacle")
        self.obstacles = placed
        self.walls = list(self.base_walls) + placed
        self.sampled_layouts.append({"target": (self.target.x_ned, self.target.y_ned), "obstacles": placed})

    # ---------- start-state curriculum (training only) ----------
    START_CLEARANCE = 0.6        # m of LiDAR clearance required at a curriculum start
    START_TRIES = 200

    def set_start_range(self, r_min=None, r_max=None, max_heading_error=None):
        """Sample later resets at horizontal range [r_min, r_max] m from the target, heading within
        +-max_heading_error rad of the target bearing. All None restores the default start distribution.
        Only the start state changes; the observation, reward, and termination are untouched."""
        if r_min is None:
            self.start_range = None
            return
        if not self.task_config.success_radius < r_min <= r_max:
            raise ValueError(f"need success_radius < r_min <= r_max, got {r_min}, {r_max}")
        self.start_range = (float(r_min), float(r_max), float(max_heading_error))

    def _sample_curriculum_start(self):
        r_min, r_max, max_err = self.start_range
        for _ in range(self.START_TRIES):
            r = self.np_random.uniform(r_min, r_max)
            phi = self.np_random.uniform(-math.pi, math.pi)
            x, y = self.target.x_ned + r * math.cos(phi), self.target.y_ned + r * math.sin(phi)
            if any(xmin <= x <= xmax and ymin <= y <= ymax for xmin, xmax, ymin, ymax in self.walls):
                continue
            self.x, self.y = x, y
            to_target = math.atan2(self.target.y_ned - y, self.target.x_ned - x)
            self.heading = so.wrap(to_target + self.np_random.uniform(-max_err, max_err))
            if float(np.min(self._lidar())) >= self.START_CLEARANCE:
                return
        raise RuntimeError(f"no collision-free curriculum start found for {self.start_range}")

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
