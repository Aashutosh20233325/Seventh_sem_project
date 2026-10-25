"""Frontier explorer (design A, 3D extension step 3/4): explore until the camera sees the target, then hand over.

Every step the controller:
  1. updates a 2D grid map at flight height (RES m cells over +-HALF m, PX4 local NED):
       - obstacle layer from the 49 LiDAR sectors (each sector's minimum range: the whole wedge up to it is free,
         its centre ray ends in an occupied cell when something was hit); occupied cells are never cleared;
       - searched layer from the camera's field of view (+-CAM_HALF_FOV, CAM_MIN..CAM_MAX m, stopped by obstacles):
         where the camera has looked, so where the target cannot be;
  2. chooses a mode: APPROACH while the camera detects the target or its estimate is valid, otherwise EXPLORE. In
     APPROACH the target's map position comes from the observation's estimate (bearing and range) and the pose; if
     the straight line to it is clear on the map the inner controller (e.g. PPO through runner.PolicyController) flies,
     else the explorer plans around the obstacles to the target and flies that path (like an EXPLORE goal);
  3. in EXPLORE, every REPLAN_STEPS (or when the goal is reached or blocked) ranks candidate viewpoints by
         score = gain * exp(-LAMBDA * cost)
     gain  = predicted number of unsearched, not-occupied cells the camera would cover from the viewpoint at its best
             of 8 headings (unknown cells are see-through: optimistic), cost = Dijkstra path length (m) on the map with
             obstacles inflated by INFLATE m (unknown cells cost UNKNOWN_COST x), and keeps the current goal unless a
             new one scores HYSTERESIS x better;
  4. follows the path with pure pursuit (look-ahead LOOKAHEAD m) and, at the goal, turns to the best viewing heading;
     the action is the env's [speed 0..1, turn -1..1] and passes through the same LiDAR safety filter as the policy.
Inputs are only the observation, the vehicle's own pose estimate (pose() -> x, y, heading NED) and the LiDAR age; the
target position is never used. `last` holds the decision for the step log.
"""

from __future__ import annotations

import math

import numpy as np
from scipy import ndimage
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra as sparse_dijkstra

import sar_observation as so
from baseline_controller import SECTOR_ANGLES
from safety_layer import LidarSafetyFilter

RES = 0.4                    # m per cell
HALF = 22.0                  # m: map covers [-HALF, HALF] in x and y
N = int(round(2 * HALF / RES))
UNKNOWN, FREE, OCC = 0, 1, 2
HIT_MARGIN = 29.0            # m: a sector reading below this hit something (max range 30 m)
CAM_HALF_FOV = 0.6           # rad (stand-in model: 0.5 * 1.204)
CAM_MIN, CAM_MAX = 1.0, 10.0  # m of ground in view from 1.8 m with the 30 deg down-pitched camera
INFLATE = 1.0                # m clearance from obstacles for planning
UNKNOWN_COST = 1.5
LAMBDA = 0.15                # 1/m: distance discount of the gain
HYSTERESIS = 1.2
REPLAN_STEPS = 10
LOOKAHEAD = 1.0              # m
GOAL_TOLERANCE = 0.8         # m
HEADING_TOLERANCE = math.radians(15)
LOST_STEPS = 20              # steps without detection and without a valid estimate before exploring again
LINE_CLEARANCE = 0.4         # m: the straight line to the target must keep this from occupied cells
STUCK_STEPS = 60             # steps without 0.3 m progress toward the goal: give it up
HEADINGS = [k * math.pi / 4 for k in range(8)]
NEIGHBOURS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
              (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class ExplorerMap:
    def __init__(self):
        self.state = np.full((N, N), UNKNOWN, np.uint8)
        self.searched = np.zeros((N, N), bool)

    @staticmethod
    def cell(x, y):
        return int((x + HALF) / RES), int((y + HALF) / RES)

    @staticmethod
    def centre(i, j):
        return -HALF + (i + 0.5) * RES, -HALF + (j + 0.5) * RES

    @staticmethod
    def inside(i, j):
        return 0 <= i < N and 0 <= j < N

    @staticmethod
    def _cells(x, y, angles, dists):
        """Cell indices (rays x samples) along rays from (x, y); -1 outside the map."""
        xs = x + np.cos(angles)[:, None] * dists[None, :]
        ys = y + np.sin(angles)[:, None] * dists[None, :]
        i = np.floor((xs + HALF) / RES).astype(int)
        j = np.floor((ys + HALF) / RES).astype(int)
        inside = (i >= 0) & (i < N) & (j >= 0) & (j < N)
        return np.where(inside, i, -1), np.where(inside, j, -1), inside

    _LIDAR_D = np.arange(0.0, HIT_MARGIN, RES / 2)
    _CAM_D = np.arange(CAM_MIN, CAM_MAX, RES / 2)
    _CAM_A = np.linspace(-CAM_HALF_FOV, CAM_HALF_FOV, 13)

    def update_lidar(self, x, y, heading, obs):
        """Each sector's wedge is free up to its minimum range; a hit marks the centre ray's end occupied."""
        ranges = np.asarray(obs[so.SLICES["lidar"]], float) * so.LIDAR_RANGE_MAX
        half_width = math.radians(270.0 / so.LIDAR_SECTORS) / 2
        centre = heading - SECTOR_ANGLES                  # NED angle = heading - scan angle (scan is +left)
        angles = np.concatenate([centre - 0.8 * half_width, centre, centre + 0.8 * half_width])
        limit = np.tile(np.maximum(np.minimum(ranges, HIT_MARGIN) - RES, 0.0), 3)
        i, j, inside = self._cells(x, y, angles, self._LIDAR_D)
        free = inside & (self._LIDAR_D[None, :] < limit[:, None])
        fi, fj = i[free], j[free]
        keep = self.state[fi, fj] != OCC
        self.state[fi[keep], fj[keep]] = FREE
        hit = ranges < HIT_MARGIN
        hi = np.floor((x + ranges[hit] * np.cos(centre[hit]) + HALF) / RES).astype(int)
        hj = np.floor((y + ranges[hit] * np.sin(centre[hit]) + HALF) / RES).astype(int)
        ok = (hi >= 0) & (hi < N) & (hj >= 0) & (hj < N)
        self.state[hi[ok], hj[ok]] = OCC

    def view(self, x, y, heading, mark=False):
        """Cells the camera covers from (x, y, heading): each ray stops at the first occupied (or off-map) cell,
        unknown is see-through. mark=True records them as searched; returns the number of unsearched cells covered."""
        i, j, inside = self._cells(x, y, heading + self._CAM_A, self._CAM_D)
        stop = ~inside | (self.state[i, j] == OCC)
        visible = np.cumsum(stop, axis=1) == 0
        flat = np.unique(i[visible] * N + j[visible])
        new = flat[~self.searched.ravel()[flat]]
        if mark:
            self.searched.ravel()[new] = True
        return int(new.size)

    def blocked(self):
        occ = self.state == OCC
        radius = int(math.ceil(INFLATE / RES))
        yy, xx = np.mgrid[-radius:radius + 1, -radius:radius + 1]
        return ndimage.binary_dilation(occ, structure=(xx * xx + yy * yy) <= radius * radius)


def dijkstra(start, blocked, state):
    """Path cost (m) and predecessor (flat index, -9999 none) from start over unblocked cells (8-connected; entering
    an unknown cell costs UNKNOWN_COST x). Compiled SciPy Dijkstra on a sparse grid graph."""
    idx = np.arange(N * N).reshape(N, N)
    cost = np.where(state == UNKNOWN, UNKNOWN_COST, 1.0) * RES
    rows, cols, weights = [], [], []
    for di, dj, w in NEIGHBOURS:
        a0, a1 = max(0, -di), N - max(0, di)
        b0, b1 = max(0, -dj), N - max(0, dj)
        src = idx[a0:a1, b0:b1]
        dst = idx[a0 + di:a1 + di, b0 + dj:b1 + dj]
        ok = ~blocked[a0:a1, b0:b1] & ~blocked[a0 + di:a1 + di, b0 + dj:b1 + dj]
        rows.append(src[ok])
        cols.append(dst[ok])
        weights.append(w * cost[a0 + di:a1 + di, b0 + dj:b1 + dj][ok])
    graph = csr_matrix((np.concatenate(weights), (np.concatenate(rows), np.concatenate(cols))), shape=(N * N, N * N))
    dist, pred = sparse_dijkstra(graph, directed=True, indices=start[0] * N + start[1], return_predecessors=True)
    return dist.reshape(N, N), pred


def path_to(pred, start, goal):
    path, c, s = [], goal[0] * N + goal[1], start[0] * N + start[1]
    while c != s:
        if c < 0:
            return None
        path.append(divmod(int(c), N))
        c = pred[c]
    return list(reversed(path))


class ExplorerController:
    """[speed, turn] from the frontier explorer, or from `inner` (e.g. PPO) once the target is seen."""

    def __init__(self, inner, pose, lidar_age=lambda: 0.0, safety_filter=None, lam=LAMBDA):
        self.inner = inner
        self.pose = pose
        self.lidar_age = lidar_age
        self.safety_filter = safety_filter if safety_filter is not None else LidarSafetyFilter()
        self.lam = lam
        self.reset()

    def reset(self):
        self.map = ExplorerMap()
        self.target = None
        self.approach_plan = 0
        self.mode = "EXPLORE"
        self.goal = None              # (i, j, heading, score)
        self.path = []
        self.steps = 0
        self.since_plan = REPLAN_STEPS
        self.lost = 0
        self.failed = set()
        self.progress = (math.inf, 0)
        self.last = None

    # ---------- planning ----------

    def _candidates(self, dist, blocked):
        """Reachable free cells next to unsearched cells, thinned to one per 2 m block (the cheapest)."""
        free = (self.map.state == FREE) & ~blocked & np.isfinite(dist)
        near_unsearched = ndimage.binary_dilation(~self.map.searched & (self.map.state != OCC))
        frontier = free & near_unsearched
        best = {}
        for i, j in zip(*np.nonzero(frontier)):
            key = (i * RES // 2, j * RES // 2)
            if key not in best or dist[i, j] < dist[best[key]]:
                best[key] = (i, j)
        return [c for c in best.values() if c not in self.failed]

    def _plan(self, x, y):
        start = self.map.cell(x, y)
        blocked = self.map.blocked()
        blocked[start] = False                         # the vehicle's own cell is always a valid start
        dist, parent = dijkstra(start, blocked, self.map.state)
        scored = []
        for c in self._candidates(dist, blocked) + [start]:
            cx, cy = self.map.centre(*c)
            gains = [(self.map.view(cx, cy, h), h) for h in HEADINGS]
            gain, heading = max(gains)
            if gain > 0:
                scored.append((gain * math.exp(-self.lam * dist[c]), c, heading, gain))
        if not scored:
            return None, 0
        score, c, heading, gain = max(scored)
        if self.goal is not None and (self.goal[0], self.goal[1]) not in self.failed:
            g = (self.goal[0], self.goal[1])
            if np.isfinite(dist[g]):
                gx, gy = self.map.centre(*g)
                current = self.map.view(gx, gy, self.goal[2]) * math.exp(-self.lam * dist[g])
                if current > 0 and score < HYSTERESIS * current:
                    c, heading, score = g, self.goal[2], current
        self.path = [] if c == start else (path_to(parent, start, c) or [])
        if c != start and not self.path:
            self.failed.add(c)
            return None, len(scored)
        self.goal = (c[0], c[1], heading, score)
        self.progress = (math.inf, self.steps)
        return self.goal, len(scored)

    # ---------- following ----------

    def _follow(self, x, y, heading):
        gx, gy = self.map.centre(self.goal[0], self.goal[1])
        if math.hypot(gx - x, gy - y) < GOAL_TOLERANCE or not self.path:
            err = wrap(self.goal[2] - heading)              # at the viewpoint: turn to the best viewing heading
            if abs(err) < HEADING_TOLERANCE:
                self.since_plan = REPLAN_STEPS               # done here: plan the next goal
                return 0.0, 0.0
            return 0.0, float(np.clip(err / (math.pi / 4), -1, 1)) * 0.8
        while len(self.path) > 1:                            # drop passed waypoints
            px, py = self.map.centre(*self.path[0])
            if math.hypot(px - x, py - y) < LOOKAHEAD:
                self.path.pop(0)
            else:
                break
        px, py = self.map.centre(*self.path[0])
        err = wrap(math.atan2(py - y, px - x) - heading)
        turn = float(np.clip(2.0 * err, -1, 1))
        speed = 0.0 if abs(err) > math.radians(60) else 0.7 * math.cos(err)
        return speed, turn

    def _target_position(self, obs, x, y, heading):
        """Target (x, y) NED from the observation's estimate or detection (bearing +left), else None."""
        f = lambda name: float(obs[so.SLICES[name]][0])  # noqa: E731
        if f("est_valid") > 0:
            b, r = math.atan2(f("est_bearing_sin"), f("est_bearing_cos")), f("est_range") * so.TARGET_RANGE_SCALE
        elif f("det_now") > 0 and f("det_range_valid") > 0:
            b, r = math.atan2(f("det_bearing_sin"), f("det_bearing_cos")), f("det_range") * so.TARGET_RANGE_SCALE
        else:
            return None
        return x + r * math.cos(heading - b), y + r * math.sin(heading - b)

    def _line_clear(self, x, y, tx, ty):
        d = math.hypot(tx - x, ty - y)
        if d < 1e-6:
            return True
        radius = int(math.ceil(LINE_CLEARANCE / RES))
        occ = ndimage.binary_dilation(self.map.state == OCC, iterations=radius) if radius else self.map.state == OCC
        for s in np.arange(0.0, max(d - 0.6, 0.0), RES / 2):         # stop short of the sphere itself
            i, j = self.map.cell(x + (tx - x) * s / d, y + (ty - y) * s / d)
            if self.map.inside(i, j) and occ[i, j]:
                return False
        return True

    def _plan_to(self, x, y, tx, ty):
        """Path to the reachable cell nearest the target (the explorer's goal while the direct line is blocked)."""
        start = self.map.cell(x, y)
        blocked = self.map.blocked()
        blocked[start] = False
        dist, pred = dijkstra(start, blocked, self.map.state)
        ii, jj = np.nonzero(np.isfinite(dist))
        if ii.size == 0:
            return False
        cx, cy = self.map.centre(ii, jj)
        k = int(np.argmin(np.hypot(cx - tx, cy - ty) + 0.05 * dist[ii, jj]))
        goal = (int(ii[k]), int(jj[k]))
        self.path = [] if goal == start else (path_to(pred, start, goal) or [])
        self.goal = (goal[0], goal[1], math.atan2(ty - cy[k], tx - cx[k]), 0.0)
        return True

    def act(self, obs):
        x, y, heading = self.pose()
        self.steps += 1
        self.map.update_lidar(x, y, heading, obs)
        self.map.view(x, y, heading, mark=True)
        f = lambda name: float(obs[so.SLICES[name]][0])  # noqa: E731
        detected, estimate = f("det_now") > 0, f("est_valid") > 0
        if detected or (self.mode == "APPROACH" and estimate):
            self.mode, self.lost = "APPROACH", 0
        elif self.mode == "APPROACH":
            self.lost += 1
            if self.lost >= LOST_STEPS:
                self.mode, self.goal, self.path, self.since_plan = "EXPLORE", None, [], REPLAN_STEPS
        if self.mode == "APPROACH":
            target = self._target_position(obs, x, y, heading)
            if target is not None:
                self.target = target
            tx, ty = self.target if self.target is not None else (None, None)
            if tx is None or self._line_clear(x, y, tx, ty):
                action = np.asarray(self.inner.act(obs), np.float32)
                self.last = {"mode": "APPROACH", "via": "inner", "target_est": None if tx is None else
                             [round(tx, 2), round(ty, 2)], "inner": getattr(self.inner, "last", None)}
                self.approach_plan = 0
                return action
            if getattr(self, "approach_plan", 0) <= 0 or not self.path:   # line blocked: go around (replan each 10)
                self._plan_to(x, y, tx, ty)
                self.approach_plan = REPLAN_STEPS
            self.approach_plan -= 1
            raw = np.array(self._follow(x, y, heading), np.float32) if self.goal is not None else np.zeros(2, np.float32)
            action, finfo = self.safety_filter.filter(obs, raw, self.lidar_age())
            self.last = {"mode": "APPROACH", "via": "path", "target_est": [round(tx, 2), round(ty, 2)],
                         "filter": finfo["reason"], "original": finfo["original"]}
            return action
        n_candidates = None
        if self.goal is not None:                            # give up a goal without progress, or a blocked path
            gx, gy = self.map.centre(self.goal[0], self.goal[1])
            d = math.hypot(gx - x, gy - y)
            if d < self.progress[0] - 0.3:
                self.progress = (d, self.steps)
            elif self.steps - self.progress[1] > STUCK_STEPS:
                self.failed.add((self.goal[0], self.goal[1]))
                self.goal, self.since_plan = None, REPLAN_STEPS
            elif self.path and self.map.blocked()[self.path[0]]:
                self.since_plan = REPLAN_STEPS
        if self.goal is None or self.since_plan >= REPLAN_STEPS:
            _, n_candidates = self._plan(x, y)
            self.since_plan = 0
        self.since_plan += 1
        if self.goal is None:
            raw = np.array([0.0, 0.6], np.float32)             # nothing left to plan: keep looking around
            reason = "no_goal"
        else:
            raw = np.array(self._follow(x, y, heading), np.float32)
            reason = "follow"
        action, finfo = self.safety_filter.filter(obs, raw, self.lidar_age())
        gx, gy = (None, None) if self.goal is None else self.map.centre(self.goal[0], self.goal[1])
        self.last = {"mode": "EXPLORE", "reason": reason, "goal": None if gx is None else [round(gx, 2), round(gy, 2)],
                     "goal_heading": None if self.goal is None else round(self.goal[2], 2),
                     "score": None if self.goal is None else round(self.goal[3], 1), "candidates": n_candidates,
                     "searched_cells": int(self.map.searched.sum()), "filter": finfo["reason"],
                     "original": finfo["original"]}
        return action
