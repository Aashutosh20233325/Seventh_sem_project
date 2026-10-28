#!/usr/bin/env python3
"""Procedural rough-terrain search worlds (step 1 of the 3D extension): one seed + difficulty -> one world.

Each world is written as a Stage C world variant, sim/worlds/variants/<name>/sar_walls.sdf (world name unchanged, so
every topic, gate and the SAR_WORLD_VARIANT selector work as before), plus terrain.obj (the terrain mesh) and
spec.json (the full description). The stand-in reads the same file (sar_standin_env.load_walls), so Gazebo and
training see one geometry.

Contents (40 x 40 m, PX4 local NED, start at the origin):
  terrain   smooth hills and craters as a triangle mesh (exact coordinates; no image-orientation convention), flattened
            to height 0 on two 3 x 3 m pads (start, target) with a smooth 2 m blend
  ruins     building shells: four wall segments each, with door/collapse gaps, partly lowered walls
  wrecks    vehicle-sized boxes; rubble: low blocks; trees: thin trunks (canopy visual only, above flight height)
  target    the red sphere (radius 0.5 m) on the target pad
The drone still holds a fixed 1.8 m above the take-off point (terrain following is step 2), so every obstacle is
either LOW (top <= 1.1 m: flown over) or TALL (top >= 2.6 m: must be flown around, visible to the 2D LiDAR), and the
terrain stays within +-0.6 m. Collision geometry is boxes and planes only. The terrain mesh is VISUAL ONLY: cameras
and the GPU LiDAR see it (they use rendering), but a mesh collision shape crashes this world's physics (DART with the
ODE collision detector segfaults in OdeMesh::fillArrays; 2026-10-25). Ground contact comes from a thin collision box
under each pad (top at height 0) and a safety plane 0.3 m below the lowest terrain. Real terrain collision
(heightmap or another collision engine) belongs to step 2.

Validity (else the seed is resampled internally and the attempt count recorded): start-target distance 10-18 m;
pads clear by >= 1.5 m of every obstacle; a grid path from start to target keeping >= 1.5 m from every tall obstacle;
difficulty 3 additionally requires a tall obstacle across the straight start-target line. hidden=True (names gen_h*)
additionally requires the target to be hidden from the start: all three sightlines (to the sphere centre and its two
edges, as in the stand-in's camera-occlusion model) cross a tall obstacle, so it must be found by exploring.

Usage: .venv/bin/python world_generator.py <name> <seed> <difficulty 1-3> [hidden]
"""

from __future__ import annotations

import dataclasses
import heapq
import json
import math
import sys
from pathlib import Path

import numpy as np

import sar_task

AREA = 20.0                       # half-size (m): the world spans [-20, 20] in x and y (NED)
PAD = 1.5                         # pad half-size (3 x 3 m)
BLEND = 2.0                       # m of smooth blend from a flat pad into the terrain
TERRAIN_STEP = 0.5                # m between mesh vertices
LOW_TOP, TALL_TOP = 1.1, 2.6      # m above the take-off point: fly-over vs fly-around obstacles
CLEARANCE = 1.5                   # m from tall obstacles: path and pad clearance
TARGET_RADIUS, TARGET_Z = 0.5, 0.055
LEVELS = {  # difficulty: counts and terrain amplitude
    1: dict(buildings=2, wrecks=2, rubble=6, trees=4, amplitude=0.3, block_line=False),
    2: dict(buildings=4, wrecks=4, rubble=12, trees=8, amplitude=0.45, block_line=False),
    3: dict(buildings=6, wrecks=6, rubble=20, trees=12, amplitude=0.6, block_line=True),
}


@dataclasses.dataclass
class Box:
    """Axis-aligned box in PX4 local NED (x north, y east); z = bottom height (up), h = height."""
    kind: str
    x: float
    y: float
    sx: float
    sy: float
    z: float
    h: float

    @property
    def top(self):
        return self.z + self.h

    def rect(self):
        return (self.x - self.sx / 2, self.x + self.sx / 2, self.y - self.sy / 2, self.y + self.sy / 2)


def rect_distance(px, py, rect):
    xmin, xmax, ymin, ymax = rect
    return math.hypot(max(xmin - px, 0, px - xmax), max(ymin - py, 0, py - ymax))


def rect_rect_distance(a, b):
    dx = max(b[0] - a[1], a[0] - b[1], 0)
    dy = max(b[2] - a[3], a[2] - b[3], 0)
    return math.hypot(dx, dy)


def pad_rect(x, y):
    return (x - PAD, x + PAD, y - PAD, y + PAD)


# ---------- terrain ----------

def terrain_heights(rng, amplitude, pads):
    """Grid (n x n) of heights (m, up) at x, y = -AREA..AREA step TERRAIN_STEP; flat (0) on the pads."""
    xs = np.arange(-AREA, AREA + 1e-9, TERRAIN_STEP)
    gx, gy = np.meshgrid(xs, xs, indexing="ij")              # gx: north, gy: east
    h = np.zeros_like(gx)
    for _ in range(rng.integers(5, 9)):                     # hills and hollows
        cx, cy = rng.uniform(-AREA, AREA, 2)
        s = rng.uniform(3.0, 8.0)
        h += rng.uniform(-1, 1) * np.exp(-((gx - cx) ** 2 + (gy - cy) ** 2) / (2 * s * s))
    for _ in range(rng.integers(1, 4)):                     # craters: a hollow with a slight rim
        cx, cy = rng.uniform(-AREA, AREA, 2)
        r = rng.uniform(1.5, 3.0)
        d2 = ((gx - cx) ** 2 + (gy - cy) ** 2) / (r * r)
        h += -1.0 * np.exp(-d2) + 0.35 * np.exp(-((np.sqrt(d2) - 1.3) ** 2) / 0.1)
    peak = np.max(np.abs(h))
    h = h / peak * amplitude if peak > 0 else h
    w = np.ones_like(h)                                     # flatten the pads with a smoothstep blend
    for px, py in pads:
        d = np.maximum(np.maximum(np.abs(gx - px) - PAD, 0), 0) ** 2 + np.maximum(np.abs(gy - py) - PAD, 0) ** 2
        t = np.clip(np.sqrt(d) / BLEND, 0, 1)
        w = np.minimum(w, t * t * (3 - 2 * t))
    return xs, h * w


def height_at(xs, h, x, y):
    """Bilinear terrain height at NED (x, y)."""
    fx = np.clip((x + AREA) / TERRAIN_STEP, 0, len(xs) - 1.000001)
    fy = np.clip((y + AREA) / TERRAIN_STEP, 0, len(xs) - 1.000001)
    i, j = int(fx), int(fy)
    a, b = fx - i, fy - j
    return float((1 - a) * (1 - b) * h[i, j] + a * (1 - b) * h[i + 1, j] + (1 - a) * b * h[i, j + 1] + a * b * h[i + 1, j + 1])


def max_height_under(xs, h, rect):
    xmin, xmax, ymin, ymax = rect
    i0, i1 = [int(np.clip((v + AREA) / TERRAIN_STEP, 0, len(xs) - 1)) for v in (xmin, xmax)]
    j0, j1 = [int(np.clip((v + AREA) / TERRAIN_STEP, 0, len(xs) - 1)) for v in (ymin, ymax)]
    return float(np.max(h[i0:i1 + 2, j0:j1 + 2])), float(np.min(h[i0:i1 + 2, j0:j1 + 2]))


# ---------- obstacles ----------

def place(rng, kind, sx, sy, height_range, xs, h, pads, placed, low):
    """One obstacle on the terrain, clear of the pads; returns a Box or None after 50 tries."""
    for _ in range(50):
        x, y = rng.uniform(-AREA + sx, AREA - sx), rng.uniform(-AREA + sy, AREA - sy)
        box = Box(kind, x, y, sx, sy, 0.0, 0.0)
        if any(rect_rect_distance(box.rect(), pad_rect(*p)) < CLEARANCE for p in pads):
            continue
        hi, lo = max_height_under(xs, h, box.rect())
        box.z = lo - 0.05                                   # sink slightly so no gap shows under the box
        top = rng.uniform(*height_range) + hi
        if low:
            top = min(top, LOW_TOP)
            if top - box.z < 0.2:
                continue
        else:
            top = max(top, TALL_TOP)
        box.h = top - box.z
        if any(rect_rect_distance(box.rect(), o.rect()) < 0.3 for o in placed):
            continue
        return box
    return None


def building(rng, xs, h, pads, placed):
    """A ruined shell: four wall segments with a door gap and collapse gaps; some walls lowered (still TALL)."""
    for _ in range(50):
        w, d = rng.uniform(4, 7), rng.uniform(4, 7)
        cx, cy = rng.uniform(-AREA + w, AREA - w), rng.uniform(-AREA + d, AREA - d)
        foot = (cx - w / 2, cx + w / 2, cy - d / 2, cy + d / 2)
        if any(rect_rect_distance(foot, pad_rect(*p)) < CLEARANCE + 1.0 for p in pads):
            continue
        if any(rect_rect_distance(foot, o.rect()) < 1.0 for o in placed):
            continue
        hi, lo = max_height_under(xs, h, foot)
        t = 0.3
        sides = [("n", cx + w / 2, cy, t, d), ("s", cx - w / 2, cy, t, d), ("e", cx, cy + d / 2, w, t), ("w", cx, cy - d / 2, w, t)]
        door = rng.integers(4)
        walls = []
        for k, (name, x, y, sx, sy) in enumerate(sides):
            top = max(rng.uniform(3.0, 6.0) * (rng.uniform(0.5, 1.0) if rng.random() < 0.3 else 1.0) + hi, TALL_TOP)
            length = sx if sx > sy else sy
            gap = 1.6 if k == door else (rng.uniform(1.0, 2.0) if rng.random() < 0.3 else 0.0)
            if gap <= 0:
                segs = [(0.0, length)]
            else:
                g0 = rng.uniform(0.6, length - gap - 0.6)
                segs = [(0.0, g0), (g0 + gap, length)]
            for a, b in segs:
                mid, ln = (a + b) / 2 - length / 2, b - a
                if sx > sy:
                    walls.append(Box("ruin", x + mid, y, ln, t, lo - 0.05, top - lo + 0.05))
                else:
                    walls.append(Box("ruin", x, y + mid, t, ln, lo - 0.05, top - lo + 0.05))
        return walls
    return []


def grid_path_exists(start, goal, tall, clearance=CLEARANCE, step=0.25):
    """A* on a grid over the area: free cells keep >= clearance from every tall obstacle."""
    n = int(2 * AREA / step) + 1
    coords = np.linspace(-AREA, AREA, n)
    gx, gy = np.meshgrid(coords, coords, indexing="ij")
    blocked = np.zeros((n, n), bool)
    for b in tall:
        xmin, xmax, ymin, ymax = b.rect()
        dx = np.maximum(np.maximum(xmin - gx, gx - xmax), 0)
        dy = np.maximum(np.maximum(ymin - gy, gy - ymax), 0)
        blocked |= np.hypot(dx, dy) < clearance
    idx = lambda p: (int(round((p[0] + AREA) / step)), int(round((p[1] + AREA) / step)))  # noqa: E731
    s, g = idx(start), idx(goal)
    if blocked[s] or blocked[g]:
        return False
    seen, heap = {s}, [(0.0, s)]
    while heap:
        _, c = heapq.heappop(heap)
        if c == g:
            return True
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nb = (c[0] + di, c[1] + dj)
            if 0 <= nb[0] < n and 0 <= nb[1] < n and nb not in seen and not blocked[nb]:
                seen.add(nb)
                heapq.heappush(heap, (math.hypot(nb[0] - g[0], nb[1] - g[1]), nb))
    return False


def segment_hits_rect(p, q, rect, margin=0.0):
    for k in range(101):
        t = k / 100
        if rect_distance(p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1]), rect) <= margin:
            return True
    return False


# ---------- generation ----------

def target_hidden(start, target, tall):
    """True if every sightline from start to the target sphere (centre and both edges) crosses a tall obstacle."""
    dx, dy = target[0] - start[0], target[1] - start[1]
    d = math.hypot(dx, dy)
    px, py = -dy / d * TARGET_RADIUS, dx / d * TARGET_RADIUS
    ends = [target, (target[0] + px, target[1] + py), (target[0] - px, target[1] - py)]
    return all(any(segment_hits_rect(start, e, b.rect()) for b in tall) for e in ends)


def generate(seed: int, difficulty: int, hidden: bool = False) -> dict:
    if difficulty not in LEVELS:
        raise ValueError(f"difficulty must be one of {sorted(LEVELS)}")
    cfg = LEVELS[difficulty]
    for attempt in range(2000 if hidden else 200):
        rng = np.random.default_rng([seed, difficulty, attempt] + ([7] if hidden else []))
        dist, ang = rng.uniform(10, 18), rng.uniform(-math.pi, math.pi)
        target = (dist * math.cos(ang), dist * math.sin(ang))
        if max(abs(target[0]), abs(target[1])) > AREA - PAD - 1:
            continue
        pads = [(0.0, 0.0), target]
        xs, h = terrain_heights(rng, cfg["amplitude"], pads)
        boxes = []
        for _ in range(cfg["buildings"]):
            boxes += building(rng, xs, h, pads, boxes)
        for _ in range(cfg["wrecks"]):
            b = place(rng, "wreck", rng.uniform(3.5, 5.0), rng.uniform(1.8, 2.4), (2.6, 3.2), xs, h, pads, boxes, low=False)
            boxes += [b] if b else []
        for _ in range(cfg["trees"]):
            b = place(rng, "tree", 0.4, 0.4, (4.0, 7.0), xs, h, pads, boxes, low=False)
            boxes += [b] if b else []
        for _ in range(cfg["rubble"]):
            s = rng.uniform(0.5, 1.5)
            b = place(rng, "rubble", s, rng.uniform(0.5, 1.5), (0.3, 1.0), xs, h, pads, boxes, low=True)
            boxes += [b] if b else []
        tall = [b for b in boxes if b.top >= TALL_TOP]
        if any(rect_distance(px, py, b.rect()) < CLEARANCE + PAD for px, py in pads for b in tall):
            continue
        if not grid_path_exists(pads[0], target, tall):
            continue
        if cfg["block_line"] and not any(segment_hits_rect(pads[0], target, b.rect()) for b in tall):
            continue
        if hidden and not target_hidden(pads[0], target, tall):
            continue
        return {"format": "sar_generated_world_v1", "seed": seed, "difficulty": difficulty, "attempt": attempt,
                **({"hidden_target": True} if hidden else {}),
                "area_half_m": AREA, "terrain_step_m": TERRAIN_STEP, "terrain_amplitude_m": cfg["amplitude"],
                "start_ned": [0.0, 0.0], "target_ned": [round(target[0], 3), round(target[1], 3)],
                "pad_half_m": PAD, "low_top_m": LOW_TOP, "tall_top_m": TALL_TOP, "clearance_m": CLEARANCE,
                "terrain_heights": np.round(h, 4).tolist(), "obstacles": [dataclasses.asdict(b) for b in boxes]}
    raise RuntimeError(f"no valid world for seed {seed} difficulty {difficulty} hidden {hidden}")


# ---------- writers ----------

def terrain_obj(spec) -> str:
    """Terrain mesh in Gazebo ENU coordinates (x east = NED y, y north = NED x, z up)."""
    h = np.array(spec["terrain_heights"])
    n = h.shape[0]
    xs = np.linspace(-AREA, AREA, n)
    lines = ["# terrain mesh generated by world_generator.py (ENU metres)"]
    for i in range(n):                                       # i: NED x (north), j: NED y (east)
        for j in range(n):
            lines.append(f"v {xs[j]:.3f} {xs[i]:.3f} {h[i, j]:.4f}")
    for i in range(n - 1):
        for j in range(n - 1):
            a, b, c, d = i * n + j + 1, i * n + j + 2, (i + 1) * n + j + 1, (i + 1) * n + j + 2
            lines += [f"f {a} {b} {d}", f"f {a} {d} {c}"]   # counter-clockwise seen from above (normal +z)
    return "\n".join(lines) + "\n"


COLORS = {"ruin": "0.55 0.5 0.45 1", "wreck": "0.3 0.32 0.28 1", "rubble": "0.45 0.42 0.38 1", "tree": "0.35 0.25 0.15 1",
          "rock": "0.3 0.27 0.24 1", "rooffall": "0.42 0.38 0.33 1", "debris": "0.38 0.35 0.31 1"}   # last three: mines


def box_model(name, b: dict) -> str:
    x_enu, y_enu, z = b["y"], b["x"], b["z"] + b["h"] / 2
    size = f"{b['sy']:.3f} {b['sx']:.3f} {b['h']:.3f}"            # ENU: x extent = NED y extent
    color = COLORS[b["kind"]]
    canopy = ""
    if b["kind"] == "tree":
        top = b["h"] / 2
        canopy = (f'<visual name="canopy"><pose>0 0 {top:.3f} 0 0 0</pose><geometry><sphere><radius>1.2</radius>'
                  '</sphere></geometry><material><ambient>0.2 0.4 0.15 1</ambient><diffuse>0.2 0.4 0.15 1</diffuse>'
                  '</material></visual>')
    return (f'    <model name="{name}">\n      <static>true</static>\n      <pose>{x_enu:.3f} {y_enu:.3f} {z:.3f} 0 0 0</pose>\n'
            f'      <link name="link"><collision name="collision"><geometry><box><size>{size}</size></box></geometry>'
            f'</collision><visual name="visual"><geometry><box><size>{size}</size></box></geometry><material>'
            f'<ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>{canopy}</link>\n    </model>\n')


def world_sdf(spec, name) -> str:
    """The default world's header (physics, scene, light, coordinates) with terrain, obstacles and target."""
    base = sar_task.WORLD_SDF.read_text()
    head = base[:base.index('    <model name="ground_plane">')]
    light_start = base.index('    <light name="sunUTC"')
    target_start = base.index('    <model name="target">')
    light_and_coords = base[light_start:target_start]
    target_block = base[target_start:base.index("</model>", target_start) + len("</model>\n")]
    tx, ty = spec["target_ned"]
    target_block = target_block.replace("<pose>3 -3 0.055 0 0 0</pose>", f"<pose>{ty:g} {tx:g} {TARGET_Z} 0 0 0</pose>")
    assert f"<pose>{ty:g} {tx:g} {TARGET_Z} 0 0 0</pose>" in target_block
    header = (f'<!-- Generated by world_generator.py: name {name!r}, seed {spec["seed"]}, difficulty {spec["difficulty"]}; '
              f'target NED ({tx:g}, {ty:g}). Do not edit by hand. -->\n')
    zmin = float(np.min(spec["terrain_heights"]))
    pads = ""
    for k, (px, py) in enumerate((spec["start_ned"], spec["target_ned"])):
        pads += (f'    <model name="pad_{k}">\n      <static>true</static>\n      <pose>{py:g} {px:g} -0.05 0 0 0</pose>\n'
                 f'      <link name="link"><collision name="collision"><geometry><box><size>{2 * PAD:g} {2 * PAD:g} 0.1</size>'
                 '</box></geometry></collision></link>\n    </model>\n')
    terrain = ('    <model name="terrain">\n      <static>true</static>\n      <link name="link">'
               '<visual name="visual"><geometry><mesh><uri>terrain.obj</uri></mesh></geometry><material>'
               '<ambient>0.5 0.45 0.35 1</ambient><diffuse>0.55 0.5 0.38 1</diffuse></material></visual></link>\n    </model>\n'
               f'    <model name="ground_plane">\n      <static>true</static>\n      <pose>0 0 {zmin - 0.3:.3f} 0 0 0</pose>\n'
               '      <link name="link"><collision name="collision"><geometry><plane><normal>0 0 1</normal><size>1 1</size>'
               '</plane></geometry></collision></link>\n    </model>\n') + pads
    obstacles = "".join(box_model(f"{b['kind']}_{k}", b) for k, b in enumerate(spec["obstacles"]))
    first_newline = head.index("\n") + 1
    return head[:first_newline] + header + head[first_newline:] + terrain + light_and_coords + target_block + obstacles \
        + "  </world>\n</sdf>\n"


def write(name: str, seed: int, difficulty: int, root: Path | None = None, hidden: bool = False) -> Path:
    """Write sim/worlds/variants/<name>/{sar_walls.sdf, terrain.obj, spec.json}; refuses to overwrite."""
    if not sar_task._VARIANT_NAME.fullmatch(name) or not name.startswith("gen_"):
        raise ValueError(f"generated world names must start with gen_ and match {sar_task._VARIANT_NAME.pattern}: {name!r}")
    spec = generate(seed, difficulty, hidden)
    out = (root or sar_task.WORLD_VARIANTS) / name
    out.mkdir(parents=True, exist_ok=False)
    (out / "terrain.obj").write_text(terrain_obj(spec))
    (out / "spec.json").write_text(json.dumps(spec) + "\n")
    (out / sar_task.WORLD_SDF.name).write_text(world_sdf(spec, name))
    return out


def main(argv):
    if len(argv) not in (4, 5) or (len(argv) == 5 and argv[4] != "hidden"):
        print(__doc__, file=sys.stderr)
        return 2
    out = write(argv[1], int(argv[2]), int(argv[3]), hidden=len(argv) == 5)
    spec = json.loads((out / "spec.json").read_text())
    kinds = {}
    for b in spec["obstacles"]:
        kinds[b["kind"]] = kinds.get(b["kind"], 0) + 1
    print(f"{out}: target NED {spec['target_ned']}, obstacles {kinds}, attempt {spec['attempt']}, "
          f"sdf sha256 {sar_task.file_sha256(out / 'sar_walls.sdf')}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
