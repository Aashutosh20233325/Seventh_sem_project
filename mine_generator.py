#!/usr/bin/env python3
"""Procedural mine worlds (mine scenario): one seed -> one underground drift network with a trapped worker.

Same output format as world_generator.py (sim/worlds/variants/<name>/{sar_walls.sdf, terrain.obj, spec.json}; spec
format sar_generated_world_v1), so the stand-in (sar_standin_env.load_walls), the explorer, the runner, the world gates
and the checker work unchanged.

Layout (PX4 local NED, start at the origin, the +-20 m area of the generated worlds):
  junctions  a 4 x 4 grid, NODE_PITCH = 8 m apart (x, y in {-8, 0, 8, 16}); each junction is a 5 x 5 m chamber (a 5 x 5
             grid takes ~2000 steps to search, beyond a live flight's memory budget)
  drifts     5 m wide tunnels between neighbouring junctions: a random spanning tree plus LOOPS extra links (mines have
             cross-cuts); the rest is solid rock (TALL boxes up to ROCK_TOP)
  roof falls ROOF_FALLS drifts fully blocked by a rock fall (TALL; the drone must find another way); never one that
             would disconnect the target
  debris     low rock piles (top <= 1.1 m, flown over) in some drifts and junctions
  target     the seated worker in a hi-vis jacket (sim/models/rescue_worker) in a junction 10-18 m from the start, hidden
             from the start (the same three-sightline test as world_generator.target_hidden), reachable with the
             generated worlds' 1.5 m clearance; the physical target is still the `target` model's 0.5 m collision sphere
             (sar_task.load_target), only its visual changes
  lighting   dim ambient light and sun (underground), lamps over junctions; the floor is flat (terrain heights 0)
The RGB detector finds the jacket with target_perception.hivis_mask.

Usage: .venv/bin/python mine_generator.py <name gen_m...> <seed>
"""

from __future__ import annotations

import dataclasses
import json
import math
import sys
from pathlib import Path

import numpy as np

import sar_task
import world_generator as wg

NODE_PITCH = 8.0                  # m between junction centres
NODES = (-8.0, 0.0, 8.0, 16.0)    # junction coordinates on both axes: a 4 x 4 network, start at (0, 0)
START = (NODES.index(0.0), NODES.index(0.0))
K = len(NODES)
DRIFT = 5.0                       # m: drift and junction width
ROCK_TOP = 4.0                    # m: rock wall height (the roof is open so Gazebo's GUI can watch from above)
LOOPS = 3
ROOF_FALLS = 2
DEBRIS = 4
CARTS = 2                         # decorative mine carts on the rails (visual only, 0.65 m high: flown over)
TARGET_MIN, TARGET_MAX = 10.0, 18.0
TARGET_WALL_OFFSET = 1.0          # m from the junction centre toward one wall (sitting against the rock)


def _edges():
    out = []
    for i, x in enumerate(NODES):
        for j, y in enumerate(NODES):
            if i + 1 < len(NODES):
                out.append(((i, j), (i + 1, j)))
            if j + 1 < len(NODES):
                out.append(((i, j), (i, j + 1)))
    return out


def _connected(open_edges, a, b):
    adj = {}
    for u, v in open_edges:
        adj.setdefault(u, []).append(v)
        adj.setdefault(v, []).append(u)
    seen, stack = {a}, [a]
    while stack:
        u = stack.pop()
        if u == b:
            return True
        for v in adj.get(u, []):
            if v not in seen:
                seen.add(v)
                stack.append(v)
    return False


def _drift_rect(e):
    (i0, j0), (i1, j1) = e
    x0, y0, x1, y1 = NODES[i0], NODES[j0], NODES[i1], NODES[j1]
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    gap = NODE_PITCH - DRIFT                                  # 3 m of drift between two junction chambers
    return (cx, cy, gap, DRIFT) if i0 != i1 else (cx, cy, DRIFT, gap)


def generate(seed: int) -> dict:
    edges = _edges()
    start = START
    for attempt in range(2000):
        rng = np.random.default_rng([seed, 31, attempt])
        # random spanning tree (randomised Kruskal) + extra loops
        parent = {(i, j): (i, j) for i in range(K) for j in range(K)}

        def find(u):
            while parent[u] != u:
                parent[u] = parent[parent[u]]
                u = parent[u]
            return u

        order = [edges[k] for k in rng.permutation(len(edges))]
        tree, rest = [], []
        for e in order:
            a, b = find(e[0]), find(e[1])
            if a != b:
                parent[a] = b
                tree.append(e)
            else:
                rest.append(e)
        open_edges = tree + rest[:LOOPS]
        closed = rest[LOOPS:]
        # target junction: 10-18 m away, then roof falls that keep it reachable
        cands = [(i, j) for i in range(K) for j in range(K)
                 if TARGET_MIN <= math.hypot(NODES[i], NODES[j]) <= TARGET_MAX]
        tnode = cands[int(rng.integers(len(cands)))]
        falls = []
        for k in rng.permutation(len(open_edges)):
            e = open_edges[int(k)]
            if len(falls) == ROOF_FALLS:
                break
            trial = [x for x in open_edges if x != e and x not in falls]
            if start not in e and tnode not in e and _connected(trial, start, tnode):
                falls.append(e)
        open_now = [e for e in open_edges if e not in falls]
        # the worker sits against one wall of the target junction (a side without a drift)
        sides = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        nb = lambda d: (tnode[0] + d[0], tnode[1] + d[1])  # noqa: E731
        walls = [d for d in sides if not any(set(e) == {tnode, nb(d)} for e in open_now)]
        if not walls:                                         # a four-way junction: no wall to sit against
            continue
        d = walls[int(rng.integers(len(walls)))]
        target = (NODES[tnode[0]] + d[0] * TARGET_WALL_OFFSET, NODES[tnode[1]] + d[1] * TARGET_WALL_OFFSET)
        boxes = _rock(closed, falls, rng)
        tall = [b for b in boxes if b.top >= wg.TALL_TOP]
        if not wg.grid_path_exists((0.0, 0.0), target, tall, clearance=wg.CLEARANCE):
            continue
        if not wg.target_hidden((0.0, 0.0), target, tall):
            continue
        boxes += _debris(open_now, falls, tnode, rng)
        lamps = _lamps(tnode, rng)
        n = int(2 * wg.AREA / wg.TERRAIN_STEP) + 1
        return {"format": "sar_generated_world_v1", "scenario": "mine", "seed": seed, "difficulty": 2, "attempt": attempt,
                "hidden_target": True, "area_half_m": wg.AREA, "terrain_step_m": wg.TERRAIN_STEP,
                "terrain_amplitude_m": 0.0, "start_ned": [0.0, 0.0], "target_ned": [round(target[0], 3), round(target[1], 3)],
                "target_model": "rescue_worker", "target_yaw_enu": round(math.atan2(-d[0], -d[1]), 3),
                "pad_half_m": wg.PAD, "low_top_m": wg.LOW_TOP, "tall_top_m": wg.TALL_TOP, "clearance_m": wg.CLEARANCE,
                "drift_width_m": DRIFT, "roof_falls": [[list(e[0]), list(e[1])] for e in falls],
                "lamps_ned": [[round(x, 2), round(y, 2)] for x, y in lamps],
                "terrain_heights": np.zeros((n, n)).tolist(), "obstacles": [dataclasses.asdict(b) for b in boxes]}
    raise RuntimeError(f"no valid mine for seed {seed}")


def _rock(closed, falls, rng):
    """Solid rock everywhere except junction chambers and open drifts: pillars, closed drifts, a perimeter band."""
    boxes = []
    pill = NODE_PITCH - DRIFT
    for x in NODES[:-1]:
        for y in NODES[:-1]:
            boxes.append(wg.Box("rock", x + NODE_PITCH / 2, y + NODE_PITCH / 2, pill, pill, 0.0, ROCK_TOP))
    for e in closed:
        cx, cy, sx, sy = _drift_rect(e)
        boxes.append(wg.Box("rock", cx, cy, sx, sy, 0.0, ROCK_TOP))
    for e in falls:
        cx, cy, sx, sy = _drift_rect(e)
        boxes.append(wg.Box("rooffall", cx, cy, sx, sy, 0.0, round(float(rng.uniform(2.8, 3.6)), 2)))
    lo, hi = NODES[0] - DRIFT / 2, NODES[-1] + DRIFT / 2       # the network's edges (same on both axes)
    band, mid, span = 2.0, (NODES[0] + NODES[-1]) / 2, hi - lo
    for edge, s in ((lo, -1), (hi, 1)):
        boxes.append(wg.Box("rock", edge + s * band / 2, mid, band, span + 2 * band, 0.0, ROCK_TOP))
        boxes.append(wg.Box("rock", mid, edge + s * band / 2, span, band, 0.0, ROCK_TOP))
    return boxes


def _debris(open_now, falls, tnode, rng):
    """Low rock piles (flown over) in open drifts away from the start and the target junction."""
    out = []
    spots = [e for e in open_now if START not in e and tnode not in e]
    for k in rng.permutation(len(spots))[:DEBRIS]:
        cx, cy, sx, sy = _drift_rect(spots[int(k)])
        out.append(wg.Box("debris", round(cx + float(rng.uniform(-0.6, 0.6)), 2), round(cy + float(rng.uniform(-0.6, 0.6)), 2),
                          round(float(rng.uniform(1.0, 2.2)), 2), round(float(rng.uniform(1.0, 2.2)), 2), 0.0,
                          round(float(rng.uniform(0.3, 0.9)), 2)))
    return out


def _lamps(tnode, rng):
    """No point lights: each makes every camera and GUI frame dearer (mines3, 2026-10-28: camera-result gaps > 1.5 s
    with 9 lamps); the mine is lit by ambient and overhead light so the Gazebo view shows it clearly."""
    return []


def world_sdf(spec, name) -> str:
    """world_generator.world_sdf with the mine's look: rock, timber, ambient and overhead light (no lamps), the hi-vis
    worker, and the story's props (_decor)."""
    sdf = wg.world_sdf(spec, name)
    sdf = sdf.replace("<!-- Generated by world_generator.py:", "<!-- Generated by mine_generator.py (mine scenario):", 1)
    # target: same model name, pose and 0.5 m collision sphere; the visual becomes the seated worker
    start = sdf.index('    <model name="target">')
    end = sdf.index("</model>", start) + len("</model>")
    block = sdf[start:end]
    v0, v1 = block.index("<visual"), block.index("</visual>") + len("</visual>")
    worker = (f'<visual name="visual"><pose>0 0 {-0.623996 - wg.TARGET_Z:.4f} 0 0 {spec["target_yaw_enu"]}</pose><geometry><mesh>'
              '<uri>model://rescue_worker/meshes/rescue_randy.dae</uri></mesh></geometry></visual>')
    sdf = sdf[:start] + block[:v0] + worker + block[v1:] + sdf[end:]
    # underground light: dim ambient, weak sun, dark background, lamps over junctions
    for a, b in (("<ambient>0.4 0.4 0.4 1</ambient>", "<ambient>0.7 0.69 0.68 1</ambient>"),
                 ("<background>0.7 0.7 0.7 1</background>", "<background>0.08 0.08 0.09 1</background>"),
                 ("<diffuse>0.904 0.904 0.904 1</diffuse>", "<diffuse>0.85 0.83 0.8 1</diffuse>")):
        assert sdf.count(a) == 1, a
        sdf = sdf.replace(a, b)
    # flat textured floor instead of the (flat) terrain mesh, rock texture on rock, roof falls and debris
    v0 = sdf.index('<visual name="visual"><geometry><mesh><uri>terrain.obj</uri>')
    v1 = sdf.index("</visual>", v0) + len("</visual>")
    size = 2 * (wg.AREA + 3)
    # the floor is visual only, as in the other generated worlds (solid ground: the start and target pads and a safety
    # plane 0.3 m below). A 46 x 46 m floor collision box (2026-10-28) coincided with camera rendering stalls (gaps
    # 1.3-2.1 s, readiness failed) and was removed; landings happen on the target pad (land_beside_target), and a
    # landing elsewhere is caught by PX4UAVEnv's stuck-landing check.
    floor = f'<geometry><box><size>{size:g} {size:g} 0.1</size></box></geometry>'
    sdf = sdf[:v0] + (f'<visual name="visual">{floor}{_material("floor", "0.85 0.85 0.85 1")}</visual>') + sdf[v1:]
    m = '    <model name="terrain">\n      <static>true</static>\n'
    assert sdf.count(m) == 1
    sdf = sdf.replace(m, m + "      <pose>0 0 -0.05 0 0 0</pose>\n")      # model pose: read by sar_standin_env.load_walls
    for kind in ("rock", "rooffall", "debris"):
        c = wg.COLORS[kind]
        sdf = sdf.replace(f"<material><ambient>{c}</ambient><diffuse>{c}</diffuse></material>",
                          _material("rock", "0.9 0.88 0.86 1" if kind == "rock" else "1 0.97 0.94 1"))
    # timber sets (two posts flush with the rock, 5 cm proud, and a cap at 3.6 m) in every open drift; visual only
    sets = ""
    for k, e in enumerate(_open_drifts(spec)):
        cx, cy, sx, sy = _drift_rect(e)
        along_x = sx < sy                                     # the drift runs along NED x (its width is along y)
        for s in (-1, 1):
            px, py = (cx, cy + s * (DRIFT / 2 + 0.1)) if along_x else (cx + s * (DRIFT / 2 + 0.1), cy)
            sets += _visual_box(f"timber_post_{k}_{s}", px, py, 1.8, 0.3, 0.3, 3.6, "timber")
        cap = (0.3, DRIFT + 0.6) if along_x else (DRIFT + 0.6, 0.3)
        sets += _visual_box(f"timber_cap_{k}", cx, cy, 3.72, cap[0], cap[1], 0.25, "timber")
    return sdf.replace("  </world>\n</sdf>\n", sets + _decor(spec) + "  </world>\n</sdf>\n")


def _decor(spec):
    """The story's props, all visual only and clear of the sensors' view in flight: the rescue borehole above the start
    junction (casing on the rock top, drilling frame, rope ending 2.6 m above the floor, above the 1.8 m flight height),
    rock chunks and broken timber on each roof fall (at most 0.15 m proud of the block) with a ROOF FALL plaque on top,
    rails along the open drifts (0.06 m) and two mine carts (0.65 m, flown over). Plaques face up: readable in the
    top-down view, never in the down-pitched drone camera (they are above it)."""
    rng = np.random.default_rng([spec["seed"], 77])
    out = ""
    sx0, sy0 = NODES[START[0]], NODES[START[1]]
    # rock roof over the start junction with a 1.4 m square opening (the borehole), casing ring around the opening
    hole, slab = 1.4, DRIFT + 0.4
    w = (slab - hole) / 2
    for k, (dx, dy, sx, sy) in enumerate(((0, (hole + w) / 2, slab, w), (0, -(hole + w) / 2, slab, w),
                                          ((hole + w) / 2, 0, w, hole), (-(hole + w) / 2, 0, w, hole))):
        out += _visual_box(f"borehole_roof_{k}", sx0 + dx, sy0 + dy, ROCK_TOP - 0.15, sx, sy, 0.3, "rock")
    for k in range(12):
        a = 2 * math.pi * k / 12
        out += _visual_box_rot(f"borehole_casing_{k}", sx0 + 0.9 * math.cos(a), sy0 + 0.9 * math.sin(a), ROCK_TOP + 0.2,
                               0.5, 0.15, 0.4, a + math.pi / 2, "steel")
    # drilling frame: four legs from the pillar tops (+-3.2, +-3.2) to a crown 9 m up, crown block, rope to 2.6 m
    top = 9.0
    for k, (dx, dy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        bx, by = sx0 + 3.2 * dx, sy0 + 3.2 * dy
        mx, my, mz = (bx + sx0) / 2, (by + sy0) / 2, (ROCK_TOP + top) / 2
        length = math.sqrt((3.2 * math.sqrt(2)) ** 2 + (top - ROCK_TOP) ** 2)
        tilt = math.atan2(3.2 * math.sqrt(2), top - ROCK_TOP)
        out += _visual_leg(f"derrick_leg_{k}", mx, my, mz, length, tilt, math.atan2(-dy, -dx))
    out += _visual_box("derrick_crown", sx0, sy0, top + 0.2, 0.9, 0.9, 0.4, "steel")
    out += _visual_box("borehole_rope", sx0, sy0, (2.6 + top) / 2, 0.04, 0.04, top - 2.6, "steel")
    out += _visual_plaque("sign_borehole", sx0 + 4.0, sy0 - 4.6, ROCK_TOP + 0.02, 2.4, 1.2, "sign_borehole")
    # roof falls: chunks on top and against the faces, broken timbers, a plaque
    for k, b in enumerate(o for o in spec["obstacles"] if o["kind"] == "rooffall"):
        top_z = b["z"] + b["h"]
        for j in range(14):
            cx = b["x"] + float(rng.uniform(-0.45, 0.45)) * b["sx"]
            cy = b["y"] + float(rng.uniform(-0.45, 0.45)) * b["sy"]
            s = float(rng.uniform(0.4, 1.0))
            z = top_z + float(rng.uniform(-0.1, 0.25)) if j < 8 else float(rng.uniform(0.3, top_z - 0.3))
            out += _visual_box_rot(f"rooffall_{k}_chunk_{j}", cx, cy, z, s, s * 0.8, s * 0.6,
                                   float(rng.uniform(0, math.pi)), "rock", roll=float(rng.uniform(-0.5, 0.5)))
        for j in range(2):
            out += _visual_box_rot(f"rooffall_{k}_timber_{j}", b["x"] + float(rng.uniform(-0.3, 0.3)) * b["sx"],
                                   b["y"] + float(rng.uniform(-0.3, 0.3)) * b["sy"], top_z + 0.1, 3.0, 0.25, 0.25,
                                   float(rng.uniform(0, math.pi)), "timber", roll=0.0, pitch=float(rng.uniform(-0.3, 0.3)))
        out += _visual_plaque(f"sign_rooffall_{k}", b["x"], b["y"], top_z + 0.8, 2.0, 1.0, "sign_rooffall")
    # rails along every open drift (junction centre to junction centre), two carts away from start and target
    opens = _open_drifts(spec)
    for k, ((i0, j0), (i1, j1)) in enumerate(opens):
        x0, y0, x1, y1 = NODES[i0], NODES[j0], NODES[i1], NODES[j1]
        along_x = j0 == j1
        for s in (-1, 1):
            off = 0.55 * s
            rx, ry = ((x0 + x1) / 2, y0 + off) if along_x else (x0 + off, (y0 + y1) / 2)
            out += _visual_box(f"rail_{k}_{s}", rx, ry, 0.03, NODE_PITCH if along_x else 0.07,
                               0.07 if along_x else NODE_PITCH, 0.06, "steel")
    tgt = tuple(spec["target_ned"])
    spots = [e for e in opens if START not in e and math.hypot((NODES[e[0][0]] + NODES[e[1][0]]) / 2 - tgt[0],
                                                             (NODES[e[0][1]] + NODES[e[1][1]]) / 2 - tgt[1]) > 6]
    for k in rng.permutation(len(spots))[:CARTS]:
        (i0, j0), (i1, j1) = spots[int(k)]
        cx, cy = (NODES[i0] + NODES[i1]) / 2, (NODES[j0] + NODES[j1]) / 2
        yaw = 0.0 if j0 == j1 else math.pi / 2                        # NED yaw along the drift
        out += _visual_box_rot(f"cart_{k}_body", cx, cy, 0.4, 1.4, 0.9, 0.5, yaw, "steel_rust")
        for w, (u, v) in enumerate(((0.45, 0.35), (0.45, -0.35), (-0.45, 0.35), (-0.45, -0.35))):
            wx = cx + u * math.cos(yaw) - v * math.sin(yaw)
            wy = cy + u * math.sin(yaw) + v * math.cos(yaw)
            out += _visual_box_rot(f"cart_{k}_wheel_{w}", wx, wy, 0.12, 0.24, 0.08, 0.24, yaw, "steel")
    return out


def _material(texture, color):
    return (f"<material><ambient>{color}</ambient><diffuse>{color}</diffuse><pbr><metal><albedo_map>"
            f"model://mine_textures/materials/textures/{texture}.png</albedo_map><roughness>0.95</roughness>"
            "<metalness>0</metalness></metal></pbr></material>")


def _visual_box_rot(name, x, y, z, sx, sy, sz, yaw_ned, texture, roll=0.0, pitch=0.0):
    """Visual-only box, NED position and NED yaw (ENU yaw = pi/2 - yaw_ned); sx along the box's own x."""
    return (f'    <model name="{name}">\n      <static>true</static>\n      <pose>{y:.3f} {x:.3f} {z:.3f} {roll:.3f} {pitch:.3f} '
            f'{math.pi / 2 - yaw_ned:.4f}</pose>\n      <link name="link"><visual name="visual"><geometry><box><size>'
            f'{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>{_material(texture, "1 1 1 1")}</visual></link>\n'
            '    </model>\n')


def _visual_leg(name, x, y, z, length, tilt, heading_ned):
    """A derrick leg: a 0.2 m box of `length` centred at (x, y, z), leaning by `tilt` toward NED heading."""
    yaw_enu = math.pi / 2 - heading_ned
    return (f'    <model name="{name}">\n      <static>true</static>\n      <pose>{y:.3f} {x:.3f} {z:.3f} 0 {tilt:.4f} '
            f'{yaw_enu:.4f}</pose>\n      <link name="link"><visual name="visual"><geometry><box><size>0.2 0.2 '
            f'{length:.3f}</size></box></geometry>{_material("steel", "1 1 1 1")}</visual></link>\n    </model>\n')


def _visual_plaque(name, x, y, z, sx, sy, texture):
    """A thin horizontal plaque, face up, sx along east: turned so the text reads left to right in the top-down view
    (north up)."""
    return (f'    <model name="{name}">\n      <static>true</static>\n      <pose>{y:.3f} {x:.3f} {z:.3f} 0 0 {math.pi / 2:.4f}'
            f'</pose>\n      <link name="link"><visual name="visual"><geometry><box><size>{sy:.3f} {sx:.3f} 0.02</size></box>'
            f'</geometry>{_material(texture, "1 1 1 1")}</visual></link>\n    </model>\n')


def _visual_box(name, x, y, z, sx, sy, sz, texture):
    """A visual-only box (no collision: the stand-in and physics ignore it), NED position, ENU pose."""
    return (f'    <model name="{name}">\n      <static>true</static>\n      <pose>{y:.3f} {x:.3f} {z:.3f} 0 0 0</pose>\n'
            f'      <link name="link"><visual name="visual"><geometry><box><size>{sy:.3f} {sx:.3f} {sz:.3f}</size></box>'
            f'</geometry>{_material(texture, "1 1 1 1")}</visual></link>\n    </model>\n')


def _open_drifts(spec):
    """Drifts that are neither solid rock nor a roof fall, recovered from the obstacle boxes."""
    rects = {(round(b["x"], 3), round(b["y"], 3)) for b in spec["obstacles"] if b["kind"] in ("rock", "rooffall")}
    return [e for e in _edges() if tuple(round(v, 3) for v in _drift_rect(e)[:2]) not in rects]


def write(name: str, seed: int, root: Path | None = None) -> Path:
    """Write sim/worlds/variants/<name>/{sar_walls.sdf, terrain.obj, spec.json}; refuses to overwrite."""
    if not sar_task._VARIANT_NAME.fullmatch(name) or not name.startswith("gen_m"):
        raise ValueError(f"mine world names must start with gen_m and match {sar_task._VARIANT_NAME.pattern}: {name!r}")
    spec = generate(seed)
    out = (root or sar_task.WORLD_VARIANTS) / name
    out.mkdir(parents=True, exist_ok=False)
    (out / "terrain.obj").write_text(wg.terrain_obj(spec))
    (out / "spec.json").write_text(json.dumps(spec) + "\n")
    (out / sar_task.WORLD_SDF.name).write_text(world_sdf(spec, name))
    return out


def main(argv):
    if len(argv) != 3:
        print(__doc__, file=sys.stderr)
        return 2
    out = write(argv[1], int(argv[2]))
    spec = json.loads((out / "spec.json").read_text())
    kinds = {}
    for b in spec["obstacles"]:
        kinds[b["kind"]] = kinds.get(b["kind"], 0) + 1
    print(f"{out}: target NED {spec['target_ned']} ({math.hypot(*spec['target_ned']):.1f} m), obstacles {kinds}, "
          f"roof falls {spec['roof_falls']}, attempt {spec['attempt']}, sdf sha256 {sar_task.file_sha256(out / 'sar_walls.sdf')}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
