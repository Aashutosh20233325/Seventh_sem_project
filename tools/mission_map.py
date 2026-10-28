#!/usr/bin/env python3
"""Mission map animation and summary card from one flown episode (presentation; offline, read-only on the logs).

Rebuilds what the drone knew, step by step, with the explorer's own map code (explorer.ExplorerMap): walls from the
logged LiDAR sectors (telemetry_full.jsonl, matched to each step by the shared monotonic clock), the area the camera
has searched, the flown path, the current goal, the moment the target was first seen, the supply-packet drop.
Writes <out>/mission_map.gif (north up, east right) and <out>/mission_summary.png.

Usage: .venv/bin/python tools/mission_map.py <run dir runs/.../restart_<name>> [--out DIR] [--every N]
"""

from __future__ import annotations

import argparse
import bisect
import io
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import explorer as ex  # noqa: E402
import sar_observation as so  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from PIL import Image  # noqa: E402

DT = 0.1                          # s of simulation per step
COLORS = {"unknown": (0.09, 0.09, 0.10), "free": (0.42, 0.40, 0.37), "occ": (0.85, 0.78, 0.66),
          "searched": (0.20, 0.62, 0.62)}


def load(run: Path):
    logs = run / "logs"
    rows = [json.loads(line) for line in open(logs / "episode.jsonl")]
    steps = [r for r in rows if r["event"] == "step"]
    world = next((r for r in rows if r["event"] == "world"), {})
    end = next((r.get("reason") for r in rows if r["event"] == "episode_end"), None)
    lidar_t, lidar = [], []
    for line in open(logs / "telemetry_full.jsonl"):
        try:
            r = json.loads(line)
        except ValueError:
            continue
        if r.get("event") == "lidar_sectors":
            lidar_t.append(r["mono"])
            lidar.append(np.asarray(r["msg"]["data"], float))
    return steps, world, end, lidar_t, lidar


def mine_cells(spec):
    """Grid cells of the open mine (not inside a tall obstacle, inside the obstacles' bounding box): the denominator
    of "searched %"."""
    import world_generator as wg
    m = ex.ExplorerMap()
    c = -ex.HALF + (np.arange(ex.N) + 0.5) * ex.RES
    gx, gy = np.meshgrid(c, c, indexing="ij")
    tall = [wg.Box(**b) for b in spec["obstacles"] if b["z"] + b["h"] >= wg.TALL_TOP]
    solid = np.zeros((ex.N, ex.N), bool)
    for b in tall:
        x0, x1, y0, y1 = b.rect()
        solid |= (gx >= x0) & (gx <= x1) & (gy >= y0) & (gy <= y1)
    xs = [v for b in tall for v in b.rect()[:2]]
    ys = [v for b in tall for v in b.rect()[2:]]
    inside = (gx > min(xs)) & (gx < max(xs)) & (gy > min(ys)) & (gy < max(ys))
    return inside & ~solid & ~m.outside


def frame(m, path, step, mode, goal, seen_at, target, drop, title, size=7.0):
    img = np.zeros((ex.N, ex.N, 3))
    img[:] = COLORS["unknown"]
    img[m.state == ex.FREE] = COLORS["free"]
    srch = m.searched & ~m.outside & (m.state != ex.OCC)
    img[srch] = 0.45 * np.array(COLORS["free"]) + 0.55 * np.array(COLORS["searched"])
    img[m.state == ex.OCC] = COLORS["occ"]
    fig, ax = plt.subplots(figsize=(size, size), dpi=100)
    fig.patch.set_facecolor("#111214")
    # rows = NED x (north, up), columns = NED y (east, right)
    ax.imshow(img, origin="lower", extent=[-ex.HALF, ex.HALF, -ex.HALF, ex.HALF], interpolation="nearest")
    if len(path) > 1:
        p = np.array(path)
        ax.plot(p[:, 1], p[:, 0], color="#ff9f1c", lw=2.2, alpha=0.95)
    ax.plot([0], [0], marker="o", ms=11, mfc="none", mec="#7fb7ff", mew=2.2)
    ax.annotate("borehole", (0, 0), xytext=(0.6, -1.8), color="#7fb7ff", fontsize=9)
    if goal is not None and mode == "EXPLORE":
        ax.plot([goal[1]], [goal[0]], marker="*", ms=15, color="#ffd23f", mec="black", mew=0.6)
    if seen_at is not None and target is not None:
        ax.plot([target[1]], [target[0]], marker="o", ms=16, color="#b6f542", mec="black", mew=1.0)
        ax.annotate(f"MINER FOUND (step {seen_at})", (target[1], target[0]), xytext=(target[1] - 6, target[0] + 1.6),
                    color="#b6f542", fontsize=10, weight="bold")
    if drop is not None:
        ax.plot([drop[1]], [drop[0]], marker="s", ms=9, color="#f5c518", mec="#1f5fd1", mew=2)
        ax.annotate("supply packet", (drop[1], drop[0]), xytext=(drop[1] + 0.6, drop[0] - 1.8), color="#f5c518", fontsize=9)
    if path:
        x, y, h = path[-1][0], path[-1][1], path[-1][2]
        ax.arrow(y, x, 1.4 * math.sin(h), 1.4 * math.cos(h), width=0.25, color="white", length_includes_head=True)
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    legend = [Patch(color=COLORS["occ"], label="rock walls (mapped by LiDAR)"),
              Patch(color=COLORS["free"], label="open tunnel, not yet searched"),
              Patch(color=0.45 * np.array(COLORS["free"]) + 0.55 * np.array(COLORS["searched"]), label="searched by the camera"),
              Line2D([], [], color="#ff9f1c", lw=2.2, label="flight path"),
              Line2D([], [], marker="*", ls="", ms=12, color="#ffd23f", mec="black", label="next place to look")]
    ax.legend(handles=legend, loc="lower left", fontsize=8, facecolor="#1b1d20", edgecolor="#444", labelcolor="white",
              framealpha=0.9)
    lim = ex.SEARCH_HALF + 0.5
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_xticks([]), ax.set_yticks([])
    ax.set_title(title, color="white", fontsize=12, loc="left")
    fig.tight_layout()
    buf = io.BytesIO()
    fig.savefig(buf, format="png", facecolor=fig.get_facecolor())
    plt.close(fig)
    buf.seek(0)
    return Image.open(buf).convert("RGB")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", type=Path)
    ap.add_argument("--out", type=Path)
    ap.add_argument("--every", type=int, default=4, help="one animation frame per N steps (default 4)")
    a = ap.parse_args(argv)
    out = a.out or a.run / "presentation"
    out.mkdir(parents=True, exist_ok=True)
    steps, world, end, lidar_t, lidar = load(a.run)
    variant = world.get("variant")
    spec = json.loads((REPO / "sim/worlds/variants" / variant / "spec.json").read_text()) if variant else None
    target = world.get("target_ned")
    m = ex.ExplorerMap()
    path, frames, seen_at, drop, dist = [], [], None, None, 0.0
    goal = None
    for k, s in enumerate(steps):
        q = s["info"]["privileged"]
        x, y, h = q["x"], q["y"], q["heading"]
        if path:
            dist += math.hypot(x - path[-1][0], y - path[-1][1])
        path.append((x, y, h))
        j = bisect.bisect_right(lidar_t, s["mono"]) - 1
        if j >= 0:
            obs = np.zeros(so.DIM, np.float32)
            obs[so.SLICES["lidar"]] = np.clip(lidar[j] / so.LIDAR_RANGE_MAX, 0, 1)
            m.update_lidar(x, y, h, obs)
        m.view(x, y, h, mark=True)
        p = s.get("policy") or {}
        mode = p.get("mode")
        goal = p.get("goal") or goal
        if seen_at is None and mode == "APPROACH":
            seen_at = s["step"]
        drop = s["info"].get("supply_drop_ned") or drop
        if k % a.every == 0 or k == len(steps) - 1:
            label = "searching" if mode == "EXPLORE" else "miner in sight: approaching"
            frames.append(frame(m, path, s["step"], mode, goal, seen_at, target, drop,
                                f"t = {s['step'] * DT:5.1f} s   {label}"))
    if frames:
        frames += [frames[-1]] * 12                       # hold the last frame ~1.5 s
        frames[0].save(out / "mission_map.gif", save_all=True, append_images=frames[1:], duration=120, loop=0,
                       optimize=True)
    # summary card
    cells = mine_cells(spec) if spec else None
    pct = 100.0 * (m.searched & cells).sum() / cells.sum() if cells is not None and cells.sum() else float("nan")
    last = steps[-1]["step"] if steps else 0
    lines = [
        ("Outcome", {"success": "miner reached", "guard_abort": "stopped by the safety guard"}.get(end, str(end))),
        ("Miner first seen", f"{seen_at * DT:.1f} s (step {seen_at})" if seen_at is not None else "not found"),
        ("Mission time", f"{last * DT:.1f} s simulated ({last} steps)"),
        ("Distance flown", f"{dist:.1f} m"),
        ("Mine area searched", f"{pct:.0f} %"),
        ("Miner position", (f"{abs(target[0]):.1f} m {'north' if target[0] >= 0 else 'south'}, "
                            f"{abs(target[1]):.1f} m {'east' if target[1] >= 0 else 'west'} of the borehole") if target else "-"),
        ("Supply packet", "dropped beside the miner" if drop else "not dropped in this flight"),
        ("World", f"{variant}"),
    ]
    card = frame(m, path, last, None, None, seen_at, target, drop, "Mission summary", size=7.0)
    fig = plt.figure(figsize=(14, 7), dpi=100)
    fig.patch.set_facecolor("#111214")
    ax_map = fig.add_axes([0.0, 0.0, 0.5, 1.0])
    ax_map.imshow(card)
    ax_map.axis("off")
    ax_txt = fig.add_axes([0.52, 0.05, 0.46, 0.9])
    ax_txt.axis("off")
    ax_txt.text(0.0, 0.97, "Rescue drone: mine search", color="white", fontsize=20, weight="bold", va="top")
    for i, (k, v) in enumerate(lines):
        ax_txt.text(0.0, 0.83 - i * 0.095, k, color="#9aa4ad", fontsize=13, va="top")
        ax_txt.text(0.42, 0.83 - i * 0.095, v, color="white", fontsize=13, va="top")
    fig.savefig(out / "mission_summary.png", facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"{out / 'mission_map.gif'} ({len(frames)} frames), {out / 'mission_summary.png'}")
    for k, v in lines:
        print(f"  {k}: {v}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
