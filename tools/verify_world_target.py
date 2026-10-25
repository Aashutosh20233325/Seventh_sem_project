#!/usr/bin/env python3
"""Read-only startup gate: the running Gazebo target must match the versioned ground truth.

Compares the live pose of model ``target`` (gz /world/<world>/pose/info, ENU) with
sar_task.load_target() from the selected world file: sim/worlds/sar_walls.sdf, or the Stage C variant named by
SAR_WORLD_VARIANT. Privileged tooling only; nothing here feeds
the policy. Exit code 0 = consistent, 1 = mismatch or missing.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import sar_task  # noqa: E402

TOLERANCE_M = 0.05


def first_message(text: str) -> str:
    """`gz topic -e -n 1` can print more than one message; keep only the first Pose_V."""
    starts = [i for i in range(len(text)) if text.startswith("header {", i) and (i == 0 or text[i - 1] == "\n")]
    return text[starts[0]:starts[1]] if len(starts) > 1 else text


def parse_pose_info(text: str, name: str = "target") -> list[tuple[float, float, float]]:
    """Positions of every pose block named ``name`` in the first Pose_V of `gz topic -e` output."""
    found = []
    for block in first_message(text).split("pose {")[1:]:
        if not re.search(rf'name: "{re.escape(name)}"', block):
            continue
        position = re.search(r"position \{([^}]*)\}", block)
        values = {"x": 0.0, "y": 0.0, "z": 0.0}       # protobuf omits zero fields
        if position:
            for axis, value in re.findall(r"([xyz]): ([-0-9.eE+]+)", position.group(1)):
                values[axis] = float(value)
        found.append((values["x"], values["y"], values["z"]))
    return found


def check(found: list[tuple[float, float, float]], target: sar_task.Target) -> list[str]:
    if len(found) != 1:
        return [f"expected exactly one live target pose, found {len(found)}"]
    x, y, z = found[0]
    expected = (target.y_ned, target.x_ned, target.z_enu)      # NED -> ENU (x_enu = y_ned)
    error = max(abs(a - b) for a, b in zip((x, y, z), expected))
    if error > TOLERANCE_M:
        return [f"live target ENU {(x, y, z)} differs from versioned {expected} by {error:.3f} m"]
    return []


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="sar_walls")
    args = parser.parse_args(argv)
    result = subprocess.run(
        ["gz", "topic", "-e", "-n", "1", "-t", f"/world/{args.world}/pose/info"],
        capture_output=True, text=True, timeout=15,
    )
    try:
        world_sdf = sar_task.selected_world_sdf()
    except ValueError as exc:
        print(f"FAIL: {exc}")
        return 1
    errors = check(parse_pose_info(result.stdout), sar_task.load_target(world_sdf))
    for error in errors:
        print(f"FAIL: {error}")
    where = f"{world_sdf.relative_to(sar_task.SIM_WORLDS.parents[1])} (sha256 {sar_task.file_sha256(world_sdf)[:12]})"
    if not errors:
        print(f"PASS: live Gazebo target matches {where}")
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
