#!/usr/bin/env python3
"""Read-only Stage C gate: the world the launcher started must be the world this process (and so the runner started
from the same shell) reads as ground truth.

PASS (exit 0) only if the run's processes.json records world_variant, world_sdf and world_sdf_sha256; the variant
selected here (SAR_WORLD_VARIANT, unset = default) is the same; the selected file is the recorded file; and its sha256
now equals the recorded one (unchanged since launch). Anything missing, unreadable or different: FAIL (exit 1).

Usage: .venv/bin/python tools/verify_world_consistency.py --run-dir <run>
"""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import sar_task  # noqa: E402


def check(manifest: dict, environ=None) -> list[str]:
    missing = [k for k in ("world_variant", "world_sdf", "world_sdf_sha256") if k not in manifest]
    if missing:
        return [f"manifest lacks {missing} (started by a launcher without world recording)"]
    try:
        variant = sar_task.selected_world_variant(environ)
        selected = sar_task.selected_world_sdf(environ)
    except ValueError as exc:
        return [str(exc)]
    errors = []
    if manifest["world_variant"] != variant:
        errors.append(f"launcher variant {manifest['world_variant']!r} != selected here {variant!r} "
                      f"({sar_task.WORLD_VARIANT_ENV})")
    if manifest["world_sdf"] is None or Path(manifest["world_sdf"]).resolve() != selected.resolve():
        errors.append(f"launcher world file {manifest['world_sdf']} != selected {selected}")
    sha = sar_task.file_sha256(selected)
    if manifest["world_sdf_sha256"] != sha:
        errors.append(f"world file sha256 {sha[:12]} != launcher-recorded {str(manifest['world_sdf_sha256'])[:12]}")
    return errors


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--run-dir", type=Path, required=True)
    args = p.parse_args(argv)
    try:
        manifest = json.loads((args.run_dir / "processes.json").read_text())
    except (OSError, ValueError) as exc:
        print(f"FAIL: cannot read {args.run_dir / 'processes.json'}: {exc}")
        return 1
    errors = check(manifest)
    for error in errors:
        print(f"FAIL: {error}")
    if errors:
        return 1
    target = sar_task.load_target(Path(manifest["world_sdf"]))
    print(f"PASS: world variant {manifest['world_variant']!r}, {manifest['world_sdf']} "
          f"(sha256 {manifest['world_sdf_sha256'][:12]}), target NED ({target.x_ned}, {target.y_ned})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
