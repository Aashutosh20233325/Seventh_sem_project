#!/usr/bin/env python3
"""Run exactly one supervised target-reaching episode with the observation-only baseline controller.

SITL use (only after the runbook gates pass; see runs/sar_demo_20261018/RUNBOOK_ISOLATED.md):
    ROS_DOMAIN_ID=43 .venv/bin/python -u run_target_reaching_episode.py --log <run>/logs/episode.jsonl \\
        --i-confirm-supervised-flight

Offline dry run against the kinematic stand-in (no ROS, no flight):
    .venv/bin/python run_target_reaching_episode.py --standin --log /tmp/episode.jsonl

Per step it logs wall UTC and monotonic time, the simulation step index, the action, the observation's
target fields, the reward, and info. Privileged values appear only under "privileged" (evaluation only;
the controller never sees them). There is never an automatic retry; close() keeps the committed landing
gates (AUTO_LAND stream release, fresh landed-and-disarmed confirmation).
"""

from __future__ import annotations

import argparse
import datetime
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

import sar_observation as so
from baseline_controller import BaselineController

TARGET_FIELDS = ("det_now", "det_bearing_sin", "det_bearing_cos", "det_range_valid", "det_range",
                 "est_valid", "est_bearing_sin", "est_bearing_cos", "est_range", "est_age", "height")


def jsonable(value):
    if isinstance(value, dict):
        return {k: jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [jsonable(v) for v in value]
    if isinstance(value, (np.floating, float)):
        return float(value) if math.isfinite(value) else str(value)
    if isinstance(value, (np.integer, np.bool_)):
        return value.item()
    return value


def record(log, event, **fields):
    line = {"event": event, "wall_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "mono": time.monotonic(), **fields}
    log.write(json.dumps(jsonable(line)) + "\n")
    log.flush()
    print(json.dumps(jsonable({k: v for k, v in line.items() if k != "observation"})), flush=True)


def obs_fields(obs):
    return {name: float(obs[so.SLICES[name]][0]) for name in TARGET_FIELDS}


def run(env, log, max_steps):
    controller = BaselineController()
    record(log, "reset_start")
    obs, info = env.reset()
    record(log, "reset_done", fields=obs_fields(obs), info=info)
    result = None
    for step in range(max_steps):
        action = controller.act(obs)
        obs, reward, terminated, truncated, info = env.step(action)
        record(log, "step", step=step, action=action, reward=reward, terminated=terminated,
               truncated=truncated, fields=obs_fields(obs), min_lidar=float(np.min(obs[so.SLICES["lidar"]]) * 30),
               info=info)
        if terminated or truncated:
            result = info.get("reason")
            break
    record(log, "episode_end", reason=result)
    return result


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--log", type=Path, required=True)
    p.add_argument("--standin", action="store_true", help="offline kinematic stand-in instead of PX4 SITL")
    p.add_argument("--i-confirm-supervised-flight", action="store_true",
                   help="required for SITL: confirms the runbook preflight gates passed and a human is supervising")
    p.add_argument("--max-steps", type=int, default=600)
    args = p.parse_args(argv)
    if not args.standin and not args.i_confirm_supervised_flight:
        p.error("SITL flight requires --i-confirm-supervised-flight after the runbook gates pass")

    if args.standin:
        from sar_standin_env import SarStandInEnv
        env = SarStandInEnv()
    else:
        from px4_uav_env import PX4UAVEnv
        env = PX4UAVEnv()
    with args.log.open("x") as log:                 # never overwrite an existing log
        try:
            reason = run(env, log, args.max_steps)
        finally:
            record(log, "close_start")
            if hasattr(env, "close"):
                env.close()
            record(log, "close_returned")
    print(f"EPISODE RESULT: {reason}", flush=True)
    return 0 if reason == "success" else 1


if __name__ == "__main__":
    sys.exit(main())
