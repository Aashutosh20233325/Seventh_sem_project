"""Stop-rule check after one series episode (read-only).
Usage: check_episode.py <run_log> <episode.jsonl or missing> <default|variant> <passive|ppo|baseline|explore> [offset_deg]
With an offset (Stage B), the start heading reached at reset_done must be within 15 deg of nominal east + offset, and
mode "baseline" requires that no policy was loaded.
Prints one line "<class> <detail>" and exits 0 only for class "ok" (flown, success, safe landing, nothing unexpected).
Classes: ok | readiness_fail (no flight; the series may continue unless it repeats) | stop (anything else: the
series must stop and be reported).
"""
import json
import math
import os
import sys

HEADING_TOLERANCE_DEG = 15.0

EXPECTED_FILTER = {None, "slow", "stop"}


def classify(run_log, episode, offset_deg=None, mode="ppo"):
    text = open(run_log, errors="replace").read() if os.path.exists(run_log) else ""
    if "stack stopped" not in text:
        return "stop", "stack not confirmed stopped"
    if "POLICY CHECK FAILED" in text:
        return "stop", "policy sha256 check failed"
    if "FLIGHT RUN: FAIL (readiness gates failed)" in text or "FLIGHT RUN: FAIL (readiness windows passed" in text:
        return "readiness_fail", text[text.index("FLIGHT RUN:"):].splitlines()[0]
    if "FLIGHT RUN: FLOWN" not in text:
        line = next((l for l in text.splitlines() if l.startswith("FLIGHT RUN:")), "no FLIGHT RUN line")
        return "stop", line
    if "PASS: fresh landed, at-rest, disarmed telemetry" not in text:
        return "stop", "gated stop never confirmed landed/disarmed"
    try:
        rows = [json.loads(l) for l in open(episode) if l.strip()]
    except (OSError, ValueError) as exc:
        return "stop", f"episode log unreadable: {exc!r}"
    kinds = [r.get("event") for r in rows]
    loaded = next((r for r in rows if r.get("event") == "policy_loaded"), None)
    if mode == "baseline":
        if loaded is not None:
            return "stop", "baseline mode but a policy was loaded"
    elif loaded is None or loaded.get("format") != "numpy" or loaded.get("torch_loaded") is not False:
        return "stop", f"policy not loaded as numpy without torch: {loaded and loaded.get('format')}"
    if offset_deg is not None:
        start = next((r for r in rows if r.get("event") == "reset_start"), {})
        done = next((r for r in rows if r.get("event") == "reset_done"), None)
        want = math.radians(offset_deg)
        if (start.get("options") or {}).get("start_yaw_offset") is None or \
                abs(start["options"]["start_yaw_offset"] - want) > 1e-9:
            return "stop", f"reset options {start.get('options')} do not match offset {offset_deg}"
        if done is not None:
            heading = done["info"]["privileged"].get("heading")
            expected = math.pi / 2 + want
            err = None if heading is None else abs(math.degrees(math.atan2(math.sin(heading - expected),
                                                                          math.cos(heading - expected))))
            if err is None or err > HEADING_TOLERANCE_DEG:
                return "stop", f"start heading error {err} deg (limit {HEADING_TOLERANCE_DEG})"
    for bad in ("guard_refuse", "guard_abort", "policy_load_failed"):
        if bad in kinds:
            return "stop", bad
    steps = [r for r in rows if r.get("event") == "step"]
    end = next((r for r in rows if r.get("event") == "episode_end"), None)
    if not steps or end is None or end.get("reason") != "success":
        return "stop", f"episode result {end and end.get('reason')}"
    if any(s["info"].get("collision") for s in steps):
        return "stop", "collision flag set"
    if mode == "explore":
        if not any(r.get("event") == "explorer" and r.get("inner") == "policy" for r in rows):
            return "stop", "explore mode but no explorer with the policy was set up"
        if not all("mode" in (s.get("policy") or {}) for s in steps):
            return "stop", "explore mode but steps without an explorer record"
    reasons = {filter_reason(s) for s in steps} if mode != "baseline" else set()
    if not reasons <= EXPECTED_FILTER:
        return "stop", f"unexpected filter reasons {sorted(map(str, reasons - EXPECTED_FILTER))}"
    main_kinds = [k for k in kinds if k != "guard_sample"]           # the guard thread may log after close()
    if main_kinds[-2:] != ["close_start", "close_returned"]:
        return "stop", "close() did not return"
    n_slow = sum(filter_reason(s) == "slow" for s in steps)
    n_stop = sum(filter_reason(s) == "stop" for s in steps)
    extra = ""
    if mode == "explore":
        modes = [(s.get("policy") or {}).get("mode") for s in steps]
        first_approach = next((k for k, m in enumerate(modes) if m == "APPROACH"), None)
        extra = f" explore_steps={modes.count('EXPLORE')} first_approach_step={first_approach}"
    return "ok", (f"success steps={len(steps)} slow={n_slow} stop={n_stop} "
                  f"min_lidar={min(s['min_lidar'] for s in steps):.2f}{extra}")


def filter_reason(step):
    """The LiDAR safety filter's reason for a step: PPO records it as policy.reason; the explorer as policy.filter,
    or, while PPO flies the approach, inside policy.inner."""
    p = step.get("policy") or {}
    if "mode" not in p:
        return p.get("reason")
    if p.get("via") == "inner" or (p.get("mode") == "APPROACH" and "inner" in p):
        return (p.get("inner") or {}).get("reason")
    return p.get("filter")




# ---------- Stage C wrapper ----------

def classify_stage_c(run_log, episode, variant, mode, offset_deg=None):
    """Passive: PASSIVE outcome, both world gates passed for this world, stack stopped. Flight: every check above
    (start heading when offset_deg is given) plus the runner's "world" record naming this world and its target.
    variant "default" means the default world (sim/worlds/sar_walls.sdf, no SAR_WORLD_VARIANT)."""
    sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[2]))   # repo root
    import sar_task
    text = open(run_log, errors="replace").read() if os.path.exists(run_log) else ""
    default = variant == "default"
    gate_target = ("PASS: live Gazebo target matches sim/worlds/sar_walls.sdf" if default
                   else f"PASS: live Gazebo target matches sim/worlds/variants/{variant}/sar_walls.sdf")
    gate_consistency = "PASS: world variant None" if default else f"PASS: world variant '{variant}'"
    if gate_target not in text or gate_consistency not in text:
        if "FLIGHT RUN: FAIL (readiness" in text:
            return "readiness_fail", text[text.index("FLIGHT RUN:"):].splitlines()[0]
        return "stop", "world gates did not both pass for this variant"
    if mode == "passive":
        if "stack stopped" not in text:
            return "stop", "stack not confirmed stopped"
        if "FLIGHT RUN: PASSIVE" not in text:
            line = next((l for l in text.splitlines() if l.startswith("FLIGHT RUN:")), "no FLIGHT RUN line")
            return ("readiness_fail" if "FAIL (readiness" in line else "stop"), line
        if os.path.exists(episode):
            return "stop", "passive run produced an episode log"
        return "ok", "passive: readiness, world-target and world-consistency gates passed; no flight"
    cls, detail = classify(run_log, episode, offset_deg, mode)
    if cls != "ok":
        return cls, detail
    rows = [json.loads(l) for l in open(episode) if l.strip()]
    world = next((r for r in rows if r.get("event") == "world"), None)
    target = sar_task.load_target(sar_task.WORLD_SDF if default else sar_task.world_variant_sdf(variant))
    want = None if default else variant
    if world is None or world.get("variant") != want or world.get("target_ned") != [target.x_ned, target.y_ned]:
        return "stop", f"runner world record {world and {k: world.get(k) for k in ('variant', 'target_ned')}} != {variant}"
    return cls, detail + f" variant={variant}"


if __name__ == "__main__":
    if len(sys.argv) not in (5, 6):
        print("usage: check_episode.py <run_log> <episode.jsonl> <default|variant> <passive|ppo|baseline|explore> [offset_deg]")
        sys.exit(2)
    offset = float(sys.argv[5]) if len(sys.argv) == 6 and float(sys.argv[5]) != 0 else None
    cls, detail = classify_stage_c(*sys.argv[1:5], offset)
    print(cls, detail)
    sys.exit(0 if cls == "ok" else 1)
