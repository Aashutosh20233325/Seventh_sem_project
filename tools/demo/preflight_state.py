"""Fail-closed pre-flight check on the passive guard observer's log, run after the observer has exited.
Usage: preflight_state.py <guard_observer.jsonl>
Exit 0 and print "ok ..." only if: the log has no "abort" record, and its last "state" record is within 5 s of the
log's last monotonic time, reports 0 /fmu/in publishers, a disarmed vehicle (arming_state 1, not failsafe) with
pre-flight checks passing, and landed. Anything else (missing, unreadable, stale, unexpected) prints "fail ..." and
exits 1.
"""
import json
import sys


def check(path):
    try:
        rows = [json.loads(line) for line in open(path) if line.strip()]
    except (OSError, ValueError) as exc:
        return False, f"unreadable log: {exc!r}"
    aborts = [r for r in rows if r.get("k") == "abort"]
    if aborts:
        return False, f"observer abort: {aborts[-1].get('reason')}"
    states = [r for r in rows if r.get("k") == "state"]
    if not states:
        return False, "no state record"
    s, t_last = states[-1], max(r["t"] for r in rows if isinstance(r.get("t"), (int, float)))
    if t_last - s["t"] > 5.0:
        return False, f"last state record {t_last - s['t']:.1f} s old"
    if s.get("fmu_in_publishers") != 0:
        return False, f"/fmu/in publishers {s.get('fmu_in_publishers')}"
    st, ld = s.get("status"), s.get("land")
    if not st or not ld:
        return False, f"status/land missing: {st} {ld}"
    arming, _nav, failsafe, preflight = st
    if arming != 1 or failsafe or not preflight:
        return False, f"vehicle state arming={arming} failsafe={failsafe} preflight={preflight}"
    if not ld[0]:
        return False, "vehicle not landed"
    return True, f"disarmed, landed, preflight ok, /fmu/in 0, mem {s.get('mem_available_mib', 0):.0f} MiB"


if __name__ == "__main__":
    ok, why = check(sys.argv[1]) if len(sys.argv) == 2 else (False, "usage")
    print(("ok " if ok else "fail ") + why)
    sys.exit(0 if ok else 1)
