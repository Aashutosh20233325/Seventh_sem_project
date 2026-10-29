#!/usr/bin/env python3
"""Plain-language progress for tools/demo/present_mine.sh (read-only: follows the series log and the episode log).

Usage: present_progress.py <series dir> <series output file>
Prints one line per stage (readiness windows, takeoff, search progress every 100 steps, miner found, success, supply
packet, landing) and exits when the series output file reports SERIES END / SERIES STOP.
"""

from __future__ import annotations

import glob
import json
import os
import re
import sys
import time

COMPASS = ((0, "north"), (45, "north-east"), (90, "east"), (135, "south-east"), (180, "south"), (-45, "north-west"),
           (-90, "west"), (-135, "south-west"), (-180, "south"))


def where(x, y):
    """NED (x north, y east) as words relative to the borehole."""
    import math
    d = math.hypot(x, y)
    if d < 1.5:
        return "at the borehole"
    a = math.degrees(math.atan2(y, x))
    name = min(COMPASS, key=lambda c: abs(c[0] - a))[1]
    return f"{d:4.1f} m {name} of the borehole"


class Follower:
    def __init__(self, path):
        self.path, self.pos, self.buf = path, 0, ""

    def lines(self):
        try:
            with open(self.path, errors="replace") as f:
                f.seek(self.pos)
                data = f.read()
                self.pos = f.tell()
        except OSError:
            return []
        self.buf += data
        *out, self.buf = self.buf.split("\n")
        return out


def say(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def main(series_dir, series_out):
    run_logs, episode, followers = set(), None, {}
    out = Follower(series_out)
    seen_mode, found_said, last_hundred = None, False, -1
    while True:
        for p in sorted(glob.glob(os.path.join(series_dir, "*.log"))):
            if p not in run_logs:
                run_logs.add(p)
                followers[p] = Follower(p)
                if p.endswith("r.log"):
                    say("Readiness failed once: starting a fresh simulator for the one allowed retry.")
        for p, f in followers.items():
            for line in f.lines():
                m = re.match(r"(w[1-6]): sim ([0-9.]+).*-> (\w+)", line)
                if line.startswith("== passive guard observer"):
                    say("Simulator is up. Readiness checks: six 25-second stability windows (about 3 minutes).")
                elif m:
                    say(f"  readiness window {m[1][1]}/6: simulation speed {float(m[2]):.2f}x -> {m[3]}")
                elif line.startswith("== T_r"):
                    say("Readiness passed (rehearsal: no takeoff)." if "presentr_" in p else
                        "*** READINESS PASSED. TAKEOFF IN ABOUT 30 SECONDS: START RECORDING NOW. ***")
                elif line.startswith("preflight state"):
                    say("Pre-flight check: " + ("ok" if " ok " in line else line))
                elif line.startswith("== SUPERVISED DEMO FLIGHT"):
                    say("*** TAKING OFF ***")
                elif line.startswith("FLIGHT RUN: PASSIVE"):
                    say("Rehearsal complete: everything passed, no takeoff (rehearsal mode).")
                elif line.startswith("FLIGHT RUN"):
                    say("Flight finished: " + line.split("EPISODE RESULT: ")[-1].rstrip(")"))
                elif line.startswith("SUPPLY PACKET DROPPED"):
                    say("Supply packet dropped beside the miner.")
                elif line.startswith("== gated stop"):
                    say("Shutting the simulator down (about 1-2 minutes)." if "presentr_" in p else
                        "Landed. Shutting the simulator down (about 1-2 minutes).")
                elif "ABORT" in line or "aborted" in line or "Traceback" in line:
                    say("Note: " + line[:160])
        if episode is None:
            hits = sorted(glob.glob(os.path.join(os.path.dirname(series_dir), "restart_" + os.path.basename(series_dir)
                                                 .removeprefix("restart_series_") + "_1_*", "logs", "episode.jsonl")))
            if hits:
                episode = Follower(hits[-1])
        if episode is not None:
            for line in episode.lines():
                try:
                    r = json.loads(line)
                except ValueError:
                    continue
                e = r.get("event")
                if e == "reset_done":
                    say("Airborne at search height. The search begins.")
                elif e == "guard_abort":
                    say(f"Safety guard stopped the flight ({r.get('reason')}); the drone lands where it is.")
                elif e == "step":
                    q, p = r["info"]["privileged"], r.get("policy") or {}
                    mode = p.get("mode")
                    if mode == "APPROACH" and not found_said:
                        found_said = True
                        say(f"*** MINER FOUND at {r['step'] * 0.1:.1f} s, drone {where(q['x'], q['y'])}: approaching ***")
                    elif r["step"] // 100 > last_hundred and mode == "EXPLORE":
                        last_hundred = r["step"] // 100
                        why = {"no_goal": " (looking around)", "return_to_start": " (search finished: returning)"}
                        say(f"  searching, {r['step'] * 0.1:5.1f} s: drone {where(q['x'], q['y'])}"
                            f"{why.get(p.get('reason'), '')}")
                    if p.get("reason") == "search_complete" and seen_mode != "done":
                        seen_mode = "done"
                        say("Search complete without finding the miner: back at the borehole.")
                    if r.get("terminated") and r["info"].get("reason") == "success":
                        say(f"*** MINER REACHED at {r['step'] * 0.1:.1f} s ***")
                elif e == "episode_end":
                    say(f"Episode over: {r.get('reason')}")
        for line in out.lines():
            if line.startswith("SERIES END") or line.startswith("SERIES STOP"):
                for f in followers.values():                     # flush what is left
                    f.lines()
                say("Run finished." if line.startswith("SERIES END") else f"Run stopped: {line}")
                return 0 if line.startswith("SERIES END") else 1
            if line.startswith("RESULT"):
                say("Result: " + line.split(": ", 1)[1].split(" steps=")[0])
        time.sleep(1)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1], sys.argv[2]))
