#!/usr/bin/env python3
"""Keep a session leader alive to own a dedicated simulator process group."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time


stop_requested = threading.Event()


def process_start_time(pid: int) -> int | None:
    try:
        stat = Path(f"/proc/{pid}/stat").read_text()
        return int(stat[stat.rfind(")") + 2 :].split()[19])
    except (OSError, ValueError, IndexError):
        return None


def request_stop(_signal: int, _frame: object) -> None:
    stop_requested.set()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--child-record", required=True, type=Path)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("a child command is required after --")

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    child = subprocess.Popen(command, stdin=subprocess.DEVNULL)
    record = {"pid": child.pid, "start_time": process_start_time(child.pid), "command": command}
    temporary = args.child_record.with_suffix(args.child_record.suffix + ".tmp")
    temporary.write_text(json.dumps(record) + "\n")
    temporary.replace(args.child_record)

    while child.poll() is None:
        time.sleep(0.1)
    # Do not release our process-group identity if a simulator descendant is
    # still alive after its direct client exits. The owned group can still be
    # stopped explicitly through this guard.
    if not stop_requested.is_set():
        stop_requested.wait()
    return child.returncode or 0


if __name__ == "__main__":
    raise SystemExit(main())
