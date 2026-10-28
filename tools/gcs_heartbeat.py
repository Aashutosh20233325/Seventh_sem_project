#!/usr/bin/env python3
"""Headless ground-station heartbeat: replaces QGroundControl for the demo (QGC drew its window on the CPU, ~1-1.5
cores, and pulled the simulation under the speed floor).

PX4 (NAV_DLL_ACT=2, unchanged) refuses to arm without a GCS and fails safe if the GCS link is lost. This sends exactly
what QGC sends for that: a MAVLink 2 HEARTBEAT (type GCS, sysid 255, compid 190) once a second to PX4's GCS link, from
the UDP port QGC uses (14550, which PX4 streams to). Nothing else: no commands, no parameters, no mission. The incoming
telemetry stream is never read (the kernel drops it once the small receive buffer is full: no CPU).

Usage: .venv/bin/python tools/gcs_heartbeat.py [--px4-port 18571] [--local-port 14550] [--rate 1.0]
"""

from __future__ import annotations

import argparse
import signal
import socket
import threading

from pymavlink.dialects.v20 import common as mavlink

GCS_SYSID, GCS_COMPID = 255, mavlink.MAV_COMP_ID_MISSIONPLANNER     # what QGroundControl uses
PX4_GCS_PORT = 18571                                                   # 18570 + PX4 instance 1 (tools/isolated_sitl.py)
LOCAL_PORT = 14550                                                     # the remote port PX4 streams to


def heartbeat_bytes(mav: mavlink.MAVLink) -> bytes:
    return mav.heartbeat_encode(mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                mavlink.MAV_STATE_ACTIVE).pack(mav)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--px4-port", type=int, default=PX4_GCS_PORT)
    ap.add_argument("--local-port", type=int, default=LOCAL_PORT)
    ap.add_argument("--rate", type=float, default=1.0, help="heartbeats per wall-clock second (default 1)")
    a = ap.parse_args(argv)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
    sock.bind(("127.0.0.1", a.local_port))                             # fails if QGroundControl holds the port
    mav = mavlink.MAVLink(None, srcSystem=GCS_SYSID, srcComponent=GCS_COMPID)
    print(f"gcs_heartbeat: 127.0.0.1:{a.local_port} -> 127.0.0.1:{a.px4_port} at {a.rate} Hz", flush=True)
    sent = 0
    while not stop.is_set():
        try:
            sock.sendto(heartbeat_bytes(mav), ("127.0.0.1", a.px4_port))
            sent += 1
        except OSError:                                                # PX4 not up yet (ICMP unreachable): keep going
            pass
        stop.wait(1.0 / a.rate)
    sock.close()
    print(f"gcs_heartbeat: stopped after {sent} heartbeats", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
