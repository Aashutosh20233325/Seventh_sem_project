"""Headless GCS heartbeat (replaces QGroundControl): sends exactly a MAVLink 2 GCS HEARTBEAT, nothing else."""
import socket
import threading

from pymavlink.dialects.v20 import common as mavlink

import tools.gcs_heartbeat as gh


def test_packet_is_a_gcs_heartbeat_from_the_qgc_ids():
    mav = mavlink.MAVLink(None, srcSystem=gh.GCS_SYSID, srcComponent=gh.GCS_COMPID)
    msg = mavlink.MAVLink(None).parse_buffer(gh.heartbeat_bytes(mav))[0]
    assert msg.get_type() == "HEARTBEAT"
    assert msg.type == mavlink.MAV_TYPE_GCS and msg.autopilot == mavlink.MAV_AUTOPILOT_INVALID
    assert (msg.get_srcSystem(), msg.get_srcComponent()) == (255, mavlink.MAV_COMP_ID_MISSIONPLANNER)


def test_sends_heartbeats_to_the_px4_port_and_nothing_else(monkeypatch):
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", 0))
    rx.settimeout(3)
    port = rx.getsockname()[1]
    stopper = {}
    real_event = threading.Event

    def event():
        e = real_event()
        stopper["e"] = e
        return e
    monkeypatch.setattr(gh.threading, "Event", event)
    monkeypatch.setattr(gh.signal, "signal", lambda *a: None)
    t = threading.Thread(target=gh.main, args=(["--px4-port", str(port), "--local-port", "0", "--rate", "20"],))
    t.start()
    try:
        parser = mavlink.MAVLink(None)
        types = set()
        for _ in range(5):
            data, src = rx.recvfrom(1024)
            types |= {m.get_type() for m in parser.parse_buffer(data)}
        assert types == {"HEARTBEAT"}
    finally:
        stopper["e"].set()
        t.join(5)
        rx.close()
    assert not t.is_alive()
