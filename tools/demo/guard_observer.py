"""PASSIVE no-flight validation of the uncommitted B3 RuntimeGuard (subscriptions only; no publishers; no PX4UAVEnv).

Live inputs feed the REAL PX4ControllerNode callbacks (position, timesync, perception) on a bare, publisher-free
node instance; the REAL RuntimeGuard reads them through the REAL px4_telemetry adapter.

Guards (all fed from the same live data; only "nominal" sees it unmodified):
  nominal    unmodified live inputs
  hrt_drop   from DROP_AT s: PX4 HRT withheld (link and camera continue)      -> expect px4_clock_stalled/stale
  cam_drop   from DROP_AT s: camera (perception) withheld                       -> expect camera_clock_stale
  link_drop  from DROP_AT s: PX4 position receipt (link) withheld               -> expect px4_telemetry_stale
  hrt_dup    from DROP_AT s: real HRT withheld; the last HRT value re-delivered with a new receipt time every 0.5 s
                                                                                -> expect px4_clock_stalled/stale
The shadow guards are synthetic dropouts applied inside this harness; the live system is untouched.

Records: raw position (receipt, DDS stamp), timesync (receipt, timestamp, estimated_offset, HRT), perception
(receipt, image stamp); 1 Hz verdicts of every guard; 1 Hz state (memory, swap, /fmu/in publishers, vehicle).
Stops early (exit 3, "abort" record): /fmu/in publisher; vehicle armed or not landed; telemetry lost > 10 s;
MemAvailable < 1.2 GiB; swap growth > 64 MiB.
Usage: guard_observer.py <out.jsonl> <duration_s> <drop_at_s>
"""
import json, sys, time, threading
from types import SimpleNamespace
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float64MultiArray
from px4_msgs.msg import TimesyncStatus, VehicleLocalPosition, VehicleStatus, VehicleLandDetected

sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[2]))   # repo root
import px4_uav_env
import run_target_reaching_episode as runner

out_path, DURATION, DROP_AT = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
MEM_FLOOR_MIB, SWAP_GROWTH_MIB, TELEMETRY_LOSS_S = 1.2 * 1024, 64.0, 10.0
FMU_IN = ("/fmu/in/offboard_control_mode", "/fmu/in/trajectory_setpoint", "/fmu/in/vehicle_command")
out = open(out_path, "x")
def emit(kind, **f):
    out.write(json.dumps({"k": kind, **f}) + "\n")

def meminfo():
    m = {l.split(":")[0]: int(l.split()[1]) for l in open("/proc/meminfo")}
    return m["MemAvailable"] / 1024.0, (m["SwapTotal"] - m["SwapFree"]) / 1024.0

# Publisher-free node state driven by the real callbacks.
C = px4_uav_env.PX4ControllerNode
node = C.__new__(C)
node.condition = threading.Condition()
for a in ("x", "y", "z", "heading", "vx", "vy"):
    setattr(node, a, 0.0)
node.heading_valid = node.position_valid = False
node.position_received_at = None
node.sim_clock_us = None
node._last_position_stamp = None
node.px4_sim_time_us = None
node.px4_sim_time_received_at = None
node.latest_perception = None
node.perception_received_at = None
C.reset_camera_clock(node)
real = runner.px4_telemetry(SimpleNamespace(node=node))

t0 = time.monotonic()
dup = {"value": None, "next": None}

def shadow(kind):
    def read():
        v = real()
        if time.monotonic() - t0 < DROP_AT:
            return v
        if kind == "hrt_drop":
            v["px4"] = shadow.frozen.get("hrt_drop", v["px4"])
        elif kind == "cam_drop":
            v["camera"] = shadow.frozen.get("cam_drop", v["camera"])
        elif kind == "link_drop":
            v["px4_link"] = shadow.frozen.get("link_drop", v["px4_link"])
        elif kind == "hrt_dup":
            base = shadow.frozen.get("hrt_dup", v["px4"])
            now = time.monotonic()
            if base is not None:
                if dup["next"] is None or now >= dup["next"]:
                    dup["value"] = (base[0], now)                     # same HRT, new receipt time
                    dup["next"] = now + 0.5
                v["px4"] = dup["value"]
        return v
    return read
shadow.frozen = {}

def freeze_at_drop():
    v = real()
    shadow.frozen = {"hrt_drop": v["px4"], "cam_drop": v["camera"], "link_drop": v["px4_link"], "hrt_dup": v["px4"]}

clock = time.monotonic
guards = {"nominal": runner.RuntimeGuard(real, clock=clock)}
for kind in ("hrt_drop", "cam_drop", "link_drop", "hrt_dup"):
    guards[kind] = runner.RuntimeGuard(shadow(kind), clock=clock)

rclpy.init()
ros = Node("passive_guard_observer")
px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)
latest = {"status": None, "status_at": None, "land": None, "land_at": None, "pos_at": None, "ts_at": None, "perc_at": None}

def on_position(m):
    C.position_callback(node, m)
    latest["pos_at"] = node.position_received_at
    emit("pos", t=node.position_received_at, ts=int(m.timestamp))

def on_timesync(m):
    C.timesync_status_callback(node, m)
    latest["ts_at"] = node.px4_sim_time_received_at
    emit("ts", t=node.px4_sim_time_received_at, ts=int(m.timestamp), off=int(m.estimated_offset), hrt=node.px4_sim_time_us)

def on_perception(m):
    C.perception_callback(node, m)
    if node.perception_received_at is not None and node.perception_received_at != latest["perc_at"]:
        latest["perc_at"] = node.perception_received_at
        emit("perc", t=node.perception_received_at, s=float(node.latest_perception["image_stamp"]))

def on_status(m):
    latest["status"] = (int(m.arming_state), int(m.nav_state), bool(m.failsafe), bool(m.pre_flight_checks_pass))
    latest["status_at"] = time.monotonic()

def on_land(m):
    latest["land"] = (bool(m.landed), bool(m.at_rest)); latest["land_at"] = time.monotonic()

ros.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", on_position, px4_qos)
ros.create_subscription(TimesyncStatus, "/fmu/out/timesync_status", on_timesync, px4_qos)
ros.create_subscription(Float64MultiArray, px4_uav_env.TARGET_PERCEPTION_TOPIC, on_perception, 10)
ros.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v4", on_status, px4_qos)
ros.create_subscription(VehicleLandDetected, "/fmu/out/vehicle_land_detected", on_land, px4_qos)

mem0, swap0 = meminfo()
emit("start", t=t0, wall_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), duration=DURATION, drop_at=DROP_AT,
     mem_available_mib=mem0, swap_used_mib=swap0,
     guard_thresholds=guards["nominal"].check()["thresholds"])
abort, dropped, next_tick = None, False, t0 + 1.0
while True:
    rclpy.spin_once(ros, timeout_sec=0.02)
    now = time.monotonic(); el = now - t0
    if not dropped and el >= DROP_AT:
        freeze_at_drop(); dropped = True
        emit("drop", t=now, frozen={k: v for k, v in shadow.frozen.items()})
    for g in guards.values():                                        # sample often enough for 0.5 s duplicates
        g.sample()
    if now >= next_tick:
        next_tick += 1.0
        verdicts = {}
        for name, g in guards.items():
            v = g.check()
            verdicts[name] = {k: v[k] for k in ("ok", "reason", "speed", "px4_rate", "px4_status", "px4_link_status",
                                               "camera_rate", "camera_status", "mem_available_mib")}
            verdicts[name]["px4_samples"] = len(g.samples["px4"])
        emit("guard", t=now, v=verdicts)
        avail, swap = meminfo()
        fmu_in = sum(ros.count_publishers(t) for t in FMU_IN)
        emit("state", t=now, mem_available_mib=avail, swap_used_mib=swap, fmu_in_publishers=fmu_in,
             status=latest["status"], land=latest["land"])
        st, ld = latest["status"], latest["land"]
        lost = [k for k in ("status_at", "land_at", "pos_at", "ts_at", "perc_at")
                if latest[k] is None and el > TELEMETRY_LOSS_S or latest[k] is not None and now - latest[k] > TELEMETRY_LOSS_S]
        if fmu_in > 0:
            abort = f"/fmu/in publisher count {fmu_in}"
        elif st is not None and now - latest["status_at"] <= 2.0 and st[0] != 1:
            abort = f"vehicle not disarmed (arming_state={st[0]})"
        elif ld is not None and now - latest["land_at"] <= 4.0 and not ld[0]:
            abort = "vehicle not landed"
        elif lost:
            abort = f"telemetry lost > {TELEMETRY_LOSS_S} s: {lost}"
        elif avail < MEM_FLOOR_MIB:
            abort = f"MemAvailable {avail:.0f} MiB < {MEM_FLOOR_MIB:.0f}"
        elif swap - swap0 > SWAP_GROWTH_MIB:
            abort = f"swap grew {swap - swap0:.0f} MiB"
        if abort:
            emit("abort", t=now, reason=abort)
            break
    if el >= DURATION:
        break
emit("end", t=time.monotonic(), aborted=abort)
out.close()
ros.destroy_node(); rclpy.shutdown()
print("ABORT: " + abort if abort else "observer completed", flush=True)
sys.exit(3 if abort else 0)
