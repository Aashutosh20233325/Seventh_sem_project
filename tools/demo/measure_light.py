"""Lightweight read-only window: NO image subscriptions.

Usage: measure_light.py <label> <seconds> <out.json> name=pid [name=pid ...]

Camera rates and the camera-clock simulation rate come from /uav/camera/{rgb,depth}/camera_info, which
Gazebo publishes with each rendered frame, using the frame's sensor (simulation) stamp. The only image
subscriber therefore stays the perception node, the same load as during a flight. The PX4 simulation
rate comes from local-position stamp increments of 0-200 ms.
"""
import json, subprocess, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleLandDetected

label, secs, out = sys.argv[1], float(sys.argv[2]), sys.argv[3]
pids = dict(a.split("=") for a in sys.argv[4:])
jiff = lambda p: (lambda f: int(f[11]) + int(f[12]))(open(f"/proc/{p}/stat").read().rsplit(")", 1)[1].split())

def mem():
    m = {l.split(":")[0]: int(l.split()[1]) for l in open("/proc/meminfo")}
    return {"ram_used_mib": (m["MemTotal"] - m["MemAvailable"]) // 1024, "swap_used_mib": (m["SwapTotal"] - m["SwapFree"]) // 1024}

def gpu():
    try:
        r = subprocess.run(["nvidia-smi", "--query-gpu=utilization.gpu,memory.used", "--format=csv,noheader,nounits"],
                           capture_output=True, text=True, timeout=10).stdout.strip().split(",")
        return float(r[0]), float(r[1])
    except Exception:
        return None, None

rclpy.init()
n = Node("perf_window_probe_light")
data = {k: [] for k in ("rgb", "depth", "perception", "lidar", "status", "land", "position")}
px4 = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE, depth=10)
st = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
n.create_subscription(CameraInfo, "/uav/camera/rgb/camera_info", lambda m: data["rgb"].append((time.monotonic(), st(m))), qos_profile_sensor_data)
n.create_subscription(CameraInfo, "/uav/camera/depth/camera_info", lambda m: data["depth"].append((time.monotonic(), st(m))), qos_profile_sensor_data)
n.create_subscription(Float64MultiArray, "/uav/target_perception", lambda m: data["perception"].append((time.monotonic(), list(m.data))), 10)
n.create_subscription(Float32MultiArray, "/uav/lidar_sectors", lambda m: data["lidar"].append((time.monotonic(), None)), 10)
n.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v4", lambda m: data["status"].append((time.monotonic(), (m.arming_state, m.nav_state, m.failsafe, m.pre_flight_checks_pass, m.gcs_connection_lost))), px4)
n.create_subscription(VehicleLandDetected, "/fmu/out/vehicle_land_detected", lambda m: data["land"].append((time.monotonic(), (m.landed, m.at_rest))), px4)
n.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", lambda m: data["position"].append((time.monotonic(), int(m.timestamp))), px4)
for _ in range(20):
    rclpy.spin_once(n, timeout_sec=0.1)
for k in data: data[k].clear()
j0 = {k: jiff(p) for k, p in pids.items()}
t0 = time.monotonic(); gpus = []; next_gpu = t0
while time.monotonic() < t0 + secs:
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.monotonic() >= next_gpu:
        gpus.append(gpu()); next_gpu += 8
t1 = time.monotonic(); wall = t1 - t0
res = {"label": label, "window_s": round(wall, 1), **mem()}
res["cpu_pct"] = {k: round(100.0 * (jiff(p) - j0[k]) / 100 / wall, 1) for k, p in pids.items()}
res["gpu_util_pct"] = [g[0] for g in gpus]; res["gpu_mem_mib_max"] = max((g[1] or 0 for g in gpus), default=None)
for k in data: res[f"{k}_hz"] = round(len(data[k]) / wall, 3)
rate = lambda s: round((s[-1][1] - s[0][1]) / (s[-1][0] - s[0][0]), 4) if len(s) > 2 else None
res["sim_rgb"] = rate(data["rgb"]); res["sim_depth"] = rate(data["depth"])
pos = data["position"]
res["sim_px4"] = round(sum(b[1] - a[1] for a, b in zip(pos, pos[1:]) if 0 < b[1] - a[1] <= 200_000) / 1e6 / (pos[-1][0] - pos[0][0]), 4) if len(pos) > 2 else None
for k in ("status", "land", "perception"):
    g = sorted(b[0] - a[0] for a, b in zip(data[k], data[k][1:]))
    res[f"{k}_gap_max"] = round(g[-1], 3) if g else None
res["last_status"] = data["status"][-1][1] if data["status"] else None
res["last_land"] = data["land"][-1][1] if data["land"] else None
json.dump(res, open(out, "w"), indent=1)
print(json.dumps(res))
n.destroy_node(); rclpy.shutdown()
