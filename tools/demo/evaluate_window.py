"""Apply the established stability-window gates to a measure_light.py JSON. Exit 0 = PASS."""
import json, sys
d = json.load(open(sys.argv[1]))
st, land = d.get("last_status"), d.get("last_land")
checks = {
    "sim_rgb>=0.35": (d["sim_rgb"] or 0) >= 0.35, "sim_depth>=0.35": (d["sim_depth"] or 0) >= 0.35,
    "sim_px4>=0.35": (d["sim_px4"] or 0) >= 0.35,
    "status_gap<=2.0": d["status_gap_max"] is not None and d["status_gap_max"] <= 2.0,
    "land_gap<=4.0": d["land_gap_max"] is not None and d["land_gap_max"] <= 4.0,
    # 2.5 s (was 1.5; approved 2026-10-28, same as run_target_reaching_episode.CAMERA_MAX_AGE_S)
    "perception_gap<=2.5": d["perception_gap_max"] is not None and d["perception_gap_max"] <= 2.5,
    "camera/depth/perception flowing": d["rgb_hz"] > 0 and d["depth_hz"] > 0 and d["perception_hz"] > 0,
    "lidar>=5Hz": d["lidar_hz"] >= 5,
    "disarmed": st is not None and st[0] == 1, "hold(nav 4)": st is not None and st[1] == 4,
    "no failsafe": st is not None and st[2] is False, "preflight": st is not None and st[3] is True,
    "gcs connected": st is not None and st[4] is False,
    "landed/at_rest": land is not None and land[0] is True and land[1] is True,
}
fails = [k for k, ok in checks.items() if not ok]
print(f"sim {d['sim_rgb']}/{d['sim_depth']}/{d['sim_px4']} | status_gap {d['status_gap_max']} land_gap {d['land_gap_max']} "
      f"perc_gap {d['perception_gap_max']} | rgb {d['rgb_hz']} depth {d['depth_hz']} perc {d['perception_hz']} lidar {d['lidar_hz']} "
      f"| ram {d['ram_used_mib']} swap {d['swap_used_mib']} -> {'PASS' if not fails else 'FAIL ' + ','.join(fails)}")
sys.exit(1 if fails else 0)
