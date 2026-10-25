"""RGB camera at 960x540 (approved 2026-10-25): the project camera model and detection equivalence with 1920x1080
on synthetic scenes (same target detected at the same distances, bearing and range nearly identical)."""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

import synthetic_scene as ss
import target_perception as tp

ROOT = Path(__file__).resolve().parent
PX4_OAKD = Path("/home/hp/PX4-Autopilot/Tools/simulation/gz/models/OakD-Lite/model.sdf")
K_LOW = ss.intrinsics(960, 540, 1.204)
TARGET = (3.0, -3.0, 0.055)


def sensor_sizes(path):
    model = ET.parse(path).getroot().find("model")
    return {s.get("name"): (int(s.findtext(".//width")), int(s.findtext(".//height"))) for s in model.iter("sensor")}


def test_project_camera_model_changes_only_the_rgb_resolution():
    ours = ROOT / "sim/models/OakD-Lite-960x540/model.sdf"
    assert sensor_sizes(ours) == {"IMX214": (960, 540), "StereoOV7251": (640, 480)}
    drone = (ROOT / "sim/models/x500_lidar_2d_oakd/model.sdf").read_text()
    assert "<uri>model://OakD-Lite-960x540</uri>" in drone and "<uri>model://OakD-Lite</uri>" not in drone
    if PX4_OAKD.exists():
        strip = lambda t: [l for l in t.splitlines() if "<width>" not in l and "<height>" not in l and l.strip()]  # noqa: E731
        ours_lines = [l for l in strip(ours.read_text()) if not l.startswith("<!--") and "960x540 instead" not in l
                      and "Everything else is unchanged" not in l]
        assert ours_lines == strip(PX4_OAKD.read_text()), "only the RGB width/height may differ from PX4's model"


@pytest.mark.parametrize("dist, yaw_off", [(2.0, 0.0), (4.0, 0.2), (6.0, -0.3), (8.0, 0.1), (10.0, 0.0), (12.0, 0.25)])
def test_detection_equivalent_at_both_resolutions(dist, yaw_off):
    bearing = math.atan2(TARGET[1], TARGET[0])                         # place the UAV `dist` m from the target
    uav = (TARGET[0] - dist * math.cos(bearing), TARGET[1] - dist * math.sin(bearing), -1.8)
    yaw = bearing + yaw_off
    hi = tp.detect(*ss.render(uav, yaw), ss.K_RGB, ss.K_DEPTH)
    rgb_lo, depth = ss.render(uav, yaw, rgb_size=(960, 540), K_rgb=K_LOW)
    lo = tp.detect(rgb_lo, depth, K_LOW, ss.K_DEPTH)
    assert lo.detected == hi.detected, (dist, yaw_off, hi, lo)
    if hi.detected:
        assert abs(math.degrees(lo.bearing - hi.bearing)) < 0.5
        if hi.range_valid:
            assert lo.range_valid and abs(lo.horizontal_range - hi.horizontal_range) < 0.1


def test_blob_threshold_scales_with_image_area():
    assert tp.MIN_BLOB_PX_FULLRES * (960 * 540) / (1920 * 1080) == pytest.approx(37.5)
    # an empty floor is never a detection at either size
    assert not tp.detect(*ss.render((0, 0, -1.8), 0.0, target=None, rgb_size=(960, 540), K_rgb=K_LOW), K_LOW, ss.K_DEPTH).detected
