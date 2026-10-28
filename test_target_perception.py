import ast
import math
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

import synthetic_scene as ss
import target_perception as tp

TARGET = np.array([3.0, -3.0, 0.055])
HOVER_ROOT_Z = 1.615          # measured hover model-root height (ULog, z=-1.64 m local)
FIXTURES = Path(__file__).with_name("fixtures") / "oakd_stand"
K_HALF = np.array([[698.6117935180664, 0, 480.0], [0, 698.6117649078369, 270.0], [0, 0, 1.0]])


def uav_at(distance, bearing_deg, yaw=0.3, z=HOVER_ROOT_Z):
    world = yaw + math.radians(bearing_deg)
    return np.array([TARGET[0] - distance * math.cos(world), TARGET[1] - distance * math.sin(world), z]), yaw


@pytest.mark.parametrize("distance", [3.0, 4.31, 6.0, 8.0])
@pytest.mark.parametrize("bearing", [0.0, 20.0, -20.0])
def test_full_view_detection_bearing_and_range(distance, bearing):
    uav, yaw = uav_at(distance, bearing)
    rgb, depth = ss.render(uav, yaw)

    det = tp.detect(rgb, depth, ss.K_RGB, ss.K_DEPTH)

    true_bearing, true_range = ss.truth_in_body(uav, yaw, TARGET)
    assert det.detected and det.range_valid and det.status == tp.STATUS_DETECTED
    assert abs(math.degrees(det.bearing - true_bearing)) < 0.5
    assert abs(det.horizontal_range - true_range) / true_range < 0.02
    assert 0.5 <= det.confidence <= 1.0


def test_target_cut_by_image_edge_is_flagged_and_still_located():
    uav, yaw = uav_at(2.0, 20.0)
    rgb, depth = ss.render(uav, yaw)

    det = tp.detect(rgb, depth, ss.K_RGB, ss.K_DEPTH)

    true_bearing, true_range = ss.truth_in_body(uav, yaw, TARGET)
    assert det.detected and det.touches_edge and det.range_valid
    assert det.confidence < 1.0
    assert abs(math.degrees(det.bearing - true_bearing)) < 1.0
    assert abs(det.horizontal_range - true_range) / true_range < 0.05


@pytest.mark.parametrize("pose", [(3.0, 0.0), (6.0, 20.0), (1.0, 0.0), (4.31, 90.0), (4.31, 180.0)])
def test_no_detection_without_visible_target(pose):
    uav, yaw = uav_at(*pose)
    absent = ss.render(uav, yaw, target=None)
    assert not tp.detect(*absent, ss.K_RGB, ss.K_DEPTH).detected
    if pose[0] == 1.0 or abs(pose[1]) >= 90:     # present but outside the camera view
        assert not tp.detect(*ss.render(uav, yaw), ss.K_RGB, ss.K_DEPTH).detected


def test_missing_depth_reports_bearing_without_range():
    uav, yaw = uav_at(4.31, 0.0)
    rgb, _ = ss.render(uav, yaw)

    det = tp.detect(rgb, None, ss.K_RGB, ss.K_DEPTH)

    assert det.detected and not det.range_valid
    assert det.status == tp.STATUS_DEPTH_INVALID and det.bearing_source == "centroid"
    assert math.isnan(det.horizontal_range)
    assert abs(math.degrees(det.bearing - ss.truth_in_body(uav, yaw, TARGET)[0])) < 1.0


@pytest.mark.parametrize("fill", [np.nan, np.inf, -np.inf, 0.05, 25.0])
def test_invalid_depth_values_never_produce_a_range(fill):
    uav, yaw = uav_at(4.31, 0.0)
    rgb, depth = ss.render(uav, yaw)

    det = tp.detect(rgb, np.full_like(depth, fill), ss.K_RGB, ss.K_DEPTH)

    assert det.detected and not det.range_valid and det.status == tp.STATUS_DEPTH_INVALID


def test_invalid_intrinsics_and_images_are_rejected():
    uav, yaw = uav_at(4.31, 0.0)
    rgb, depth = ss.render(uav, yaw)
    bad_K = ss.K_RGB.copy()
    bad_K[0, 0] = np.nan

    assert tp.detect(rgb, depth, bad_K, ss.K_DEPTH).status == tp.STATUS_INPUT_INVALID
    assert tp.detect(rgb[..., 0], depth, ss.K_RGB, ss.K_DEPTH).status == tp.STATUS_INPUT_INVALID
    assert not tp.detect(rgb, depth, ss.K_RGB, np.zeros((3, 3))).range_valid


def test_small_red_speck_is_not_a_target():
    rgb = np.full((1080, 1920, 3), 190, np.uint8)
    rgb[500:508, 900:908] = (220, 10, 10)          # 64 px < minimum blob size
    assert tp.detect(rgb, None, ss.K_RGB, ss.K_DEPTH).status == tp.STATUS_NO_TARGET


@pytest.mark.parametrize("name,truth_deg,edge", [
    ("stand_d4.31_b0.png", 0.401, False),
    ("stand_d8_b0.png", 0.215, False),
    ("stand_d2_b20_edge.png", 21.008, True),
])
def test_recorded_gazebo_frames(name, truth_deg, edge):
    rgb = np.asarray(Image.open(FIXTURES / name).convert("RGB"))

    det = tp.detect(rgb, None, K_HALF, None)

    # Recorded stand: the bearing is camera-centred and the RGB-only path ignores the mount translation.
    assert det.detected and det.touches_edge == edge and not det.range_valid
    assert abs(math.degrees(det.bearing) - truth_deg) < (1.0 if edge else 0.2)


# Rendered colours measured in Gazebo frames (concept scenes, 2026-10-28): jacket in tunnel lamp light and in sunlight,
# then things that must never count as a target.
HIVIS_JACKET = [(113, 121, 44), (154, 156, 54), (100, 106, 38)]
NOT_TARGETS = {"yellow supply case": (130, 115, 36), "caution tape": (250, 205, 20), "warning sign": (230, 190, 30),
               "foliage / tarp": (53, 71, 33), "tree canopy": (51, 102, 38), "sand": (150, 134, 112),
               "concrete": (150, 146, 138), "dark rock": (78, 70, 62), "dim jacket (too dark)": (50, 55, 18)}


def test_hivis_mask_accepts_jackets_and_rejects_lookalikes():
    for c in HIVIS_JACKET:
        assert tp.hivis_mask(np.array([[c]], np.uint8))[0, 0], c
    for name, c in NOT_TARGETS.items():
        px = np.array([[c]], np.uint8)
        assert not tp.hivis_mask(px)[0, 0] and not tp.target_mask(px)[0, 0], name


@pytest.mark.parametrize("distance", [3.0, 6.0, 9.0])
@pytest.mark.parametrize("bearing", [0.0, 20.0])
def test_hivis_target_detected_with_bearing_and_range(distance, bearing):
    uav, yaw = uav_at(distance, bearing)
    rgb, depth = ss.render(uav, yaw, color="hivis")
    assert tp.red_mask(rgb).sum() == 0 and tp.hivis_mask(rgb).sum() > 0   # found by the hi-vis rule alone
    det = tp.detect(rgb, depth, ss.K_RGB, ss.K_DEPTH)
    true_bearing, true_range = ss.truth_in_body(uav, yaw, TARGET)
    assert det.detected and det.range_valid
    assert abs(math.degrees(det.bearing - true_bearing)) < 0.5
    assert abs(det.horizontal_range - true_range) / true_range < 0.02


def test_large_yellow_and_sand_areas_are_not_targets():
    for name in ("yellow supply case", "caution tape", "sand"):
        rgb = np.zeros((540, 960, 3), np.uint8)
        rgb[200:340, 380:580] = NOT_TARGETS[name]
        depth = np.full((480, 640), 5.0, np.float32)
        assert not tp.detect(rgb, depth, ss.intrinsics(960, 540, 1.204), ss.K_DEPTH).detected, name


def test_message_round_trip_and_rejection():
    uav, yaw = uav_at(4.31, 0.0)
    det = tp.detect(*ss.render(uav, yaw), ss.K_RGB, ss.K_DEPTH)
    values = tp.encode(det, 12.5, 12.5, 30.0)

    msg = tp.decode(values)

    assert msg["detected"] and msg["range_valid"] and msg["image_stamp"] == 12.5
    assert msg["bearing"] == pytest.approx(det.bearing)
    assert tp.decode(values[:-1]) is None
    assert tp.decode([2.0] + values[1:]) is None
    stale = list(values)
    stale[1] = math.nan
    assert tp.decode(stale) is None
    broken = list(values)
    broken[5] = math.nan
    assert tp.decode(broken) is None
    no_target = tp.decode(tp.encode(tp.Detection(False, tp.STATUS_NO_TARGET), 1.0, 1.0, 1.0))
    assert not no_target["detected"] and not no_target["range_valid"]


def test_perception_module_has_no_simulator_or_ground_truth_access():
    source = Path(tp.__file__).read_text()
    imported = set()
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.Import):
            imported |= {a.name.split(".")[0] for a in node.names}
        elif isinstance(node, ast.ImportFrom):
            imported.add(node.module.split(".")[0])
    assert imported <= {"__future__", "math", "dataclasses", "numpy", "scipy"}
    for forbidden in ("pose/info", "gz.", "rclpy", "TARGET_X", "3.0, -3.0", "sar_task"):
        assert forbidden not in source
