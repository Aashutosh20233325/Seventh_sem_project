"""Static checks of the versioned simulation assets under sim/."""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

SIM = Path(__file__).with_name("sim")


def _pose(text):
    return [float(v) for v in text.split()]


def test_world_contains_red_target_sphere_with_audited_geometry():
    world = ET.parse(SIM / "worlds/sar_walls.sdf").getroot().find("world")
    assert world.get("name") == "sar_walls"
    target = [m for m in world.findall("model") if m.get("name") == "target"]
    assert len(target) == 1
    target = target[0]
    assert target.findtext("static").strip() == "true"
    assert _pose(target.findtext("pose")) == [3, -3, 0.055, 0, 0, 0]
    radii = [float(r.text) for r in target.iter("radius")]
    assert radii == [0.5, 0.5]                          # collision and visual
    assert _pose(target.find(".//visual/material/diffuse").text)[:3] == [1, 0, 0]
    assert world.findtext("gravity").split() == ["0", "0", "-9.8"]
    assert world.find("physics").findtext("real_time_factor") == "1.0"


def test_uav_model_keeps_stock_lidar_mount_and_adds_pitched_oakd():
    model = ET.parse(SIM / "models/x500_lidar_2d_oakd/model.sdf").getroot().find("model")
    assert model.get("name") == "x500_lidar_2d_oakd"
    includes = {i.findtext("uri"): i for i in model.findall("include")}
    assert set(includes) == {"x500", "model://lidar_2d_v2", "model://OakD-Lite"}
    assert all(i.get("merge") == "true" for i in includes.values())
    # Identical to PX4 x500_lidar_2d so PX4 GZBridge's lidar subscription still applies.
    assert _pose(includes["model://lidar_2d_v2"].findtext("pose")) == [0.12, 0, 0.26, 0, 0, 0]
    cam = _pose(includes["model://OakD-Lite"].findtext("pose"))
    assert cam[:3] == [0.17, 0.03, 0.242]
    assert math.isclose(cam[4], math.radians(30.0), abs_tol=1e-9)
    assert cam[3] == cam[5] == 0.0
    joints = {j.get("name"): (j.findtext("parent"), j.findtext("child")) for j in model.findall("joint")}
    assert joints == {"LidarJoint": ("base_link", "link"), "CameraJoint": ("base_link", "camera_link")}
