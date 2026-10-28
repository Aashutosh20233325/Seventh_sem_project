"""Mine worlds (mine scenario): determinism, validity (hidden but reachable worker, tunnels, roof falls), the world
file read back by the stand-in and the task exactly as generated, the worker as the target's visual, Gazebo acceptance."""
import json
import math
import shutil
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest

import mine_generator as mg
import sar_standin_env as se
import sar_task
import world_generator as wg

COMMITTED = {"gen_mine_s1": 1, "gen_mine_s2": 2, "gen_mine_s3": 3}


@pytest.mark.parametrize("seed", [1, 7, 42])
def test_generation_is_deterministic(seed):
    assert mg.generate(seed) == mg.generate(seed)


@pytest.mark.parametrize("seed", range(1, 21))
def test_every_mine_meets_the_validity_rules(seed):
    spec = mg.generate(seed)
    boxes = [wg.Box(**b) for b in spec["obstacles"]]
    tall = [b for b in boxes if b.top >= wg.TALL_TOP]
    target = tuple(spec["target_ned"])
    assert mg.TARGET_MIN - mg.TARGET_WALL_OFFSET <= math.hypot(*target) <= mg.TARGET_MAX + mg.TARGET_WALL_OFFSET
    assert wg.grid_path_exists((0.0, 0.0), target, tall)                  # reachable with the 1.5 m clearance
    assert wg.target_hidden((0.0, 0.0), target, tall)                     # not visible from the start
    assert len(spec["roof_falls"]) == mg.ROOF_FALLS
    assert all(b.top <= wg.LOW_TOP for b in boxes if b.kind == "debris")  # debris is flown over
    assert all(b.top >= wg.TALL_TOP for b in boxes if b.kind in ("rock", "rooffall"))
    assert all(max(abs(v) for v in b.rect()) <= wg.AREA + 2.6 for b in boxes)
    # the start junction is free: nothing tall within half a drift width of the origin
    assert all(wg.rect_distance(0.0, 0.0, b.rect()) >= mg.DRIFT / 2 - 1e-9 for b in tall)


def test_names_must_be_mine_worlds(tmp_path):
    for bad in ("gen_h2_s1", "mine_s1", "gen_m s1"):
        with pytest.raises(ValueError):
            mg.write(bad, 1, root=tmp_path)


@pytest.mark.parametrize("name, seed", COMMITTED.items())
def test_committed_mines_are_exactly_what_the_generator_writes(tmp_path, name, seed):
    out = mg.write(name, seed, root=tmp_path)
    for f in ("sar_walls.sdf", "terrain.obj", "spec.json"):
        assert (out / f).read_text() == (sar_task.WORLD_VARIANTS / name / f).read_text(), f


@pytest.mark.parametrize("name", COMMITTED)
def test_gazebo_world_standin_and_task_agree(monkeypatch, name):
    spec = json.loads((sar_task.WORLD_VARIANTS / name / "spec.json").read_text())
    monkeypatch.setenv(sar_task.WORLD_VARIANT_ENV, name)
    target = sar_task.load_target()
    assert [target.x_ned, target.y_ned] == pytest.approx(spec["target_ned"]) and target.radius == 0.5
    tall = sorted(wg.Box(**b).rect() for b in spec["obstacles"] if b["z"] + b["h"] >= wg.TALL_TOP)
    walls = sorted(se.load_walls())
    assert len(walls) == len(tall)
    d = np.abs(np.array(walls)[:, None, :] - np.array(tall)[None, :, :]).max(axis=2)
    assert np.all(d.min(axis=1) < 2e-3) and np.all(d.min(axis=0) < 2e-3)
    obs, _ = se.SarStandInEnv().reset(seed=0)
    assert np.min(obs[se.so.SLICES["lidar"]]) < 0.2                        # tunnel walls are close


@pytest.mark.parametrize("name", COMMITTED)
def test_target_is_the_worker_and_the_mine_is_lit(name):
    world = ET.parse(sar_task.WORLD_VARIANTS / name / "sar_walls.sdf").getroot().find("world")
    target = world.find("model[@name='target']")
    uris = [u.text for u in target.iter("uri")]
    assert uris == ["model://rescue_worker/meshes/rescue_randy.dae"]       # the worker, not the red sphere
    assert target.find(".//collision/geometry/sphere/radius").text.strip() == "0.5"
    assert all("1 0 0" not in (m.text or "") for m in target.iter("diffuse"))
    assert world.findall("light[@type='point']") == []                     # no lamps: they made rendering stall
    assert float(world.findtext("scene/ambient").split()[0]) >= 0.6         # lit by ambient and overhead light
    assert world.findall(".//collision/geometry/mesh") == []                # no mesh collision (DART crash)
    terrain = world.find("model[@name='terrain']")
    assert terrain.find("link/collision") is None                           # visual floor only (a floor collision box
    z = float(terrain.findtext("pose").split()[2])                          # coincided with camera rendering stalls)
    h = float(terrain.findtext("link/visual/geometry/box/size").split()[2])
    assert z + h / 2 == pytest.approx(0.0)                                  # visual top at the floor
    pads = [m for m in world.findall("model") if m.get("name", "").startswith("pad_")]
    assert len(pads) == 2 and all(m.find(".//collision/geometry/box") is not None for m in pads)   # solid pads


def test_worker_model_is_visual_only_and_cc0():
    from pathlib import Path
    d = Path(__file__).resolve().parent / "sim/models/rescue_worker"
    sdf = (d / "model.sdf").read_text()
    assert "<collision" not in sdf and "<plugin" not in sdf and "<include" not in sdf
    assert "CC0" in (d / "model.config").read_text()
    assert (d / "meshes/rescue_randy.dae").exists() and (d / "materials/textures/RescueRandy_Diffuse.jpg").exists()


@pytest.mark.skipif(shutil.which("gz") is None, reason="gz not on PATH (source ROS)")
@pytest.mark.parametrize("name", COMMITTED)
def test_gazebo_accepts_the_mine(name):
    d = sar_task.WORLD_VARIANTS / name
    p = subprocess.run(["gz", "sdf", "-k", "sar_walls.sdf"], cwd=d, capture_output=True, text=True, timeout=60)
    assert "Valid" in p.stdout, p.stdout + p.stderr


DECOR = ("borehole_", "derrick_", "sign_", "rooffall_", "rail_", "cart_", "timber_")


@pytest.mark.parametrize("name", COMMITTED)
def test_story_props_are_visual_only_and_clear_of_the_flight(name):
    """Borehole, derrick, plaques, roof-fall chunks, rails, carts and timber sets: visuals of ONE static model with no
    collision (physics and the stand-in ignore them; one entity keeps Gazebo's memory down); the rope ends above the
    1.8 m flight height; carts and rails are low enough to fly over; roof-fall chunks stick out at most 0.15 m (+ chunk
    half-size slack) beyond the block they decorate."""
    world = ET.parse(sar_task.WORLD_VARIANTS / name / "sar_walls.sdf").getroot().find("world")
    spec = json.loads((sar_task.WORLD_VARIANTS / name / "spec.json").read_text())
    names = [m.get("name") for m in world.findall("model")]
    assert not any(n.startswith(DECOR) for n in names if not n.startswith("rooffall_"))   # no separate prop models
    props = world.find("model[@name='mine_props']")
    assert props.find(".//collision") is None
    vis = {v.get("name"): v for v in props.iter("visual")}
    assert len(vis) > 100 and all(n.startswith(DECOR) for n in vis)
    pose = lambda v: [float(x) for x in v.findtext("pose").split()]          # noqa: E731
    size = lambda v: [float(x) for x in v.findtext("geometry/box/size").split()]   # noqa: E731
    rope = vis["borehole_rope"]
    assert pose(rope)[2] - size(rope)[2] / 2 >= 2.5                          # rope ends above the drone
    for n, v in vis.items():
        if n.startswith(("cart_", "rail_")):
            assert pose(v)[2] + size(v)[2] / 2 <= wg.LOW_TOP                 # flown over like debris
    falls = [b for b in spec["obstacles"] if b["kind"] == "rooffall"]
    for n, v in vis.items():
        if "_chunk_" in n:
            ned_y, ned_x = pose(v)[:2]                                        # SDF poses are ENU (east, north)
            s = max(size(v))
            assert any(abs(ned_x - b["x"]) <= b["sx"] / 2 + 0.15 + s / 2 and abs(ned_y - b["y"]) <= b["sy"] / 2 + 0.15 + s / 2
                       for b in falls)
    assert len(world.findall("model")) < 100                                  # obstacles + target + pads + props


def test_plaques_never_look_like_a_target():
    from pathlib import Path
    from PIL import Image
    import target_perception as tp
    d = Path(__file__).resolve().parent / "sim/models/mine_textures/materials/textures"
    for f in sorted(d.glob("*.png")):
        assert int(tp.target_mask(np.asarray(Image.open(f).convert("RGB"))).sum()) == 0, f.name
