"""world_generator: determinism, validity rules, Gazebo/stand-in consistency, terrain mesh, committed worlds. Offline."""
import json
import math
import shutil
import subprocess

import numpy as np
import pytest

import sar_standin_env as se
import sar_task
import world_generator as wg

COMMITTED = {"gen_l1_s1": (1, 1), "gen_l2_s1": (1, 2), "gen_l3_s1": (1, 3)}


@pytest.fixture(autouse=True)
def no_variant(monkeypatch):
    monkeypatch.delenv(sar_task.WORLD_VARIANT_ENV, raising=False)


@pytest.mark.parametrize("seed, level", [(1, 1), (2, 2), (3, 3), (7, 3), (11, 1)])
def test_generation_is_deterministic(seed, level):
    assert wg.generate(seed, level) == wg.generate(seed, level)


@pytest.mark.parametrize("seed, level", [(s, l) for s in range(1, 6) for l in (1, 2, 3)])
def test_every_world_meets_the_validity_rules(seed, level):
    spec = wg.generate(seed, level)
    h = np.array(spec["terrain_heights"])
    xs = np.arange(-wg.AREA, wg.AREA + 1e-9, wg.TERRAIN_STEP)
    tx, ty = spec["target_ned"]
    assert 10 <= math.hypot(tx, ty) <= 18
    assert np.max(np.abs(h)) <= spec["terrain_amplitude_m"] + 1e-6
    for px, py in ((0.0, 0.0), (tx, ty)):                                   # flat pads
        on_pad = (np.abs(xs[:, None] - px) <= wg.PAD) & (np.abs(xs[None, :] - py) <= wg.PAD)
        assert np.all(np.abs(h[on_pad]) < 1e-6)
    boxes = [wg.Box(**b) for b in spec["obstacles"]]
    for b in boxes:
        assert b.top <= wg.LOW_TOP + 1e-9 or b.top >= wg.TALL_TOP - 1e-9, b    # unambiguous at the 1.8 m hold height
        assert all(abs(v) <= wg.AREA for v in b.rect())
    tall = [b for b in boxes if b.top >= wg.TALL_TOP]
    for px, py in ((0.0, 0.0), (tx, ty)):
        assert all(wg.rect_rect_distance(b.rect(), wg.pad_rect(px, py)) >= wg.CLEARANCE - 1e-9 for b in boxes)
    assert wg.grid_path_exists((0.0, 0.0), (tx, ty), tall)
    if level == 3:
        assert any(wg.segment_hits_rect((0.0, 0.0), (tx, ty), b.rect()) for b in tall)


def test_difficulty_increases_clutter():
    counts = [len(wg.generate(4, l)["obstacles"]) for l in (1, 2, 3)]
    assert counts[0] < counts[1] < counts[2]


def test_bad_inputs_are_refused(tmp_path):
    with pytest.raises(ValueError):
        wg.generate(1, 4)
    with pytest.raises(ValueError):
        wg.write("world1", 1, 1, root=tmp_path)                            # must start with gen_
    wg.write("gen_t", 1, 1, root=tmp_path)
    with pytest.raises(FileExistsError):
        wg.write("gen_t", 1, 1, root=tmp_path)


def test_terrain_mesh_matches_the_heights_in_enu(tmp_path):
    out = wg.write("gen_t", 5, 2, root=tmp_path)
    spec = json.loads((out / "spec.json").read_text())
    h = np.array(spec["terrain_heights"])
    n = h.shape[0]
    verts = [tuple(map(float, l.split()[1:])) for l in (out / "terrain.obj").read_text().splitlines() if l.startswith("v ")]
    faces = [l for l in (out / "terrain.obj").read_text().splitlines() if l.startswith("f ")]
    assert len(verts) == n * n and len(faces) == 2 * (n - 1) ** 2
    xs = np.linspace(-wg.AREA, wg.AREA, n)
    for i, j in ((0, 0), (n - 1, 0), (0, n - 1), (n // 3, 2 * n // 3)):
        x_enu, y_enu, z = verts[i * n + j]
        assert (x_enu, y_enu) == pytest.approx((xs[j], xs[i])) and z == pytest.approx(h[i, j], abs=1e-4)
    a, b, d = (np.array(verts[int(k) - 1]) for k in faces[0].split()[1:])  # first triangle faces up
    assert np.cross(b - a, d - a)[2] > 0


@pytest.mark.parametrize("name, args", COMMITTED.items())
def test_committed_worlds_are_exactly_what_the_generator_writes(tmp_path, name, args):
    out = wg.write(name, *args, root=tmp_path)
    for f in ("sar_walls.sdf", "terrain.obj", "spec.json"):
        assert (out / f).read_text() == (sar_task.WORLD_VARIANTS / name / f).read_text(), f


@pytest.mark.parametrize("name", COMMITTED)
def test_gazebo_world_and_standin_agree(monkeypatch, name):
    spec = json.loads((sar_task.WORLD_VARIANTS / name / "spec.json").read_text())
    monkeypatch.setenv(sar_task.WORLD_VARIANT_ENV, name)
    target = sar_task.load_target()                                        # what PX4UAVEnv and the gates read
    assert [target.x_ned, target.y_ned] == pytest.approx(spec["target_ned"]) and target.radius == 0.5
    tall = sorted(wg.Box(**b).rect() for b in spec["obstacles"] if b["z"] + b["h"] >= wg.TALL_TOP)
    walls = sorted(se.load_walls())
    assert len(walls) == len(tall)                                         # the stand-in sees exactly the tall obstacles
    d = np.abs(np.array(walls)[:, None, :] - np.array(tall)[None, :, :]).max(axis=2)
    assert np.all(d.min(axis=1) < 2e-3) and np.all(d.min(axis=0) < 2e-3)   # one-to-one within 2 mm (SDF stores mm)
    env = se.SarStandInEnv()
    assert env.nominal_target == target and len(env.base_walls) == len(tall)
    obs, _ = env.reset(seed=0)
    assert np.min(obs[se.so.SLICES["lidar"]]) < 1.0                        # something is in LiDAR range


def test_default_world_is_unchanged_for_the_standin():
    assert se.load_walls() == se.load_walls(sar_task.WORLD_SDF) and len(se.load_walls()) == 4


@pytest.mark.skipif(shutil.which("gz") is None, reason="gz not on PATH (source ROS)")
@pytest.mark.parametrize("name", COMMITTED)
def test_gazebo_accepts_the_world(name):
    d = sar_task.WORLD_VARIANTS / name
    p = subprocess.run(["gz", "sdf", "-k", "sar_walls.sdf"], cwd=d, capture_output=True, text=True, timeout=60)
    assert "Valid" in p.stdout, p.stdout + p.stderr
