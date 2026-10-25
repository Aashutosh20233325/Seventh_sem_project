"""Stage C world variants: selection, variant files, generator provenance, launcher wiring, the consistency gate,
the world-target gate and the runner's world record. Offline only."""
import difflib
import json
from types import SimpleNamespace

import pytest

import run_target_reaching_episode as runner
import sar_task
from sar_standin_env import SarStandInEnv, valid_random_target
from test_split_rendering import OAKD, harness, iso  # noqa: F401  (harness is a fixture)
from tools import make_world_variant as mwv
from tools import verify_world_consistency as vwc
from tools import verify_world_target as vwt

VARIANTS = {"c1_ne": (2.5, 2.5), "c2_sw": (-4.5, -2.5), "c5_w": (-1.0, -4.5)}


@pytest.fixture(autouse=True)
def no_variant(monkeypatch):
    monkeypatch.delenv(sar_task.WORLD_VARIANT_ENV, raising=False)


# ---------- selection ----------

def test_default_world_when_unset():
    assert sar_task.selected_world_variant() is None and sar_task.selected_world_sdf() == sar_task.WORLD_SDF
    assert sar_task.load_target() == sar_task.load_target(sar_task.WORLD_SDF)


@pytest.mark.parametrize("name, xy", VARIANTS.items())
def test_load_target_follows_the_variant_at_call_time(monkeypatch, name, xy):
    monkeypatch.setenv(sar_task.WORLD_VARIANT_ENV, name)
    t = sar_task.load_target()
    assert (t.x_ned, t.y_ned) == xy and t.radius == 0.5
    assert SarStandInEnv().nominal_target == t


@pytest.mark.parametrize("value", ["", "C1_NE", "../c1_ne", "c1 ne", "c1_ne/", "nope", "_c1"])
def test_bad_or_unknown_variants_are_refused(monkeypatch, value):
    monkeypatch.setenv(sar_task.WORLD_VARIANT_ENV, value)
    with pytest.raises(ValueError):
        sar_task.load_target()


# ---------- variant files ----------

@pytest.mark.parametrize("name, xy", VARIANTS.items())
def test_variant_differs_from_the_default_world_only_in_the_target_pose(name, xy):
    base = sar_task.WORLD_SDF.read_text().splitlines()
    var = sar_task.world_variant_sdf(name).read_text().splitlines()
    changed = [l for l in difflib.unified_diff(base, var, lineterm="", n=0) if l[:1] in "+-" and l[:3] not in ("+++", "---")]
    assert len(changed) == 3, changed
    assert changed[0] == "-      <pose>3 -3 0.055 0 0 0</pose>" or changed[0].startswith("+<!-- Stage C world variant")
    assert sum(l.startswith("+<!-- Stage C world variant") for l in changed) == 1
    assert sum(l.startswith("+      <pose>") for l in changed) == 1
    assert valid_random_target(*xy, SarStandInEnv().walls)
    assert '<world name="sar_walls">' in "\n".join(var)


@pytest.mark.parametrize("name, xy", VARIANTS.items())
def test_committed_variants_are_exactly_what_the_generator_writes(name, xy):
    expected = mwv.variant_text(sar_task.WORLD_SDF.read_text(), name, *xy)
    assert sar_task.world_variant_sdf(name).read_text() == expected


def test_generator_refuses_invalid_targets_names_and_overwrites(tmp_path, monkeypatch):
    with pytest.raises(SystemExit):
        mwv.main(["x1", "-5.5", "1.0"])                                  # fails the validity rule
    with pytest.raises(SystemExit):
        mwv.main(["Bad", "2.5", "2.5"])
    monkeypatch.setattr(sar_task, "WORLD_VARIANTS", tmp_path)
    assert mwv.main(["t1", "2.5", "2.5"]) == 0
    with pytest.raises(FileExistsError):
        mwv.main(["t1", "2.5", "2.5"])


# ---------- launcher ----------

def test_resolve_world_default_and_variant(monkeypatch):
    oakd = iso.PROFILES["sar_oakd"]
    assert iso.resolve_world(oakd, None) == (None, sar_task.WORLD_SDF)
    assert iso.resolve_world(iso.PROFILES["legacy"], None) == (None, None)
    sdf = sar_task.world_variant_sdf("c2_sw")
    assert iso.resolve_world(oakd, "c2_sw", {sar_task.WORLD_VARIANT_ENV: "c2_sw"}) == (sdf.parent, sdf)


@pytest.mark.parametrize("profile, variant, env, message", [
    ("sar_oakd", "c2_sw", {}, "in the environment"),                     # flag without the env var
    ("sar_oakd", None, {sar_task.WORLD_VARIANT_ENV: "c2_sw"}, "in the environment"),
    ("sar_oakd", "c2_sw", {sar_task.WORLD_VARIANT_ENV: "c5_w"}, "in the environment"),
    ("legacy", "c2_sw", {sar_task.WORLD_VARIANT_ENV: "c2_sw"}, "project-asset profile"),
    ("sar_oakd", "nope", {sar_task.WORLD_VARIANT_ENV: "nope"}, "unknown world variant"),
])
def test_resolve_world_refuses_inconsistent_requests(profile, variant, env, message):
    with pytest.raises(iso.IsolationError, match=message):
        iso.resolve_world(iso.PROFILES[profile], variant, env)


def test_gz_env_points_px4_at_the_variant_directory_and_default_is_unchanged():
    oakd = iso.PROFILES["sar_oakd"]
    vdir = sar_task.world_variant_sdf("c5_w").parent
    script = iso.gz_env_script(oakd, worlds_dir=vdir).decode()
    assert f"export PX4_GZ_WORLDS={vdir}\n" in script and f"{iso.SIM_MODELS}:{vdir}:" in script
    assert iso.gz_env_script(oakd) == iso.gz_env_script(oakd, worlds_dir=iso.SIM_WORLDS)
    assert iso.gz_gui_env_script(oakd, worlds_dir=vdir).decode().count(f"PX4_GZ_WORLDS={vdir}") == 1


def test_start_records_the_world_in_the_manifest(harness, monkeypatch):  # noqa: F811
    monkeypatch.setenv(sar_task.WORLD_VARIANT_ENV, "c2_sw")
    iso.start(harness.run, OAKD, split_rendering=True, world_variant="c2_sw")
    m = json.loads((harness.run / iso.MANIFEST).read_text())
    sdf = sar_task.world_variant_sdf("c2_sw")
    assert (m["world_variant"], m["world_sdf"], m["world_sdf_sha256"]) == ("c2_sw", str(sdf), sar_task.file_sha256(sdf))
    assert f"PX4_GZ_WORLDS={sdf.parent}" in (harness.run / "px4_rootfs/gz_env.sh").read_text()
    assert f"PX4_GZ_WORLDS={sdf.parent}" in (harness.run / iso.GUI_ENV_FILE).read_text()
    assert vwc.check(m) == []                                            # same shell: consistent


def test_default_start_records_the_default_world(harness):  # noqa: F811
    iso.start(harness.run, OAKD)
    m = json.loads((harness.run / iso.MANIFEST).read_text())
    assert m["world_variant"] is None and m["world_sdf"] == str(sar_task.WORLD_SDF)
    assert f"PX4_GZ_WORLDS={iso.SIM_WORLDS}\n" in (harness.run / "px4_rootfs/gz_env.sh").read_text()
    assert vwc.check(m) == []


# ---------- consistency gate ----------

def manifest(variant):
    sdf = sar_task.WORLD_SDF if variant is None else sar_task.world_variant_sdf(variant)
    return {"world_variant": variant, "world_sdf": str(sdf), "world_sdf_sha256": sar_task.file_sha256(sdf)}


@pytest.mark.parametrize("m, env, message", [
    (manifest("c2_sw"), {}, "launcher variant 'c2_sw' != selected here None"),
    (manifest(None), {sar_task.WORLD_VARIANT_ENV: "c2_sw"}, "launcher variant None != selected here 'c2_sw'"),
    ({**manifest("c2_sw"), "world_sdf_sha256": "0" * 64}, {sar_task.WORLD_VARIANT_ENV: "c2_sw"}, "sha256"),
    ({"world": "sar_walls"}, {}, "manifest lacks"),
    (manifest(None), {sar_task.WORLD_VARIANT_ENV: "bogus"}, "unknown world variant"),
])
def test_consistency_gate_fails_closed(m, env, message):
    errors = vwc.check(m, env)
    assert errors and any(message in e for e in errors), errors


def test_consistency_gate_cli(tmp_path, monkeypatch, capsys):
    (tmp_path / "processes.json").write_text(json.dumps(manifest("c5_w")))
    monkeypatch.setenv(sar_task.WORLD_VARIANT_ENV, "c5_w")
    assert vwc.main(["--run-dir", str(tmp_path)]) == 0 and "PASS" in capsys.readouterr().out
    monkeypatch.delenv(sar_task.WORLD_VARIANT_ENV)
    assert vwc.main(["--run-dir", str(tmp_path)]) == 1
    assert vwc.main(["--run-dir", str(tmp_path / "missing")]) == 1


# ---------- world-target gate and runner ----------

def pose_info(x_enu, y_enu, z=0.055):
    return f'pose {{\n  name: "target"\n  id: 9\n  position {{\n    x: {x_enu}\n    y: {y_enu}\n    z: {z}\n  }}\n}}\n'


@pytest.mark.parametrize("variant, live_enu, ok", [("c2_sw", (-2.5, -4.5), True), ("c2_sw", (3, -3), False),
                                                   (None, (3, -3), True), (None, (-2.5, -4.5), False)])
def test_world_target_gate_compares_with_the_selected_world(monkeypatch, capsys, variant, live_enu, ok):
    if variant:
        monkeypatch.setenv(sar_task.WORLD_VARIANT_ENV, variant)
    monkeypatch.setattr(vwt.subprocess, "run", lambda *a, **k: SimpleNamespace(stdout=pose_info(*live_enu)))
    assert vwt.main([]) == (0 if ok else 1)
    out = capsys.readouterr().out
    assert ("PASS" in out) == ok
    if ok:
        assert ("variants/c2_sw" in out) == (variant == "c2_sw")


def test_runner_records_the_world(tmp_path, monkeypatch):
    monkeypatch.setenv(sar_task.WORLD_VARIANT_ENV, "c5_w")
    log = tmp_path / "episode.jsonl"
    runner.main(["--standin", "--log", str(log)])
    w = next(json.loads(l) for l in log.read_text().splitlines() if '"event": "world"' in l)
    sdf = sar_task.world_variant_sdf("c5_w")
    assert (w["variant"], w["path"], w["sha256"], w["target_ned"]) == ("c5_w", str(sdf), sar_task.file_sha256(sdf), [-1.0, -4.5])
