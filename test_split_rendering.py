"""Opt-in split rendering in tools/isolated_sitl.py: no processes are launched by these tests."""
import json
import subprocess
from types import SimpleNamespace

import pytest

from tools import isolated_sitl as iso

OAKD = iso.PROFILES["sar_oakd"]
D3D12 = "export GALLIUM_DRIVER=d3d12\nexport MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA\n"


class FakeTime:
    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        self.t += 1.0
        return self.t

    def sleep(self, _s):
        pass


@pytest.fixture
def harness(monkeypatch, tmp_path):
    spawned = []

    def fake_spawn(run_dir, name, command, env, cwd):
        n = len(spawned) + 1
        rec = {"pid": 100 * n, "pgid": 100 * n, "start_time": n, "command": command, "owned_command": command,
               "child_pid": 100 * n + 1, "child_start_time": 10 + n, "log": str(run_dir / "logs" / f"{name}.log")}
        spawned.append({"name": name, "command": command, "env": env, "cwd": cwd})
        return rec

    alive = {"dead": set()}

    def fake_start_time(pid):
        if pid in alive["dead"]:
            return None
        return (pid - 1) // 100 + 10 if pid % 100 == 1 else pid // 100

    base = {"PATH": "/usr/bin:/bin", "HOME": "/home/hp", "GALLIUM_DRIVER": "d3d12",
            "MESA_D3D12_DEFAULT_ADAPTER_NAME": "NVIDIA", "LIBGL_ALWAYS_SOFTWARE": "0"}
    monkeypatch.setattr(iso, "_check_paths", lambda run_dir, profile=None: None)
    monkeypatch.setattr(iso, "_assert_endpoint_free", lambda port: None)
    monkeypatch.setattr(iso, "sourced_environment", lambda domain_id=iso.DOMAIN_ID: dict(base))
    monkeypatch.setattr(iso, "_assert_domain_and_partition_unused", lambda env: None)
    monkeypatch.setattr(iso, "_spawn", fake_spawn)
    monkeypatch.setattr(iso, "_proc_start_time", fake_start_time)
    monkeypatch.setattr(iso, "time", FakeTime())
    monkeypatch.setattr(iso.subprocess, "run", lambda *a, **k: SimpleNamespace(
        stdout=f"/world/{OAKD.world}/clock\n".encode(), returncode=0))
    return SimpleNamespace(spawned=spawned, run=tmp_path / "run", alive=alive)


def names(h):
    return [s["name"] for s in h.spawned]


def test_default_start_is_unchanged(harness):
    iso.start(harness.run, OAKD)
    assert names(harness) == ["xrce_agent", "px4_gazebo", "lidar_bridge", "observation_publisher", "target_perception"]
    px4_env = harness.spawned[1]["env"]
    assert "HEADLESS" not in px4_env
    assert not (harness.run / iso.GUI_ENV_FILE).exists()
    manifest = json.loads((harness.run / iso.MANIFEST).read_text())
    assert "gz_gui" not in manifest and "gz_gui" not in manifest["components"]
    assert manifest["gz_rendering"] == "Mesa d3d12 (NVIDIA adapter)"
    assert D3D12 in (harness.run / "px4_rootfs/gz_env.sh").read_text()


def test_split_start_runs_server_headless_on_gpu_and_owns_a_cpu_gui(harness):
    iso.start(harness.run, OAKD, split_rendering=True)
    assert names(harness) == ["xrce_agent", "px4_gazebo", "gz_gui", "lidar_bridge",
                              "observation_publisher", "target_perception"]
    px4, gui = harness.spawned[1], harness.spawned[2]
    assert px4["env"]["HEADLESS"] == "1"
    assert D3D12 in (harness.run / "px4_rootfs/gz_env.sh").read_text()          # server keeps the GPU
    assert gui["command"] == ["bash", "-c", f". {harness.run.resolve() / iso.GUI_ENV_FILE} && exec gz sim -g"]
    for var in iso.GL_SELECTION_VARS + ("HEADLESS",):
        assert var not in gui["env"]                                              # nothing inherited
    assert gui["env"]["GZ_PARTITION"] == iso.GZ_PARTITION
    manifest = json.loads((harness.run / iso.MANIFEST).read_text())
    assert list(manifest["components"]) == names(harness)
    assert "split rendering" in manifest["gz_rendering"] and "HEADLESS=1" in manifest["gz_gui"]


@pytest.mark.parametrize("profile, software", [(iso.PROFILES["legacy"], False), (OAKD, True)])
def test_split_rendering_refuses_unsupported_combinations_before_doing_anything(harness, profile, software):
    with pytest.raises(iso.IsolationError, match="split-rendering"):
        iso.start(harness.run, profile, software_rendering=software, split_rendering=True)
    assert harness.spawned == [] and not harness.run.exists()


def test_gui_exit_during_startup_is_reported_and_nothing_is_signalled(harness, monkeypatch):
    harness.alive["dead"].add(301)                                                # gz_gui child (3rd spawn)
    killed = []
    monkeypatch.setattr(iso.os, "killpg", lambda *a: killed.append(a))
    with pytest.raises(iso.IsolationError, match="gz GUI exited"):
        iso.start(harness.run, OAKD, split_rendering=True)
    manifest = json.loads((harness.run / iso.MANIFEST).read_text())
    assert "gz_gui" in manifest["components"] and killed == []                   # left for inspection


def _source(script_path):
    out = subprocess.run(["bash", "-c", f". {script_path} && env"], capture_output=True, text=True,
                         env={"PATH": "/usr/bin:/bin", "GALLIUM_DRIVER": "d3d12",
                              "MESA_D3D12_DEFAULT_ADAPTER_NAME": "NVIDIA"}, check=True).stdout
    return dict(line.split("=", 1) for line in out.splitlines() if "=" in line)


def test_sourced_env_files_select_different_renderers_with_the_same_assets(tmp_path):
    run = tmp_path / "run"
    iso._prepare_run_dir(run, OAKD, split_rendering=True)
    server = _source(run / "px4_rootfs/gz_env.sh")
    gui = _source(run / iso.GUI_ENV_FILE)
    assert server["GALLIUM_DRIVER"] == "d3d12" and server["MESA_D3D12_DEFAULT_ADAPTER_NAME"] == "NVIDIA"
    assert gui["GALLIUM_DRIVER"] == "llvmpipe" and "MESA_D3D12_DEFAULT_ADAPTER_NAME" not in gui
    for key in ("GZ_SIM_RESOURCE_PATH", "PX4_GZ_MODELS", "PX4_GZ_WORLDS", "GZ_SIM_SYSTEM_PLUGIN_PATH"):
        assert gui[key] == server[key]
    assert str(iso.SIM_MODELS) in gui["GZ_SIM_RESOURCE_PATH"]


def test_default_run_dir_has_no_gui_env_file(tmp_path):
    iso._prepare_run_dir(tmp_path / "run", OAKD)
    assert not (tmp_path / "run" / iso.GUI_ENV_FILE).exists()


def test_gated_stop_signals_the_gui_before_the_server(monkeypatch):
    manifest = {"components": {n: {"pgid": i} for i, n in enumerate(
        ["xrce_agent", "px4_gazebo", "gz_gui", "lidar_bridge", "observation_publisher", "target_perception"], 1)}}
    order = []
    monkeypatch.setattr(iso, "_verified_record", lambda r: True)
    monkeypatch.setattr(iso.os, "killpg", lambda pgid, sig: order.append(pgid))
    monkeypatch.setattr(iso, "_wait_group_exit", lambda pgid, t: True)
    assert iso._signal_owned_groups(manifest) == []
    assert order == [6, 5, 4, 3, 2, 1]                                           # gz_gui (3) before PX4 (2)


def _render_fixture(monkeypatch, split, gui_d3d12=False, gui_llvmpipe=8, server_llvmpipe=0):
    comps = {"px4_gazebo": {"pgid": 200}}
    if split:
        comps["gz_gui"] = {"pgid": 300}
    rendering = ("server: Mesa d3d12 (NVIDIA adapter); GUI: Mesa llvmpipe (split rendering, launcher-owned gz_gui)"
                 if split else "Mesa d3d12 (NVIDIA adapter)")
    monkeypatch.setattr(iso, "_load_manifest", lambda run: {"gz_rendering": rendering, "components": comps})
    monkeypatch.setattr(iso, "_verified_record", lambda r: True)
    members = {200: {201, 202} if split else {201, 202, 203}, 300: {301}}
    monkeypatch.setattr(iso, "_live_group_members", lambda pgid: members[pgid])
    cmds = {201: ["px4"], 202: ["gz", "sim", "--verbose=1", "-r", "-s", "w.sdf"], 203: ["gz", "sim", "-g"],
            301: ["gz", "sim", "-g"]}
    monkeypatch.setattr(iso, "_cmdline", lambda pid: cmds.get(pid))
    gui_pid = 301 if split else 203
    monkeypatch.setattr(iso, "_maps_contains", lambda pid, n: (pid == 202) or (pid == gui_pid and gui_d3d12))
    monkeypatch.setattr(iso, "_thread_names", lambda pid: (
        ["gz"] + ["llvmpipe-0"] * (gui_llvmpipe if pid == gui_pid else server_llvmpipe if pid == 202 else 0)))


def test_render_check_passes_only_with_gpu_server_and_cpu_gui(monkeypatch, tmp_path):
    _render_fixture(monkeypatch, split=True)
    report = iso.render_report(tmp_path)
    assert report["ok"] and report["processes"]["server"]["d3d12_mapped"] is True
    assert report["processes"]["gui"] == {"pid": 301, "d3d12_mapped": False, "llvmpipe_threads": 8}


@pytest.mark.parametrize("kw", [dict(gui_d3d12=True), dict(gui_llvmpipe=0), dict(server_llvmpipe=4)])
def test_render_check_fails_when_isolation_did_not_happen(monkeypatch, tmp_path, kw):
    _render_fixture(monkeypatch, split=True, **kw)
    assert iso.render_report(tmp_path)["ok"] is False


def test_render_check_default_mode_requires_only_a_gpu_server(monkeypatch, tmp_path):
    _render_fixture(monkeypatch, split=False, gui_d3d12=True, gui_llvmpipe=0)
    report = iso.render_report(tmp_path)
    assert report["ok"] and set(report["processes"]) == {"server", "gui"}


def test_cli_exposes_flag_and_rejects_legacy_split(tmp_path, capsys):
    assert iso.main(["start", "--profile", "legacy", "--split-rendering", "--run-dir", str(tmp_path / "r")]) == 2
    assert "split-rendering" in capsys.readouterr().err and not (tmp_path / "r").exists()
