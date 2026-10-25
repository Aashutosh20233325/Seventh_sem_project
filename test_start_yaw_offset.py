"""Start-heading offset (Stage B): shared validation, stand-in and live-env behaviour, and the runner option.
Offline only; the live env is exercised with the lightweight shims from test_px4_uav_env_safety."""
import json
import math

import numpy as np
import pytest

import run_target_reaching_episode as runner
import sar_task
from sar_standin_env import SarStandInEnv
from test_px4_uav_env_safety import load_environment, make_environment


@pytest.mark.parametrize("options, expected", [(None, 0.0), ({}, 0.0), ({"start_yaw_offset": 0.5}, 0.5),
                                               ({"start_yaw_offset": -math.pi}, -math.pi), ({"start_yaw_offset": 1}, 1.0)])
def test_valid_offsets(options, expected):
    assert sar_task.start_yaw_offset(options) == expected


@pytest.mark.parametrize("options", [{"start_yaw": 0.1}, {"start_yaw_offset": float("nan")},
                                     {"start_yaw_offset": math.pi + 1e-6}, {"start_yaw_offset": True},
                                     {"start_yaw_offset": "1.0"}, {"start_yaw_offset": None}])
def test_invalid_offsets_are_refused(options):
    with pytest.raises(ValueError):
        sar_task.start_yaw_offset(options)


def test_wrap_angle():
    assert sar_task.wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)


# ---------- stand-in ----------

def test_standin_without_offset_is_unchanged():
    a, ia = SarStandInEnv().reset(seed=3)
    b, ib = SarStandInEnv().reset(seed=3, options={"start_yaw_offset": 0.0})
    np.testing.assert_array_equal(a, b)
    assert ia["privileged"]["heading"] == ib["privileged"]["heading"] == pytest.approx(math.pi / 2)


@pytest.mark.parametrize("deg, heading_deg", [(90, 180), (-90, 0), (180, -90), (-135, -45)])
def test_standin_offset_turns_the_start_heading(deg, heading_deg):
    env = SarStandInEnv()
    _, info = env.reset(seed=3, options={"start_yaw_offset": math.radians(deg)})
    assert math.degrees(info["privileged"]["heading"]) == pytest.approx(heading_deg)
    assert (env.x, env.y) == (0.0, 0.0)


def test_standin_offset_cannot_combine_with_randomized_start():
    with pytest.raises(ValueError, match="cannot be combined"):
        SarStandInEnv(randomize_start=True).reset(seed=1, options={"start_yaw_offset": 0.5})


# ---------- live env (shims) ----------

def test_live_fly_to_start_uses_the_offset_heading(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_environment(module)
    env.node.TAKEOFF_TARGET_Z = -1.8
    calls = []
    env._fly_to = lambda x, y, z, yaw: calls.append((x, y, z, yaw))
    env._fly_to_start()                                            # no reset yet: nominal
    env._start_yaw = sar_task.wrap_angle(module.PX4UAVEnv.START_YAW + math.radians(90))
    env._fly_to_start()
    assert calls[0] == (0.0, 0.0, -1.8, pytest.approx(math.pi / 2))
    assert calls[1] == (0.0, 0.0, -1.8, pytest.approx(math.pi))


@pytest.mark.parametrize("options", [{"bogus": 1}, {"start_yaw_offset": 4.0}])
def test_live_reset_refuses_bad_options_before_commanding_anything(monkeypatch, options):
    module = load_environment(monkeypatch)
    env = make_environment(module)
    env._episode_active = False
    touched = []
    env._wait_until_landed = lambda **k: touched.append("landed_wait")
    env._takeoff = lambda: touched.append("takeoff")
    env.node.action[:] = 0.7                                       # must stay untouched
    with pytest.raises(ValueError):
        env.reset(options=options)
    assert touched == [] and np.all(env.node.action == 0.7)


def test_live_privileged_info_reports_heading(monkeypatch):
    module = load_environment(monkeypatch)
    env = make_environment(module)
    env.node.heading = 1.25
    env._distance_to_target = lambda: 3.0
    assert env._privileged_info()["heading"] == 1.25


# ---------- runner ----------

def test_runner_passes_and_logs_the_offset(tmp_path):
    log = tmp_path / "episode.jsonl"
    runner.main(["--standin", "--start-yaw-offset-deg", "90", "--log", str(log)])
    recs = [json.loads(l) for l in log.read_text().splitlines()]
    start = next(r for r in recs if r["event"] == "reset_start")
    done = next(r for r in recs if r["event"] == "reset_done")
    assert start["options"] == {"start_yaw_offset": pytest.approx(math.pi / 2)}
    assert done["info"]["privileged"]["heading"] == pytest.approx(math.pi)


def test_runner_without_the_option_logs_no_options(tmp_path):
    log = tmp_path / "episode.jsonl"
    assert runner.main(["--standin", "--log", str(log)]) == 0
    start = next(json.loads(l) for l in log.read_text().splitlines() if '"reset_start"' in l)
    assert start["options"] is None


@pytest.mark.parametrize("value", ["181", "-200", "nan", "inf"])
def test_runner_refuses_out_of_range_offsets(tmp_path, value):
    with pytest.raises(SystemExit):
        runner.main(["--standin", "--start-yaw-offset-deg", value, "--log", str(tmp_path / "x.jsonl")])
    assert not (tmp_path / "x.jsonl").exists()
