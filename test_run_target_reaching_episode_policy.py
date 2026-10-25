"""Offline tests for the runner's PPO path (--policy): load-time interface checks, the always-on LiDAR safety
filter, fail-closed handling of unknown LiDAR age and malformed policy output, and stand-in episodes. No ROS."""
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

import run_target_reaching_episode as runner
import sar_observation as so
from sar_standin_env import SarStandInEnv

ROOT = Path(__file__).resolve().parent
E1R_300K = ROOT / "runs/rl/exp20261022_randtarget300k/E1R_s1/final_model.zip"


def events(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def obs_with(ranges_m):
    obs = np.zeros(so.DIM, np.float32)
    obs[so.SLICES["lidar"]] = np.clip(np.asarray(ranges_m, np.float32) / so.LIDAR_RANGE_MAX, 0, 1)
    return obs


CLEAR = obs_with(np.full(so.LIDAR_SECTORS, 30.0))
FRONT_BLOCKED = obs_with(np.full(so.LIDAR_SECTORS, 1.0))      # every sector at 1.0 m: front < 1.2 m stop clearance


def controller(action, age=0.0):
    return runner.PolicyController(lambda obs: np.asarray(action, np.float32), lambda: age)


# ---------- PolicyController ----------

def test_clear_path_passes_the_policy_action_unchanged():
    c = controller([0.6, -0.3])
    np.testing.assert_allclose(c.act(CLEAR), [0.6, -0.3])
    assert c.last["reason"] is None and c.last["intervened"] is False and c.last["lidar_age_s"] == 0.0


def test_blocked_front_stops_and_turns_away():
    c = controller([1.0, 0.0])
    a = c.act(FRONT_BLOCKED)
    assert a[0] == 0.0 and abs(a[1]) == pytest.approx(0.8)
    assert c.last["reason"] == "stop" and c.last["original"] == [1.0, 0.0]


@pytest.mark.parametrize("age", [None, 0.6, float("nan")])
def test_unknown_or_stale_lidar_age_hovers(age):
    c = controller([1.0, 0.5], age=age)
    np.testing.assert_array_equal(c.act(CLEAR), [0.0, 0.0])
    assert c.last["reason"] == "lidar_stale"


@pytest.mark.parametrize("raw", [[float("nan"), 0.0], [0.5, float("inf")], [0.5], [0.5, 0.1, 0.2]])
def test_malformed_policy_output_hovers_and_is_never_clipped_into_a_command(raw):
    c = controller(raw)
    np.testing.assert_array_equal(c.act(CLEAR), [0.0, 0.0])
    assert c.last["reason"] == "policy_invalid" and c.last["intervened"] is True
    json.dumps(c.last)                                                    # loggable


def test_filter_only_reduces_speed():
    c = controller([1.0, 0.2])
    a = c.act(obs_with(np.full(so.LIDAR_SECTORS, 2.0)))                   # slow zone: cap 0.8 * 0.8 / 1.8
    assert a[0] == pytest.approx(0.8 * (2.0 - 1.2) / 1.8) and a[1] == pytest.approx(0.2)


# ---------- runner integration (stand-in) ----------

class FacingWallEnv(SarStandInEnv):
    """Starts at full LiDAR range facing the north wall (x = 4.5), as in test_safety_layer."""

    def reset(self, *, seed=None, options=None):
        super().reset(seed=0)
        self.x, self.y, self.heading = 2.0, -3.0, 0.0
        return self._observe(self._lidar()), {"privileged": {}}


class Reckless:
    def act(self, obs):
        return np.array([1.0, 0.0], np.float32)


def test_reckless_policy_collides_without_the_filter_but_not_through_the_runner(tmp_path):
    with (tmp_path / "raw.jsonl").open("x") as log:
        assert runner.execute(FacingWallEnv(perception_noise=False), log, 200, None, Reckless()) == "collision"
    path = tmp_path / "filtered.jsonl"
    with path.open("x") as log:
        result = runner.execute(FacingWallEnv(perception_noise=False), log, 200, None, controller([1.0, 0.0]))
    assert result != "collision"
    steps = [e for e in events(path) if e["event"] == "step"]
    assert {"stop", "slow"} & {s["policy"]["reason"] for s in steps}
    for s in steps:                                                       # the logged action is the applied one
        assert s["action"][0] <= s["policy"]["original"][0] + 1e-6


def test_baseline_steps_carry_no_policy_record(tmp_path):
    path = tmp_path / "episode.jsonl"
    assert runner.main(["--standin", "--log", str(path)]) == 0
    assert all("policy" not in e for e in events(path) if e["event"] == "step")


# ---------- loading ----------

def spaces(obs_shape=(so.DIM,), low=so.LOW, high=so.HIGH, act_low=(0.0, -1.0), act_high=(1.0, 1.0)):
    return (SimpleNamespace(shape=obs_shape, low=np.asarray(low), high=np.asarray(high)),
            SimpleNamespace(shape=(len(act_low),), low=np.asarray(act_low), high=np.asarray(act_high)))


def test_matching_spaces_are_accepted():
    runner.check_policy_spaces(*spaces())


@pytest.mark.parametrize("kwargs, message", [
    ({"obs_shape": (49,), "low": np.zeros(49), "high": np.ones(49)}, "observation shape"),
    ({"high": np.full(so.DIM, 2.0)}, "observation bounds"),
    ({"act_low": (-1.0, -1.0)}, "action space"),
    ({"act_low": (0.0, -1.0, 0.0), "act_high": (1.0, 1.0, 1.0)}, "action space"),
])
def test_mismatched_spaces_are_refused(kwargs, message):
    with pytest.raises(ValueError, match=message):
        runner.check_policy_spaces(*spaces(**kwargs))


def test_a_failed_load_is_logged_and_no_environment_is_created(tmp_path, monkeypatch):
    def bad(path):
        raise ValueError("policy does not match the env interface: observation shape (49,) != (65,)")
    monkeypatch.setattr(runner, "load_policy", bad)
    import sar_standin_env
    monkeypatch.setattr(sar_standin_env, "SarStandInEnv", lambda *a, **k: pytest.fail("environment created"))
    path = tmp_path / "episode.jsonl"
    with pytest.raises(ValueError):
        runner.main(["--standin", "--policy", str(tmp_path / "bad.zip"), "--log", str(path)])
    assert [e["event"] for e in events(path)] == ["policy_load_failed"]


@pytest.mark.skipif(not E1R_300K.exists(), reason="E1R 300k checkpoint not present (runs/rl is git-ignored)")
def test_standin_episode_with_the_e1r_300k_checkpoint(tmp_path):
    path = tmp_path / "episode.jsonl"
    assert runner.main(["--standin", "--policy", str(E1R_300K), "--log", str(path)]) == 0
    recs = events(path)
    kinds = [e["event"] for e in recs]
    assert kinds[:3] == ["policy_loaded", "world", "reset_start"] and kinds[-2:] == ["close_start", "close_returned"]
    loaded = recs[0]
    assert loaded["sha256"] == hashlib.sha256(E1R_300K.read_bytes()).hexdigest()
    assert loaded["torch_threads"] == 1 and loaded["safety_filter"]["stop_clearance_m"] == 1.2
    steps = [e for e in recs if e["event"] == "step"]
    assert steps[-1]["info"]["reason"] == "success"
    assert all(s["policy"]["lidar_age_s"] == 0.0 and len(s["policy"]["original"]) == 2 for s in steps)
