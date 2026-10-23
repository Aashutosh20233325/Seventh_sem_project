"""Offline interface compatibility between saved PPO checkpoints, the stand-in, and the live PX4 env contract.

Checks the interface from metadata and code, not from the files' existence. Checkpoints live in git-ignored
runs/rl/; tests skip when they are absent.
"""
import ast
import json
from pathlib import Path

import numpy as np
import pytest

import sar_observation as so
import sar_task
from sar_standin_env import SarStandInEnv

ROOT = Path(__file__).resolve().parent
E1R_300K = [ROOT / f"runs/rl/exp20261022_randtarget300k/E1R_s{s}" for s in (1, 2, 3)]


def _live_env_source():
    return ast.parse((ROOT / "px4_uav_env.py").read_text())


def test_live_env_uses_the_sar_obs_v1_bounds_and_the_standin_action_space():
    src = (ROOT / "px4_uav_env.py").read_text()
    assert "low=sar_observation.LOW" in src and "high=sar_observation.HIGH" in src
    assert "[0.0, -1.0]" in src and "[1.0, 1.0]" in src
    env = SarStandInEnv()
    np.testing.assert_array_equal(env.action_space.low, [0.0, -1.0])
    np.testing.assert_array_equal(env.action_space.high, [1.0, 1.0])
    np.testing.assert_array_equal(env.observation_space.low, so.LOW)
    np.testing.assert_array_equal(env.observation_space.high, so.HIGH)


def test_live_and_standin_action_scaling_match():
    """Both map speed 1 -> 1.0 m/s along the heading and turn 1 -> 0.5 rad/s NED yaw rate (clockwise)."""
    import sar_standin_env as se
    tree = _live_env_source()
    consts = {t.id: n.value.value for n in ast.walk(tree) if isinstance(n, ast.Assign)
              for t in n.targets if isinstance(t, ast.Name) and isinstance(n.value, ast.Constant)}
    assert consts["MAX_SPEED"] == se.MAX_SPEED == 1.0
    assert consts["MAX_YAW_RATE"] == se.MAX_YAW_RATE == 0.5


def test_live_and_standin_share_task_and_step_duration():
    import sar_standin_env as se
    src = (ROOT / "px4_uav_env.py").read_text()
    assert "STEP_DURATION = 0.1" in src and se.DT == 0.1
    assert "sar_task.TaskConfig()" in src and "sar_task.load_target()" in src
    assert SarStandInEnv().task_config == sar_task.TaskConfig()


def test_known_difference_perception_age_clock_is_documented():
    """Live perception freshness is wall-clock (1.5 s); the stand-in's is simulation time (1.0 s). At 0.35x
    sim speed the live limit is ~0.5 s of sim time. This test pins the difference so a change is noticed."""
    src = (ROOT / "px4_uav_env.py").read_text()
    assert "PERCEPTION_MAX_AGE = 1.5" in src and "now - node.perception_received_at" in src
    assert so.ObservationBuilder().perception_max_age_s == 1.0


@pytest.mark.parametrize("run", E1R_300K, ids=lambda p: p.name)
def test_e1r_300k_checkpoint_matches_the_contract(run):
    model_path = run / "final_model.zip"
    if not model_path.is_file():
        pytest.skip(f"{model_path} not present (runs/ is git-ignored)")
    PPO = pytest.importorskip("stable_baselines3").PPO
    model = PPO.load(model_path, device="cpu")
    assert model.observation_space.shape == (so.DIM,) == (65,)
    np.testing.assert_array_equal(model.observation_space.low, so.LOW)
    np.testing.assert_array_equal(model.observation_space.high, so.HIGH)
    np.testing.assert_array_equal(model.action_space.low, [0.0, -1.0])
    np.testing.assert_array_equal(model.action_space.high, [1.0, 1.0])
    config = json.loads((run / "config.json").read_text())
    assert config["observation_version"] == so.VERSION and config["observation_dim"] == so.DIM
    obs, _ = SarStandInEnv(randomize_start=True).reset(seed=0)
    action, _ = model.predict(obs, deterministic=True)
    assert action.shape == (2,) and 0.0 <= action[0] <= 1.0 and -1.0 <= action[1] <= 1.0


def test_live_runner_drives_the_baseline_not_a_checkpoint():
    """The supervised episode runner has no PPO path; using a checkpoint live would be a new, unreviewed change."""
    src = (ROOT / "run_target_reaching_episode.py").read_text()
    assert "BaselineController()" in src and "PPO" not in src and "stable_baselines3" not in src
