"""numpy_policy: exact parity with stable-baselines3 deterministic predict, refusal of what it cannot reproduce,
load-time validation, the runner's torch-free .npz path, and training's automatic export. Offline only."""
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
import torch.nn as nn
from stable_baselines3 import PPO

import numpy_policy as npp
import sar_observation as so
from sar_standin_env import SarStandInEnv

ROOT = Path(__file__).resolve().parent
E1R = [ROOT / f"runs/rl/exp20261022_randtarget300k/E1R_s{s}/final_model.zip" for s in (1, 2, 3)]
TOL = 1e-5


def observations(model, n_random=1000, episodes=3, seed=0):
    rng = np.random.default_rng(seed)
    obs = [rng.uniform(so.LOW, so.HIGH).astype(np.float32) for _ in range(n_random)]
    obs += [so.LOW.copy(), so.HIGH.copy()]
    env = SarStandInEnv(randomize_start=True)
    for ep in range(episodes):
        o, _ = env.reset(seed=700 + ep)
        for _ in range(600):
            obs.append(o)
            o, _, term, trunc, _ = env.step(model.predict(o, deterministic=True)[0])
            if term or trunc:
                break
    return obs


def assert_parity(model, policy, obs):
    for o in obs:
        expected = model.predict(o, deterministic=True)[0]
        got = policy.predict(o)
        assert got.shape == (2,) and got.dtype == np.float32
        assert np.max(np.abs(expected - got)) <= TOL
        assert np.all(got >= [0.0, -1.0]) and np.all(got <= [1.0, 1.0])


def export(model, tmp_path, name="policy"):
    zip_path = tmp_path / f"{name}.zip"
    model.save(zip_path)
    out = tmp_path / f"{name}.npz"
    meta = npp.export_policy(zip_path, out)
    return PPO.load(zip_path, device="cpu"), npp.NumpyPolicy.load(out), meta, out


@pytest.mark.parametrize("checkpoint", E1R, ids=["s1", "s2", "s3"])
def test_parity_with_the_e1r_300k_checkpoints(checkpoint, tmp_path):
    if not checkpoint.exists():
        pytest.skip("E1R 300k checkpoint not present (runs/rl is git-ignored)")
    model = PPO.load(checkpoint, device="cpu")
    out = tmp_path / "p.npz"
    meta = npp.export_policy(checkpoint, out)
    policy = npp.NumpyPolicy.load(out)
    assert meta["source_sha256"] == npp.file_sha256(checkpoint) and meta["activations"] == ["tanh", "tanh", None]
    assert meta["layer_shapes"] == [[64, 65], [64, 64], [2, 64]]
    assert_parity(model, policy, observations(model))


@pytest.mark.parametrize("kwargs", [
    {"net_arch": [32, 32, 16], "activation_fn": nn.ReLU},
    {"net_arch": dict(pi=[128], vf=[16, 16]), "activation_fn": nn.Tanh},
    {"net_arch": [64, 64], "share_features_extractor": False},
], ids=["relu_3_layers", "separate_pi_vf", "unshared_extractor"])
def test_other_training_architectures_export_exactly(kwargs, tmp_path):
    model = PPO("MlpPolicy", SarStandInEnv(), policy_kwargs=kwargs, seed=4, device="cpu", n_steps=64, batch_size=32)
    loaded, policy, meta, _ = export(model, tmp_path)
    assert_parity(loaded, policy, observations(loaded, n_random=300, episodes=1))


@pytest.mark.parametrize("kwargs, message", [
    ({"policy_kwargs": {"activation_fn": nn.ELU}}, "unsupported layer"),
    ({"use_sde": True}, "action distribution"),
])
def test_architectures_it_cannot_reproduce_are_refused(kwargs, message, tmp_path):
    model = PPO("MlpPolicy", SarStandInEnv(), seed=4, device="cpu", n_steps=64, batch_size=32, **kwargs)
    model.save(tmp_path / "p.zip")
    with pytest.raises(ValueError, match=message):
        npp.export_policy(tmp_path / "p.zip", tmp_path / "p.npz")
    assert not (tmp_path / "p.npz").exists()


def test_export_never_overwrites(tmp_path):
    model = PPO("MlpPolicy", SarStandInEnv(), seed=4, device="cpu", n_steps=64, batch_size=32)
    _, _, _, out = export(model, tmp_path)
    with pytest.raises(FileExistsError):
        npp.export_policy(tmp_path / "policy.zip", out)


def rewrite(src, dst, **changes):
    with np.load(src, allow_pickle=False) as z:
        arrays = {k: z[k] for k in z.files}
    arrays.update(changes)
    with open(dst, "xb") as f:
        np.savez(f, **arrays)
    return dst


@pytest.fixture
def exported(tmp_path):
    model = PPO("MlpPolicy", SarStandInEnv(), seed=4, device="cpu", n_steps=64, batch_size=32)
    return export(model, tmp_path)[3]


def test_load_refuses_a_wrong_format(exported, tmp_path):
    meta = json.loads(str(np.load(exported)["meta"]))
    bad = rewrite(exported, tmp_path / "bad.npz", meta=np.array(json.dumps({**meta, "format": "other"})))
    with pytest.raises(ValueError, match="not a sar_numpy_mlp_v1 file"):
        npp.NumpyPolicy.load(bad)


@pytest.mark.parametrize("change, message", [
    (lambda z: {"w1": z["w1"][:, :10]}, "does not follow width"),
    (lambda z: {"w0": np.where(np.arange(z["w0"].size).reshape(z["w0"].shape) == 0, np.nan, z["w0"])}, "non-finite"),
    (lambda z: {"action_low": np.zeros(3, np.float32), "action_high": np.ones(3, np.float32)}, "action dim"),
])
def test_load_refuses_inconsistent_or_non_finite_weights(exported, tmp_path, change, message):
    with np.load(exported) as z:
        bad = rewrite(exported, tmp_path / "bad.npz", **change(z))
    with pytest.raises(ValueError, match=message):
        npp.NumpyPolicy.load(bad)


def test_predict_refuses_a_wrong_observation_shape(exported):
    with pytest.raises(ValueError, match="observation shape"):
        npp.NumpyPolicy.load(exported).predict(np.zeros(49, np.float32))


def test_runner_npz_path_never_imports_torch(exported, tmp_path):
    log = tmp_path / "episode.jsonl"
    code = ("import sys, run_target_reaching_episode as r\n"
            f"rc = r.main(['--standin', '--policy', {str(exported)!r}, '--log', {str(log)!r}, '--max-steps', '20'])\n"
            "print('TORCH' if 'torch' in sys.modules else 'NO_TORCH', rc)\n")
    p = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, cwd=ROOT, timeout=120)
    assert p.returncode == 0 and "NO_TORCH" in p.stdout, p.stdout + p.stderr
    recs = [json.loads(line) for line in log.read_text().splitlines()]
    loaded = recs[0]
    assert loaded["event"] == "policy_loaded" and loaded["format"] == "numpy" and loaded["torch_loaded"] is False
    assert loaded["torch_threads"] is None and loaded["numpy_export"]["source_sha256"] == npp.file_sha256(tmp_path / "policy.zip")
    assert all("policy" in e for e in recs if e["event"] == "step")


def test_training_exports_its_models(tmp_path):
    import train_ppo
    run_dir = tmp_path / "ppo"
    train_ppo.main(["--total-timesteps", "256", "--n-envs", "1", "--n-steps", "128", "--batch-size", "64",
                    "--checkpoint-freq", "128", "--eval-freq", "128", "--eval-episodes", "1",
                    "--final-eval-episodes", "1", "--seed", "3", "--run-dir", str(run_dir)])
    config = json.loads((run_dir / "config.json").read_text())
    assert config["numpy_exports"] == {"final_model": "final_policy_numpy.npz", "best_model": "best_policy_numpy.npz"}
    for name in ("final", "best"):
        model = PPO.load(run_dir / f"{name}_model.zip", device="cpu")
        assert_parity(model, npp.NumpyPolicy.load(run_dir / f"{name}_policy_numpy.npz"),
                      observations(model, n_random=200, episodes=1))
