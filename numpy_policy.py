"""Torch-free flight copy of a stable-baselines3 PPO policy (deterministic actions only).

Training keeps using stable-baselines3/torch with whatever MLP works best; ``export_policy`` then writes the
action path of the checkpoint to a small ``.npz`` (format ``sar_numpy_mlp_v1``) that the flight runner evaluates
with numpy alone, saving the ~300 MiB that importing torch costs in the flight process.

What is reproduced, exactly as ``PPO.predict(obs, deterministic=True)`` computes it for these checkpoints:
FlattenExtractor (float32 observation, no normalisation), ``mlp_extractor.policy_net`` (any sequence of Linear
layers with Tanh or ReLU), ``action_net`` (Linear), the DiagGaussian mean as the deterministic action, then
clipping to the action-space bounds. Anything else (squashed output, other distributions or extractors, other
activations or layer types, a separate actor feature extractor) is refused at export rather than approximated.
``test_numpy_policy.py`` checks parity with stable-baselines3 on stand-in episodes and random observations.

Usage:
    .venv/bin/python numpy_policy.py export <checkpoint.zip> <out.npz>
"""

from __future__ import annotations

import datetime
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

FORMAT = "sar_numpy_mlp_v1"
ACTIVATIONS = {"tanh": np.tanh, "relu": lambda x: np.maximum(x, 0.0)}


def file_sha256(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def _layers_from_sb3(model):
    """[(W, b, activation or None)] for the action path of an SB3 ActorCriticPolicy, or ValueError."""
    import torch.nn as nn
    from stable_baselines3.common.distributions import DiagGaussianDistribution
    from stable_baselines3.common.policies import ActorCriticPolicy
    from stable_baselines3.common.torch_layers import FlattenExtractor

    p = model.policy
    problems = []
    if type(p) is not ActorCriticPolicy:
        problems.append(f"policy class {type(p).__name__}")
    if p.squash_output:
        problems.append("squashed output")
    if not isinstance(p.action_dist, DiagGaussianDistribution):
        problems.append(f"action distribution {type(p.action_dist).__name__}")
    extractor = p.features_extractor if p.share_features_extractor else p.pi_features_extractor
    if type(extractor) is not FlattenExtractor:
        problems.append(f"features extractor {type(extractor).__name__}")
    if getattr(p, "normalize_images", False) and len(model.observation_space.shape) != 1:
        problems.append("image observation")
    children = dict(p.mlp_extractor.named_children())
    if set(children) != {"policy_net", "value_net"}:
        problems.append(f"mlp_extractor parts {sorted(children)}")
    layers, pending = [], None
    for module in list(children.get("policy_net", [])) + [p.action_net]:
        if isinstance(module, nn.Linear) and module.bias is not None:
            if pending is not None:
                layers.append((*pending, None))
            pending = (module.weight.detach().cpu().numpy().astype(np.float32),
                       module.bias.detach().cpu().numpy().astype(np.float32))
        elif isinstance(module, (nn.Tanh, nn.ReLU)) and pending is not None:
            layers.append((*pending, "tanh" if isinstance(module, nn.Tanh) else "relu"))
            pending = None
        else:
            problems.append(f"unsupported layer {module!r}")
    if pending is None:
        problems.append("action_net is not a final Linear layer")
    else:
        layers.append((*pending, None))
    if problems:
        raise ValueError("cannot export exactly: " + "; ".join(problems))
    return layers


def export_policy(checkpoint, out_path):
    """Write the numpy copy of an SB3 PPO checkpoint; refuses to overwrite. Returns the metadata dict."""
    import stable_baselines3
    import torch
    from stable_baselines3 import PPO

    checkpoint, out_path = Path(checkpoint), Path(out_path)
    model = PPO.load(str(checkpoint), device="cpu")
    layers = _layers_from_sb3(model)
    meta = {
        "format": FORMAT,
        "source": str(checkpoint.resolve()),
        "source_sha256": file_sha256(checkpoint),
        "exported_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "stable_baselines3": stable_baselines3.__version__,
        "torch": torch.__version__,
        "activations": [a for _, _, a in layers],
        "layer_shapes": [list(w.shape) for w, _, _ in layers],
    }
    arrays = {"obs_low": model.observation_space.low.astype(np.float32),
              "obs_high": model.observation_space.high.astype(np.float32),
              "action_low": model.action_space.low.astype(np.float32),
              "action_high": model.action_space.high.astype(np.float32),
              "meta": np.array(json.dumps(meta))}
    for i, (w, b, _) in enumerate(layers):
        arrays[f"w{i}"], arrays[f"b{i}"] = w, b
    with open(out_path, "xb") as f:
        np.savez(f, **arrays)
    return meta


class _Box:
    def __init__(self, low, high):
        self.low, self.high, self.shape = low, high, low.shape


class NumpyPolicy:
    """predict(obs) -> deterministic action, numpy only. Same interface checks as the SB3 path (spaces)."""

    def __init__(self, layers, obs_low, obs_high, action_low, action_high, meta):
        self.layers = layers
        self.observation_space = _Box(obs_low, obs_high)
        self.action_space = _Box(action_low, action_high)
        self.meta = meta

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as z:
            meta = json.loads(str(z["meta"]))
            if meta.get("format") != FORMAT:
                raise ValueError(f"not a {FORMAT} file: format {meta.get('format')!r}")
            n = len(meta["activations"])
            layers = [(z[f"w{i}"].astype(np.float32), z[f"b{i}"].astype(np.float32), meta["activations"][i])
                      for i in range(n)]
            spaces = [z[k].astype(np.float32) for k in ("obs_low", "obs_high", "action_low", "action_high")]
        problems = []
        width = spaces[0].shape[0]
        for i, (w, b, act) in enumerate(layers):
            if w.ndim != 2 or w.shape[1] != width or b.shape != (w.shape[0],):
                problems.append(f"layer {i} shape {w.shape}/{b.shape} does not follow width {width}")
                break
            if act not in ACTIVATIONS and not (act is None and i == n - 1):
                problems.append(f"layer {i} activation {act!r}")
            if not (np.all(np.isfinite(w)) and np.all(np.isfinite(b))):
                problems.append(f"layer {i} has non-finite weights")
            width = w.shape[0]
        if not layers or layers[-1][2] is not None:
            problems.append("missing final linear layer")
        elif width != spaces[2].shape[0]:
            problems.append(f"output width {width} != action dim {spaces[2].shape[0]}")
        if problems:
            raise ValueError(f"invalid {FORMAT} file {path}: " + "; ".join(problems))
        return cls(layers, *spaces, meta)

    def predict(self, obs):
        x = np.asarray(obs, dtype=np.float32).reshape(-1)
        if x.shape != self.observation_space.shape:
            raise ValueError(f"observation shape {x.shape} != {self.observation_space.shape}")
        for w, b, act in self.layers:
            x = w @ x + b
            if act is not None:
                x = ACTIVATIONS[act](x)
        return np.clip(x, self.action_space.low, self.action_space.high).astype(np.float32)


def main(argv):
    if len(argv) != 4 or argv[1] != "export":
        print(__doc__, file=sys.stderr)
        return 2
    print(json.dumps(export_policy(argv[2], argv[3]), indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
