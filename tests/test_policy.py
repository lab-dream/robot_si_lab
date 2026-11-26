"""Smoke test of tools/run_policy.py with an untrained ACT policy (venv_lerobot only; skipped in venv_ros).

  ~/robot_si/venv_lerobot/bin/python -m pytest -q tests/test_policy.py
"""
import importlib.util
import os
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import numpy as np
import pytest

# A skip mark, not a module-level importorskip: the launch_testing plugin that comes with a sourced
# ROS 2 turns a skip raised at import time into a skip of the whole session.
pytestmark = pytest.mark.skipif(importlib.util.find_spec("lerobot") is None, reason="needs venv_lerobot")

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))


@pytest.fixture(scope="module")
def checkpoint(tmp_path_factory):
    """A tiny random ACT policy saved the way lerobot-train saves pretrained_model."""
    import torch
    from lerobot.configs.types import FeatureType, PolicyFeature
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors

    img = PolicyFeature(type=FeatureType.VISUAL, shape=(3, 128, 128))
    cfg = ACTConfig(
        input_features={"observation.state": PolicyFeature(type=FeatureType.STATE, shape=(7,)),
                        "observation.images.front": img, "observation.images.wrist": img},
        output_features={"action": PolicyFeature(type=FeatureType.ACTION, shape=(7,))},
        chunk_size=10, n_action_steps=10, dim_model=64, dim_feedforward=128, n_heads=2,
        n_encoder_layers=1, n_decoder_layers=1, pretrained_backbone_weights=None, device="cpu")
    vec = {"mean": torch.zeros(7), "std": torch.ones(7), "min": -torch.ones(7), "max": torch.ones(7)}
    pix = {k: torch.full((3, 1, 1), v) for k, v in [("mean", 0.5), ("std", 0.25), ("min", 0.0), ("max", 1.0)]}
    stats = {"observation.state": vec, "action": vec,
             "observation.images.front": pix, "observation.images.wrist": pix}
    out = tmp_path_factory.mktemp("act") / "pretrained_model"
    ACTPolicy(cfg).save_pretrained(out)
    pre, post = make_pre_post_processors(cfg, dataset_stats=stats)
    pre.save_pretrained(out)
    post.save_pretrained(out)
    return out


def test_runner_returns_joint_targets(checkpoint):
    from policy_runner import PolicyRunner

    runner = PolicyRunner(checkpoint, device="cpu")
    runner.reset()
    img = np.zeros((128, 128, 3), dtype=np.uint8)
    a = runner.act(np.zeros(7), img, img)
    assert a.shape == (7,) and np.all(np.isfinite(a))


def test_episode_loop_runs_in_sim(checkpoint):
    from policy_runner import PolicyRunner
    from robot_si.sim import OmySim
    from run_policy import run_episode

    runner = PolicyRunner(checkpoint, device="cpu")
    sim = OmySim(image_size=(128, 128))
    frames = []
    ok, steps = run_episode(sim, runner, seed=0, max_steps=15, frames=frames)
    sim.close()
    assert steps == 15 and not ok  # an untrained policy does not place the cube
    assert len(frames) == 15 and frames[0].shape == (128, 128, 3)
