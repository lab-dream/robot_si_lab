"""Load a trained LeRobot policy and turn one observation into one action (run in venv_lerobot).

Shared by run_policy.py (MuJoCo, no ROS) and ros_policy.py (ROS 2 node). The observation is the
same frame that bag_to_lerobot.py writes: 7 joint values and two 128x128 rgb images.
"""
from pathlib import Path

import numpy as np
import torch
from lerobot.common.control_utils import predict_action
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import get_policy_class, make_pre_post_processors

TASK = "Pick up the red cube and place it on the blue pad."  # same text as bag_to_lerobot.py


class PolicyRunner:
    def __init__(self, path, device=None):
        """path: a pretrained_model folder written by lerobot-train (any policy type: act, diffusion, ...)."""
        path = Path(path)
        if not (path / "config.json").exists():
            raise SystemExit(f"{path} has no config.json; pass the .../checkpoints/<step>/pretrained_model folder")
        self.device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
        cfg = PreTrainedConfig.from_pretrained(path)
        cfg.device = self.device.type
        self.policy = get_policy_class(cfg.type).from_pretrained(path, config=cfg)
        self.policy.to(self.device).eval()
        overrides = {"device_processor": {"device": self.device.type}}
        self.pre, self.post = make_pre_post_processors(cfg, pretrained_path=path,
                                                       preprocessor_overrides=overrides)
        self.type = cfg.type

    def reset(self):
        """Call at the start of every episode: drops the queued action chunk."""
        self.policy.reset()
        self.pre.reset()
        self.post.reset()

    def act(self, state, front, wrist):
        """state: (7,) joints + gripper, front/wrist: (H, W, 3) uint8. Returns (7,) joint targets."""
        obs = {"observation.state": np.asarray(state, dtype=np.float32),
               "observation.images.front": np.ascontiguousarray(front),
               "observation.images.wrist": np.ascontiguousarray(wrist)}
        action = predict_action(obs, self.policy, self.device, self.pre, self.post,
                                use_amp=False, task=TASK, robot_type="omy")
        return action.squeeze(0).cpu().numpy().astype(np.float64)
