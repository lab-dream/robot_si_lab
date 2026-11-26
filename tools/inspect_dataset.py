"""Load a converted LeRobot dataset, print a quality report and save a preview GIF per episode.

  python tools/inspect_dataset.py datasets/omy_pick_place
"""
import argparse
from pathlib import Path

import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from PIL import Image


def to_uint8(img):
    """LeRobot returns images as float CHW tensors in [0, 1]."""
    arr = img.numpy() if hasattr(img, "numpy") else np.asarray(img)
    if arr.shape[0] == 3:
        arr = arr.transpose(1, 2, 0)
    return (arr * 255).clip(0, 255).astype(np.uint8) if arr.dtype != np.uint8 else arr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root", help="dataset folder")
    ap.add_argument("--repo-id", default="local/omy_pick_place")
    ap.add_argument("--gif-dir", default=None, help="where to write previews (default: <root>/previews)")
    args = ap.parse_args()

    ds = LeRobotDataset(args.repo_id, root=args.root)
    print(f"episodes: {ds.num_episodes}, frames: {ds.num_frames}, fps: {ds.fps}")
    print(f"features: {sorted(ds.features)}")
    gif_dir = Path(args.gif_dir or Path(args.root) / "previews")
    gif_dir.mkdir(parents=True, exist_ok=True)

    ep_index = np.asarray(ds.hf_dataset["episode_index"])
    for ep in range(ds.num_episodes):
        idx = np.flatnonzero(ep_index == ep)
        state = np.stack([ds[int(i)]["observation.state"].numpy() for i in idx])
        action = np.stack([ds[int(i)]["action"].numpy() for i in idx])
        lag = np.abs(action[:, :6] - state[:, :6]).max()
        jumps = np.abs(np.diff(action[:, :6], axis=0)).max() if len(idx) > 1 else 0.0
        grip_changes = int(np.count_nonzero(np.diff((action[:, 6] > 0.5).astype(int))))
        print(f"episode {ep}: {len(idx)} frames ({len(idx) / ds.fps:.1f} s), "
              f"max |action-state| {lag:.3f} rad, max action jump {jumps:.3f} rad/step, "
              f"gripper open/close events {grip_changes}")

        frames = []
        for i in idx[::2]:
            item = ds[int(i)]
            frames.append(Image.fromarray(np.hstack([to_uint8(item["observation.images.front"]),
                                                     to_uint8(item["observation.images.wrist"])])))
        gif = gif_dir / f"episode_{ep:03d}.gif"
        frames[0].save(gif, save_all=True, append_images=frames[1:], duration=100, loop=0)
        print(f"  preview: {gif}")


if __name__ == "__main__":
    main()
