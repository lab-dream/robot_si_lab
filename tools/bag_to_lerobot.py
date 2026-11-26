"""Convert a rosbag2 recorded from omy_sim into a LeRobot dataset (no ROS needed, run in venv_lerobot).

  python tools/bag_to_lerobot.py bags/demos datasets/omy_pick_place

One LeRobot frame = the /joint_states, /omy/action and two camera images that share a header stamp.
Episodes are split by /omy/episode. Frames of the episode that was already running when the
recording started are dropped, so every kept episode begins with an /omy/reset inside the bag.
"""
import argparse
import shutil
from collections import defaultdict
from pathlib import Path

import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from rosbags.highlevel import AnyReader

FPS = 20
TASK = "Pick up the red cube and place it on the blue pad."
JOINT_NAMES = [f"Joint{i}" for i in range(1, 7)] + ["gripper"]
STAMPED = {"/joint_states": "state", "/omy/action": "action",
           "/front_cam/image_raw": "front", "/wrist_cam/image_raw": "wrist"}
EPISODE_TOPIC = "/omy/episode"


def read_bag(bag_path):
    """Return {episode: [frame, ...]} with frames in time order."""
    frames = defaultdict(dict)  # stamp -> {part: value, "episode": int}
    episode = None
    first_episode = None  # running when the recording started: no reset in the bag, dropped
    with AnyReader([Path(bag_path)]) as reader:
        conns = [c for c in reader.connections if c.topic in STAMPED or c.topic == EPISODE_TOPIC]
        missing = (set(STAMPED) | {EPISODE_TOPIC}) - {c.topic for c in conns}
        if missing:
            raise SystemExit(f"bag is missing topics: {sorted(missing)}")
        for conn, _, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic == EPISODE_TOPIC:
                episode = msg.data
                if first_episode is None:
                    first_episode = episode
                continue
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            frame = frames[stamp]
            frame.setdefault("episode", episode)
            part = STAMPED[conn.topic]
            if part in ("state", "action"):
                frame[part] = np.asarray(msg.position, dtype=np.float32)
            else:
                frame[part] = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)

    episodes = defaultdict(list)
    incomplete = 0
    for stamp in sorted(frames):
        f = frames[stamp]
        if f["episode"] is None or f["episode"] == first_episode:
            continue
        if not all(k in f for k in STAMPED.values()):
            incomplete += 1
            continue
        episodes[f["episode"]].append(f)
    if incomplete:
        print(f"skipped {incomplete} incomplete frames (a topic was dropped at that stamp)")
    return episodes


def features(image_shape):
    img = {"dtype": "image", "shape": image_shape, "names": ["height", "width", "channels"]}
    return {
        "observation.state": {"dtype": "float32", "shape": (7,), "names": JOINT_NAMES},
        "action": {"dtype": "float32", "shape": (7,), "names": JOINT_NAMES},
        "observation.images.front": img,
        "observation.images.wrist": img,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="rosbag2 folder")
    ap.add_argument("out", help="output dataset folder")
    ap.add_argument("--repo-id", default="local/omy_pick_place")
    ap.add_argument("--min-frames", type=int, default=20, help="drop episodes shorter than this")
    ap.add_argument("--overwrite", action="store_true")
    args = ap.parse_args()

    out = Path(args.out)
    if out.exists():
        if not args.overwrite:
            raise SystemExit(f"{out} exists; pass --overwrite to replace it")
        shutil.rmtree(out)

    episodes = read_bag(args.bag)
    if not episodes:
        raise SystemExit("no complete frames found inside an episode; did you call /omy/reset while recording?")
    image_shape = next(iter(episodes.values()))[0]["front"].shape

    ds = LeRobotDataset.create(repo_id=args.repo_id, fps=FPS, features=features(image_shape),
                               root=out, robot_type="omy", use_videos=False)
    for ep_idx in sorted(episodes):
        frames = episodes[ep_idx]
        if len(frames) < args.min_frames:
            print(f"episode {ep_idx}: {len(frames)} frames, too short, skipped")
            continue
        for f in frames:
            ds.add_frame({
                "observation.state": f["state"],
                "action": f["action"],
                "observation.images.front": f["front"],
                "observation.images.wrist": f["wrist"],
                "task": TASK,
            })
        ds.save_episode()
        print(f"episode {ep_idx}: {len(frames)} frames saved")
    ds.finalize()
    print(f"dataset written to {out}: {ds.num_episodes} episodes, {ds.num_frames} frames")


if __name__ == "__main__":
    main()
