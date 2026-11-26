"""Run a trained policy in the MuJoCo OMY scene and report the success rate (no ROS, run in venv_lerobot).

  python tools/run_policy.py outputs/act_omy/checkpoints/last/pretrained_model --episodes 10
  python tools/run_policy.py <pretrained_model> --episodes 3 --viewer        # watch it live
  python tools/run_policy.py <pretrained_model> --episodes 1 --gif outputs/policy.gif

Each step follows omy_sim: read joints and both cameras, ask the policy for the 7 joint targets,
apply them for one 50 ms control step. An episode succeeds when the cube rests on the pad with the
gripper open. Cube positions come from --seed, so two policies can be compared on the same cubes.
"""
import argparse
import time

from policy_runner import PolicyRunner
from robot_si.sim import CONTROL_HZ, OmySim


def observe(sim):
    state = [*sim.joint_positions(), sim.gripper_position()]
    return state, sim.render("front_cam"), sim.render("wrist_cam")


def run_episode(sim, runner, seed, max_steps, viewer=None, frames=None):
    sim.reset(seed=seed)
    runner.reset()
    for step in range(max_steps):
        state, front, wrist = observe(sim)
        if frames is not None:
            frames.append(front)
        a = runner.act(state, front, wrist)
        sim.set_command(a[:6], a[6])
        sim.step()
        if viewer is not None:
            viewer.sync()
            time.sleep(1.0 / CONTROL_HZ)
        if sim.is_success():
            return True, step + 1
    return False, max_steps


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("policy", help="pretrained_model folder from lerobot-train")
    ap.add_argument("--episodes", type=int, default=10)
    ap.add_argument("--max-steps", type=int, default=400, help="steps per episode (20 Hz, 400 = 20 s)")
    ap.add_argument("--seed", type=int, default=1000, help="cube position seed of the first episode")
    ap.add_argument("--image-size", type=int, default=128, help="must match the training images")
    ap.add_argument("--device", default=None, help="cuda or cpu (default: cuda if available)")
    ap.add_argument("--viewer", action="store_true", help="open the MuJoCo viewer, real-time speed")
    ap.add_argument("--gif", help="save the front camera of the first episode as a GIF")
    args = ap.parse_args()

    runner = PolicyRunner(args.policy, args.device)
    print(f"policy {runner.type} on {runner.device}")
    sim = OmySim(image_size=(args.image_size, args.image_size))
    viewer = None
    if args.viewer:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(sim.model, sim.data)

    n_success, frames = 0, [] if args.gif else None
    t0 = time.monotonic()
    for ep in range(args.episodes):
        ok, steps = run_episode(sim, runner, args.seed + ep, args.max_steps, viewer,
                                frames if ep == 0 else None)
        n_success += ok
        print(f"episode {ep} (seed {args.seed + ep}): {'success' if ok else 'fail'} after {steps / CONTROL_HZ:.1f} s")
    print(f"{n_success}/{args.episodes} episodes succeeded ({time.monotonic() - t0:.0f} s wall time)")

    if args.gif:
        from PIL import Image
        imgs = [Image.fromarray(f) for f in frames[::2]]
        imgs[0].save(args.gif, save_all=True, append_images=imgs[1:], duration=100, loop=0)
        print(f"gif: {args.gif} ({len(imgs)} frames)")
    if viewer is not None:
        viewer.close()
    sim.close()


if __name__ == "__main__":
    main()
