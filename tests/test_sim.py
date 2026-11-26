import os

os.environ.setdefault("MUJOCO_GL", "egl")

import numpy as np
import pytest

from robot_si import tasks
from robot_si.sim import R_TOOL_DOWN, OmySim


@pytest.fixture(scope="module")
def sim():
    s = OmySim(image_size=(128, 128))
    yield s
    s.close()


def test_home_pose_points_tool_down(sim):
    sim.reset(seed=0)
    pos, rot = sim.tcp_pose()
    assert np.allclose(pos, [0.3, 0.0, 0.25], atol=0.01)
    assert np.allclose(rot, R_TOOL_DOWN, atol=0.05)


@pytest.mark.parametrize("target", [[0.28, -0.05, 0.02], [0.42, 0.15, 0.02], [0.35, -0.2, 0.12]])
def test_ik_reaches_workspace(sim, target):
    sim.reset(seed=0)
    q = sim.solve_ik(np.array(target), R_TOOL_DOWN)
    sim.data.qpos[sim.qpos_idx] = q
    import mujoco
    mujoco.mj_forward(sim.model, sim.data)
    pos, _ = sim.tcp_pose()
    assert np.linalg.norm(pos - target) < 0.002


def test_cameras_render_images(sim):
    sim.reset(seed=0)
    for cam in ["front_cam", "wrist_cam"]:
        img = sim.render(cam)
        assert img.shape == (128, 128, 3)
        assert img.std() > 5  # not a blank frame


def test_joint_command_is_tracked(sim):
    sim.reset(seed=0)
    q_goal = sim.home_q + np.array([0.3, 0.1, -0.1, 0.0, 0.2, 0.0])
    sim.set_command(q_goal, 0.0)
    sim.step(40)  # 2 s
    assert np.abs(sim.joint_positions() - q_goal).max() < 0.02


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_scripted_pick_and_place_succeeds(sim, seed):
    sim.reset(seed=seed)
    tasks.run(sim, tasks.pick_and_place(sim))
    assert sim.is_success(), f"cube at {sim.cube_position()}"
    tasks.run(sim, tasks.go_home(sim))
    assert np.abs(sim.joint_positions() - sim.home_q).max() < 0.02


def test_tf_chain_reaches_tcp(sim):
    """Composing world -> base_unit -> link1 ... link6 -> tcp gives the tcp pose MuJoCo reports."""
    import mujoco
    sim.reset(seed=3)
    sim.data.qpos[sim.qpos_idx] = [0.3, -0.4, 1.1, 0.5, 1.2, -0.7]
    mujoco.mj_forward(sim.model, sim.data)
    tf = {child: (parent, p, q) for parent, child, p, q in sim.frames()}
    p, R = np.zeros(3), np.eye(3)
    chain, f = [], "tcp"
    while f != "world":
        chain.append(f)
        f = tf[f][0]
    for f in reversed(chain):
        _, pi, qi = tf[f]
        Ri = np.zeros(9)
        mujoco.mju_quat2Mat(Ri, qi)
        p, R = p + R @ pi, R @ Ri.reshape(3, 3)
    pos, rot = sim.tcp_pose()
    assert chain[-1] == "base_unit" and len(chain) == 8
    assert np.allclose(p, pos, atol=1e-9) and np.allclose(R, rot, atol=1e-9)
