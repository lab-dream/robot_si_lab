"""MuJoCo wrapper for the OMY tabletop scene.

Plain Python, no ROS: the ROS 2 bridge node and the tests both build on this class.
"""
from pathlib import Path

import mujoco
import numpy as np

SCENE_XML = Path(__file__).resolve().parent.parent / "assets" / "robotis_omy" / "scene_tabletop.xml"

ARM_JOINTS = [f"Joint{i}" for i in range(1, 7)]
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 1.0  # actuator range is 0..1.1; 1.0 squeezes a 4 cm cube
CONTROL_HZ = 20

# Tool pointing straight down: tcp -y axis = world -z, finger spread (tcp x) along world y.
R_TOOL_DOWN = np.array([[0.0, 0.0, 1.0],
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0]])
HOME_POS = np.array([0.3, 0.0, 0.25])

TF_BODIES = ["base_unit"] + [f"link{i}" for i in range(1, 7)] + ["cube", "target"]
TF_TREE = ([("world", "base_unit"), ("base_unit", "link1")]
           + [(f"link{i}", f"link{i + 1}") for i in range(1, 6)]
           + [("link6", "tcp"), ("link6", "wrist_cam"), ("world", "front_cam"),
              ("world", "cube"), ("world", "target")])


class OmySim:
    def __init__(self, xml_path=SCENE_XML, image_size=(256, 256)):
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)
        self.n_substeps = int(round(1.0 / CONTROL_HZ / self.model.opt.timestep))
        self.qpos_idx = np.array([self.model.joint(j).qposadr[0] for j in ARM_JOINTS])
        self.dof_idx = np.array([self.model.joint(j).dofadr[0] for j in ARM_JOINTS])
        self.tcp_id = self.model.site("tcp").id
        self.cube_qadr = self.model.joint("cube").qposadr[0]
        self.target_id = self.model.body("target").id
        self.image_size = image_size
        self._renderer = None
        self.home_q = self.solve_ik(HOME_POS, R_TOOL_DOWN, q_init=np.array([0, -0.5, 1.5, 0.6, 0, 0.0]))
        self.reset()

    # ---------------- state ----------------
    def reset(self, seed=None):
        """Robot at home, gripper open, cube at a random spot in front of the robot."""
        rng = np.random.default_rng(seed)
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qpos_idx] = self.home_q
        cube_xy = np.array([rng.uniform(0.28, 0.42), rng.uniform(-0.05, 0.15)])
        self.data.qpos[self.cube_qadr:self.cube_qadr + 7] = [*cube_xy, 0.02, 1, 0, 0, 0]
        self.set_command(self.home_q, GRIPPER_OPEN)
        mujoco.mj_forward(self.model, self.data)
        self.step(10)  # let the cube settle

    def set_command(self, q_arm, gripper):
        self.data.ctrl[:6] = q_arm
        self.data.ctrl[6] = gripper

    def step(self, n_control_steps=1):
        for _ in range(n_control_steps * self.n_substeps):
            # Gravity compensation on the arm, as the real OMY controller does.
            self.data.qfrc_applied[self.dof_idx] = self.data.qfrc_bias[self.dof_idx]
            mujoco.mj_step(self.model, self.data)

    def joint_positions(self):
        return self.data.qpos[self.qpos_idx].copy()

    def gripper_position(self):
        return float(self.data.qpos[self.model.joint("rh_r1").qposadr[0]])

    def tcp_pose(self):
        return self.data.site_xpos[self.tcp_id].copy(), self.data.site_xmat[self.tcp_id].reshape(3, 3).copy()

    def cube_position(self):
        return self.data.qpos[self.cube_qadr:self.cube_qadr + 3].copy()

    def target_position(self):
        return self.data.xpos[self.target_id].copy()

    def is_success(self):
        """Cube resting on the target pad and the gripper open."""
        cube, target = self.cube_position(), self.target_position()
        return bool(np.linalg.norm(cube[:2] - target[:2]) < 0.04 and cube[2] < 0.03
                    and self.data.ctrl[6] < 0.1)

    # ---------------- kinematics ----------------
    def solve_ik(self, pos, rot, q_init=None, iters=200, tol=1e-4):
        """Damped least squares IK for the tcp site. Works on a scratch copy of the data."""
        d = mujoco.MjData(self.model)
        d.qpos[:] = self.data.qpos
        d.qpos[self.qpos_idx] = self.joint_positions() if q_init is None else q_init
        jacp, jacr = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
        for _ in range(iters):
            mujoco.mj_kinematics(self.model, d)
            mujoco.mj_comPos(self.model, d)
            err_p = pos - d.site_xpos[self.tcp_id]
            err_r = _rot_error(rot, d.site_xmat[self.tcp_id].reshape(3, 3))
            err = np.concatenate([err_p, err_r])
            if np.linalg.norm(err) < tol:
                break
            mujoco.mj_jacSite(self.model, d, jacp, jacr, self.tcp_id)
            J = np.vstack([jacp, jacr])[:, self.dof_idx]
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), err)
            d.qpos[self.qpos_idx] += np.clip(dq, -0.2, 0.2)
        return d.qpos[self.qpos_idx].copy()

    # ---------------- frames (TF) ----------------
    def frames(self):
        """Frame tree for TF: list of (parent, child, position, quaternion wxyz) in the parent frame.

        world -> base_unit -> link1 -> ... -> link6 -> tcp, link6 -> wrist_cam,
        world -> front_cam, world -> cube, world -> target. Camera frames follow the ROS optical
        convention (z forward, x right, y down), so they match the image header frame_id.
        """
        world = (np.zeros(3), np.eye(3))
        poses = {"world": world}
        for name in TF_BODIES:
            b = self.model.body(name).id
            poses[name] = (self.data.xpos[b], self.data.xmat[b].reshape(3, 3))
        poses["tcp"] = self.tcp_pose()
        for cam in ("front_cam", "wrist_cam"):
            c = self.model.camera(cam).id
            # MuJoCo cameras look along -z with y up; flip y and z for the optical frame
            poses[cam] = (self.data.cam_xpos[c], self.data.cam_xmat[c].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0]))
        out = []
        for parent, child in TF_TREE:
            (pp, Rp), (pc, Rc) = poses[parent], poses[child]
            q = np.zeros(4)
            mujoco.mju_mat2Quat(q, (Rp.T @ Rc).flatten())
            out.append((parent, child, Rp.T @ (pc - pp), q))
        return out

    # ---------------- cameras ----------------
    def render(self, camera):
        if self._renderer is None:
            h, w = self.image_size
            self._renderer = mujoco.Renderer(self.model, h, w)
        self._renderer.update_scene(self.data, camera=camera)
        return self._renderer.render().copy()

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None


def _rot_error(R_target, R_current):
    """Axis-angle vector (world frame) that rotates R_current onto R_target."""
    q_t, q_c, q_e = np.zeros(4), np.zeros(4), np.zeros(4)
    mujoco.mju_mat2Quat(q_t, R_target.flatten())
    mujoco.mju_mat2Quat(q_c, R_current.flatten())
    mujoco.mju_negQuat(q_c, q_c)
    mujoco.mju_mulQuat(q_e, q_t, q_c)
    v = np.zeros(3)
    mujoco.mju_quat2Vel(v, q_e, 1.0)
    return v
