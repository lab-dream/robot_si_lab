"""Predefined tasks as streams of (arm joint command, gripper command), one per control step.

The ROS 2 task services and the scripted demo both replay these streams.
"""
import numpy as np

from robot_si.sim import CONTROL_HZ, GRIPPER_CLOSED, GRIPPER_OPEN, R_TOOL_DOWN, OmySim

APPROACH_Z = 0.12  # tcp height above the table while moving
GRASP_Z = 0.02     # tcp height at the cube center


def _cartesian_segment(sim, q_start, p_from, p_to, gripper, seconds):
    """Straight tcp line from p_from to p_to, tool kept pointing down."""
    n = max(1, int(seconds * CONTROL_HZ))
    q = q_start
    for s in np.linspace(0.0, 1.0, n + 1)[1:]:
        q = sim.solve_ik(p_from + s * (p_to - p_from), R_TOOL_DOWN, q_init=q, iters=50)
        yield q, gripper


def _hold(q, gripper, seconds):
    for _ in range(int(seconds * CONTROL_HZ)):
        yield q, gripper


def go_home(sim: OmySim, seconds=2.0):
    """Joint-space move from the current pose back to home, gripper open."""
    q0 = sim.joint_positions()
    n = int(seconds * CONTROL_HZ)
    for s in np.linspace(0.0, 1.0, n + 1)[1:]:
        s = 3 * s**2 - 2 * s**3  # smooth start and stop
        yield q0 + s * (sim.home_q - q0), GRIPPER_OPEN
    yield from _hold(sim.home_q, GRIPPER_OPEN, 1.0)  # the arm servos are overdamped; let it settle


def pick_and_place(sim: OmySim, pick=None, place=None):
    """Pick the cube (current position by default) and put it on the target pad."""
    pick = sim.cube_position() if pick is None else np.asarray(pick, float)
    place = sim.target_position() if place is None else np.asarray(place, float)
    above_pick = np.array([pick[0], pick[1], APPROACH_Z])
    at_pick = np.array([pick[0], pick[1], GRASP_Z])
    above_place = np.array([place[0], place[1], APPROACH_Z])
    at_place = np.array([place[0], place[1], GRASP_Z + 0.005])

    p_now, _ = sim.tcp_pose()
    q = sim.joint_positions()
    steps = [
        (p_now, above_pick, GRIPPER_OPEN, 1.5),
        (above_pick, at_pick, GRIPPER_OPEN, 1.0),
        None,  # close the gripper
        (at_pick, above_pick, GRIPPER_CLOSED, 1.0),
        (above_pick, above_place, GRIPPER_CLOSED, 1.5),
        (above_place, at_place, GRIPPER_CLOSED, 1.0),
        "open",
        (at_place, above_place, GRIPPER_OPEN, 1.0),
    ]
    for step in steps:
        if step is None:
            yield from _hold(q, GRIPPER_CLOSED, 0.8)
        elif step == "open":
            yield from _hold(q, GRIPPER_OPEN, 0.5)
        else:
            for q, g in _cartesian_segment(sim, q, *step):
                yield q, g


def run(sim: OmySim, stream, on_step=None):
    """Execute a command stream in the simulator (no ROS). on_step(sim, q, g) is called after each step."""
    for q, g in stream:
        sim.set_command(q, g)
        sim.step()
        if on_step is not None:
            on_step(sim, q, g)
