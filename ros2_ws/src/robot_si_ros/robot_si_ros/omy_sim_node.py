"""MuJoCo <-> ROS 2 bridge for the OMY tabletop scene.

Publishes (every 50 ms, all with the same header stamp):
  /joint_states            sensor_msgs/JointState   Joint1..6 + gripper (observation)
  /omy/action              sensor_msgs/JointState   command applied this step (action)
  /front_cam/image_raw     sensor_msgs/Image        rgb8
  /wrist_cam/image_raw     sensor_msgs/Image        rgb8
  /omy/episode             std_msgs/Int32           episode index (+1 on every /omy/reset)
  /tf                      tf2_msgs/TFMessage       world, base_unit, link1..6, tcp, cameras, cube, target
Subscribes:
  /joint_command           std_msgs/Float64MultiArray  [j1..j6, gripper] absolute joint targets
  /omy/tcp_delta           std_msgs/Float64MultiArray  [dx, dy, dz, gripper] tcp move in metres (teleop)
Services (std_srvs/Trigger, reply when the motion is finished):
  /omy/go_home, /omy/pick_place, /omy/reset
"""
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray, Int32
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from robot_si import tasks
from robot_si.sim import ARM_JOINTS, CONTROL_HZ, GRIPPER_OPEN, R_TOOL_DOWN, OmySim

JOINT_NAMES = ARM_JOINTS + ["gripper"]
CAMERAS = ["front_cam", "wrist_cam"]
TCP_MIN = np.array([0.15, -0.35, 0.015])
TCP_MAX = np.array([0.55, 0.35, 0.35])


class OmySimNode(Node):
    def __init__(self):
        super().__init__("omy_sim")
        self.declare_parameter("image_size", 128)
        self.declare_parameter("publish_images", True)
        self.declare_parameter("viewer", False)
        self.declare_parameter("seed", -1)
        size = self.get_parameter("image_size").value
        self.publish_images = self.get_parameter("publish_images").value
        seed = self.get_parameter("seed").value
        self.rng = np.random.default_rng(None if seed < 0 else seed)

        self.sim = OmySim(image_size=(size, size))
        self.lock = threading.Lock()
        self.stream = None            # active task: iterator of (q, gripper)
        self.stream_done = threading.Event()
        self.pending_reset = False
        self.tcp_deltas = []
        self.episode = -1
        self.q_cmd = self.sim.home_q.copy()
        self.g_cmd = GRIPPER_OPEN

        cb = ReentrantCallbackGroup()
        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.act_pub = self.create_publisher(JointState, "/omy/action", 10)
        self.ep_pub = self.create_publisher(Int32, "/omy/episode", 10)
        self.img_pubs = {c: self.create_publisher(Image, f"/{c}/image_raw", 10) for c in CAMERAS}
        self.tf_pub = TransformBroadcaster(self)
        self.create_subscription(Float64MultiArray, "/joint_command", self.on_joint_command, 10, callback_group=cb)
        self.create_subscription(Float64MultiArray, "/omy/tcp_delta", self.on_tcp_delta, 10, callback_group=cb)
        self.create_service(Trigger, "/omy/go_home", self.srv_go_home, callback_group=cb)
        self.create_service(Trigger, "/omy/pick_place", self.srv_pick_place, callback_group=cb)
        self.create_service(Trigger, "/omy/reset", self.srv_reset, callback_group=cb)

        self.viewer = None
        if self.get_parameter("viewer").value:
            import mujoco.viewer
            self.viewer = mujoco.viewer.launch_passive(self.sim.model, self.sim.data)
        self.get_logger().info("OMY simulator ready. Try: ros2 service call /omy/pick_place std_srvs/srv/Trigger")

    # ---------------- commands ----------------
    def on_joint_command(self, msg):
        if len(msg.data) != 7:
            self.get_logger().warn("/joint_command needs 7 values: j1..j6, gripper")
            return
        with self.lock:
            self.stream = None
            self.q_cmd = np.array(msg.data[:6])
            self.g_cmd = float(msg.data[6])

    def on_tcp_delta(self, msg):
        if len(msg.data) != 4:
            self.get_logger().warn("/omy/tcp_delta needs 4 values: dx, dy, dz, gripper")
            return
        with self.lock:
            self.stream = None
            self.tcp_deltas.append(np.array(msg.data))

    def _run_task(self, make_stream, response, label):
        with self.lock:
            self.stream_done.clear()
            self.stream = make_stream()
        response.success = self.stream_done.wait(timeout=30.0)
        response.message = f"{label} done" if response.success else f"{label} interrupted or timed out"
        if label == "pick_place" and response.success:
            response.success = self.sim.is_success()
            response.message = "cube placed on target" if response.success else "cube not on target"
        return response

    def srv_go_home(self, request, response):
        return self._run_task(lambda: tasks.go_home(self.sim), response, "go_home")

    def srv_pick_place(self, request, response):
        return self._run_task(lambda: tasks.pick_and_place(self.sim), response, "pick_place")

    def srv_reset(self, request, response):
        with self.lock:
            self.stream_done.clear()
            self.pending_reset = True
        response.success = self.stream_done.wait(timeout=5.0)
        response.message = f"episode {self.episode}"
        return response

    # ---------------- simulation loop ----------------
    def loop(self):
        """One control step. Called from the main thread only: the OpenGL context lives there."""
        with self.lock:
            if self.pending_reset:
                self.pending_reset = False
                self.stream = None
                self.sim.reset(seed=int(self.rng.integers(1 << 31)))
                self.episode += 1
                self.q_cmd, self.g_cmd = self.sim.home_q.copy(), GRIPPER_OPEN
                self.stream_done.set()
            elif self.stream is not None:
                try:
                    self.q_cmd, self.g_cmd = next(self.stream)
                except StopIteration:
                    self.stream = None
                    self.stream_done.set()
            elif self.tcp_deltas:
                d = np.sum(self.tcp_deltas, axis=0)
                gripper = self.tcp_deltas[-1][3]
                self.tcp_deltas.clear()
                pos, _ = self.sim.tcp_pose()
                target = np.clip(pos + d[:3], TCP_MIN, TCP_MAX)
                self.q_cmd = self.sim.solve_ik(target, R_TOOL_DOWN, q_init=self.q_cmd, iters=30)
                self.g_cmd = float(gripper)
            q_cmd, g_cmd = self.q_cmd.copy(), self.g_cmd

        self.sim.set_command(q_cmd, g_cmd)
        self.sim.step()
        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()
        self.publish(q_cmd, g_cmd)

    def publish(self, q_cmd, g_cmd):
        stamp = self.get_clock().now().to_msg()
        self.ep_pub.publish(Int32(data=self.episode))
        js = JointState(name=JOINT_NAMES,
                        position=[*self.sim.joint_positions().tolist(), self.sim.gripper_position()])
        js.header.stamp = stamp
        act = JointState(name=JOINT_NAMES, position=[*q_cmd.tolist(), float(g_cmd)])
        act.header.stamp = stamp
        self.js_pub.publish(js)
        self.act_pub.publish(act)
        self.tf_pub.sendTransform([self._transform(stamp, *f) for f in self.sim.frames()])
        if self.publish_images:
            for cam, pub in self.img_pubs.items():
                img = self.sim.render(cam)
                msg = Image(height=img.shape[0], width=img.shape[1], encoding="rgb8",
                            step=img.shape[1] * 3, data=img.tobytes())
                msg.header.stamp = stamp
                msg.header.frame_id = cam
                pub.publish(msg)

    @staticmethod
    def _transform(stamp, parent, child, p, q):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, p)
        r = t.transform.rotation
        r.w, r.x, r.y, r.z = map(float, q)
        return t


def main():
    rclpy.init()
    node = OmySimNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    # Callbacks run in a background thread; the 20 Hz sim/render loop stays on this thread.
    threading.Thread(target=executor.spin, daemon=True).start()
    period = 1.0 / CONTROL_HZ
    next_tick = time.monotonic()
    try:
        while rclpy.ok():
            node.loop()
            next_tick += period
            time.sleep(max(0.0, next_tick - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.sim.close()
        node.destroy_node()
        rclpy.try_shutdown()
