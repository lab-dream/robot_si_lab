"""ROS 2 node that drives omy_sim with a trained policy, in place of the keyboard teleop.

Run in venv_lerobot with ROS 2 sourced (rclpy comes from /opt/ros, torch and lerobot from the venv):
  source /opt/ros/jazzy/setup.bash
  source ~/robot_si/venv_lerobot/bin/activate
  python tools/ros_policy.py outputs/act_omy/checkpoints/last/pretrained_model

Subscribes /joint_states, /front_cam/image_raw, /wrist_cam/image_raw and /omy/episode;
publishes /joint_command. omy_sim sends the three observation topics with one header stamp every
50 ms, so the node answers once per stamp, when it holds all three. /omy/reset starts a new
episode: the node drops the queued action chunk. Ctrl+C stops it; omy_sim then holds the last command.
"""
import argparse

import numpy as np
import rclpy
from policy_runner import PolicyRunner
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray, Int32


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


class PolicyNode(Node):
    def __init__(self, runner):
        super().__init__("policy")
        self.runner = runner
        self.latest = {}       # part -> (stamp, value)
        self.last_stamp = None
        self.episode = None
        self.steps = 0
        self.cmd_pub = self.create_publisher(Float64MultiArray, "/joint_command", 10)
        self.create_subscription(JointState, "/joint_states", self.on_state, qos_profile_sensor_data)
        for cam in ("front", "wrist"):
            self.create_subscription(Image, f"/{cam}_cam/image_raw",
                                     lambda msg, cam=cam: self.on_image(cam, msg), qos_profile_sensor_data)
        self.create_subscription(Int32, "/omy/episode", self.on_episode, 10)
        self.get_logger().info(f"{runner.type} policy on {runner.device}, waiting for omy_sim topics")

    def on_episode(self, msg):
        if msg.data != self.episode:
            if self.episode is not None:
                self.get_logger().info(f"episode {msg.data}: policy reset")
            self.episode = msg.data
            self.runner.reset()

    def on_state(self, msg):
        self.latest["state"] = (stamp_ns(msg), np.asarray(msg.position, dtype=np.float32))
        self.try_act()

    def on_image(self, cam, msg):
        img = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, 3).copy()
        self.latest[cam] = (stamp_ns(msg), img)
        self.try_act()

    def try_act(self):
        if len(self.latest) < 3:
            return
        stamps = {s for s, _ in self.latest.values()}
        if len(stamps) != 1 or self.last_stamp in stamps:
            return  # parts of different control steps, or this step is already answered
        self.last_stamp = stamps.pop()
        a = self.runner.act(self.latest["state"][1], self.latest["front"][1], self.latest["wrist"][1])
        self.cmd_pub.publish(Float64MultiArray(data=a.tolist()))
        self.steps += 1
        if self.steps == 1:
            self.get_logger().info("first command sent; try: ros2 service call /omy/reset std_srvs/srv/Trigger")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("policy", help="pretrained_model folder from lerobot-train")
    ap.add_argument("--device", default=None, help="cuda or cpu (default: cuda if available)")
    args = ap.parse_args()
    runner = PolicyRunner(args.policy, args.device)
    rclpy.init()
    node = PolicyNode(runner)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
