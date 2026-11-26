"""Keyboard teleoperation for the OMY simulator (run in its own terminal).

  w/s : +x/-x    a/d : +y/-y    r/f : up/down    space : open/close gripper
  n   : new episode (/omy/reset)    h : go home (/omy/go_home)    q : quit
"""
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

STEP = 0.01  # metres per key press
MOVES = {"w": (STEP, 0, 0), "s": (-STEP, 0, 0), "a": (0, STEP, 0), "d": (0, -STEP, 0),
         "r": (0, 0, STEP), "f": (0, 0, -STEP)}


class OmyTeleopKey(Node):
    def __init__(self):
        super().__init__("omy_teleop_key")
        self.pub = self.create_publisher(Float64MultiArray, "/omy/tcp_delta", 10)
        self.reset_cli = self.create_client(Trigger, "/omy/reset")
        self.home_cli = self.create_client(Trigger, "/omy/go_home")
        self.gripper = 0.0

    def send(self, dx, dy, dz):
        self.pub.publish(Float64MultiArray(data=[float(dx), float(dy), float(dz), self.gripper]))

    def call(self, client):
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if future.result() is not None:
            print(f"\r{future.result().message}")


def read_key(timeout=0.1):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if ready else None


def main():
    rclpy.init()
    node = OmyTeleopKey()
    print(__doc__)
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            key = read_key()
            if key is None:
                continue
            if key == "q":
                break
            if key in MOVES:
                node.send(*MOVES[key])
            elif key == " ":
                node.gripper = 1.0 if node.gripper < 0.5 else 0.0
                node.send(0, 0, 0)
                print(f"\rgripper {'closed' if node.gripper > 0.5 else 'open'}")
            elif key == "n":
                node.gripper = 0.0
                node.call(node.reset_cli)
            elif key == "h":
                node.gripper = 0.0
                node.call(node.home_cli)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.try_shutdown()
