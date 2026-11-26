"""Morning lab: send a command, the turtle performs a predefined task.

- Service /turtle_task/draw_square (std_srvs/Trigger): drive a fixed square, reply when done.
- Topic   /turtle_task/goal (geometry_msgs/Point): drive to (x, y), no reply.
"""
import math
import threading

import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from turtlesim.msg import Pose

SQUARE = [(3.0, 3.0), (8.0, 3.0), (8.0, 8.0), (3.0, 8.0), (3.0, 3.0)]


class TurtleTaskServer(Node):
    def __init__(self):
        super().__init__("turtle_task_server")
        group = ReentrantCallbackGroup()
        self.pose = None
        self.waypoints = []
        self.done = threading.Event()
        self.done.set()

        self.cmd_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.create_subscription(Pose, "/turtle1/pose", self.on_pose, 10, callback_group=group)
        self.create_subscription(Point, "/turtle_task/goal", self.on_goal, 10, callback_group=group)
        self.create_service(Trigger, "/turtle_task/draw_square", self.on_draw_square, callback_group=group)
        self.create_timer(0.05, self.control_step, callback_group=group)
        self.get_logger().info("Ready: /turtle_task/draw_square (service), /turtle_task/goal (topic)")

    def on_pose(self, msg):
        self.pose = msg

    def on_goal(self, msg):
        self.get_logger().info(f"Goal received: ({msg.x:.1f}, {msg.y:.1f})")
        self.start([(msg.x, msg.y)])

    def on_draw_square(self, request, response):
        self.get_logger().info("draw_square requested")
        self.start(SQUARE)
        response.success = self.done.wait(timeout=60.0)
        response.message = "square done" if response.success else "timed out"
        return response

    def start(self, waypoints):
        self.waypoints = list(waypoints)
        self.done.clear()

    def control_step(self):
        """Simple go-to-point controller: turn toward the waypoint, then drive."""
        if self.pose is None or not self.waypoints:
            return
        x, y = self.waypoints[0]
        dx, dy = x - self.pose.x, y - self.pose.y
        dist = math.hypot(dx, dy)
        cmd = Twist()
        if dist < 0.05:
            self.waypoints.pop(0)
            if not self.waypoints:
                self.done.set()
        else:
            heading_err = math.atan2(math.sin(math.atan2(dy, dx) - self.pose.theta),
                                     math.cos(math.atan2(dy, dx) - self.pose.theta))
            cmd.angular.z = 4.0 * heading_err
            if abs(heading_err) < 0.1:
                cmd.linear.x = min(2.0, 1.5 * dist)
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = TurtleTaskServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
