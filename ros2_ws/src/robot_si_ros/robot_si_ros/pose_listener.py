"""Morning lab, Subscriber: print the pose of turtle1 once per second.

  ros2 run robot_si_ros pose_listener
"""
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class PoseListener(Node):
    def __init__(self):
        super().__init__("pose_listener")
        self.create_subscription(Pose, "/turtle1/pose", self.on_pose, 10)

    def on_pose(self, msg):
        # turtlesim publishes about 60 times per second; log at most once per second
        self.get_logger().info(f"x={msg.x:.2f} y={msg.y:.2f} theta={msg.theta:.2f}",
                               throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = PoseListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
