"""Exercise 1-2, Subscriber: fill in every blank to print the turtle pose once per second.

  python3 exercises/pose_listener_blank.py     (turtlesim must be running)

Answer: ros2_ws/src/robot_si_ros/robot_si_ros/pose_listener.py
"""
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class PoseListener(Node):
    def __init__(self):
        super().__init__("my_pose_listener")
        # (1) subscribe: message type Pose, topic /turtle1/pose, callback self.on_pose, queue 10
        self.create_subscription(____, "____", ____, 10)

    def on_pose(self, msg):
        # (2) print x, y and theta of the received message
        self.get_logger().info(f"x={msg.____:.2f} y={msg.____:.2f} theta={msg.____:.2f}",
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


if __name__ == "__main__":
    main()
