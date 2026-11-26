"""Morning lab, Publisher: drive turtle1 in a circle by publishing Twist at 10 Hz.

  ros2 run robot_si_ros turtle_circle --ros-args -p speed:=2.0 -p turn:=1.0
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TurtleCircle(Node):
    def __init__(self):
        super().__init__("turtle_circle")
        self.speed = self.declare_parameter("speed", 2.0).value  # m/s
        self.turn = self.declare_parameter("turn", 1.0).value    # rad/s
        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.create_timer(0.1, self.on_timer)
        self.get_logger().info(f"Publishing /turtle1/cmd_vel: speed={self.speed}, turn={self.turn}")

    def on_timer(self):
        msg = Twist()
        msg.linear.x = self.speed
        msg.angular.z = self.turn
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TurtleCircle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
