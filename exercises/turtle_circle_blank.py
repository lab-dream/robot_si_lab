"""Exercise 1-1, Publisher: fill in every blank so the turtle drives in a circle.

  python3 exercises/turtle_circle_blank.py     (turtlesim must be running)

Answer: ros2_ws/src/robot_si_ros/robot_si_ros/turtle_circle.py
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TurtleCircle(Node):
    def __init__(self):
        super().__init__("my_turtle_circle")
        # (1) publisher: message type Twist, topic /turtle1/cmd_vel, queue 10
        self.pub = self.create_publisher(____, "____", 10)
        # (2) call self.on_timer every 0.1 s (10 Hz)
        self.create_timer(____, self.on_timer)

    def on_timer(self):
        msg = Twist()
        # (3) forward speed 2.0 m/s, turn rate 1.0 rad/s
        msg.linear.x = ____
        msg.angular.z = ____
        # (4) send the message
        self.pub.____(msg)


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


if __name__ == "__main__":
    main()
