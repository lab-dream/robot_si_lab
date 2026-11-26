"""Exercise 2, Launch: fill in every blank to start three nodes with one command.

  ros2 launch exercises/my_turtle_blank.launch.py

Answer: ros2_ws/src/robot_si_ros/launch/turtle_circle.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # (1) the turtlesim window: package turtlesim, executable turtlesim_node
        Node(package="____", executable="____"),
        # (2) the circle publisher from robot_si_ros, with parameter speed = 3.0
        Node(package="robot_si_ros", executable="____", output="screen",
             parameters=[{"____": 3.0}]),
        # (3) the pose subscriber from robot_si_ros
        Node(package="robot_si_ros", executable="____", output="screen"),
    ])
