"""turtlesim + circle publisher + pose subscriber in one command.

  ros2 launch robot_si_ros turtle_circle.launch.py speed:=3.0 turn:=1.5
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("speed", default_value="2.0"),
        DeclareLaunchArgument("turn", default_value="1.0"),
        Node(package="turtlesim", executable="turtlesim_node", name="turtlesim"),
        Node(package="robot_si_ros", executable="turtle_circle", output="screen",
             parameters=[{"speed": LaunchConfiguration("speed"), "turn": LaunchConfiguration("turn")}]),
        Node(package="robot_si_ros", executable="pose_listener", output="screen"),
    ])
