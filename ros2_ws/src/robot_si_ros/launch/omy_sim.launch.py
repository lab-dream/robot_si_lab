from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    viewer = LaunchConfiguration("viewer")
    image_size = LaunchConfiguration("image_size")
    return LaunchDescription([
        DeclareLaunchArgument("viewer", default_value="true", description="Open the MuJoCo viewer window"),
        DeclareLaunchArgument("image_size", default_value="128", description="Camera image width/height"),
        Node(package="robot_si_ros", executable="omy_sim", output="screen",
             parameters=[{"viewer": viewer, "image_size": image_size}]),
    ])
