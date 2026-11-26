from glob import glob

from setuptools import find_packages, setup

package_name = "robot_si_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Suhan Park",
    maintainer_email="park94@kw.ac.kr",
    description="ROS 2 lab nodes for the Robot SI imitation learning course.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "turtle_task_server = robot_si_ros.turtle_task_server:main",
            "omy_sim = robot_si_ros.omy_sim_node:main",
            "omy_teleop_key = robot_si_ros.omy_teleop_key:main",
            "scripted_demo = robot_si_ros.scripted_demo:main",
            "turtle_circle = robot_si_ros.turtle_circle:main",
            "pose_listener = robot_si_ros.pose_listener:main",
        ],
    },
)
