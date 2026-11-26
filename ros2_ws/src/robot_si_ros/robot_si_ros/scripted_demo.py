"""Scripted demonstrations: for each episode call /omy/reset, /omy/pick_place, /omy/go_home.

Stands in for a human teleoperator so the whole data pipeline can be tested without a keyboard.
  ros2 run robot_si_ros scripted_demo --ros-args -p episodes:=3
"""
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    node = Node("scripted_demo")
    episodes = node.declare_parameter("episodes", 3).value
    clients = {name: node.create_client(Trigger, f"/omy/{name}") for name in ["reset", "pick_place", "go_home"]}
    for client in clients.values():
        if not client.wait_for_service(timeout_sec=30.0):
            node.get_logger().error(f"{client.srv_name} not available. Is omy_sim running?")
            rclpy.try_shutdown()
            raise SystemExit(1)

    n_success = 0
    for ep in range(episodes):
        for name in ["reset", "pick_place", "go_home"]:
            future = clients[name].call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=60.0)
            result = future.result()
            node.get_logger().info(f"episode {ep}: {name} -> {result.message if result else 'no reply'}")
            if name == "pick_place" and result is not None and result.success:
                n_success += 1
    node.get_logger().info(f"{n_success}/{episodes} episodes succeeded")
    node.destroy_node()
    rclpy.try_shutdown()
    raise SystemExit(0 if n_success == episodes else 1)
