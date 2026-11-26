#!/usr/bin/env bash
# End-to-end check of tools/ros_policy.py: the policy node drives omy_sim over /joint_command.
#   bash tests/ros/test_ros_policy.sh outputs/act_omy/checkpoints/last/pretrained_model
# Run from the repo root after building ros2_ws (README 2) and installing the 4-7 extras into venv_lerobot.
set -euo pipefail
set -m
POLICY=${1:?usage: test_ros_policy.sh <pretrained_model folder>}
set +u  # ROS setup scripts read unset variables
source /opt/ros/jazzy/setup.bash; source ~/robot_si/venv_ros/bin/activate; source ros2_ws/install/setup.bash
set -u

ros2 launch robot_si_ros omy_sim.launch.py viewer:=false > /tmp/omy_launch.log 2>&1 &
LAUNCH_PID=$!
POLICY_PID=
stop_group() {
  kill -INT -- -$1 2>/dev/null || return 0
  for _ in $(seq 20); do kill -0 -- -$1 2>/dev/null || return 0; sleep 0.5; done
  echo "process group $1 did not stop on SIGINT, killing it" >&2
  kill -KILL -- -$1 2>/dev/null || true
}
trap 'rc=$?; [ -n "$POLICY_PID" ] && stop_group $POLICY_PID; stop_group $LAUNCH_PID; exit $rc' EXIT

for _ in $(seq 60); do
  ros2 service list 2>/dev/null | grep -q /omy/reset && break
  sleep 1
done
ros2 service call /omy/reset std_srvs/srv/Trigger > /dev/null

echo "== start the policy node (venv_lerobot)"
~/robot_si/venv_lerobot/bin/python tools/ros_policy.py "$POLICY" > /tmp/ros_policy.log 2>&1 &
POLICY_PID=$!
for _ in $(seq 60); do grep -q "first command sent" /tmp/ros_policy.log && break; sleep 1; done
cat /tmp/ros_policy.log

echo "== /joint_command rate"
timeout -s INT 6 ros2 topic hz /joint_command > /tmp/cmd_hz.log 2>&1 || true
tail -2 /tmp/cmd_hz.log
python3 - <<'PY'
import re, sys
rates = [float(r) for r in re.findall(r"average rate: ([\d.]+)", open("/tmp/cmd_hz.log").read())]
print(f"rates {rates}")
sys.exit(0 if rates and rates[-1] > 15 else 1)
PY

echo "== cube on the pad within 20 s (same check as OmySim.is_success, from /tf)"
python3 - <<'PY'
import sys, time
import rclpy
from tf2_msgs.msg import TFMessage
rclpy.init()
node = rclpy.create_node("check_cube")
pos = {}
def on_tf(msg):
    for t in msg.transforms:
        if t.header.frame_id == "world" and t.child_frame_id in ("cube", "target"):
            v = t.transform.translation
            pos[t.child_frame_id] = (v.x, v.y, v.z)
node.create_subscription(TFMessage, "/tf", on_tf, 10)
end, ok = time.monotonic() + 20, False
while time.monotonic() < end and not ok:
    rclpy.spin_once(node, timeout_sec=0.1)
    if len(pos) == 2:
        (cx, cy, cz), (tx, ty, _) = pos["cube"], pos["target"]
        ok = ((cx - tx) ** 2 + (cy - ty) ** 2) ** 0.5 < 0.04 and cz < 0.03
print(f"cube {pos.get('cube')}, target {pos.get('target')}, on pad: {ok}")
sys.exit(0 if ok else 1)
PY
echo "PASS ros policy"
