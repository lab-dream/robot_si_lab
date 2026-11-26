#!/usr/bin/env bash
# End-to-end check of the MuJoCo bridge: topics stream and the task services work.
# Run from the repo root with venv_ros active, after building ros2_ws (see README).
set -euo pipefail
set +u; source ros2_ws/install/setup.bash; set -u  # ROS setup scripts read unset variables

setsid ros2 launch robot_si_ros omy_sim.launch.py viewer:=false > /tmp/omy_launch.log 2>&1 &
LAUNCH_PID=$!
# Ctrl+C the whole launch process group, as a terminal would; force-kill whatever is left after 10 s.
stop_launch() {
  kill -INT -- -$LAUNCH_PID 2>/dev/null || return 0
  for _ in $(seq 20); do kill -0 -- -$LAUNCH_PID 2>/dev/null || return 0; sleep 0.5; done
  echo "launch did not stop on SIGINT, killing it" >&2
  kill -KILL -- -$LAUNCH_PID 2>/dev/null || true
}
trap 'rc=$?; stop_launch; exit $rc' EXIT

for _ in $(seq 60); do
  ros2 service list 2>/dev/null | grep -q /omy/pick_place && break
  sleep 1
done

echo "== topics"
for t in /joint_states /omy/action /front_cam/image_raw /wrist_cam/image_raw; do
  timeout 10 ros2 topic echo --once --field header.stamp "$t" > /dev/null
  echo "ok $t"
done

echo "== tf: world -> tcp at the home pose"
timeout 6 ros2 run tf2_ros tf2_echo world tcp > /tmp/omy_tf.log 2>&1 || true
T=$(grep -m1 "Translation" /tmp/omy_tf.log); echo "$T"
python3 - "$T" <<'PY'
import re, sys
p = [float(v) for v in re.findall(r"-?\d+\.\d+", sys.argv[1])]
sys.exit(0 if len(p) == 3 and max(abs(a - b) for a, b in zip(p, [0.3, 0.0, 0.25])) < 0.01 else 1)
PY

echo "== services"
for s in reset go_home pick_place; do
  timeout 60 ros2 service call /omy/$s std_srvs/srv/Trigger | tee /tmp/omy_srv.log | grep -E "success|message" || true
  grep -q "success=True" /tmp/omy_srv.log
done

echo "== /joint_command"
ros2 topic pub --times 3 --rate 5 /joint_command std_msgs/msg/Float64MultiArray "{data: [0.0, -0.3, 1.2, 0.6, 1.57, 0.0, 0.0]}"
sleep 5  # the position servos are overdamped
J=$(ros2 topic echo --once --field position /joint_states)
echo "joint_states after command: $J"
python3 - "$J" <<'PY'
import re, sys
q = [float(v) for v in re.findall(r"-?\d+\.\d+(?:e-?\d+)?", sys.argv[1])]
target = [0.0, -0.3, 1.2, 0.6, 1.57, 0.0]
err = max(abs(a - b) for a, b in zip(q[:6], target))
print(f"max joint error {err:.3f} rad")
sys.exit(0 if err < 0.05 else 1)
PY

echo "PASS omy sim"
