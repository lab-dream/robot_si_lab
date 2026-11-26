#!/usr/bin/env bash
# End-to-end check of data collection: keyboard teleop moves the arm, and three scripted
# episodes recorded with scripts/record_demos.sh land in a rosbag2.
# Run from the repo root with venv_ros active, after building ros2_ws (see README).
set -euo pipefail
set -m  # job control: each background job gets its own process group and keeps SIGINT
set +u; source ros2_ws/install/setup.bash; set -u  # ROS setup scripts read unset variables
OUT=$(mktemp -d)/demos

ros2 launch robot_si_ros omy_sim.launch.py viewer:=false > /tmp/omy_launch.log 2>&1 &
LAUNCH_PID=$!
REC_PID=
# Ctrl+C a process group, as a terminal would; force-kill whatever is left after 10 s.
stop_group() {
  kill -INT -- -$1 2>/dev/null || return 0
  for _ in $(seq 20); do kill -0 -- -$1 2>/dev/null || return 0; sleep 0.5; done
  echo "process group $1 did not stop on SIGINT, killing it" >&2
  kill -KILL -- -$1 2>/dev/null || true
}
trap 'rc=$?; [ -n "$REC_PID" ] && stop_group $REC_PID; stop_group $LAUNCH_PID; exit $rc' EXIT

for _ in $(seq 60); do
  ros2 service list 2>/dev/null | grep -q /omy/pick_place && break
  sleep 1
done

joints() { ros2 topic echo --once --field position /joint_states; }

echo "== teleop: n, then w x5 and r x3"
BEFORE=$(joints)
# script(1) gives the teleop node the terminal it expects; keys are typed with pauses.
(sleep 3; printf n; sleep 3; printf wwwww; sleep 1; printf rrr; sleep 3; printf q) \
  | script -qec "ros2 run robot_si_ros omy_teleop_key" /dev/null > /tmp/teleop.log
AFTER=$(joints)
python3 - "$BEFORE" "$AFTER" <<'PY'
import re, sys
num = r"-?\d+\.\d+(?:e-?\d+)?"
a, b = ([float(v) for v in re.findall(num, s)][:6] for s in sys.argv[1:])
moved = max(abs(x - y) for x, y in zip(a, b))
print(f"largest joint change {moved:.3f} rad")
sys.exit(0 if moved > 0.02 else 1)
PY

echo "== record 3 scripted episodes"
scripts/record_demos.sh "$OUT" > /tmp/record.log 2>&1 &
REC_PID=$!
sleep 3
ros2 run robot_si_ros scripted_demo --ros-args -p episodes:=3
sleep 1
stop_group $REC_PID; REC_PID=
ros2 bag info "$OUT" | tee /tmp/bag_info.log
for t in /joint_states /omy/action /omy/episode /front_cam/image_raw /wrist_cam/image_raw; do
  n=$(awk -v t="$t" '$0 ~ "Topic: "t" " {for (i=1;i<=NF;i++) if ($i=="Count:") print $(i+1)}' /tmp/bag_info.log)
  echo "$t: $n messages"
  [ "${n:-0}" -gt 300 ]
done

echo "bag: $OUT"
echo "PASS record demos"
