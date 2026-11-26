#!/usr/bin/env bash
# Check of the publisher/subscriber/launch exercises: the answer launch file drives the turtle,
# and each blank exercise works once its blanks are filled with the intended answers.
# Run from the repo root after building ros2_ws (see README).
set -euo pipefail
set -m  # job control: each background job gets its own process group and keeps SIGINT
export QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-offscreen}
set +u; source ros2_ws/install/setup.bash; set -u  # ROS setup scripts read unset variables
TMP=$(mktemp -d)

PGIDS=()
start() {  # start <log> <cmd...>: background job, its own process group under set -m
  local log=$1; shift
  "$@" > "$log" 2>&1 &
  PGIDS+=($!)
}
stop_all() {
  for p in "${PGIDS[@]}"; do kill -INT -- -$p 2>/dev/null || true; done
  for p in "${PGIDS[@]}"; do
    for _ in $(seq 20); do kill -0 -- -$p 2>/dev/null || break; sleep 0.5; done
    kill -KILL -- -$p 2>/dev/null || true
  done
  PGIDS=()
}
trap 'rc=$?; stop_all; rm -rf "$TMP"; exit $rc' EXIT

wait_for() {  # wait_for <seconds> <cmd...>
  local n=$1; shift
  for _ in $(seq "$n"); do "$@" >/dev/null 2>&1 && return 0; sleep 1; done
  return 1
}
pose_xy() { ros2 topic echo --once /turtle1/pose turtlesim/msg/Pose | awk '/^x:/{x=$2} /^y:/{y=$2} END{print x, y}'; }
moved() { python3 -c "import sys; a,b,c,d=map(float,sys.argv[1:]); sys.exit(0 if abs(a-c)+abs(b-d)>0.3 else 1)" $1 $2 $3 $4; }

# fill <blank file> <out file> <answer>...: replace each ____ in order
fill() {
  python3 - "$@" <<'EOF'
import sys
src, out, answers = sys.argv[1], sys.argv[2], sys.argv[3:]
text = open(src).read()
assert text.count("____") == len(answers), (src, text.count("____"), len(answers))
for a in answers:
    text = text.replace("____", a, 1)
open(out, "w").write(text)
EOF
}
fill exercises/turtle_circle_blank.py "$TMP/pub.py" Twist /turtle1/cmd_vel 0.1 2.0 1.0 publish
fill exercises/pose_listener_blank.py "$TMP/sub.py" Pose /turtle1/pose self.on_pose x y theta
fill exercises/my_turtle_blank.launch.py "$TMP/my.launch.py" turtlesim turtlesim_node turtle_circle speed pose_listener

echo "== answer launch file"
start "$TMP/launch.log" ros2 launch robot_si_ros turtle_circle.launch.py speed:=3.0
wait_for 30 bash -c "ros2 node list | grep -q /pose_listener"
read X0 Y0 < <(pose_xy); sleep 2; read X1 Y1 < <(pose_xy)
echo "pose $X0 $Y0 -> $X1 $Y1"; moved $X0 $Y0 $X1 $Y1
ros2 param get /turtle_circle speed | tee "$TMP/param.log"; grep -q "3.0" "$TMP/param.log"
wait_for 10 grep -q "x=" "$TMP/launch.log"
stop_all

echo "== filled exercises 1-1, 1-2"
start "$TMP/sim.log" ros2 run turtlesim turtlesim_node
wait_for 30 bash -c "ros2 topic list | grep -q /turtle1/pose"
start "$TMP/pub.log" python3 "$TMP/pub.py"
start "$TMP/sub.log" python3 "$TMP/sub.py"
wait_for 20 grep -q "x=" "$TMP/sub.log" || { cat "$TMP/sub.log"; exit 1; }
read X0 Y0 < <(pose_xy); sleep 2; read X1 Y1 < <(pose_xy)
echo "pose $X0 $Y0 -> $X1 $Y1"; moved $X0 $Y0 $X1 $Y1
stop_all

echo "== filled exercise 2"
start "$TMP/my.log" ros2 launch "$TMP/my.launch.py"
wait_for 30 bash -c "ros2 node list | grep -q /pose_listener"
ros2 param get /turtle_circle speed | grep -q "3.0"
wait_for 10 grep -q "x=" "$TMP/my.log"

echo "PASS turtle pub/sub"
