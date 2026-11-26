#!/usr/bin/env bash
# End-to-end check of the morning lab: service and topic commands move the turtle.
# Run from the repo root after building ros2_ws (see README).
set -euo pipefail
export QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-offscreen}
set +u; source ros2_ws/install/setup.bash; set -u  # ROS setup scripts read unset variables

setsid ros2 launch robot_si_ros turtle_task.launch.py > /tmp/turtle_launch.log 2>&1 &
LAUNCH_PID=$!
# Ctrl+C the whole launch process group, as a terminal would; force-kill whatever is left after 10 s.
stop_launch() {
  kill -INT -- -$LAUNCH_PID 2>/dev/null || return 0
  for _ in $(seq 20); do kill -0 -- -$LAUNCH_PID 2>/dev/null || return 0; sleep 0.5; done
  echo "launch did not stop on SIGINT, killing it" >&2
  kill -KILL -- -$LAUNCH_PID 2>/dev/null || true
}
trap 'rc=$?; stop_launch; exit $rc' EXIT

for _ in $(seq 30); do
  ros2 service list 2>/dev/null | grep -q /turtle_task/draw_square && break
  sleep 1
done

pose_xy() { ros2 topic echo --once /turtle1/pose turtlesim/msg/Pose | awk '/^x:/{x=$2} /^y:/{y=$2} END{print x, y}'; }
near() { python3 -c "import sys; x,y,tx,ty=map(float,sys.argv[1:]); sys.exit(0 if abs(x-tx)<0.2 and abs(y-ty)<0.2 else 1)" $1 $2 $3 $4; }

echo "== service: draw_square"
ros2 service call /turtle_task/draw_square std_srvs/srv/Trigger | tee /tmp/turtle_srv.log
grep -q "success=True" /tmp/turtle_srv.log
read X Y < <(pose_xy); echo "pose after square: $X $Y"
near $X $Y 3.0 3.0

echo "== topic: goal (8, 5)"
ros2 topic pub --once /turtle_task/goal geometry_msgs/msg/Point "{x: 8.0, y: 5.0}"
sleep 6
read X Y < <(pose_xy); echo "pose after goal: $X $Y"
near $X $Y 8.0 5.0

echo "PASS turtle task"
