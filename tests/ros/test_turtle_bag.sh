#!/usr/bin/env bash
# Check of the rosbag2 lab: record the square task, reset the turtle, replay only /turtle1/cmd_vel.
# The replayed commands are open loop, so the end point drifts away from where the recording ended.
# Run from the repo root after building ros2_ws (see README).
set -euo pipefail
set -m  # job control: each background job gets its own process group and keeps SIGINT
export QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-offscreen}
set +u; source ros2_ws/install/setup.bash; set -u  # ROS setup scripts read unset variables
TMP=$(mktemp -d)
BAG="$TMP/square"

PGIDS=()
start() {  # start <log> <cmd...>: background job, its own process group under set -m
  local log=$1; shift
  "$@" > "$log" 2>&1 &
  PGIDS+=($!)
}
stop() {  # stop <pgid>: Ctrl+C, then force-kill after 10 s
  kill -INT -- -$1 2>/dev/null || return 0
  for _ in $(seq 20); do kill -0 -- -$1 2>/dev/null || return 0; sleep 0.5; done
  kill -KILL -- -$1 2>/dev/null || true
}
trap 'rc=$?; for p in "${PGIDS[@]}"; do stop $p; done; rm -rf "$TMP"; exit $rc' EXIT

pose_xy() { ros2 topic echo --once /turtle1/pose turtlesim/msg/Pose | awk '/^x:/{x=$2} /^y:/{y=$2} END{print x, y}'; }

start "$TMP/sim.log" ros2 run turtlesim turtlesim_node
start "$TMP/task.log" ros2 run robot_si_ros turtle_task_server
for _ in $(seq 30); do ros2 service list 2>/dev/null | grep -q /turtle_task/draw_square && break; sleep 1; done

echo "== record"
start "$TMP/rec.log" ros2 bag record -o "$BAG" /turtle1/cmd_vel /turtle1/pose
REC=${PGIDS[-1]}
sleep 3
ros2 service call /turtle_task/draw_square std_srvs/srv/Trigger | grep -q "success=True"
sleep 1
stop $REC
ros2 bag info "$BAG" | tee "$TMP/info.log"
grep -q "/turtle1/cmd_vel" "$TMP/info.log"

echo "== replay /turtle1/cmd_vel after reset"
stop ${PGIDS[1]}   # the task server would fight the replayed commands
ros2 service call /reset std_srvs/srv/Empty > /dev/null
sleep 1
ros2 bag play "$BAG" --topics /turtle1/cmd_vel
sleep 1
read X Y < <(pose_xy)
# Pass when the replay moved the turtle off the reset pose (5.54, 5.54); the distance to (3, 3)
# is printed, not checked, because open-loop timing drift changes it from run to run.
python3 -c "import math,sys; x,y=map(float,sys.argv[1:]); m=math.hypot(x-5.54,y-5.54); e=math.hypot(x-3,y-3); print(f'replay end ({x:.2f}, {y:.2f}), {e:.2f} from the recorded end (3, 3)'); sys.exit(0 if m>1.0 else 1)" $X $Y

echo "PASS turtle bag"
