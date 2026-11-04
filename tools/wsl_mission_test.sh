#!/usr/bin/env bash
# Autonomous coverage check of the prototype twin: Webots (fast) + ROS + Nav2 + coverage mission, no
# viewer; waits for the report and keeps the launch log in results/mission_launch_<world>.log.
#   bash tools/wsl_mission_test.sh [world] [area: demo | s0,s1,t0,t1] [timeout_s] [speed m/s] [throttle]
WORLD=${1:-winter_dam}; AREA=${2:-demo}; TMO=${3:-1800}; SPEED=${4:-0.11}; THR=${5:-full}
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPORT="$REPO/results/mission_report_$WORLD.txt"
rm -f "$REPORT"
source /opt/ros/humble/setup.bash
source ~/wcr_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
LOG=/tmp/wcr_proto_mission.log
ros2 launch wcr_proto proto.launch.py world:=$WORLD foxglove:=false mission:=true fast:=true area:=$AREA \
  speed:=$SPEED throttle:=$THR > $LOG 2>&1 &
LPID=$!
t0=$(date +%s)
while [ ! -f "$REPORT" ] && [ $(( $(date +%s) - t0 )) -lt $TMO ]; do
  sleep 20
  echo "[$(( $(date +%s) - t0 )) s] $(grep -E 'coverage_mission.*(poses|started|lane|footprint|cannot|Nav2|restart|not completed)' $LOG | tail -1 | sed 's/.*\]: //')"
done
echo "== Nav2 / mission warnings"
grep -E '\[(planner_server|controller_server|bt_navigator)-[0-9]+\] \[(WARN|ERROR)\]' $LOG \
  | sed -E 's/\[[0-9]+\.[0-9]+\]//' | sort | uniq -c | sort -rn | head -12
echo "== report"
cat "$REPORT" 2>/dev/null || echo "no report after $TMO s"
kill -INT $LPID; sleep 8; kill -9 $LPID 2>/dev/null
pkill -f '[w]ebots_controller' 2>/dev/null
taskkill.exe /F /IM webots-bin.exe > /dev/null 2>&1; taskkill.exe /F /IM webots.exe > /dev/null 2>&1   # the Windows window outlives the launch
cp $LOG "$REPO/results/mission_launch_$WORLD.log"; true
