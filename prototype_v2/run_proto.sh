#!/usr/bin/env bash
# WCR PROTOTYPE twin: Webots (Windows) + ROS 2 (WSL) + Foxglove, on the dam faces of wcr_dam_sim.
#   bash run_proto.sh [winter_dam|goldeneye_dam|goldeneye_gorge] [extra launch args]
# extra args:  mission:=true [area:=demo]     autonomous coverage + report in results/
#              nav:=true                      Nav2 only: send goals from Foxglove
#              throttle:=full|2..7|auto       thrust (2..7 = servo-tester settings, measured 100..400 g)
#              speed:=0.11                    m/s (0.11 = N20 60 rpm; more = faster demo)
#              foxglove:=true rviz:=false     (defaults)
# Foxglove: open ws://localhost:8765, import ros/wcr_proto2/resource/wcr_proto2_foxglove.json
# Drive by hand from a second WSL terminal:
#   source ~/wcr_ws/install/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.11 -p turn:=0.5
WORLD=${1:-winter_dam}
shift || true
source /opt/ros/humble/setup.bash
source ~/wcr_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch wcr_proto2 proto.launch.py world:="$WORLD" "$@"
