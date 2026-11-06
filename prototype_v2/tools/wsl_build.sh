#!/usr/bin/env bash
# Build the twin's ROS package into ~/wcr_ws (next to wcr_dam_sim's packages, which it reuses).
set -e
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p ~/wcr_ws/src
[ -e ~/wcr_ws/src/wcr_proto2 ] || ln -s "$REPO/ros/wcr_proto2" ~/wcr_ws/src/wcr_proto2
source /opt/ros/humble/setup.bash
cd ~/wcr_ws
colcon build --symlink-install --packages-select wcr_proto2 2>&1 | tail -15
