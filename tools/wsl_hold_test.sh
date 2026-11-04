#!/usr/bin/env bash
# Does the prototype hold on a dam face at a throttle setting? Spawn on the face (on the deployment
# ledge, removed at 4 s), drive 0.2 m up, park, report how far it slides in 20 s (fast mode, no viewer).
#   bash tools/wsl_hold_test.sh [world] [throttle: 2..7 | full | grams] [tyre_mu] [tyre_fds]
WORLD=${1:-winter_dam}; THR=${2:-7}; TYRE=(); [ -n "$3" ] && TYRE+=("tyre_mu:=$3"); [ -n "$4" ] && TYRE+=("tyre_fds:=$4")
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source ~/wcr_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
LOG=/tmp/wcr_proto_hold.log
ros2 launch wcr_proto proto.launch.py world:=$WORLD foxglove:=false fast:=true throttle:=$THR "${TYRE[@]}" > $LOG 2>&1 &
LPID=$!
for i in $(seq 1 40); do sleep 3; ros2 topic list 2>/dev/null | grep -q ground_truth && break; done
python3 - "$WORLD" "$THR" <<'EOF'
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
rclpy.init()
n = Node('proto_hold_probe', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
st = {}
n.create_subscription(Odometry, '/ground_truth', lambda m: st.__setitem__('gt', m), 10)
n.create_subscription(Float32, '/thrust', lambda m: st.__setitem__('thrust', m.data), 10)
simt = lambda: n.get_clock().now().nanoseconds * 1e-9
def wait_until(t_end):
    w0 = time.time()
    while simt() < t_end and time.time() - w0 < 120:
        rclpy.spin_once(n, timeout_sec=0.05)
def pos():
    p = st['gt'].pose.pose.position
    return np.array([p.x, p.y, p.z])
pub = n.create_publisher(Twist, '/cmd_vel', 10)
w0 = time.time()
while 'gt' not in st and time.time() - w0 < 60:
    rclpy.spin_once(n, timeout_sec=0.05)
p_spawn = pos()
wait_until(4.5)                                   # cradle (deployment ledge) gone at 4 s
up = Twist()
up.linear.x = 0.11
w0 = time.time()
while simt() < 6.5 and time.time() - w0 < 60:     # drive 0.2 m up the face, off the ledge
    pub.publish(up)
    rclpy.spin_once(n, timeout_sec=0.02)
wait_until(7.5)
p0 = pos()
wait_until(27.5)
d = pos() - p0
slid = float(np.linalg.norm(d))
print(f'{sys.argv[1]:16s} throttle {sys.argv[2]:>5s} ({st.get("thrust", 0):.0f} g): drove up '
      f'{np.linalg.norm(p0 - p_spawn) * 100:.0f} cm, then moved {slid * 100:.1f} cm in 20 s parked: '
      f'{"HOLDS" if slid < 0.03 else "SLIDES DOWN"} (dz {d[2] * 100:+.1f} cm)')
EOF
kill -INT $LPID; sleep 6; kill -9 $LPID 2>/dev/null
pkill -f '[w]ebots_controller' 2>/dev/null
taskkill.exe /F /IM webots-bin.exe > /dev/null 2>&1; taskkill.exe /F /IM webots.exe > /dev/null 2>&1; true
