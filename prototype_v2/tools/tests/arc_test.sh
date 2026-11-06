#!/usr/bin/env bash
# Arc / reverse test of the prototype twin: how much yaw an arc gives and how far the robot slides.
#   bash tools/tests/arc_test.sh WORLD THROTTLE 'mu_side mu_roll' 'fds_side fds_roll'
# e.g. bash tools/tests/arc_test.sh goldeneye_gorge full '0.35 0.45' '0.02 0'
WORLD=${1:-winter_dam}; THR=${2:-full}; MU=${3:-0.7}; FDS=${4:-0}
source /opt/ros/humble/setup.bash
source ~/wcr_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
LOG=/tmp/proto_arc_$WORLD.log
ros2 launch wcr_proto2 proto.launch.py world:=$WORLD foxglove:=false throttle:=$THR fast:=true "tyre_mu:=$MU" "tyre_fds:=$FDS" > $LOG 2>&1 &
LPID=$!
for i in $(seq 1 40); do sleep 3; ros2 topic list 2>/dev/null | grep -q ground_truth && break; done
python3 - <<'EOF'
import math, time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
rclpy.init()
n = Node('proto_arc_probe', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
st = {}
n.create_subscription(Odometry, '/ground_truth', lambda m: st.__setitem__('gt', m), 10)
n.create_subscription(Float32, '/thrust', lambda m: st.__setitem__('thrust', m.data), 10)
pub = n.create_publisher(Twist, '/cmd_vel', 10)
simt = lambda: n.get_clock().now().nanoseconds * 1e-9
def spin(sec, vx=None, wz=0.0):
    t0, w0 = simt(), time.time()
    while simt() - t0 < sec and time.time() - w0 < sec * 6 + 30:
        if vx is not None:
            c = Twist(); c.linear.x = vx; c.angular.z = wz; pub.publish(c)
        rclpy.spin_once(n, timeout_sec=0.02)
def R(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
face = None
def state():
    g = st['gt'].pose.pose
    p = np.array([g.position.x, g.position.y, g.position.z]); Rm = R((g.orientation.x, g.orientation.y, g.orientation.z, g.orientation.w))
    nrm = Rm[:, 2]; up = np.array([0, 0, 1.0]); up_f = up - nrm * (up @ nrm); up_f /= np.linalg.norm(up_f); ac = np.cross(nrm, up_f)
    yaw = math.degrees(math.atan2(Rm[:, 0] @ ac, Rm[:, 0] @ up_f))
    return p, yaw, up_f, ac
spin(10)
p0, y0, up, ac = state()
print('start yaw %.1f, thrust %s g' % (y0, st.get('thrust')))
def phase(name, sec, vx, wz):
    pa, ya, _, _ = state()
    spin(sec, vx, wz)
    pb, yb, _, _ = state()
    d = pb - pa
    print('%-28s yaw %+6.1f -> %+6.1f (d %+6.1f)  up %+.3f  across %+.3f m' % (name, ya, yb, yb - ya, d @ up, d @ ac))
phase('arc fwd wz -0.3 4 s', 4, 0.11, -0.3)
phase('fwd wz +0.25 6 s (back up?)', 6, 0.11, 0.25)
phase('fwd wz +0.25 6 s', 6, 0.11, 0.25)
phase('rev wz -0.25 6 s', 6, -0.11, -0.25)
phase('rev wz +0.25 6 s', 6, -0.11, 0.25)
phase('idle 8 s', 8, 0.0, 0.0)
EOF
kill -INT $LPID; sleep 6; kill -9 $LPID 2>/dev/null
pkill -f '[w]ebots_controller' 2>/dev/null
taskkill.exe /F /IM webots-bin.exe > /dev/null 2>&1; taskkill.exe /F /IM webots.exe > /dev/null 2>&1   # the Windows window outlives the launch
true
