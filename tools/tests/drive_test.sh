#!/usr/bin/env bash
# Drive test of the prototype twin: hold on the wall, forward, turn, report ground-truth motion.
#   proto_drive_test.sh WORLD THROTTLE
WORLD=${1:-winter_dam}; THR=${2:-full}
source /opt/ros/humble/setup.bash
source ~/wcr_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
LOG=/tmp/proto_drive_$WORLD.log
ros2 launch wcr_proto proto.launch.py world:=$WORLD foxglove:=false throttle:=$THR > $LOG 2>&1 &
LPID=$!
for i in $(seq 1 40); do sleep 3; ros2 topic list 2>/dev/null | grep -q ground_truth && break; done
python3 - "$WORLD" <<'EOF'
import math, sys, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
rclpy.init()
n = Node('proto_drive_probe', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
st = {'gt': None, 'thrust': None, 'odom': None}
n.create_subscription(Odometry, '/ground_truth', lambda m: st.__setitem__('gt', m), 10)
n.create_subscription(Odometry, '/odom', lambda m: st.__setitem__('odom', m), 10)
n.create_subscription(Float32, '/thrust', lambda m: st.__setitem__('thrust', m.data), 10)
pub = n.create_publisher(Twist, '/cmd_vel', 10)
def simt():
    return n.get_clock().now().nanoseconds * 1e-9
def spin(sec, cmd=None):
    t0 = simt(); w0 = time.time()
    while simt() - t0 < sec and time.time() - w0 < sec * 6 + 30:
        if cmd is not None:
            pub.publish(cmd)
        rclpy.spin_once(n, timeout_sec=0.05)
def pose():
    g = st['gt']
    if g is None: return None
    p, q = g.pose.pose.position, g.pose.pose.orientation
    return (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)
def yaw_on_wall(q):
    # robot x axis in world, projected: angle between robot x and world-up projected on the face
    x, y, z, w = q
    rx = (1 - 2*(y*y+z*z), 2*(x*y+z*w), 2*(x*z-y*w))
    rz = (2*(x*z+y*w), 2*(y*z-x*w), 1-2*(x*x+y*y))
    up = (0, 0, 1)
    d = sum(a*b for a, b in zip(up, rz)); upp = [u - d*r for u, r in zip(up, rz)]
    nrm = math.sqrt(sum(v*v for v in upp)) or 1
    upp = [v/nrm for v in upp]
    c = sum(a*b for a, b in zip(rx, upp)); cr = [rz[1]*0, 0, 0]
    s = sum(a*b for a, b in zip(rz, [upp[1]*rx[2]-upp[2]*rx[1], upp[2]*rx[0]-upp[0]*rx[2], upp[0]*rx[1]-upp[1]*rx[0]]))
    return math.degrees(math.atan2(s, c))
spin(8)
p0 = pose(); print('start', p0 and [round(v, 3) for v in p0[0]], 'thrust g', st['thrust'])
spin(6)
p1 = pose(); print('after 6 s idle (cradle gone): moved %.3f m, yaw %.1f deg' % (math.dist(p0[0], p1[0]), yaw_on_wall(p1[1])))
fw = Twist(); fw.linear.x = 0.11
spin(8, fw)
p2 = pose(); print('forward 8 s: moved %.3f m (expect ~0.88), dz %.3f, yaw %.1f' % (math.dist(p1[0], p2[0]), p2[0][2] - p1[0][2], yaw_on_wall(p2[1])))
tw = Twist(); tw.angular.z = 0.5
spin(6, tw)
p3 = pose(); print('turn 6 s at 0.5 rad/s: yaw %.1f -> %.1f deg (expect +172), drift %.3f m' % (yaw_on_wall(p2[1]), yaw_on_wall(p3[1]), math.dist(p2[0], p3[0])))
spin(6, fw)
p4 = pose(); print('forward 6 s after turn: moved %.3f m, dz %.3f' % (math.dist(p3[0], p4[0]), p4[0][2] - p3[0][2]))
tw.angular.z = -0.5
spin(6, tw)
spin(10)
p5 = pose(); print('turn back, idle 10 s: yaw %.1f, slide %.3f m' % (yaw_on_wall(p5[1]), math.dist(p4[0], p5[0])))
o = st['odom']; print('odom x %.2f y %.2f' % (o.pose.pose.position.x, o.pose.pose.position.y) if o else 'no odom', 'thrust g', st['thrust'])
EOF
echo "== log"
grep -E 'throttle|ERROR|Error|rror' $LOG | head -20
kill -INT $LPID; sleep 6; kill -9 $LPID 2>/dev/null
pkill -f '[w]ebots_controller' 2>/dev/null
taskkill.exe /F /IM webots-bin.exe > /dev/null 2>&1; true
