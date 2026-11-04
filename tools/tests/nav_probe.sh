#!/usr/bin/env bash
# Sample /cmd_vel, /odometry/filtered and the true wall pose (map frame) of the running twin.
source /opt/ros/humble/setup.bash
source ~/wcr_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 - <<'EOF'
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
rclpy.init()
n = Node('proto_probe', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
st = {}
n.create_subscription(Twist, '/cmd_vel', lambda m: st.__setitem__('cmd', m), 10)
n.create_subscription(Twist, '/cmd_vel_nav', lambda m: st.__setitem__('nav', m), 10)
n.create_subscription(Odometry, '/odometry/filtered', lambda m: st.__setitem__('ekf', m), 10)
buf = Buffer(); TransformListener(buf, n)
def yaw(q): return math.degrees(2 * math.atan2(q.z, q.w))
t0 = time.time()
last = 0
while time.time() - t0 < 40:
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() - last < 2: continue
    last = time.time()
    try:
        tf = buf.lookup_transform('map', 'base_link', rclpy.time.Time()); tr = tf.transform
        pose = 'map s %.2f t %.2f yaw %+6.1f' % (tr.translation.x, tr.translation.y, yaw(tr.rotation))
    except Exception as e:
        pose = 'no tf'
    c, v = st.get('cmd'), st.get('nav')
    print('%s | cmd vx %+.3f wz %+.3f | nav vx %+.3f wz %+.3f' % (pose, c.linear.x if c else 0, c.angular.z if c else 0,
          v.linear.x if v else 0, v.angular.z if v else 0))
EOF
