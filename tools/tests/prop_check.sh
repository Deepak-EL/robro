#!/usr/bin/env bash
# Is the propeller's display angle advancing in /joint_states and TF?
source /opt/ros/humble/setup.bash
source ~/wcr_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 - <<'EOF'
import math, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
rclpy.init()
n = Node('prop_check', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
st = {}
n.create_subscription(JointState, '/joint_states', lambda m: st.__setitem__('js', m), 10)
b = Buffer(); TransformListener(b, n)
t0, last = time.time(), 0
while time.time() - t0 < 8:
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() - last > 1.5 and 'js' in st:
        last = time.time()
        js = st['js']
        i = js.name.index('prop_joint') if 'prop_joint' in js.name else None
        try:
            q = b.lookup_transform('base_link', 'prop_link', rclpy.time.Time()).transform.rotation
            yaw = math.degrees(2 * math.atan2(q.z, q.w))
        except Exception:
            yaw = float('nan')
        print('prop_joint', None if i is None else round(js.position[i], 2), 'rad,', None if i is None else round(js.velocity[i], 1),
              'rad/s | TF prop yaw %.0f deg' % yaw)
EOF
