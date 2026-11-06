"""webots_ros2_driver plugin for the WCR prototype (robot controller "<extern>").

Same ROS interface as the dam sim's WcrDriver, so its nodes (inspection detector, wall frames,
coverage mission, EKF, Nav2) and the Foxglove layout work unchanged:

  sub  /cmd_vel        geometry_msgs/Twist   x forward, z yaw (skid steer: y is ignored); stops after 0.5 s
  pub  /odom           nav_msgs/Odometry     wheel encoders (forward speed) + gyro (yaw rate), frame odom
       /joint_states   sensor_msgs/JointState  wheel angles
       /ground_truth   nav_msgs/Odometry     true 3D pose in the Webots world (frame world)
       /adhesion       std_msgs/Float32      propeller thrust pressing the robot on the wall [N]
       /thrust         std_msgs/Float32      the same in grams (as on the team's weighing scale)
       /clock          rosgraph_msgs/Clock
  tf   odom -> base_link unless WCR_ODOM_TF=0 (then the robot_localization EKF publishes it)
  env  WCR_THROTTLE    '2'..'7' (servo-tester settings, measured thrust), 'full' (default), 'auto'
       WCR_MAX_SPEED   m/s (above 0.11 = faster-than-real demo)

Hardware note: the real N20s have no encoders and the robot has no IMU yet; the twin assumes the
encoder version of the N20 and an MPU6050 (sensor kit).
"""
import math
import os

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster

from wcr_proto2 import proto_model as model
from wcr_proto2.proto_model import G, Thrust, forward_speed, inverse_kinematics, prop_speed

WHEELS = ('FL', 'FR', 'BL', 'BR')
LEFT = ('FL', 'BL')
CMD_TIMEOUT = 0.5
CRADLE_RELEASE = 4.0
PROP_Z = 0.0535              # hub height above the wall (base frame)
PROP_DISPLAY_W = 12.0        # rad/s shown in RViz / Foxglove at full thrust (~2 turns/s)


def diag6(*v):
    return [float(v[i]) if i == j else 0.0 for i in range(6) for j in range(6)]


POSE_COV = diag6(1e-3, 1e-3, 1e6, 1e6, 1e6, 1e-2)
TWIST_COV = diag6(1e-4, 1e-4, 1e6, 1e6, 1e6, 1e-3)


def quat_from_matrix(R):
    m00, m01, m02, m10, m11, m12, m20, m21, m22 = R
    tr = m00 + m11 + m22
    if tr > 0:
        s = 2 * math.sqrt(tr + 1)
        return ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, s / 4)
    if m00 > m11 and m00 > m22:
        s = 2 * math.sqrt(1 + m00 - m11 - m22)
        return (s / 4, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    if m11 > m22:
        s = 2 * math.sqrt(1 + m11 - m00 - m22)
        return ((m01 + m10) / s, s / 4, (m12 + m21) / s, (m02 - m20) / s)
    s = 2 * math.sqrt(1 + m22 - m00 - m11)
    return ((m02 + m20) / s, (m12 + m21) / s, s / 4, (m10 - m01) / s)


class ProtoDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        self.dt = self.robot.getBasicTimeStep() / 1000.0
        self.self_node = self.robot.getSelf()
        ms = int(self.robot.getBasicTimeStep())
        self.wheel = {n: self.robot.getDevice(f'{n}_wheel') for n in WHEELS}
        self.wheel_enc = {n: self.robot.getDevice(f'{n}_wheel_sensor') for n in WHEELS}
        for mot in self.wheel.values():
            mot.setPosition(float('inf'))
            mot.setVelocity(0.0)
        for s in self.wheel_enc.values():
            s.enable(ms)
        self.imu = self.robot.getDevice('imu')
        self.imu.enable(ms)
        self.gyro = self.robot.getDevice('gyro')
        self.gyro.enable(ms)
        self.prop = self.robot.getDevice('prop_motor')
        self.prop.setPosition(float('inf'))
        self.thrust = Thrust(os.environ.get('WCR_THROTTLE', 'full'))
        self.period = float(properties.get('publishPeriodMs', '20')) / 1000.0
        model.set_max_speed(float(os.environ.get('WCR_MAX_SPEED', model.MAX_SPEED)))
        self.wmax = {n: mot.getMaxVelocity() for n, mot in self.wheel.items()}

        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node('wcr_proto2_driver')
        self.node.create_subscription(Twist, 'cmd_vel', self._on_cmd, 1)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_odom = self.node.create_publisher(Odometry, 'odom', qos)
        self.pub_gt = self.node.create_publisher(Odometry, 'ground_truth', qos)
        self.pub_js = self.node.create_publisher(JointState, 'joint_states', qos)
        self.pub_adh = self.node.create_publisher(Float32, 'adhesion', qos)
        self.pub_thrust = self.node.create_publisher(Float32, 'thrust', qos)
        self.pub_clock = self.node.create_publisher(Clock, '/clock', 10)
        self.tf = TransformBroadcaster(self.node) if os.environ.get('WCR_ODOM_TF', '1') != '0' else None
        self.cmd, self.t_cmd = (0.0, 0.0), -1.0
        self.x = self.y = self.yaw = 0.0
        self.prev_wheel = None
        self.t_pub = -1.0
        self.cradle_removed = False
        self.prop_angle, self.t_prop = 0.0, 0.0
        self.node.get_logger().info(f'throttle {self.thrust.mode}: target thrust '
                                    f'{self.thrust.target() / G * 1000:.0f} g, robot {model.MASS * 1000:.0f} g')

    def _on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.t_cmd = self.robot.getTime()

    def _remove_cradle(self):
        """The deployment cradle (a ledge under the robot) only holds it until the thrust is on."""
        children = self.robot.getRoot().getField('children')
        for i in reversed(range(children.getCount())):
            node = children.getMFNode(i)
            name = node.getField('name') if node.getTypeName() == 'Solid' else None
            if name is not None and name.getSFString().startswith('cradle'):
                node.remove()
        self.cradle_removed = True

    def _stamp(self, t):
        return Time(sec=int(t), nanosec=int((t - int(t)) * 1e9))

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        t = self.robot.getTime()
        self.pub_clock.publish(Clock(clock=self._stamp(t)))
        vx, wz = self.cmd if t - self.t_cmd < CMD_TIMEOUT else (0.0, 0.0)
        wl, wr = inverse_kinematics(vx, wz)
        for n in WHEELS:
            w = wl if n in LEFT else wr
            self.wheel[n].setVelocity(max(-self.wmax[n], min(self.wmax[n], w)))
        f = self.thrust.update(self.imu.getQuaternion(), self.dt)
        self.self_node.addForceWithOffset([0.0, 0.0, -f], [0.0, 0.0, PROP_Z], True)   # at the hub, into the wall
        self.prop.setVelocity(prop_speed(f))
        if not self.cradle_removed and t > CRADLE_RELEASE:
            self._remove_cradle()

        # odometry in the wall plane: forward speed from the wheels, yaw rate from the gyro
        wheel = {n: s.getValue() for n, s in self.wheel_enc.items()}
        gyro = self.gyro.getValues()
        if not all(math.isfinite(v) for v in [*wheel.values(), *gyro]):
            return
        if self.prev_wheel is None:
            self.prev_wheel = wheel
        speed = {n: (wheel[n] - self.prev_wheel[n]) / self.dt for n in WHEELS}
        self.prev_wheel = wheel
        bvx, bwz = forward_speed(speed), gyro[2]
        self.x += math.cos(self.yaw) * bvx * self.dt
        self.y += math.sin(self.yaw) * bvx * self.dt
        self.yaw += bwz * self.dt

        if t - self.t_pub < self.period - 1e-9:
            return
        self.t_pub = t
        stamp = self._stamp(t)
        q = (0.0, 0.0, math.sin(self.yaw / 2), math.cos(self.yaw / 2))
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_link'
        p = odom.pose.pose
        p.position.x, p.position.y = self.x, self.y
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = bvx, bwz
        odom.pose.covariance = POSE_COV
        odom.twist.covariance = TWIST_COV
        self.pub_odom.publish(odom)
        if self.tf is not None:
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, 'odom', 'base_link'
            tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
            r = tf.transform.rotation
            r.x, r.y, r.z, r.w = q
            self.tf.sendTransform(tf)

        # propeller: a slow display angle for RViz / Foxglove (up to ~2 turns/s with thrust); the real
        # rotor (~900 rad/s) would only strobe at this publish rate
        w_disp = PROP_DISPLAY_W * math.sqrt(max(0.0, f) / (model.THRUST_MAX_G * G / 1000))
        self.prop_angle = (self.prop_angle + w_disp * (t - self.t_prop)) % (2 * math.pi)
        self.t_prop = t
        js = JointState()
        js.header.stamp = stamp
        js.name = [f'{n}_wheel_joint' for n in WHEELS] + ['prop_joint']
        js.position = [float(wheel[n]) for n in WHEELS] + [self.prop_angle]
        js.velocity = [float(speed[n]) for n in WHEELS] + [w_disp]
        self.pub_js.publish(js)

        gt = Odometry()
        gt.header.stamp, gt.header.frame_id, gt.child_frame_id = stamp, 'world', 'base_link'
        pos, R = self.self_node.getPosition(), self.self_node.getOrientation()
        gp = gt.pose.pose
        gp.position.x, gp.position.y, gp.position.z = pos
        gp.orientation.x, gp.orientation.y, gp.orientation.z, gp.orientation.w = quat_from_matrix(R)
        self.pub_gt.publish(gt)
        self.pub_adh.publish(Float32(data=float(f)))
        self.pub_thrust.publish(Float32(data=float(f / G * 1000)))
