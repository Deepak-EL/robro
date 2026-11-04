"""Models of the physical WCR prototype shared by the driver plugin (pure Python, no ROS / Webots).

  thrust    A2212 1000 KV + 1045 prop on 3S. Throttle settings 2..7 are the team's servo-tester knob
            positions with the thrust measured on a kitchen scale; 'full' is the typical datasheet
            value for this motor / prop / 3S (~850 g); 'auto' is the planned upgrade (ESP32 drives the
            ESC from the IMU tilt): just enough thrust for the surface angle x SAFETY, capped at 'full'.
            The motor spins up with a first-order lag (TAU).
  drive     4 x N20 (60 rpm) skid steer: body twist (vx, wz) -> left / right wheel speeds. A skid-steer
            robot turns slower than the wheel speed difference suggests (the wheels scrub
            sideways); SKID = effective track / geometric track, applied both ways.
"""
import math

G = 9.81
MASS = 0.439                 # kg: 369 g hardware (370 g measured) + 70 g sensor kit (cad/parts.json)
MU = 0.7                     # rubber tyre on concrete (world contact properties)
COM_H, HALF_WB = 0.029, 0.10  # m: centre of mass above the wall, half wheelbase (tipping check)

THRUST_G = {'2': 100.0, '3': 200.0, '4': 250.0, '5': 300.0, '6': 350.0, '7': 400.0, 'full': 850.0}
THRUST_MAX_G = THRUST_G['full']
SAFETY, TAU = 1.3, 0.15

WHEEL_R = 0.0175
TRACK = 0.19                 # m, left-right wheel centres
SKID = 1.5                   # effective / geometric track for turning (wheelbase 0.20 >= track 0.19; Webots test)
N20_RPM = 60.0
MAX_W = N20_RPM * 2 * math.pi / 60          # 6.28 rad/s
MAX_SPEED = MAX_W * WHEEL_R                 # 0.11 m/s
speed_scale = 1.0


def set_max_speed(v):
    """Allow wheel speeds up to v m/s (faster-than-real demos only)."""
    global speed_scale
    speed_scale = max(1.0, v / MAX_SPEED)


def surface_angle(q):
    """Angle between the robot's z axis (out of the wall) and world up, from an (x, y, z, w)
    quaternion; None while the reading is invalid."""
    if not q or not all(math.isfinite(v) and abs(v) <= 1.01 for v in q):
        return None
    return math.acos(max(-1.0, min(1.0, 1 - 2 * (q[0] ** 2 + q[1] ** 2))))


def required_thrust(beta):
    """Pressing force [N] that keeps the robot from sliding (friction) and peeling (tipping)."""
    s, c = math.sin(beta), math.cos(beta)
    return max(0.0, MASS * G * (s / MU - c), MASS * G * abs(s) * COM_H / HALF_WB)


class Thrust:
    """Propeller thrust [N] for a throttle setting ('2'..'7', 'full', 'auto' or grams)."""

    def __init__(self, throttle='full'):
        self.mode = str(throttle)
        self.force = 0.0
        self.beta = None

    def target(self):
        if self.mode == 'auto':
            need = SAFETY * required_thrust(self.beta) if self.beta is not None else THRUST_MAX_G * G / 1000
            return min(THRUST_MAX_G * G / 1000, need)
        g = THRUST_G[self.mode] if self.mode in THRUST_G else float(self.mode)
        return g * G / 1000

    def update(self, quaternion, dt):
        beta = surface_angle(quaternion)
        if beta is not None:
            self.beta = beta
        self.force += (self.target() - self.force) * min(1.0, dt / TAU)
        return self.force


def prop_speed(force):
    """Visual rotor speed [rad/s] (thrust ~ rpm^2)."""
    return 15.0 + 45.0 * math.sqrt(max(0.0, force) / (THRUST_MAX_G * G / 1000))


def inverse_kinematics(vx, wz):
    """Body twist -> (left, right) wheel speeds [rad/s], scaled together to the N20 limit."""
    half = TRACK * SKID / 2
    vl, vr = vx - wz * half, vx + wz * half
    lim = MAX_W * speed_scale * WHEEL_R
    k = max(1.0, abs(vl) / lim, abs(vr) / lim)
    return vl / k / WHEEL_R, vr / k / WHEEL_R


def forward_speed(wheel_speed):
    """Forward speed [m/s] from {wheel: rad/s} (mean of the four wheels)."""
    return sum(wheel_speed.values()) / len(wheel_speed) * WHEEL_R
