"""CAD model of the physical WCR prototype (the thrust-adhesion foam-board robot), for its digital twin.

Rough measurements from the build photo and the team's parts list; every number is a constant
below, so a measured value can replace an estimate in one place.

Frame (mm): origin = centre of the foam board, on its WALL-SIDE face (z = 0).
  +x forward (L298N end), +y left, +z away from the wall (the component side).
The propeller blows air away from the wall (+z), so its thrust presses the robot onto the wall.

  python tools/build_proto_cad.py  -> cad/  (wcr_proto.step, preview.png, preview_top.png, parts.json)
                                      meshes/ (body, wheel, prop as .stl + .obj, for Webots / ROS)
needs: cadquery trimesh numpy pillow
"""
import json
import math
from pathlib import Path

import cadquery as cq
import numpy as np

ROOT = Path(__file__).resolve().parents[1]

# ---------------------------------------------------------------- measurements (mm, g)
BOARD_L, BOARD_W, BOARD_T = 290.0, 200.0, 5.0      # 5 mm foam board, 29 x 20 cm
HOLE_D = 170.0                                     # round cut-out under the propeller
WHEEL_X = 100.0                                    # wheel axles at x = +-100 (wheelbase 200)
WHEEL_D, WHEEL_W = 35.0, 10.0                      # 3.5 cm tyre, 1 cm wide
WHEEL_OUT_Y = BOARD_W / 2                          # wheel outer face flush with the board side
N20_W, N20_H = 12.0, 10.0                          # N20 cross-section (flats top/bottom)
N20_GEAR, N20_MOTOR, N20_CAP, N20_SHAFT = 9.0, 15.0, 2.0, 10.0
AXLE_Z = BOARD_T + N20_H / 2                       # N20 lies flat in its clamp on the board
STRIP_L, STRIP_W, STRIP_T = 200.0, 25.0, 4.0       # plywood motor strip across the cut-out, front-back
PROP_D = 254.0                                     # 1045 propeller: 10 in diameter, 4.5 in pitch
MOTOR_D = 27.7                                     # A2212 1000 KV outrunner

MASS = dict(foam_board=60, plywood_strip=12, bldc_motor=60, propeller=12, esc=24, n20_motor=10,
            wheel=20, n20_clamp=1.5, l298n=25, esp32=10, zip_tie=0.5,
            lidar=42, lidar_mast=10, camera=6, camera_mast=4, imu=2)

# Sensor kit for the autonomous twin (NOT on the hardware yet): an LD06-class 2D lidar on a plywood
# mast at the rear (scan plane 110 mm above the wall, clear of the propeller disc), two small camera
# modules on a front mast (wall camera tilted 70 deg down, front camera looking ahead) and an
# MPU6050 IMU. Positions in this CAD frame (mm); the twin's sensors sit at the same places.
SENSOR_KIT = True
LIDAR_AT = (-139.0, 0.0, 102.5)          # scan-plane centre
WALL_CAM_AT = (155.0, 40.0, 82.5)        # camera centre, looks +x pitched 70 deg toward the wall
WALL_CAM_PITCH = 70.0
FRONT_CAM_AT = (146.0, 22.0, 52.5)       # looks +x
IMU_AT = (-118.0, 6.0, BOARD_T)
WIRING_G = 38.0          # jumpers, 14/16 AWG leads, XT60 (rest of the measured 370 g): spread over the board
TOTAL_MEASURED_G = 370   # whole robot on the scale (battery stays off the robot, XT60 lead)

COL = dict(foam=(0.94, 0.88, 0.92), ply=(0.78, 0.62, 0.42), black=(0.06, 0.06, 0.07),
           tyre=(0.07, 0.07, 0.08), white=(0.93, 0.92, 0.88), steel=(0.72, 0.73, 0.75),
           copper=(0.80, 0.45, 0.20), red=(0.75, 0.10, 0.10), green=(0.10, 0.60, 0.25),
           blue=(0.15, 0.25, 0.55), cap=(0.10, 0.10, 0.35), dark=(0.20, 0.21, 0.24),
           yellow=(0.90, 0.75, 0.10), gold=(0.85, 0.70, 0.30), brown=(0.40, 0.25, 0.12),
           orange=(0.95, 0.50, 0.10))


# ---------------------------------------------------------------- helpers
def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane('XY').box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cyl(axis, c, r, a0, a1):
    """Cylinder along 'x' | 'y' | 'z' through point c (the other two coordinates), from a0 to a1."""
    wp = {'z': cq.Workplane('XY'), 'x': cq.Workplane('YZ'), 'y': cq.Workplane('XZ')}[axis]
    i = 'xyz'.index(axis)
    u, v = [c[k] for k in range(3) if k != i]
    if axis == 'y':                                    # XZ workplane: normal -y
        return wp.workplane(offset=-a1).center(u, v).circle(r).extrude(a1 - a0)
    return wp.workplane(offset=a0).center(u, v).circle(r).extrude(a1 - a0)


def revolve(points):
    """Closed (r, z) profile revolved about z."""
    return cq.Workplane('XZ').polyline(points).close().revolve(360, (0, 0, 0), (0, 1, 0))


def soft(shape, r, sel='|Z'):
    try:
        return shape.edges(sel).fillet(r)
    except Exception:
        return shape


def T(x=0, y=0, z=0):
    return cq.Location(cq.Vector(x, y, z))


def R(axis, deg):
    return cq.Location(cq.Vector(0, 0, 0), cq.Vector(*{'x': (1, 0, 0), 'y': (0, 1, 0), 'z': (0, 0, 1)}[axis]), deg)


# ---------------------------------------------------------------- structure
def foam_board():
    b = soft(box(-BOARD_L / 2, BOARD_L / 2, -BOARD_W / 2, BOARD_W / 2, 0, BOARD_T), 4)
    nx = WHEEL_D / 2 + 3.5                                 # wheel notches, open to the sides
    ny = WHEEL_W + 2
    for x in (-WHEEL_X, WHEEL_X):
        for s in (-1, 1):
            y0 = s * (WHEEL_OUT_Y - ny)
            b = b.cut(box(x - nx, x + nx, min(y0, s * 120), max(y0, s * 120), -1, BOARD_T + 1))
    return b.cut(cyl('z', (0, 0, 0), HOLE_D / 2, -1, BOARD_T + 1))


def plywood_strip():
    """Plywood strip across the cut-out (front-back, i.e. vertical when climbing); the motor sits on it."""
    return box(-STRIP_L / 2, STRIP_L / 2, -STRIP_W / 2, STRIP_W / 2, BOARD_T, BOARD_T + STRIP_T)


def zip_tie(z0, z1, half_w):
    """Black zip-tie loop around a stack from z0 to z1, half_w wide, band across y at x = 0."""
    t = 1.2
    return box(-1.5, 1.5, -half_w - t, half_w + t, z0 - t, z1 + t).cut(
        box(-2, 2, -half_w, half_w, z0, z1))


# ---------------------------------------------------------------- thrust unit (z up from the strip top)
MOUNT_Z = BOARD_T + STRIP_T                                # 9: top of the strip


def bldc_motor():
    """A2212 outrunner with cross mount, stator, copper bell, shaft and prop adapter (motor link)."""
    z = MOUNT_Z
    mount = cyl('z', (0, 0, 0), 11, z, z + 3)
    for a in (45, 135):
        arm = box(-19, 19, -3, 3, z, z + 3).rotate((0, 0, 0), (0, 0, 1), a)
        mount = mount.union(soft(arm, 2.5))
    for a in (45, 135, 225, 315):                          # screw holes in the arm ends
        c = (16 * math.cos(math.radians(a)), 16 * math.sin(math.radians(a)), 0)
        mount = mount.cut(cyl('z', c, 1.6, z - 1, z + 4))
    stator = cyl('z', (0, 0, 0), MOTOR_D / 2 - 0.6, z + 3, z + 11)
    bell = soft(cyl('z', (0, 0, 0), MOTOR_D / 2, z + 11, z + 29), 1.5, '>Z')
    shaft = cyl('z', (0, 0, 0), 1.6, z + 29, z + 33)
    adapter = cyl('z', (0, 0, 0), 4.5, z + 29, z + 33)
    return dict(motor_mount=(mount, COL['steel']), motor_stator=(stator, COL['dark']),
                motor_bell=(bell, COL['copper']), motor_shaft=(shaft.union(adapter), COL['steel']))


PROP_Z = MOUNT_Z + 37                                      # 46: prop hub centre
PROP_LOC = T(z=PROP_Z) * R('z', 35)                        # blades drawn at 35 deg, like the photo


def propeller():
    """1045 two-blade prop in its own frame (hub centre at the origin, axis z) + spinner nut."""
    pitch = 13.0                                           # mean blade angle (4.5 in pitch at ~0.7 R)
    rt = PROP_D / 2
    plan = [(7, -7), (35, -11), (80, -9), (115, -5.5), (rt - 1, -2), (rt, 0), (rt - 1, 2.5),
            (115, 6), (80, 9), (35, 10.5), (7, 7)]
    blade = (cq.Workplane('XY').polyline(plan).close().extrude(2.0).translate((0, 0, -1))
             .rotate((0, 0, 0), (1, 0, 0), pitch))
    prop = cyl('z', (0, 0, 0), 8, -4, 4).union(blade).union(blade.rotate((0, 0, 0), (0, 0, 1), 180))
    prop = prop.cut(cyl('z', (0, 0, 0), 1.6, -5, 5))
    nut = revolve([(0, 4), (4.5, 4), (4.5, 7), (3.5, 11), (1.5, 15), (0, 16)])
    return dict(propeller=(prop, COL['black']), spinner_nut=(nut, COL['steel']))


# ---------------------------------------------------------------- drive unit (N20 frame: axis = +y, output face y = 0)
def n20_motor():
    def flat_cyl(r, y0, y1):
        return cyl('y', (0, 0, 0), r, y0, y1).intersect(box(-r, r, y0, y1, -N20_H / 2, N20_H / 2))
    gear = box(-N20_W / 2, N20_W / 2, -N20_GEAR, 0, -N20_H / 2, N20_H / 2)
    can = flat_cyl(N20_W / 2, -N20_GEAR - N20_MOTOR, -N20_GEAR - 0.5)
    cap = flat_cyl(N20_W / 2 - 0.8, -N20_GEAR - N20_MOTOR - N20_CAP, -N20_GEAR - N20_MOTOR)
    for x in (-3, 3):                                      # solder tabs
        cap = cap.union(box(x - 1, x + 1, -N20_GEAR - N20_MOTOR - N20_CAP - 2, -N20_GEAR - N20_MOTOR, -0.3, 0.3))
    shaft = cyl('y', (0, 0, 0), 1.5, 0, N20_SHAFT)
    return dict(n20_gearbox=(gear, COL['steel']), n20_can=(can.union(shaft), COL['steel']),
                n20_endcap=(cap, COL['black']))


CLAMP_Y = (-N20_GEAR - 12.5, -N20_GEAR - 2.5)              # 10 mm wide band over the motor can


def n20_clamp():
    """Standard N20 clamp (white bracket): bridge over the motor, two legs, two ears with screw holes."""
    y0, y1 = CLAMP_Y
    zb, zt = -N20_H / 2, N20_H / 2 + 0.1                   # board top, inside of the bridge
    wi, wo, t = N20_W / 2 + 0.1, N20_W / 2 + 1.6, 1.6
    c = soft(box(-wo, wo, y0, y1, zt, zt + t), 1.2, '|Y and >Z')
    for s in (-1, 1):
        c = c.union(box(min(s * wi, s * wo), max(s * wi, s * wo), y0, y1, zb, zt + t))
        ear = box(min(s * wo, s * 11), max(s * wo, s * 11), (y0 + y1) / 2 - 3.45, (y0 + y1) / 2 + 3.45, zb, zb + t)
        ear = ear.union(cyl('z', (s * 11, (y0 + y1) / 2, 0), 3.45, zb, zb + t))
        c = c.union(ear).cut(cyl('z', (s * 11, (y0 + y1) / 2, 0), 1.1, zb - 1, zb + t + 1))
    return c


def wheel():
    """Wheel in its own frame: axle = y, centred; outer face at +y. Black tread tyre, white hub."""
    r, ri, h = WHEEL_D / 2, 12.5, WHEEL_W / 2
    tyre = revolve([(ri, -h), (r - 1.5, -h), (r, -h + 1.5), (r, h - 1.5), (r - 1.5, h), (ri, h)])
    for k in range(22):                                    # staggered tread blocks
        z0, z1 = (-h - 1, -0.6) if k % 2 else (0.6, h + 1)
        g = box(r - 1.0, r + 1, -0.7, 0.7, z0, z1).rotate((0, 0, 0), (0, 0, 1), k * 360 / 22)
        tyre = tyre.cut(g)
    hub = revolve([(1.5, -h + 0.5), (ri, -h + 0.5), (ri, h - 0.5), (9, h - 0.5), (9, h - 2),
                   (4, h - 2), (4, h - 0.5), (1.5, h - 0.5)])
    for k in range(10):                                    # ring of lightening holes (photo)
        a = math.radians(k * 36)
        hub = hub.cut(cyl('z', (6.5 * math.cos(a), 6.5 * math.sin(a), 0), 1.0, h - 2.5, h))
    to_y = R('x', -90)                                     # +z face -> +y (outer)
    return dict(tyre=(tyre.val().moved(to_y), COL['tyre']), hub=(hub.val().moved(to_y), COL['white']))


# ---------------------------------------------------------------- electronics (local z = 0 on the board top)
def l298n():
    p = {}
    so = None
    for x in (-18, 18):
        for y in (-18, 18):
            s = cyl('z', (x, y, 0), 2.0, 0, 3)
            so = s if so is None else so.union(s)
    p['l298n_standoffs'] = (so, COL['steel'])
    p['l298n_pcb'] = (soft(box(-21.5, 21.5, -21.5, 21.5, 3, 4.6), 1), COL['red'])
    hs = box(-11.5, 11.5, 4, 20, 4.6, 29.6)
    for k in range(6):
        x = -9.5 + k * 3.8
        hs = hs.cut(box(x, x + 1.6, 3, 21, 11, 31))
    p['l298n_heatsink'] = (hs, COL['black'])
    tb = box(-21, -14, -5, 5, 4.6, 14.6).union(box(14, 21, -5, 5, 4.6, 14.6)).union(
        box(-7.5, 7.5, -21, -14, 4.6, 14.6))
    p['l298n_terminals'] = (tb, COL['green'])
    p['l298n_capacitor'] = (cyl('z', (-11, -9, 0), 4, 4.6, 15), COL['cap'])
    return p


def esp32():
    p = {}
    hd = box(-19, 19, 11, 13.5, 0, 2.5).union(box(-19, 19, -13.5, -11, 0, 2.5))
    p['esp32_pins'] = (hd.union(box(-19, 19, 11, 13.5, 4.1, 6.6)).union(box(-19, 19, -13.5, -11, 4.1, 6.6)),
                       COL['black'])
    p['esp32_pcb'] = (box(-25.5, 25.5, -14, 14, 2.5, 4.1), COL['black'])
    p['esp32_module'] = (box(-2, 15.5, -9, 9, 4.1, 6.9).union(box(15.5, 23.5, -9, 9, 4.1, 4.9)),
                         COL['steel'])
    p['esp32_usb'] = (box(-25.5, -19.5, -4, 4, 4.1, 7.0), COL['steel'])
    return p


def esc():
    """40 A ESC, long axis x: heat-shrunk body with a printed label, 3 motor leads with bullet
    connectors toward +x (the motor), battery leads and the signal lead toward -x."""
    L, W, H = 50.0, 25.0, 8.0
    body = soft(box(-L / 2, L / 2, -W / 2, W / 2, 0, H), 3.0, '|X')          # shrink-wrapped edges
    for s in (-1, 1):                                                           # tapered shrink ends
        body = body.cut(box(s * L / 2 - 1.5, s * L / 2 + 1.5, -W, W, H - 1.2, H + 1).union(
            box(s * L / 2 - 1.5, s * L / 2 + 1.5, -W, W, -1, 1.2)))
    p = {'esc_body': (body, COL['black'])}
    label = box(-17, 17, -10, 10, H - 0.01, H + 0.3)
    stripe = box(-17, 17, 5, 10, H + 0.3, H + 0.45)
    p['esc_label'] = (label, COL['dark'])
    p['esc_label_stripe'] = (stripe, COL['blue'])
    try:
        txt = (cq.Workplane('XY').workplane(offset=H + 0.3).center(-2, -3)
               .text('40A', 9, 0.4, kind='bold', halign='center', valign='center'))
        p['esc_label_text'] = (txt, COL['white'])
    except Exception:
        pass
    leads = {}
    for y, c in ((-6, 'black'), (0, 'red'), (6, 'yellow')):                    # motor leads + bullets
        leads.setdefault(c, []).append(cyl('x', (0, y, H / 2), 1.1, L / 2 - 1, L / 2 + 18))
        leads.setdefault('gold', []).append(cyl('x', (0, y, H / 2), 1.7, L / 2 + 18, L / 2 + 26))
    for y, c in ((-5, 'black'), (3, 'red')):                                   # 14 AWG battery leads
        leads.setdefault(c, []).append(cyl('x', (0, y, H / 2), 1.5, -L / 2 - 14, -L / 2 + 1))
    for y, c in ((8.5, 'brown'), (9.7, 'red'), (10.9, 'orange')):              # signal lead + plug
        leads.setdefault(c, []).append(cyl('x', (0, y, H / 2), 0.55, -L / 2 - 16, -L / 2 + 1))
    leads.setdefault('black', []).append(box(-L / 2 - 24, -L / 2 - 16, 7.4, 12.0, H / 2 - 1.3, H / 2 + 1.3))
    for c, shapes in leads.items():
        u = shapes[0]
        for s in shapes[1:]:
            u = u.union(s)
        p[f'esc_wires_{c}'] = (u, COL[c])
    return p


# ---------------------------------------------------------------- sensor kit (CAD frame)
def lidar_kit():
    x, y, zs = LIDAR_AT
    zb = zs - 28.0                                         # LD06: scan plane ~28 mm above its base
    mast = box(x - 6, x + 6, y - 10, y + 10, BOARD_T, zb)
    body = soft(box(x - 19.3, x + 19.3, y - 19.3, y + 19.3, zb, zb + 20), 3)
    turret = cyl('z', (x, y, 0), 17, zb + 20, zb + 33.5)
    window = cyl('z', (x, y, 0), 17.3, zs - 4, zs + 4)
    return dict(lidar_mast=(mast, COL['ply']), lidar_body=(body.union(turret.cut(window)), COL['dark']),
                lidar_window=(window.cut(cyl('z', (x, y, 0), 14, zs - 5, zs + 5)), COL['black']))


def camera_module(c, pitch):
    """Small camera board (24 x 24 mm) facing +x, lens out front; pitched toward the wall (-z)."""
    pcb = box(-1.5, 1.5, -12, 12, -12, 12)
    lens = cyl('x', (0, 0, 0), 4.5, 1.5, 8).union(cyl('x', (0, 0, 0), 3, 8, 9))
    loc = T(*c) * R('y', pitch)
    return pcb.val().moved(loc), lens.val().moved(loc)


def camera_kit():
    x, y = WALL_CAM_AT[0] - 17, WALL_CAM_AT[1]
    mast = box(x - 5, x + 5, y - 5, y + 5, BOARD_T, WALL_CAM_AT[2] + 6)
    arm = box(x - 5, WALL_CAM_AT[0] - 4, y - 5, y + 5, WALL_CAM_AT[2] + 6, WALL_CAM_AT[2] + 10)
    arm2 = box(x - 5, FRONT_CAM_AT[0] - 1.5, FRONT_CAM_AT[1] - 5, y - 5, FRONT_CAM_AT[2] - 4, FRONT_CAM_AT[2] + 4)
    p = {'camera_mast': (mast.union(arm).union(arm2), COL['ply'])}
    pcb_w, lens_w = camera_module(WALL_CAM_AT, WALL_CAM_PITCH)
    pcb_f, lens_f = camera_module(FRONT_CAM_AT, 0)
    p['wall_camera'] = (pcb_w, COL['blue'])
    p['wall_camera_lens'] = (lens_w, COL['black'])
    p['front_camera'] = (pcb_f, COL['blue'])
    p['front_camera_lens'] = (lens_f, COL['black'])
    return p


def imu_board():
    x, y, z = IMU_AT
    return dict(imu=(box(x - 10.5, x + 10.5, y - 8, y + 8, z + 2, z + 3.6), COL['blue']),
                imu_chip=(box(x - 2, x + 2, y - 2, y + 2, z + 3.6, z + 4.5), COL['black']))


# ---------------------------------------------------------------- assembly
GROUPS = {}          # link name -> list of (shape, location) in the assembly frame, for the meshes


def place(assy, name, shape, loc, color, link, kind):
    shp = shape.val() if isinstance(shape, cq.Workplane) else shape
    assy.add(shp, name=name, loc=loc, color=cq.Color(*color), metadata={'kind': kind})
    GROUPS.setdefault(link, []).append((shp, loc, color))
    return shp


def corners():
    """(name, x, side): side +1 = left (+y), -1 = right."""
    return [('FL', WHEEL_X, 1), ('FR', WHEEL_X, -1), ('BL', -WHEEL_X, 1), ('BR', -WHEEL_X, -1)]


def n20_loc(x, side):
    face_y = WHEEL_OUT_Y - (WHEEL_W + 1)                   # wheel inner face 1 mm off the gearbox
    loc = T(x, side * face_y, AXLE_Z)
    return loc * R('z', 180) if side < 0 else loc


def wheel_loc(x, side):
    return n20_loc(x, side) * T(0, 1 + WHEEL_W / 2, 0)


L298N_AT = (122.0, -32.0)                                  # front, right of the strip
ESP32_AT = (-112.0, 38.0)                                  # rear left
ESC_AT = (-118.0, -40.0)                                   # rear right, motor leads toward the motor
PLACED = []          # (name, kind, mass_g, centre_mm)


def build():
    a = cq.Assembly(name='wcr_proto')

    def add(name, shape, loc, color, link, kind, mass=None):
        shp = place(a, name, shape, loc, color, link, kind)
        if mass is not None:
            c = shp.moved(loc).Center()
            PLACED.append((name, kind, mass, (c.x, c.y, c.z)))

    add('foam_board', foam_board(), T(), COL['foam'], 'body', 'foam_board', MASS['foam_board'])
    add('plywood_strip', plywood_strip(), T(), COL['ply'], 'body', 'plywood_strip', MASS['plywood_strip'])
    for k, (n, (s, c)) in enumerate(bldc_motor().items()):
        add(n, s, T(), c, 'body', 'bldc_motor', MASS['bldc_motor'] if n == 'motor_bell' else None)
    for i, x in enumerate((-9, 9)):                        # ties over the motor mount
        add(f'zip_tie_motor_{i}', zip_tie(BOARD_T, MOUNT_Z + 3, STRIP_W / 2), T(x), COL['black'], 'body',
            'zip_tie', MASS['zip_tie'])
    for i, x in enumerate((-92, 92)):                      # ties holding the strip on the board
        add(f'zip_tie_strip_{i}', zip_tie(0, MOUNT_Z, STRIP_W / 2), T(x), COL['black'], 'body',
            'zip_tie', MASS['zip_tie'])
    for n, (s, c) in propeller().items():
        add(n, s, PROP_LOC, c, 'prop', 'propeller', MASS['propeller'] if n == 'propeller' else None)

    for cn, x, side in corners():
        for n, (s, c) in n20_motor().items():
            add(f'{cn}_{n}', s, n20_loc(x, side), c, 'body', 'n20_motor',
                MASS['n20_motor'] if n == 'n20_can' else None)
        add(f'{cn}_n20_clamp', n20_clamp(), n20_loc(x, side), COL['white'], 'body', 'n20_clamp', MASS['n20_clamp'])
        for n, (s, c) in wheel().items():
            add(f'{cn}_{n}', s, wheel_loc(x, side), c, f'wheel_{cn}', 'wheel', MASS['wheel'] if n == 'tyre' else None)

    for parts, at, kind in ((l298n(), L298N_AT, 'l298n'), (esp32(), ESP32_AT, 'esp32'),
                            (esc(), ESC_AT, 'esc')):
        first = True
        for n, (s, c) in parts.items():
            add(n, s, T(at[0], at[1], BOARD_T), c, 'body', kind, MASS[kind] if first else None)
            first = False
    if SENSOR_KIT:
        kit_mass = dict(lidar_body=MASS['lidar'], lidar_mast=MASS['lidar_mast'], camera_mast=MASS['camera_mast'],
                        wall_camera=MASS['camera'], front_camera=MASS['camera'], imu=MASS['imu'])
        for parts in (lidar_kit(), camera_kit(), imu_board()):
            for n, (s, c) in parts.items():
                add(n, s, T(), c, 'body', 'sensor_kit', kit_mass.get(n))
    PLACED.append(('wiring', 'wiring', WIRING_G, (0.0, 0.0, BOARD_T + 3)))
    return a


# ---------------------------------------------------------------- outputs
def preview(assy, png, az_deg, el_deg, s=3.0):
    """Coloured preview (point splatting of the tessellated parts)."""
    import trimesh
    from PIL import Image
    az, el = math.radians(az_deg), math.radians(el_deg)
    Rz = np.array([[math.cos(az), -math.sin(az), 0], [math.sin(az), math.cos(az), 0], [0, 0, 1]])
    E = np.array([[1, 0, 0], [0, math.cos(el), -math.sin(el)], [0, math.sin(el), math.cos(el)]])
    V = E @ np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]) @ Rz
    light = np.array([0.3, 0.5, 0.8]) / np.linalg.norm([0.3, 0.5, 0.8])
    P, C = [], []
    for shape, name, loc, color in assy:
        v, f = shape.moved(loc).tessellate(0.2, 0.3)
        m = trimesh.Trimesh(np.array([p.toTuple() for p in v]), np.array(f), process=False)
        col = np.array(color.toTuple()[:3]) * 255 if color else np.array([200, 200, 200])
        pts, fi = trimesh.sample.sample_surface(m, int(max(1500, m.area * 25)))
        sh = 0.35 + 0.65 * np.abs(m.face_normals[fi] @ V.T @ light)
        P.append(pts @ V.T)
        C.append(np.clip(np.maximum(col, 25) * sh[:, None], 0, 255))
    P, C = np.vstack(P), np.vstack(C)
    u = ((P[:, 0] - P[:, 0].min()) * s).astype(int) + 20
    v = ((P[:, 1].max() - P[:, 1]) * s).astype(int) + 20
    img = np.full((v.max() + 21, u.max() + 21, 3), 255.0)
    o = np.argsort(P[:, 2])
    img[v[o], u[o]] = C[o]
    Image.fromarray(img.astype(np.uint8)).save(png)


def export_meshes(out):
    """Meshes per simulation link and colour, in that link's frame (mm): <link>_<k>.obj / .stl,
    listed with their colours in meshes.json (read by tools/build_proto_webots.py).
      body   CAD frame (everything fixed: board, strip, motor, N20s, clamps, electronics, sensors)
      wheel  wheel frame (axle = y, outer face +y; left wheel, rotate 180 deg about z for the right)
      prop   prop frame (hub centre, axis z)"""
    import trimesh
    out.mkdir(parents=True, exist_ok=True)
    for f in [*out.glob('*.obj'), *out.glob('*.stl')]:
        f.unlink()
    frames = dict(body=cq.Location(), wheel=wheel_loc(*corners()[0][1:]), prop=PROP_LOC)
    links = dict(body=GROUPS['body'], wheel=GROUPS['wheel_FL'], prop=GROUPS['prop'])
    index = {}
    for stem, items in links.items():
        inv = frames[stem].inverse
        by_col = {}
        for shp, loc, col in items:
            v, f = shp.moved(inv * loc).tessellate(0.08, 0.2)
            by_col.setdefault(col, []).append(trimesh.Trimesh(np.array([p.toTuple() for p in v]), np.array(f)))
        index[stem] = []
        for k, (col, ms) in enumerate(by_col.items()):
            m = trimesh.util.concatenate(ms)
            name = f'{stem}_{k}'
            m.export(out / f'{name}.stl')
            trimesh.graph.smooth_shade(m, angle=math.radians(30)).export(out / f'{name}.obj', include_normals=True)
            index[stem].append(dict(file=name, color=list(col)))
        print(f'  {stem}: {len(by_col)} colour meshes, {sum(len(m.faces) for ms in by_col.values() for m in ms)} tris')
    (out / 'meshes.json').write_text(json.dumps(index, indent=1))


def main():
    out = ROOT / 'cad'
    out.mkdir(parents=True, exist_ok=True)
    assy = build()
    step = out / 'wcr_proto.step'
    assy.export(str(step), write_pcurves=False)
    total = sum(m for _, _, m, _ in PLACED)
    com = np.sum([np.array(c) * m for _, _, m, c in PLACED], axis=0) / total
    kinds = {}
    for _, k, m, _ in PLACED:
        kinds[k] = kinds.get(k, 0) + m
    bb = assy.toCompound().BoundingBox()
    manifest = dict(
        units='mm, g', frame='origin board centre on the wall-side face; +x forward (L298N), +y left, +z away from wall',
        step=step.name, total_mass_g=round(total, 1), measured_mass_g=TOTAL_MEASURED_G,
        hardware_mass_g=round(sum(m for _, k, m, _ in PLACED if k != 'sensor_kit'), 1),
        sensor_kit_mass_g=round(sum(m for _, k, m, _ in PLACED if k == 'sensor_kit'), 1),
        sensors_mm=dict(lidar=LIDAR_AT, wall_camera=WALL_CAM_AT, wall_camera_pitch_deg=WALL_CAM_PITCH,
                        front_camera=FRONT_CAM_AT, imu=IMU_AT) if SENSOR_KIT else None,
        centre_of_mass_mm=[round(float(v), 1) for v in com],
        size_mm=[round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)],
        wheel=dict(diameter=WHEEL_D, width=WHEEL_W, axle_z=AXLE_Z, wall_gap_board=round(WHEEL_D / 2 - AXLE_Z, 1),
                   positions={n: [x, side * (WHEEL_OUT_Y - WHEEL_W / 2)] for n, x, side in corners()}),
        prop=dict(diameter=PROP_D, hub_z=PROP_Z, blows='+z (away from wall)'),
        mass_by_kind_g=kinds,
        parts=[dict(name=n, kind=k, mass_g=m, centre_mm=[round(v, 1) for v in c]) for n, k, m, c in PLACED])
    (out / 'parts.json').write_text(json.dumps(manifest, indent=1))
    print(f'{step}: {step.stat().st_size / 1e6:.2f} MB, {len(list(assy))} solids')
    print(f'mass {total:.0f} g (measured {TOTAL_MEASURED_G} g), CoM {com.round(1)} mm, size {manifest["size_mm"]} mm')
    print(f'board-to-wall gap {manifest["wheel"]["wall_gap_board"]} mm')
    preview(assy, out / 'preview.png', -50, 28)
    preview(assy, out / 'preview_top.png', 90, 90)
    export_meshes(ROOT / 'ros' / 'wcr_proto' / 'meshes')


if __name__ == '__main__':
    main()
