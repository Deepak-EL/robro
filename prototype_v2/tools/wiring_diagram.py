"""Fritzing-style wiring diagram of the WCR prototype (v2) -> docs/figures/wiring_diagram.png / .svg

Parts: 3S LiPo (off-board, XT60 lead), 40 A ESC + A2212 1000 KV (1045 prop), ESP32 DevKit (30 pin),
L298N, 4 x N20 60 rpm. The planned sensor kit of the autonomous twin (MPU6050, LD06 lidar) is drawn
dashed with net labels. The ESP32 pin numbers are a suggested mapping (PINS below): change them to
match the robot's ESP32 code.
  python tools/wiring_diagram.py
"""
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.patheffects as pe
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyBboxPatch, Polygon, Rectangle, Arc, Ellipse

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'docs' / 'figures'

# ESP32 DevKit V1 (30 pin), USB at the bottom; headers top -> bottom
ESP_LEFT = ['EN', 'VP', 'VN', 'D34', 'D35', 'D32', 'D33', 'D25', 'D26', 'D27', 'D14', 'D12', 'D13', 'GND', 'VIN']
ESP_RIGHT = ['D23', 'D22', 'TX0', 'RX0', 'D21', 'D19', 'D18', 'D5', 'TX2', 'RX2', 'D4', 'D2', 'D15', 'GND', '3V3']
PINS = dict(ENA='D32', IN1='D33', IN2='D25', IN3='D26', IN4='D27', ENB='D14', ESC='D18',
            SDA='D21', SCL='D22', LIDAR_RX='RX2')

C = dict(red='#d32f2f', black='#212121', orange='#ef8a17', brown='#7b4a1e', yellow='#f4c20d', green='#2e9e44',
         blue='#1e6fd9', purple='#8e44ad', white='#f5f5f5', grey='#9e9e9e', cyan='#17a2b8', pink='#e84393')

fig, ax = plt.subplots(figsize=(16, 10.4), dpi=150)
ax.set_xlim(0, 160)
ax.set_ylim(0, 104)
ax.set_aspect('equal')
ax.axis('off')
fig.patch.set_facecolor('white')

WIRES = []          # (points, color, lw, dashed)
DOTS = []


def wire(pts, color, lw=2.2, dashed=False):
    WIRES.append((pts, color, lw, dashed))


def rbox(x0, y0, x1, y1, fc, ec='#111', r=0.8, lw=1.0, z=1, ls='-'):
    ax.add_patch(FancyBboxPatch((x0, y0), x1 - x0, y1 - y0, boxstyle=f'round,pad=0,rounding_size={r}',
                                fc=fc, ec=ec, lw=lw, zorder=z, ls=ls))


def text(x, y, s, size=7, color='#111', ha='center', va='center', weight='normal', z=6, rot=0, style='normal'):
    ax.text(x, y, s, fontsize=size, color=color, ha=ha, va=va, weight=weight, zorder=z, rotation=rot,
            family='DejaVu Sans', style=style)


def pin(x, y, z=5):
    ax.add_patch(Rectangle((x - 0.55, y - 0.55), 1.1, 1.1, fc='#d4af37', ec='#6b5a12', lw=0.5, zorder=z))


# ---------------------------------------------------------------- ESP32
EX0, EX1, EY0, EY1 = 66, 94, 37, 79
LX, RX = 68.0, 92.0


def esp_y(i):
    return 74 - 2.4 * i


def esp_pin(name, side=None):
    if side != 'R' and name in ESP_LEFT:
        return LX, esp_y(ESP_LEFT.index(name))
    return RX, esp_y(ESP_RIGHT.index(name))


rbox(EX0, EY0, EX1, EY1, '#1b1b1d', r=1.2, lw=1.2)
for x in (LX, RX):
    ax.add_patch(Rectangle((x - 1.1, esp_y(14) - 1.3), 2.2, esp_y(0) - esp_y(14) + 2.6, fc='#0b0b0b', ec='none', zorder=2))
for i, (l, r) in enumerate(zip(ESP_LEFT, ESP_RIGHT)):
    y = esp_y(i)
    pin(LX, y)
    pin(RX, y)
    text(LX + 1.6, y, l, 5.2, '#f0f0f0', ha='left')
    text(RX - 1.6, y, r, 5.2, '#f0f0f0', ha='right')
rbox(75, 63, 85, 78, '#cfcfcf', '#777', r=0.4, z=2)                    # WROOM module
rbox(75.8, 63.8, 84.2, 72.5, '#b9b9b9', '#888', r=0.3, z=3)
xs = [76.2 + k * 0.9 for k in range(9)]
ax.plot(sum([[x, x] for x in xs], []), sum([[73.3, 77.2] if k % 2 == 0 else [77.2, 73.3] for k in range(9)], []),
        color='#c49a2c', lw=0.9, zorder=3)
text(80, 68.6, 'ESP32-\nWROOM-32', 5.2, '#333', weight='bold', z=4)
rbox(77.5, 37.2, 82.5, 40.2, '#c0c0c0', '#666', r=0.3, z=3)              # micro USB
for bx, lab in ((75.4, 'EN'), (84.6, 'BOOT')):
    rbox(bx - 1.3, 41.0, bx + 1.3, 43.4, '#e0e0e0', '#555', r=0.3, z=3)
    ax.add_patch(Circle((bx, 42.2), 0.6, fc='#333', zorder=4))
    text(bx, 44.5, lab, 4.5, '#ddd')
text(80, 81, 'ESP32 DevKit V1', 8, weight='bold')

# ---------------------------------------------------------------- L298N
LX0, LX1, LY0, LY1 = 28, 50, 44, 72
rbox(LX0, LY0, LX1, LY1, '#c62828', '#7a1010', r=0.8, lw=1.2)
ax.add_patch(Rectangle((35.5, 55), 9.5, 15, fc='#141414', ec='#000', zorder=2))  # heatsink
for k in range(7):
    ax.add_patch(Rectangle((36.0 + k * 1.3, 55.6), 0.6, 13.8, fc='#3a3a3a', ec='none', zorder=3))
ax.add_patch(Circle((46.2, 69.6), 1.5, fc='#1f3fa0', ec='#0d1d55', zorder=2))
text(46.2, 69.6, '470µ', 3.4, 'white')
text(40.5, 52.0, 'L298N', 7.5, 'white', weight='bold')
L_OUT = dict(OUT1=68.0, OUT2=65.5, OUT3=53.0, OUT4=50.5)
for (a, b) in (('OUT1', 'OUT2'), ('OUT3', 'OUT4')):
    rbox(LX0 - 0.6, L_OUT[b] - 1.4, LX0 + 2.6, L_OUT[a] + 1.4, '#2fa84f', '#14562a', r=0.3, z=2)
    for n in (a, b):
        ax.add_patch(Circle((LX0 + 1.0, L_OUT[n]), 0.7, fc='#b0b0b0', ec='#555', zorder=3))
        text(LX0 + 3.4, L_OUT[n], n, 4.6, 'white', ha='left')
L_PWR = {'+12V': 33.0, 'GND': 38.0, '+5V': 43.0}
rbox(31, LY0 - 0.6, 45, LY0 + 2.6, '#2fa84f', '#14562a', r=0.3, z=2)
for n, x in L_PWR.items():
    ax.add_patch(Circle((x, LY0 + 1.0), 0.7, fc='#b0b0b0', ec='#555', zorder=3))
    text(x, LY0 + 3.6, n, 4.6, 'white')
L_HDR = dict(ENA=66.0, IN1=63.6, IN2=61.2, IN3=58.8, IN4=56.4, ENB=54.0)
for n, y in L_HDR.items():
    pin(LX1 - 0.9, y)
    text(LX1 - 2.2, y, n, 4.6, 'white', ha='right')
text(39, 77.6, 'L298N dual H-bridge', 7.5, weight='bold')
text(39, 75.9, 'remove the ENA / ENB jumpers (PWM from the ESP32);\nkeep the 5 V regulator jumper ON',
     4.6, '#555', va='top', style='italic')

# ---------------------------------------------------------------- N20 motors
MOT = dict(FL=84.0, BL=70.0, FR=46.0, BR=32.0)


def n20(cy, name):
    ax.add_patch(Circle((6, cy), 4.2, fc='#151515', ec='#000', zorder=2))           # tyre
    for k in range(16):
        import math
        a = k * math.pi / 8
        ax.plot([6 + 3.6 * math.cos(a), 6 + 4.2 * math.cos(a)], [cy + 3.6 * math.sin(a), cy + 4.2 * math.sin(a)],
                color='#444', lw=0.8, zorder=3)
    ax.add_patch(Circle((6, cy), 2.4, fc='#eeeeee', ec='#999', zorder=3))            # white hub
    ax.add_patch(Circle((6, cy), 0.6, fc='#888', zorder=4))
    ax.add_patch(Rectangle((10.4, cy - 2.0), 3.6, 4.0, fc='#c9c9c9', ec='#666', lw=0.6, zorder=2))    # gearbox
    ax.add_patch(Rectangle((14.0, cy - 1.8), 5.4, 3.6, fc='#dcdcdc', ec='#666', lw=0.6, zorder=2))    # can
    ax.add_patch(Rectangle((19.4, cy - 1.5), 1.0, 3.0, fc='#222', ec='#000', lw=0.5, zorder=2))       # end cap
    ax.add_patch(Rectangle((10.9, cy - 2.6), 2.6, 5.2, fc='#f7f7f7', ec='#aaa', lw=0.5, zorder=3))    # clamp
    ax.plot([10.2, 10.4], [cy, cy], color='#888', lw=2, zorder=2)
    text(14.5, cy + 3.6, f'{name}  N20 60 rpm', 5.4, weight='bold')


for n, cy in MOT.items():
    n20(cy, n)
text(11, 92.5, 'LEFT side (FL + BL in parallel)', 6, C['blue'], weight='bold')
text(11, 25.4, 'RIGHT side (FR + BR in parallel,\npolarity swapped: mirrored mounting)', 5.6, C['blue'], weight='bold')

BUS_P, BUS_N = 25.0, 23.5
for n in ('FL', 'BL'):
    cy = MOT[n]
    if n == 'FL':
        wire([(20.4, cy + 0.9), (BUS_P, cy + 0.9), (BUS_P, L_OUT['OUT1']), (LX0, L_OUT['OUT1'])], C['red'], 1.8)
        wire([(20.4, cy - 0.9), (BUS_N, cy - 0.9), (BUS_N, L_OUT['OUT2']), (LX0, L_OUT['OUT2'])], C['black'], 1.8)
    else:
        wire([(20.4, cy + 0.9), (BUS_P, cy + 0.9)], C['red'], 1.8)
        wire([(20.4, cy - 0.9), (BUS_N, cy - 0.9)], C['black'], 1.8)
        DOTS += [(BUS_P, cy + 0.9), (BUS_N, cy - 0.9)]
for n in ('FR', 'BR'):
    cy = MOT[n]
    if n == 'FR':
        wire([(20.4, cy + 0.9), (BUS_P, cy + 0.9), (BUS_P, L_OUT['OUT4']), (LX0, L_OUT['OUT4'])], C['red'], 1.8)
        wire([(20.4, cy - 0.9), (BUS_N, cy - 0.9), (BUS_N, L_OUT['OUT3']), (LX0, L_OUT['OUT3'])], C['black'], 1.8)
    else:
        wire([(20.4, cy + 0.9), (BUS_P, cy + 0.9), (BUS_P, MOT['FR'] + 0.9)], C['red'], 1.8)
        wire([(20.4, cy - 0.9), (BUS_N, cy - 0.9), (BUS_N, MOT['FR'] - 0.9)], C['black'], 1.8)
        DOTS += [(BUS_P, MOT['FR'] + 0.9), (BUS_N, MOT['FR'] - 0.9)]

# ---------------------------------------------------------------- L298N control -> ESP32
ctrl_col = dict(ENA=C['purple'], IN1=C['blue'], IN2=C['cyan'], IN3=C['green'], IN4=C['yellow'], ENB=C['pink'])
for k, n in enumerate(('ENA', 'IN1', 'IN2', 'IN3', 'IN4', 'ENB')):
    yl = L_HDR[n]
    _, ye = esp_pin(PINS[n], 'L')
    xk = 64 - 1.6 * k
    wire([(LX1 - 0.9, yl), (xk, yl), (xk, ye), (LX, ye)], ctrl_col[n], 1.6)

# L298N 5 V regulator -> ESP32 VIN, common GND
wire([(L_PWR['+5V'], LY0 + 1.0), (L_PWR['+5V'], esp_pin('VIN')[1]), (LX, esp_pin('VIN')[1])], C['red'], 1.8)
wire([(L_PWR['GND'], LY0 + 1.0), (L_PWR['GND'], esp_pin('GND', 'L')[1]), (LX, esp_pin('GND', 'L')[1])], C['black'], 1.8)

# ---------------------------------------------------------------- battery + XT60
rbox(58, 5, 98, 18, '#2b2b2b', '#000', r=1.2, lw=1.2)
rbox(60, 6.5, 96, 16.5, '#f2c200', '#9c7c00', r=0.8, z=2)
text(78, 13.2, '3S LiPo  11.1 V (12.6 V full)', 8, '#222', weight='bold')
text(78, 9.4, 'off the robot, on the XT60 lead', 5.8, '#333', style='italic')
for y, col in ((13.0, C['red']), (11.0, C['black'])):
    ax.plot([58, 55.5], [y, y], color=col, lw=3.2, zorder=2, solid_capstyle='butt')
ax.add_patch(Polygon([(50.5, 9.0), (55.5, 9.0), (55.5, 15.0), (51.5, 15.0), (50.5, 14.0)], fc='#f0c000', ec='#8a6d00',
                     lw=0.8, zorder=3))
text(53, 16.3, 'XT60', 5.6, weight='bold')
BAT_P = [(50.5, 13.0), (46.0, 13.0), (46.0, 21.0)]
BAT_N = [(50.5, 11.0), (48.0, 11.0), (48.0, 24.0)]
wire(BAT_P + [(L_PWR['+12V'], 21.0), (L_PWR['+12V'], LY0 + 1.0)], C['red'], 3.2)
wire(BAT_N + [(L_PWR['GND'], 24.0), (L_PWR['GND'], LY0 + 1.0)], C['black'], 3.2)
wire([(46.0, 21.0), (107.0, 21.0), (107.0, 57.0), (112.0, 57.0)], C['red'], 3.2)
wire([(48.0, 24.0), (105.0, 24.0), (105.0, 59.0), (112.0, 59.0)], C['black'], 3.2)
DOTS += [(46.0, 21.0), (48.0, 24.0)]
text(76, 22.6, 'battery + (14 AWG)', 5.2, C['red'])
text(76, 25.6, 'battery − / common GND', 5.2, C['black'])

# ---------------------------------------------------------------- ESC + BLDC
rbox(112, 54, 130, 64, '#151515', '#000', r=1.4, lw=1.2)
ax.add_patch(Rectangle((113.5, 55.4), 15, 7.2, fc='#2d2d2d', ec='none', zorder=2))
ax.add_patch(Rectangle((113.5, 61.0), 15, 1.6, fc='#1e4fa8', ec='none', zorder=3))
text(121, 58.2, '40A ESC', 8, 'white', weight='bold')
text(121, 66, '40 A brushless ESC (BEC 5 V)', 6.5, weight='bold')
wire([(112, 61.5), (101.0, 61.5), (101.0, esp_pin('D18', 'R')[1]), (RX, esp_pin('D18', 'R')[1])], C['orange'], 1.6)
wire([(112, 60.5), (103.0, 60.5), (103.0, esp_pin('GND', 'R')[1]), (RX, esp_pin('GND', 'R')[1])], C['brown'], 1.6)
ax.plot([112, 109.5], [62.6, 62.6], color=C['red'], lw=1.6, zorder=3,
        path_effects=[pe.Stroke(linewidth=2.6, foreground='#222'), pe.Normal()])
text(109.1, 62.7, '×', 7, C['red'], weight='bold')
text(108.4, 64.0, 'BEC+ n/c', 4.4, '#555', ha='right', style='italic')
text(95.0, 59.0, 'servo PWM\n50 Hz, 1–2 ms', 4.4, C['orange'], ha='left', va='top')
MX, MY = 142.0, 59.0
for k, (y, col) in enumerate(((57.0, C['black']), (59.0, C['red']), (61.0, C['yellow']))):
    wire([(130, y), (132.0, y)], col, 1.8)
    ax.add_patch(Rectangle((132.0, y - 0.55), 2.2, 1.1, fc='#d4af37', ec='#6b5a12', lw=0.5, zorder=4))
    wire([(134.2, y), (MX - 4.6, MY + (y - 59.0) * 0.6)], col, 1.8)
ax.add_patch(Circle((MX, MY), 4.8, fc='#c8862a', ec='#6b4210', lw=1.0, zorder=2))     # bell (top view)
ax.add_patch(Circle((MX, MY), 3.2, fc='#e0a64a', ec='#8a5a18', lw=0.6, zorder=3))
for k in range(4):
    import math
    a = math.pi / 4 + k * math.pi / 2
    ax.add_patch(Circle((MX + 2.2 * math.cos(a), MY + 2.2 * math.sin(a)), 0.45, fc='#5a3a10', zorder=4))
blade = [(0, -0.5), (3, -1.3), (9, -1.1), (13, -0.4), (13.3, 0), (13, 0.5), (9, 1.2), (3, 1.4), (0, 0.5)]
for s in (1, -1):
    import math
    a = math.radians(78)
    pts = [(MX + 0.8 * s * (u * math.cos(a) - v * math.sin(a)), MY + 0.8 * s * (u * math.sin(a) + v * math.cos(a)))
           for u, v in blade]
    ax.add_patch(Polygon(pts, fc='#111', ec='#000', alpha=0.88, zorder=5))
ax.add_patch(Circle((MX, MY), 1.4, fc='#d8d8d8', ec='#666', zorder=6))
text(MX + 6.0, MY + 8.5, 'A2212 1000 KV\n+ 1045 prop', 6.5, weight='bold', ha='left')
text(MX + 6.0, MY - 6.0, 'swap any two phase\nwires to reverse it:\nthe prop must blow\nAWAY from the wall', 4.8,
     '#555', ha='left', style='italic')

# ---------------------------------------------------------------- planned sensor kit (dashed, net labels)
rbox(100, 70.5, 158, 101, 'none', '#1e6fd9', r=1.5, lw=1.2, ls='--', z=1)
text(129, 99.3, 'Sensor kit of the autonomous twin: PLANNED, not on the robot yet', 6.2, C['blue'], weight='bold')


def tag(x, y, s, col, ha='left'):
    ax.text(x, y, s, fontsize=4.9, color='white', ha=ha, va='center', zorder=7, weight='bold',
            bbox=dict(boxstyle='round,pad=0.25', fc=col, ec='#222', lw=0.5))


for name, col, lab in ((PINS['SCL'], C['green'], 'SCL→IMU'), (PINS['SDA'], C['cyan'], 'SDA→IMU'),
                       (PINS['LIDAR_RX'], C['purple'], '←LD06 TX')):
    y = esp_pin(name, 'R')[1]
    wire([(RX, y), (94.6, y)], col, 1.4, dashed=True)
    tag(94.9, y, lab, col)
rbox(104, 77, 118, 92, '#1a4fb5', '#0c2a66', r=0.6, z=2)              # MPU6050
ax.add_patch(Rectangle((109.5, 83.5), 3, 3, fc='#111', zorder=3))
text(111, 89.5, 'MPU6050\nIMU', 5.6, 'white', weight='bold')
for k, (n, col, to) in enumerate((('VCC', C['red'], '3V3'), ('GND', C['black'], 'GND'), ('SCL', C['green'], 'D22'),
                                  ('SDA', C['cyan'], 'D21'))):
    y = 81.5 - k * 1.3 + 0.0
    x = 105.5 + k * 3.5
    pin(x, 78.2)
    text(x, 79.8, n, 3.9, 'white')
    wire([(x, 78.2), (x, 74.4 - k * 0.0)], col, 1.3, dashed=True)
    tag(x, 73.2, to, col, ha='center')
ax.add_patch(Circle((136, 86), 7.0, fc='#2c2c2c', ec='#000', zorder=2))  # LD06
ax.add_patch(Circle((136, 86), 5.0, fc='#3f3f3f', ec='#111', zorder=3))
ax.add_patch(Circle((136, 86), 2.2, fc='#111', zorder=4))
text(136, 95.4, 'LD06 2D lidar', 5.8, weight='bold')
for k, (n, col, to) in enumerate((('5V', C['red'], 'VIN 5V'), ('GND', C['black'], 'GND'), ('TX', C['purple'], 'RX2'))):
    x = 131.5 + k * 4.5
    pin(x, 78.0)
    text(x + 0.9, 76.4, n, 3.9, ha='left')
    wire([(x, 78.0), (x, 74.4)], col, 1.3, dashed=True)
    tag(x, 73.2, to, col, ha='center')
text(150.5, 88.5, 'wall + front\ncameras: USB /\nCSI to a\ncompanion\ncomputer (Pi),\nnot the ESP32', 4.6,
     '#555', style='italic')


# ---------------------------------------------------------------- draw the wires (horizontal runs hop over verticals)
def segments(pts):
    return list(zip(pts[:-1], pts[1:]))


VERT = []
for i, (pts, col, lw, dashed) in enumerate(WIRES):
    for (a, b) in segments(pts):
        if abs(a[0] - b[0]) < 1e-6 and abs(a[1] - b[1]) > 1e-6:
            VERT.append((i, a[0], min(a[1], b[1]), max(a[1], b[1])))


def stroke(xs, ys, col, lw, dashed, z=3):
    ax.plot(xs, ys, color=col, lw=lw, zorder=z, ls='--' if dashed else '-', solid_capstyle='round',
            solid_joinstyle='round', path_effects=[pe.Stroke(linewidth=lw + 1.1, foreground='#1a1a1a'), pe.Normal()])


HOP = 0.9
for i, (pts, col, lw, dashed) in enumerate(WIRES):
    z = 3.5 if lw > 3 else 3
    for (a, b) in segments(pts):
        if abs(a[1] - b[1]) < 1e-6 and abs(a[0] - b[0]) > 1e-6:          # horizontal: look for crossings
            y = a[1]
            lo, hi = sorted((a[0], b[0]))
            xs = sorted(x for j, x, y0, y1 in VERT if j != i and lo + HOP < x < hi - HOP and y0 + 0.2 < y < y1 - 0.2)
            d = 1 if b[0] > a[0] else -1
            xs = xs if d > 0 else xs[::-1]
            cur = a[0]
            for x in xs:
                stroke([cur, x - d * HOP], [y, y], col, lw, dashed, z)
                import numpy as np
                t = np.linspace(np.pi, 0, 20) if d > 0 else np.linspace(0, np.pi, 20)
                stroke(x + HOP * np.cos(t), y + HOP * np.sin(t), col, lw, dashed, z)
                cur = x + d * HOP
            stroke([cur, b[0]], [y, y], col, lw, dashed, z)
        else:
            stroke([a[0], b[0]], [a[1], b[1]], col, lw, dashed, z)
for x, y in DOTS:
    ax.add_patch(Circle((x, y), 0.65, fc='#111', zorder=6))

# ---------------------------------------------------------------- pin table
TX0, TY = 112.0, 45.5
rows = [('ESP32 pin', 'goes to', 'function'),
        (PINS['ENA'], 'L298N ENA', 'left wheels speed (PWM)'),
        (PINS['IN1'] + ' / ' + PINS['IN2'], 'L298N IN1 / IN2', 'left wheels direction'),
        (PINS['IN3'] + ' / ' + PINS['IN4'], 'L298N IN3 / IN4', 'right wheels direction'),
        (PINS['ENB'], 'L298N ENB', 'right wheels speed (PWM)'),
        (PINS['ESC'], 'ESC signal', 'thrust: 50 Hz, 1000–2000 µs'),
        ('VIN / GND', 'L298N +5V / GND', 'ESP32 power, common ground'),
        ('GND (right)', 'ESC signal GND', 'common ground'),
        ('D21 / D22', 'MPU6050 SDA / SCL', 'planned: tilt for auto thrust'),
        ('RX2 (GPIO16)', 'LD06 TX', 'planned: lidar scans')]
cw = (11.5, 14.5, 20.5)
for r, row in enumerate(rows):
    y = TY - r * 2.35
    fc = '#263238' if r == 0 else ('#f4f6f8' if r % 2 else 'white')
    ax.add_patch(Rectangle((TX0, y - 1.175), sum(cw), 2.35, fc=fc, ec='#b0bec5', lw=0.5, zorder=1))
    x = TX0
    for k, s in enumerate(row):
        text(x + 0.6, y, s, 5.0, 'white' if r == 0 else '#111', ha='left', weight='bold' if r == 0 or k == 0 else 'normal')
        x += cw[k]
text(TX0, TY - len(rows) * 2.35 - 0.3, 'Suggested pin mapping: edit PINS in tools/wiring_diagram.py to match the ESP32 code.',
     4.6, '#555', ha='left', va='top', style='italic')

text(2, 101.5, 'WCR prototype v2: wiring diagram', 12, weight='bold', ha='left')
text(2, 98.3, 'thrust: 3S LiPo → 40 A ESC → A2212 1000 KV + 1045 prop   |   drive: ESP32 → L298N → 4 × N20 (skid steer)',
     6.8, '#444', ha='left')

OUT.mkdir(parents=True, exist_ok=True)
fig.savefig(OUT / 'wiring_diagram.png', dpi=200, bbox_inches='tight', facecolor='white')
fig.savefig(OUT / 'wiring_diagram.svg', bbox_inches='tight', facecolor='white')
print(OUT / 'wiring_diagram.png')
