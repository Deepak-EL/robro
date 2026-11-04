"""Render ros/wcr_proto/resource/wcr_proto.urdf (all joints at zero) the way RViz / Foxglove
assemble it: each link's STL visuals with their origins, placed by the joint chain from base_link.
Also draws the sensor frames (x red, y green, z blue).

  python tools/render_urdf.py -> cad/urdf_check.png (iso view + side view)
needs: numpy trimesh pillow
"""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'ros' / 'wcr_proto'


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def tf(origin):
    T = np.eye(4)
    if origin is not None:
        T[:3, :3] = rpy_matrix(*[float(v) for v in origin.get('rpy', '0 0 0').split()])
        T[:3, 3] = [float(v) for v in origin.get('xyz', '0 0 0').split()]
    return T


def load():
    root = ET.parse(PKG / 'resource' / 'wcr_proto.urdf').getroot()
    colors = {m.get('name'): [float(v) for v in m.find('color').get('rgba').split()[:3]]
              for m in root.findall('material') if m.find('color') is not None}
    parent = {j.find('child').get('link'): (j.find('parent').get('link'), tf(j.find('origin')))
              for j in root.findall('joint')}

    def world(link):
        T = np.eye(4)
        while link in parent:
            link, Tj = parent[link]
            T = Tj @ T
        return T
    meshes, frames = [], {}
    for link in root.findall('link'):
        name = link.get('name')
        Tw = world(name)
        frames[name] = Tw
        for v in link.findall('visual'):
            g = v.find('geometry/mesh')
            if g is None:
                continue
            m = trimesh.load(PKG / g.get('filename').replace('package://wcr_proto/', ''), force='mesh')
            m.apply_scale([float(s) for s in g.get('scale', '1 1 1').split()][0])
            m.apply_transform(Tw @ tf(v.find('origin')))
            meshes.append((m, np.array(colors[v.find('material').get('name')]) * 255))
    return meshes, frames


def render(meshes, frames, az, el, s=2.5):
    az, el = math.radians(az), math.radians(el)
    Rz = np.array([[math.cos(az), -math.sin(az), 0], [math.sin(az), math.cos(az), 0], [0, 0, 1]])
    E = np.array([[1, 0, 0], [0, math.cos(el), -math.sin(el)], [0, math.sin(el), math.cos(el)]])
    V = E @ np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]) @ Rz
    light = np.array([0.3, 0.5, 0.8]) / np.linalg.norm([0.3, 0.5, 0.8])
    P, C = [], []
    for m, col in meshes:
        pts, fi = trimesh.sample.sample_surface(m, int(max(1500, m.area * 2.5e7)))
        sh = 0.35 + 0.65 * np.abs(m.face_normals[fi] @ V.T @ light)
        P.append(pts @ V.T)
        C.append(np.clip(np.maximum(col, 25) * sh[:, None], 0, 255))
    for name in ('lidar_link', 'wall_camera_link', 'front_camera_link', 'imu_link'):    # sensor axes
        T = frames[name]
        for k, col in enumerate(((230, 30, 30), (30, 190, 30), (30, 60, 230))):
            seg = T[:3, 3] + np.outer(np.linspace(0, 0.04, 60), T[:3, k])
            P.append(seg @ V.T + np.array([0, 0, 1.0]))                         # drawn on top
            C.append(np.tile(col, (60, 1)))
    P, C = np.vstack(P), np.vstack(C)
    px = s * 1000
    u = ((P[:, 0] - P[:, 0].min()) * px).astype(int) + 15
    v = ((P[:, 1].max() - P[:, 1]) * px).astype(int) + 15
    img = np.full((v.max() + 16, u.max() + 16, 3), 255.0)
    o = np.argsort(P[:, 2])
    img[v[o], u[o]] = C[o]
    return Image.fromarray(img.astype(np.uint8))


def main():
    meshes, frames = load()
    a, b = render(meshes, frames, -50, 28), render(meshes, frames, 0, 0)
    out = Image.new('RGB', (a.width + b.width, max(a.height, b.height)), 'white')
    out.paste(a, (0, 0))
    out.paste(b, (a.width, 0))
    out.save(ROOT / 'cad' / 'urdf_check.png')
    print('links', len(frames), 'visual meshes', len(meshes))
    for n in ('lidar_link', 'wall_camera_optical', 'front_camera_optical', 'prop_link', 'FL_wheel_link', 'BR_wheel_link'):
        print(f'  {n:22s} at {np.round(frames[n][:3, 3], 4)}  z axis {np.round(frames[n][:3, 2], 3)}')


if __name__ == '__main__':
    main()
