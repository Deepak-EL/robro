"""Fit the drive surface of a dam face to its VISUAL mesh, for the twin's collision face.

The dam worlds use a best-fit plane (a Box) for the face collision. The prototype rides only
7.5 mm off the wall, so where the visual mesh is in front of that plane (GoldenEye upstream: the
main face mesh is 7-23 mm behind it and tilted, a lower band 30 mm in front) the robot is drawn
inside the wall. This fits h(s, t) = h0 + a s + b t (height above the layout face plane, metres)
to the given visual mesh over the drive area; launch/proto.launch.py uses the numbers (FACE_FIT).

  python tools/fit_face.py goldeneye_dam obj_Object_7.obj
needs: numpy trimesh pyyaml
"""
import sys
from pathlib import Path

import numpy as np
import trimesh
import yaml

D = Path(__file__).resolve().parents[2] / 'wcr_dam_sim'


def main(world, mesh):
    doc = yaml.safe_load((D / f'wcr_description/worlds/inspection/{world}.yaml').read_text())
    f = doc['face']
    base, up, ac, n = (np.array(f[k], float) for k in ('base', 'up', 'across', 'normal'))
    m = trimesh.load(D / 'wcr_description/worlds/meshes/goldeneye_dam' / mesh, force='mesh')
    S, T = np.meshgrid(np.arange(0.0, 5.01, 0.25), np.arange(-3.0, 3.01, 0.25), indexing='ij')
    o = base + S.ravel()[:, None] * up + T.ravel()[:, None] * ac + n * 0.5
    loc, ray, _ = m.ray.intersects_location(o, np.tile(-n, (len(o), 1)), multiple_hits=False)
    s, t, h = S.ravel()[ray], T.ravel()[ray], (loc - base) @ n
    A = np.c_[np.ones_like(s), s, t]
    coef, *_ = np.linalg.lstsq(A, h, rcond=None)
    res = h - A @ coef
    print(f'{world} {mesh}: {len(h)} points, h = {coef[0] * 1000:.1f} mm + {coef[1] * 1000:.2f} mm/m * s + '
          f'{coef[2] * 1000:.2f} mm/m * t, residual rms {res.std() * 1000:.1f} mm, max {np.abs(res).max() * 1000:.1f} mm')
    print(f"FACE_FIT['{world}'] = ({coef[0]:.5f}, {coef[1]:.6f}, {coef[2]:.6f})")


if __name__ == '__main__':
    main(*sys.argv[1:3])
