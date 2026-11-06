"""How far in front of the collision face (layout face plane) is the VISUAL dam surface, over the
drive area? Rays along -normal from 0.5 m out; first hit on any visual mesh of the dam Solid."""
import re
import sys
from pathlib import Path

import numpy as np
import trimesh
import yaml

D = Path(r'C:\Users\DEEPAK E L\Downloads\wcr_dam_sim')
world = sys.argv[1] if len(sys.argv) > 1 else 'goldeneye_dam'
doc = yaml.safe_load((D / f'wcr_description/worlds/inspection/{world}.yaml').read_text())
f = doc['face']
base, up, ac, n = (np.array(f[k]) for k in ('base', 'up', 'across', 'normal'))
txt = (D / f'webots/worlds/{world}.wbt').read_text()
solid = txt[txt.find('Solid {'):txt.find('boundingObject')]           # the dam Solid's visual part
urls = re.findall(r'geometry Mesh \{ url \[ "\.\./\.\./(wcr_description/[^"]+\.obj)" \]', solid)
print(world, len(urls), 'visual meshes; Solid header:', solid[:200].replace('\n', ' '))
ms = [trimesh.load(D / u, force='mesh') for u in urls]
m = trimesh.util.concatenate(ms)
S = np.arange(0.6, 4.01, 0.4)
Tt = np.arange(-1.5, 1.51, 0.5)
orig = np.array([base + s * up + t * ac + n * 0.5 for s in S for t in Tt])
loc, ray, _ = m.ray.intersects_location(orig, np.tile(-n, (len(orig), 1)), multiple_hits=False)
h = np.full(len(orig), np.nan)
for p, r in zip(loc, ray):
    h[r] = (p - base) @ n
h = h.reshape(len(S), len(Tt)) * 1000
print('visual surface height above the collision face [mm]; cols t =', list(np.round(Tt, 1)))
for s, row in zip(S, h):
    print(f's {s:3.1f}: ' + ' '.join(f'{v:6.1f}' for v in row))
print('max', np.nanmax(h).round(1), 'mean', np.nanmean(h).round(1))

print('per mesh: first-hit height at t=0 for each s [mm]')
for u, mi in zip(urls, ms):
    o = np.array([base + s * up + n * 0.5 for s in S])
    lo, rr, _ = mi.ray.intersects_location(o, np.tile(-n, (len(o), 1)), multiple_hits=False)
    if len(lo):
        hh = {int(r): round(float((p - base) @ n) * 1000, 1) for p, r in zip(lo, rr)}
        print(f'  {Path(u).name:22s}', [hh.get(i, None) for i in range(len(S))])
