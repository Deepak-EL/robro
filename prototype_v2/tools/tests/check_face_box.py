"""Check the launch file's face_box(): the box rotation must map x, y, z onto up, across, normal."""
import importlib.util
from pathlib import Path

import numpy as np
import yaml

L = '/mnt/c/Users/DEEPAK E L/Downloads/wcr_proto2/ros/wcr_proto2/launch/proto.launch.py'
spec = importlib.util.spec_from_file_location('proto_launch', L)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
for w in ('goldeneye_gorge', 'goldeneye_dam', 'winter_dam'):
    lay = Path(f'/mnt/c/Users/DEEPAK E L/Downloads/wcr_dam_sim/wcr_description/worlds/inspection/{w}.yaml')
    txt = mod.face_box(lay)
    rot = [float(v) for v in txt.split('rotation ')[1].split('\n')[0].split()]
    a, ang = np.array(rot[:3]), rot[3]
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    Rm = np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K
    f = yaml.safe_load(lay.read_text())['face']
    err = max(np.abs(Rm[:, 0] - f['up']).max(), np.abs(Rm[:, 1] - f['across']).max(), np.abs(Rm[:, 2] - f['normal']).max())
    print(w, 'rotation error', round(float(err), 6))
print(mod.face_box(Path('/mnt/c/Users/DEEPAK E L/Downloads/wcr_dam_sim/wcr_description/worlds/inspection/goldeneye_gorge.yaml')))
