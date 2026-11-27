#!/usr/bin/env python3
"""One top-down (default) or oblique picture of the robot in the scene from a temporary vision sensor with explicit handling
(removed at the end). Local-only; the display stays disabled. Usage: shots_topdown.py OUT.png [height_m=3.0] [tilt_deg=0] [back_m=0]"""
import math, sys
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
out = sys.argv[1]; h = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0; tilt = math.radians(float(sys.argv[3])) if len(sys.argv) > 3 else 0.0
back = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
sim = RemoteAPIClient().require('sim'); robot = sim.getObject('/myRobot'); rx, ry = sim.getObjectPosition(robot, -1)[:2]
vs = sim.createVisionSensor(3, [800, 600, 0, 0], [0.05, 12.0, math.radians(60), 0.1, 0, 0, 0, 0, 0, 0, 0])
try:
    sim.setObjectPosition(vs, -1, [rx, ry - back, h]); sim.setObjectOrientation(vs, -1, [math.pi - tilt, 0, 0])
    sim.handleVisionSensor(vs)
    img, res = sim.getVisionSensorImg(vs); w, hh = res
    a = np.frombuffer(img, np.uint8).reshape(hh, w, 3)[::-1]
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    plt.imsave(out, a); print('saved', out, 'mean', float(a.mean()))
finally:
    sim.removeObjects([vs])
