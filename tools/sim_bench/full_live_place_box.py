#!/usr/bin/env python3
"""Places a static box D metres ahead of /myRobot (its +Y axis is the forward direction) for the stuck scenario of the full-robot live run (local-only). Usage: full_live_place_box.py D_m"""
import math, sys
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
sim = RemoteAPIClient().require('sim'); r = sim.getObject('/myRobot'); m = sim.getObjectMatrix(r, -1)
fx, fy = m[1], m[5]; n = math.hypot(fx, fy); fx, fy = fx / n, fy / n; d = float(sys.argv[1]); x, y = sim.getObjectPosition(r, -1)[:2]
b = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [0.1, 1.2, 0.4], 0)
sim.setObjectPosition(b, -1, [x + fx * d, y + fy * d, 0.2]); sim.setObjectOrientation(b, -1, [0, 0, math.atan2(fy, fx)])
sim.setObjectInt32Param(b, sim.shapeintparam_static, 1); sim.setObjectInt32Param(b, sim.shapeintparam_respondable, 1)
sim.setObjectAlias(b, 'stuck_test_box'); print('box', b, 'at', x + fx * d, y + fy * d, 'robot', x, y)
