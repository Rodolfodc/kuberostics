#!/usr/bin/env python3
"""Puts /myRobot at x y and model yaw (deg), robot stationary. Usage: bridge_live_pose.py X Y YAW_DEG   (local-only)"""
import math, sys
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
sim = RemoteAPIClient().require('sim'); r = sim.getObject('/myRobot')
x, y, yaw = float(sys.argv[1]), float(sys.argv[2]), math.radians(float(sys.argv[3]))
sim.setObjectPose(r, [x, y, 0.08, 0, 0, math.sin(yaw / 2), math.cos(yaw / 2)], -1)
sim.resetDynamicObject(sim.handle_all) if hasattr(sim, 'resetDynamicObject') else None
print('pose now', sim.getObjectPose(r, -1))
